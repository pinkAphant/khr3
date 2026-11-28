import React, { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { useRapier } from "@react-three/rapier";
import type RAPIER from "@dimforge/rapier3d-compat";
import { useCustomGravity } from "@pinkaphant/ecctrl/gravity";
import {
    EcctrlVehicle,
    ShapeCastWheel,
    ThrustPropeller,
    type EcctrlVehicleHandle,
    type EcctrlVehicleProps,
    type PropellerInfoType,
    type ShapeCastWheelProps,
    type ThrustPropellerProps,
    type VehicleInput,
    type WheelInfoType,
} from "@pinkaphant/ecctrl/vehicle";
import {
    BundleSceneHost,
    captureSubtreeOrigins,
    registerMountedSubtree,
    restoreSubtreeOrigin,
    type SubtreeOrigin,
} from "./clone.js";
import {
    applyRapierBodyMass,
    assertSceneIdentityWorldScale,
    assertUnitScale,
    bodyPoseMatrix,
    computeLumpedMassProps,
    type LumpedMassProps,
    type QuatTuple,
    type Vec3Tuple,
} from "./inertia.js";
import { KhrCollider } from "./KhrCollider.js";
import {
    assignRef,
    type EcctrlPropellerTag,
    type KhrAssetBundle,
    type KhrBodySpec,
    type KhrCarConfig,
    type KhrColliderSpec,
    type KhrDroneConfig,
    KhrValidationError,
} from "./types.js";
import { KhrVehicleJoints, mergeVehiclePanelUserData, type KhrVehicleJointsHandle } from "./vehicleJoints.js";
import { STANDARD_GRAVITY, solveWheelLoads, type SolvedWheel } from "./wheelLoads.js";

export { mergeVehiclePanelUserData, solveWheelLoads, type SolvedWheel };
export type { KhrVehicleJointsHandle };

export type KhrWheelConfig = Partial<Omit<ShapeCastWheelProps, "ref" | "key" | "children">>;
export type KhrPropellerConfig = Partial<Omit<ThrustPropellerProps, "ref" | "key" | "children">>;

export interface KhrVehicleProps extends Omit<EcctrlVehicleProps, "ref" | "carConfig" | "droneConfig"> {
    ref?: React.Ref<EcctrlVehicleHandle>;
    bundle: KhrAssetBundle;
    vehicleRef?: React.Ref<EcctrlVehicleHandle | null>;
    jointsRef?: React.Ref<KhrVehicleJointsHandle>;
    carConfig?: Partial<KhrCarConfig>;
    droneConfig?: Partial<KhrDroneConfig>;
    wheelConfig?: KhrWheelConfig;
    frontWheelConfig?: KhrWheelConfig;
    rearWheelConfig?: KhrWheelConfig;
    propellerConfig?: KhrPropellerConfig;
}

const UNSPRUNG_CORNER_RATIO = 0.035;
const _samplePos = new THREE.Vector3();

function readPosTuple(pos: EcctrlVehicleProps["position"]): Vec3Tuple {
    if (typeof pos === "number")
        return [pos, pos, pos];
    if (Array.isArray(pos))
        return [Number(pos[0] ?? 0), Number(pos[1] ?? 0), Number(pos[2] ?? 0)];
    if (pos && typeof pos === "object" && "x" in pos) {
        const v = pos as { x: number; y: number; z: number };
        return [Number(v.x ?? 0), Number(v.y ?? 0), Number(v.z ?? 0)];
    }
    return [0, 0, 0];
}

function readSpawnQuat(
    rotation: EcctrlVehicleProps["rotation"],
    quaternion: EcctrlVehicleProps["quaternion"],
): QuatTuple {
    if (quaternion) {
        if (Array.isArray(quaternion)) {
            const q = new THREE.Quaternion(
                Number(quaternion[0] ?? 0),
                Number(quaternion[1] ?? 0),
                Number(quaternion[2] ?? 0),
                Number(quaternion[3] ?? 1),
            ).normalize();
            return [q.x, q.y, q.z, q.w];
        }
        if (typeof quaternion === "object" && "x" in quaternion) {
            const qObj = quaternion as { x: number; y: number; z: number; w: number };
            const q = new THREE.Quaternion(qObj.x, qObj.y, qObj.z, qObj.w).normalize();
            return [q.x, q.y, q.z, q.w];
        }
    }
    if (rotation) {
        const euler = new THREE.Euler();
        if (rotation instanceof THREE.Euler) {
            euler.copy(rotation);
        }
        else if (Array.isArray(rotation)) {
            euler.set(
                Number(rotation[0] ?? 0),
                Number(rotation[1] ?? 0),
                Number(rotation[2] ?? 0),
                typeof rotation[3] === "string" ? (rotation[3] as THREE.EulerOrder) : "XYZ",
            );
        }
        else if (typeof rotation === "object" && "x" in rotation) {
            const rObj = rotation as { x: number; y: number; z: number; order?: THREE.EulerOrder };
            euler.set(Number(rObj.x ?? 0), Number(rObj.y ?? 0), Number(rObj.z ?? 0), rObj.order ?? "XYZ");
        }
        const q = new THREE.Quaternion().setFromEuler(euler).normalize();
        return [q.x, q.y, q.z, q.w];
    }
    return [0, 0, 0, 1];
}

interface ChassisMassSummary {
    lumpedMass: LumpedMassProps;
    linearVelocity?: Vec3Tuple;
    angularVelocity?: Vec3Tuple;
    gravityFactor?: number;
}

function computeChassisMassProps(bodies: readonly KhrBodySpec[]): ChassisMassSummary {
    const dyn = bodies.filter((b) => !b.isKinematic && b.joint === null);
    if (dyn.length === 0)
        throw new KhrValidationError("KhrVehicle: bundle has no dynamic rigid body for the chassis");

    const lumpedMass = computeLumpedMassProps(dyn);
    if (!lumpedMass)
        throw new KhrValidationError("KhrVehicle: chassis dynamic mass must be positive");

    let gravityFactor: number | undefined;
    if (dyn.some((b) => b.gravityFactor !== undefined)) {
        const g0 = dyn[0].gravityFactor ?? 1;
        for (const b of dyn) {
            if (Math.abs((b.gravityFactor ?? 1) - g0) > 1e-6) {
                throw new KhrValidationError(
                    `KhrVehicle: all dynamic chassis bodies must share gravityFactor (got ${g0} vs ${b.gravityFactor ?? 1} on "${b.nodeName}")`,
                );
            }
        }
        gravityFactor = g0;
    }

    let linearVelocity: Vec3Tuple | undefined;
    if (dyn.some((b) => b.linearVelocity !== undefined)) {
        let vx = 0, vy = 0, vz = 0;
        for (const b of dyn) {
            const lv = b.linearVelocity ?? [0, 0, 0];
            vx += lv[0] * b.mass;
            vy += lv[1] * b.mass;
            vz += lv[2] * b.mass;
        }
        linearVelocity = [vx / lumpedMass.mass, vy / lumpedMass.mass, vz / lumpedMass.mass];
    }

    const rootDyn = dyn.find((b) => b.angularVelocity !== undefined);
    const angularVelocity: Vec3Tuple | undefined = rootDyn?.angularVelocity
        ? [rootDyn.angularVelocity[0], rootDyn.angularVelocity[1], rootDyn.angularVelocity[2]]
        : undefined;

    return {
        lumpedMass,
        ...(linearVelocity ? { linearVelocity } : {}),
        ...(angularVelocity ? { angularVelocity } : {}),
        ...(gravityFactor !== undefined ? { gravityFactor } : {}),
    };
}

function buildChassisColliders(bodies: readonly KhrBodySpec[]): Array<{ key: string; spec: KhrColliderSpec }> {
    const out: Array<{ key: string; spec: KhrColliderSpec }> = [];
    const one = new THREE.Vector3(1, 1, 1);
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scl = new THREE.Vector3();

    for (const b of bodies) {
        if (b.joint !== null || (b.isKinematic && b.hasExplicitMotion))
            continue;
        const bodyMat = bodyPoseMatrix(b);
        for (let idx = 0; idx < b.colliders.length; idx++) {
            const c = b.colliders[idx];
            const colMat = new THREE.Matrix4().compose(
                new THREE.Vector3().fromArray(c.localPosition),
                new THREE.Quaternion().fromArray(c.localQuaternion),
                one,
            );
            bodyMat.clone().multiply(colMat).decompose(pos, quat, scl);
            quat.normalize();
            out.push({
                key: `${b.nodeName}:${idx}`,
                spec: {
                    ...c,
                    localPosition: [pos.x, pos.y, pos.z],
                    localQuaternion: [quat.x, quat.y, quat.z, quat.w],
                },
            });
        }
    }
    if (out.length === 0)
        throw new KhrValidationError("KhrVehicle: chassis has no colliders");
    return out;
}

function computeRotorLocalMatrix(
    node: THREE.Object3D,
    scene: THREE.Object3D,
    center: readonly [number, number, number],
    quaternion?: readonly [number, number, number, number],
): THREE.Matrix4 {
    const nodeToScene = new THREE.Matrix4();
    let cur: THREE.Object3D | null = node;
    while (cur && cur !== scene) {
        if (!cur.parent)
            throw new KhrValidationError(`Rotor node "${node.name}" is detached from bundle.scene`);
        cur.updateMatrix();
        nodeToScene.premultiply(cur.matrix);
        cur = cur.parent;
    }
    if (cur !== scene && node !== scene)
        throw new KhrValidationError(`Rotor node "${node.name}" is not under bundle.scene`);
    const slotInScene = new THREE.Matrix4().compose(
        new THREE.Vector3(center[0], center[1], center[2]),
        quaternion
            ? new THREE.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]).normalize()
            : new THREE.Quaternion(),
        new THREE.Vector3(1, 1, 1),
    );
    return slotInScene.invert().multiply(nodeToScene);
}

function clearSingleWheelInfo(w: WheelInfoType): void {
    w.rayHit = null;
    w.rayHitBody = null;
    w.floatImp.set(0, 0, 0);
    w.lngFricImp.set(0, 0, 0);
    w.latFricImp.set(0, 0, 0);
    w.lngSlipRatio = 0;
    w.latSlipRatio = 0;
    w.slipStrength = 0;
    w.isOnPlatform = false;
    w.effInertia = 0;
}

function resetWheelHits(handle: EcctrlVehicleHandle | null, filter?: (name: string | undefined) => boolean): void {
    if (!handle)
        return;
    for (const wheelRef of handle.wheelsInfo.values()) {
        const w = wheelRef.current as WheelInfoType | null;
        if (w && (!filter || filter(w.name)))
            clearSingleWheelInfo(w);
    }
}

function resetPropellerOutputs(handle: EcctrlVehicleHandle | null): void {
    if (!handle)
        return;
    for (const propRef of handle.propellersInfo.values()) {
        const p = propRef.current as PropellerInfoType | null;
        if (!p)
            continue;
        p.lx = 0; p.ly = 0; p.lz = 0;
        p.ax = 0; p.ay = 0; p.az = 0;
        p.thrustDir.set(0, 0, 0);
        p.torqueDir.set(0, 0, 0);
        p.thrustImpulse.set(0, 0, 0);
        p.torqueImpulse.set(0, 0, 0);
        p.finalThrottle = 0;
        p.throttle = 0;
        p.setThrottle?.(0);
    }
}

interface DisabledRotorsFlags {
    wheelsAllOff: boolean;
    frontOff: boolean;
    rearOff: boolean;
    propsOff: boolean;
    frontWheelNames: ReadonlySet<string>;
}

function resetDisabledRotors(handle: EcctrlVehicleHandle | null, opts: DisabledRotorsFlags): void {
    if (opts.wheelsAllOff) {
        resetWheelHits(handle);
    }
    else if (opts.frontOff || opts.rearOff) {
        resetWheelHits(handle, (name) => (name && opts.frontWheelNames.has(name) ? opts.frontOff : opts.rearOff));
    }
    if (opts.propsOff)
        resetPropellerOutputs(handle);
}

function RotorVisualSlot({
    object,
    center,
    quaternion,
    scene,
    origin,
}: {
    object: THREE.Object3D;
    center: readonly [number, number, number];
    quaternion?: readonly [number, number, number, number];
    scene: THREE.Object3D;
    origin?: SubtreeOrigin;
}): React.JSX.Element {
    const slotRef = useRef<THREE.Group | null>(null);
    useLayoutEffect(() => {
        const slot = slotRef.current;
        if (!slot?.parent)
            return;
        const snap = origin ?? captureSubtreeOrigins([object], scene).get(object)!;
        scene.updateMatrixWorld(true);
        const localMat = computeRotorLocalMatrix(object, scene, center, quaternion);
        if (snap.origParent)
            snap.origParent.remove(object);
        const pos = new THREE.Vector3(), quat = new THREE.Quaternion(), scl = new THREE.Vector3();
        localMat.decompose(pos, quat, scl);
        object.position.copy(pos);
        object.quaternion.copy(quat.normalize());
        object.scale.copy(scl);
        slot.add(object);
        const untrack = registerMountedSubtree(snap.ownerScene, () => {
            const p = object.position.clone(), q = object.quaternion.clone(), s = object.scale.clone();
            if (object.parent === slot)
                slot.remove(object);
            restoreSubtreeOrigin(object, snap);
            return () => {
                if (snap.origParent)
                    snap.origParent.remove(object);
                object.position.copy(p);
                object.quaternion.copy(q);
                object.scale.copy(s);
                slot.add(object);
            };
        });
        return () => {
            untrack();
            if (object.parent === slot)
                slot.remove(object);
            restoreSubtreeOrigin(object, snap);
        };
    }, [object, center, quaternion, scene, origin]);
    return <group ref={slotRef} />;
}

function hasActiveMovement(m: VehicleInput): boolean {
    return Boolean(
        m.forward || m.backward || m.steerLeft || m.steerRight || m.brake ||
        m.throttleUp || m.throttleDown || m.yawLeft || m.yawRight ||
        m.pitchForward || m.pitchBackward || m.rollLeft || m.rollRight ||
        (m.joystickL && (Math.abs(m.joystickL.x) > 1e-4 || Math.abs(m.joystickL.y) > 1e-4)) ||
        (m.joystickR && (Math.abs(m.joystickR.x) > 1e-4 || Math.abs(m.joystickR.y) > 1e-4)),
    );
}

const patchedHandles = new WeakSet<EcctrlVehicleHandle>();

export function KhrVehicle({ scale, colliders, bundle, ...rest }: KhrVehicleProps): React.JSX.Element {
    if (colliders !== undefined) {
        throw new KhrValidationError("KhrVehicle: colliders prop cannot be overridden; edit colliders in the .glb asset");
    }
    assertUnitScale(scale, "KhrVehicle");
    useMemo(() => assertSceneIdentityWorldScale(bundle.scene), [bundle]);

    const {
        ref,
        vehicleRef,
        jointsRef,
        position,
        rotation,
        quaternion,
        enable = true,
        enableCustomGravity = false,
        carConfig,
        droneConfig,
        wheelConfig,
        frontWheelConfig,
        rearWheelConfig,
        propellerConfig,
        linearVelocity,
        angularVelocity,
        gravityScale,
        children,
        ...ecctrlRest
    } = rest;

    const { world } = useRapier();
    const worldGravMag = Math.hypot(world.gravity.x, world.gravity.y, world.gravity.z);
    const refGrav = worldGravMag > 1e-4 ? worldGravMag : STANDARD_GRAVITY;

    const [isZeroG, setIsZeroG] = useState(() => (enableCustomGravity ? true : worldGravMag <= 1e-6));
    const zeroGRef = useRef(isZeroG);
    const [chassisBody, setChassisBody] = useState<RAPIER.RigidBody | null>(null);

    const chassis = useMemo(() => computeChassisMassProps(bundle.bodies), [bundle]);
    const collidersList = useMemo(() => buildChassisColliders(bundle.bodies), [bundle.bodies]);

    useEffect(() => {
        if (chassisBody)
            applyRapierBodyMass(chassisBody, chassis.lumpedMass);
    }, [chassisBody, chassis.lumpedMass]);

    const solvedWheels = useMemo(
        () => solveWheelLoads(bundle.wheels, bundle.totalDynamicMass, bundle.centerOfMass, 0.7, refGrav),
        [bundle.wheels, bundle.totalDynamicMass, bundle.centerOfMass, refGrav],
    );
    const rotorOrigins = useMemo(
        () => captureSubtreeOrigins([...bundle.wheels.map((w) => w.object), ...bundle.propellers.map((p) => p.object)], bundle.scene),
        [bundle.wheels, bundle.propellers, bundle.scene],
    );

    const forwardSign = useMemo(() => {
        const steered = solvedWheels.filter((w) => w.tag.steer && !w.tag.steerInvert);
        if (steered.length === 0)
            return 1;
        const meanSteerZ = steered.reduce((s, w) => s + w.mountPos[2], 0) / steered.length;
        return meanSteerZ < bundle.centerOfMass[2] - 1e-3 ? -1 : 1;
    }, [solvedWheels, bundle.centerOfMass]);

    const frontWheelNames = useMemo(
        () => new Set(solvedWheels.filter((w) => (w.mountPos[2] - bundle.centerOfMass[2]) * forwardSign >= 0).map((w) => w.tag.nodeName)),
        [solvedWheels, bundle.centerOfMass, forwardSign],
    );
    const wheelsAllOff = !enable || isZeroG || (wheelConfig?.enable === false && frontWheelConfig?.enable !== true && rearWheelConfig?.enable !== true);
    const frontOff = frontWheelConfig?.enable === false || (frontWheelConfig?.enable === undefined && wheelConfig?.enable === false);
    const rearOff = rearWheelConfig?.enable === false || (rearWheelConfig?.enable === undefined && wheelConfig?.enable === false);
    const propsOff = !enable || propellerConfig?.enable === false;

    const disabledRotorsRef = useRef<DisabledRotorsFlags>({ wheelsAllOff, frontOff, rearOff, propsOff, frontWheelNames });
    disabledRotorsRef.current = { wheelsAllOff, frontOff, rearOff, propsOff, frontWheelNames };

    const innerRef = useRef<EcctrlVehicleHandle | null>(null);
    useLayoutEffect(() => {
        resetDisabledRotors(innerRef.current, disabledRotorsRef.current);
    }, [wheelsAllOff, frontOff, rearOff, propsOff, frontWheelNames]);

    useFrame(() => {
        const handle = innerRef.current;
        if (handle?.body && handle.body !== chassisBody)
            setChassisBody(handle.body);
        if (!enable)
            return;
        let gSq: number;
        if (enableCustomGravity) {
            if (handle?.body && !handle.body.isSleeping()) {
                gSq = handle.gravityMag * handle.gravityMag;
            }
            else {
                const t = handle?.body?.translation();
                if (t)
                    _samplePos.set(t.x, t.y, t.z);
                else
                    _samplePos.set(bodyPos[0], bodyPos[1], bodyPos[2]);
                gSq = useCustomGravity.getState().gravityField(_samplePos).lengthSq();
            }
        }
        else {
            const g = world.gravity;
            gSq = g.x * g.x + g.y * g.y + g.z * g.z;
        }
        const nextZeroG = gSq <= 1e-12;
        if (nextZeroG !== zeroGRef.current) {
            zeroGRef.current = nextZeroG;
            setIsZeroG(nextZeroG);
        }
        if (nextZeroG) {
            resetWheelHits(handle);
        }
        else {
            const flags = disabledRotorsRef.current;
            if (flags.wheelsAllOff || flags.frontOff || flags.rearOff || flags.propsOff)
                resetDisabledRotors(handle, flags);
        }
    });

    const [px, py, pz] = readPosTuple(position);
    const [qx, qy, qz, qw] = readSpawnQuat(rotation, quaternion);
    const bodyPos = useMemo<Vec3Tuple>(() => [px, py, pz], [px, py, pz]);
    const bodyQuat = useMemo<QuatTuple>(() => [qx, qy, qz, qw], [qx, qy, qz, qw]);

    const assignVehicleRef = useCallback(
        (handle: EcctrlVehicleHandle | null) => {
            if (handle && !patchedHandles.has(handle)) {
                patchedHandles.add(handle);
                const origSetMovement = handle.setMovement;
                handle.setMovement = (state: VehicleInput) => {
                    origSetMovement(state);
                    if (hasActiveMovement(state) && handle.body?.isSleeping())
                        handle.body.wakeUp();
                };
            }
            innerRef.current = handle;
            setChassisBody(handle?.body ?? null);
            assignRef(vehicleRef, handle);
            assignRef(ref, handle);
        },
        [ref, vehicleRef],
    );

    const omiSteerAngle = useMemo(
        () => bundle.wheels.find((w) => w.steer && w.maxSteeringAngle !== undefined && w.maxSteeringAngle > 0)?.maxSteeringAngle,
        [bundle.wheels],
    );
    const mergedCarConfig = useMemo(
        () => (bundle.tune || carConfig || omiSteerAngle !== undefined
            ? { ...(omiSteerAngle !== undefined ? { maxSteerAngle: omiSteerAngle } : {}), ...bundle.tune, ...carConfig }
            : undefined),
        [omiSteerAngle, bundle.tune, carConfig],
    );
    const mergedDroneConfig = useMemo(
        () => (bundle.droneTune || droneConfig ? { ...bundle.droneTune, ...droneConfig } : undefined),
        [bundle.droneTune, droneConfig],
    );

    const effLinVel = linearVelocity ?? chassis.linearVelocity;
    const effAngVel = angularVelocity ?? chassis.angularVelocity;
    const effGravScale = gravityScale ?? chassis.gravityFactor;
    const wheelEnable = enable && !isZeroG;

    return (
        <>
            <EcctrlVehicle
                ref={assignVehicleRef}
                enable={enable}
                enableCustomGravity={enableCustomGravity}
                position={bodyPos}
                quaternion={bodyQuat}
                {...(effLinVel !== undefined ? { linearVelocity: effLinVel } : {})}
                {...(effAngVel !== undefined ? { angularVelocity: effAngVel } : {})}
                {...(effGravScale !== undefined ? { gravityScale: effGravScale } : {})}
                carConfig={mergedCarConfig}
                droneConfig={mergedDroneConfig}
                {...ecctrlRest}
            >
                <BundleSceneHost bundle={bundle} owner="KhrVehicle" />
                {children}
                {collidersList.map(({ key, spec }) => (
                    <KhrCollider key={key} c={spec} />
                ))}
                {solvedWheels.map(({ tag, cornerMassKg, springK, dampingC, mountPos }) => {
                    const axleCfg = frontWheelNames.has(tag.nodeName) ? frontWheelConfig : rearWheelConfig;
                    const wheelVol = Math.max(Math.PI * tag.radius * tag.radius * tag.width, 1e-6);
                    const wheelModelDensity = tag.wheelDensity ?? (UNSPRUNG_CORNER_RATIO * cornerMassKg) / wheelVol;
                    const tagWheelProps: KhrWheelConfig = {
                        rayLength: tag.travel,
                        rayShapeR: tag.radius,
                        rayShapeH: tag.width * 0.5,
                        wheelModelRadius: tag.radius,
                        wheelModelDensity,
                        springK,
                        dampingC,
                        steerWheel: tag.steer,
                        driveWheel: tag.drive,
                        brakeWheel: tag.brake,
                        maxBrakeTorque: tag.maxBrakeTorque ?? cornerMassKg * refGrav * tag.radius,
                        ...(tag.steerInvert !== undefined ? { steerInvert: tag.steerInvert } : {}),
                        ...(tag.driveInvert !== undefined ? { driveInvert: tag.driveInvert } : {}),
                        ...(tag.driveTorqueWeight !== undefined ? { driveTorqueWeight: tag.driveTorqueWeight } : {}),
                        ...(tag.rollingResistanceCoef !== undefined ? { rollingResistanceCoef: tag.rollingResistanceCoef } : {}),
                        ...(tag.tireGripFactor !== undefined ? { tireGripFactor: tag.tireGripFactor } : {}),
                        ...(tag.latFrictionEllipseScale !== undefined ? { latFrictionEllipseScale: tag.latFrictionEllipseScale } : {}),
                        ...(tag.relaxLngRate !== undefined ? { relaxLngRate: tag.relaxLngRate } : {}),
                        ...(tag.relaxLatRate !== undefined ? { relaxLatRate: tag.relaxLatRate } : {}),
                        ...(tag.lngSlipRatioCurveData !== undefined ? { lngSlipRatioCurveData: tag.lngSlipRatioCurveData } : {}),
                        ...(tag.latSlipRatioCurveData !== undefined ? { latSlipRatioCurveData: tag.latSlipRatioCurveData } : {}),
                    };
                    return (
                        <ShapeCastWheel
                            key={tag.nodeName}
                            name={tag.nodeName}
                            position={mountPos}
                            quaternion={tag.quaternion ?? [0, 0, 0, 1]}
                            {...tagWheelProps}
                            {...wheelConfig}
                            {...axleCfg}
                            enable={wheelEnable && (axleCfg?.enable ?? wheelConfig?.enable ?? true)}
                        >
                            <RotorVisualSlot
                                object={tag.object}
                                center={tag.position}
                                quaternion={tag.quaternion}
                                scene={bundle.scene}
                                origin={rotorOrigins.get(tag.object)}
                            />
                        </ShapeCastWheel>
                    );
                })}
                {bundle.propellers.map((tag: EcctrlPropellerTag) => {
                    const propEnabled = enable && (propellerConfig?.enable ?? true);
                    const visual = (
                        <RotorVisualSlot
                            object={tag.object}
                            center={tag.position}
                            quaternion={tag.quaternion}
                            scene={bundle.scene}
                            origin={rotorOrigins.get(tag.object)}
                        />
                    );
                    if (!propEnabled) {
                        return (
                            <group
                                key={`${tag.nodeName}:off`}
                                name={tag.nodeName}
                                position={[tag.position[0], tag.position[1], tag.position[2]]}
                                quaternion={tag.quaternion ?? [0, 0, 0, 1]}
                            >
                                {visual}
                            </group>
                        );
                    }
                    return (
                        <ThrustPropeller
                            key={tag.nodeName}
                            name={tag.nodeName}
                            position={[tag.position[0], tag.position[1], tag.position[2]]}
                            quaternion={tag.quaternion ?? [0, 0, 0, 1]}
                            debug={false}
                            maxThrust={tag.maxThrust}
                            {...(tag.torqueRatio !== undefined ? { torqueRatio: tag.torqueRatio } : {})}
                            {...(tag.invertThrust !== undefined ? { invertThrust: tag.invertThrust } : {})}
                            {...(tag.invertTorque !== undefined ? { invertTorque: tag.invertTorque } : {})}
                            {...propellerConfig}
                            enable={true}
                        >
                            {visual}
                        </ThrustPropeller>
                    );
                })}
            </EcctrlVehicle>
            <KhrVehicleJoints
                bundleScene={bundle.scene}
                bodies={bundle.bodies}
                vehicleRef={innerRef}
                chassisBody={chassisBody}
                spawnPos={bodyPos}
                spawnQuat={bodyQuat}
                enableCustomGravity={enableCustomGravity}
                linearVelocity={effLinVel}
                angularVelocity={effAngVel}
                gravityScale={effGravScale}
                userData={ecctrlRest.userData}
                jointsRef={jointsRef}
            />
        </>
    );
}
