import React, { useCallback, useContext, useEffect, useImperativeHandle, useLayoutEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { RigidBody, useRapier, type RapierRigidBody } from "@react-three/rapier";
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
    MountSubtreeSlot,
    captureSubtreeOrigins,
    registerMountedSubtree,
    restoreSubtreeOrigin,
    type SubtreeOrigin,
} from "./clone.js";
import {
    assertSceneIdentityWorldScale,
    assertUnitScale,
    bodyPoseMatrix,
    computeLumpedMassProps,
    type LumpedMassProps,
    type QuatTuple,
    type Vec3Tuple,
} from "./inertia.js";
import {
    ATTACHED_PANEL_INTERACTION_GROUPS,
    buildJointRuntime,
    createRapierImpulseJoint,
    isJointOpen,
    openTargetForLimits,
} from "./joints.js";
import { toColliderMass, type PlannedCollider } from "./KhrAssembly.js";
import { type ColliderMass, KhrCollider } from "./KhrCollider.js";
import {
    JointRegistryContext,
    assignRef,
    type EcctrlPropellerTag,
    type JointRuntime,
    type KhrAssetBundle,
    type KhrBodySpec,
    type KhrCarConfig,
    type KhrColliderSpec,
    type KhrDroneConfig,
    type KhrJointSpec,
    KhrValidationError,
} from "./types.js";
import { STANDARD_GRAVITY, solveWheelLoads, type SolvedWheel } from "./wheelLoads.js";

export { solveWheelLoads, type SolvedWheel };

export type KhrWheelConfig = Partial<Omit<ShapeCastWheelProps, "ref" | "key" | "children">>;
export type KhrPropellerConfig = Partial<Omit<ThrustPropellerProps, "ref" | "key" | "children">>;

export interface KhrVehicleJointsHandle {
    readonly hasJoints: boolean;
    readonly jointNames: readonly string[];
    getBody: (nodeName: string) => RapierRigidBody | null;
    getJointRuntime: (nodeName: string) => JointRuntime | null;
    getJointAngle: (nodeName: string) => number | null;
    setJointTarget: (nodeName: string, target: number) => void;
    setRoleTarget: (role: string, target: number) => void;
    toggleHitObject: (hit: THREE.Object3D | null) => boolean;
    toggleRolePrefix: (prefix: string) => void;
    shiftBodies: (dx: number, dy: number, dz: number) => void;
    syncBodiesToParent: () => void;
}

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
const ATTACHED_PANEL_USER_DATA = { ecctrl: { excludeVehicleRay: true } };
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

interface ChassisMassSummary extends LumpedMassProps {
    linearVelocity?: Vec3Tuple;
    angularVelocity?: Vec3Tuple;
    gravityFactor?: number;
}

function computeChassisMassProps(bodies: readonly KhrBodySpec[]): ChassisMassSummary {
    const dyn = bodies.filter((b) => !b.isKinematic && b.joint === null);
    if (dyn.length === 0)
        throw new KhrValidationError("KhrVehicle: bundle has no dynamic rigid body for the chassis");
    const lumped = computeLumpedMassProps(dyn);
    if (!lumped || !(lumped.mass > 0))
        throw new KhrValidationError("KhrVehicle: chassis dynamic mass must be positive");
    if (!lumped.inertia.every((v) => Number.isFinite(v) && v > 0))
        throw new KhrValidationError("KhrVehicle: chassis principal inertia must be positive and finite");

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
        linearVelocity = [vx / lumped.mass, vy / lumped.mass, vz / lumped.mass];
    }

    const rootDyn = dyn.find((b) => b.angularVelocity !== undefined);
    const angularVelocity: Vec3Tuple | undefined = rootDyn?.angularVelocity
        ? [rootDyn.angularVelocity[0], rootDyn.angularVelocity[1], rootDyn.angularVelocity[2]]
        : undefined;

    return {
        mass: lumped.mass,
        com: lumped.com,
        inertia: lumped.inertia,
        inertiaFrame: lumped.inertiaFrame,
        ...(linearVelocity ? { linearVelocity } : {}),
        ...(angularVelocity ? { angularVelocity } : {}),
        ...(gravityFactor !== undefined ? { gravityFactor } : {}),
    };
}

function buildChassisColliders(
    bodies: readonly KhrBodySpec[],
    chassis: LumpedMassProps,
): Array<{ key: string; spec: KhrColliderSpec; massProperties?: ColliderMass }> {
    const out: Array<{ key: string; spec: KhrColliderSpec; massProperties?: ColliderMass }> = [];
    const one = new THREE.Vector3(1, 1, 1);
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scl = new THREE.Vector3();

    for (const b of bodies) {
        // Include dynamic chassis roots AND collider-only child/root nodes (`hasExplicitMotion === false`)
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
            const isFirst = out.length === 0;
            const rigidColMat = isFirst ? new THREE.Matrix4().compose(pos, quat, one) : undefined;
            out.push({
                key: `${b.nodeName}:${idx}`,
                spec: {
                    ...c,
                    localPosition: [pos.x, pos.y, pos.z],
                    localQuaternion: [quat.x, quat.y, quat.z, quat.w],
                },
                ...(isFirst ? { massProperties: toColliderMass(chassis, rigidColMat, quat.clone()) } : {}),
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

interface PlannedVehiclePart {
    spec: KhrBodySpec;
    joint: KhrJointSpec;
    parent: PlannedVehiclePart | null;
    restRelParentPos: Vec3Tuple;
    restRelParentQuat: QuatTuple;
    restRelChassisPos: Vec3Tuple;
    restRelChassisQuat: QuatTuple;
    axisParent: Vec3Tuple;
    axisChild: Vec3Tuple;
    partMass?: ColliderMass;
    colliders: PlannedCollider[];
}

function planVehicleJointParts(bodies: readonly KhrBodySpec[]): PlannedVehiclePart[] {
    const dyn = bodies.filter((b) => !b.isKinematic);
    const byName = new Map(dyn.map((b) => [b.nodeName, b]));
    const jointed = dyn.filter((b) => b.joint !== null);
    if (jointed.length === 0)
        return [];

    const depthMemo = new Map<string, number>();
    const depthOf = (b: KhrBodySpec, seen = new Set<string>()): number => {
        const hit = depthMemo.get(b.nodeName);
        if (hit !== undefined)
            return hit;
        if (!b.joint) {
            depthMemo.set(b.nodeName, 0);
            return 0;
        }
        if (seen.has(b.nodeName))
            throw new KhrValidationError(`Joint cycle through "${b.nodeName}"`);
        seen.add(b.nodeName);
        const parent = byName.get(b.joint.connectedNodeName);
        if (!parent) {
            throw new KhrValidationError(`Vehicle joint on "${b.nodeName}" targets "${b.joint.connectedNodeName}", which is not a dynamic KHR body`);
        }
        const d = 1 + depthOf(parent, seen);
        depthMemo.set(b.nodeName, d);
        return d;
    };

    jointed.sort((a, b) => depthOf(a) - depthOf(b));
    const one = new THREE.Vector3(1, 1, 1);
    const rpPos = new THREE.Vector3(), rpQuat = new THREE.Quaternion();
    const rcPos = new THREE.Vector3(), rcQuat = new THREE.Quaternion(), scl = new THREE.Vector3();
    const partByName = new Map<string, PlannedVehiclePart>();

    return jointed.map((b) => {
        const joint = b.joint!;
        const parent = partByName.get(joint.connectedNodeName) ?? null;
        const bodyMat = bodyPoseMatrix(b);
        const parentMat = parent ? bodyPoseMatrix(parent.spec) : new THREE.Matrix4();
        parentMat.clone().invert().multiply(bodyMat).decompose(rpPos, rpQuat, scl);
        bodyMat.decompose(rcPos, rcQuat, scl);
        rcQuat.normalize();

        const axChildVec = new THREE.Vector3().fromArray(joint.axis).normalize();
        const axParentVec = axChildVec.clone().applyQuaternion(rcQuat).normalize();
        const anchorParent: Vec3Tuple = parent
            ? [rcPos.x - parent.restRelChassisPos[0], rcPos.y - parent.restRelChassisPos[1], rcPos.z - parent.restRelChassisPos[2]]
            : [rpPos.x, rpPos.y, rpPos.z];
        const bodyAlignInv = new THREE.Matrix4().compose(new THREE.Vector3().fromArray(b.localPosition), new THREE.Quaternion(), one).invert();
        const ownLumped = computeLumpedMassProps([b], bodyAlignInv);
        const frameMat = new THREE.Matrix4().makeRotationFromQuaternion(rcQuat);
        const cPos = new THREE.Vector3(), cQuat = new THREE.Quaternion();
        const colliders: PlannedCollider[] = b.colliders.map((spec, idx) => {
            const colQuat = new THREE.Quaternion().fromArray(spec.localQuaternion);
            const local = new THREE.Matrix4().compose(new THREE.Vector3().fromArray(spec.localPosition), colQuat, one).premultiply(frameMat);
            local.decompose(cPos, cQuat, scl);
            cQuat.normalize();
            const planned: PlannedCollider = {
                spec: spec.collisionGroups === undefined ? { ...spec, collisionGroups: ATTACHED_PANEL_INTERACTION_GROUPS } : spec,
                position: [cPos.x, cPos.y, cPos.z],
                quaternion: [cQuat.x, cQuat.y, cQuat.z, cQuat.w],
            };
            if (idx === 0 && ownLumped)
                planned.massProperties = toColliderMass(ownLumped, local, cQuat);
            return planned;
        });

        const plan: PlannedVehiclePart = {
            spec: b,
            joint,
            parent,
            restRelParentPos: anchorParent,
            restRelParentQuat: [rcQuat.x, rcQuat.y, rcQuat.z, rcQuat.w],
            restRelChassisPos: [rcPos.x, rcPos.y, rcPos.z],
            restRelChassisQuat: [rcQuat.x, rcQuat.y, rcQuat.z, rcQuat.w],
            axisParent: [axParentVec.x, axParentVec.y, axParentVec.z],
            axisChild: [axChildVec.x, axChildVec.y, axChildVec.z],
            ...(ownLumped && colliders.length === 0 ? { partMass: toColliderMass(ownLumped) } : {}),
            colliders,
        };
        partByName.set(b.nodeName, plan);
        return plan;
    });
}

function VehicleArticulatedPanel({
    part,
    spawnPos,
    spawnQuat,
    linearVelocity,
    angularVelocity,
    gravityScale,
    parentBody,
    origin,
    onBody,
    onJointReady,
}: {
    part: PlannedVehiclePart;
    spawnPos: readonly [number, number, number];
    spawnQuat: readonly [number, number, number, number];
    linearVelocity?: [number, number, number];
    angularVelocity?: [number, number, number];
    gravityScale?: number;
    parentBody: RAPIER.RigidBody | null;
    origin?: SubtreeOrigin;
    onBody: (nodeName: string, rb: RapierRigidBody | null) => void;
    onJointReady: (nodeName: string, obj: THREE.Object3D, rt: JointRuntime | null) => void;
}): React.JSX.Element {
    const { rapier, world } = useRapier();
    const registry = useContext(JointRegistryContext);
    const rtRef = useRef<JointRuntime | null>(null);
    const [childBody, setChildBody] = useState<RapierRigidBody | null>(null);

    const worldPose = useMemo(() => {
        const sq = new THREE.Quaternion(spawnQuat[0], spawnQuat[1], spawnQuat[2], spawnQuat[3]).normalize();
        const pos = new THREE.Vector3().fromArray(part.restRelChassisPos).applyQuaternion(sq).add(new THREE.Vector3(spawnPos[0], spawnPos[1], spawnPos[2]));
        return { pos: [pos.x, pos.y, pos.z] as Vec3Tuple, quat: [sq.x, sq.y, sq.z, sq.w] as QuatTuple };
    }, [part.restRelChassisPos, spawnPos, spawnQuat]);

    const assignChildRef = useCallback((rb: RapierRigidBody | null) => {
        setChildBody(rb);
        onBody(part.spec.nodeName, rb);
    }, [part.spec.nodeName, onBody]);

    useEffect(() => () => onBody(part.spec.nodeName, null), [part.spec.nodeName, onBody]);

    useEffect(() => {
        const pm = part.partMass;
        if (childBody && pm) {
            childBody.setAdditionalMassProperties(pm.mass, pm.centerOfMass, pm.principalAngularInertia, pm.angularInertiaLocalFrame, true);
            childBody.recomputeMassPropertiesFromColliders();
        }
    }, [childBody, part.partMass]);

    useEffect(() => {
        if (!parentBody || !childBody)
            return;
        const imp = createRapierImpulseJoint(
            rapier,
            world,
            parentBody,
            childBody,
            part.joint,
            part.restRelParentPos,
            [0, 0, 0],
            part.axisParent,
            part.axisChild,
            part.restRelParentQuat,
        );
        const rt = buildJointRuntime({
            rapier,
            world,
            nodeName: part.spec.nodeName,
            role: part.joint.role,
            kind: part.joint.kind,
            object: part.spec.object,
            axisParent: part.axisParent,
            anchorParent: part.restRelParentPos,
            limits: part.joint.limits,
            drive: part.joint.drive,
            breakaway: part.spec.breakaway,
            rapierJoint: imp,
            parentBody,
            childBody,
        });
        rtRef.current = rt;
        registry?.add(rt);
        onJointReady(rt.nodeName, part.spec.object, rt);
        return () => {
            rtRef.current = null;
            registry?.delete(rt);
            onJointReady(rt.nodeName, part.spec.object, null);
            if (rt.rapierJoint && world.getImpulseJoint(rt.rapierJoint.handle))
                world.removeImpulseJoint(rt.rapierJoint, true);
        };
    }, [rapier, world, part, parentBody, childBody, registry, onJointReady]);

    const onForce = useMemo(
        () =>
            part.spec.breakaway
                ? (p: { totalForceMagnitude: number }) => {
                    if (world.timestep > 0)
                        rtRef.current?.applyImpactImpulse(p.totalForceMagnitude * world.timestep);
                }
                : undefined,
        [part.spec.breakaway, world],
    );

    const linVel = part.spec.linearVelocity ?? linearVelocity;
    const angVel = part.spec.angularVelocity ?? angularVelocity;
    const gravScale = part.spec.gravityFactor ?? gravityScale;

    return (
        <RigidBody
            ref={assignChildRef}
            type={part.spec.isKinematic ? "kinematicPosition" : "dynamic"}
            position={worldPose.pos}
            quaternion={worldPose.quat}
            colliders={false}
            userData={ATTACHED_PANEL_USER_DATA}
            onContactForce={onForce}
            {...(linVel ? { linearVelocity: linVel } : {})}
            {...(angVel ? { angularVelocity: angVel } : {})}
            {...(gravScale === undefined ? {} : { gravityScale: gravScale })}
        >
            <MountSubtreeSlot object={part.spec.object} position={[0, 0, 0]} quaternion={part.restRelChassisQuat} origin={origin} />
            {part.colliders.map((col, cIdx) => (
                <KhrCollider key={`vj_${part.spec.nodeName}:${cIdx}`} c={col.spec} position={col.position} quaternion={col.quaternion} massProperties={col.massProperties} />
            ))}
        </RigidBody>
    );
}

const _pPos = new THREE.Vector3();
const _pQuat = new THREE.Quaternion();
const _rQuat = new THREE.Quaternion();
const _cQuat = new THREE.Quaternion();
const _anchor = new THREE.Vector3();

function KhrVehicleJoints({
    bundleScene,
    bodies,
    vehicleRef,
    chassisBody,
    spawnPos,
    spawnQuat,
    enableCustomGravity = false,
    linearVelocity,
    angularVelocity,
    gravityScale,
    jointsRef,
}: {
    bundleScene: THREE.Object3D;
    bodies: readonly KhrBodySpec[];
    vehicleRef: React.RefObject<EcctrlVehicleHandle | null>;
    chassisBody: RAPIER.RigidBody | null;
    spawnPos: readonly [number, number, number];
    spawnQuat: readonly [number, number, number, number];
    enableCustomGravity?: boolean;
    linearVelocity?: [number, number, number];
    angularVelocity?: [number, number, number];
    gravityScale?: number;
    jointsRef?: React.Ref<KhrVehicleJointsHandle>;
}): React.JSX.Element | null {
    const { world } = useRapier();
    const applyGravityField = useCustomGravity((state) => state.applyGravityField);
    const parts = useMemo(() => planVehicleJointParts(bodies), [bodies]);
    const origins = useMemo(
        () => (parts.length > 0 ? captureSubtreeOrigins(parts.map((p) => p.spec.object), bundleScene) : null),
        [parts, bundleScene],
    );
    const bodyMapRef = useRef(new Map<string, RapierRigidBody>());
    const [partBodies, setPartBodies] = useState<ReadonlyMap<string, RapierRigidBody>>(() => new Map());
    const runtimeMapRef = useRef(new Map<string, JointRuntime>());
    const byObjectRef = useRef(new Map<THREE.Object3D, JointRuntime>());

    useFrame(() => {
        if (!enableCustomGravity || parts.length === 0 || world.timestep <= 0)
            return;
        for (const rb of bodyMapRef.current.values()) {
            if (rb.isDynamic())
                applyGravityField(rb, world.timestep);
        }
    });

    const handleBody = useCallback((nodeName: string, rb: RapierRigidBody | null) => {
        if (rb)
            bodyMapRef.current.set(nodeName, rb);
        else
            bodyMapRef.current.delete(nodeName);
        setPartBodies((prev) => {
            if ((prev.get(nodeName) ?? null) === rb)
                return prev;
            const next = new Map(prev);
            if (rb)
                next.set(nodeName, rb);
            else
                next.delete(nodeName);
            return next;
        });
    }, []);

    const handleJointReady = useCallback((nodeName: string, obj: THREE.Object3D, rt: JointRuntime | null) => {
        if (rt) {
            runtimeMapRef.current.set(nodeName, rt);
            byObjectRef.current.set(obj, rt);
        }
        else {
            runtimeMapRef.current.delete(nodeName);
            byObjectRef.current.delete(obj);
        }
    }, []);

    useImperativeHandle(jointsRef, () => ({
        get hasJoints() {
            return parts.length > 0;
        },
        get jointNames() {
            return parts.map((p) => p.spec.nodeName);
        },
        getBody: (nodeName: string) => bodyMapRef.current.get(nodeName) ?? null,
        getJointRuntime: (nodeName: string) => runtimeMapRef.current.get(nodeName) ?? null,
        getJointAngle: (nodeName: string) => runtimeMapRef.current.get(nodeName)?.getCoordinate() ?? null,
        setJointTarget: (nodeName: string, target: number) => {
            runtimeMapRef.current.get(nodeName)?.applyTarget(target);
        },
        setRoleTarget: (role: string, target: number) => {
            for (const rt of runtimeMapRef.current.values())
                if (rt.role === role) rt.applyTarget(target);
        },
        toggleHitObject: (hit: THREE.Object3D | null) => {
            for (let cur = hit; cur; cur = cur.parent) {
                const rt = byObjectRef.current.get(cur);
                if (rt && rt.limits && rt.drive?.stiffness !== undefined) {
                    rt.applyTarget(isJointOpen(rt) ? 0 : openTargetForLimits(rt.limits));
                    return true;
                }
            }
            return false;
        },
        toggleRolePrefix: (prefix: string) => {
            const low = prefix.toLowerCase();
            const matches = [...runtimeMapRef.current.values()].filter(
                (rt) => rt.limits !== null && rt.drive?.stiffness !== undefined && ((rt.role ?? "").toLowerCase().startsWith(low) || rt.nodeName.toLowerCase().startsWith(low)),
            );
            if (matches.length === 0)
                return;
            const anyOpen = matches.some((rt) => isJointOpen(rt));
            for (const rt of matches)
                rt.applyTarget(anyOpen ? 0 : openTargetForLimits(rt.limits!));
        },
        shiftBodies: (dx: number, dy: number, dz: number) => {
            for (const part of parts) {
                const rb = bodyMapRef.current.get(part.spec.nodeName);
                const rt = runtimeMapRef.current.get(part.spec.nodeName);
                if (!rb || rt?.detached)
                    continue;
                const t = rb.translation();
                rb.setTranslation({ x: t.x + dx, y: t.y + dy, z: t.z + dz }, true);
            }
        },
        syncBodiesToParent: () => {
            const parent = vehicleRef.current?.body;
            if (!parent)
                return;
            const pt = parent.translation(), pr = parent.rotation(), lv = parent.linvel(), av = parent.angvel();
            _pPos.set(pt.x, pt.y, pt.z);
            _pQuat.set(pr.x, pr.y, pr.z, pr.w);
            for (const part of parts) {
                const rb = bodyMapRef.current.get(part.spec.nodeName);
                const rt = runtimeMapRef.current.get(part.spec.nodeName);
                if (!rb || rt?.detached)
                    continue;
                _anchor.fromArray(part.restRelChassisPos).applyQuaternion(_pQuat).add(_pPos);
                _cQuat.copy(_pQuat).multiply(_rQuat.fromArray(part.restRelChassisQuat)).normalize();
                rb.setTranslation({ x: _anchor.x, y: _anchor.y, z: _anchor.z }, true);
                rb.setRotation({ x: _cQuat.x, y: _cQuat.y, z: _cQuat.z, w: _cQuat.w }, true);
                rb.setLinvel(lv, true);
                rb.setAngvel(av, true);
            }
        },
    }), [parts, vehicleRef]);

    if (parts.length === 0)
        return null;
    return (
        <>
            {parts.map((part) => (
                <VehicleArticulatedPanel
                    key={part.spec.nodeName}
                    part={part}
                    spawnPos={spawnPos}
                    spawnQuat={spawnQuat}
                    linearVelocity={linearVelocity}
                    angularVelocity={angularVelocity}
                    gravityScale={gravityScale}
                    parentBody={part.parent ? (partBodies.get(part.parent.spec.nodeName) ?? null) : (chassisBody ?? vehicleRef.current?.body ?? null)}
                    origin={origins?.get(part.spec.object)}
                    onBody={handleBody}
                    onJointReady={handleJointReady}
                />
            ))}
        </>
    );
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
    const collidersList = useMemo(() => buildChassisColliders(bundle.bodies, chassis), [bundle.bodies, chassis]);
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
        return meanSteerZ < chassis.com[2] - 1e-3 ? -1 : 1;
    }, [solvedWheels, chassis.com]);

    const frontWheelNames = useMemo(
        () => new Set(solvedWheels.filter((w) => (w.mountPos[2] - chassis.com[2]) * forwardSign >= 0).map((w) => w.tag.nodeName)),
        [solvedWheels, chassis.com, forwardSign],
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
                {collidersList.map(({ key, spec, massProperties }) => (
                    <KhrCollider key={key} c={spec} massProperties={massProperties} />
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
                jointsRef={jointsRef}
            />
        </>
    );
}
