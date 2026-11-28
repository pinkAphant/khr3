import React, { useCallback, useContext, useEffect, useImperativeHandle, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { RigidBody, useRapier, type RapierRigidBody, type RigidBodyProps } from "@react-three/rapier";
import type RAPIER from "@dimforge/rapier3d-compat";
import { useCustomGravity } from "@pinkaphant/ecctrl/gravity";
import type { EcctrlVehicleHandle } from "@pinkaphant/ecctrl/vehicle";
import { MountSubtreeSlot, captureSubtreeOrigins, type SubtreeOrigin } from "./clone.js";
import {
    applyRapierBodyMass,
    bodyPoseMatrix,
    computeLumpedMassProps,
    type LumpedMassProps,
    type QuatTuple,
    type Vec3Tuple,
} from "./inertia.js";
import {
    ATTACHED_PANEL_INTERACTION_GROUPS,
    buildJointRuntime,
    closedTargetForLimits,
    createRapierImpulseJoint,
    isJointOpen,
    openTargetForLimits,
    toggleDrivenJoint,
} from "./joints.js";
import { type PlannedCollider } from "./KhrAssembly.js";
import { KhrCollider } from "./KhrCollider.js";
import {
    JointRegistryContext,
    type JointRuntime,
    type KhrBodySpec,
    type KhrJointSpec,
    KhrValidationError,
} from "./types.js";

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

export interface PlannedVehiclePart {
    spec: KhrBodySpec;
    joint: KhrJointSpec;
    parent: PlannedVehiclePart | null;
    restRelParentPos: Vec3Tuple;
    restRelParentQuat: QuatTuple;
    restRelChassisPos: Vec3Tuple;
    restRelChassisQuat: QuatTuple;
    axisParent: Vec3Tuple;
    axisChild: Vec3Tuple;
    lumpedMass: LumpedMassProps | null;
    colliders: PlannedCollider[];
}

export function mergeVehiclePanelUserData(userData?: RigidBodyProps["userData"]): Record<string, unknown> {
    const base = (userData && typeof userData === "object" ? userData : {}) as Record<string, unknown>;
    const ecctrl = (base.ecctrl && typeof base.ecctrl === "object" ? base.ecctrl : {}) as Record<string, unknown>;
    return {
        ...base,
        ecctrl: {
            ...ecctrl,
            excludeVehicleRay: true,
        },
    };
}

export function planVehicleJointParts(bodies: readonly KhrBodySpec[]): PlannedVehiclePart[] {
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
        const rcQuatTuple: QuatTuple = [rcQuat.x, rcQuat.y, rcQuat.z, rcQuat.w];
        const invPartFrame = new THREE.Matrix4()
            .compose(new THREE.Vector3().fromArray(b.localPosition), new THREE.Quaternion(), one)
            .invert();
        const lumpedMass = computeLumpedMassProps([b], invPartFrame);
        const frameMat = new THREE.Matrix4().makeRotationFromQuaternion(rcQuat);
        const cPos = new THREE.Vector3(), cQuat = new THREE.Quaternion();
        const colliders: PlannedCollider[] = b.colliders.map((spec) => {
            const colQuat = new THREE.Quaternion().fromArray(spec.localQuaternion);
            const local = new THREE.Matrix4().compose(new THREE.Vector3().fromArray(spec.localPosition), colQuat, one).premultiply(frameMat);
            local.decompose(cPos, cQuat, scl);
            cQuat.normalize();
            return {
                spec: spec.collisionGroups === undefined ? { ...spec, collisionGroups: ATTACHED_PANEL_INTERACTION_GROUPS } : spec,
                position: [cPos.x, cPos.y, cPos.z],
                quaternion: [cQuat.x, cQuat.y, cQuat.z, cQuat.w],
            };
        });

        const plan: PlannedVehiclePart = {
            spec: b,
            joint,
            parent,
            restRelParentPos: anchorParent,
            restRelParentQuat: rcQuatTuple,
            restRelChassisPos: [rcPos.x, rcPos.y, rcPos.z],
            restRelChassisQuat: rcQuatTuple,
            axisParent: [axParentVec.x, axParentVec.y, axParentVec.z],
            axisChild: [axChildVec.x, axChildVec.y, axChildVec.z],
            lumpedMass,
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
    panelUserData,
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
    panelUserData: Record<string, unknown>;
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
        if (childBody && part.lumpedMass)
            applyRapierBodyMass(childBody, part.lumpedMass);
    }, [childBody, part.lumpedMass]);

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
            userData={panelUserData}
            onContactForce={onForce}
            {...(linVel ? { linearVelocity: linVel } : {})}
            {...(angVel ? { angularVelocity: angVel } : {})}
            {...(gravScale === undefined ? {} : { gravityScale: gravScale })}
        >
            <MountSubtreeSlot object={part.spec.object} position={[0, 0, 0]} quaternion={part.restRelChassisQuat} origin={origin} />
            {part.colliders.map((col, cIdx) => (
                <KhrCollider key={`vj_${part.spec.nodeName}:${cIdx}`} c={col.spec} position={col.position} quaternion={col.quaternion} />
            ))}
        </RigidBody>
    );
}

const _pPos = new THREE.Vector3();
const _pQuat = new THREE.Quaternion();
const _anchor = new THREE.Vector3();

export function KhrVehicleJoints({
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
    userData,
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
    userData?: RigidBodyProps["userData"];
    jointsRef?: React.Ref<KhrVehicleJointsHandle>;
}): React.JSX.Element | null {
    const { world } = useRapier();
    const applyGravityField = useCustomGravity((state) => state.applyGravityField);
    const parts = useMemo(() => planVehicleJointParts(bodies), [bodies]);
    const origins = useMemo(
        () => (parts.length > 0 ? captureSubtreeOrigins(parts.map((p) => p.spec.object), bundleScene) : null),
        [parts, bundleScene],
    );
    const panelUserData = useMemo(() => mergeVehiclePanelUserData(userData), [userData]);
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
                if (rt && toggleDrivenJoint(rt))
                    return true;
            }
            return false;
        },
        toggleRolePrefix: (prefix: string) => {
            const low = prefix.toLowerCase();
            const matches = [...runtimeMapRef.current.values()].filter(
                (rt) => !rt.detached && rt.drive?.stiffness !== undefined && ((rt.role ?? "").toLowerCase().startsWith(low) || rt.nodeName.toLowerCase().startsWith(low)),
            );
            if (matches.length === 0)
                return;
            const anyOpen = matches.some((rt) => isJointOpen(rt));
            for (const rt of matches) {
                const limits = rt.limits ?? [0, Math.PI / 2];
                rt.applyTarget(anyOpen ? closedTargetForLimits(limits) : openTargetForLimits(limits));
            }
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
                rb.setTranslation({ x: _anchor.x, y: _anchor.y, z: _anchor.z }, true);
                rb.setRotation(pr, true);
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
                    panelUserData={panelUserData}
                    parentBody={part.parent ? (partBodies.get(part.parent.spec.nodeName) ?? null) : (chassisBody ?? vehicleRef.current?.body ?? null)}
                    origin={origins?.get(part.spec.object)}
                    onBody={handleBody}
                    onJointReady={handleJointReady}
                />
            ))}
        </>
    );
}
