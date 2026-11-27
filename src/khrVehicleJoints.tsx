import React, { useCallback, useContext, useEffect, useImperativeHandle, useMemo, useRef } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { RigidBody, useRapier, type RapierRigidBody } from "@react-three/rapier";
import { useCustomGravity } from "@pinkaphant/ecctrl/gravity";
import type { EcctrlVehicleHandle } from "@pinkaphant/ecctrl/vehicle";
import { MountSubtreeSlot, captureSubtreeOrigins, type SubtreeOrigin } from "./BundleSceneHost.js";
import { buildJointRuntime, createRapierImpulseJoint } from "./jointDynamics.js";
import { KhrCollider } from "./KhrCollider.js";
import { type KhrBodySpec } from "./khrTypes.js";
import {
    isVehicleJointOpen,
    openTargetForLimits,
    planVehicleJointParts,
    shiftAttachedPanels,
    stepVehicleArticulatedPanels,
    syncAttachedPanelsToParent,
    type PlannedVehiclePart,
} from "./khrVehicleJointHelpers.js";
import { JointRegistryContext, type JointRuntime } from "./physicsEcs.js";

const ATTACHED_PANEL_USER_DATA = { ecctrl: { excludeVehicleRay: true } };

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

interface VehiclePanelProps {
    part: PlannedVehiclePart;
    spawnPos: readonly [number, number, number];
    spawnQuat: readonly [number, number, number, number];
    linearVelocity?: [number, number, number];
    angularVelocity?: [number, number, number];
    gravityScale?: number;
    parentBodyGetter: () => RapierRigidBody | null;
    origin?: SubtreeOrigin;
    onBody: (nodeName: string, rb: RapierRigidBody | null) => void;
    onJointReady: (nodeName: string, obj: THREE.Object3D, rt: JointRuntime | null) => void;
}

function VehicleArticulatedPanel({
    part, spawnPos, spawnQuat, linearVelocity, angularVelocity, gravityScale, parentBodyGetter, origin, onBody, onJointReady,
}: VehiclePanelProps): React.JSX.Element {
    const { rapier, world } = useRapier();
    const registry = useContext(JointRegistryContext);
    const childRef = useRef<RapierRigidBody | null>(null);
    const rtRef = useRef<JointRuntime | null>(null);
    const parentBodyGetterRef = useRef(parentBodyGetter);
    parentBodyGetterRef.current = parentBodyGetter;

    const worldPose = useMemo(() => {
        const sq = new THREE.Quaternion(spawnQuat[0], spawnQuat[1], spawnQuat[2], spawnQuat[3]).normalize();
        const pos = new THREE.Vector3().fromArray(part.restRelChassisPos).applyQuaternion(sq).add(new THREE.Vector3(spawnPos[0], spawnPos[1], spawnPos[2]));
        return { pos: [pos.x, pos.y, pos.z] as [number, number, number], quat: [sq.x, sq.y, sq.z, sq.w] as [number, number, number, number] };
    }, [part.restRelChassisPos, spawnPos, spawnQuat]);

    useEffect(() => {
        onBody(part.spec.nodeName, childRef.current);
        return () => onBody(part.spec.nodeName, null);
    }, [part.spec.nodeName, onBody]);

    useEffect(() => {
        const parentBody = parentBodyGetterRef.current(), childBody = childRef.current;
        if (!parentBody || !childBody) return;
        const imp = createRapierImpulseJoint(
            rapier, world, parentBody, childBody, part.joint,
            part.restRelParentPos, [0, 0, 0], part.axisParent, part.axisChild, part.restRelParentQuat,
        );
        const rt = buildJointRuntime({
            rapier, world, nodeName: part.spec.nodeName, role: part.joint.role, kind: part.joint.kind, object: part.spec.object,
            axisParent: part.axisParent, anchorParent: part.restRelParentPos, restRelQuat: [0, 0, 0, 1],
            limits: part.joint.limits, drive: part.joint.drive, breakaway: part.spec.breakaway, rapierJoint: imp, parentBody, childBody,
        });
        rtRef.current = rt;
        registry?.add(rt);
        onJointReady(rt.nodeName, part.spec.object, rt);
        return () => {
            rtRef.current = null;
            registry?.delete(rt);
            onJointReady(rt.nodeName, part.spec.object, null);
            if (rt.rapierJoint && world.getImpulseJoint(rt.rapierJoint.handle)) world.removeImpulseJoint(rt.rapierJoint, true);
        };
    }, [rapier, world, part, registry, onJointReady]);

    const onForce = part.spec.breakaway
        ? (p: { totalForceMagnitude: number }) => { rtRef.current?.applyImpactImpulse(p.totalForceMagnitude * Math.max(world.timestep, 1 / 240)); }
        : undefined;
    const linVel = part.spec.linearVelocity ?? linearVelocity;
    const angVel = part.spec.angularVelocity ?? angularVelocity;
    const gravScale = part.spec.gravityFactor ?? gravityScale;

    return (
        <RigidBody
            ref={childRef} type={part.spec.isKinematic ? "kinematicPosition" : "dynamic"}
            position={worldPose.pos} quaternion={worldPose.quat} colliders={false}
            userData={ATTACHED_PANEL_USER_DATA} onContactForce={onForce}
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

export function KhrVehicleJoints({
    bodies, vehicleRef, spawnPos, spawnQuat, enableCustomGravity = false, linearVelocity, angularVelocity, gravityScale, jointsRef,
}: {
    bodies: readonly KhrBodySpec[];
    vehicleRef: React.RefObject<EcctrlVehicleHandle | null>;
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
    const origins = useMemo(() => (parts.length > 0 ? captureSubtreeOrigins(parts.map((p) => p.spec.object)) : null), [parts]);
    const bodyMapRef = useRef(new Map<string, RapierRigidBody>());
    const runtimeMapRef = useRef(new Map<string, JointRuntime>());
    const byObjectRef = useRef(new Map<THREE.Object3D, JointRuntime>());
    const detachedClearedRef = useRef(new WeakSet<RapierRigidBody>());

    useFrame(() => {
        if (parts.length === 0) return;
        stepVehicleArticulatedPanels({
            parts, bodyMap: bodyMapRef.current, runtimeMap: runtimeMapRef.current, detachedCleared: detachedClearedRef.current,
            enableCustomGravity, timestep: world.timestep, applyGravityField: enableCustomGravity ? applyGravityField : undefined,
        });
    });

    const handleBody = useCallback((nodeName: string, rb: RapierRigidBody | null) => {
        if (rb) bodyMapRef.current.set(nodeName, rb); else bodyMapRef.current.delete(nodeName);
    }, []);
    const handleJointReady = useCallback((nodeName: string, obj: THREE.Object3D, rt: JointRuntime | null) => {
        if (rt) { runtimeMapRef.current.set(nodeName, rt); byObjectRef.current.set(obj, rt); }
        else { runtimeMapRef.current.delete(nodeName); byObjectRef.current.delete(obj); }
    }, []);

    useImperativeHandle(jointsRef, () => ({
        get hasJoints() { return parts.length > 0; },
        get jointNames() { return parts.map((p) => p.spec.nodeName); },
        getBody: (nodeName: string) => bodyMapRef.current.get(nodeName) ?? null,
        getJointRuntime: (nodeName: string) => runtimeMapRef.current.get(nodeName) ?? null,
        getJointAngle: (nodeName: string) => runtimeMapRef.current.get(nodeName)?.getCoordinate() ?? null,
        setJointTarget: (nodeName: string, target: number) => { runtimeMapRef.current.get(nodeName)?.applyTarget(target); },
        setRoleTarget: (role: string, target: number) => {
            for (const rt of runtimeMapRef.current.values()) if (rt.role === role) rt.applyTarget(target);
        },
        toggleHitObject: (hit: THREE.Object3D | null) => {
            for (let cur = hit; cur; cur = cur.parent) {
                const rt = byObjectRef.current.get(cur);
                if (rt && rt.limits && rt.drive?.stiffness !== undefined) {
                    rt.applyTarget(isVehicleJointOpen(rt) ? 0 : openTargetForLimits(rt.limits));
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
            if (matches.length === 0) return;
            const anyOpen = matches.some((rt) => isVehicleJointOpen(rt));
            for (const rt of matches) rt.applyTarget(anyOpen ? 0 : openTargetForLimits(rt.limits!));
        },
        shiftBodies: (dx: number, dy: number, dz: number) => shiftAttachedPanels(parts, bodyMapRef.current, runtimeMapRef.current, dx, dy, dz),
        syncBodiesToParent: () => syncAttachedPanelsToParent(parts, bodyMapRef.current, runtimeMapRef.current, vehicleRef.current?.body ?? null),
    }), [parts, vehicleRef]);

    if (parts.length === 0) return null;
    return (
        <>
            {parts.map((part) => (
                <VehicleArticulatedPanel
                    key={part.spec.nodeName} part={part} spawnPos={spawnPos} spawnQuat={spawnQuat}
                    linearVelocity={linearVelocity} angularVelocity={angularVelocity} gravityScale={gravityScale}
                    parentBodyGetter={() => (part.parent ? bodyMapRef.current.get(part.parent.spec.nodeName) ?? null : vehicleRef.current?.body ?? null)}
                    origin={origins?.get(part.spec.object)} onBody={handleBody} onJointReady={handleJointReady}
                />
            ))}
        </>
    );
}
