import React, { useCallback, useContext, useEffect, useLayoutEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { type RapierRigidBody, RigidBody, type RigidBodyProps, useRapier } from "@react-three/rapier";
import type { PartPlan, RootPlan } from "./assemblyPlan.js";
import { MountSubtreeSlot, captureSubtreeOrigins, type SubtreeOrigin } from "./BundleSceneHost.js";
import { buildJointRuntime, createRapierImpulseJoint } from "./jointDynamics.js";
import { KhrCollider } from "./KhrCollider.js";
import { JointRegistryContext, type JointRuntime } from "./physicsEcs.js";

export type AssemblyRigidBodyProps = Omit<
    RigidBodyProps,
    "type" | "position" | "rotation" | "quaternion" | "colliders" | "children" | "ref"
>;

export interface AssemblyRootBodyProps {
    plan: RootPlan;
    position: [number, number, number];
    quaternion: [number, number, number, number];
    fixedRoot: boolean;
    rigidBodyProps?: AssemblyRigidBodyProps;
    onBody: (nodeName: string, rb: React.RefObject<RapierRigidBody | null> | null) => void;
    onJoints: (rootName: string, joints: Map<string, JointRuntime> | null) => void;
    children?: React.ReactNode;
}

function ArticulatedPartBody({
    part,
    rootSpec,
    rootPos,
    rootQuat,
    parentBodyRef,
    origin,
    onBody,
    onJointReady,
}: {
    part: PartPlan;
    rootSpec: RootPlan["spec"];
    rootPos: readonly [number, number, number];
    rootQuat: readonly [number, number, number, number];
    parentBodyRef: React.RefObject<RapierRigidBody | null>;
    origin?: SubtreeOrigin;
    onBody: (nodeName: string, rb: React.RefObject<RapierRigidBody | null> | null) => void;
    onJointReady: (nodeName: string, rt: JointRuntime | null) => void;
}): React.JSX.Element {
    const { rapier, world } = useRapier();
    const registry = useContext(JointRegistryContext);
    const childRef = useRef<RapierRigidBody | null>(null);
    const rtRef = useRef<JointRuntime | null>(null);

    const worldPose = useMemo(() => {
        const rq = new THREE.Quaternion(rootQuat[0], rootQuat[1], rootQuat[2], rootQuat[3]).normalize();
        const pos = new THREE.Vector3().fromArray(part.restRelRootPos).applyQuaternion(rq).add(new THREE.Vector3(rootPos[0], rootPos[1], rootPos[2]));
        return {
            pos: [pos.x, pos.y, pos.z] as [number, number, number],
            quat: [rq.x, rq.y, rq.z, rq.w] as [number, number, number, number],
        };
    }, [part.restRelRootPos, rootPos, rootQuat]);

    useEffect(() => {
        onBody(part.spec.nodeName, childRef);
        return () => onBody(part.spec.nodeName, null);
    }, [part.spec.nodeName, onBody]);

    useEffect(() => {
        const rb = childRef.current;
        const pm = part.partMass;
        if (rb && pm) {
            rb.setAdditionalMassProperties(pm.mass, pm.centerOfMass, pm.principalAngularInertia, pm.angularInertiaLocalFrame, true);
            rb.recomputeMassPropertiesFromColliders();
        }
    }, [part.partMass]);

    useEffect(() => {
        const parentBody = parentBodyRef.current;
        const childBody = childRef.current;
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
            restRelQuat: [0, 0, 0, 1],
            limits: part.joint.limits,
            drive: part.joint.drive,
            breakaway: part.spec.breakaway,
            rapierJoint: imp,
            parentBody,
            childBody,
        });
        rtRef.current = rt;
        registry?.add(rt);
        onJointReady(rt.nodeName, rt);
        return () => {
            rtRef.current = null;
            registry?.delete(rt);
            onJointReady(rt.nodeName, null);
            if (rt.rapierJoint && world.getImpulseJoint(rt.rapierJoint.handle))
                world.removeImpulseJoint(rt.rapierJoint, true);
        };
    }, [rapier, world, part, parentBodyRef, registry, onJointReady]);

    const onForce = part.spec.breakaway
        ? (p: { totalForceMagnitude: number }) => { rtRef.current?.applyImpactImpulse(p.totalForceMagnitude * Math.max(world.timestep, 1 / 240)); }
        : undefined;

    const linVel = part.spec.linearVelocity ?? rootSpec.linearVelocity;
    const angVel = part.spec.angularVelocity ?? rootSpec.angularVelocity;
    const gravScale = part.spec.gravityFactor ?? rootSpec.gravityFactor;

    return (
        <RigidBody
            ref={childRef}
            type={part.spec.isKinematic ? "kinematicPosition" : "dynamic"}
            position={worldPose.pos}
            quaternion={worldPose.quat}
            colliders={false}
            onContactForce={onForce}
            {...(linVel ? { linearVelocity: linVel } : {})}
            {...(angVel ? { angularVelocity: angVel } : {})}
            {...(gravScale === undefined ? {} : { gravityScale: gravScale })}
        >
            <MountSubtreeSlot object={part.spec.object} position={[0, 0, 0]} quaternion={part.restRelRootQuat} origin={origin} />
            {part.colliders.map((col, cIdx) => (
                <KhrCollider
                    key={`${part.spec.nodeName}:${cIdx}`}
                    c={col.spec}
                    position={col.position}
                    quaternion={col.quaternion}
                    massProperties={col.massProperties}
                />
            ))}
        </RigidBody>
    );
}

export function AssemblyRootBody({
    plan,
    position,
    quaternion,
    fixedRoot,
    rigidBodyProps,
    onBody,
    onJoints,
    children,
}: AssemblyRootBodyProps): React.JSX.Element {
    const rbRef = useRef<RapierRigidBody | null>(null);
    const partBodyRefs = useMemo(
        () => new Map(plan.parts.map((p) => [p.spec.nodeName, { current: null as RapierRigidBody | null }])),
        [plan],
    );
    const partOrigins = useMemo(
        () => (plan.parts.length > 0 ? captureSubtreeOrigins(plan.parts.map((p) => p.spec.object)) : null),
        [plan.parts],
    );
    const jointsMapRef = useRef(new Map<string, JointRuntime>());
    const rootSpec = plan.spec;

    useEffect(() => {
        onBody(rootSpec.nodeName, rbRef);
        onJoints(rootSpec.nodeName, jointsMapRef.current);
        return () => {
            onBody(rootSpec.nodeName, null);
            onJoints(rootSpec.nodeName, null);
        };
    }, [rootSpec.nodeName, onBody, onJoints]);

    useEffect(() => {
        const rb = rbRef.current;
        const rm = plan.rootMass;
        if (rb && rm) {
            rb.setAdditionalMassProperties(rm.mass, rm.centerOfMass, rm.principalAngularInertia, rm.angularInertiaLocalFrame, true);
            rb.recomputeMassPropertiesFromColliders();
        }
    }, [plan.rootMass]);

    const handlePartBody = useCallback((nodeName: string, refObj: React.RefObject<RapierRigidBody | null> | null) => {
        const slot = partBodyRefs.get(nodeName);
        if (slot)
            slot.current = refObj?.current ?? null;
        onBody(nodeName, refObj);
    }, [partBodyRefs, onBody]);

    const handleJointReady = useCallback((nodeName: string, rt: JointRuntime | null) => {
        if (rt)
            jointsMapRef.current.set(nodeName, rt);
        else
            jointsMapRef.current.delete(nodeName);
    }, []);

    const bodyType = rootSpec.isKinematic ? (fixedRoot ? "fixed" : "kinematicPosition") : "dynamic";

    return (
        <>
            <RigidBody
                {...rigidBodyProps}
                ref={rbRef}
                type={bodyType}
                position={position}
                quaternion={quaternion}
                colliders={false}
                {...(rootSpec.linearVelocity ? { linearVelocity: rootSpec.linearVelocity } : {})}
                {...(rootSpec.angularVelocity ? { angularVelocity: rootSpec.angularVelocity } : {})}
                {...(rootSpec.gravityFactor === undefined ? {} : { gravityScale: rootSpec.gravityFactor })}
            >
                {children}
                {plan.colliders.map((col, idx) => (
                    <KhrCollider key={`r${idx}`} c={col.spec} position={col.position} quaternion={col.quaternion} massProperties={col.massProperties} />
                ))}
            </RigidBody>
            {plan.parts.map((part) => (
                <ArticulatedPartBody
                    key={part.spec.nodeName}
                    part={part}
                    rootSpec={rootSpec}
                    rootPos={position}
                    rootQuat={quaternion}
                    parentBodyRef={part.parent ? partBodyRefs.get(part.parent.spec.nodeName)! : rbRef}
                    origin={partOrigins?.get(part.spec.object)}
                    onBody={handlePartBody}
                    onJointReady={handleJointReady}
                />
            ))}
        </>
    );
}
