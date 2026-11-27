import React, { useCallback, useImperativeHandle, useMemo, useRef } from "react";
import type { ThreeElements } from "@react-three/fiber";
import type { RapierRigidBody } from "@react-three/rapier";
import * as THREE from "three";
import { planAssembly } from "./assemblyPlan.js";
import { type AssemblyRigidBodyProps, AssemblyRootBody } from "./AssemblyRootBody.js";
import { BundleSceneHost, MountSubtreeSlot, captureSubtreeOrigins } from "./BundleSceneHost.js";
import { assertSceneIdentityWorldScale, assertUnitScale } from "./khrInertia.js";
import { type KhrAssetBundle } from "./khrTypes.js";
import type { JointRuntime } from "./physicsEcs.js";

export interface KhrAssemblyHandle {
    /** RigidBody owning `nodeName`, or null if absent from the KHR bundle. */
    getBody: (nodeName: string) => RapierRigidBody | null;
    /** Sets the authored Rapier motor position target on one articulated node. */
    setJointTarget: (nodeName: string, target: number) => void;
    /** Sets the motor position target on every articulated node whose `extras.role` matches. */
    setRoleTarget: (role: string, target: number) => void;
}

type GroupProps = Omit<ThreeElements["group"], "ref" | "position" | "rotation" | "quaternion" | "scale" | "children">;

export interface KhrAssemblyProps extends GroupProps {
    ref?: React.Ref<KhrAssemblyHandle>;
    bundle: KhrAssetBundle;
    position?: [number, number, number];
    rotation?: [number, number, number];
    quaternion?: [number, number, number, number];
    scale?: ThreeElements["group"]["scale"];
    fixedRoot?: boolean;
    rigidBodyProps?: AssemblyRigidBodyProps;
    children?: React.ReactNode;
}

export function KhrAssembly({
    ref,
    bundle,
    position = [0, 0, 0],
    rotation,
    quaternion,
    scale,
    fixedRoot = false,
    rigidBodyProps,
    children,
    ...groupProps
}: KhrAssemblyProps): React.JSX.Element {
    assertUnitScale(scale, "KhrAssembly");
    bundle.scene.updateMatrixWorld(true);
    assertSceneIdentityWorldScale(bundle.scene);

    const plans = useMemo(() => planAssembly(bundle), [bundle]);
    const origins = useMemo(() => (plans.length > 1 ? captureSubtreeOrigins(plans.map((p) => p.mountNode)) : null), [plans]);
    const [px, py, pz] = position;
    const [rx, ry, rz] = rotation ?? [0, 0, 0];
    const [qx, qy, qz, qw] = quaternion ?? [NaN, NaN, NaN, NaN];
    const spawnQuat = useMemo(
        () => (quaternion ? new THREE.Quaternion(qx, qy, qz, qw) : new THREE.Quaternion().setFromEuler(new THREE.Euler(rx, ry, rz))),
        [qx, qy, qz, qw, rx, ry, rz],
    );
    const rootPoses = useMemo(
        () =>
            plans.map((p) => {
                const pos = new THREE.Vector3().fromArray(p.spec.localPosition).applyQuaternion(spawnQuat).add(new THREE.Vector3(px, py, pz));
                const quat = spawnQuat.clone().multiply(new THREE.Quaternion().fromArray(p.spec.localQuaternion));
                return {
                    position: [pos.x, pos.y, pos.z] as [number, number, number],
                    quaternion: [quat.x, quat.y, quat.z, quat.w] as [number, number, number, number],
                };
            }),
        [plans, spawnQuat, px, py, pz],
    );

    const bodiesRef = useRef(new Map<string, React.RefObject<RapierRigidBody | null>>());
    const jointsRef = useRef(new Map<string, Map<string, JointRuntime>>());
    const nodeToRoot = useMemo(() => {
        const m = new Map<string, string>();
        for (const p of plans) {
            m.set(p.spec.nodeName, p.spec.nodeName);
            for (const part of p.parts)
                m.set(part.spec.nodeName, p.spec.nodeName);
        }
        return m;
    }, [plans]);

    const handleBody = useCallback((nodeName: string, rb: React.RefObject<RapierRigidBody | null> | null) => {
        if (rb)
            bodiesRef.current.set(nodeName, rb);
        else
            bodiesRef.current.delete(nodeName);
    }, []);
    const handleJoints = useCallback((rootName: string, map: Map<string, JointRuntime> | null) => {
        if (map)
            jointsRef.current.set(rootName, map);
        else
            jointsRef.current.delete(rootName);
    }, []);

    useImperativeHandle(ref, () => ({
        getBody: (nodeName: string) => bodiesRef.current.get(nodeName)?.current ?? null,
        setJointTarget: (nodeName: string, target: number) => {
            const rt = jointsRef.current.get(nodeToRoot.get(nodeName) ?? "")?.get(nodeName);
            if (rt)
                rt.applyTarget(target);
        },
        setRoleTarget: (role: string, target: number) => {
            for (const map of jointsRef.current.values()) {
                for (const rt of map.values()) {
                    if (rt.role === role)
                        rt.applyTarget(target);
                }
            }
        },
    }), [nodeToRoot]);

    if (plans.length === 1) {
        const plan = plans[0];
        return (
            <group {...groupProps}>
                <AssemblyRootBody
                    key={`${bundle.scene.uuid}:${plan.spec.nodeName}`}
                    plan={plan}
                    position={rootPoses[0].position}
                    quaternion={rootPoses[0].quaternion}
                    fixedRoot={fixedRoot}
                    rigidBodyProps={rigidBodyProps}
                    onBody={handleBody}
                    onJoints={handleJoints}
                >
                    <group position={plan.sceneInRootPos} quaternion={plan.sceneInRootQuat}>
                        <BundleSceneHost bundle={bundle} owner="KhrAssembly" />
                        {children}
                    </group>
                </AssemblyRootBody>
            </group>
        );
    }

    return (
        <group {...groupProps}>
            <group position={position} quaternion={[spawnQuat.x, spawnQuat.y, spawnQuat.z, spawnQuat.w]}>
                <BundleSceneHost bundle={bundle} owner="KhrAssembly" />
                {children}
            </group>
            {plans.map((plan, idx) => (
                <AssemblyRootBody
                    key={`${bundle.scene.uuid}:${plan.spec.nodeName}`}
                    plan={plan}
                    position={rootPoses[idx].position}
                    quaternion={rootPoses[idx].quaternion}
                    fixedRoot={fixedRoot}
                    rigidBodyProps={rigidBodyProps}
                    onBody={handleBody}
                    onJoints={handleJoints}
                >
                    <MountSubtreeSlot
                        object={plan.mountNode}
                        position={plan.mountPosition}
                        quaternion={plan.mountQuaternion}
                        origin={origins?.get(plan.mountNode)}
                    />
                </AssemblyRootBody>
            ))}
        </group>
    );
}
