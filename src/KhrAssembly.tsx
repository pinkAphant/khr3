import React, { useCallback, useContext, useEffect, useImperativeHandle, useMemo, useRef } from "react";
import * as THREE from "three";
import { useFrame, type ThreeElements } from "@react-three/fiber";
import { RigidBody, useRapier, type RapierRigidBody, type RigidBodyProps } from "@react-three/rapier";
import { useCustomGravity } from "@pinkaphant/ecctrl/gravity";
import { BundleSceneHost, MountSubtreeSlot, captureSubtreeOrigins, type SubtreeOrigin } from "./clone.js";
import {
    applyRapierBodyMass,
    assertSceneIdentityWorldScale,
    assertUnitScale,
    bodyPoseMatrix,
    computeLumpedMassProps,
    isLiveRapierBody,
    type LumpedMassProps,
} from "./inertia.js";
import { buildJointRuntime, createRapierImpulseJoint, toggleDrivenJoint } from "./joints.js";
import { KhrCollider } from "./KhrCollider.js";
import {
    JointRegistryContext,
    type JointRuntime,
    type KhrAssetBundle,
    type KhrBodySpec,
    type KhrColliderSpec,
    type KhrJointSpec,
    KhrValidationError,
} from "./types.js";

type Vec3 = [number, number, number];
type Quat = [number, number, number, number];

export interface PlannedCollider {
    spec: KhrColliderSpec;
    position: Vec3;
    quaternion: Quat;
}

export interface PartPlan {
    spec: KhrBodySpec;
    joint: KhrJointSpec;
    parent: PartPlan | null;
    restRelParentPos: Vec3;
    restRelParentQuat: Quat;
    restRelRootPos: Vec3;
    restRelRootQuat: Quat;
    axisParent: Vec3;
    axisChild: Vec3;
    lumpedMass: LumpedMassProps | null;
    colliders: PlannedCollider[];
}

export interface RootPlan {
    spec: KhrBodySpec;
    sceneInRootPos: Vec3;
    sceneInRootQuat: Quat;
    mountNode: THREE.Object3D;
    mountPosition: Vec3;
    mountQuaternion: Quat;
    lumpedMass: LumpedMassProps | null;
    colliders: PlannedCollider[];
    parts: PartPlan[];
}

function matrixToRoot(node: THREE.Object3D, sceneRoot: THREE.Object3D): THREE.Matrix4 {
    const out = new THREE.Matrix4();
    for (let cur: THREE.Object3D | null = node; cur && cur !== sceneRoot; cur = cur.parent) {
        if (!cur.parent)
            throw new KhrValidationError(`Node "${node.name}" is detached from bundle.scene`);
        cur.updateMatrix();
        out.premultiply(cur.matrix);
    }
    return out;
}

function isSceneDescendant(node: THREE.Object3D, ancestor: THREE.Object3D): boolean {
    for (let cur: THREE.Object3D | null = node; cur; cur = cur.parent) {
        if (cur === ancestor)
            return true;
    }
    return false;
}

function lowestCommonAncestor(nodes: readonly THREE.Object3D[], sceneRoot: THREE.Object3D): THREE.Object3D {
    let candidate = nodes[0];
    while (candidate && candidate !== sceneRoot) {
        if (nodes.every((n) => isSceneDescendant(n, candidate)))
            return candidate;
        candidate = candidate.parent ?? sceneRoot;
    }
    return sceneRoot;
}

function planOwnColliders(body: KhrBodySpec, frameQuat?: THREE.Quaternion): PlannedCollider[] {
    const one = new THREE.Vector3(1, 1, 1);
    const frameMat = frameQuat ? new THREE.Matrix4().makeRotationFromQuaternion(frameQuat) : null;
    const pos = new THREE.Vector3(), quat = new THREE.Quaternion(), scl = new THREE.Vector3();
    return body.colliders.map((spec) => {
        const colQuat = new THREE.Quaternion().fromArray(spec.localQuaternion);
        const local = new THREE.Matrix4().compose(new THREE.Vector3().fromArray(spec.localPosition), colQuat, one);
        if (frameMat)
            local.premultiply(frameMat);
        local.decompose(pos, quat, scl);
        quat.normalize();
        return {
            spec,
            position: [pos.x, pos.y, pos.z],
            quaternion: [quat.x, quat.y, quat.z, quat.w],
        };
    });
}

export function planAssembly(bundle: KhrAssetBundle): RootPlan[] {
    const byName = new Map(bundle.bodies.map((b) => [b.nodeName, b]));
    const rootMemo = new Map<string, KhrBodySpec>();
    const depthMemo = new Map<string, number>();

    const resolveRootAndDepth = (b: KhrBodySpec, visiting = new Set<string>()): { root: KhrBodySpec; depth: number } => {
        const cachedRoot = rootMemo.get(b.nodeName);
        const cachedDepth = depthMemo.get(b.nodeName);
        if (cachedRoot && cachedDepth !== undefined)
            return { root: cachedRoot, depth: cachedDepth };
        if (!b.joint) {
            rootMemo.set(b.nodeName, b);
            depthMemo.set(b.nodeName, 0);
            return { root: b, depth: 0 };
        }
        if (visiting.has(b.nodeName))
            throw new KhrValidationError(`Joint cycle through "${b.nodeName}"`);
        visiting.add(b.nodeName);
        const parent = byName.get(b.joint.connectedNodeName);
        if (!parent) {
            throw new KhrValidationError(`Joint on "${b.nodeName}" targets "${b.joint.connectedNodeName}", which is not a KHR body`);
        }
        const up = resolveRootAndDepth(parent, visiting);
        rootMemo.set(b.nodeName, up.root);
        depthMemo.set(b.nodeName, up.depth + 1);
        return { root: up.root, depth: up.depth + 1 };
    };

    for (const b of bundle.bodies)
        resolveRootAndDepth(b);

    const plans = bundle.bodies
        .filter((b) => !b.joint)
        .map((root): RootPlan => {
            const rootMat = bodyPoseMatrix(root);
            const invRoot = rootMat.clone().invert();
            const descendants = bundle.bodies
                .filter((b) => b.joint && rootMemo.get(b.nodeName) === root)
                .sort((a, b) => (depthMemo.get(a.nodeName) ?? 0) - (depthMemo.get(b.nodeName) ?? 0));

            const partByName = new Map<string, PartPlan>();
            const parts: PartPlan[] = descendants.map((b) => {
                const joint = b.joint!;
                const parent = partByName.get(joint.connectedNodeName) ?? null;
                if (parent && !isSceneDescendant(b.object, parent.spec.object)) {
                    throw new KhrValidationError(`Jointed node "${b.nodeName}" must be a scene-graph descendant of its jointed parent "${joint.connectedNodeName}"`);
                }
                const parentSpec = parent ? parent.spec : root;
                const relParentMat = bodyPoseMatrix(parentSpec).clone().invert().multiply(bodyPoseMatrix(b));
                const relRootMat = invRoot.clone().multiply(bodyPoseMatrix(b));
                const rpPos = new THREE.Vector3(), tmpQ = new THREE.Quaternion(), rpScl = new THREE.Vector3();
                relParentMat.decompose(rpPos, tmpQ, rpScl);
                const rrPos = new THREE.Vector3(), rrQuat = new THREE.Quaternion();
                relRootMat.decompose(rrPos, rrQuat, rpScl);
                rrQuat.normalize();

                const axChildVec = new THREE.Vector3().fromArray(joint.axis).normalize();
                const axParentVec = axChildVec.clone().applyQuaternion(rrQuat).normalize();
                const anchorParent: Vec3 = parent
                    ? [rrPos.x - parent.restRelRootPos[0], rrPos.y - parent.restRelRootPos[1], rrPos.z - parent.restRelRootPos[2]]
                    : [rpPos.x, rpPos.y, rpPos.z];

                const rrQuatTuple: Quat = [rrQuat.x, rrQuat.y, rrQuat.z, rrQuat.w];
                const invPartFrame = new THREE.Matrix4()
                    .compose(new THREE.Vector3().fromArray(b.localPosition), new THREE.Quaternion().fromArray(root.localQuaternion), new THREE.Vector3(1, 1, 1))
                    .invert();
                const lumpedMass = computeLumpedMassProps([b], invPartFrame);
                const colliders = planOwnColliders(b, rrQuat);

                const plan: PartPlan = {
                    spec: b,
                    joint,
                    parent,
                    restRelParentPos: anchorParent,
                    restRelParentQuat: rrQuatTuple,
                    restRelRootPos: [rrPos.x, rrPos.y, rrPos.z],
                    restRelRootQuat: rrQuatTuple,
                    axisParent: [axParentVec.x, axParentVec.y, axParentVec.z],
                    axisChild: [axChildVec.x, axChildVec.y, axChildVec.z],
                    lumpedMass,
                    colliders,
                };
                partByName.set(b.nodeName, plan);
                return plan;
            });

            const lumpedMass = computeLumpedMassProps([root], invRoot);
            const colliders = planOwnColliders(root);
            const mountNode = lowestCommonAncestor([root.object, ...descendants.map((d) => d.object)], bundle.scene);
            const mountInRoot = invRoot.clone().multiply(matrixToRoot(mountNode, bundle.scene));

            const scenePos = new THREE.Vector3(), sceneQuat = new THREE.Quaternion(), sceneScl = new THREE.Vector3();
            invRoot.decompose(scenePos, sceneQuat, sceneScl);
            sceneQuat.normalize();
            const mountPos = new THREE.Vector3(), mountQuat = new THREE.Quaternion();
            mountInRoot.decompose(mountPos, mountQuat, sceneScl);
            mountQuat.normalize();

            return {
                spec: root,
                sceneInRootPos: [scenePos.x, scenePos.y, scenePos.z],
                sceneInRootQuat: [sceneQuat.x, sceneQuat.y, sceneQuat.z, sceneQuat.w],
                mountNode,
                mountPosition: [mountPos.x, mountPos.y, mountPos.z],
                mountQuaternion: [mountQuat.x, mountQuat.y, mountQuat.z, mountQuat.w],
                lumpedMass,
                colliders,
                parts,
            };
        });

    for (const a of plans) {
        for (const b of plans) {
            if (a !== b && isSceneDescendant(b.mountNode, a.mountNode)) {
                throw new KhrValidationError(`Root bodies "${a.spec.nodeName}" and "${b.spec.nodeName}" share scene subtree "${a.mountNode.name}"; split them into separate glTF branches`);
            }
        }
    }
    return plans;
}

export type AssemblyRigidBodyProps = Omit<
    RigidBodyProps,
    "type" | "position" | "rotation" | "quaternion" | "colliders" | "children" | "ref"
>;

export function ArticulatedPartBody({
    part,
    rootPos,
    rootQuat,
    linearVelocity,
    angularVelocity,
    gravityScale,
    parentBody,
    userData,
    origin,
    onBody,
    onJointReady,
}: {
    part: {
        readonly spec: KhrBodySpec;
        readonly joint: KhrJointSpec;
        readonly restRelParentPos: Vec3;
        readonly restRelParentQuat: Quat;
        readonly restRelRootPos: Vec3;
        readonly restRelRootQuat: Quat;
        readonly axisParent: Vec3;
        readonly axisChild: Vec3;
        readonly lumpedMass: LumpedMassProps | null;
        readonly colliders: readonly PlannedCollider[];
    };
    rootPos: readonly [number, number, number];
    rootQuat: readonly [number, number, number, number];
    linearVelocity?: [number, number, number];
    angularVelocity?: [number, number, number];
    gravityScale?: number;
    parentBody: RAPIER_RigidBodyLike | null;
    userData?: RigidBodyProps["userData"];
    origin?: SubtreeOrigin;
    onBody: (nodeName: string, rb: RapierRigidBody | null) => void;
    onJointReady: (nodeName: string, obj: THREE.Object3D, rt: JointRuntime | null) => void;
}): React.JSX.Element {
    const { rapier, world } = useRapier();
    const registry = useContext(JointRegistryContext);
    const rtRef = useRef<JointRuntime | null>(null);
    const [childBody, setChildBody] = React.useState<RapierRigidBody | null>(null);

    const worldPose = useMemo(() => {
        const rq = new THREE.Quaternion(rootQuat[0], rootQuat[1], rootQuat[2], rootQuat[3]).normalize();
        const pos = new THREE.Vector3().fromArray(part.restRelRootPos).applyQuaternion(rq).add(new THREE.Vector3(rootPos[0], rootPos[1], rootPos[2]));
        return {
            pos: [pos.x, pos.y, pos.z] as Vec3,
            quat: [rq.x, rq.y, rq.z, rq.w] as Quat,
        };
    }, [part.restRelRootPos, rootPos, rootQuat]);

    const assignChildRef = useCallback((rb: RapierRigidBody | null) => {
        setChildBody(rb);
        onBody(part.spec.nodeName, rb);
    }, [part.spec.nodeName, onBody]);

    useEffect(() => () => onBody(part.spec.nodeName, null), [part.spec.nodeName, onBody]);

    useEffect(() => {
        if (childBody && part.lumpedMass && isLiveRapierBody(world, childBody))
            applyRapierBodyMass(childBody, part.lumpedMass, world);
    }, [world, childBody, part.lumpedMass]);

    useEffect(() => {
        if (!parentBody || !childBody || !isLiveRapierBody(world, parentBody) || !isLiveRapierBody(world, childBody))
            return;
        const imp = createRapierImpulseJoint(
            rapier,
            world,
            parentBody as RapierRigidBody,
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
            parentBody: parentBody as RapierRigidBody,
            childBody,
        });
        rtRef.current = rt;
        registry?.add(rt);
        onJointReady(rt.nodeName, part.spec.object, rt);
        return () => {
            rtRef.current = null;
            registry?.delete(rt);
            onJointReady(rt.nodeName, part.spec.object, null);
            if (rt.rapierJoint && world.getImpulseJoint(rt.rapierJoint.handle) === rt.rapierJoint)
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
            onContactForce={onForce}
            {...(userData === undefined ? {} : { userData })}
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
                />
            ))}
        </RigidBody>
    );
}

type RAPIER_RigidBodyLike = { readonly handle: number };

function AssemblyRootBody({
    plan,
    bundleScene,
    position,
    quaternion,
    fixedRoot,
    rigidBodyProps,
    onBody,
    onJointReady,
    children,
}: {
    plan: RootPlan;
    bundleScene: THREE.Object3D;
    position: Vec3;
    quaternion: Quat;
    fixedRoot: boolean;
    rigidBodyProps?: AssemblyRigidBodyProps;
    onBody: (nodeName: string, rb: RapierRigidBody | null) => void;
    onJointReady: (nodeName: string, obj: THREE.Object3D, rt: JointRuntime | null) => void;
    children?: React.ReactNode;
}): React.JSX.Element {
    const { world } = useRapier();
    const [rootBody, setRootBody] = React.useState<RapierRigidBody | null>(null);
    const [partBodies, setPartBodies] = React.useState<ReadonlyMap<string, RapierRigidBody>>(() => new Map());

    const partOrigins = useMemo(
        () => (plan.parts.length > 0 ? captureSubtreeOrigins(plan.parts.map((p) => p.spec.object), bundleScene) : null),
        [plan.parts, bundleScene],
    );

    const rootSpec = plan.spec;
    const assignRootRef = useCallback((rb: RapierRigidBody | null) => {
        setRootBody(rb);
        onBody(rootSpec.nodeName, rb);
    }, [rootSpec.nodeName, onBody]);

    useEffect(() => () => onBody(rootSpec.nodeName, null), [rootSpec.nodeName, onBody]);

    useEffect(() => {
        if (rootBody && plan.lumpedMass && isLiveRapierBody(world, rootBody))
            applyRapierBodyMass(rootBody, plan.lumpedMass, world);
    }, [world, rootBody, plan.lumpedMass]);

    const handlePartBody = useCallback((nodeName: string, rb: RapierRigidBody | null) => {
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
        onBody(nodeName, rb);
    }, [onBody]);

    const bodyType = rootSpec.isKinematic ? (fixedRoot ? "fixed" : "kinematicPosition") : "dynamic";

    return (
        <>
            <RigidBody
                {...rigidBodyProps}
                ref={assignRootRef}
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
                    <KhrCollider key={`r${idx}`} c={col.spec} position={col.position} quaternion={col.quaternion} />
                ))}
            </RigidBody>
            {plan.parts.map((part) => (
                <ArticulatedPartBody
                    key={part.spec.nodeName}
                    part={part}
                    rootPos={position}
                    rootQuat={quaternion}
                    linearVelocity={rootSpec.linearVelocity}
                    angularVelocity={rootSpec.angularVelocity}
                    gravityScale={rootSpec.gravityFactor}
                    parentBody={part.parent ? (partBodies.get(part.parent.spec.nodeName) ?? null) : rootBody}
                    userData={rigidBodyProps?.userData}
                    origin={partOrigins?.get(part.spec.object)}
                    onBody={handlePartBody}
                    onJointReady={onJointReady}
                />
            ))}
        </>
    );
}

export interface KhrAssemblyHandle {
    readonly jointNames: readonly string[];
    getBody: (nodeName: string) => RapierRigidBody | null;
    getJointRuntime: (nodeName: string) => JointRuntime | null;
    setJointTarget: (nodeName: string, target: number) => void;
    setRoleTarget: (role: string, target: number) => void;
    toggleHitObject: (hit: THREE.Object3D | null) => boolean;
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
    enableCustomGravity?: boolean;
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
    enableCustomGravity = false,
    rigidBodyProps,
    children,
    ...groupProps
}: KhrAssemblyProps): React.JSX.Element {
    assertUnitScale(scale, "KhrAssembly");
    useMemo(() => assertSceneIdentityWorldScale(bundle.scene), [bundle]);

    const { world } = useRapier();
    const applyGravityField = useCustomGravity((s) => s.applyGravityField);
    const plans = useMemo(() => planAssembly(bundle), [bundle]);
    const isStaticMultiRootCompound = useMemo(
        () => fixedRoot && plans.length > 1 && plans.every((p) => p.spec.isKinematic && p.parts.length === 0),
        [fixedRoot, plans],
    );

    const compoundColliders = useMemo(() => {
        if (!isStaticMultiRootCompound)
            return null;
        const one = new THREE.Vector3(1, 1, 1);
        const pos = new THREE.Vector3(), quat = new THREE.Quaternion(), scl = new THREE.Vector3();
        const out: Array<{ key: string; spec: KhrColliderSpec; position: Vec3; quaternion: Quat }> = [];
        for (const p of plans) {
            const bMat = bodyPoseMatrix(p.spec);
            for (let idx = 0; idx < p.spec.colliders.length; idx++) {
                const c = p.spec.colliders[idx];
                const cMat = new THREE.Matrix4().compose(
                    new THREE.Vector3().fromArray(c.localPosition),
                    new THREE.Quaternion().fromArray(c.localQuaternion),
                    one,
                );
                bMat.clone().multiply(cMat).decompose(pos, quat, scl);
                quat.normalize();
                out.push({
                    key: `${p.spec.nodeName}:${idx}`,
                    spec: c,
                    position: [pos.x, pos.y, pos.z],
                    quaternion: [quat.x, quat.y, quat.z, quat.w],
                });
            }
        }
        return out;
    }, [isStaticMultiRootCompound, plans]);

    const origins = useMemo(
        () => (!isStaticMultiRootCompound && plans.length > 1 ? captureSubtreeOrigins(plans.map((p) => p.mountNode), bundle.scene) : null),
        [isStaticMultiRootCompound, plans, bundle.scene],
    );

    const [px, py, pz] = position;
    const [rx, ry, rz] = rotation ?? [0, 0, 0];
    const hasQuat = quaternion !== undefined;
    const qx = quaternion?.[0] ?? 0;
    const qy = quaternion?.[1] ?? 0;
    const qz = quaternion?.[2] ?? 0;
    const qw = quaternion?.[3] ?? 1;
    const spawnQuat = useMemo(
        () => (hasQuat ? new THREE.Quaternion(qx, qy, qz, qw).normalize() : new THREE.Quaternion().setFromEuler(new THREE.Euler(rx, ry, rz))),
        [hasQuat, qx, qy, qz, qw, rx, ry, rz],
    );

    const rootPoses = useMemo(
        () =>
            plans.map((p) => {
                const pos = new THREE.Vector3().fromArray(p.spec.localPosition).applyQuaternion(spawnQuat).add(new THREE.Vector3(px, py, pz));
                const quat = spawnQuat.clone().multiply(new THREE.Quaternion().fromArray(p.spec.localQuaternion));
                return {
                    position: [pos.x, pos.y, pos.z] as Vec3,
                    quaternion: [quat.x, quat.y, quat.z, quat.w] as Quat,
                };
            }),
        [plans, spawnQuat, px, py, pz],
    );

    const bodiesRef = useRef(new Map<string, RapierRigidBody>());
    const jointsRef = useRef(new Map<string, JointRuntime>());
    const byObjectRef = useRef(new Map<THREE.Object3D, JointRuntime>());

    const handleBody = useCallback((nodeName: string, rb: RapierRigidBody | null) => {
        if (rb)
            bodiesRef.current.set(nodeName, rb);
        else
            bodiesRef.current.delete(nodeName);
    }, []);

    const handleJointReady = useCallback((nodeName: string, obj: THREE.Object3D, rt: JointRuntime | null) => {
        if (rt) {
            jointsRef.current.set(nodeName, rt);
            byObjectRef.current.set(obj, rt);
        }
        else {
            jointsRef.current.delete(nodeName);
            byObjectRef.current.delete(obj);
        }
    }, []);

    useFrame(() => {
        if (!enableCustomGravity || world.timestep <= 0)
            return;
        for (const rb of bodiesRef.current.values()) {
            if (isLiveRapierBody(world, rb) && rb.isDynamic())
                applyGravityField(rb, world.timestep);
        }
    });

    useImperativeHandle(ref, () => ({
        get jointNames() {
            return [...jointsRef.current.keys()];
        },
        getBody: (nodeName: string) => bodiesRef.current.get(nodeName) ?? null,
        getJointRuntime: (nodeName: string) => jointsRef.current.get(nodeName) ?? null,
        setJointTarget: (nodeName: string, target: number) => {
            jointsRef.current.get(nodeName)?.applyTarget(target);
        },
        setRoleTarget: (role: string, target: number) => {
            for (const rt of jointsRef.current.values()) {
                if (rt.role === role)
                    rt.applyTarget(target);
            }
        },
        toggleHitObject: (hit: THREE.Object3D | null) => {
            for (let cur = hit; cur; cur = cur.parent) {
                const rt = byObjectRef.current.get(cur);
                if (rt && toggleDrivenJoint(rt))
                    return true;
            }
            return false;
        },
    }), []);

    const assignCompoundRef = useCallback((rb: RapierRigidBody | null) => {
        for (const p of plans) {
            if (rb)
                bodiesRef.current.set(p.spec.nodeName, rb);
            else
                bodiesRef.current.delete(p.spec.nodeName);
        }
    }, [plans]);

    useEffect(() => {
        if (!isStaticMultiRootCompound)
            return;
        return () => {
            for (const p of plans)
                bodiesRef.current.delete(p.spec.nodeName);
        };
    }, [isStaticMultiRootCompound, plans]);

    if (isStaticMultiRootCompound && compoundColliders) {
        return (
            <group {...groupProps}>
                <RigidBody
                    {...rigidBodyProps}
                    ref={assignCompoundRef}
                    type="fixed"
                    position={position}
                    quaternion={[spawnQuat.x, spawnQuat.y, spawnQuat.z, spawnQuat.w]}
                    colliders={false}
                >
                    <BundleSceneHost bundle={bundle} owner="KhrAssembly" />
                    {children}
                    {compoundColliders.map((col) => (
                        <KhrCollider key={col.key} c={col.spec} position={col.position} quaternion={col.quaternion} />
                    ))}
                </RigidBody>
            </group>
        );
    }

    if (plans.length === 1) {
        const plan = plans[0];
        return (
            <group {...groupProps}>
                <AssemblyRootBody
                    key={`${bundle.scene.uuid}:${plan.spec.nodeName}`}
                    plan={plan}
                    bundleScene={bundle.scene}
                    position={rootPoses[0].position}
                    quaternion={rootPoses[0].quaternion}
                    fixedRoot={fixedRoot}
                    rigidBodyProps={rigidBodyProps}
                    onBody={handleBody}
                    onJointReady={handleJointReady}
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
                    bundleScene={bundle.scene}
                    position={rootPoses[idx].position}
                    quaternion={rootPoses[idx].quaternion}
                    fixedRoot={fixedRoot}
                    rigidBodyProps={rigidBodyProps}
                    onBody={handleBody}
                    onJointReady={handleJointReady}
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
