import * as THREE from "three";
import { bodyPoseMatrix, computeLumpedMassProps, type LumpedMassProps } from "./khrMassProps.js";
import { type KhrAssetBundle, type KhrBodySpec, type KhrColliderSpec, type KhrJointSpec, KhrValidationError } from "./khrTypes.js";

type Vec3 = [number, number, number];
type Quat = [number, number, number, number];

export interface ColliderMass {
    mass: number;
    centerOfMass: { x: number; y: number; z: number };
    principalAngularInertia: { x: number; y: number; z: number };
    angularInertiaLocalFrame: { x: number; y: number; z: number; w: number };
}

export interface PlannedCollider {
    spec: KhrColliderSpec;
    local: THREE.Matrix4;
    position: Vec3;
    quaternion: Quat;
    massProperties?: ColliderMass;
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
    inertia: number;
    partMass?: ColliderMass;
    colliders: PlannedCollider[];
}

export interface RootPlan {
    spec: KhrBodySpec;
    sceneInRootPos: Vec3;
    sceneInRootQuat: Quat;
    mountNode: THREE.Object3D;
    mountInRoot: THREE.Matrix4;
    mountPosition: Vec3;
    mountQuaternion: Quat;
    rootMass?: ColliderMass;
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

function lowestCommonAncestor(nodes: readonly THREE.Object3D[], sceneRoot: THREE.Object3D): THREE.Object3D {
    const chain = (n: THREE.Object3D): THREE.Object3D[] => {
        const list: THREE.Object3D[] = [];
        for (let cur: THREE.Object3D | null = n; cur; cur = cur.parent) {
            list.push(cur);
            if (cur === sceneRoot) break;
        }
        return list;
    };
    let shared = new Set(chain(nodes[0]));
    for (let i = 1; i < nodes.length; i++) {
        const setI = new Set(chain(nodes[i]));
        shared = new Set([...shared].filter((x) => setI.has(x)));
    }
    return chain(nodes[0]).find((c) => shared.has(c)) ?? sceneRoot;
}

export function toColliderMass(lumped: LumpedMassProps, colMat?: THREE.Matrix4, colQuat?: THREE.Quaternion): ColliderMass {
    const com = new THREE.Vector3().fromArray(lumped.com);
    const frame = new THREE.Quaternion().fromArray(lumped.inertiaFrame);
    if (colMat && colQuat) {
        com.applyMatrix4(colMat.clone().invert());
        frame.premultiply(colQuat.clone().invert()).normalize();
    }
    return {
        mass: lumped.mass,
        centerOfMass: { x: com.x, y: com.y, z: com.z },
        principalAngularInertia: { x: lumped.inertia[0], y: lumped.inertia[1], z: lumped.inertia[2] },
        angularInertiaLocalFrame: { x: frame.x, y: frame.y, z: frame.z, w: frame.w },
    };
}

function planOwnColliders(body: KhrBodySpec, lumped: LumpedMassProps | null, frameQuat?: THREE.Quaternion): PlannedCollider[] {
    const one = new THREE.Vector3(1, 1, 1);
    const frameMat = frameQuat ? new THREE.Matrix4().makeRotationFromQuaternion(frameQuat) : null;
    const pos = new THREE.Vector3(), quat = new THREE.Quaternion(), scl = new THREE.Vector3();
    return body.colliders.map((spec, idx) => {
        const colQuat = new THREE.Quaternion().fromArray(spec.localQuaternion);
        const local = new THREE.Matrix4().compose(new THREE.Vector3().fromArray(spec.localPosition), colQuat, one);
        if (frameMat)
            local.premultiply(frameMat);
        local.decompose(pos, quat, scl);
        quat.normalize();
        const planned: PlannedCollider = {
            spec,
            local,
            position: [pos.x, pos.y, pos.z],
            quaternion: [quat.x, quat.y, quat.z, quat.w],
        };
        if (idx === 0 && lumped)
            planned.massProperties = toColliderMass(lumped, local, quat);
        return planned;
    });
}

function isSceneDescendant(node: THREE.Object3D, ancestor: THREE.Object3D): boolean {
    for (let cur: THREE.Object3D | null = node; cur; cur = cur.parent) {
        if (cur === ancestor)
            return true;
    }
    return false;
}

function approxPartInertia(b: KhrBodySpec, axisChild: Vec3): number {
    if (b.joint?.kind === "fixed")
        return 0;
    if (b.joint?.kind === "prismatic")
        return b.mass;
    const ax = new THREE.Vector3().fromArray(axisChild).normalize();
    const qInv = new THREE.Quaternion().fromArray(b.inertiaOrientation).invert();
    const axLocal = ax.clone().applyQuaternion(qInv);
    const [d0, d1, d2] = b.inertiaDiagonal;
    const com = new THREE.Vector3().fromArray(b.centerOfMass);
    const along = com.dot(ax);
    return d0 * axLocal.x * axLocal.x + d1 * axLocal.y * axLocal.y + d2 * axLocal.z * axLocal.z + b.mass * (com.lengthSq() - along * along);
}

/** Plans independent Rapier bodies for roots and jointed dynamic children connected by Rapier ImpulseJoints. */
export function planAssembly(bundle: KhrAssetBundle): RootPlan[] {
    const byName = new Map(bundle.bodies.map((b) => [b.nodeName, b]));
    const findRoot = (b: KhrBodySpec, seen = new Set<string>()): KhrBodySpec => {
        if (!b.joint)
            return b;
        if (seen.has(b.nodeName))
            throw new KhrValidationError(`Joint cycle through "${b.nodeName}"`);
        seen.add(b.nodeName);
        const parent = byName.get(b.joint.connectedNodeName);
        if (!parent) {
            throw new KhrValidationError(`Joint on "${b.nodeName}" targets "${b.joint.connectedNodeName}", which is not a KHR body`);
        }
        return findRoot(parent, seen);
    };

    const plans = bundle.bodies
        .filter((b) => !b.joint)
        .map((root): RootPlan => {
            const rootMat = bodyPoseMatrix(root);
            const invRoot = rootMat.clone().invert();
            const descendants = bundle.bodies.filter((b) => b.joint && findRoot(b) === root);
            const depthOf = (b: KhrBodySpec): number => b.joint ? 1 + depthOf(byName.get(b.joint.connectedNodeName)!) : 0;
            descendants.sort((a, b) => depthOf(a) - depthOf(b));

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
                const rpPos = new THREE.Vector3(), rpQuat = new THREE.Quaternion(), rpScl = new THREE.Vector3();
                relParentMat.decompose(rpPos, rpQuat, rpScl);
                rpQuat.normalize();
                const rrPos = new THREE.Vector3(), rrQuat = new THREE.Quaternion();
                relRootMat.decompose(rrPos, rrQuat, rpScl);
                rrQuat.normalize();

                const axChildVec = new THREE.Vector3().fromArray(joint.axis).normalize();
                const axParentVec = axChildVec.clone().applyQuaternion(rrQuat).normalize();
                const axisChild: Vec3 = [axChildVec.x, axChildVec.y, axChildVec.z];
                const axisParent: Vec3 = [axParentVec.x, axParentVec.y, axParentVec.z];
                const anchorParent: Vec3 = parent
                    ? [rrPos.x - parent.restRelRootPos[0], rrPos.y - parent.restRelRootPos[1], rrPos.z - parent.restRelRootPos[2]]
                    : [rpPos.x, rpPos.y, rpPos.z];

                const bodyAlignInv = new THREE.Matrix4().compose(new THREE.Vector3().fromArray(b.localPosition), new THREE.Quaternion().fromArray(root.localQuaternion), new THREE.Vector3(1, 1, 1)).invert();
                const ownLumped = b.isKinematic ? null : computeLumpedMassProps([b], bodyAlignInv);
                const colliders = planOwnColliders(b, ownLumped, rrQuat);
                const partMass = ownLumped && colliders.length === 0 ? toColliderMass(ownLumped) : undefined;

                const plan: PartPlan = {
                    spec: b,
                    joint,
                    parent,
                    restRelParentPos: anchorParent,
                    restRelParentQuat: [rrQuat.x, rrQuat.y, rrQuat.z, rrQuat.w],
                    restRelRootPos: [rrPos.x, rrPos.y, rrPos.z],
                    restRelRootQuat: [rrQuat.x, rrQuat.y, rrQuat.z, rrQuat.w],
                    axisParent,
                    axisChild,
                    inertia: approxPartInertia(b, axisChild),
                    ...(partMass ? { partMass } : {}),
                    colliders,
                };
                partByName.set(b.nodeName, plan);
                return plan;
            });

            const rootLumped = root.isKinematic ? null : computeLumpedMassProps([root], invRoot);
            const colliders = planOwnColliders(root, rootLumped);
            const rootMass = rootLumped && colliders.length === 0 ? toColliderMass(rootLumped) : undefined;
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
                mountInRoot,
                mountPosition: [mountPos.x, mountPos.y, mountPos.z],
                mountQuaternion: [mountQuat.x, mountQuat.y, mountQuat.z, mountQuat.w],
                ...(rootMass ? { rootMass } : {}),
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
