import React, { useLayoutEffect, useRef } from "react";
import * as THREE from "three";
import { type KhrAssetBundle, type KhrCarConfig, type KhrCurveData, KhrValidationError } from "./types.js";

const mountedScenes = new WeakSet<THREE.Object3D>();
const clonedScenes = new WeakSet<THREE.Object3D>();
const disposedScenes = new WeakSet<THREE.Object3D>();
const templateByClone = new WeakMap<THREE.Object3D, THREE.Object3D>();
const subtreeRestorersByScene = new WeakMap<THREE.Object3D, Set<() => () => void>>();

function findSceneRoot(obj: THREE.Object3D): THREE.Object3D {
    let cur: THREE.Object3D = obj;
    while (cur.parent)
        cur = cur.parent;
    return cur;
}

export function registerMountedSubtree(ownerScene: THREE.Object3D, restoreForClone: () => () => void): () => void {
    let set = subtreeRestorersByScene.get(ownerScene);
    if (!set) {
        set = new Set();
        subtreeRestorersByScene.set(ownerScene, set);
    }
    set.add(restoreForClone);
    return () => {
        set?.delete(restoreForClone);
    };
}

export function claimBundleMount(bundle: KhrAssetBundle, ownerComponent: string): void {
    if (disposedScenes.has(bundle.scene)) {
        throw new KhrValidationError(`${ownerComponent}: KhrAssetBundle "${bundle.scene.name || bundle.scene.uuid}" has already been disposed`);
    }
    if (mountedScenes.has(bundle.scene)) {
        throw new KhrValidationError(
            `${ownerComponent}: KhrAssetBundle "${bundle.scene.name || bundle.scene.uuid}" is already mounted; call cloneKhrBundle(template) per concurrent instance`,
        );
    }
    mountedScenes.add(bundle.scene);
}

export function releaseBundleMount(bundle: KhrAssetBundle): void {
    mountedScenes.delete(bundle.scene);
}

function remapNode(srcRoot: THREE.Object3D, node: THREE.Object3D, dstRoot: THREE.Object3D): THREE.Object3D {
    if (node === srcRoot)
        return dstRoot;
    const path: number[] = [];
    let cur: THREE.Object3D | null = node;
    while (cur && cur !== srcRoot) {
        const parent: THREE.Object3D | null = cur.parent;
        if (!parent)
            throw new KhrValidationError(`cloneKhrBundle: body node "${node.name}" is detached from bundle.scene`);
        const idx = parent.children.indexOf(cur);
        if (idx < 0)
            throw new KhrValidationError(`cloneKhrBundle: child index missing for "${node.name}"`);
        path.push(idx);
        cur = parent;
    }
    let out = dstRoot;
    for (let i = path.length - 1; i >= 0; i--)
        out = out.children[path[i]];
    return out;
}

function cloneCurve(curve?: KhrCurveData): KhrCurveData | undefined {
    if (!curve)
        return undefined;
    return {
        ...curve,
        points: curve.points.map((pt) => ({ ...pt })),
    };
}

function cloneCarTune(tune?: Partial<KhrCarConfig>): Partial<KhrCarConfig> | undefined {
    if (!tune)
        return undefined;
    return {
        ...tune,
        ...(tune.gearRatios ? { gearRatios: [...tune.gearRatios] } : {}),
        ...(tune.engineTorqueCurveData ? { engineTorqueCurveData: cloneCurve(tune.engineTorqueCurveData) } : {}),
        ...(tune.steerAngleCurveData ? { steerAngleCurveData: cloneCurve(tune.steerAngleCurveData) } : {}),
    };
}

/**
 * Clones a parsed `KhrAssetBundle` in O(nodes) while sharing GPU `BufferGeometry` / `Material` buffers
 * and Rapier `Float32Array` collider vertices. Only restores subtrees belonging to `bundle.scene`.
 */
export function cloneKhrBundle(bundle: KhrAssetBundle): KhrAssetBundle {
    if (disposedScenes.has(bundle.scene)) {
        throw new KhrValidationError(`cloneKhrBundle: cannot clone disposed KhrAssetBundle "${bundle.scene.name || bundle.scene.uuid}"`);
    }
    const scopedSet = subtreeRestorersByScene.get(bundle.scene);
    const restorers = scopedSet ? [...scopedSet] : [];
    const reattach: Array<() => void> = [];
    for (let i = restorers.length - 1; i >= 0; i--)
        reattach.push(restorers[i]());

    try {
        const scene = bundle.scene.clone(true);
        clonedScenes.add(scene);
        templateByClone.set(scene, templateByClone.get(bundle.scene) ?? bundle.scene);
        const sockets: Record<string, [number, number, number]> = Object.create(null);
        for (const [k, v] of Object.entries(bundle.sockets))
            sockets[k] = [v[0], v[1], v[2]];

        return {
            scene,
            bodies: bundle.bodies.map((b) => {
                const drives = b.joint ? b.joint.drives.map((d) => ({ ...d })) : [];
                return {
                    ...b,
                    centerOfMass: [...b.centerOfMass],
                    inertiaDiagonal: [...b.inertiaDiagonal],
                    inertiaOrientation: [...b.inertiaOrientation],
                    ...(b.linearVelocity ? { linearVelocity: [...b.linearVelocity] } : {}),
                    ...(b.angularVelocity ? { angularVelocity: [...b.angularVelocity] } : {}),
                    localPosition: [...b.localPosition],
                    localQuaternion: [...b.localQuaternion],
                    colliders: b.colliders.map((c) => ({
                        ...c,
                        shape: c.shape.type === "box" ? { ...c.shape, size: [...c.shape.size] } : { ...c.shape },
                        localPosition: [...c.localPosition],
                        localQuaternion: [...c.localQuaternion],
                    })),
                    joint: b.joint
                        ? {
                            ...b.joint,
                            axis: [...b.joint.axis],
                            limits: b.joint.limits ? [...b.joint.limits] : null,
                            drive: drives[0] ?? (b.joint.drive ? { ...b.joint.drive } : null),
                            drives,
                            anchor1: [...b.joint.anchor1],
                            anchor2: [...b.joint.anchor2],
                        }
                        : null,
                    ...(b.breakaway ? { breakaway: { ...b.breakaway } } : {}),
                    object: remapNode(bundle.scene, b.object, scene),
                };
            }),
            wheels: bundle.wheels.map((w) => ({
                ...w,
                object: remapNode(bundle.scene, w.object, scene),
                position: [...w.position],
                ...(w.quaternion ? { quaternion: [...w.quaternion] } : {}),
                ...(w.lngSlipRatioCurveData ? { lngSlipRatioCurveData: cloneCurve(w.lngSlipRatioCurveData) } : {}),
                ...(w.latSlipRatioCurveData ? { latSlipRatioCurveData: cloneCurve(w.latSlipRatioCurveData) } : {}),
            })),
            propellers: bundle.propellers.map((p) => ({
                ...p,
                object: remapNode(bundle.scene, p.object, scene),
                position: [...p.position],
                ...(p.quaternion ? { quaternion: [...p.quaternion] } : {}),
            })),
            sockets,
            totalDynamicMass: bundle.totalDynamicMass,
            centerOfMass: [...bundle.centerOfMass],
            ...(bundle.tune ? { tune: cloneCarTune(bundle.tune) } : {}),
            ...(bundle.droneTune ? { droneTune: { ...bundle.droneTune } } : {}),
        };
    }
    finally {
        for (let i = reattach.length - 1; i >= 0; i--)
            reattach[i]();
    }
}

export function disposeSceneGpuResources(roots: readonly THREE.Object3D[]): void {
    const geos = new Set<THREE.BufferGeometry>();
    const mats = new Set<THREE.Material>();
    const texs = new Set<THREE.Texture>();
    for (const root of roots) {
        root.traverse((child) => {
            const mesh = child as THREE.Mesh;
            if (mesh.geometry && !geos.has(mesh.geometry)) {
                geos.add(mesh.geometry);
                mesh.geometry.dispose();
            }
            const matList = Array.isArray(mesh.material) ? mesh.material : mesh.material ? [mesh.material] : [];
            for (const mat of matList) {
                if (!mats.has(mat)) {
                    mats.add(mat);
                    for (const val of Object.values(mat)) {
                        if (val && typeof val === "object" && "isTexture" in val && (val as THREE.Texture).isTexture) {
                            const tex = val as THREE.Texture;
                            if (!texs.has(tex)) {
                                texs.add(tex);
                                tex.dispose();
                            }
                        }
                    }
                    mat.dispose();
                }
            }
        });
    }
}

/**
 * Releases mount ownership. On root templates (or when `{ disposeTemplate: true }` is passed on a clone),
 * disposes unique GPU geometries, materials, and textures once.
 */
export function disposeKhrBundle(bundle: KhrAssetBundle, options?: { disposeTemplate?: boolean }): void {
    mountedScenes.delete(bundle.scene);
    if (disposedScenes.has(bundle.scene))
        return;
    disposedScenes.add(bundle.scene);

    const isClone = clonedScenes.has(bundle.scene);
    if (isClone && !options?.disposeTemplate)
        return;

    const targetRoot = isClone ? (templateByClone.get(bundle.scene) ?? bundle.scene) : bundle.scene;
    disposedScenes.add(targetRoot);
    disposeSceneGpuResources([
        targetRoot,
        ...bundle.wheels.map((w) => w.object),
        ...bundle.propellers.map((p) => p.object),
    ]);
}

export interface SubtreeOrigin {
    ownerScene: THREE.Object3D;
    origParent: THREE.Object3D | null;
    origIndex: number;
    origOrder: ReadonlyMap<THREE.Object3D, number> | null;
    origPos: THREE.Vector3;
    origQuat: THREE.Quaternion;
    origScale: THREE.Vector3;
}

export function captureSubtreeOrigins(objects: readonly THREE.Object3D[], explicitScene?: THREE.Object3D): Map<THREE.Object3D, SubtreeOrigin> {
    const parentOrders = new Map<THREE.Object3D, Map<THREE.Object3D, number>>();
    const origins = new Map<THREE.Object3D, SubtreeOrigin>();
    for (const obj of objects) {
        const ownerScene = explicitScene ?? findSceneRoot(obj);
        const origParent = obj.parent;
        let origOrder: ReadonlyMap<THREE.Object3D, number> | null = null;
        let origIndex = -1;
        if (origParent) {
            let order = parentOrders.get(origParent);
            if (!order) {
                order = new Map<THREE.Object3D, number>();
                origParent.children.forEach((child, idx) => order!.set(child, idx));
                parentOrders.set(origParent, order);
            }
            origOrder = order;
            origIndex = order.get(obj) ?? origParent.children.indexOf(obj);
        }
        origins.set(obj, {
            ownerScene,
            origParent,
            origIndex,
            origOrder,
            origPos: obj.position.clone(),
            origQuat: obj.quaternion.clone(),
            origScale: obj.scale.clone(),
        });
    }
    return origins;
}

export function restoreSubtreeOrigin(object: THREE.Object3D, snap: SubtreeOrigin): void {
    object.position.copy(snap.origPos);
    object.quaternion.copy(snap.origQuat);
    object.scale.copy(snap.origScale);
    const { origParent, origIndex, origOrder } = snap;
    if (!origParent)
        return;
    if (object.parent && object.parent !== origParent)
        object.parent.remove(object);
    const existingIdx = origParent.children.indexOf(object);
    if (existingIdx >= 0)
        origParent.children.splice(existingIdx, 1);

    let insertAt = origParent.children.length;
    if (origOrder && origOrder.size > 0) {
        for (let i = 0; i < origParent.children.length; i++) {
            if ((origOrder.get(origParent.children[i]) ?? Infinity) > origIndex) {
                insertAt = i;
                break;
            }
        }
    }
    else if (origIndex >= 0) {
        insertAt = Math.min(origIndex, origParent.children.length);
    }
    object.parent = origParent;
    origParent.children.splice(insertAt, 0, object);
}

export function BundleSceneHost({ bundle, owner }: { bundle: KhrAssetBundle; owner: string }): React.JSX.Element {
    const hostRef = useRef<THREE.Group | null>(null);
    useLayoutEffect(() => {
        const host = hostRef.current;
        if (!host)
            return;
        claimBundleMount(bundle, owner);
        host.add(bundle.scene);
        return () => {
            if (bundle.scene.parent === host)
                host.remove(bundle.scene);
            releaseBundleMount(bundle);
        };
    }, [bundle, owner]);
    return React.createElement("group", { ref: hostRef });
}

export function MountSubtreeSlot({
    object,
    position,
    quaternion,
    origin,
}: {
    object: THREE.Object3D;
    position: [number, number, number];
    quaternion: [number, number, number, number];
    origin?: SubtreeOrigin;
}): React.JSX.Element {
    const slotRef = useRef<THREE.Group | null>(null);
    useLayoutEffect(() => {
        const slot = slotRef.current;
        if (!slot)
            return;
        const snap = origin ?? captureSubtreeOrigins([object]).get(object)!;
        if (snap.origParent)
            snap.origParent.remove(object);
        object.position.set(0, 0, 0);
        object.quaternion.set(0, 0, 0, 1);
        object.scale.set(1, 1, 1);
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
    }, [object, origin]);
    return React.createElement("group", { ref: slotRef, position, quaternion });
}
