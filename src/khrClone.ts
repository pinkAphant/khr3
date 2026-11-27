import * as THREE from "three";
import { type KhrAssetBundle, type KhrCarConfig, type KhrCurveData, KhrValidationError } from "./khrTypes.js";

const mountedScenes = new WeakSet<THREE.Object3D>();
const clonedScenes = new WeakSet<THREE.Object3D>();
const disposedScenes = new WeakSet<THREE.Object3D>();
const activeSubtreeRestorers: Array<() => () => void> = [];

export function registerMountedSubtree(restoreForClone: () => () => void): () => void {
    activeSubtreeRestorers.push(restoreForClone);
    return () => {
        const idx = activeSubtreeRestorers.indexOf(restoreForClone);
        if (idx >= 0)
            activeSubtreeRestorers.splice(idx, 1);
    };
}

export function claimBundleMount(bundle: KhrAssetBundle, ownerComponent: string): void {
    if (disposedScenes.has(bundle.scene)) {
        throw new KhrValidationError(`${ownerComponent}: KhrAssetBundle "${bundle.scene.name || bundle.scene.uuid}" has already been disposed`);
    }
    if (mountedScenes.has(bundle.scene)) {
        throw new KhrValidationError(`${ownerComponent}: KhrAssetBundle "${bundle.scene.name || bundle.scene.uuid}" is already mounted; call cloneKhrBundle(template) per concurrent instance`);
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
 * Clones a parsed `KhrAssetBundle` in O(nodes) while sharing parsed `.glb` buffers,
 * GPU `BufferGeometry` / `Material` buffers, and Rapier `Float32Array` collider vertices.
 */
export function cloneKhrBundle(bundle: KhrAssetBundle): KhrAssetBundle {
    if (disposedScenes.has(bundle.scene)) {
        throw new KhrValidationError(`cloneKhrBundle: cannot clone disposed KhrAssetBundle "${bundle.scene.name || bundle.scene.uuid}"`);
    }
    const reattach: Array<() => void> = [];
    for (let i = activeSubtreeRestorers.length - 1; i >= 0; i--) {
        reattach.push(activeSubtreeRestorers[i]());
    }
    try {
        const scene = bundle.scene.clone(true);
        clonedScenes.add(scene);
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
            sockets: Object.fromEntries(Object.entries(bundle.sockets).map(([k, v]) => [k, [...v]])),
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

/**
 * Releases clone mount ownership while keeping shared template GPU resources intact.
 * When called on a root template bundle, disposes unique GPU geometries, materials, and textures once.
 */
export function disposeKhrBundle(bundle: KhrAssetBundle): void {
    mountedScenes.delete(bundle.scene);
    if (clonedScenes.has(bundle.scene) || disposedScenes.has(bundle.scene)) {
        disposedScenes.add(bundle.scene);
        return;
    }
    disposedScenes.add(bundle.scene);
    const geos = new Set<THREE.BufferGeometry>();
    const mats = new Set<THREE.Material>();
    const texs = new Set<THREE.Texture>();
    const disposeTree = (obj: THREE.Object3D): void => {
        obj.traverse((child) => {
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
    };
    disposeTree(bundle.scene);
    for (const w of bundle.wheels)
        disposeTree(w.object);
    for (const p of bundle.propellers)
        disposeTree(p.object);
}
