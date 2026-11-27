import type * as THREE from "three";
import { setSocket } from "./khrExtras.js";
import type { RawGltfJson } from "./khrGlb.js";
import { computeBodyInertia, computeRelativePose, readFiniteVec3 } from "./khrInertia.js";
import { decodePhysicsMaterial } from "./khrParsers.js";
import { isPlainObject, type RawColliderDef, resolveColliderShape } from "./khrPluginHelpers.js";
import {
    type KhrBodySpec,
    type KhrCarConfig,
    type KhrDroneConfig,
    type KhrImplicitShape,
    KhrValidationError,
} from "./khrTypes.js";

type RawNode = NonNullable<RawGltfJson["nodes"]>[number];

export function findOwnerBodyIdx(idx: number, parentIdx: Int32Array, rawNodes: readonly RawNode[]): number {
    for (let p = parentIdx[idx]; p !== -1; p = parentIdx[p]) {
        if (rawNodes[p].extensions?.KHR_physics_rigid_bodies?.motion)
            return p;
    }
    return -1;
}

export function resolveConnectedBodyIdx(cIdx: number, parentIdx: Int32Array, rawNodes: readonly RawNode[]): number {
    const phys = rawNodes[cIdx]?.extensions?.KHR_physics_rigid_bodies;
    const hasOwnBody = isPlainObject(phys) && (
        phys.motion !== undefined ||
        ((phys.collider !== undefined || phys.joint !== undefined) && findOwnerBodyIdx(cIdx, parentIdx, rawNodes) === -1)
    );
    if (hasOwnBody)
        return cIdx;
    const owner = findOwnerBodyIdx(cIdx, parentIdx, rawNodes);
    return owner !== -1 ? owner : cIdx;
}

export function isInsideRotor(idx: number, parentIdx: Int32Array, rawNodes: readonly RawNode[]): boolean {
    for (let p = idx; p !== -1; p = parentIdx[p]) {
        const node = rawNodes[p];
        const ex = node.extras;
        if (isPlainObject(ex) && (ex.ecctrlWheel !== undefined || ex.ecctrlPropeller !== undefined))
            return true;
        if (isPlainObject(node.extensions) && node.extensions.OMI_vehicle_wheel !== undefined)
            return true;
    }
    return false;
}

export function attachBodyColliders(
    rawNodes: readonly RawNode[],
    parentIdx: Int32Array,
    nodeNames: readonly string[],
    resolveObj: (idx: number) => THREE.Object3D | undefined,
    bodyByIdx: ReadonlyMap<number, KhrBodySpec>,
    shapes: readonly KhrImplicitShape[],
    materials: Array<Record<string, unknown>>,
    collisionGroups: readonly number[],
): void {
    for (let i = 0; i < rawNodes.length; i++) {
        if (isInsideRotor(i, parentIdx, rawNodes))
            continue;
        const rawCol = rawNodes[i].extensions?.KHR_physics_rigid_bodies?.collider;
        if (rawCol === undefined)
            continue;
        const obj = resolveObj(i);
        if (!obj)
            continue;
        const ownerIdx = bodyByIdx.has(i) ? i : findOwnerBodyIdx(i, parentIdx, rawNodes);
        const owner = bodyByIdx.get(ownerIdx);
        if (!owner)
            continue;
        const shape = resolveColliderShape(rawCol, nodeNames[i], shapes, nodeNames, resolveObj);
        const colDef = rawCol as RawColliderDef;
        const rel = ownerIdx === i
            ? { localPosition: [0, 0, 0] as [number, number, number], localQuaternion: [0, 0, 0, 1] as [number, number, number, number] }
            : computeRelativePose(owner.object, obj);
        const mat = decodePhysicsMaterial(materials, colDef.physicsMaterial, nodeNames[i]);
        let colGroups: number | undefined;
        if (colDef.collisionFilter !== undefined) {
            if (typeof colDef.collisionFilter !== "number" || !Number.isInteger(colDef.collisionFilter) ||
                colDef.collisionFilter < 0 || colDef.collisionFilter >= collisionGroups.length) {
                throw new KhrValidationError(`Node "${nodeNames[i]}" references out-of-bounds collisionFilter ${String(colDef.collisionFilter)}`);
            }
            colGroups = collisionGroups[colDef.collisionFilter];
        }
        owner.colliders.push({
            shape,
            localPosition: rel.localPosition,
            localQuaternion: rel.localQuaternion,
            ...mat,
            ...(colGroups === undefined ? {} : { collisionGroups: colGroups }),
        });
    }
}

export function computeMissingBodyInertias(
    bodyByIdx: ReadonlyMap<number, KhrBodySpec>,
    explicitInertiaSet: ReadonlySet<number>,
): void {
    for (const [idx, body] of bodyByIdx) {
        if (!explicitInertiaSet.has(idx) && !body.isKinematic) {
            body.inertiaDiagonal = computeBodyInertia(body.colliders, body.mass, body.centerOfMass, body.inertiaOrientation, body.nodeName);
        }
    }
}

export function extractSceneExtras(
    json: RawGltfJson,
    sockets: Record<string, [number, number, number]>,
): {
    ikSockets: Array<[string, [number, number, number]]>;
    tune?: Partial<KhrCarConfig>;
    droneTune?: Partial<KhrDroneConfig>;
} {
    const ikSockets: Array<[string, [number, number, number]]> = [];
    const sceneExtras = json.scenes?.[json.scene ?? 0]?.extras ?? json.extras;
    if (sceneExtras?.ikSockets !== undefined) {
        if (!isPlainObject(sceneExtras.ikSockets)) {
            throw new KhrValidationError("scene.extras.ikSockets must be an object");
        }
        for (const [k, v] of Object.entries(sceneExtras.ikSockets)) {
            const pos = readFiniteVec3(v, `scene.extras.ikSockets.${k}`);
            setSocket(sockets, k, pos);
            ikSockets.push([k, pos]);
        }
    }
    const tune = isPlainObject(sceneExtras?.tune) ? (sceneExtras.tune as Partial<KhrCarConfig>) : undefined;
    const droneTune = isPlainObject(sceneExtras?.droneTune) ? (sceneExtras.droneTune as Partial<KhrDroneConfig>) : undefined;
    return { ikSockets, tune, droneTune };
}
