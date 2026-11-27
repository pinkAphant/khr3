import * as THREE from "three";
import type { GLTF, GLTFLoaderPlugin, GLTFParser } from "three/examples/jsm/loaders/GLTFLoader.js";
import type { RawGltfJson } from "./khrGlb.js";
import { extractMeshColliderShape } from "./khrInertia.js";
import { type KhrColliderShape, type KhrImplicitShape, KhrValidationError } from "./khrTypes.js";

export const isPlainObject = (v: unknown): v is Record<string, unknown> => typeof v === "object" && !!v && !Array.isArray(v);

export function reqArr<T>(val: unknown, label: string): T[] {
    if (val === undefined)
        return [];
    if (!Array.isArray(val))
        throw new KhrValidationError(`${label} must be an array`);
    return val as T[];
}

export interface RawColliderDef {
    geometry?: {
        shape?: number;
        node?: number;
        convexHull?: boolean;
    };
    physicsMaterial?: number;
    collisionFilter?: number;
}

export const HEADLESS_TEXTURE_EXT_NAMES = [
    "EXT_texture_webp",
    "EXT_texture_avif",
    "KHR_texture_basisu",
] as const;

export function isHeadlessTextureEnv(): boolean {
    return typeof document === "undefined" && (typeof createImageBitmap === "undefined" || typeof self === "undefined");
}

export function stubTextureInParser(parser: GLTFParser, textureIndex: number): Promise<THREE.Texture> {
    const tex = new THREE.Texture();
    tex.flipY = false;
    parser.associations.set(tex, { textures: textureIndex } as never);
    return Promise.resolve(tex);
}

export function patchHeadlessTexturePlugins(parser: GLTFParser): void {
    const plugins = parser.plugins;
    if (!plugins)
        return;
    for (const extName of HEADLESS_TEXTURE_EXT_NAMES) {
        const existing = plugins[extName];
        if (!existing)
            continue;
        existing.loadTexture = (textureIndex: number): Promise<THREE.Texture> | null => {
            const textureDef = (parser.json as { textures?: Array<{ extensions?: Record<string, unknown> }> } | undefined)?.textures?.[textureIndex];
            if (!textureDef?.extensions?.[extName])
                return null;
            return stubTextureInParser(parser, textureIndex);
        };
    }
}

export function createHeadlessTexturePlugin(): (parser: GLTFParser) => GLTFLoaderPlugin {
    return (parser: GLTFParser): GLTFLoaderPlugin => ({
        name: "khr3_headless_texture",
        beforeRoot(): Promise<void> | null {
            patchHeadlessTexturePlugins(parser);
            return null;
        },
        loadTexture: (textureIndex: number): Promise<THREE.Texture> => stubTextureInParser(parser, textureIndex),
    });
}

export function buildNodeGraphLookup(
    gltf: GLTF,
    rawNodes: NonNullable<RawGltfJson["nodes"]>,
    rawGltf: RawGltfJson | undefined = gltf.parser?.json as RawGltfJson | undefined,
): {
    parentIdx: Int32Array;
    nodeNames: string[];
    resolveObj: (idx: number) => THREE.Object3D | undefined;
} {
    const parentIdx = new Int32Array(rawNodes.length).fill(-1);
    for (let i = 0; i < rawNodes.length; i++) {
        const node = rawNodes[i];
        if (!isPlainObject(node))
            throw new KhrValidationError(`GLTF node[${i}] must be an object`);
        const label = typeof node.name === "string" && node.name.length > 0 ? node.name : String(i);
        if (node.children !== undefined) {
            if (!Array.isArray(node.children))
                throw new KhrValidationError(`GLTF node "${label}" children must be an array`);
            for (const c of node.children) {
                if (typeof c !== "number" || !Number.isInteger(c) || c < 0 || c >= rawNodes.length) {
                    throw new KhrValidationError(`GLTF node "${label}" has out-of-bounds child index ${String(c)}`);
                }
            }
        }
    }

    let sceneRoots: number[] | undefined;
    const inActiveScene = new Set<number>();
    if (Array.isArray(rawGltf?.scenes)) {
        const sceneIdx = rawGltf.scene ?? 0;
        const sceneDef = rawGltf.scenes[sceneIdx];
        if (!isPlainObject(sceneDef))
            throw new KhrValidationError(`GLTF scene[${sceneIdx}] must be an object`);
        if (sceneDef.nodes !== undefined && !Array.isArray(sceneDef.nodes)) {
            throw new KhrValidationError(`GLTF scene[${sceneIdx}].nodes must be an array`);
        }
        sceneRoots = (sceneDef.nodes ?? []).map((n, pos) => {
            if (typeof n !== "number" || !Number.isInteger(n) || n < 0 || n >= rawNodes.length) {
                throw new KhrValidationError(`GLTF scene[${sceneIdx}].nodes[${pos}] has invalid node index ${String(n)}`);
            }
            return n;
        });
        const stack = [...sceneRoots];
        while (stack.length > 0) {
            const cur = stack.pop()!;
            if (inActiveScene.has(cur))
                continue;
            inActiveScene.add(cur);
            const children = rawNodes[cur].children;
            if (children)
                for (const c of children) stack.push(c);
        }
    }
    else {
        for (let i = 0; i < rawNodes.length; i++)
            inActiveScene.add(i);
    }

    const rootSet = sceneRoots ? new Set(sceneRoots) : undefined;
    for (const i of inActiveScene) {
        const children = rawNodes[i].children;
        if (children) {
            for (const c of children) {
                if (parentIdx[c] !== -1 || rootSet?.has(c)) {
                    const label = typeof rawNodes[c].name === "string" && rawNodes[c].name!.length > 0 ? rawNodes[c].name! : String(c);
                    throw new KhrValidationError(`GLTF node "${label}" has multiple parents in active scene`);
                }
                for (let anc = i; anc !== -1; anc = parentIdx[anc]) {
                    if (anc === c) {
                        const label = typeof rawNodes[c].name === "string" && rawNodes[c].name!.length > 0 ? rawNodes[c].name! : String(c);
                        throw new KhrValidationError(`GLTF node "${label}" has cyclic parent in active scene`);
                    }
                }
                parentIdx[c] = i;
            }
        }
    }

    const usedNames = new Set<string>();
    const nodeNames = new Array<string>(rawNodes.length);
    const assignName = (idx: number, dedupe: boolean): void => {
        const rawName = rawNodes[idx].name;
        let name = typeof rawName === "string" && rawName.length > 0 ? rawName : `node_${idx}`;
        if (dedupe) {
            if (usedNames.has(name))
                name = `${name}_${idx}`;
            while (usedNames.has(name))
                name = `${name}_`;
            usedNames.add(name);
        }
        nodeNames[idx] = name;
    };
    for (let i = 0; i < rawNodes.length; i++)
        if (inActiveScene.has(i)) assignName(i, true);
    for (let i = 0; i < rawNodes.length; i++)
        if (!inActiveScene.has(i)) assignName(i, false);

    const byNodeIdx = new Map<number, THREE.Object3D>();
    const byName = new Map<string, THREE.Object3D>();
    gltf.scene.traverse((obj) => {
        const assoc = gltf.parser?.associations?.get(obj) as { nodes?: number } | undefined;
        const nIdx = assoc?.nodes;
        if (typeof nIdx === "number" && inActiveScene.has(nIdx) && !byNodeIdx.has(nIdx))
            byNodeIdx.set(nIdx, obj);
        if (obj.name && !byName.has(obj.name))
            byName.set(obj.name, obj);
    });

    if (sceneRoots && gltf.scene.children.length === sceneRoots.length) {
        const seen = new Set<number>();
        const matchTree = (idx: number, obj: THREE.Object3D): void => {
            if (seen.has(idx))
                return;
            seen.add(idx);
            if (!byNodeIdx.has(idx))
                byNodeIdx.set(idx, obj);
            const children = rawNodes[idx].children;
            if (!children || children.length === 0)
                return;
            const offset = obj.children.length - children.length;
            if (offset < 0)
                return;
            for (let k = 0; k < children.length; k++)
                matchTree(children[k], obj.children[offset + k]);
        };
        for (let k = 0; k < sceneRoots.length; k++)
            matchTree(sceneRoots[k], gltf.scene.children[k]);
    }

    return {
        parentIdx,
        nodeNames,
        resolveObj: (idx: number): THREE.Object3D | undefined => {
            if (idx < 0 || idx >= rawNodes.length || !inActiveScene.has(idx))
                return undefined;
            const hit = byNodeIdx.get(idx);
            if (hit)
                return hit;
            const name = nodeNames[idx];
            const sanitized = THREE.PropertyBinding.sanitizeNodeName(name);
            return byName.get(name) ?? byName.get(sanitized) ?? gltf.scene.getObjectByName(name);
        },
    };
}

export function resolveColliderShape(
    col: unknown,
    ownerName: string,
    shapes: readonly KhrImplicitShape[],
    nodeNames: readonly string[],
    resolveObj: (idx: number) => THREE.Object3D | undefined,
): KhrColliderShape {
    if (!isPlainObject(col))
        throw new KhrValidationError(`Node "${ownerName}" has invalid collider (expected object)`);
    const geo = col.geometry;
    if (!isPlainObject(geo))
        throw new KhrValidationError(`Node "${ownerName}" is missing collider.geometry object`);
    if (geo.shape !== undefined) {
        if (typeof geo.shape !== "number" || !Number.isInteger(geo.shape) || geo.shape < 0 || geo.shape >= shapes.length) {
            throw new KhrValidationError(`Node "${ownerName}" references missing collider.geometry.shape ${String(geo.shape)}`);
        }
        return shapes[geo.shape];
    }
    if (geo.node !== undefined) {
        if (typeof geo.node !== "number" || !Number.isInteger(geo.node)) {
            throw new KhrValidationError(`Node "${ownerName}" has invalid collider.geometry.node`);
        }
        if (geo.convexHull !== undefined && typeof geo.convexHull !== "boolean") {
            throw new KhrValidationError(`Node "${ownerName}" has non-boolean collider.geometry.convexHull`);
        }
        const target = resolveObj(geo.node);
        const meshName = nodeNames[geo.node] ?? `node_${geo.node}`;
        if (!target)
            throw new KhrValidationError(`Node "${ownerName}" references missing mesh node "${meshName}"`);
        return extractMeshColliderShape(target, meshName, Boolean(geo.convexHull));
    }
    throw new KhrValidationError(`Node "${ownerName}" collider.geometry must specify shape or node`);
}
