import * as THREE from "three";
import { GLTFLoader, type GLTF, type GLTFLoaderPlugin, type GLTFParser } from "three/examples/jsm/loaders/GLTFLoader.js";
import {
    collectNodeSockets,
    decodeBreakawaySpec,
    decodeCollisionGroups,
    decodeJoint,
    decodeMotion,
    decodePhysicsMaterial,
    decodePropellerTag,
    decodeShape,
    decodeWheelTag,
    type KhrNodeMeta,
    type KhrTemplateMeta,
} from "./decoders.js";
import { type RawGltfJson, validateGlbHeader } from "./glb.js";
import {
    assertSceneIdentityWorldScale,
    computeAssemblyMassAndCom,
    computeRelativePose,
    extractMeshColliderShape,
    readFiniteVec3,
} from "./inertia.js";
import {
    type EcctrlPropellerTag,
    type EcctrlWheelTag,
    isObj,
    type KhrAssetBundle,
    type KhrBodySpec,
    type KhrCarConfig,
    type KhrColliderShape,
    type KhrDroneConfig,
    type KhrImplicitShape,
    type KhrPluginOptions,
    KhrValidationError,
    type LoadKhrGlbOptions,
} from "./types.js";

export const KHR_BUNDLE_USERDATA_KEY = "khrBundle";

const HEADLESS_TEXTURE_EXT_NAMES = ["EXT_texture_webp", "EXT_texture_avif", "KHR_texture_basisu"] as const;
const _nodePos = new THREE.Vector3();
const _nodeQuat = new THREE.Quaternion();

export function isHeadlessTextureEnv(): boolean {
    return typeof document === "undefined" && (typeof createImageBitmap === "undefined" || typeof self === "undefined");
}

function stubTextureInParser(parser: GLTFParser, textureIndex: number): Promise<THREE.Texture> {
    const tex = new THREE.Texture();
    tex.flipY = false;
    parser.associations.set(tex, { textures: textureIndex } as never);
    return Promise.resolve(tex);
}

function patchHeadlessTexturePlugins(parser: GLTFParser): void {
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

function reqArr<T>(val: unknown, label: string): T[] {
    if (val === undefined)
        return [];
    if (!Array.isArray(val))
        throw new KhrValidationError(`${label} must be an array`);
    return val as T[];
}

const templateMetaMap = new WeakMap<THREE.Object3D, KhrTemplateMeta>();

export function applyKhrCustomExtractors(
    clonedBundle: KhrAssetBundle,
    templateBundle: KhrAssetBundle,
    extractRole?: KhrPluginOptions["extractRole"],
    isSocketNode?: KhrPluginOptions["isSocketNode"],
): void {
    const meta = templateMetaMap.get(templateBundle.scene);
    if (!meta)
        return;
    if (extractRole) {
        for (let i = 0; i < clonedBundle.bodies.length; i++) {
            const rawBody = meta.bodies[i];
            const dstBody = clonedBundle.bodies[i];
            if (!rawBody || !dstBody)
                continue;
            const role = extractRole(rawBody.nodeName, rawBody.extras)
                ?? (typeof rawBody.extras.role === "string" ? rawBody.extras.role : null);
            dstBody.role = role;
            if (dstBody.joint)
                dstBody.joint.role = role;
        }
    }
    if (isSocketNode) {
        const sockets: Record<string, [number, number, number]> = Object.create(null);
        for (const node of meta.nodes) {
            const custom = isSocketNode(node.nodeName, node.extras);
            if (custom || node.extras.isIkAnchor === true || typeof node.extras.socketId === "string") {
                const key = typeof custom === "string" && custom.length > 0
                    ? custom
                    : (typeof node.extras.socketId === "string" ? node.extras.socketId : node.nodeName);
                sockets[key] = [...node.worldPosition];
            }
            if (node.contactDots) {
                for (const [role, pos] of node.contactDots)
                    sockets[role] = [...pos];
            }
        }
        for (const [key, pos] of meta.ikSockets)
            sockets[key] = [...pos];
        clonedBundle.sockets = sockets;
    }
}

type RawNode = NonNullable<RawGltfJson["nodes"]>[number];

function buildNodeGraphLookup(
    gltf: GLTF,
    rawNodes: NonNullable<RawGltfJson["nodes"]>,
    rawGltf: RawGltfJson | undefined,
): {
    parentIdx: Int32Array;
    nodeNames: string[];
    resolveObj: (idx: number) => THREE.Object3D | undefined;
} {
    const parentIdx = new Int32Array(rawNodes.length).fill(-1);
    for (let i = 0; i < rawNodes.length; i++) {
        const node = rawNodes[i];
        if (!isObj(node))
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
        if (!isObj(sceneDef))
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

function findOwnerBodyIdx(idx: number, parentIdx: Int32Array, rawNodes: readonly RawNode[]): number {
    for (let p = parentIdx[idx]; p !== -1; p = parentIdx[p]) {
        if (rawNodes[p].extensions?.KHR_physics_rigid_bodies?.motion)
            return p;
    }
    return -1;
}

function resolveConnectedBodyIdx(cIdx: number, parentIdx: Int32Array, rawNodes: readonly RawNode[]): number {
    const phys = rawNodes[cIdx]?.extensions?.KHR_physics_rigid_bodies;
    const hasOwnBody = isObj(phys) && (
        phys.motion !== undefined ||
        ((phys.collider !== undefined || phys.joint !== undefined) && findOwnerBodyIdx(cIdx, parentIdx, rawNodes) === -1)
    );
    if (hasOwnBody)
        return cIdx;
    const owner = findOwnerBodyIdx(cIdx, parentIdx, rawNodes);
    return owner !== -1 ? owner : cIdx;
}

function isInsideRotor(idx: number, parentIdx: Int32Array, rawNodes: readonly RawNode[]): boolean {
    for (let p = idx; p !== -1; p = parentIdx[p]) {
        const node = rawNodes[p];
        const ex = node.extras;
        if (isObj(ex) && (ex.ecctrlWheel !== undefined || ex.ecctrlPropeller !== undefined))
            return true;
        if (isObj(node.extensions) && node.extensions.OMI_vehicle_wheel !== undefined)
            return true;
    }
    return false;
}

function resolveColliderShape(
    col: unknown,
    ownerName: string,
    shapes: readonly KhrImplicitShape[],
    nodeNames: readonly string[],
    resolveObj: (idx: number) => THREE.Object3D | undefined,
): KhrColliderShape {
    if (!isObj(col))
        throw new KhrValidationError(`Node "${ownerName}" has invalid collider (expected object)`);
    const geo = col.geometry;
    if (!isObj(geo))
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

export function extractKhrBundleFromGltf(gltf: GLTF, options?: KhrPluginOptions): KhrAssetBundle {
    if (!gltf.scene)
        throw new KhrValidationError("GLTF asset has no active scene");
    assertSceneIdentityWorldScale(gltf.scene);
    const json = gltf.parser?.json as RawGltfJson | undefined;
    if (!json)
        throw new KhrValidationError("GLTFParser is missing parsed JSON");
    if (json.extensions !== undefined && !isObj(json.extensions))
        throw new KhrValidationError("glTF extensions must be an object");

    const khrPhys = json.extensions?.KHR_physics_rigid_bodies;
    if (khrPhys !== undefined && !isObj(khrPhys))
        throw new KhrValidationError("KHR_physics_rigid_bodies extension must be an object");
    const khrShapes = json.extensions?.KHR_implicit_shapes;
    if (khrShapes !== undefined && !isObj(khrShapes))
        throw new KhrValidationError("KHR_implicit_shapes extension must be an object");
    const omiExt = json.extensions?.OMI_vehicle_wheel;
    if (omiExt !== undefined && (!isObj(omiExt) || !Array.isArray(omiExt.wheels))) {
        throw new KhrValidationError("OMI_vehicle_wheel.wheels must be an array");
    }

    const omiWheels = omiExt ? (omiExt.wheels as unknown[]) : [];
    const shapes = reqArr<Record<string, unknown>>(khrShapes?.shapes, "KHR_implicit_shapes.shapes").map((s, i) => decodeShape(s, i));
    const materials = reqArr<Record<string, unknown>>(khrPhys?.physicsMaterials, "KHR_physics_rigid_bodies.physicsMaterials");
    const collisionGroups = decodeCollisionGroups(reqArr<Record<string, unknown>>(khrPhys?.collisionFilters, "KHR_physics_rigid_bodies.collisionFilters"));
    const joints = reqArr<Record<string, unknown>>(khrPhys?.physicsJoints, "KHR_physics_rigid_bodies.physicsJoints");
    const rawNodes = reqArr<RawNode>(json.nodes, "glTF nodes");

    const { parentIdx, nodeNames, resolveObj } = buildNodeGraphLookup(gltf, rawNodes, json);
    const nodeMetas: KhrNodeMeta[] = [];
    const sockets: Record<string, [number, number, number]> = Object.create(null);

    for (let i = 0; i < rawNodes.length; i++) {
        const obj = resolveObj(i);
        if (!obj)
            continue;
        const rawExt = rawNodes[i].extensions;
        if (rawExt !== undefined && !isObj(rawExt))
            throw new KhrValidationError(`Node "${nodeNames[i]}" has invalid extensions`);
        const extras = isObj(rawNodes[i].extras) ? rawNodes[i].extras! : {};
        nodeMetas.push(collectNodeSockets(obj, nodeNames[i], extras, sockets, options?.isSocketNode));
    }

    const bodies: KhrBodySpec[] = [];
    const bodyMetas: Array<{ nodeName: string; extras: Record<string, unknown> }> = [];
    const bodyByIdx = new Map<number, KhrBodySpec>();
    const wheels: EcctrlWheelTag[] = [];
    const propellers: EcctrlPropellerTag[] = [];

    for (let i = 0; i < rawNodes.length; i++) {
        const raw = rawNodes[i];
        const name = nodeNames[i];
        const obj = resolveObj(i);
        if (!obj)
            continue;
        const extras = isObj(raw.extras) ? raw.extras : {};
        const hasOmiWheel = isObj(raw.extensions) && raw.extensions.OMI_vehicle_wheel !== undefined;
        if (extras.ecctrlWheel !== undefined || extras.ecctrlPropeller !== undefined || hasOmiWheel) {
            if (parentIdx[i] !== -1 && isInsideRotor(parentIdx[i], parentIdx, rawNodes)) {
                throw new KhrValidationError(`Nested rotor "${name}" inside detached rotor is unsupported`);
            }
            if (extras.ecctrlPropeller !== undefined && (extras.ecctrlWheel !== undefined || hasOmiWheel)) {
                throw new KhrValidationError(`Node "${name}" cannot be both wheel and propeller`);
            }
            if (extras.ecctrlPropeller !== undefined) {
                propellers.push(decodePropellerTag(obj, name, extras.ecctrlPropeller, gltf.scene));
            }
            else {
                let rawOmiWheel: unknown;
                if (hasOmiWheel) {
                    const omiRef = raw.extensions!.OMI_vehicle_wheel;
                    if (!isObj(omiRef) || typeof omiRef.wheel !== "number" || !Number.isInteger(omiRef.wheel) ||
                        omiRef.wheel < 0 || omiRef.wheel >= omiWheels.length) {
                        throw new KhrValidationError(`Node "${name}" references invalid OMI_vehicle_wheel.wheel ${String(isObj(omiRef) ? omiRef.wheel : omiRef)}`);
                    }
                    rawOmiWheel = omiWheels[omiRef.wheel];
                }
                wheels.push(decodeWheelTag(obj, name, extras.ecctrlWheel, gltf.scene, rawOmiWheel));
            }
            continue;
        }
        if (isInsideRotor(i, parentIdx, rawNodes))
            continue;
        const phys = raw.extensions?.KHR_physics_rigid_bodies;
        if (phys === undefined)
            continue;
        if (!isObj(phys))
            throw new KhrValidationError(`Node "${name}" has invalid KHR_physics_rigid_bodies`);
        if (phys.motion === undefined && phys.collider === undefined && phys.joint === undefined)
            continue;
        if (phys.motion === undefined && findOwnerBodyIdx(i, parentIdx, rawNodes) !== -1) {
            if (phys.joint !== undefined)
                throw new KhrValidationError(`Node "${name}" has joint without motion`);
            continue;
        }
        if (phys.motion !== undefined && !isObj(phys.motion))
            throw new KhrValidationError(`Node "${name}" has invalid motion`);

        const motion = decodeMotion(
            (phys.motion ?? {}) as Record<string, unknown>,
            phys.motion !== undefined,
            name,
        );
        const role = options?.extractRole?.(name, extras) ?? (typeof extras.role === "string" ? extras.role : null);
        let joint: KhrBodySpec["joint"] = null;
        if (phys.joint !== undefined) {
            if (!isObj(phys.joint))
                throw new KhrValidationError(`Node "${name}" has invalid joint`);
            const rawJ = phys.joint;
            if (typeof rawJ.joint !== "number" || !Number.isInteger(rawJ.joint))
                throw new KhrValidationError(`Joint on "${name}" is missing numeric joint index`);
            if (typeof rawJ.connectedNode !== "number" || !Number.isInteger(rawJ.connectedNode))
                throw new KhrValidationError(`Joint on "${name}" is missing numeric connectedNode index`);
            if (rawJ.enableCollision !== undefined && typeof rawJ.enableCollision !== "boolean")
                throw new KhrValidationError(`Joint on "${name}" has non-boolean enableCollision`);

            const cIdx = resolveConnectedBodyIdx(rawJ.connectedNode, parentIdx, rawNodes);
            const cName = nodeNames[cIdx] ?? `node_${cIdx}`;
            const cObj = resolveObj(cIdx);
            if (!cObj || !resolveObj(rawJ.connectedNode))
                throw new KhrValidationError(`Joint on "${name}" targets missing connectedNode "${cName}"`);
            if (isInsideRotor(rawJ.connectedNode, parentIdx, rawNodes) || isInsideRotor(cIdx, parentIdx, rawNodes))
                throw new KhrValidationError(`Joint on "${name}" cannot target detached rotor "${cName}"`);
            joint = decodeJoint(joints, rawJ.joint, name, cObj, obj, Boolean(rawJ.enableCollision), role, cName);
        }
        obj.getWorldPosition(_nodePos);
        obj.getWorldQuaternion(_nodeQuat);
        const breakaway = decodeBreakawaySpec(extras.breakaway);
        const spec: KhrBodySpec = {
            nodeName: name,
            ...motion,
            localPosition: [_nodePos.x, _nodePos.y, _nodePos.z],
            localQuaternion: [_nodeQuat.x, _nodeQuat.y, _nodeQuat.z, _nodeQuat.w],
            colliders: [],
            joint,
            ...(breakaway ? { breakaway } : {}),
            object: obj,
            role,
        };
        bodies.push(spec);
        bodyMetas.push({ nodeName: name, extras });
        bodyByIdx.set(i, spec);
    }

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
        const colDef = rawCol as { physicsMaterial?: unknown; collisionFilter?: unknown };
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

    for (const body of bodies) {
        if (!body.isKinematic && !body.hasExplicitInertia && body.colliders.length === 0) {
            throw new KhrValidationError(`Dynamic node "${body.nodeName}" is missing motion.inertiaDiagonal and has no colliders`);
        }
    }

    const { totalDynamicMass, centerOfMass } = computeAssemblyMassAndCom(bodies);
    const ikSockets: Array<[string, [number, number, number]]> = [];
    const sceneExtras = json.scenes?.[json.scene ?? 0]?.extras ?? json.extras;
    if (sceneExtras?.ikSockets !== undefined) {
        if (!isObj(sceneExtras.ikSockets))
            throw new KhrValidationError("scene.extras.ikSockets must be an object");
        for (const [k, v] of Object.entries(sceneExtras.ikSockets)) {
            const pos = readFiniteVec3(v, `scene.extras.ikSockets.${k}`);
            sockets[k] = pos;
            ikSockets.push([k, pos]);
        }
    }
    const rawTune = isObj(sceneExtras?.tune) ? sceneExtras.tune : (isObj(sceneExtras?.ecctrl) ? sceneExtras.ecctrl : undefined);
    const tune = rawTune as Partial<KhrCarConfig> | undefined;
    const droneTune = isObj(sceneExtras?.droneTune) ? (sceneExtras.droneTune as Partial<KhrDroneConfig>) : undefined;

    templateMetaMap.set(gltf.scene, { nodes: nodeMetas, bodies: bodyMetas, ikSockets });

    return {
        scene: gltf.scene,
        bodies,
        wheels,
        propellers,
        sockets,
        totalDynamicMass,
        centerOfMass,
        ...(tune ? { tune } : {}),
        ...(droneTune ? { droneTune } : {}),
    };
}

export function createKhrGltfPlugin(options?: KhrPluginOptions): (parser: GLTFParser) => GLTFLoaderPlugin {
    return (parser: GLTFParser): GLTFLoaderPlugin => {
        const headless = isHeadlessTextureEnv();
        return {
            name: "KHR_physics_rigid_bodies",
            ...(headless
                ? {
                    beforeRoot(): Promise<void> | null {
                        patchHeadlessTexturePlugins(parser);
                        return null;
                    },
                    loadTexture: (textureIndex: number): Promise<THREE.Texture> => stubTextureInParser(parser, textureIndex),
                }
                : {}),
            afterRoot(gltf: GLTF): Promise<void> | null {
                gltf.userData[KHR_BUNDLE_USERDATA_KEY] = extractKhrBundleFromGltf(gltf, options);
                return null;
            },
        };
    };
}

export async function loadKhrGlb(source: ArrayBuffer | Uint8Array, options?: LoadKhrGlbOptions): Promise<KhrAssetBundle> {
    const buf = source instanceof ArrayBuffer
        ? source
        : (source.byteOffset === 0 && source.byteLength === source.buffer.byteLength && source.buffer instanceof ArrayBuffer
            ? source.buffer
            : source.buffer.slice(source.byteOffset, source.byteOffset + source.byteLength) as ArrayBuffer);
    validateGlbHeader(buf);
    const loader = options?.loader ?? new GLTFLoader();
    const plugin = createKhrGltfPlugin(options);
    loader.register(plugin);
    try {
        const toErr = (err: unknown): KhrValidationError =>
            err instanceof KhrValidationError
                ? err
                : new KhrValidationError(err instanceof Error ? err.message : "Failed to parse GLB");
        const gltf = await new Promise<{ userData: Record<string, unknown> }>((resolve, reject) => {
            loader.parse(buf, "", resolve as never, (err) => reject(toErr(err)));
        });
        const bundle = gltf.userData[KHR_BUNDLE_USERDATA_KEY] as KhrAssetBundle | undefined;
        if (!bundle)
            throw new KhrValidationError("Missing parsed KHR bundle on GLTF userData");
        return bundle;
    }
    finally {
        loader.unregister(plugin);
    }
}
