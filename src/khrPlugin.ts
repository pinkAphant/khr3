import * as THREE from "three";
import type { GLTF, GLTFLoaderPlugin, GLTFParser } from "three/examples/jsm/loaders/GLTFLoader.js";
import {
    attachBodyColliders,
    computeMissingBodyInertias,
    extractSceneExtras,
    findOwnerBodyIdx,
    isInsideRotor,
    resolveConnectedBodyIdx,
} from "./khrBundleFinalize.js";
import {
    applyKhrCustomExtractors,
    collectNodeSockets,
    decodeBreakawaySpec,
    decodePropellerTag,
    decodeWheelTag,
    type KhrNodeMeta,
    registerKhrTemplateMeta,
} from "./khrExtras.js";
import type { RawGltfJson } from "./khrGlb.js";
import { assertSceneIdentityWorldScale, computeAssemblyMassAndCom } from "./khrInertia.js";
import { decodeCollisionGroups, decodeJoint, decodeMotion, decodeShape } from "./khrParsers.js";
import { buildNodeGraphLookup, createHeadlessTexturePlugin, isHeadlessTextureEnv, isPlainObject } from "./khrPluginHelpers.js";
import { patchHeadlessTexturePlugins, reqArr, stubTextureInParser } from "./khrPluginHelpers.js";
import {
    type EcctrlPropellerTag,
    type EcctrlWheelTag,
    type KhrAssetBundle,
    type KhrBodySpec,
    type KhrPluginOptions,
    KhrValidationError,
} from "./khrTypes.js";

export const KHR_BUNDLE_USERDATA_KEY = "khrBundle";

export { applyKhrCustomExtractors, createHeadlessTexturePlugin, isHeadlessTextureEnv };

export function extractKhrBundleFromGltf(gltf: GLTF, options?: KhrPluginOptions): KhrAssetBundle {
    if (!gltf.scene)
        throw new KhrValidationError("GLTF asset has no active scene");
    gltf.scene.updateMatrixWorld(true);
    assertSceneIdentityWorldScale(gltf.scene);
    const json = gltf.parser?.json as RawGltfJson | undefined;
    if (!json)
        throw new KhrValidationError("GLTFParser is missing parsed JSON");
    if (json.extensions !== undefined && !isPlainObject(json.extensions)) {
        throw new KhrValidationError("glTF extensions must be an object");
    }
    const khrPhys = json.extensions?.KHR_physics_rigid_bodies;
    if (khrPhys !== undefined && !isPlainObject(khrPhys)) {
        throw new KhrValidationError("KHR_physics_rigid_bodies extension must be an object");
    }
    const khrShapes = json.extensions?.KHR_implicit_shapes;
    if (khrShapes !== undefined && !isPlainObject(khrShapes)) {
        throw new KhrValidationError("KHR_implicit_shapes extension must be an object");
    }
    const omiExt = json.extensions?.OMI_vehicle_wheel;
    if (omiExt !== undefined && !isPlainObject(omiExt)) {
        throw new KhrValidationError("OMI_vehicle_wheel extension must be an object");
    }
    if (omiExt !== undefined && !Array.isArray(omiExt.wheels)) {
        throw new KhrValidationError("OMI_vehicle_wheel.wheels must be an array");
    }

    const omiWheels = omiExt ? (omiExt.wheels as unknown[]) : [];
    const shapes = reqArr<Record<string, unknown>>(khrShapes?.shapes, "KHR_implicit_shapes.shapes").map((s, i) => decodeShape(s, i));
    const materials = reqArr<Record<string, unknown>>(khrPhys?.physicsMaterials, "KHR_physics_rigid_bodies.physicsMaterials");
    const collisionGroups = decodeCollisionGroups(reqArr<Record<string, unknown>>(khrPhys?.collisionFilters, "KHR_physics_rigid_bodies.collisionFilters"));
    const joints = reqArr<Record<string, unknown>>(khrPhys?.physicsJoints, "KHR_physics_rigid_bodies.physicsJoints");
    const rawNodes = reqArr<NonNullable<RawGltfJson["nodes"]>[number]>(json.nodes, "glTF nodes");

    const { parentIdx, nodeNames, resolveObj } = buildNodeGraphLookup(gltf, rawNodes, json);
    gltf.scene.traverse((child) => {
        if (child.userData && Object.getPrototypeOf(child.userData) !== Object.prototype) {
            Object.setPrototypeOf(child.userData, Object.prototype);
        }
    });
    const nodeMetas: KhrNodeMeta[] = [];
    const sockets: Record<string, [number, number, number]> = {};
    for (let i = 0; i < rawNodes.length; i++) {
        const obj = resolveObj(i);
        if (!obj)
            continue;
        if (Object.getPrototypeOf(obj.userData) !== Object.prototype) {
            Object.setPrototypeOf(obj.userData, Object.prototype);
        }
        const rawExt = rawNodes[i].extensions;
        if (rawExt !== undefined && !isPlainObject(rawExt)) {
            throw new KhrValidationError(`Node "${nodeNames[i]}" has invalid extensions`);
        }
        if (isPlainObject(rawExt)) {
            if (Object.hasOwn(rawExt, "KOLMO_emitter") && rawExt.KOLMO_emitter !== undefined && !Object.hasOwn(obj.userData, "KOLMO_emitter")) {
                obj.userData.KOLMO_emitter = structuredClone(rawExt.KOLMO_emitter);
            }
        }
        const extras = isPlainObject(rawNodes[i].extras) ? rawNodes[i].extras! : {};
        nodeMetas.push(collectNodeSockets(obj, nodeNames[i], extras, sockets, options?.isSocketNode));
    }

    const bodies: KhrBodySpec[] = [];
    const bodyMetas: Array<{ nodeName: string; extras: Record<string, unknown> }> = [];
    const bodyByIdx = new Map<number, KhrBodySpec>();
    const explicitInertiaSet = new Set<number>();
    const wheels: EcctrlWheelTag[] = [];
    const propellers: EcctrlPropellerTag[] = [];

    for (let i = 0; i < rawNodes.length; i++) {
        const raw = rawNodes[i];
        const name = nodeNames[i];
        const obj = resolveObj(i);
        if (!obj)
            continue;
        const extras = isPlainObject(raw.extras) ? raw.extras : {};
        const hasOmiWheel = isPlainObject(raw.extensions) && raw.extensions.OMI_vehicle_wheel !== undefined;
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
                    if (!isPlainObject(omiRef) || typeof omiRef.wheel !== "number" || !Number.isInteger(omiRef.wheel) ||
                        omiRef.wheel < 0 || omiRef.wheel >= omiWheels.length) {
                        throw new KhrValidationError(`Node "${name}" references invalid OMI_vehicle_wheel.wheel ${String(isPlainObject(omiRef) ? omiRef.wheel : omiRef)}`);
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
        if (!isPlainObject(phys))
            throw new KhrValidationError(`Node "${name}" has invalid KHR_physics_rigid_bodies`);
        if (phys.motion === undefined && phys.collider === undefined && phys.joint === undefined)
            continue;
        if (phys.motion === undefined && findOwnerBodyIdx(i, parentIdx, rawNodes) !== -1) {
            if (phys.joint !== undefined)
                throw new KhrValidationError(`Node "${name}" has joint without motion`);
            continue;
        }
        if (phys.motion !== undefined && !isPlainObject(phys.motion)) {
            throw new KhrValidationError(`Node "${name}" has invalid motion`);
        }
        const { hasExplicitInertia, ...motion } = decodeMotion(
            (phys.motion ?? {}) as Record<string, unknown>,
            phys.motion !== undefined,
            name,
        );
        if (hasExplicitInertia)
            explicitInertiaSet.add(i);
        const role = options?.extractRole?.(name, extras) ?? (typeof extras.role === "string" ? extras.role : null);
        let joint: KhrBodySpec["joint"] = null;
        if (phys.joint !== undefined) {
            if (!isPlainObject(phys.joint))
                throw new KhrValidationError(`Node "${name}" has invalid joint`);
            const rawJ = phys.joint;
            if (typeof rawJ.joint !== "number" || !Number.isInteger(rawJ.joint)) {
                throw new KhrValidationError(`Joint on "${name}" is missing numeric joint index`);
            }
            if (typeof rawJ.connectedNode !== "number" || !Number.isInteger(rawJ.connectedNode)) {
                throw new KhrValidationError(`Joint on "${name}" is missing numeric connectedNode index`);
            }
            if (rawJ.enableCollision !== undefined && typeof rawJ.enableCollision !== "boolean") {
                throw new KhrValidationError(`Joint on "${name}" has non-boolean enableCollision`);
            }
            const cIdx = resolveConnectedBodyIdx(rawJ.connectedNode, parentIdx, rawNodes);
            const cName = nodeNames[cIdx] ?? `node_${cIdx}`;
            const cObj = resolveObj(cIdx);
            if (!cObj || !resolveObj(rawJ.connectedNode))
                throw new KhrValidationError(`Joint on "${name}" targets missing connectedNode "${cName}"`);
            if (isInsideRotor(rawJ.connectedNode, parentIdx, rawNodes) || isInsideRotor(cIdx, parentIdx, rawNodes))
                throw new KhrValidationError(`Joint on "${name}" cannot target detached rotor "${cName}"`);
            joint = decodeJoint(joints, rawJ.joint, name, cObj, obj, Boolean(rawJ.enableCollision), role, cName);
        }
        const wp = obj.getWorldPosition(new THREE.Vector3());
        const wq = obj.getWorldQuaternion(new THREE.Quaternion());
        const breakaway = decodeBreakawaySpec(extras.breakaway);
        const spec: KhrBodySpec = {
            nodeName: name,
            ...motion,
            localPosition: [wp.x, wp.y, wp.z],
            localQuaternion: [wq.x, wq.y, wq.z, wq.w],
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

    attachBodyColliders(rawNodes, parentIdx, nodeNames, resolveObj, bodyByIdx, shapes, materials, collisionGroups);
    computeMissingBodyInertias(bodyByIdx, explicitInertiaSet);

    const { totalDynamicMass, centerOfMass } = computeAssemblyMassAndCom(bodies);
    const { ikSockets, tune, droneTune } = extractSceneExtras(json, sockets);
    registerKhrTemplateMeta(gltf.scene, { nodes: nodeMetas, bodies: bodyMetas, ikSockets });

    return {
        scene: gltf.scene,
        bodies,
        wheels,
        propellers,
        sockets,
        totalDynamicMass,
        centerOfMass,
        tune,
        droneTune,
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
