import type * as THREE from "three";
import { GLTFExporter } from "three/examples/jsm/exporters/GLTFExporter.js";
import {
    encodeCollisionFilter,
    encodeJoint,
    encodeMotion,
    encodeOmiWheel,
    encodePhysicsMaterial,
    encodeShape,
} from "./khrExportEncoders.js";
import { ensureHeadlessFileReader, packJsonToGlb, patchGlbJsonChunk } from "./khrGlb.js";
import { type KhrExportNodeSpec, KhrValidationError, type OmiVehicleWheelSpec } from "./khrTypes.js";

export { patchGlbJsonChunk };

const isPos = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v > 0;
const isNonNeg = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v >= 0;

function synthesizeOmiWheel(raw: unknown): OmiVehicleWheelSpec | null {
    if (!raw || typeof raw !== "object" || Array.isArray(raw))
        return null;
    const ec = raw as Record<string, unknown>;
    if (!isPos(ec.radius) || !isPos(ec.width) || !isNonNeg(ec.travel) || !isNonNeg(ec.sag) || ec.sag > ec.travel)
        return null;
    if (typeof ec.steer !== "boolean" || typeof ec.drive !== "boolean" || typeof ec.brake !== "boolean")
        return null;
    const springK = isNonNeg(ec.springK) ? ec.springK : undefined;
    const dampingC = isNonNeg(ec.dampingC) ? ec.dampingC : undefined;
    const maxForce = isNonNeg(ec.maxBrakeTorque) ? ec.maxBrakeTorque / ec.radius : undefined;
    const maxSteeringAngle = isNonNeg(ec.maxSteeringAngle) ? ec.maxSteeringAngle : undefined;
    return {
        radius: ec.radius,
        width: ec.width,
        steering: ec.steer,
        propulsion: ec.drive,
        suspensionTravel: ec.travel,
        ...(maxForce === undefined ? {} : { maxForce }),
        ...(maxSteeringAngle === undefined ? {} : { maxSteeringAngle }),
        ...(springK === undefined ? {} : { suspensionStiffness: springK }),
        ...(dampingC === undefined ? {} : { suspensionDampingCompression: dampingC, suspensionDampingRebound: dampingC }),
    };
}

function stripUnloadedTextures(root: THREE.Object3D): Array<{ rec: Record<string, unknown>; slot: string; tex: unknown }> {
    const stripped: Array<{ rec: Record<string, unknown>; slot: string; tex: unknown }> = [];
    root.traverse((obj) => {
        const rawMat = (obj as { material?: unknown }).material;
        for (const m of Array.isArray(rawMat) ? rawMat : rawMat ? [rawMat] : []) {
            if (!m || typeof m !== "object") continue;
            const rec = m as Record<string, unknown>;
            for (const slot of Object.keys(rec)) {
                const val = rec[slot];
                if (val && typeof val === "object" && (val as { isTexture?: boolean }).isTexture && !(val as THREE.Texture).image) {
                    stripped.push({ rec, slot, tex: val });
                    rec[slot] = null;
                }
            }
        }
    });
    return stripped;
}

export async function exportKhrGlb(
    root: THREE.Object3D,
    specs: readonly KhrExportNodeSpec[],
    sceneExtras?: Record<string, unknown>,
): Promise<ArrayBuffer> {
    ensureHeadlessFileReader();
    root.updateMatrixWorld(true);
    const exporter = new GLTFExporter();
    const stripped = stripUnloadedTextures(root);
    let rawBuf: ArrayBuffer;
    try {
        const res = await exporter.parseAsync(root, { binary: true });
        rawBuf = res instanceof ArrayBuffer ? res : packJsonToGlb(res);
    }
    finally {
        for (const item of stripped)
            item.rec[item.slot] = item.tex;
    }
    return patchGlbJsonChunk(rawBuf, (gltf) => {
        const nodes = gltf.nodes ?? [];
        const nameToIndices = new Map<string, number[]>();
        nodes.forEach((n, idx) => {
            if (!n.name)
                return;
            const list = nameToIndices.get(n.name);
            if (list)
                list.push(idx);
            else
                nameToIndices.set(n.name, [idx]);
        });
        const requireNodeIdx = (name: string): number => {
            const list = nameToIndices.get(name);
            if (!list || list.length === 0)
                throw new KhrValidationError(`exportKhrGlb: missing scene node "${name}"`);
            if (list.length > 1)
                throw new KhrValidationError(`exportKhrGlb: ambiguous duplicate scene node "${name}" (${list.length} matches)`);
            return list[0];
        };

        const shapes: Record<string, unknown>[] = [];
        const shapeIndexByJson = new Map<string, number>();
        const materials: Record<string, unknown>[] = [];
        const matIndexByJson = new Map<string, number>();
        const filters: Record<string, unknown>[] = [];
        const filterIndexByJson = new Map<string, number>();
        const joints: Record<string, unknown>[] = [];
        const omiWheels: Record<string, unknown>[] = [];
        const seenSpecNodes = new Set<string>();
        const used = new Set(gltf.extensionsUsed ?? []);
        let hasPhysics = false;
        const hasPreIndexedOmiWheel = specs.some(
            (s) => s.extensions && typeof s.extensions === "object" && !Array.isArray(s.extensions) &&
                Object.hasOwn(s.extensions, "OMI_vehicle_wheel") && s.extensions.OMI_vehicle_wheel !== undefined,
        );

        for (const spec of specs) {
            if (seenSpecNodes.has(spec.nodeName)) {
                throw new KhrValidationError(`exportKhrGlb: duplicate spec for node "${spec.nodeName}"`);
            }
            seenSpecNodes.add(spec.nodeName);
            const node = nodes[requireNodeIdx(spec.nodeName)];
            if (spec.extras) {
                node.extras = { ...node.extras, ...spec.extras };
            }
            if (spec.extensions !== undefined) {
                if (!spec.extensions || typeof spec.extensions !== "object" || Array.isArray(spec.extensions)) {
                    throw new KhrValidationError(`exportKhrGlb: extensions on "${spec.nodeName}" must be an object`);
                }
                if (Object.hasOwn(spec.extensions, "OMI_vehicle_wheel") && spec.extensions.OMI_vehicle_wheel !== undefined && spec.wheel !== undefined) {
                    throw new KhrValidationError(`exportKhrGlb: node "${spec.nodeName}" cannot specify both wheel and extensions.OMI_vehicle_wheel`);
                }
                if (
                    Object.hasOwn(spec.extensions, "KHR_physics_rigid_bodies") &&
                    spec.extensions.KHR_physics_rigid_bodies !== undefined &&
                    (spec.motion !== undefined || spec.collider !== undefined || spec.joint !== undefined)
                ) {
                    throw new KhrValidationError(`exportKhrGlb: node "${spec.nodeName}" cannot specify both extensions.KHR_physics_rigid_bodies and motion/collider/joint`);
                }
                node.extensions = { ...node.extensions, ...spec.extensions };
                for (const extName of Object.keys(spec.extensions))
                    used.add(extName);
            }
            const khr: Record<string, unknown> = {};
            if (spec.motion) {
                hasPhysics = true;
                khr.motion = encodeMotion(spec.motion, spec.nodeName);
            }
            if (spec.collider) {
                hasPhysics = true;
                const col: Record<string, unknown> = {};
                if (spec.collider.shape) {
                    const encoded = encodeShape(spec.collider.shape, spec.nodeName);
                    const key = JSON.stringify(encoded);
                    let sIdx = shapeIndexByJson.get(key);
                    if (sIdx === undefined) {
                        sIdx = shapes.length;
                        shapes.push(encoded);
                        shapeIndexByJson.set(key, sIdx);
                    }
                    col.geometry = { shape: sIdx };
                }
                else if (spec.collider.meshNodeName) {
                    if (typeof spec.collider.convexHull !== "boolean") {
                        throw new KhrValidationError(`exportKhrGlb: mesh collider on "${spec.nodeName}" requires explicit boolean convexHull`);
                    }
                    col.geometry = {
                        node: requireNodeIdx(spec.collider.meshNodeName),
                        convexHull: spec.collider.convexHull,
                    };
                }
                else {
                    throw new KhrValidationError(`exportKhrGlb: collider on "${spec.nodeName}" requires shape or meshNodeName`);
                }
                const mat = encodePhysicsMaterial(spec.collider, spec.nodeName);
                if (mat) {
                    const key = JSON.stringify(mat);
                    let mIdx = matIndexByJson.get(key);
                    if (mIdx === undefined) {
                        mIdx = materials.length;
                        materials.push(mat);
                        matIndexByJson.set(key, mIdx);
                    }
                    col.physicsMaterial = mIdx;
                }
                const filter = encodeCollisionFilter(spec.collider, spec.nodeName);
                if (filter) {
                    const key = JSON.stringify(filter);
                    let fIdx = filterIndexByJson.get(key);
                    if (fIdx === undefined) {
                        fIdx = filters.length;
                        filters.push(filter);
                        filterIndexByJson.set(key, fIdx);
                    }
                    col.collisionFilter = fIdx;
                }
                khr.collider = col;
            }
            if (spec.joint) {
                hasPhysics = true;
                const connIdx = requireNodeIdx(spec.joint.connectedNodeName);
                joints.push(encodeJoint(spec.joint, spec.nodeName));
                khr.joint = {
                    joint: joints.length - 1,
                    connectedNode: connIdx,
                    enableCollision: Boolean(spec.joint.enableCollision),
                };
            }
            const wheelSpec = spec.wheel ?? (!hasPreIndexedOmiWheel ? synthesizeOmiWheel(spec.extras?.ecctrlWheel) : null);
            if (wheelSpec) {
                const wIdx = omiWheels.length;
                omiWheels.push(encodeOmiWheel(wheelSpec, spec.nodeName));
                used.add("OMI_vehicle_wheel");
                node.extensions = {
                    ...node.extensions,
                    OMI_vehicle_wheel: { wheel: wIdx },
                };
            }
            if (Object.keys(khr).length > 0) {
                node.extensions = {
                    ...node.extensions,
                    KHR_physics_rigid_bodies: khr,
                };
            }
        }
        if (hasPreIndexedOmiWheel && omiWheels.length > 0) {
            throw new KhrValidationError("exportKhrGlb: cannot mix spec.wheel with spec.extensions.OMI_vehicle_wheel in the same call");
        }
        if (sceneExtras) {
            const scene = gltf.scenes?.[gltf.scene ?? 0];
            if (scene)
                scene.extras = { ...scene.extras, ...sceneExtras };
            else
                gltf.extras = { ...gltf.extras, ...sceneExtras };
        }
        gltf.extensions ??= {};
        if (shapes.length > 0) {
            used.add("KHR_implicit_shapes");
            gltf.extensions.KHR_implicit_shapes = { shapes };
        }
        if (hasPhysics) {
            used.add("KHR_physics_rigid_bodies");
            gltf.extensions.KHR_physics_rigid_bodies = {
                ...(materials.length > 0 ? { physicsMaterials: materials } : {}),
                ...(filters.length > 0 ? { collisionFilters: filters } : {}),
                ...(joints.length > 0 ? { physicsJoints: joints } : {}),
            };
        }
        if (omiWheels.length > 0) {
            used.add("OMI_vehicle_wheel");
            gltf.extensions.OMI_vehicle_wheel = { wheels: omiWheels };
        }
        if (used.size > 0)
            gltf.extensionsUsed = [...used];
    });
}
