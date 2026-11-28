import type * as THREE from "three";
import { GLTFExporter } from "three/examples/jsm/exporters/GLTFExporter.js";
import { snapshotSceneWithMountedSubtrees } from "./clone.js";
import { ensureHeadlessFileReader, packJsonToGlb, patchGlbJsonChunk } from "./glb.js";
import { readFiniteVec3, readUnitQuat } from "./inertia.js";
import {
    COMBINE_MODES,
    isFiniteNum,
    isNonNegNum,
    isObj,
    isPosNum,
    type KhrExportNodeSpec,
    type KhrImplicitShape,
    type KhrJointDriveSpec,
    KhrValidationError,
    type OmiVehicleWheelSpec,
    optFiniteNum,
} from "./types.js";

export { patchGlbJsonChunk };

function encodeShape(shape: KhrImplicitShape, nodeName: string): Record<string, unknown> {
    if (shape.type === "box") {
        if (!Array.isArray(shape.size) || shape.size.length !== 3 || !shape.size.every(isPosNum)) {
            throw new KhrValidationError(`exportKhrGlb: invalid box.size on "${nodeName}"`);
        }
        return { type: "box", box: { size: shape.size } };
    }
    if (shape.type === "sphere") {
        if (!isPosNum(shape.radius))
            throw new KhrValidationError(`exportKhrGlb: invalid sphere.radius on "${nodeName}"`);
        return { type: "sphere", sphere: { radius: shape.radius } };
    }
    if (!isPosNum(shape.height) || !isPosNum(shape.radiusTop) || !isPosNum(shape.radiusBottom)) {
        throw new KhrValidationError(`exportKhrGlb: invalid ${shape.type} dimensions on "${nodeName}"`);
    }
    if (Math.abs(shape.radiusTop - shape.radiusBottom) > 1e-6) {
        throw new KhrValidationError(`exportKhrGlb: tapered ${shape.type} (${shape.radiusTop} !== ${shape.radiusBottom}) on "${nodeName}" is unsupported`);
    }
    return {
        type: shape.type,
        [shape.type]: { height: shape.height, radiusTop: shape.radiusTop, radiusBottom: shape.radiusBottom },
    };
}

function normalizeJointAxis(axis: unknown, kind: string, nodeName: string): { primary: 0 | 1 | 2; unit: [number, number, number]; isCardinal: boolean } {
    if (!Array.isArray(axis) || axis.length !== 3 || !axis.every(isFiniteNum)) {
        throw new KhrValidationError(`exportKhrGlb: ${kind} joint on "${nodeName}" requires a finite axis`);
    }
    const len = Math.hypot(axis[0], axis[1], axis[2]);
    if (len <= 1e-8)
        throw new KhrValidationError(`exportKhrGlb: ${kind} joint on "${nodeName}" requires non-zero axis`);
    const ux = axis[0] / len, uy = axis[1] / len, uz = axis[2] / len;
    const ax = Math.abs(ux), ay = Math.abs(uy), az = Math.abs(uz);
    const primary: 0 | 1 | 2 = ax >= ay && ax >= az ? 0 : ay >= az ? 1 : 2;
    const maxComp = primary === 0 ? ax : primary === 1 ? ay : az;
    return { primary, unit: [ux, uy, uz], isCardinal: maxComp >= 1 - 1e-6 };
}

function encodeJointDrive(
    drive: KhrJointDriveSpec,
    defaultType: "linear" | "angular",
    defaultAxis: 0 | 1 | 2,
    nodeName: string,
): Record<string, unknown> {
    if (!isObj(drive))
        throw new KhrValidationError(`exportKhrGlb: joint drive on "${nodeName}" must be an object`);
    if (drive.mode !== "force" && drive.mode !== "acceleration") {
        throw new KhrValidationError(`exportKhrGlb: joint drive on "${nodeName}" requires drive.mode ("force" | "acceleration")`);
    }
    if (drive.type !== undefined && drive.type !== "linear" && drive.type !== "angular")
        throw new KhrValidationError(`exportKhrGlb: joint drive on "${nodeName}" has invalid drive.type`);
    if (drive.axis !== undefined && drive.axis !== 0 && drive.axis !== 1 && drive.axis !== 2)
        throw new KhrValidationError(`exportKhrGlb: joint drive on "${nodeName}" has invalid drive.axis`);

    const positionTarget = optFiniteNum(drive.positionTarget, `joint drive.positionTarget on "${nodeName}"`);
    const velocityTarget = optFiniteNum(drive.velocityTarget, `joint drive.velocityTarget on "${nodeName}"`);
    if (positionTarget === undefined && velocityTarget === undefined) {
        throw new KhrValidationError(`exportKhrGlb: joint drive on "${nodeName}" requires finite positionTarget or velocityTarget`);
    }
    const stiffness = optFiniteNum(drive.stiffness, `joint drive.stiffness on "${nodeName}"`, true);
    const damping = optFiniteNum(drive.damping, `joint drive.damping on "${nodeName}"`, true);
    const maxForce = optFiniteNum(drive.maxForce, `joint drive.maxForce on "${nodeName}"`, true);
    return {
        type: drive.type ?? defaultType,
        mode: drive.mode,
        axis: drive.axis ?? defaultAxis,
        ...(positionTarget === undefined ? {} : { positionTarget }),
        ...(velocityTarget === undefined ? {} : { velocityTarget }),
        ...(stiffness === undefined ? {} : { stiffness }),
        ...(damping === undefined ? {} : { damping }),
        ...(maxForce === undefined ? {} : { maxForce }),
    };
}

function encodeJoint(j: NonNullable<KhrExportNodeSpec["joint"]>, nodeName: string): Record<string, unknown> {
    if (j.kind !== "fixed" && j.kind !== "revolute" && j.kind !== "prismatic") {
        throw new KhrValidationError(`exportKhrGlb: invalid joint.kind "${String(j.kind)}" on "${nodeName}"`);
    }
    if (j.kind === "fixed") {
        if (j.drive || (j.drives && j.drives.length > 0)) {
            throw new KhrValidationError(`exportKhrGlb: fixed joint on "${nodeName}" cannot define drives`);
        }
        return { limits: [{ linearAxes: [0, 1, 2], min: 0, max: 0 }, { angularAxes: [0, 1, 2], min: 0, max: 0 }] };
    }
    const { primary: ax, unit, isCardinal } = normalizeJointAxis(j.axis, j.kind, nodeName);
    const lockedAxes = ([0, 1, 2] as const).filter((i) => i !== ax);
    if (j.limits && (!Number.isFinite(j.limits[0]) || !Number.isFinite(j.limits[1]) || j.limits[0] > j.limits[1])) {
        throw new KhrValidationError(`exportKhrGlb: joint on "${nodeName}" has invalid joint.limits`);
    }
    const stiffness = optFiniteNum(j.limitStiffness, `joint.limitStiffness on "${nodeName}"`, true);
    const damping = optFiniteNum(j.limitDamping, `joint.limitDamping on "${nodeName}"`, true);
    const freeKey = j.kind === "revolute" ? "angularAxes" : "linearAxes";
    const freeLimit = j.limits || stiffness !== undefined || damping !== undefined
        ? [{ [freeKey]: [ax], ...(j.limits ? { min: j.limits[0], max: j.limits[1] } : {}), ...(stiffness === undefined ? {} : { stiffness }), ...(damping === undefined ? {} : { damping }) }]
        : [];
    const out: Record<string, unknown> = {
        limits: j.kind === "revolute"
            ? [{ linearAxes: [0, 1, 2], min: 0, max: 0 }, { angularAxes: lockedAxes, min: 0, max: 0 }, ...freeLimit]
            : [{ angularAxes: [0, 1, 2], min: 0, max: 0 }, { linearAxes: lockedAxes, min: 0, max: 0 }, ...freeLimit],
        ...(!isCardinal || unit[ax] < 0 ? { extras: { jointAxis: unit } } : {}),
    };
    const drives = j.drives ?? (j.drive ? [j.drive] : []);
    if (drives.length > 0) {
        out.drives = drives.map((d) => encodeJointDrive(d, j.kind === "revolute" ? "angular" : "linear", ax, nodeName));
    }
    return out;
}

function encodeMotion(m: NonNullable<KhrExportNodeSpec["motion"]>, nodeName: string): Record<string, unknown> {
    const isKinematic = Boolean(m.isKinematic);
    if (!isKinematic && !isPosNum(m.mass))
        throw new KhrValidationError(`exportKhrGlb: dynamic node "${nodeName}" requires positive finite motion.mass`);
    if (isKinematic && m.mass !== undefined)
        optFiniteNum(m.mass, `kinematic motion.mass on "${nodeName}"`, true);
    if (m.inertiaDiagonal && readFiniteVec3(m.inertiaDiagonal, `${nodeName}.motion.inertiaDiagonal`).some((n) => (isKinematic ? n < 0 : n <= 0))) {
        throw new KhrValidationError(`exportKhrGlb: invalid motion.inertiaDiagonal on "${nodeName}"`);
    }
    const gravityFactor = optFiniteNum(m.gravityFactor, `motion.gravityFactor on "${nodeName}"`);
    return {
        isKinematic,
        ...(m.mass === undefined ? {} : { mass: m.mass }),
        ...(m.centerOfMass ? { centerOfMass: readFiniteVec3(m.centerOfMass, `${nodeName}.motion.centerOfMass`) } : {}),
        ...(m.inertiaDiagonal ? { inertiaDiagonal: m.inertiaDiagonal } : {}),
        ...(m.inertiaOrientation ? { inertiaOrientation: readUnitQuat(m.inertiaOrientation, `${nodeName}.motion.inertiaOrientation`) } : {}),
        ...(m.linearVelocity ? { linearVelocity: readFiniteVec3(m.linearVelocity, `${nodeName}.motion.linearVelocity`) } : {}),
        ...(m.angularVelocity ? { angularVelocity: readFiniteVec3(m.angularVelocity, `${nodeName}.motion.angularVelocity`) } : {}),
        ...(gravityFactor === undefined ? {} : { gravityFactor }),
    };
}

function encodePhysicsMaterial(col: NonNullable<KhrExportNodeSpec["collider"]>, nodeName: string): Record<string, unknown> | null {
    const friction = optFiniteNum(col.friction, `collider.friction on "${nodeName}"`, true);
    const restitution = optFiniteNum(col.restitution, `collider.restitution on "${nodeName}"`, true);
    for (const [k, v] of [["frictionCombine", col.frictionCombine], ["restitutionCombine", col.restitutionCombine]] as const) {
        if (v !== undefined && (typeof v !== "string" || !COMBINE_MODES.has(v))) {
            throw new KhrValidationError(`exportKhrGlb: invalid collider.${k} "${String(v)}" on "${nodeName}"`);
        }
    }
    if (friction === undefined && restitution === undefined && col.frictionCombine === undefined && col.restitutionCombine === undefined) {
        return null;
    }
    return {
        ...(friction === undefined ? {} : { staticFriction: friction, dynamicFriction: friction }),
        ...(restitution === undefined ? {} : { restitution }),
        ...(col.frictionCombine === undefined ? {} : { frictionCombine: col.frictionCombine }),
        ...(col.restitutionCombine === undefined ? {} : { restitutionCombine: col.restitutionCombine }),
    };
}

function validateCanonicalSystems(arr: unknown, label: string, nodeName: string): string[] {
    if (!Array.isArray(arr) || arr.some((s) => typeof s !== "string" || s.length === 0)) {
        throw new KhrValidationError(`exportKhrGlb: collider.${label} on "${nodeName}" must be an array of non-empty strings`);
    }
    return [...new Set(arr)];
}

function encodeCollisionFilter(col: NonNullable<KhrExportNodeSpec["collider"]>, nodeName: string): Record<string, unknown> | null {
    if (col.collideWithSystems !== undefined && col.notCollideWithSystems !== undefined) {
        throw new KhrValidationError(`exportKhrGlb: collider on "${nodeName}" cannot specify both collideWithSystems and notCollideWithSystems`);
    }
    if (col.collisionSystems === undefined && col.collideWithSystems === undefined && col.notCollideWithSystems === undefined) {
        return null;
    }
    return {
        ...(col.collisionSystems === undefined ? {} : { collisionSystems: validateCanonicalSystems(col.collisionSystems, "collisionSystems", nodeName) }),
        ...(col.collideWithSystems === undefined ? {} : { collideWithSystems: validateCanonicalSystems(col.collideWithSystems, "collideWithSystems", nodeName) }),
        ...(col.notCollideWithSystems === undefined ? {} : { notCollideWithSystems: validateCanonicalSystems(col.notCollideWithSystems, "notCollideWithSystems", nodeName) }),
    };
}

function encodeOmiWheel(wheel: OmiVehicleWheelSpec, nodeName: string): Record<string, unknown> {
    if (!isObj(wheel))
        throw new KhrValidationError(`exportKhrGlb: wheel on "${nodeName}" must be an object`);
    if (!isPosNum(wheel.radius))
        throw new KhrValidationError(`exportKhrGlb: wheel.radius on "${nodeName}" must be > 0`);
    if (!isPosNum(wheel.width))
        throw new KhrValidationError(`exportKhrGlb: wheel.width on "${nodeName}" must be > 0`);
    for (const [k, v] of [["steering", wheel.steering], ["propulsion", wheel.propulsion]] as const) {
        if (v !== undefined && typeof v !== "boolean")
            throw new KhrValidationError(`exportKhrGlb: wheel.${k} on "${nodeName}" must be boolean`);
    }
    const maxForce = optFiniteNum(wheel.maxForce, `wheel.maxForce on "${nodeName}"`, true);
    const maxSteeringAngle = optFiniteNum(wheel.maxSteeringAngle, `wheel.maxSteeringAngle on "${nodeName}"`, true);
    const suspensionStiffness = optFiniteNum(wheel.suspensionStiffness, `wheel.suspensionStiffness on "${nodeName}"`, true);
    const suspensionDampingCompression = optFiniteNum(wheel.suspensionDampingCompression, `wheel.suspensionDampingCompression on "${nodeName}"`, true);
    const suspensionDampingRebound = optFiniteNum(wheel.suspensionDampingRebound, `wheel.suspensionDampingRebound on "${nodeName}"`, true);
    const suspensionTravel = optFiniteNum(wheel.suspensionTravel, `wheel.suspensionTravel on "${nodeName}"`, true);
    return {
        radius: wheel.radius,
        width: wheel.width,
        ...(maxForce === undefined ? {} : { maxForce }),
        ...(wheel.steering === undefined ? {} : { steering: wheel.steering }),
        ...(wheel.propulsion === undefined ? {} : { propulsion: wheel.propulsion }),
        ...(maxSteeringAngle === undefined ? {} : { maxSteeringAngle }),
        ...(suspensionStiffness === undefined ? {} : { suspensionStiffness }),
        ...(suspensionDampingCompression === undefined ? {} : { suspensionDampingCompression }),
        ...(suspensionDampingRebound === undefined ? {} : { suspensionDampingRebound }),
        ...(suspensionTravel === undefined ? {} : { suspensionTravel }),
    };
}

function synthesizeOmiWheel(raw: unknown): OmiVehicleWheelSpec | null {
    if (!isObj(raw))
        return null;
    if (!isPosNum(raw.radius) || !isPosNum(raw.width) || !isNonNegNum(raw.travel) || !isNonNegNum(raw.sag) || raw.sag > raw.travel)
        return null;
    if (typeof raw.steer !== "boolean" || typeof raw.drive !== "boolean" || typeof raw.brake !== "boolean")
        return null;
    const springK = isNonNegNum(raw.springK) ? raw.springK : undefined;
    const dampingC = isNonNegNum(raw.dampingC) ? raw.dampingC : undefined;
    const maxForce = isNonNegNum(raw.maxBrakeTorque) ? raw.maxBrakeTorque / raw.radius : undefined;
    const maxSteeringAngle = isNonNegNum(raw.maxSteeringAngle) ? raw.maxSteeringAngle : undefined;
    return {
        radius: raw.radius,
        width: raw.width,
        steering: raw.steer,
        propulsion: raw.drive,
        suspensionTravel: raw.travel,
        ...(maxForce === undefined ? {} : { maxForce }),
        ...(maxSteeringAngle === undefined ? {} : { maxSteeringAngle }),
        ...(springK === undefined ? {} : { suspensionStiffness: springK }),
        ...(dampingC === undefined ? {} : { suspensionDampingCompression: dampingC, suspensionDampingRebound: dampingC }),
    };
}

/**
 * Replaces meshes' material references with shallow material clones ONLY when an unloaded headless Texture
 * (`!tex.image`) is present, keeping live WebGL materials untouched across async export.
 */
function isolateUnloadedTextures(root: THREE.Object3D): Array<{ mesh: THREE.Mesh; origMat: THREE.Material | THREE.Material[]; clones: THREE.Material[] }> {
    const swaps: Array<{ mesh: THREE.Mesh; origMat: THREE.Material | THREE.Material[]; clones: THREE.Material[] }> = [];
    const clonedByMat = new Map<THREE.Material, THREE.Material>();

    const sanitizeMat = (mat: THREE.Material): THREE.Material => {
        const cached = clonedByMat.get(mat);
        if (cached)
            return cached;
        const rec = mat as unknown as Record<string, unknown>;
        let hasUnloaded = false;
        for (const slot of Object.keys(rec)) {
            const val = rec[slot];
            if (val && typeof val === "object" && (val as { isTexture?: boolean }).isTexture && !(val as THREE.Texture).image) {
                hasUnloaded = true;
                break;
            }
        }
        if (!hasUnloaded)
            return mat;
        const clone = mat.clone();
        const cloneRec = clone as unknown as Record<string, unknown>;
        for (const slot of Object.keys(cloneRec)) {
            const val = cloneRec[slot];
            if (val && typeof val === "object" && (val as { isTexture?: boolean }).isTexture && !(val as THREE.Texture).image) {
                cloneRec[slot] = null;
            }
        }
        clonedByMat.set(mat, clone);
        return clone;
    };

    root.traverse((obj) => {
        const mesh = obj as THREE.Mesh;
        if (!mesh.isMesh || !mesh.material)
            return;
        if (Array.isArray(mesh.material)) {
            const next = mesh.material.map(sanitizeMat);
            if (next.some((m, idx) => m !== (mesh.material as THREE.Material[])[idx])) {
                swaps.push({ mesh, origMat: mesh.material, clones: next.filter((m, idx) => m !== (mesh.material as THREE.Material[])[idx]) });
                mesh.material = next;
            }
        }
        else {
            const next = sanitizeMat(mesh.material);
            if (next !== mesh.material) {
                swaps.push({ mesh, origMat: mesh.material, clones: [next] });
                mesh.material = next;
            }
        }
    });
    return swaps;
}

function internJson(table: Record<string, unknown>[], map: Map<string, number>, item: Record<string, unknown>): number {
    const key = JSON.stringify(item);
    let idx = map.get(key);
    if (idx === undefined) {
        idx = table.length;
        table.push(item);
        map.set(key, idx);
    }
    return idx;
}

export async function exportKhrGlb(
    root: THREE.Object3D,
    specs: readonly KhrExportNodeSpec[],
    sceneExtras?: Record<string, unknown>,
): Promise<ArrayBuffer> {
    ensureHeadlessFileReader();
    const exportRoot = snapshotSceneWithMountedSubtrees(root);
    exportRoot.updateMatrixWorld(true);
    const exporter = new GLTFExporter();
    const swaps = isolateUnloadedTextures(exportRoot);
    let rawBuf: ArrayBuffer;
    try {
        const res = await exporter.parseAsync(exportRoot, { binary: true });
        rawBuf = res instanceof ArrayBuffer ? res : packJsonToGlb(res);
    }
    finally {
        for (const s of swaps) {
            s.mesh.material = s.origMat;
            for (const c of s.clones)
                c.dispose();
        }
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
        const shapeMap = new Map<string, number>();
        const materials: Record<string, unknown>[] = [];
        const matMap = new Map<string, number>();
        const filters: Record<string, unknown>[] = [];
        const filterMap = new Map<string, number>();
        const joints: Record<string, unknown>[] = [];
        const jointMap = new Map<string, number>();
        const omiWheels: Record<string, unknown>[] = [];
        const wheelMap = new Map<string, number>();
        const seenSpecNodes = new Set<string>();
        const used = new Set(gltf.extensionsUsed ?? []);
        let hasPhysics = false;

        const hasPreIndexedOmiWheel = specs.some(
            (s) => isObj(s.extensions) && Object.hasOwn(s.extensions, "OMI_vehicle_wheel") && s.extensions.OMI_vehicle_wheel !== undefined,
        );

        for (const spec of specs) {
            if (seenSpecNodes.has(spec.nodeName)) {
                throw new KhrValidationError(`exportKhrGlb: duplicate spec for node "${spec.nodeName}"`);
            }
            seenSpecNodes.add(spec.nodeName);
            const node = nodes[requireNodeIdx(spec.nodeName)];
            if (spec.extras)
                node.extras = { ...node.extras, ...spec.extras };
            if (spec.extensions !== undefined) {
                if (!isObj(spec.extensions))
                    throw new KhrValidationError(`exportKhrGlb: extensions on "${spec.nodeName}" must be an object`);
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
                    col.geometry = { shape: internJson(shapes, shapeMap, encodeShape(spec.collider.shape, spec.nodeName)) };
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
                if (mat)
                    col.physicsMaterial = internJson(materials, matMap, mat);
                const filter = encodeCollisionFilter(spec.collider, spec.nodeName);
                if (filter)
                    col.collisionFilter = internJson(filters, filterMap, filter);
                khr.collider = col;
            }
            if (spec.joint) {
                hasPhysics = true;
                const connIdx = requireNodeIdx(spec.joint.connectedNodeName);
                const jIdx = internJson(joints, jointMap, encodeJoint(spec.joint, spec.nodeName));
                khr.joint = {
                    joint: jIdx,
                    connectedNode: connIdx,
                    enableCollision: Boolean(spec.joint.enableCollision),
                };
            }
            const wheelSpec = spec.wheel ?? (!hasPreIndexedOmiWheel ? synthesizeOmiWheel(spec.extras?.ecctrlWheel) : null);
            if (wheelSpec) {
                const wIdx = internJson(omiWheels, wheelMap, encodeOmiWheel(wheelSpec, spec.nodeName));
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
