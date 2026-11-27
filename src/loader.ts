import * as THREE from "three";
import { GLTFLoader, type GLTF, type GLTFLoaderPlugin, type GLTFParser } from "three/examples/jsm/loaders/GLTFLoader.js";
import { type RawGltfJson, validateGlbHeader } from "./glb.js";
import {
    assertIdentityWorldScale,
    assertSceneIdentityWorldScale,
    computeAssemblyMassAndCom,
    computeBodyInertia,
    computeRelativePose,
    extractMeshColliderShape,
    readFiniteVec3,
    readUnitQuat,
} from "./inertia.js";
import {
    COMBINE_MODES,
    type EcctrlPropellerTag,
    type EcctrlWheelTag,
    isFiniteNum,
    isNonNegNum,
    isObj,
    isPosNum,
    type KhrAssetBundle,
    type KhrBodySpec,
    type KhrBreakawaySpec,
    type KhrCarConfig,
    type KhrColliderShape,
    type KhrCombineMode,
    type KhrCurveData,
    type KhrDroneConfig,
    type KhrImplicitShape,
    type KhrJointDriveSpec,
    type KhrJointSpec,
    type KhrPluginOptions,
    KhrValidationError,
    type LoadKhrGlbOptions,
    optFiniteNum,
} from "./types.js";

export const KHR_BUNDLE_USERDATA_KEY = "khrBundle";

const HEADLESS_TEXTURE_EXT_NAMES = ["EXT_texture_webp", "EXT_texture_avif", "KHR_texture_basisu"] as const;

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

function decodeShape(raw: Record<string, unknown>, idx: number): KhrImplicitShape {
    if (!isObj(raw))
        throw new KhrValidationError(`KHR_implicit_shapes[${idx}] must be an object`);
    const type = raw.type;
    if (type === "box") {
        if (raw.box !== undefined && !isObj(raw.box))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid box`);
        const box = isObj(raw.box) ? raw.box : {};
        const size = box.size === undefined ? [1, 1, 1] : box.size;
        if (!Array.isArray(size) || size.length !== 3 || !size.every(isPosNum)) {
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid box.size`);
        }
        return { type: "box", size: [size[0], size[1], size[2]] };
    }
    if (type === "sphere") {
        if (raw.sphere !== undefined && !isObj(raw.sphere))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid sphere`);
        const sphere = isObj(raw.sphere) ? raw.sphere : {};
        const radius = sphere.radius === undefined ? 0.5 : sphere.radius;
        if (!isPosNum(radius))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid sphere.radius`);
        return { type: "sphere", radius };
    }
    if (type === "cylinder" || type === "capsule") {
        if (raw[type] !== undefined && !isObj(raw[type]))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}`);
        const g = isObj(raw[type]) ? (raw[type] as Record<string, unknown>) : {};
        const height = g.height === undefined ? (type === "cylinder" ? 2.0 : 1.0) : g.height;
        if (!isPosNum(height))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}.height`);
        const radiusTop = g.radiusTop === undefined ? 0.5 : g.radiusTop;
        const radiusBottom = g.radiusBottom === undefined ? 0.5 : g.radiusBottom;
        if (!isPosNum(radiusTop))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}.radiusTop`);
        if (!isPosNum(radiusBottom))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}.radiusBottom`);
        if (Math.abs(radiusTop - radiusBottom) > 1e-6) {
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] tapered ${type} (${radiusTop} !== ${radiusBottom}) is unsupported`);
        }
        return { type, height, radiusTop, radiusBottom };
    }
    throw new KhrValidationError(`KHR_implicit_shapes[${idx}] unsupported shape type "${String(type)}"`);
}

function optCombineMode(v: unknown, label: string): KhrCombineMode | undefined {
    if (v === undefined)
        return undefined;
    if (typeof v !== "string" || !COMBINE_MODES.has(v as KhrCombineMode)) {
        throw new KhrValidationError(`Invalid ${label}: "${String(v)}"`);
    }
    return v as KhrCombineMode;
}

function decodePhysicsMaterial(
    rawMaterials: Array<Record<string, unknown>>,
    idx: unknown,
    ownerName: string,
): {
    friction?: number;
    restitution?: number;
    frictionCombine?: KhrCombineMode;
    restitutionCombine?: KhrCombineMode;
} {
    if (idx === undefined)
        return {};
    if (typeof idx !== "number" || !Number.isInteger(idx) || idx < 0 || idx >= rawMaterials.length) {
        throw new KhrValidationError(`Node "${ownerName}" references out-of-bounds physicsMaterial ${String(idx)}`);
    }
    const mat = rawMaterials[idx];
    if (!isObj(mat))
        throw new KhrValidationError(`Invalid physicsMaterials[${idx}] on "${ownerName}"`);
    const dynF = optFiniteNum(mat.dynamicFriction, `physicsMaterials[${idx}].dynamicFriction`, true);
    const statF = optFiniteNum(mat.staticFriction, `physicsMaterials[${idx}].staticFriction`, true);
    const rest = optFiniteNum(mat.restitution, `physicsMaterials[${idx}].restitution`, true);
    const frictionCombine = optCombineMode(mat.frictionCombine, `physicsMaterials[${idx}].frictionCombine`);
    const restitutionCombine = optCombineMode(mat.restitutionCombine, `physicsMaterials[${idx}].restitutionCombine`);
    return {
        ...(dynF !== undefined || statF !== undefined ? { friction: dynF ?? statF } : {}),
        ...(rest === undefined ? {} : { restitution: rest }),
        ...(frictionCombine === undefined ? {} : { frictionCombine }),
        ...(restitutionCombine === undefined ? {} : { restitutionCombine }),
    };
}

function decodeCollisionGroups(rawFilters: Array<Record<string, unknown>>): number[] {
    const collisionSystemIds = new Map<string, number>([["default", 0]]);
    let nextCollisionBit = 1;
    const bitFor = (sys: string): number => {
        const hit = collisionSystemIds.get(sys);
        if (hit !== undefined)
            return hit;
        if (nextCollisionBit > 15) {
            throw new KhrValidationError(`Exceeded Rapier 16-group limit for collisionSystem "${sys}"`);
        }
        const b = nextCollisionBit++;
        collisionSystemIds.set(sys, b);
        return b;
    };
    const maskFor = (arr: unknown, label: string): number => {
        if (arr === undefined)
            return 0;
        if (!Array.isArray(arr))
            throw new KhrValidationError(`collisionFilter.${label} must be an array`);
        let mask = 0;
        for (const s of arr) {
            if (typeof s !== "string" || s.length === 0) {
                throw new KhrValidationError(`collisionFilter.${label} has invalid system`);
            }
            mask |= 1 << bitFor(s);
        }
        return mask;
    };
    return rawFilters.map((f, idx) => {
        if (!isObj(f))
            throw new KhrValidationError(`collisionFilters[${idx}] must be an object`);
        if (f.collideWithSystems !== undefined && f.notCollideWithSystems !== undefined) {
            throw new KhrValidationError(`collisionFilters[${idx}] cannot specify both collideWithSystems and notCollideWithSystems`);
        }
        const membership = maskFor(f.collisionSystems, "collisionSystems");
        const filter = f.collideWithSystems === undefined
            ? 0xffff & ~maskFor(f.notCollideWithSystems, "notCollideWithSystems")
            : maskFor(f.collideWithSystems, "collideWithSystems");
        return ((membership & 0xffff) << 16) | (filter & 0xffff);
    });
}

function decodeMotion(
    motion: Record<string, unknown>,
    hasMotion: boolean,
    name: string,
): Omit<KhrBodySpec, "nodeName" | "localPosition" | "localQuaternion" | "colliders" | "joint" | "object" | "role"> & {
    hasExplicitInertia: boolean;
} {
    if (motion.isKinematic !== undefined && typeof motion.isKinematic !== "boolean") {
        throw new KhrValidationError(`Node "${name}" has non-boolean motion.isKinematic`);
    }
    const isKinematic = !hasMotion || Boolean(motion.isKinematic);
    let mass = 0;
    if (!isKinematic) {
        if (!isPosNum(motion.mass))
            throw new KhrValidationError(`Dynamic node "${name}" is missing positive finite motion.mass`);
        mass = motion.mass;
    }
    else if (motion.mass !== undefined) {
        if (!isNonNegNum(motion.mass))
            throw new KhrValidationError(`Kinematic node "${name}" has invalid motion.mass`);
        mass = motion.mass;
    }
    const centerOfMass = motion.centerOfMass === undefined
        ? ([0, 0, 0] as [number, number, number])
        : readFiniteVec3(motion.centerOfMass, `${name}.motion.centerOfMass`);
    const inertiaOrientation = motion.inertiaOrientation === undefined
        ? ([0, 0, 0, 1] as [number, number, number, number])
        : readUnitQuat(motion.inertiaOrientation, `${name}.motion.inertiaOrientation`);
    let inertiaDiagonal: [number, number, number] = [0, 0, 0];
    const hasExplicitInertia = motion.inertiaDiagonal !== undefined;
    if (hasExplicitInertia) {
        inertiaDiagonal = readFiniteVec3(motion.inertiaDiagonal, `${name}.motion.inertiaDiagonal`);
        if (inertiaDiagonal.some((n) => (isKinematic ? n < 0 : n <= 0))) {
            throw new KhrValidationError(`Node "${name}" has invalid motion.inertiaDiagonal`);
        }
    }
    const linearVelocity = motion.linearVelocity === undefined ? undefined : readFiniteVec3(motion.linearVelocity, `${name}.motion.linearVelocity`);
    const angularVelocity = motion.angularVelocity === undefined ? undefined : readFiniteVec3(motion.angularVelocity, `${name}.motion.angularVelocity`);
    const gravityFactor = optFiniteNum(motion.gravityFactor, `${name}.motion.gravityFactor`);
    return {
        isKinematic,
        hasExplicitMotion: hasMotion,
        mass,
        centerOfMass,
        inertiaDiagonal,
        inertiaOrientation,
        hasExplicitInertia,
        ...(linearVelocity ? { linearVelocity } : {}),
        ...(angularVelocity ? { angularVelocity } : {}),
        ...(gravityFactor === undefined ? {} : { gravityFactor }),
    };
}

const axisVector = (axis: 0 | 1 | 2): [number, number, number] => (axis === 0 ? [1, 0, 0] : axis === 1 ? [0, 1, 0] : [0, 0, 1]);

function decodeJointDrives(rawDrives: unknown[], jointIdx: number, nodeName: string): KhrJointDriveSpec[] {
    return rawDrives.map((d, i) => {
        const label = `physicsJoints[${jointIdx}].drives[${i}] on "${nodeName}"`;
        if (!isObj(d))
            throw new KhrValidationError(`${label} must be an object`);
        if (d.mode !== "force" && d.mode !== "acceleration")
            throw new KhrValidationError(`${label} missing valid drive.mode`);
        if (d.axis !== 0 && d.axis !== 1 && d.axis !== 2)
            throw new KhrValidationError(`${label} missing valid drive.axis (0|1|2)`);
        if (d.type !== undefined && d.type !== "linear" && d.type !== "angular")
            throw new KhrValidationError(`${label} invalid drive.type`);
        const positionTarget = optFiniteNum(d.positionTarget, `${label} drive.positionTarget`);
        const velocityTarget = optFiniteNum(d.velocityTarget, `${label} drive.velocityTarget`);
        if (positionTarget === undefined && velocityTarget === undefined) {
            throw new KhrValidationError(`${label} requires finite drive.positionTarget or drive.velocityTarget`);
        }
        const stiffness = optFiniteNum(d.stiffness, `${label} drive.stiffness`, true);
        const damping = optFiniteNum(d.damping, `${label} drive.damping`, true);
        const maxForce = optFiniteNum(d.maxForce, `${label} drive.maxForce`, true);
        return {
            ...(d.type ? { type: d.type } : {}),
            mode: d.mode,
            axis: d.axis,
            ...(positionTarget === undefined ? {} : { positionTarget }),
            ...(velocityTarget === undefined ? {} : { velocityTarget }),
            ...(stiffness === undefined ? {} : { stiffness }),
            ...(damping === undefined ? {} : { damping }),
            ...(maxForce === undefined ? {} : { maxForce }),
        };
    });
}

type AxisState = "unlimited" | "locked" | [number, number];
type LimitGain = { stiffness?: number; damping?: number } | undefined;

function decodeJoint(
    rawJoints: Array<Record<string, unknown>>,
    jointIdx: number,
    nodeName: string,
    connectedNode: THREE.Object3D,
    childNode: THREE.Object3D,
    enableCollision: boolean,
    role: string | null,
    connectedNodeName: string = connectedNode.name,
): KhrJointSpec {
    const raw = rawJoints[jointIdx];
    if (!isObj(raw))
        throw new KhrValidationError(`physicsJoints[${jointIdx}] out of bounds for "${nodeName}"`);
    if (raw.limits !== undefined && !Array.isArray(raw.limits))
        throw new KhrValidationError(`physicsJoints[${jointIdx}].limits on "${nodeName}" must be an array`);
    if (raw.drives !== undefined && !Array.isArray(raw.drives))
        throw new KhrValidationError(`physicsJoints[${jointIdx}].drives on "${nodeName}" must be an array`);

    const rawLimits = (raw.limits ?? []) as unknown[];
    const rawDrives = (raw.drives ?? []) as unknown[];
    const lin: [AxisState, AxisState, AxisState] = ["unlimited", "unlimited", "unlimited"];
    const ang: [AxisState, AxisState, AxisState] = ["unlimited", "unlimited", "unlimited"];
    const linGain: [LimitGain, LimitGain, LimitGain] = [undefined, undefined, undefined];
    const angGain: [LimitGain, LimitGain, LimitGain] = [undefined, undefined, undefined];

    const applyAxes = (axesRaw: unknown, lim: Record<string, unknown>, state: AxisState[], gains: LimitGain[]): void => {
        if (axesRaw === undefined)
            return;
        if (!Array.isArray(axesRaw))
            throw new KhrValidationError(`physicsJoints[${jointIdx}].limits axes on "${nodeName}" must be an array`);
        const axes = axesRaw.map((ax) => {
            if (ax !== 0 && ax !== 1 && ax !== 2)
                throw new KhrValidationError(`physicsJoints[${jointIdx}].limits invalid axis ${String(ax)} on "${nodeName}"`);
            return ax;
        });
        const min = optFiniteNum(lim.min, `physicsJoints[${jointIdx}] limit.min on "${nodeName}"`);
        const max = optFiniteNum(lim.max, `physicsJoints[${jointIdx}] limit.max on "${nodeName}"`);
        if ((lim.min !== undefined || lim.max !== undefined) && (min === undefined || max === undefined)) {
            throw new KhrValidationError(`physicsJoints[${jointIdx}].limits on "${nodeName}" must specify both min and max`);
        }
        if (min !== undefined && max !== undefined && min > max) {
            throw new KhrValidationError(`physicsJoints[${jointIdx}] limit.min > limit.max on "${nodeName}"`);
        }
        const stiffness = optFiniteNum(lim.stiffness, `physicsJoints[${jointIdx}] limit.stiffness on "${nodeName}"`, true);
        const damping = optFiniteNum(lim.damping, `physicsJoints[${jointIdx}] limit.damping on "${nodeName}"`, true);
        const locked = min !== undefined && max !== undefined && max - min <= 1e-6;
        if (!locked && axes.length > 1) {
            throw new KhrValidationError(`physicsJoints[${jointIdx}] coupled multi-axis limit on "${nodeName}" is unsupported`);
        }
        for (const ax of axes) {
            state[ax] = locked ? "locked" : (min !== undefined && max !== undefined ? [min, max] : "unlimited");
            if (!locked && (stiffness !== undefined || damping !== undefined)) {
                gains[ax] = {
                    ...(stiffness === undefined ? {} : { stiffness }),
                    ...(damping === undefined ? {} : { damping }),
                };
            }
        }
    };

    for (const lim of rawLimits) {
        if (!isObj(lim))
            throw new KhrValidationError(`physicsJoints[${jointIdx}].limits entry on "${nodeName}" must be an object`);
        applyAxes(lim.linearAxes, lim, lin, linGain);
        applyAxes(lim.angularAxes, lim, ang, angGain);
    }

    const freeLin = ([0, 1, 2] as const).filter((ax) => lin[ax] !== "locked");
    const freeAng = ([0, 1, 2] as const).filter((ax) => ang[ax] !== "locked");
    let kind: KhrJointSpec["kind"];
    let axis: [number, number, number] = [0, 1, 0];
    let limits: [number, number] | null = null;
    let gain: LimitGain;

    if (freeLin.length === 0 && freeAng.length === 0) {
        kind = "fixed";
    }
    else if (freeLin.length === 0 && freeAng.length === 1) {
        kind = "revolute";
        axis = axisVector(freeAng[0]);
        const s = ang[freeAng[0]];
        limits = Array.isArray(s) ? s : null;
        gain = angGain[freeAng[0]];
    }
    else if (freeAng.length === 0 && freeLin.length === 1) {
        kind = "prismatic";
        axis = axisVector(freeLin[0]);
        const s = lin[freeLin[0]];
        limits = Array.isArray(s) ? s : null;
        gain = linGain[freeLin[0]];
    }
    else {
        throw new KhrValidationError(`physicsJoints[${jointIdx}] unsupported DOF (${freeLin.length} lin, ${freeAng.length} ang) on "${nodeName}"`);
    }

    const drives = decodeJointDrives(rawDrives, jointIdx, nodeName);
    if (
        isObj(raw.extras) &&
        Array.isArray(raw.extras.jointAxis) &&
        raw.extras.jointAxis.length === 3 &&
        raw.extras.jointAxis.every(isFiniteNum)
    ) {
        const [jx, jy, jz] = raw.extras.jointAxis as [number, number, number];
        const jLen = Math.hypot(jx, jy, jz);
        if (jLen > 1e-8)
            axis = [jx / jLen, jy / jLen, jz / jLen];
    }

    const anchor1Vec = childNode
        .getWorldPosition(new THREE.Vector3())
        .sub(connectedNode.getWorldPosition(new THREE.Vector3()))
        .applyQuaternion(connectedNode.getWorldQuaternion(new THREE.Quaternion()).invert());

    return {
        index: jointIdx,
        nodeName,
        connectedNodeName,
        kind,
        axis,
        limits,
        enableCollision,
        role,
        ...(gain?.stiffness === undefined ? {} : { limitStiffness: gain.stiffness }),
        ...(gain?.damping === undefined ? {} : { limitDamping: gain.damping }),
        drive: drives[0] ?? null,
        drives,
        anchor1: [anchor1Vec.x, anchor1Vec.y, anchor1Vec.z],
        anchor2: [0, 0, 0],
    };
}

const _quat = new THREE.Quaternion();
const _invParentQuat = new THREE.Quaternion();
const _localOrigin = new THREE.Vector3(0, 0, 0);

function getRelativeQuatArray(obj: THREE.Object3D, parent?: THREE.Object3D): [number, number, number, number] {
    obj.updateWorldMatrix(true, false);
    obj.getWorldQuaternion(_quat);
    if (parent) {
        parent.updateWorldMatrix(true, false);
        _quat.premultiply(parent.getWorldQuaternion(_invParentQuat).invert());
    }
    if (_quat.lengthSq() > 0)
        _quat.normalize();
    else
        _quat.identity();
    return [_quat.x, _quat.y, _quat.z, _quat.w];
}

/**
 * Resolves the physical hub shaft center of a wheel or propeller node:
 * uses the authored glTF node origin (`getWorldPosition`) whenever the node has a non-zero transform
 * or its local mesh bounds contain `(0, 0, 0)` (preventing 3-blade rotor / caliper AABB wobble),
 * falling back to mesh AABB center only when an uncentered mesh was exported with its node left at `(0, 0, 0)`.
 */
function resolveRotorHubPosition(obj: THREE.Object3D): [number, number, number] {
    const wp = obj.getWorldPosition(new THREE.Vector3());
    if (wp.lengthSq() > 1e-8)
        return [wp.x, wp.y, wp.z];

    const invObj = obj.matrixWorld.clone().invert();
    const rel = new THREE.Matrix4();
    const v = new THREE.Vector3();
    const localBox = new THREE.Box3();
    obj.traverse((child) => {
        const mesh = child as THREE.Mesh;
        if (!mesh.isMesh || !mesh.geometry)
            return;
        const pos = mesh.geometry.getAttribute("position");
        if (!pos)
            return;
        rel.multiplyMatrices(invObj, mesh.matrixWorld);
        for (let i = 0; i < pos.count; i++)
            localBox.expandByPoint(v.fromBufferAttribute(pos, i).applyMatrix4(rel));
    });
    if (localBox.isEmpty() || localBox.containsPoint(_localOrigin))
        return [wp.x, wp.y, wp.z];

    const worldBox = new THREE.Box3().setFromObject(obj);
    const c = worldBox.getCenter(new THREE.Vector3());
    return [c.x, c.y, c.z];
}

function optRotorNum(obj: Record<string, unknown>, key: string, tagKind: string, owner: string, strictlyPos = false): number | undefined {
    const val = obj[key];
    if (val === undefined)
        return undefined;
    if (!isFiniteNum(val) || (strictlyPos ? val <= 0 : val < 0)) {
        throw new KhrValidationError(`${tagKind} "${owner}" has invalid ${tagKind}.${key}`);
    }
    return val;
}

function reqRotorNum(obj: Record<string, unknown>, key: string, tagKind: string, owner: string, strictlyPos = false): number {
    const val = obj[key];
    if (!isFiniteNum(val) || (strictlyPos ? val <= 0 : val < 0)) {
        throw new KhrValidationError(`${tagKind} "${owner}" is missing valid ${tagKind}.${key}`);
    }
    return val;
}

function optRotorBool(obj: Record<string, unknown>, key: string, tagKind: string, owner: string): boolean | undefined {
    const val = obj[key];
    if (val === undefined)
        return undefined;
    if (typeof val !== "boolean")
        throw new KhrValidationError(`${tagKind} "${owner}" has non-boolean ${tagKind}.${key}`);
    return val;
}

function reqRotorBool(obj: Record<string, unknown>, key: string, tagKind: string, owner: string): boolean {
    const val = obj[key];
    if (typeof val !== "boolean")
        throw new KhrValidationError(`${tagKind} "${owner}" is missing boolean ${tagKind}.${key}`);
    return val;
}

function optBoolOrNumFlag(
    obj: Record<string, unknown>,
    key: string,
    tagKind: string,
    owner: string,
): { flag: boolean; numeric?: number } | undefined {
    const val = obj[key];
    if (val === undefined)
        return undefined;
    if (typeof val === "boolean")
        return { flag: val };
    if (isNonNegNum(val))
        return { flag: val !== 0, numeric: val };
    throw new KhrValidationError(`${tagKind} "${owner}" has invalid ${tagKind}.${key}`);
}

function optCurve(val: unknown, key: string, owner: string): KhrCurveData | undefined {
    if (val === undefined)
        return undefined;
    if (!isObj(val) || !Array.isArray(val.points) || val.points.length === 0) {
        throw new KhrValidationError(`ecctrlWheel "${owner}" has invalid ecctrlWheel.${key}`);
    }
    for (const pt of val.points) {
        if (!isObj(pt) || !isFiniteNum(pt.x) || !isFiniteNum(pt.y)) {
            throw new KhrValidationError(`ecctrlWheel "${owner}" has invalid point in ecctrlWheel.${key}`);
        }
    }
    if (val.samples !== undefined && !isPosNum(val.samples)) {
        throw new KhrValidationError(`ecctrlWheel "${owner}" has invalid samples in ecctrlWheel.${key}`);
    }
    return val as unknown as KhrCurveData;
}

function decodeWheelTag(
    obj: THREE.Object3D,
    name: string,
    rawWheel: unknown,
    scene?: THREE.Object3D,
    rawOmiWheel?: unknown,
): EcctrlWheelTag {
    assertIdentityWorldScale(obj, name);
    const hasOmi = rawOmiWheel !== undefined;
    if (hasOmi && !isObj(rawOmiWheel))
        throw new KhrValidationError(`OMI_vehicle_wheel "${name}" must be an object`);
    if (rawWheel !== undefined && !isObj(rawWheel))
        throw new KhrValidationError(`ecctrlWheel "${name}" must be an object`);
    if (!hasOmi && rawWheel === undefined)
        throw new KhrValidationError(`ecctrlWheel "${name}" must be an object`);

    const ec = isObj(rawWheel) ? rawWheel : {};
    const omi = isObj(rawOmiWheel) ? rawOmiWheel : undefined;

    const omiRadius = omi ? optRotorNum(omi, "radius", "OMI_vehicle_wheel", name, true) : undefined;
    const omiWidth = omi ? optRotorNum(omi, "width", "OMI_vehicle_wheel", name, true) : undefined;
    const radius = optRotorNum(ec, "radius", "ecctrlWheel", name, true) ?? omiRadius;
    const width = optRotorNum(ec, "width", "ecctrlWheel", name, true) ?? omiWidth;
    if (radius === undefined)
        throw new KhrValidationError(`Wheel "${name}" is missing positive finite radius`);
    if (width === undefined)
        throw new KhrValidationError(`Wheel "${name}" is missing positive finite width`);

    const omiTravel = omi ? optRotorNum(omi, "suspensionTravel", "OMI_vehicle_wheel", name) : undefined;
    const omiMaxForce = omi ? optRotorNum(omi, "maxForce", "OMI_vehicle_wheel", name) : undefined;
    const omiMaxSteerRaw = omi ? optRotorNum(omi, "maxSteeringAngle", "OMI_vehicle_wheel", name) : undefined;
    const omiStiffness = omi ? optRotorNum(omi, "suspensionStiffness", "OMI_vehicle_wheel", name) : undefined;
    const omiDampComp = omi ? optRotorNum(omi, "suspensionDampingCompression", "OMI_vehicle_wheel", name) : undefined;
    const omiDampReb = omi ? optRotorNum(omi, "suspensionDampingRebound", "OMI_vehicle_wheel", name) : undefined;
    const omiSteer = omi ? optBoolOrNumFlag(omi, "steering", "OMI_vehicle_wheel", name) : undefined;
    const omiProp = omi ? optBoolOrNumFlag(omi, "propulsion", "OMI_vehicle_wheel", name) : undefined;
    const omiMaxSteer = omiMaxSteerRaw ?? (omiSteer?.numeric !== undefined && omiSteer.numeric > 0 ? omiSteer.numeric : undefined);

    const travel = omi ? optRotorNum(ec, "travel", "ecctrlWheel", name) ?? omiTravel ?? 0.25 : reqRotorNum(ec, "travel", "ecctrlWheel", name);
    const sag = omi
        ? optRotorNum(ec, "sag", "ecctrlWheel", name) ?? (travel > 0 ? Math.min(travel * 0.4, 0.1) : 0)
        : reqRotorNum(ec, "sag", "ecctrlWheel", name);
    if (sag > travel) {
        throw new KhrValidationError(`ecctrlWheel "${name}" has invalid sag (${sag}) > travel (${travel})`);
    }
    const steer = omi
        ? optRotorBool(ec, "steer", "ecctrlWheel", name) ?? omiSteer?.flag ?? (omiMaxSteer !== undefined ? omiMaxSteer > 0 : false)
        : reqRotorBool(ec, "steer", "ecctrlWheel", name);
    const drive = omi
        ? optRotorBool(ec, "drive", "ecctrlWheel", name) ?? omiProp?.flag ?? false
        : reqRotorBool(ec, "drive", "ecctrlWheel", name);
    const brake = omi
        ? optRotorBool(ec, "brake", "ecctrlWheel", name) ?? true
        : reqRotorBool(ec, "brake", "ecctrlWheel", name);

    const omiDamping = omiDampComp !== undefined && omiDampReb !== undefined
        ? 0.5 * (omiDampComp + omiDampReb)
        : (omiDampComp ?? omiDampReb);
    const springK = optRotorNum(ec, "springK", "ecctrlWheel", name) ?? omiStiffness;
    const dampingC = optRotorNum(ec, "dampingC", "ecctrlWheel", name) ?? omiDamping;
    const maxBrakeTorque = optRotorNum(ec, "maxBrakeTorque", "ecctrlWheel", name)
        ?? (omiMaxForce !== undefined ? omiMaxForce * radius : undefined);
    const maxSteeringAngle = optRotorNum(ec, "maxSteeringAngle", "ecctrlWheel", name) ?? omiMaxSteer;

    return {
        nodeName: name,
        object: obj,
        position: resolveRotorHubPosition(obj),
        quaternion: getRelativeQuatArray(obj, scene),
        radius,
        width,
        steer,
        steerInvert: optRotorBool(ec, "steerInvert", "ecctrlWheel", name),
        drive,
        driveInvert: optRotorBool(ec, "driveInvert", "ecctrlWheel", name),
        brake,
        travel,
        sag,
        springK,
        dampingC,
        maxBrakeTorque,
        ...(maxSteeringAngle === undefined ? {} : { maxSteeringAngle }),
        rollingResistanceCoef: optRotorNum(ec, "rollingResistanceCoef", "ecctrlWheel", name),
        wheelDensity: optRotorNum(ec, "wheelDensity", "ecctrlWheel", name, true),
        driveTorqueWeight: optRotorNum(ec, "driveTorqueWeight", "ecctrlWheel", name),
        tireGripFactor: optRotorNum(ec, "tireGripFactor", "ecctrlWheel", name),
        latFrictionEllipseScale: optRotorNum(ec, "latFrictionEllipseScale", "ecctrlWheel", name),
        relaxLngRate: optRotorNum(ec, "relaxLngRate", "ecctrlWheel", name),
        relaxLatRate: optRotorNum(ec, "relaxLatRate", "ecctrlWheel", name),
        lngSlipRatioCurveData: optCurve(ec.lngSlipRatioCurveData, "lngSlipRatioCurveData", name),
        latSlipRatioCurveData: optCurve(ec.latSlipRatioCurveData, "latSlipRatioCurveData", name),
    };
}

function decodePropellerTag(
    obj: THREE.Object3D,
    name: string,
    rawPropeller: unknown,
    scene?: THREE.Object3D,
): EcctrlPropellerTag {
    assertIdentityWorldScale(obj, name);
    if (!isObj(rawPropeller))
        throw new KhrValidationError(`ecctrlPropeller "${name}" must be an object`);
    if (!isPosNum(rawPropeller.maxThrust))
        throw new KhrValidationError(`ecctrlPropeller "${name}" is missing positive finite ecctrlPropeller.maxThrust`);
    const torqueRatio = optFiniteNum(rawPropeller.torqueRatio, `ecctrlPropeller "${name}".torqueRatio`);
    return {
        nodeName: name,
        object: obj,
        position: resolveRotorHubPosition(obj),
        quaternion: getRelativeQuatArray(obj, scene),
        maxThrust: rawPropeller.maxThrust,
        torqueRatio,
        invertThrust: optRotorBool(rawPropeller, "invertThrust", "ecctrlPropeller", name),
        invertTorque: optRotorBool(rawPropeller, "invertTorque", "ecctrlPropeller", name),
    };
}

function decodeBreakawaySpec(raw: unknown): KhrBreakawaySpec | undefined {
    if (!isObj(raw) || !isPosNum(raw.breakImpulseNs))
        return undefined;
    const latch = isPosNum(raw.latchImpulseNs) ? raw.latchImpulseNs : undefined;
    return { ...(latch === undefined ? {} : { latchImpulseNs: latch }), breakImpulseNs: raw.breakImpulseNs };
}

interface KhrNodeMeta {
    nodeName: string;
    extras: Record<string, unknown>;
    worldPosition: [number, number, number];
    contactDots?: Array<[string, [number, number, number]]>;
}

interface KhrTemplateMeta {
    nodes: KhrNodeMeta[];
    bodies: Array<{ nodeName: string; extras: Record<string, unknown> }>;
    ikSockets: Array<[string, [number, number, number]]>;
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

const _dotVec = new THREE.Vector3();
const _dotQuat = new THREE.Quaternion();

function collectNodeSockets(
    obj: THREE.Object3D,
    name: string,
    extras: Record<string, unknown>,
    sockets: Record<string, [number, number, number]>,
    isSocketNode?: KhrPluginOptions["isSocketNode"],
): KhrNodeMeta {
    if (extras.socketId !== undefined && (typeof extras.socketId !== "string" || extras.socketId.length === 0)) {
        throw new KhrValidationError(`Node "${name}" has invalid extras.socketId`);
    }
    const wp = obj.getWorldPosition(new THREE.Vector3());
    const worldPosition: [number, number, number] = [wp.x, wp.y, wp.z];
    const custom = isSocketNode?.(name, extras);
    if (custom || extras.isIkAnchor === true || typeof extras.socketId === "string") {
        const key = typeof custom === "string" && custom.length > 0
            ? custom
            : (typeof extras.socketId === "string" ? extras.socketId : name);
        sockets[key] = [...worldPosition];
    }
    let contactDots: Array<[string, [number, number, number]]> | undefined;
    if (extras.contactDots !== undefined) {
        if (!Array.isArray(extras.contactDots)) {
            throw new KhrValidationError(`Node "${name}" has invalid extras.contactDots (expected array)`);
        }
        obj.getWorldQuaternion(_dotQuat);
        contactDots = [];
        for (const dot of extras.contactDots) {
            if (
                !isObj(dot) ||
                typeof dot.role !== "string" ||
                dot.role.length === 0 ||
                !Array.isArray(dot.pos) ||
                dot.pos.length !== 3 ||
                !dot.pos.every(isFiniteNum)
            ) {
                throw new KhrValidationError(`Node "${name}" has invalid entry in extras.contactDots`);
            }
            _dotVec.set(dot.pos[0], dot.pos[1], dot.pos[2]).applyQuaternion(_dotQuat).add(wp);
            const dotPos: [number, number, number] = [_dotVec.x, _dotVec.y, _dotVec.z];
            sockets[dot.role] = dotPos;
            contactDots.push([dot.role, dotPos]);
        }
    }
    return {
        nodeName: name,
        extras,
        worldPosition,
        ...(contactDots ? { contactDots } : {}),
    };
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
    const explicitInertiaSet = new Set<number>();
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

    for (const [idx, body] of bodyByIdx) {
        if (!explicitInertiaSet.has(idx) && !body.isKinematic) {
            body.inertiaDiagonal = computeBodyInertia(body.colliders, body.mass, body.centerOfMass, body.inertiaOrientation, body.nodeName);
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
