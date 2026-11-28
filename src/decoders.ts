import * as THREE from "three";
import {
    assertIdentityWorldScale,
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
    type KhrBodySpec,
    type KhrBreakawaySpec,
    type KhrCombineMode,
    type KhrCurveData,
    type KhrEmitterSpec,
    type KhrImplicitShape,
    type KhrJointDriveSpec,
    type KhrJointSpec,
    type KhrPluginOptions,
    KhrValidationError,
    optFiniteNum,
} from "./types.js";

export interface KhrNodeMeta {
    nodeName: string;
    extras: Record<string, unknown>;
    worldPosition: [number, number, number];
    emitterSocket?: string;
    inferredSocket?: string;
    contactDots?: Array<[string, [number, number, number]]>;
}

export interface KhrTemplateMeta {
    nodes: KhrNodeMeta[];
    bodies: Array<{ nodeName: string; extras: Record<string, unknown> }>;
    ikSockets: Array<[string, [number, number, number]]>;
}

const _quat = new THREE.Quaternion();
const _invParentQuat = new THREE.Quaternion();
const _localOrigin = new THREE.Vector3(0, 0, 0);
const _wpA = new THREE.Vector3();
const _wpB = new THREE.Vector3();
const _wqInv = new THREE.Quaternion();
const _dotVec = new THREE.Vector3();
const _dotQuat = new THREE.Quaternion();

export function decodeShape(raw: Record<string, unknown>, idx: number): KhrImplicitShape {
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

export function decodePhysicsMaterial(
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

export function decodeCollisionGroups(rawFilters: Array<Record<string, unknown>>): number[] {
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

export function decodeMotion(
    motion: Record<string, unknown>,
    hasMotion: boolean,
    name: string,
): Omit<KhrBodySpec, "nodeName" | "localPosition" | "localQuaternion" | "colliders" | "joint" | "object" | "role"> {
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
    const hasExplicitCom = motion.centerOfMass !== undefined;
    const centerOfMass = !hasExplicitCom
        ? ([0, 0, 0] as [number, number, number])
        : readFiniteVec3(motion.centerOfMass, `${name}.motion.centerOfMass`);
    const hasExplicitInertiaOrientation = motion.inertiaOrientation !== undefined;
    const inertiaOrientation = !hasExplicitInertiaOrientation
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
        hasExplicitCom,
        hasExplicitInertia,
        hasExplicitInertiaOrientation,
        mass,
        centerOfMass,
        inertiaDiagonal,
        inertiaOrientation,
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

export function decodeJoint(
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
        .getWorldPosition(_wpA)
        .sub(connectedNode.getWorldPosition(_wpB))
        .applyQuaternion(connectedNode.getWorldQuaternion(_wqInv).invert());

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
    obj.getWorldPosition(_wpA);
    if (_wpA.lengthSq() > 1e-8)
        return [_wpA.x, _wpA.y, _wpA.z];

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
        return [_wpA.x, _wpA.y, _wpA.z];

    const worldBox = new THREE.Box3().setFromObject(obj);
    const c = worldBox.getCenter(_wpB);
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

export function decodeWheelTag(
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

export function decodePropellerTag(
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

export function decodeBreakawaySpec(raw: unknown): KhrBreakawaySpec | undefined {
    if (!isObj(raw) || !isPosNum(raw.breakImpulseNs))
        return undefined;
    const latch = isPosNum(raw.latchImpulseNs) ? raw.latchImpulseNs : undefined;
    return { ...(latch === undefined ? {} : { latchImpulseNs: latch }), breakImpulseNs: raw.breakImpulseNs };
}

const SOCKET_PREFIX_RE = /^(?:socket|ik)_(.+)$/i;
const SEAT_NODE_RE = /^seat(?:_[lr])?$/i;
const _seatBox = new THREE.Box3();
const _seatCenter = new THREE.Vector3();

function resolveNodePivotOrMeshCenter(obj: THREE.Object3D): [number, number, number] {
    obj.getWorldPosition(_wpA);
    if (_wpA.lengthSq() > 1e-8)
        return [_wpA.x, _wpA.y, _wpA.z];
    _seatBox.makeEmpty();
    obj.traverse((child) => {
        const mesh = child as THREE.Mesh;
        if (mesh.isMesh && mesh.geometry)
            _seatBox.expandByObject(mesh);
    });
    if (!_seatBox.isEmpty()) {
        _seatBox.getCenter(_seatCenter);
        return [_seatCenter.x, _seatCenter.y, _seatCenter.z];
    }
    return [_wpA.x, _wpA.y, _wpA.z];
}

export function extractDefaultBodyRole(extras: Record<string, unknown>): string | null {
    for (const key of ["role", "vehicleRole", "buildingRole", "weaponRole"] as const) {
        const val = extras[key];
        if (typeof val === "string" && val.length > 0)
            return val;
    }
    return null;
}

export function collectNodeSockets(
    obj: THREE.Object3D,
    name: string,
    extras: Record<string, unknown>,
    sockets: Record<string, [number, number, number]>,
    emitters: KhrEmitterSpec[],
    scene?: THREE.Object3D,
    rawExtensions?: Record<string, unknown>,
    isSocketNode?: KhrPluginOptions["isSocketNode"],
): KhrNodeMeta {
    if (extras.socketId !== undefined && (typeof extras.socketId !== "string" || extras.socketId.length === 0)) {
        throw new KhrValidationError(`Node "${name}" has invalid extras.socketId`);
    }
    obj.getWorldPosition(_wpA);
    const worldPosition: [number, number, number] = [_wpA.x, _wpA.y, _wpA.z];

    let emitterSocket: string | undefined;
    const rawEmitter = rawExtensions?.KOLMO_emitter;
    if (rawEmitter !== undefined) {
        if (!isObj(rawEmitter))
            throw new KhrValidationError(`Node "${name}" has invalid extensions.KOLMO_emitter`);
        if (rawEmitter.socket !== undefined) {
            if (typeof rawEmitter.socket !== "string" || rawEmitter.socket.length === 0) {
                throw new KhrValidationError(`Node "${name}" has invalid KOLMO_emitter.socket`);
            }
            emitterSocket = rawEmitter.socket;
            sockets[emitterSocket] = [...worldPosition];
            sockets[name] = [...worldPosition];
        }
        if (rawEmitter.kind !== undefined) {
            if (typeof rawEmitter.kind !== "string" || rawEmitter.kind.length === 0) {
                throw new KhrValidationError(`Node "${name}" has invalid KOLMO_emitter.kind`);
            }
            if (rawEmitter.drive !== undefined && (typeof rawEmitter.drive !== "string" || rawEmitter.drive.length === 0)) {
                throw new KhrValidationError(`Node "${name}" has invalid KOLMO_emitter.drive`);
            }
            let color: [number, number, number] | undefined;
            if (rawEmitter.color !== undefined) {
                color = readFiniteVec3(rawEmitter.color, `${name}.KOLMO_emitter.color`);
            }
            const size = optFiniteNum(rawEmitter.size, `${name}.KOLMO_emitter.size`, true);
            emitters.push({
                nodeName: name,
                kind: rawEmitter.kind,
                ...(typeof rawEmitter.drive === "string" ? { drive: rawEmitter.drive } : {}),
                ...(color ? { color } : {}),
                ...(size === undefined ? {} : { size }),
                position: [...worldPosition],
                quaternion: getRelativeQuatArray(obj, scene),
                object: obj,
            });
        }
    }

    let inferredSocket: string | undefined;
    const custom = isSocketNode?.(name, extras);
    if (custom || extras.isIkAnchor === true || typeof extras.socketId === "string") {
        const key = typeof custom === "string" && custom.length > 0
            ? custom
            : (typeof extras.socketId === "string" ? extras.socketId : name);
        sockets[key] = [...worldPosition];
    }
    else if (!emitterSocket) {
        const prefixMatch = SOCKET_PREFIX_RE.exec(name);
        if (prefixMatch) {
            inferredSocket = prefixMatch[1].toLowerCase();
            sockets[inferredSocket] ??= [...worldPosition];
            sockets[name] ??= [...worldPosition];
        }
        else if (SEAT_NODE_RE.test(name)) {
            const seatPos = resolveNodePivotOrMeshCenter(obj);
            sockets[name] = seatPos;
            if (/^seat(?:_l)?$/i.test(name))
                sockets.seat ??= [...seatPos];
            inferredSocket = name;
        }
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
            _dotVec.set(dot.pos[0], dot.pos[1], dot.pos[2]).applyQuaternion(_dotQuat).add(_wpA);
            const dotPos: [number, number, number] = [_dotVec.x, _dotVec.y, _dotVec.z];
            sockets[dot.role] = dotPos;
            contactDots.push([dot.role, dotPos]);
        }
    }
    return {
        nodeName: name,
        extras,
        worldPosition,
        ...(emitterSocket ? { emitterSocket } : {}),
        ...(inferredSocket ? { inferredSocket } : {}),
        ...(contactDots ? { contactDots } : {}),
    };
}
