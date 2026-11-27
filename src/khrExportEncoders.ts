import { readFiniteVec3, readUnitQuat } from "./khrInertia.js";
import {
    type KhrCombineMode,
    type KhrExportNodeSpec,
    type KhrImplicitShape,
    type KhrJointDriveSpec,
    KhrValidationError,
    type OmiVehicleWheelSpec,
} from "./khrTypes.js";

const isPosNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v > 0;
const isObj = (v: unknown): v is Record<string, unknown> => typeof v === "object" && !!v && !Array.isArray(v);
const COMBINE_MODES = new Set<KhrCombineMode>(["average", "minimum", "maximum", "multiply"]);

export function optFinite(v: unknown, label: string, nonNeg = false): number | undefined {
    if (v !== undefined) {
        if (typeof v !== "number" || !Number.isFinite(v) || (nonNeg && v < 0)) {
            throw new KhrValidationError(`exportKhrGlb: ${label} must be finite${nonNeg ? " >= 0" : ""}`);
        }
        return v;
    }
    return undefined;
}

export function encodeShape(shape: KhrImplicitShape, nodeName: string): Record<string, unknown> {
    if (shape.type === "box") {
        if (!Array.isArray(shape.size) || shape.size.length !== 3 || !shape.size.every(isPosNum)) {
            throw new KhrValidationError(`exportKhrGlb: invalid box.size on "${nodeName}"`);
        }
        return { type: "box", box: { size: shape.size } };
    }
    if (shape.type === "sphere") {
        if (!isPosNum(shape.radius)) {
            throw new KhrValidationError(`exportKhrGlb: invalid sphere.radius on "${nodeName}"`);
        }
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
    if (!Array.isArray(axis) || axis.length !== 3 || !axis.every((n) => typeof n === "number" && Number.isFinite(n))) {
        throw new KhrValidationError(`exportKhrGlb: ${kind} joint on "${nodeName}" requires a finite axis`);
    }
    const len = Math.hypot(axis[0], axis[1], axis[2]);
    if (len <= 1e-8) {
        throw new KhrValidationError(`exportKhrGlb: ${kind} joint on "${nodeName}" requires non-zero axis`);
    }
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
    if (drive.type !== undefined && drive.type !== "linear" && drive.type !== "angular") {
        throw new KhrValidationError(`exportKhrGlb: joint drive on "${nodeName}" has invalid drive.type`);
    }
    if (drive.axis !== undefined && drive.axis !== 0 && drive.axis !== 1 && drive.axis !== 2) {
        throw new KhrValidationError(`exportKhrGlb: joint drive on "${nodeName}" has invalid drive.axis`);
    }
    const positionTarget = optFinite(drive.positionTarget, `joint drive.positionTarget on "${nodeName}"`);
    const velocityTarget = optFinite(drive.velocityTarget, `joint drive.velocityTarget on "${nodeName}"`);
    if (positionTarget === undefined && velocityTarget === undefined) {
        throw new KhrValidationError(`exportKhrGlb: joint drive on "${nodeName}" requires finite positionTarget or velocityTarget`);
    }
    const stiffness = optFinite(drive.stiffness, `joint drive.stiffness on "${nodeName}"`, true);
    const damping = optFinite(drive.damping, `joint drive.damping on "${nodeName}"`, true);
    const maxForce = optFinite(drive.maxForce, `joint drive.maxForce on "${nodeName}"`, true);
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

export function encodeJoint(j: NonNullable<KhrExportNodeSpec["joint"]>, nodeName: string): Record<string, unknown> {
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
    const stiffness = optFinite(j.limitStiffness, `joint.limitStiffness on "${nodeName}"`, true);
    const damping = optFinite(j.limitDamping, `joint.limitDamping on "${nodeName}"`, true);
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

export function encodeMotion(m: NonNullable<KhrExportNodeSpec["motion"]>, nodeName: string): Record<string, unknown> {
    const isKinematic = Boolean(m.isKinematic);
    if (!isKinematic && !isPosNum(m.mass)) {
        throw new KhrValidationError(`exportKhrGlb: dynamic node "${nodeName}" requires positive finite motion.mass`);
    }
    if (isKinematic && m.mass !== undefined)
        optFinite(m.mass, `kinematic motion.mass on "${nodeName}"`, true);
    if (m.inertiaDiagonal && readFiniteVec3(m.inertiaDiagonal, `${nodeName}.motion.inertiaDiagonal`).some((n) => (isKinematic ? n < 0 : n <= 0))) {
        throw new KhrValidationError(`exportKhrGlb: invalid motion.inertiaDiagonal on "${nodeName}"`);
    }
    if (m.gravityFactor !== undefined && (typeof m.gravityFactor !== "number" || !Number.isFinite(m.gravityFactor))) {
        throw new KhrValidationError(`exportKhrGlb: invalid motion.gravityFactor on "${nodeName}"`);
    }
    return {
        isKinematic,
        ...(m.mass === undefined ? {} : { mass: m.mass }),
        ...(m.centerOfMass ? { centerOfMass: readFiniteVec3(m.centerOfMass, `${nodeName}.motion.centerOfMass`) } : {}),
        ...(m.inertiaDiagonal ? { inertiaDiagonal: m.inertiaDiagonal } : {}),
        ...(m.inertiaOrientation ? { inertiaOrientation: readUnitQuat(m.inertiaOrientation, `${nodeName}.motion.inertiaOrientation`) } : {}),
        ...(m.linearVelocity ? { linearVelocity: readFiniteVec3(m.linearVelocity, `${nodeName}.motion.linearVelocity`) } : {}),
        ...(m.angularVelocity ? { angularVelocity: readFiniteVec3(m.angularVelocity, `${nodeName}.motion.angularVelocity`) } : {}),
        ...(m.gravityFactor === undefined ? {} : { gravityFactor: m.gravityFactor }),
    };
}

export function encodePhysicsMaterial(col: NonNullable<KhrExportNodeSpec["collider"]>, nodeName: string): Record<string, unknown> | null {
    const friction = optFinite(col.friction, `collider.friction on "${nodeName}"`, true);
    const restitution = optFinite(col.restitution, `collider.restitution on "${nodeName}"`, true);
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

function validateSystemList(arr: unknown, label: string, nodeName: string): string[] {
    if (!Array.isArray(arr) || arr.some((s) => typeof s !== "string" || s.length === 0)) {
        throw new KhrValidationError(`exportKhrGlb: collider.${label} on "${nodeName}" must be an array of non-empty strings`);
    }
    return arr;
}

export function encodeCollisionFilter(col: NonNullable<KhrExportNodeSpec["collider"]>, nodeName: string): Record<string, unknown> | null {
    if (col.collideWithSystems !== undefined && col.notCollideWithSystems !== undefined) {
        throw new KhrValidationError(`exportKhrGlb: collider on "${nodeName}" cannot specify both collideWithSystems and notCollideWithSystems`);
    }
    if (col.collisionSystems === undefined && col.collideWithSystems === undefined && col.notCollideWithSystems === undefined) {
        return null;
    }
    return {
        ...(col.collisionSystems === undefined ? {} : { collisionSystems: validateSystemList(col.collisionSystems, "collisionSystems", nodeName) }),
        ...(col.collideWithSystems === undefined ? {} : { collideWithSystems: validateSystemList(col.collideWithSystems, "collideWithSystems", nodeName) }),
        ...(col.notCollideWithSystems === undefined ? {} : { notCollideWithSystems: validateSystemList(col.notCollideWithSystems, "notCollideWithSystems", nodeName) }),
    };
}

export function encodeOmiWheel(wheel: OmiVehicleWheelSpec, nodeName: string): Record<string, unknown> {
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
    const maxForce = optFinite(wheel.maxForce, `wheel.maxForce on "${nodeName}"`, true);
    const maxSteeringAngle = optFinite(wheel.maxSteeringAngle, `wheel.maxSteeringAngle on "${nodeName}"`, true);
    const suspensionStiffness = optFinite(wheel.suspensionStiffness, `wheel.suspensionStiffness on "${nodeName}"`, true);
    const suspensionDampingCompression = optFinite(wheel.suspensionDampingCompression, `wheel.suspensionDampingCompression on "${nodeName}"`, true);
    const suspensionDampingRebound = optFinite(wheel.suspensionDampingRebound, `wheel.suspensionDampingRebound on "${nodeName}"`, true);
    const suspensionTravel = optFinite(wheel.suspensionTravel, `wheel.suspensionTravel on "${nodeName}"`, true);
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
