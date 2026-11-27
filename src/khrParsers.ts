import {
    assertIdentityWorldScale,
    assertSceneIdentityWorldScale,
    computeBodyInertia,
    computeRelativePose,
    computeShapeInertia,
    extractMeshColliderShape,
    readFiniteVec3,
    readUnitQuat,
} from "./khrInertia.js";
import { type KhrBodySpec, KhrValidationError } from "./khrTypes.js";

export { GLB_BIN_CHUNK, GLB_JSON_CHUNK, GLB_MAGIC, parseGlbJson, validateGlbHeader, type RawGltfJson } from "./khrGlb.js";
export { assertIdentityWorldScale, assertSceneIdentityWorldScale, computeBodyInertia, computeRelativePose, computeShapeInertia, extractMeshColliderShape };
export { decodeJoint } from "./khrJointParser.js";
export { decodePhysicsMaterial, decodeShape } from "./khrShapeParser.js";

const isPos = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v > 0;
const isNonNeg = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v >= 0;

function optNum(v: unknown, label: string, nonNeg = false): number | undefined {
    if (v !== undefined) {
        if (typeof v !== "number" || !Number.isFinite(v) || (nonNeg && v < 0)) {
            throw new KhrValidationError(`Invalid ${label}`);
        }
        return v;
    }
    return undefined;
}

export function decodeMotion(
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
        if (!isPos(motion.mass))
            throw new KhrValidationError(`Dynamic node "${name}" is missing positive finite motion.mass`);
        mass = motion.mass;
    }
    else if (motion.mass !== undefined) {
        if (!isNonNeg(motion.mass))
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
    const gravityFactor = optNum(motion.gravityFactor, `${name}.motion.gravityFactor`);
    return {
        isKinematic,
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
        if (!f || typeof f !== "object" || Array.isArray(f)) {
            throw new KhrValidationError(`collisionFilters[${idx}] must be an object`);
        }
        if (f.collideWithSystems !== undefined && f.notCollideWithSystems !== undefined) {
            throw new KhrValidationError(`collisionFilters[${idx}] cannot specify both collideWithSystems and notCollideWithSystems`);
        }
        const membership = maskFor(f.collisionSystems, "collisionSystems");
        const filter = f.collideWithSystems === undefined
            ? 65535 & ~maskFor(f.notCollideWithSystems, "notCollideWithSystems")
            : maskFor(f.collideWithSystems, "collideWithSystems");
        return ((membership & 65535) << 16) | (filter & 65535);
    });
}
