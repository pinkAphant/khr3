import { type KhrCombineMode, type KhrImplicitShape, KhrValidationError } from "./khrTypes.js";

const isObj = (v: unknown): v is Record<string, unknown> => typeof v === "object" && !!v && !Array.isArray(v);
const isPos = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v > 0;
const COMBINE_MODES = new Set<KhrCombineMode>(["average", "minimum", "maximum", "multiply"]);

function optNum(v: unknown, label: string, nonNeg = false): number | undefined {
    if (v !== undefined) {
        if (typeof v !== "number" || !Number.isFinite(v) || (nonNeg && v < 0)) {
            throw new KhrValidationError(`Invalid ${label}`);
        }
        return v;
    }
    return undefined;
}

function optCombineMode(v: unknown, label: string): KhrCombineMode | undefined {
    if (v === undefined)
        return undefined;
    if (typeof v !== "string" || !COMBINE_MODES.has(v as KhrCombineMode)) {
        throw new KhrValidationError(`Invalid ${label}: "${String(v)}"`);
    }
    return v as KhrCombineMode;
}

export function decodeShape(raw: Record<string, unknown>, idx: number): KhrImplicitShape {
    if (!isObj(raw))
        throw new KhrValidationError(`KHR_implicit_shapes[${idx}] must be an object`);
    const type = raw.type;
    if (type === "box") {
        if (raw.box !== undefined && !isObj(raw.box))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid box`);
        const box = isObj(raw.box) ? raw.box : {};
        const size = box.size === undefined ? [1, 1, 1] : box.size;
        if (!Array.isArray(size) || size.length !== 3 || !size.every(isPos)) {
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid box.size`);
        }
        return { type: "box", size: [size[0], size[1], size[2]] };
    }
    if (type === "sphere") {
        if (raw.sphere !== undefined && !isObj(raw.sphere))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid sphere`);
        const sphere = isObj(raw.sphere) ? raw.sphere : {};
        const radius = sphere.radius === undefined ? 0.5 : sphere.radius;
        if (!isPos(radius))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid sphere.radius`);
        return { type: "sphere", radius };
    }
    if (type === "cylinder" || type === "capsule") {
        if (raw[type] !== undefined && !isObj(raw[type]))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}`);
        const g = isObj(raw[type]) ? (raw[type] as Record<string, unknown>) : {};
        const height = g.height === undefined ? (type === "cylinder" ? 2.0 : 1.0) : g.height;
        if (!isPos(height))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}.height`);
        if (g.radius !== undefined && !isPos(g.radius))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}.radius`);
        const baseR = g.radius === undefined ? 0.5 : g.radius;
        const radiusTop = g.radiusTop === undefined ? baseR : g.radiusTop;
        const radiusBottom = g.radiusBottom === undefined ? baseR : g.radiusBottom;
        if (!isPos(radiusTop))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}.radiusTop`);
        if (!isPos(radiusBottom))
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] invalid ${type}.radiusBottom`);
        if (Math.abs(radiusTop - radiusBottom) > 1e-6) {
            throw new KhrValidationError(`KHR_implicit_shapes[${idx}] tapered ${type} (${radiusTop} !== ${radiusBottom}) is unsupported`);
        }
        return { type, height, radiusTop, radiusBottom };
    }
    throw new KhrValidationError(`KHR_implicit_shapes[${idx}] unsupported shape type "${String(type)}"`);
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
    const dynF = optNum(mat.dynamicFriction, `physicsMaterials[${idx}].dynamicFriction`, true);
    const statF = optNum(mat.staticFriction, `physicsMaterials[${idx}].staticFriction`, true);
    const rest = optNum(mat.restitution, `physicsMaterials[${idx}].restitution`, true);
    const frictionCombine = optCombineMode(mat.frictionCombine, `physicsMaterials[${idx}].frictionCombine`);
    const restitutionCombine = optCombineMode(mat.restitutionCombine, `physicsMaterials[${idx}].restitutionCombine`);
    return {
        ...(dynF !== undefined || statF !== undefined ? { friction: dynF ?? statF } : {}),
        ...(rest === undefined ? {} : { restitution: rest }),
        ...(frictionCombine === undefined ? {} : { frictionCombine }),
        ...(restitutionCombine === undefined ? {} : { restitutionCombine }),
    };
}
