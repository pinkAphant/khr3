import * as THREE from "three";
import { type EcctrlWheelTag } from "./khrTypes.js";

const G_SI = 9.81;

export interface SolvedWheel {
    tag: EcctrlWheelTag;
    cornerMassKg: number;
    springK: number;
    dampingC: number;
    mountPos: [number, number, number];
}

function projectToLowerBoundedSimplex(values: readonly number[], lowerBound: number, totalSum: number): number[] {
    const n = values.length;
    const shifted = values.map((v) => v - lowerBound);
    const rem = Math.max(0, totalSum - n * lowerBound);
    const sorted = [...shifted].sort((a, b) => b - a);
    let prefix = 0;
    let k = 0;
    let theta = 0;
    for (let i = 0; i < n; i++) {
        prefix += sorted[i];
        const candidate = (prefix - rem) / (i + 1);
        if (sorted[i] - candidate > 0) {
            k = i + 1;
            theta = candidate;
        }
    }
    if (k === 0)
        theta = (prefix - rem) / n;
    return shifted.map((v) => lowerBound + Math.max(v - theta, 0));
}

function solveCornerMasses(
    wheels: readonly EcctrlWheelTag[],
    totalMassKg: number,
    com: readonly [number, number, number],
): number[] {
    const n = wheels.length;
    if (n === 0)
        return [];
    if (n === 1)
        return [totalMassKg];

    const mBase = totalMassKg / n;
    const xs = wheels.map((w) => w.position[0] - com[0]);
    const zs = wheels.map((w) => w.position[2] - com[2]);
    const meanX = xs.reduce((s, v) => s + v, 0) / n;
    const meanZ = zs.reduce((s, v) => s + v, 0) / n;
    const cx = xs.map((v) => v - meanX);
    const cz = zs.map((v) => v - meanZ);

    let sxx = 0;
    let szz = 0;
    let sxz = 0;
    for (let i = 0; i < n; i++) {
        sxx += cx[i] * cx[i];
        szz += cz[i] * cz[i];
        sxz += cx[i] * cz[i];
    }

    const bx = -totalMassKg * meanX;
    const bz = -totalMassKg * meanZ;
    const det = sxx * szz - sxz * sxz;
    const trace = sxx + szz;
    let lx = 0;
    let lz = 0;

    if (det > 1e-10 * trace * trace && det > 1e-12) {
        lx = (szz * bx - sxz * bz) / det;
        lz = (sxx * bz - sxz * bx) / det;
    }
    else if (trace > 1e-10) {
        const angle = 0.5 * Math.atan2(2 * sxz, sxx - szz);
        const ux = Math.cos(angle);
        const uz = Math.sin(angle);
        const normSq = ux * ux * sxx + 2 * ux * uz * sxz + uz * uz * szz;
        if (normSq > 1e-10) {
            const proj = (ux * bx + uz * bz) / normSq;
            lx = proj * ux;
            lz = proj * uz;
        }
    }

    const raw = wheels.map((_, i) => mBase + lx * cx[i] + lz * cz[i]);
    const mMin = Math.min(0.05 * totalMassKg, totalMassKg / n);
    if (raw.every((m) => m >= mMin - 1e-9))
        return raw;
    return projectToLowerBoundedSimplex(raw, mMin, totalMassKg);
}

/**
 * Distributes `totalMassKg` across N >= 1 wheels around `com`, resolves fallback
 * `springK` / `dampingC` when omitted on the `.glb` wheel descriptor, and returns
 * natural scene-space `mountPos` (`tag.position + lift`).
 */
export function solveWheelLoads(
    wheels: readonly EcctrlWheelTag[],
    totalMassKg: number,
    com: readonly [number, number, number],
    dampingRatio = 0.7,
): SolvedWheel[] {
    if (wheels.length === 0)
        return [];

    const cornerMasses = solveCornerMasses(wheels, totalMassKg, com);
    return wheels.map((tag, i) => {
        const cornerMassKg = cornerMasses[i];
        const springK = tag.springK ?? (cornerMassKg * G_SI) / Math.max(tag.sag, 1e-4);
        const dampingC = tag.dampingC ?? 2 * dampingRatio * Math.sqrt(springK * cornerMassKg);
        const lift = Math.max(0, tag.travel - tag.sag);
        const mountOffset = new THREE.Vector3(0, lift, 0);
        if (tag.quaternion) {
            mountOffset.applyQuaternion(
                new THREE.Quaternion(tag.quaternion[0], tag.quaternion[1], tag.quaternion[2], tag.quaternion[3]).normalize(),
            );
        }
        const mountPos: [number, number, number] = [
            tag.position[0] + mountOffset.x,
            tag.position[1] + mountOffset.y,
            tag.position[2] + mountOffset.z,
        ];
        return { tag, cornerMassKg, springK, dampingC, mountPos };
    });
}
