import * as THREE from "three";
import { type KhrJointDriveSpec, type KhrJointSpec, KhrValidationError } from "./khrTypes.js";

const isObj = (v: unknown): v is Record<string, unknown> => typeof v === "object" && !!v && !Array.isArray(v);
const axisVector = (axis: 0 | 1 | 2): [number, number, number] => axis === 0 ? [1, 0, 0] : axis === 1 ? [0, 1, 0] : [0, 0, 1];

function optNum(v: unknown, label: string, nonNeg = false): number | undefined {
    if (v !== undefined) {
        if (typeof v !== "number" || !Number.isFinite(v) || (nonNeg && v < 0)) {
            throw new KhrValidationError(`Invalid ${label}`);
        }
        return v;
    }
    return undefined;
}

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
        const positionTarget = optNum(d.positionTarget, `${label} drive.positionTarget`);
        const velocityTarget = optNum(d.velocityTarget, `${label} drive.velocityTarget`);
        if (positionTarget === undefined && velocityTarget === undefined) {
            throw new KhrValidationError(`${label} requires finite drive.positionTarget or drive.velocityTarget`);
        }
        const stiffness = optNum(d.stiffness, `${label} drive.stiffness`, true);
        const damping = optNum(d.damping, `${label} drive.damping`, true);
        const maxForce = optNum(d.maxForce, `${label} drive.maxForce`, true);
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
    if (raw.limits !== undefined && !Array.isArray(raw.limits)) {
        throw new KhrValidationError(`physicsJoints[${jointIdx}].limits on "${nodeName}" must be an array`);
    }
    if (raw.drives !== undefined && !Array.isArray(raw.drives)) {
        throw new KhrValidationError(`physicsJoints[${jointIdx}].drives on "${nodeName}" must be an array`);
    }
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
            if (ax !== 0 && ax !== 1 && ax !== 2) {
                throw new KhrValidationError(`physicsJoints[${jointIdx}].limits invalid axis ${String(ax)} on "${nodeName}"`);
            }
            return ax;
        });
        const min = optNum(lim.min, `physicsJoints[${jointIdx}] limit.min on "${nodeName}"`);
        const max = optNum(lim.max, `physicsJoints[${jointIdx}] limit.max on "${nodeName}"`);
        if ((lim.min !== undefined || lim.max !== undefined) && (min === undefined || max === undefined)) {
            throw new KhrValidationError(`physicsJoints[${jointIdx}].limits on "${nodeName}" must specify both min and max`);
        }
        if (min !== undefined && max !== undefined && min > max) {
            throw new KhrValidationError(`physicsJoints[${jointIdx}] limit.min > limit.max on "${nodeName}"`);
        }
        const stiffness = optNum(lim.stiffness, `physicsJoints[${jointIdx}] limit.stiffness on "${nodeName}"`, true);
        const damping = optNum(lim.damping, `physicsJoints[${jointIdx}] limit.damping on "${nodeName}"`, true);
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
    if (isObj(raw.extras) && Array.isArray(raw.extras.jointAxis) && raw.extras.jointAxis.length === 3 &&
        raw.extras.jointAxis.every((n) => typeof n === "number" && Number.isFinite(n))) {
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
