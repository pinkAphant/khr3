import * as THREE from "three";
import type { KhrBodySpec } from "./khrTypes.js";

export type Vec3Tuple = [number, number, number];
export type QuatTuple = [number, number, number, number];

export interface LumpedMassProps {
    mass: number;
    com: Vec3Tuple;
    inertia: Vec3Tuple;
    inertiaFrame: QuatTuple;
}

const _pos = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _qExtra = new THREE.Quaternion();
const _comVec = new THREE.Vector3();
const _one = new THREE.Vector3(1, 1, 1);
const _mat = new THREE.Matrix4();
const _tmpMat = new THREE.Matrix4();
const _rCom = new THREE.Vector3();

export function bodyPoseMatrix(body: KhrBodySpec, target = new THREE.Matrix4()): THREE.Matrix4 {
    _pos.fromArray(body.localPosition);
    _quat.fromArray(body.localQuaternion);
    return target.compose(_pos, _quat, _one);
}

export function jacobiPrincipalInertia(
    ixx: number,
    iyy: number,
    izz: number,
    ixy: number,
    ixz: number,
    iyz: number,
): { inertia: Vec3Tuple; inertiaFrame: QuatTuple } {
    const a: [Vec3Tuple, Vec3Tuple, Vec3Tuple] = [
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ];
    const v: [Vec3Tuple, Vec3Tuple, Vec3Tuple] = [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ];
    for (let iter = 0; iter < 32; iter++) {
        const tr = Math.max(a[0][0] + a[1][1] + a[2][2], 0);
        let p = 0, q = 1, maxOff = Math.abs(a[0][1]);
        if (Math.abs(a[0][2]) > maxOff) {
            p = 0;
            q = 2;
            maxOff = Math.abs(a[0][2]);
        }
        if (Math.abs(a[1][2]) > maxOff) {
            p = 1;
            q = 2;
            maxOff = Math.abs(a[1][2]);
        }
        if (maxOff <= 1e-12 * tr)
            break;
        const apq = a[p][q], app = a[p][p], aqq = a[q][q];
        const tau = (aqq - app) / (2 * apq);
        const t = tau >= 0 ? 1 / (tau + Math.sqrt(1 + tau * tau)) : -1 / (-tau + Math.sqrt(1 + tau * tau));
        const c = 1 / Math.sqrt(1 + t * t), s = t * c;
        const r = 3 - p - q;
        a[p][p] = c * c * app - 2 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2 * s * c * apq + c * c * aqq;
        a[p][q] = a[q][p] = 0;
        const arp = a[r][p], arq = a[r][q];
        a[r][p] = a[p][r] = c * arp - s * arq;
        a[r][q] = a[q][r] = s * arp + c * arq;
        for (let i = 0; i < 3; i++) {
            const vip = v[i][p], viq = v[i][q];
            v[i][p] = c * vip - s * viq;
            v[i][q] = s * vip + c * viq;
        }
    }
    const det = v[0][0] * (v[1][1] * v[2][2] - v[1][2] * v[2][1])
        - v[0][1] * (v[1][0] * v[2][2] - v[1][2] * v[2][0])
        + v[0][2] * (v[1][0] * v[2][1] - v[1][1] * v[2][0]);
    if (det < 0) {
        v[0][2] = -v[0][2];
        v[1][2] = -v[1][2];
        v[2][2] = -v[2][2];
    }
    _mat.set(
        v[0][0], v[0][1], v[0][2], 0,
        v[1][0], v[1][1], v[1][2], 0,
        v[2][0], v[2][1], v[2][2], 0,
        0, 0, 0, 1,
    );
    _quat.setFromRotationMatrix(_mat).normalize();
    return {
        inertia: [a[0][0], a[1][1], a[2][2]],
        inertiaFrame: [_quat.x, _quat.y, _quat.z, _quat.w],
    };
}

function resolvePose(body: KhrBodySpec, invFrame?: THREE.Matrix4): void {
    if (invFrame) {
        _mat.multiplyMatrices(invFrame, bodyPoseMatrix(body, _tmpMat)).decompose(_pos, _quat, _comVec);
        _quat.normalize();
    }
    else {
        _pos.fromArray(body.localPosition);
        _quat.fromArray(body.localQuaternion).normalize();
    }
}

export function computeLumpedMassProps(bodies: readonly KhrBodySpec[], invFrame?: THREE.Matrix4): LumpedMassProps | null {
    const dyn = bodies.filter((b) => !b.isKinematic && b.mass > 0);
    if (dyn.length === 0)
        return null;
    let totalMass = 0, mx = 0, my = 0, mz = 0;
    for (const b of dyn) {
        resolvePose(b, invFrame);
        _rCom.fromArray(b.centerOfMass).applyQuaternion(_quat);
        totalMass += b.mass;
        mx += (_pos.x + _rCom.x) * b.mass;
        my += (_pos.y + _rCom.y) * b.mass;
        mz += (_pos.z + _rCom.z) * b.mass;
    }
    if (!(totalMass > 0))
        return null;
    const com: Vec3Tuple = [mx / totalMass, my / totalMass, mz / totalMass];
    let ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
    const m = _mat.elements;
    for (const b of dyn) {
        resolvePose(b, invFrame);
        _rCom.fromArray(b.centerOfMass).applyQuaternion(_quat);
        const dx = _pos.x + _rCom.x - com[0];
        const dy = _pos.y + _rCom.y - com[1];
        const dz = _pos.z + _rCom.z - com[2];
        _mat.makeRotationFromQuaternion(_quat.multiply(_qExtra.fromArray(b.inertiaOrientation)));
        const [d0, d1, d2] = b.inertiaDiagonal;
        ixx += m[0] * m[0] * d0 + m[4] * m[4] * d1 + m[8] * m[8] * d2 + b.mass * (dy * dy + dz * dz);
        iyy += m[1] * m[1] * d0 + m[5] * m[5] * d1 + m[9] * m[9] * d2 + b.mass * (dx * dx + dz * dz);
        izz += m[2] * m[2] * d0 + m[6] * m[6] * d1 + m[10] * m[10] * d2 + b.mass * (dx * dx + dy * dy);
        ixy += m[0] * m[1] * d0 + m[4] * m[5] * d1 + m[8] * m[9] * d2 - b.mass * dx * dy;
        ixz += m[0] * m[2] * d0 + m[4] * m[6] * d1 + m[8] * m[10] * d2 - b.mass * dx * dz;
        iyz += m[1] * m[2] * d0 + m[5] * m[6] * d1 + m[9] * m[10] * d2 - b.mass * dy * dz;
    }
    if (dyn.length === 1) {
        const b = dyn[0];
        resolvePose(b, invFrame);
        _quat.multiply(_qExtra.fromArray(b.inertiaOrientation)).normalize();
        return {
            mass: totalMass,
            com,
            inertia: [b.inertiaDiagonal[0], b.inertiaDiagonal[1], b.inertiaDiagonal[2]],
            inertiaFrame: [_quat.x, _quat.y, _quat.z, _quat.w],
        };
    }
    const diagSum = Math.max(ixx + iyy + izz, 0);
    if (Math.max(Math.abs(ixy), Math.abs(ixz), Math.abs(iyz)) <= 1e-12 * diagSum) {
        return {
            mass: totalMass,
            com,
            inertia: [ixx, iyy, izz],
            inertiaFrame: [0, 0, 0, 1],
        };
    }
    const { inertia, inertiaFrame } = jacobiPrincipalInertia(ixx, iyy, izz, ixy, ixz, iyz);
    return { mass: totalMass, com, inertia, inertiaFrame };
}
