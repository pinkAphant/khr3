import * as THREE from "three";
import { ConvexGeometry } from "three/examples/jsm/geometries/ConvexGeometry.js";
import { type KhrBodySpec, type KhrColliderShape, type KhrColliderSpec, KhrValidationError } from "./types.js";

export type Vec3Tuple = [number, number, number];
export type QuatTuple = [number, number, number, number];

export interface LumpedMassProps {
    mass: number;
    com: Vec3Tuple;
    inertia: Vec3Tuple;
    inertiaFrame: QuatTuple;
}

const SCALE_EPS = 1e-4;
const _wsScratch = new THREE.Vector3();

function isUnitScalar(v: unknown): boolean {
    return typeof v === "number" && Number.isFinite(v) && Math.abs(v - 1) <= 1e-9;
}

export function assertUnitScale(scale: unknown, owner: string): void {
    if (scale === undefined)
        return;
    let ok = false;
    if (typeof scale === "number")
        ok = isUnitScalar(scale);
    else if (Array.isArray(scale))
        ok = scale.length === 3 && scale.every(isUnitScalar);
    else if (typeof scale === "object" && scale && "x" in scale && "y" in scale && "z" in scale) {
        const v = scale as { x: unknown; y: unknown; z: unknown };
        ok = isUnitScalar(v.x) && isUnitScalar(v.y) && isUnitScalar(v.z);
    }
    if (!ok) {
        throw new KhrValidationError(
            `${owner}: scale must be 1 (got ${typeof scale === "number" && !Number.isFinite(scale) ? String(scale) : JSON.stringify(scale)}); bake scale into the .glb at import`,
        );
    }
}

export function readFiniteVec3(val: unknown, fieldLabel: string): Vec3Tuple {
    if (!Array.isArray(val) || val.length !== 3 || !val.every((n) => typeof n === "number" && Number.isFinite(n))) {
        throw new KhrValidationError(`Invalid ${fieldLabel}: expected finite 3-tuple`);
    }
    return [val[0], val[1], val[2]];
}

export function readUnitQuat(val: unknown, fieldLabel: string): QuatTuple {
    if (!Array.isArray(val) || val.length !== 4 || !val.every((n) => typeof n === "number" && Number.isFinite(n))) {
        throw new KhrValidationError(`Invalid ${fieldLabel}: expected finite 4-tuple`);
    }
    const norm = Math.hypot(val[0], val[1], val[2], val[3]);
    if (!(norm > 1e-6))
        throw new KhrValidationError(`Invalid ${fieldLabel}: zero-norm quaternion`);
    return [val[0] / norm, val[1] / norm, val[2] / norm, val[3] / norm];
}

export function assertIdentityWorldScale(obj: THREE.Object3D, nodeName: string): void {
    obj.getWorldScale(_wsScratch);
    if (
        !Number.isFinite(_wsScratch.x) || !Number.isFinite(_wsScratch.y) || !Number.isFinite(_wsScratch.z) ||
        Math.abs(_wsScratch.x - 1) > SCALE_EPS || Math.abs(_wsScratch.y - 1) > SCALE_EPS || Math.abs(_wsScratch.z - 1) > SCALE_EPS
    ) {
        throw new KhrValidationError(
            `Non-identity world scale [${_wsScratch.x.toFixed(4)}, ${_wsScratch.y.toFixed(4)}, ${_wsScratch.z.toFixed(4)}] on "${nodeName}"; bake scale=[1,1,1]`,
        );
    }
    const ls = obj.scale;
    if (
        !Number.isFinite(ls.x) || !Number.isFinite(ls.y) || !Number.isFinite(ls.z) ||
        Math.abs(ls.x - 1) > SCALE_EPS || Math.abs(ls.y - 1) > SCALE_EPS || Math.abs(ls.z - 1) > SCALE_EPS
    ) {
        throw new KhrValidationError(`Non-identity local scale [${ls.x.toFixed(4)}, ${ls.y.toFixed(4)}, ${ls.z.toFixed(4)}] on "${nodeName}"; bake scale=[1,1,1]`);
    }
}

export function assertSceneIdentityWorldScale(root: THREE.Object3D): void {
    root.updateMatrixWorld(true);
    root.traverse((child) => {
        assertIdentityWorldScale(child, child.name || (child === root ? "gltf.scene" : "unnamed_node"));
    });
}

export function computeRelativePose(ownerObj: THREE.Object3D, childObj: THREE.Object3D): {
    localPosition: Vec3Tuple;
    localQuaternion: QuatTuple;
} {
    const ownerPos = ownerObj.getWorldPosition(new THREE.Vector3());
    const ownerInvQuat = ownerObj.getWorldQuaternion(new THREE.Quaternion()).invert();
    const pos = childObj.getWorldPosition(new THREE.Vector3()).sub(ownerPos).applyQuaternion(ownerInvQuat);
    const quat = ownerInvQuat.multiply(childObj.getWorldQuaternion(new THREE.Quaternion()));
    return {
        localPosition: [pos.x, pos.y, pos.z],
        localQuaternion: [quat.x, quat.y, quat.z, quat.w],
    };
}

export function extractMeshColliderShape(targetObj: THREE.Object3D, meshNodeName: string, convexHull: boolean): KhrColliderShape {
    assertIdentityWorldScale(targetObj, meshNodeName);
    const invOwner = targetObj.matrixWorld.clone().invert();
    const rel = new THREE.Matrix4();
    const v = new THREE.Vector3();
    const verts: number[] = [];
    const indices: number[] = [];
    targetObj.traverse((child) => {
        const mesh = child as THREE.Mesh;
        if (!mesh.isMesh || !mesh.geometry)
            return;
        assertIdentityWorldScale(mesh, mesh.name || meshNodeName);
        const posAttr = mesh.geometry.getAttribute("position");
        if (!posAttr || posAttr.count === 0)
            return;
        rel.multiplyMatrices(invOwner, mesh.matrixWorld);
        const base = verts.length / 3;
        for (let i = 0; i < posAttr.count; i++) {
            v.fromBufferAttribute(posAttr, i).applyMatrix4(rel);
            if (!Number.isFinite(v.x) || !Number.isFinite(v.y) || !Number.isFinite(v.z)) {
                throw new KhrValidationError(`Non-finite vertex in collider mesh "${meshNodeName}"`);
            }
            verts.push(v.x, v.y, v.z);
        }
        const idx = mesh.geometry.getIndex();
        if (idx) {
            for (let i = 0; i < idx.count; i++)
                indices.push(base + idx.getX(i));
        }
        else {
            for (let i = 0; i < posAttr.count; i++)
                indices.push(base + i);
        }
    });
    if (verts.length < 9)
        throw new KhrValidationError(`Collider node "${meshNodeName}" has fewer than 3 vertices`);
    if (convexHull) {
        return { type: "convexHull", vertices: new Float32Array(verts), meshNodeName };
    }
    if (indices.length < 3)
        throw new KhrValidationError(`Trimesh node "${meshNodeName}" has fewer than 3 indices`);
    return { type: "trimesh", vertices: new Float32Array(verts), indices: new Uint32Array(indices), meshNodeName };
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
    const mat = new THREE.Matrix4().set(
        v[0][0], v[0][1], v[0][2], 0,
        v[1][0], v[1][1], v[1][2], 0,
        v[2][0], v[2][1], v[2][2], 0,
        0, 0, 0, 1,
    );
    const q = new THREE.Quaternion().setFromRotationMatrix(mat).normalize();
    return {
        inertia: [a[0][0], a[1][1], a[2][2]],
        inertiaFrame: [q.x, q.y, q.z, q.w],
    };
}

interface ShapeUnitMassProps {
    volume: number;
    centroid: THREE.Vector3;
    /** Unit-mass 3x3 inertia tensor [ixx, iyy, izz, ixy, ixz, iyz] about `centroid` in shape local axes. */
    unitTensor: [number, number, number, number, number, number];
}

/**
 * Exact signed-tetrahedron divergence integration for closed triangle meshes (`trimesh` / `convexHull`).
 * Falls back cleanly to AABB slab inertia only when the mesh is flat/open with near-zero enclosed volume.
 */
function integrateTriangleMeshProps(vertices: Float32Array, indices: Uint32Array | null): ShapeUnitMassProps | null {
    const triCount = indices ? Math.floor(indices.length / 3) : Math.floor(vertices.length / 9);
    if (triCount < 4)
        return null;

    let rx = 0, ry = 0, rz = 0;
    const vCount = Math.floor(vertices.length / 3);
    for (let i = 0; i < vCount; i++) {
        rx += vertices[i * 3];
        ry += vertices[i * 3 + 1];
        rz += vertices[i * 3 + 2];
    }
    rx /= vCount;
    ry /= vCount;
    rz /= vCount;

    let vol6 = 0;
    let cx24 = 0, cy24 = 0, cz24 = 0;
    let cxx120 = 0, cyy120 = 0, czz120 = 0;
    let cxy120 = 0, cxz120 = 0, cyz120 = 0;

    for (let t = 0; t < triCount; t++) {
        const i0 = (indices ? indices[t * 3] : t * 3) * 3;
        const i1 = (indices ? indices[t * 3 + 1] : t * 3 + 1) * 3;
        const i2 = (indices ? indices[t * 3 + 2] : t * 3 + 2) * 3;
        const ax = vertices[i0] - rx, ay = vertices[i0 + 1] - ry, az = vertices[i0 + 2] - rz;
        const bx = vertices[i1] - rx, by = vertices[i1 + 1] - ry, bz = vertices[i1 + 2] - rz;
        const cx = vertices[i2] - rx, cy = vertices[i2 + 1] - ry, cz = vertices[i2 + 2] - rz;

        const det = ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
        vol6 += det;
        const sx = ax + bx + cx, sy = ay + by + cy, sz = az + bz + cz;
        cx24 += det * sx;
        cy24 += det * sy;
        cz24 += det * sz;

        cxx120 += det * (sx * sx + ax * ax + bx * bx + cx * cx);
        cyy120 += det * (sy * sy + ay * ay + by * by + cy * cy);
        czz120 += det * (sz * sz + az * az + bz * bz + cz * cz);
        cxy120 += det * (sx * sy + ax * ay + bx * by + cx * cy);
        cxz120 += det * (sx * sz + ax * az + bx * bz + cx * cz);
        cyz120 += det * (sy * sz + ay * az + by * bz + cy * cz);
    }

    const sign = vol6 < 0 ? -1 : 1;
    const volume = (sign * vol6) / 6;
    if (!(volume > 1e-9))
        return null;

    const invV = 1 / volume;
    const comRelX = (sign * cx24) / (24 * volume);
    const comRelY = (sign * cy24) / (24 * volume);
    const comRelZ = (sign * cz24) / (24 * volume);

    const cxx = ((sign * cxx120) / 120) * invV - comRelX * comRelX;
    const cyy = ((sign * cyy120) / 120) * invV - comRelY * comRelY;
    const czz = ((sign * czz120) / 120) * invV - comRelZ * comRelZ;
    const cxy = ((sign * cxy120) / 120) * invV - comRelX * comRelY;
    const cxz = ((sign * cxz120) / 120) * invV - comRelX * comRelZ;
    const cyz = ((sign * cyz120) / 120) * invV - comRelY * comRelZ;

    const ixx = cyy + czz;
    const iyy = cxx + czz;
    const izz = cxx + cyy;
    if (!(ixx > 0 && iyy > 0 && izz > 0))
        return null;

    return {
        volume,
        centroid: new THREE.Vector3(rx + comRelX, ry + comRelY, rz + comRelZ),
        unitTensor: [ixx, iyy, izz, -cxy, -cxz, -cyz],
    };
}

function fallbackAabbProps(vertices: Float32Array): ShapeUnitMassProps {
    const box = new THREE.Box3();
    const pt = new THREE.Vector3();
    for (let i = 0; i < vertices.length; i += 3)
        box.expandByPoint(pt.set(vertices[i], vertices[i + 1], vertices[i + 2]));
    const s = box.getSize(new THREE.Vector3());
    const sx = Math.max(s.x, 1e-3), sy = Math.max(s.y, 1e-3), sz = Math.max(s.z, 1e-3);
    return {
        volume: sx * sy * sz,
        centroid: box.getCenter(new THREE.Vector3()),
        unitTensor: [(sy * sy + sz * sz) / 12, (sx * sx + sz * sz) / 12, (sx * sx + sy * sy) / 12, 0, 0, 0],
    };
}

function unitInertiaForShape(shape: KhrColliderShape): ShapeUnitMassProps {
    if (shape.type === "box") {
        const [x, y, z] = shape.size;
        return {
            volume: x * y * z,
            centroid: new THREE.Vector3(),
            unitTensor: [(y * y + z * z) / 12, (x * x + z * z) / 12, (x * x + y * y) / 12, 0, 0, 0],
        };
    }
    if (shape.type === "sphere") {
        const r = shape.radius;
        const i = 0.4 * r * r;
        return {
            volume: (4 / 3) * Math.PI * r * r * r,
            centroid: new THREE.Vector3(),
            unitTensor: [i, i, i, 0, 0, 0],
        };
    }
    if (shape.type === "trimesh") {
        return integrateTriangleMeshProps(shape.vertices, shape.indices) ?? fallbackAabbProps(shape.vertices);
    }
    if (shape.type === "convexHull") {
        if (shape.vertices.length >= 12) {
            const pts: THREE.Vector3[] = [];
            for (let i = 0; i < shape.vertices.length; i += 3) {
                const vx = shape.vertices[i], vy = shape.vertices[i + 1], vz = shape.vertices[i + 2];
                let dup = false;
                for (let j = 0; j < pts.length; j++) {
                    const p = pts[j];
                    if (Math.abs(p.x - vx) < 1e-5 && Math.abs(p.y - vy) < 1e-5 && Math.abs(p.z - vz) < 1e-5) {
                        dup = true;
                        break;
                    }
                }
                if (!dup)
                    pts.push(new THREE.Vector3(vx, vy, vz));
            }
            if (pts.length >= 4) {
                try {
                    const hullGeo = new ConvexGeometry(pts);
                    const pos = hullGeo.getAttribute("position");
                    const exact = pos && pos.count >= 12
                        ? integrateTriangleMeshProps(pos.array as Float32Array, null)
                        : null;
                    hullGeo.dispose();
                    if (exact)
                        return exact;
                }
                catch {
                    // Coplanar point clouds fall back to slab AABB below
                }
            }
        }
        return fallbackAabbProps(shape.vertices);
    }
    const r = shape.radiusTop;
    const h = shape.height;
    if (shape.type === "capsule") {
        const cylVol = Math.PI * r * r * h;
        const totVol = cylVol + (4 / 3) * Math.PI * r * r * r;
        const fc = cylVol / totVol;
        const fh = 1 - fc;
        const iy = 0.5 * fc * r * r + 0.4 * fh * r * r;
        const ix = fc * ((h * h) / 12 + 0.25 * r * r) + fh * (0.4 * r * r + 0.25 * h * h + 0.375 * h * r);
        return {
            volume: totVol,
            centroid: new THREE.Vector3(),
            unitTensor: [ix, iy, ix, 0, 0, 0],
        };
    }
    const ix = (3 * r * r + h * h) / 12;
    return {
        volume: Math.PI * r * r * h,
        centroid: new THREE.Vector3(),
        unitTensor: [ix, 0.5 * r * r, ix, 0, 0, 0],
    };
}

export function computeShapeInertia(shape: KhrColliderShape, mass: number): Vec3Tuple {
    const [ixx, iyy, izz, ixy, ixz, iyz] = unitInertiaForShape(shape).unitTensor;
    if (Math.abs(ixy) + Math.abs(ixz) + Math.abs(iyz) > 1e-12 * (ixx + iyy + izz)) {
        const { inertia } = jacobiPrincipalInertia(ixx, iyy, izz, ixy, ixz, iyz);
        return [mass * inertia[0], mass * inertia[1], mass * inertia[2]];
    }
    return [mass * ixx, mass * iyy, mass * izz];
}

export function computeBodyInertia(
    colliders: readonly KhrColliderSpec[],
    mass: number,
    centerOfMass: readonly [number, number, number],
    inertiaOrientation: readonly [number, number, number, number],
    nodeName: string,
): Vec3Tuple {
    if (colliders.length === 0) {
        throw new KhrValidationError(`Dynamic node "${nodeName}" is missing motion.inertiaDiagonal and has no colliders`);
    }
    const shapes = colliders.map((c) => unitInertiaForShape(c.shape));
    let totalVol = 0;
    for (const s of shapes)
        totalVol += s.volume;
    const invFrame = new THREE.Quaternion().fromArray(inertiaOrientation).invert();
    const com = new THREE.Vector3().fromArray(centerOfMass);
    const colQuat = new THREE.Quaternion();
    const relQuat = new THREE.Quaternion();
    const offset = new THREE.Vector3();
    const rotMat = new THREE.Matrix4();
    const m = rotMat.elements;
    let ix = 0, iy = 0, iz = 0;

    for (let i = 0; i < colliders.length; i++) {
        const col = colliders[i];
        const shp = shapes[i];
        const mShare = totalVol > 0 ? mass * (shp.volume / totalVol) : mass / colliders.length;
        colQuat.fromArray(col.localQuaternion);
        offset
            .copy(shp.centroid)
            .applyQuaternion(colQuat)
            .add(new THREE.Vector3().fromArray(col.localPosition))
            .sub(com)
            .applyQuaternion(invFrame);
        relQuat.copy(invFrame).multiply(colQuat);
        rotMat.makeRotationFromQuaternion(relQuat);
        // R * U * R^T diagonal elements (column-major m: col0=[0,1,2], col1=[4,5,6], col2=[8,9,10])
        const [txx, tyy, tzz, txy, txz, tyz] = shp.unitTensor;
        const r00 = m[0], r01 = m[4], r02 = m[8];
        const r10 = m[1], r11 = m[5], r12 = m[9];
        const r20 = m[2], r21 = m[6], r22 = m[10];
        const rx = r00 * (txx * r00 + txy * r01 + txz * r02)
            + r01 * (txy * r00 + tyy * r01 + tyz * r02)
            + r02 * (txz * r00 + tyz * r01 + tzz * r02);
        const ry = r10 * (txx * r10 + txy * r11 + txz * r12)
            + r11 * (txy * r10 + tyy * r11 + tyz * r12)
            + r12 * (txz * r10 + tyz * r11 + tzz * r12);
        const rz = r20 * (txx * r20 + txy * r21 + txz * r22)
            + r21 * (txy * r20 + tyy * r21 + tyz * r22)
            + r22 * (txz * r20 + tyz * r21 + tzz * r22);
        ix += mShare * (rx + offset.y * offset.y + offset.z * offset.z);
        iy += mShare * (ry + offset.x * offset.x + offset.z * offset.z);
        iz += mShare * (rz + offset.x * offset.x + offset.y * offset.y);
    }

    if (!Number.isFinite(ix) || !Number.isFinite(iy) || !Number.isFinite(iz) || ix <= 0 || iy <= 0 || iz <= 0) {
        throw new KhrValidationError(`Dynamic node "${nodeName}" has degenerate collider geometry; cannot compute positive motion.inertiaDiagonal`);
    }
    return [ix, iy, iz];
}

export function computeAssemblyMassAndCom(bodies: readonly KhrBodySpec[]): {
    totalDynamicMass: number;
    centerOfMass: Vec3Tuple;
} {
    let totalDynamicMass = 0, mx = 0, my = 0, mz = 0;
    const q = new THREE.Quaternion();
    const c = new THREE.Vector3();
    for (const b of bodies) {
        if (b.isKinematic)
            continue;
        totalDynamicMass += b.mass;
        c.fromArray(b.centerOfMass).applyQuaternion(q.fromArray(b.localQuaternion));
        mx += (b.localPosition[0] + c.x) * b.mass;
        my += (b.localPosition[1] + c.y) * b.mass;
        mz += (b.localPosition[2] + c.z) * b.mass;
    }
    return {
        totalDynamicMass,
        centerOfMass: totalDynamicMass > 0 ? [mx / totalDynamicMass, my / totalDynamicMass, mz / totalDynamicMass] : [0, 0, 0],
    };
}

const _one = new THREE.Vector3(1, 1, 1);

export function bodyPoseMatrix(body: KhrBodySpec, target = new THREE.Matrix4()): THREE.Matrix4 {
    return target.compose(
        new THREE.Vector3().fromArray(body.localPosition),
        new THREE.Quaternion().fromArray(body.localQuaternion),
        _one,
    );
}

function decomposeBodyInFrame(
    body: KhrBodySpec,
    invFrame: THREE.Matrix4 | undefined,
    posOut: THREE.Vector3,
    quatOut: THREE.Quaternion,
    sclScratch: THREE.Vector3,
    matScratch: THREE.Matrix4,
): void {
    if (invFrame) {
        matScratch.multiplyMatrices(invFrame, bodyPoseMatrix(body, matScratch)).decompose(posOut, quatOut, sclScratch);
        quatOut.normalize();
    }
    else {
        posOut.fromArray(body.localPosition);
        quatOut.fromArray(body.localQuaternion).normalize();
    }
}

export function computeLumpedMassProps(bodies: readonly KhrBodySpec[], invFrame?: THREE.Matrix4): LumpedMassProps | null {
    const dyn = bodies.filter((b) => !b.isKinematic && b.mass > 0);
    if (dyn.length === 0)
        return null;

    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const qExtra = new THREE.Quaternion();
    const rCom = new THREE.Vector3();
    const scl = new THREE.Vector3();
    const mat = new THREE.Matrix4();

    if (dyn.length === 1) {
        const b = dyn[0];
        decomposeBodyInFrame(b, invFrame, pos, quat, scl, mat);
        rCom.fromArray(b.centerOfMass).applyQuaternion(quat);
        const frameQuat = quat.clone().multiply(qExtra.fromArray(b.inertiaOrientation)).normalize();
        return {
            mass: b.mass,
            com: [pos.x + rCom.x, pos.y + rCom.y, pos.z + rCom.z],
            inertia: [b.inertiaDiagonal[0], b.inertiaDiagonal[1], b.inertiaDiagonal[2]],
            inertiaFrame: [frameQuat.x, frameQuat.y, frameQuat.z, frameQuat.w],
        };
    }

    let totalMass = 0, mx = 0, my = 0, mz = 0;
    for (const b of dyn) {
        decomposeBodyInFrame(b, invFrame, pos, quat, scl, mat);
        rCom.fromArray(b.centerOfMass).applyQuaternion(quat);
        totalMass += b.mass;
        mx += (pos.x + rCom.x) * b.mass;
        my += (pos.y + rCom.y) * b.mass;
        mz += (pos.z + rCom.z) * b.mass;
    }
    if (!(totalMass > 0))
        return null;
    const com: Vec3Tuple = [mx / totalMass, my / totalMass, mz / totalMass];

    let ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
    const m = mat.elements;
    for (const b of dyn) {
        decomposeBodyInFrame(b, invFrame, pos, quat, scl, mat);
        rCom.fromArray(b.centerOfMass).applyQuaternion(quat);
        const dx = pos.x + rCom.x - com[0];
        const dy = pos.y + rCom.y - com[1];
        const dz = pos.z + rCom.z - com[2];
        mat.makeRotationFromQuaternion(quat.multiply(qExtra.fromArray(b.inertiaOrientation)));
        const [d0, d1, d2] = b.inertiaDiagonal;
        ixx += m[0] * m[0] * d0 + m[4] * m[4] * d1 + m[8] * m[8] * d2 + b.mass * (dy * dy + dz * dz);
        iyy += m[1] * m[1] * d0 + m[5] * m[5] * d1 + m[9] * m[9] * d2 + b.mass * (dx * dx + dz * dz);
        izz += m[2] * m[2] * d0 + m[6] * m[6] * d1 + m[10] * m[10] * d2 + b.mass * (dx * dx + dy * dy);
        ixy += m[0] * m[1] * d0 + m[4] * m[5] * d1 + m[8] * m[9] * d2 - b.mass * dx * dy;
        ixz += m[0] * m[2] * d0 + m[4] * m[6] * d1 + m[8] * m[10] * d2 - b.mass * dx * dz;
        iyz += m[1] * m[2] * d0 + m[5] * m[6] * d1 + m[9] * m[10] * d2 - b.mass * dy * dz;
    }

    const diagSum = Math.max(ixx + iyy + izz, 0);
    if (Math.max(Math.abs(ixy), Math.abs(ixz), Math.abs(iyz)) <= 1e-12 * diagSum) {
        return { mass: totalMass, com, inertia: [ixx, iyy, izz], inertiaFrame: [0, 0, 0, 1] };
    }
    const { inertia, inertiaFrame } = jacobiPrincipalInertia(ixx, iyy, izz, ixy, ixz, iyz);
    return { mass: totalMass, com, inertia, inertiaFrame };
}
