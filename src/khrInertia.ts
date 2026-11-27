import * as THREE from "three";
import { type KhrBodySpec, type KhrColliderShape, type KhrColliderSpec, KhrValidationError } from "./khrTypes.js";

const SCALE_EPS = 1e-4;

function isUnitScalar(v: unknown): boolean {
    return typeof v === "number" && Number.isFinite(v) && Math.abs(v - 1) <= 1e-9;
}

export function assertUnitScale(scale: unknown, owner: string): void {
    if (scale === undefined) return;
    let ok = false;
    if (typeof scale === "number") ok = isUnitScalar(scale);
    else if (Array.isArray(scale)) ok = scale.length === 3 && scale.every(isUnitScalar);
    else if (typeof scale === "object" && scale && "x" in scale && "y" in scale && "z" in scale) {
        const v = scale as { x: unknown; y: unknown; z: unknown };
        ok = isUnitScalar(v.x) && isUnitScalar(v.y) && isUnitScalar(v.z);
    }
    if (!ok) throw new KhrValidationError(`${owner}: scale must be 1 (got ${typeof scale === "number" && !Number.isFinite(scale) ? String(scale) : JSON.stringify(scale)}); bake scale into the .glb at import`);
}

export function readFiniteVec3(val: unknown, fieldLabel: string): [number, number, number] {
    if (!Array.isArray(val) || val.length !== 3 || !val.every((n) => typeof n === "number" && Number.isFinite(n))) {
        throw new KhrValidationError(`Invalid ${fieldLabel}: expected finite 3-tuple`);
    }
    return [val[0], val[1], val[2]];
}

export function readUnitQuat(val: unknown, fieldLabel: string): [number, number, number, number] {
    if (!Array.isArray(val) || val.length !== 4 || !val.every((n) => typeof n === "number" && Number.isFinite(n))) {
        throw new KhrValidationError(`Invalid ${fieldLabel}: expected finite 4-tuple`);
    }
    const norm = Math.hypot(val[0], val[1], val[2], val[3]);
    if (!(norm > 1e-6))
        throw new KhrValidationError(`Invalid ${fieldLabel}: zero-norm quaternion`);
    return [val[0] / norm, val[1] / norm, val[2] / norm, val[3] / norm];
}

export function assertIdentityWorldScale(obj: THREE.Object3D, nodeName: string): void {
    const ws = obj.getWorldScale(new THREE.Vector3());
    if (!Number.isFinite(ws.x) || !Number.isFinite(ws.y) || !Number.isFinite(ws.z) ||
        Math.abs(ws.x - 1) > SCALE_EPS || Math.abs(ws.y - 1) > SCALE_EPS || Math.abs(ws.z - 1) > SCALE_EPS) {
        throw new KhrValidationError(`Non-identity world scale [${ws.x.toFixed(4)}, ${ws.y.toFixed(4)}, ${ws.z.toFixed(4)}] on "${nodeName}"; bake scale=[1,1,1]`);
    }
    const ls = obj.scale;
    if (!Number.isFinite(ls.x) || !Number.isFinite(ls.y) || !Number.isFinite(ls.z) ||
        Math.abs(ls.x - 1) > SCALE_EPS || Math.abs(ls.y - 1) > SCALE_EPS || Math.abs(ls.z - 1) > SCALE_EPS) {
        throw new KhrValidationError(`Non-identity local scale [${ls.x.toFixed(4)}, ${ls.y.toFixed(4)}, ${ls.z.toFixed(4)}] on "${nodeName}"; bake scale=[1,1,1]`);
    }
}

export function assertSceneIdentityWorldScale(root: THREE.Object3D): void {
    root.traverse((child) => {
        assertIdentityWorldScale(child, child.name || (child === root ? "gltf.scene" : "unnamed_node"));
    });
}

export function computeRelativePose(ownerObj: THREE.Object3D, childObj: THREE.Object3D): {
    localPosition: [number, number, number];
    localQuaternion: [number, number, number, number];
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

function unitInertiaForShape(shape: KhrColliderShape): {
    volume: number;
    centroid: THREE.Vector3;
    unitInertia: [number, number, number];
} {
    if (shape.type === "box") {
        const [x, y, z] = shape.size;
        return {
            volume: x * y * z,
            centroid: new THREE.Vector3(),
            unitInertia: [(y * y + z * z) / 12, (x * x + z * z) / 12, (x * x + y * y) / 12],
        };
    }
    if (shape.type === "sphere") {
        const r = shape.radius;
        const i = 0.4 * r * r;
        return {
            volume: (4 / 3) * Math.PI * r * r * r,
            centroid: new THREE.Vector3(),
            unitInertia: [i, i, i],
        };
    }
    if (shape.type === "convexHull" || shape.type === "trimesh") {
        const box = new THREE.Box3();
        const pt = new THREE.Vector3();
        const verts = shape.vertices;
        for (let i = 0; i < verts.length; i += 3)
            box.expandByPoint(pt.set(verts[i], verts[i + 1], verts[i + 2]));
        const s = box.getSize(new THREE.Vector3());
        return {
            volume: s.x * s.y * s.z,
            centroid: box.getCenter(new THREE.Vector3()),
            unitInertia: [(s.y * s.y + s.z * s.z) / 12, (s.x * s.x + s.z * s.z) / 12, (s.x * s.x + s.y * s.y) / 12],
        };
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
            unitInertia: [ix, iy, ix],
        };
    }
    const ix = (3 * r * r + h * h) / 12;
    return {
        volume: Math.PI * r * r * h,
        centroid: new THREE.Vector3(),
        unitInertia: [ix, 0.5 * r * r, ix],
    };
}

export function computeShapeInertia(shape: KhrColliderShape, mass: number): [number, number, number] {
    const [ix, iy, iz] = unitInertiaForShape(shape).unitInertia;
    return [mass * ix, mass * iy, mass * iz];
}

export function computeBodyInertia(
    colliders: readonly KhrColliderSpec[],
    mass: number,
    centerOfMass: readonly [number, number, number],
    inertiaOrientation: readonly [number, number, number, number],
    nodeName: string,
): [number, number, number] {
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
        offset.copy(shp.centroid).applyQuaternion(colQuat).add(new THREE.Vector3().fromArray(col.localPosition)).sub(com).applyQuaternion(invFrame);
        relQuat.copy(invFrame).multiply(colQuat);
        rotMat.makeRotationFromQuaternion(relQuat);
        const [u0, u1, u2] = shp.unitInertia;
        const rx = m[0] * m[0] * u0 + m[4] * m[4] * u1 + m[8] * m[8] * u2;
        const ry = m[1] * m[1] * u0 + m[5] * m[5] * u1 + m[9] * m[9] * u2;
        const rz = m[2] * m[2] * u0 + m[6] * m[6] * u1 + m[10] * m[10] * u2;
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
    centerOfMass: [number, number, number];
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
