import * as THREE from "three";
import type { RapierRigidBody } from "@react-three/rapier";
import { type KhrBodySpec, type KhrColliderShape, KhrValidationError } from "./types.js";

export type Vec3Tuple = [number, number, number];
export type QuatTuple = [number, number, number, number];

export interface RapierBodyMassTarget {
    readonly mass: number;
    readonly com: Vec3Tuple;
    readonly hasExplicitCom: boolean;
    readonly inertiaDiagonal: Vec3Tuple;
    readonly inertiaFrame: QuatTuple;
    readonly hasExplicitInertia: boolean;
}

const SCALE_EPS = 1e-4;
const _wsScratch = new THREE.Vector3();
const _one = new THREE.Vector3(1, 1, 1);
const _tmpPos = new THREE.Vector3();
const _tmpQuat = new THREE.Quaternion();
const _tmpQuatB = new THREE.Quaternion();

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

export function bodyPoseMatrix(body: KhrBodySpec, target = new THREE.Matrix4()): THREE.Matrix4 {
    return target.compose(
        _tmpPos.fromArray(body.localPosition),
        _tmpQuat.fromArray(body.localQuaternion),
        _one,
    );
}

export function computeAssemblyMassAndCom(bodies: readonly KhrBodySpec[]): {
    totalDynamicMass: number;
    centerOfMass: Vec3Tuple;
} {
    let totalDynamicMass = 0, mx = 0, my = 0, mz = 0;
    for (const b of bodies) {
        if (b.isKinematic)
            continue;
        totalDynamicMass += b.mass;
        _tmpPos.fromArray(b.centerOfMass).applyQuaternion(_tmpQuat.fromArray(b.localQuaternion));
        mx += (b.localPosition[0] + _tmpPos.x) * b.mass;
        my += (b.localPosition[1] + _tmpPos.y) * b.mass;
        mz += (b.localPosition[2] + _tmpPos.z) * b.mass;
    }
    return {
        totalDynamicMass,
        centerOfMass: totalDynamicMass > 0 ? [mx / totalDynamicMass, my / totalDynamicMass, mz / totalDynamicMass] : [0, 0, 0],
    };
}

/**
 * Builds a `RigidBody`-local mass specification from a `KhrBodySpec`.
 * When `hasExplicitInertia` is false, Rapier WASM (`parry3d`) computes exact volume, CoM,
 * parallel-axis tensor, and principal inertia directly from the attached colliders.
 */
export function buildBodyMassTarget(
    body: KhrBodySpec,
    bodyRotInRigidBody: readonly [number, number, number, number] = [0, 0, 0, 1],
    bodyOffsetInRigidBody: readonly [number, number, number] = [0, 0, 0],
): RapierBodyMassTarget | null {
    if (body.isKinematic || !(body.mass > 0))
        return null;
    _tmpQuat.set(bodyRotInRigidBody[0], bodyRotInRigidBody[1], bodyRotInRigidBody[2], bodyRotInRigidBody[3]).normalize();
    _tmpPos.fromArray(body.centerOfMass).applyQuaternion(_tmpQuat);
    const com: Vec3Tuple = [
        bodyOffsetInRigidBody[0] + _tmpPos.x,
        bodyOffsetInRigidBody[1] + _tmpPos.y,
        bodyOffsetInRigidBody[2] + _tmpPos.z,
    ];
    _tmpQuatB.copy(_tmpQuat).multiply(new THREE.Quaternion().fromArray(body.inertiaOrientation)).normalize();
    return {
        mass: body.mass,
        com,
        hasExplicitCom: Boolean(body.hasExplicitCom) || (
            Math.abs(com[0]) > 1e-9 || Math.abs(com[1]) > 1e-9 || Math.abs(com[2]) > 1e-9
        ),
        inertiaDiagonal: [body.inertiaDiagonal[0], body.inertiaDiagonal[1], body.inertiaDiagonal[2]],
        inertiaFrame: [_tmpQuatB.x, _tmpQuatB.y, _tmpQuatB.z, _tmpQuatB.w],
        hasExplicitInertia: Boolean(body.hasExplicitInertia),
    };
}

/**
 * Applies `KHR_physics_rigid_bodies` mass properties to a live Rapier WASM `RigidBody`:
 * - Explicit `motion.inertiaDiagonal`: passes authored `{ mass, com, inertiaDiagonal, inertiaFrame }` directly to Rapier.
 * - Omitted `motion.inertiaDiagonal`: delegates volume, CoM, parallel-axis lumping, and Jacobi principal-axis inertia
 *   to Rapier's Rust `parry3d` engine across all attached colliders, preserving explicit `motion.centerOfMass` if authored.
 */
export function applyRapierBodyMass(rb: RapierRigidBody, target: RapierBodyMassTarget | null): void {
    if (!target || !(target.mass > 0))
        return;
    const n = rb.numColliders();
    if (target.hasExplicitInertia) {
        for (let i = 0; i < n; i++)
            rb.collider(i).setDensity(0);
        rb.setAdditionalMassProperties(
            target.mass,
            { x: target.com[0], y: target.com[1], z: target.com[2] },
            { x: target.inertiaDiagonal[0], y: target.inertiaDiagonal[1], z: target.inertiaDiagonal[2] },
            { x: target.inertiaFrame[0], y: target.inertiaFrame[1], z: target.inertiaFrame[2], w: target.inertiaFrame[3] },
            true,
        );
        rb.recomputeMassPropertiesFromColliders();
        return;
    }

    rb.setAdditionalMassProperties(0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0, w: 1 }, false);
    for (let i = 0; i < n; i++)
        rb.collider(i).setDensity(1);
    rb.recomputeMassPropertiesFromColliders();
    const unitMass = rb.mass();

    if (n > 0 && unitMass > 1e-9) {
        const rho = target.mass / unitMass;
        for (let i = 0; i < n; i++)
            rb.collider(i).setDensity(rho);
        rb.recomputeMassPropertiesFromColliders();
        if (target.hasExplicitCom) {
            const pi = rb.principalInertia();
            const pf = rb.principalInertiaLocalFrame();
            for (let i = 0; i < n; i++)
                rb.collider(i).setDensity(0);
            rb.setAdditionalMassProperties(
                target.mass,
                { x: target.com[0], y: target.com[1], z: target.com[2] },
                { x: pi.x, y: pi.y, z: pi.z },
                { x: pf.x, y: pf.y, z: pf.z, w: pf.w },
                true,
            );
            rb.recomputeMassPropertiesFromColliders();
        }
        return;
    }

    for (let i = 0; i < n; i++)
        rb.collider(i).setDensity(0);
    const diag = 0.04 * target.mass;
    rb.setAdditionalMassProperties(
        target.mass,
        { x: target.com[0], y: target.com[1], z: target.com[2] },
        { x: diag, y: diag, z: diag },
        { x: 0, y: 0, z: 0, w: 1 },
        true,
    );
    rb.recomputeMassPropertiesFromColliders();
}
