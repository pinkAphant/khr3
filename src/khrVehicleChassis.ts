import * as THREE from "three";
import type { EcctrlVehicleProps } from "@pinkaphant/ecctrl/vehicle";
import { toColliderMass, type ColliderMass } from "./assemblyPlan.js";
import { assertUnitScale } from "./khrInertia.js";
import { bodyPoseMatrix, computeLumpedMassProps, type LumpedMassProps, type QuatTuple, type Vec3Tuple } from "./khrMassProps.js";
import { type KhrBodySpec, type KhrColliderSpec, KhrValidationError } from "./khrTypes.js";

export function assertVehicleProps(scale: unknown, colliders: unknown): void {
    if (colliders !== undefined) {
        throw new KhrValidationError("KhrVehicle: colliders prop cannot be overridden; edit colliders in the .glb asset");
    }
    assertUnitScale(scale, "KhrVehicle");
}

export function readPosTuple(pos: EcctrlVehicleProps["position"]): Vec3Tuple {
    if (typeof pos === "number")
        return [pos, pos, pos];
    if (Array.isArray(pos))
        return [Number(pos[0] ?? 0), Number(pos[1] ?? 0), Number(pos[2] ?? 0)];
    if (pos && typeof pos === "object" && "x" in pos) {
        const v = pos as { x: number; y: number; z: number };
        return [Number(v.x ?? 0), Number(v.y ?? 0), Number(v.z ?? 0)];
    }
    return [0, 0.5, 0];
}

export function readSpawnQuat(
    rotation: EcctrlVehicleProps["rotation"],
    quaternion: EcctrlVehicleProps["quaternion"],
): QuatTuple {
    if (quaternion) {
        if (Array.isArray(quaternion)) {
            const q = new THREE.Quaternion(
                Number(quaternion[0] ?? 0),
                Number(quaternion[1] ?? 0),
                Number(quaternion[2] ?? 0),
                Number(quaternion[3] ?? 1),
            ).normalize();
            return [q.x, q.y, q.z, q.w];
        }
        if (typeof quaternion === "object" && "x" in quaternion) {
            const qObj = quaternion as { x: number; y: number; z: number; w: number };
            const q = new THREE.Quaternion(qObj.x, qObj.y, qObj.z, qObj.w).normalize();
            return [q.x, q.y, q.z, q.w];
        }
    }
    if (rotation) {
        const euler = new THREE.Euler();
        if (rotation instanceof THREE.Euler) {
            euler.copy(rotation);
        }
        else if (Array.isArray(rotation)) {
            euler.set(
                Number(rotation[0] ?? 0),
                Number(rotation[1] ?? 0),
                Number(rotation[2] ?? 0),
                typeof rotation[3] === "string" ? (rotation[3] as THREE.EulerOrder) : "XYZ",
            );
        }
        else if (typeof rotation === "object" && "x" in rotation) {
            const rObj = rotation as { x: number; y: number; z: number; order?: THREE.EulerOrder };
            euler.set(Number(rObj.x ?? 0), Number(rObj.y ?? 0), Number(rObj.z ?? 0), rObj.order ?? "XYZ");
        }
        const q = new THREE.Quaternion().setFromEuler(euler).normalize();
        return [q.x, q.y, q.z, q.w];
    }
    return [0, 0, 0, 1];
}

export interface ChassisMassSummary extends LumpedMassProps {
    linearVelocity?: Vec3Tuple;
    angularVelocity?: Vec3Tuple;
    gravityFactor?: number;
}

export function computeChassisMassProps(bodies: readonly KhrBodySpec[]): ChassisMassSummary {
    const allDyn = bodies.filter((b) => !b.isKinematic);
    const dyn = allDyn.filter((b) => b.joint === null);
    if (dyn.length === 0) {
        throw new KhrValidationError("KhrVehicle: bundle has no dynamic rigid body for the chassis");
    }
    const lumped = computeLumpedMassProps(dyn);
    if (!lumped || !(lumped.mass > 0)) {
        throw new KhrValidationError("KhrVehicle: chassis dynamic mass must be positive");
    }
    if (!lumped.inertia.every((v) => Number.isFinite(v) && v > 0)) {
        throw new KhrValidationError("KhrVehicle: chassis principal inertia must be positive and finite");
    }

    let gravityFactor: number | undefined;
    if (dyn.some((b) => b.gravityFactor !== undefined)) {
        const g0 = dyn[0].gravityFactor ?? 1;
        for (const b of dyn) {
            if (Math.abs((b.gravityFactor ?? 1) - g0) > 1e-6) {
                throw new KhrValidationError(
                    `KhrVehicle: all dynamic chassis bodies must share gravityFactor (got ${g0} vs ${b.gravityFactor ?? 1} on "${b.nodeName}")`,
                );
            }
        }
        gravityFactor = g0;
    }

    let linearVelocity: Vec3Tuple | undefined;
    if (dyn.some((b) => b.linearVelocity !== undefined)) {
        let vx = 0;
        let vy = 0;
        let vz = 0;
        for (const b of dyn) {
            const lv = b.linearVelocity ?? [0, 0, 0];
            vx += lv[0] * b.mass;
            vy += lv[1] * b.mass;
            vz += lv[2] * b.mass;
        }
        linearVelocity = [vx / lumped.mass, vy / lumped.mass, vz / lumped.mass];
    }

    const rootDyn = dyn.find((b) => !b.joint && b.angularVelocity !== undefined) ?? dyn.find((b) => b.angularVelocity !== undefined);
    const angularVelocity: Vec3Tuple | undefined = rootDyn?.angularVelocity
        ? [rootDyn.angularVelocity[0], rootDyn.angularVelocity[1], rootDyn.angularVelocity[2]]
        : undefined;

    return {
        mass: lumped.mass,
        com: lumped.com,
        inertia: lumped.inertia,
        inertiaFrame: lumped.inertiaFrame,
        ...(linearVelocity ? { linearVelocity } : {}),
        ...(angularVelocity ? { angularVelocity } : {}),
        ...(gravityFactor !== undefined ? { gravityFactor } : {}),
    };
}

export function buildChassisColliders(
    bodies: readonly KhrBodySpec[],
    chassis: LumpedMassProps,
): Array<{ key: string; spec: KhrColliderSpec; massProperties?: ColliderMass }> {
    const out: Array<{ key: string; spec: KhrColliderSpec; massProperties?: ColliderMass }> = [];
    const one = new THREE.Vector3(1, 1, 1);
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scl = new THREE.Vector3();

    for (const b of bodies) {
        if (b.isKinematic || b.joint !== null)
            continue;
        const bodyMat = bodyPoseMatrix(b);
        for (let idx = 0; idx < b.colliders.length; idx++) {
            const c = b.colliders[idx];
            const colMat = new THREE.Matrix4().compose(
                new THREE.Vector3().fromArray(c.localPosition),
                new THREE.Quaternion().fromArray(c.localQuaternion),
                one,
            );
            bodyMat.clone().multiply(colMat).decompose(pos, quat, scl);
            quat.normalize();
            const isFirst = out.length === 0;
            const rigidColMat = isFirst ? new THREE.Matrix4().compose(pos, quat, one) : undefined;
            out.push({
                key: `${b.nodeName}:${idx}`,
                spec: {
                    ...c,
                    localPosition: [pos.x, pos.y, pos.z],
                    localQuaternion: [quat.x, quat.y, quat.z, quat.w],
                },
                ...(isFirst ? { massProperties: toColliderMass(chassis, rigidColMat, quat.clone()) } : {}),
            });
        }
    }
    if (out.length === 0) {
        throw new KhrValidationError("KhrVehicle: chassis has no colliders");
    }
    return out;
}

export function computeRotorLocalMatrix(
    node: THREE.Object3D,
    scene: THREE.Object3D,
    center: readonly [number, number, number],
    quaternion?: readonly [number, number, number, number],
): THREE.Matrix4 {
    const nodeToScene = new THREE.Matrix4();
    let cur: THREE.Object3D | null = node;
    while (cur && cur !== scene) {
        if (!cur.parent) {
            throw new KhrValidationError(`Rotor node "${node.name}" is detached from bundle.scene`);
        }
        cur.updateMatrix();
        nodeToScene.premultiply(cur.matrix);
        cur = cur.parent;
    }
    if (cur !== scene && node !== scene) {
        throw new KhrValidationError(`Rotor node "${node.name}" is not under bundle.scene`);
    }
    const slotInScene = new THREE.Matrix4().compose(
        new THREE.Vector3(center[0], center[1], center[2]),
        quaternion
            ? new THREE.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]).normalize()
            : new THREE.Quaternion(),
        new THREE.Vector3(1, 1, 1),
    );
    return slotInScene.invert().multiply(nodeToScene);
}
