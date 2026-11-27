import * as THREE from "three";
import type RAPIER from "@dimforge/rapier3d-compat";
import { toColliderMass, type ColliderMass, type PlannedCollider } from "./assemblyPlan.js";
import { bodyPoseMatrix, computeLumpedMassProps } from "./khrMassProps.js";
import { type KhrBodySpec, type KhrJointSpec, KhrValidationError } from "./khrTypes.js";
import type { JointRuntime } from "./physicsEcs.js";

/** Membership group 1 (0x0002), filter ~0x0002 (0xfffd): collides with world/actors, never with sibling attached vehicle panels. */
export const ATTACHED_PANEL_COLLISION_GROUPS = 0x0002fffd;
export const DETACHED_PANEL_COLLISION_GROUPS = 0xffffffff;

type Vec3 = [number, number, number];
type Quat = [number, number, number, number];

export interface PlannedVehiclePart {
    spec: KhrBodySpec;
    joint: KhrJointSpec;
    parent: PlannedVehiclePart | null;
    restRelParentPos: Vec3;
    restRelParentQuat: Quat;
    restRelChassisPos: Vec3;
    restRelChassisQuat: Quat;
    axisParent: Vec3;
    axisChild: Vec3;
    partMass?: ColliderMass;
    colliders: PlannedCollider[];
}

export function planVehicleJointParts(bodies: readonly KhrBodySpec[]): PlannedVehiclePart[] {
    const dyn = bodies.filter((b) => !b.isKinematic);
    const byName = new Map(dyn.map((b) => [b.nodeName, b]));
    const jointed = dyn.filter((b) => b.joint !== null);
    if (jointed.length === 0) return [];

    const depthOf = (b: KhrBodySpec, seen = new Set<string>()): number => {
        if (!b.joint) return 0;
        if (seen.has(b.nodeName)) throw new KhrValidationError(`Joint cycle through "${b.nodeName}"`);
        seen.add(b.nodeName);
        const parent = byName.get(b.joint.connectedNodeName);
        if (!parent) {
            throw new KhrValidationError(`Vehicle joint on "${b.nodeName}" targets "${b.joint.connectedNodeName}", which is not a dynamic KHR body`);
        }
        return 1 + depthOf(parent, seen);
    };

    jointed.sort((a, b) => depthOf(a) - depthOf(b));
    const one = new THREE.Vector3(1, 1, 1);
    const rpPos = new THREE.Vector3(), rpQuat = new THREE.Quaternion();
    const rcPos = new THREE.Vector3(), rcQuat = new THREE.Quaternion(), scl = new THREE.Vector3();
    const partByName = new Map<string, PlannedVehiclePart>();

    return jointed.map((b) => {
        const joint = b.joint!;
        const parent = partByName.get(joint.connectedNodeName) ?? null;
        const bodyMat = bodyPoseMatrix(b);
        const parentMat = parent ? bodyPoseMatrix(parent.spec) : new THREE.Matrix4();
        parentMat.clone().invert().multiply(bodyMat).decompose(rpPos, rpQuat, scl);
        rpQuat.normalize();
        bodyMat.decompose(rcPos, rcQuat, scl);
        rcQuat.normalize();

        const axChildVec = new THREE.Vector3().fromArray(joint.axis).normalize();
        const axParentVec = axChildVec.clone().applyQuaternion(rcQuat).normalize();
        const anchorParent: Vec3 = parent
            ? [rcPos.x - parent.restRelChassisPos[0], rcPos.y - parent.restRelChassisPos[1], rcPos.z - parent.restRelChassisPos[2]]
            : [rpPos.x, rpPos.y, rpPos.z];
        const bodyAlignInv = new THREE.Matrix4().compose(new THREE.Vector3().fromArray(b.localPosition), new THREE.Quaternion(), one).invert();
        const ownLumped = computeLumpedMassProps([b], bodyAlignInv);
        const frameMat = new THREE.Matrix4().makeRotationFromQuaternion(rcQuat);
        const cPos = new THREE.Vector3(), cQuat = new THREE.Quaternion();
        const colliders: PlannedCollider[] = b.colliders.map((spec, idx) => {
            const colQuat = new THREE.Quaternion().fromArray(spec.localQuaternion);
            const local = new THREE.Matrix4().compose(new THREE.Vector3().fromArray(spec.localPosition), colQuat, one).premultiply(frameMat);
            local.decompose(cPos, cQuat, scl);
            cQuat.normalize();
            const planned: PlannedCollider = {
                spec: spec.collisionGroups === undefined ? { ...spec, collisionGroups: ATTACHED_PANEL_COLLISION_GROUPS } : spec,
                local,
                position: [cPos.x, cPos.y, cPos.z],
                quaternion: [cQuat.x, cQuat.y, cQuat.z, cQuat.w],
            };
            if (idx === 0 && ownLumped) planned.massProperties = toColliderMass(ownLumped, local, cQuat);
            return planned;
        });

        const plan: PlannedVehiclePart = {
            spec: b, joint, parent,
            restRelParentPos: anchorParent,
            restRelParentQuat: [rcQuat.x, rcQuat.y, rcQuat.z, rcQuat.w],
            restRelChassisPos: [rcPos.x, rcPos.y, rcPos.z],
            restRelChassisQuat: [rcQuat.x, rcQuat.y, rcQuat.z, rcQuat.w],
            axisParent: [axParentVec.x, axParentVec.y, axParentVec.z],
            axisChild: [axChildVec.x, axChildVec.y, axChildVec.z],
            ...(ownLumped && colliders.length === 0 ? { partMass: toColliderMass(ownLumped) } : {}),
            colliders,
        };
        partByName.set(b.nodeName, plan);
        return plan;
    });
}

export function openTargetForLimits(limits: readonly [number, number]): number {
    return Math.abs(limits[0]) > Math.abs(limits[1]) ? limits[0] : limits[1];
}

export function isVehicleJointOpen(rt: JointRuntime): boolean {
    if (!rt.limits) return false;
    const openTgt = openTargetForLimits(rt.limits);
    return Math.abs(openTgt) > 1e-6 && Math.abs((rt.positionTarget ?? rt.getCoordinate()) - openTgt) < Math.abs(openTgt) * 0.5;
}

const _pPos = new THREE.Vector3();
const _pQuat = new THREE.Quaternion();
const _rQuat = new THREE.Quaternion();
const _cQuat = new THREE.Quaternion();
const _anchor = new THREE.Vector3();

export function shiftAttachedPanels(
    parts: readonly PlannedVehiclePart[],
    bodyMap: ReadonlyMap<string, RAPIER.RigidBody>,
    runtimeMap: ReadonlyMap<string, JointRuntime>,
    dx: number,
    dy: number,
    dz: number,
): void {
    for (const part of parts) {
        const rb = bodyMap.get(part.spec.nodeName);
        const rt = runtimeMap.get(part.spec.nodeName);
        if (!rb || rt?.detached) continue;
        const t = rb.translation();
        rb.setTranslation({ x: t.x + dx, y: t.y + dy, z: t.z + dz }, true);
    }
}

export function syncAttachedPanelsToParent(
    parts: readonly PlannedVehiclePart[],
    bodyMap: ReadonlyMap<string, RAPIER.RigidBody>,
    runtimeMap: ReadonlyMap<string, JointRuntime>,
    parentBody: RAPIER.RigidBody | null,
): void {
    if (!parentBody) return;
    const pt = parentBody.translation(), pr = parentBody.rotation(), lv = parentBody.linvel(), av = parentBody.angvel();
    _pPos.set(pt.x, pt.y, pt.z);
    _pQuat.set(pr.x, pr.y, pr.z, pr.w);
    for (const part of parts) {
        const rb = bodyMap.get(part.spec.nodeName), rt = runtimeMap.get(part.spec.nodeName);
        if (!rb || rt?.detached) continue;
        _anchor.fromArray(part.restRelChassisPos).applyQuaternion(_pQuat).add(_pPos);
        _cQuat.copy(_pQuat).multiply(_rQuat.fromArray(part.restRelChassisQuat)).normalize();
        rb.setTranslation({ x: _anchor.x, y: _anchor.y, z: _anchor.z }, true);
        rb.setRotation({ x: _cQuat.x, y: _cQuat.y, z: _cQuat.z, w: _cQuat.w }, true);
        rb.setLinvel(lv, true);
        rb.setAngvel(av, true);
    }
}

export function stepVehicleArticulatedPanels(params: {
    parts: readonly PlannedVehiclePart[];
    bodyMap: ReadonlyMap<string, RAPIER.RigidBody>;
    runtimeMap: ReadonlyMap<string, JointRuntime>;
    detachedCleared: WeakSet<RAPIER.RigidBody>;
    enableCustomGravity: boolean;
    timestep: number;
    applyGravityField?: (body: RAPIER.RigidBody, timeStep: number) => void;
}): void {
    const { parts, bodyMap, runtimeMap, detachedCleared, enableCustomGravity, timestep, applyGravityField } = params;
    for (const part of parts) {
        const rb = bodyMap.get(part.spec.nodeName);
        if (!rb) continue;
        const rt = runtimeMap.get(part.spec.nodeName);
        if (rt?.detached && !detachedCleared.has(rb)) {
            detachedCleared.add(rb);
            rb.userData = {};
            const n = rb.numColliders();
            for (let i = 0; i < n; i++) {
                if (part.spec.colliders[i]?.collisionGroups === undefined) rb.collider(i).setCollisionGroups(DETACHED_PANEL_COLLISION_GROUPS);
            }
        }
        if (enableCustomGravity && timestep > 0 && applyGravityField && rb.isDynamic()) applyGravityField(rb, timestep);
    }
}
