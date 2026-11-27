import * as THREE from "three";
import type RAPIER from "@dimforge/rapier3d-compat";
import { type KhrBreakawaySpec, type KhrJointDriveSpec, type KhrJointSpec, KhrValidationError } from "./khrTypes.js";
import type { JointRuntime } from "./physicsEcs.js";

/**
 * Rapier interaction group bitmask for attached vehicle articulated panels:
 * member of group 14 (`0x4000`), collides with all groups except 14 (`0xbfff`),
 * preventing adjacent hinged panels (e.g. `door_fl` <-> `door_rl` / `hood`) from
 * fighting across joints while colliding normally with world geometry and projectiles.
 */
export const VEHICLE_PANEL_INTERACTION_GROUPS = 0x0002fffd;

const _q1 = new THREE.Quaternion();
const _q2 = new THREE.Quaternion();
const _qRel = new THREE.Quaternion();
const _restInv = new THREE.Quaternion();
const _deltaPos = new THREE.Vector3();
const _axWorld = new THREE.Vector3();

export function createRapierImpulseJoint(
    rapier: typeof RAPIER,
    world: RAPIER.World,
    parentBody: RAPIER.RigidBody,
    childBody: RAPIER.RigidBody,
    joint: KhrJointSpec,
    anchorParent: readonly [number, number, number],
    anchorChild: readonly [number, number, number],
    axisParent: readonly [number, number, number],
    axisChild: readonly [number, number, number],
    restFrameRelQuat: readonly [number, number, number, number],
): RAPIER.ImpulseJoint {
    const a1 = { x: anchorParent[0], y: anchorParent[1], z: anchorParent[2] };
    const a2 = { x: anchorChild[0], y: anchorChild[1], z: anchorChild[2] };
    const qRel = _qRel.set(restFrameRelQuat[0], restFrameRelQuat[1], restFrameRelQuat[2], restFrameRelQuat[3]).normalize();
    const axFromChild = _axWorld.set(axisChild[0], axisChild[1], axisChild[2]).applyQuaternion(qRel);
    const ax = (axFromChild.lengthSq() > 1e-12 ? axFromChild : _axWorld.set(axisParent[0], axisParent[1], axisParent[2])).normalize();
    let data: RAPIER.JointData;
    if (joint.kind === "fixed") {
        const pr = parentBody.rotation();
        const cr = childBody.rotation();
        const bodyRel = _q1.set(pr.x, pr.y, pr.z, pr.w).invert().multiply(_q2.set(cr.x, cr.y, cr.z, cr.w)).normalize();
        const useBodyRel = Math.abs(bodyRel.x) + Math.abs(bodyRel.y) + Math.abs(bodyRel.z) <= 1e-6 && Math.abs(Math.abs(bodyRel.w) - 1) <= 1e-6;
        const f1 = useBodyRel
            ? { x: bodyRel.x, y: bodyRel.y, z: bodyRel.z, w: bodyRel.w }
            : { x: qRel.x, y: qRel.y, z: qRel.z, w: qRel.w };
        data = rapier.JointData.fixed(
            a1,
            f1,
            a2,
            { x: 0, y: 0, z: 0, w: 1 },
        );
    }
    else if (joint.kind === "revolute") {
        data = rapier.JointData.revolute(a1, a2, { x: ax.x, y: ax.y, z: ax.z });
        if (joint.limits) {
            data.limitsEnabled = true;
            data.limits = [joint.limits[0], joint.limits[1]];
        }
    }
    else {
        data = rapier.JointData.prismatic(a1, a2, { x: ax.x, y: ax.y, z: ax.z });
        if (joint.limits) {
            data.limitsEnabled = true;
            data.limits = [joint.limits[0], joint.limits[1]];
        }
    }
    const imp = world.createImpulseJoint(data, parentBody, childBody, true);
    imp.setContactsEnabled(Boolean(joint.enableCollision));
    configureJointDrive(rapier, imp, joint.kind, joint.drive, joint.drive?.positionTarget ?? null);
    return imp;
}

export function configureJointDrive(
    rapier: typeof RAPIER,
    imp: RAPIER.ImpulseJoint,
    kind: KhrJointSpec["kind"],
    drive: KhrJointDriveSpec | null,
    targetPos: number | null,
): void {
    if (kind === "fixed" || !drive)
        return;
    const unit = imp as RAPIER.UnitImpulseJoint & { setMotorMaxForce?: (maxForce: number) => void };
    unit.configureMotorModel(
        drive.mode === "force" ? rapier.MotorModel.ForceBased : rapier.MotorModel.AccelerationBased,
    );
    const effTargetPos = targetPos ?? drive.positionTarget ?? (drive.stiffness !== undefined && drive.stiffness > 0 ? 0 : null);
    if (effTargetPos !== null && drive.stiffness !== undefined) {
        if (drive.velocityTarget !== undefined) {
            unit.configureMotor(effTargetPos, drive.velocityTarget, drive.stiffness, drive.damping ?? 0);
        }
        else {
            unit.configureMotorPosition(effTargetPos, drive.stiffness, drive.damping ?? 0);
        }
    }
    else if (drive.velocityTarget !== undefined) {
        unit.configureMotorVelocity(drive.velocityTarget, drive.damping ?? 0);
    }
    if (drive.maxForce !== undefined) {
        unit.setMotorMaxForce?.(drive.maxForce);
    }
}

export function measureJointCoordinate(
    parentBody: RAPIER.RigidBody | null,
    childBody: RAPIER.RigidBody | null,
    kind: KhrJointSpec["kind"],
    axisParent: readonly [number, number, number],
    anchorParent: readonly [number, number, number],
    restRelQuat: readonly [number, number, number, number],
): number {
    if (!parentBody || !childBody || kind === "fixed")
        return 0;
    const pr = parentBody.rotation();
    const cr = childBody.rotation();
    _q1.set(pr.x, pr.y, pr.z, pr.w);
    _q2.set(cr.x, cr.y, cr.z, cr.w);
    if (kind === "prismatic") {
        const pt = parentBody.translation();
        const ct = childBody.translation();
        _deltaPos.set(anchorParent[0], anchorParent[1], anchorParent[2]).applyQuaternion(_q1);
        _deltaPos.set(ct.x - (pt.x + _deltaPos.x), ct.y - (pt.y + _deltaPos.y), ct.z - (pt.z + _deltaPos.z));
        _axWorld.set(axisParent[0], axisParent[1], axisParent[2]).applyQuaternion(_q1).normalize();
        return _deltaPos.dot(_axWorld);
    }
    _restInv.set(restRelQuat[0], restRelQuat[1], restRelQuat[2], restRelQuat[3]).invert();
    _qRel.copy(_q1).invert().multiply(_q2).premultiply(_restInv).normalize();
    if (_qRel.w < 0)
        _qRel.set(-_qRel.x, -_qRel.y, -_qRel.z, -_qRel.w);
    const sinHalf = _qRel.x * axisParent[0] + _qRel.y * axisParent[1] + _qRel.z * axisParent[2];
    return 2 * Math.atan2(sinHalf, _qRel.w);
}

export function buildJointRuntime(params: {
    rapier: typeof RAPIER;
    world?: RAPIER.World;
    nodeName: string;
    role: string | null;
    kind: KhrJointSpec["kind"];
    object: THREE.Object3D;
    axisParent: readonly [number, number, number];
    anchorParent: readonly [number, number, number];
    restRelQuat: readonly [number, number, number, number];
    limits: [number, number] | null;
    drive: KhrJointDriveSpec | null;
    breakaway?: KhrBreakawaySpec;
    rapierJoint: RAPIER.ImpulseJoint;
    parentBody: RAPIER.RigidBody;
    childBody: RAPIER.RigidBody;
}): JointRuntime {
    const { rapier, world, nodeName, role, kind, object, axisParent, anchorParent, restRelQuat, limits, drive, breakaway, rapierJoint, parentBody, childBody } = params;
    const rt: JointRuntime = {
        nodeName,
        role,
        kind,
        object,
        axis: new THREE.Vector3(axisParent[0], axisParent[1], axisParent[2]).normalize(),
        limits,
        drive,
        positionTarget: drive?.positionTarget ?? null,
        velocityTarget: drive?.velocityTarget ?? 0,
        rapierJoint,
        parentBody,
        childBody,
        unlatched: false,
        detached: false,
        getCoordinate: () => measureJointCoordinate(rt.parentBody, rt.childBody, kind, axisParent, anchorParent, restRelQuat),
        applyTarget: (target: number) => {
            if (rt.detached || !rt.rapierJoint)
                return;
            if (!drive || drive.stiffness === undefined) {
                throw new KhrValidationError(`Joint "${nodeName}" has no authored drive stiffness; cannot set a position target`);
            }
            rt.unlatched = false;
            rt.positionTarget = target;
            configureJointDrive(rapier, rt.rapierJoint, kind, drive, target);
            if (parentBody.isDynamic())
                parentBody.wakeUp();
            if (childBody.isDynamic())
                childBody.wakeUp();
        },
        applyImpactImpulse: (impulseNs: number) => {
            if (rt.detached || !breakaway || !(impulseNs > 0))
                return rt.detached ? "detached" : rt.unlatched ? "unlatched" : "intact";
            if (impulseNs >= breakaway.breakImpulseNs) {
                if (world && rt.rapierJoint && world.getImpulseJoint(rt.rapierJoint.handle)) {
                    world.removeImpulseJoint(rt.rapierJoint, true);
                }
                rt.rapierJoint = null;
                rt.detached = true;
                rt.unlatched = true;
                rt.positionTarget = null;
                const ud = childBody.userData as { ecctrl?: Record<string, unknown> } | undefined;
                if (ud?.ecctrl) {
                    childBody.userData = { ...ud, ecctrl: { ...ud.ecctrl, excludeVehicleRay: false } };
                }
                for (let i = 0; i < childBody.numColliders(); i++) {
                    const col = childBody.collider(i);
                    const cg = col.collisionGroups();
                    if (cg === VEHICLE_PANEL_INTERACTION_GROUPS || cg === 0x4000bfff) {
                        col.setCollisionGroups(0xffffffff);
                        col.setSolverGroups(0xffffffff);
                    }
                }
                if (childBody.isDynamic())
                    childBody.wakeUp();
                return "detached";
            }
            if (breakaway.latchImpulseNs !== undefined && impulseNs >= breakaway.latchImpulseNs && !rt.unlatched) {
                rt.unlatched = true;
                rt.positionTarget = null;
                if (kind !== "fixed" && rt.rapierJoint) {
                    const unit = rt.rapierJoint as RAPIER.UnitImpulseJoint;
                    unit.configureMotorPosition(0, 0, Math.min(drive?.damping ?? 1, 1.5));
                }
                if (childBody.isDynamic())
                    childBody.wakeUp();
                return "unlatched";
            }
            return rt.unlatched ? "unlatched" : "intact";
        },
    };
    return rt;
}
