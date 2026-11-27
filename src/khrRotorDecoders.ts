import * as THREE from "three";
import { assertIdentityWorldScale } from "./khrInertia.js";
import { type EcctrlPropellerTag, type EcctrlWheelTag, type KhrCurveData, KhrValidationError } from "./khrTypes.js";

const _quat = new THREE.Quaternion();
const _invParentQuat = new THREE.Quaternion();

const isObj = (v: unknown): v is Record<string, unknown> => typeof v === "object" && !!v && !Array.isArray(v);
const isFiniteNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);

function getRelativeQuatArray(obj: THREE.Object3D, parent?: THREE.Object3D): [number, number, number, number] {
    obj.updateWorldMatrix(true, false);
    obj.getWorldQuaternion(_quat);
    if (parent) {
        parent.updateWorldMatrix(true, false);
        _quat.premultiply(parent.getWorldQuaternion(_invParentQuat).invert());
    }
    if (_quat.lengthSq() > 0)
        _quat.normalize();
    else
        _quat.identity();
    return [_quat.x || 0, _quat.y || 0, _quat.z || 0, _quat.w || 0];
}

function optNum(obj: Record<string, unknown>, key: string, tagKind: string, owner: string, strictlyPos = false): number | undefined {
    const val = obj[key];
    if (val === undefined)
        return undefined;
    if (!isFiniteNum(val) || (strictlyPos ? val <= 0 : val < 0)) {
        throw new KhrValidationError(`${tagKind} "${owner}" has invalid ${key}: ${tagKind}.${key}`);
    }
    return val;
}

function reqNum(obj: Record<string, unknown>, key: string, tagKind: string, owner: string, strictlyPos = false): number {
    const val = obj[key];
    if (!isFiniteNum(val) || (strictlyPos ? val <= 0 : val < 0)) {
        throw new KhrValidationError(`${tagKind} "${owner}" is missing valid ${tagKind}.${key}`);
    }
    return val;
}

function optBool(obj: Record<string, unknown>, key: string, tagKind: string, owner: string): boolean | undefined {
    const val = obj[key];
    if (val === undefined)
        return undefined;
    if (typeof val !== "boolean") {
        throw new KhrValidationError(`${tagKind} "${owner}" has non-boolean ${tagKind}.${key}`);
    }
    return val;
}

function optBoolOrNumFlag(
    obj: Record<string, unknown>,
    key: string,
    tagKind: string,
    owner: string,
): { flag: boolean; numeric?: number } | undefined {
    const val = obj[key];
    if (val === undefined)
        return undefined;
    if (typeof val === "boolean")
        return { flag: val };
    if (isFiniteNum(val) && val >= 0)
        return { flag: val !== 0, numeric: val };
    throw new KhrValidationError(`${tagKind} "${owner}" has invalid ${tagKind}.${key}`);
}

function reqBool(obj: Record<string, unknown>, key: string, tagKind: string, owner: string): boolean {
    const val = obj[key];
    if (typeof val !== "boolean") {
        throw new KhrValidationError(`${tagKind} "${owner}" is missing boolean ${tagKind}.${key}`);
    }
    return val;
}

function optCurve(val: unknown, key: string, owner: string): KhrCurveData | undefined {
    if (val === undefined)
        return undefined;
    if (!isObj(val) || !Array.isArray(val.points) || val.points.length === 0) {
        throw new KhrValidationError(`ecctrlWheel "${owner}" has invalid ecctrlWheel.${key}`);
    }
    for (const pt of val.points) {
        if (!isObj(pt) || !isFiniteNum(pt.x) || !isFiniteNum(pt.y)) {
            throw new KhrValidationError(`ecctrlWheel "${owner}" has invalid point in ecctrlWheel.${key}`);
        }
    }
    if (val.samples !== undefined && (!isFiniteNum(val.samples) || val.samples <= 0)) {
        throw new KhrValidationError(`ecctrlWheel "${owner}" has invalid samples in ecctrlWheel.${key}`);
    }
    return val as unknown as KhrCurveData;
}

export function decodeWheelTag(
    obj: THREE.Object3D,
    name: string,
    rawWheel: unknown,
    scene?: THREE.Object3D,
    rawOmiWheel?: unknown,
): EcctrlWheelTag {
    assertIdentityWorldScale(obj, name);
    const hasOmi = rawOmiWheel !== undefined;
    if (hasOmi && !isObj(rawOmiWheel))
        throw new KhrValidationError(`OMI_vehicle_wheel "${name}" must be an object`);
    if (rawWheel !== undefined && !isObj(rawWheel))
        throw new KhrValidationError(`ecctrlWheel "${name}" must be an object`);
    if (!hasOmi && rawWheel === undefined)
        throw new KhrValidationError(`ecctrlWheel "${name}" must be an object`);

    const ec = isObj(rawWheel) ? rawWheel : {};
    const omi = isObj(rawOmiWheel) ? rawOmiWheel : undefined;

    const radius = omi
        ? optNum(ec, "radius", "ecctrlWheel", name, true) ?? reqNum(omi, "radius", "OMI_vehicle_wheel", name, true)
        : reqNum(ec, "radius", "ecctrlWheel", name, true);
    const width = omi
        ? optNum(ec, "width", "ecctrlWheel", name, true) ?? reqNum(omi, "width", "OMI_vehicle_wheel", name, true)
        : reqNum(ec, "width", "ecctrlWheel", name, true);
    if (omi) {
        reqNum(omi, "radius", "OMI_vehicle_wheel", name, true);
        reqNum(omi, "width", "OMI_vehicle_wheel", name, true);
    }

    const omiTravel = omi ? optNum(omi, "suspensionTravel", "OMI_vehicle_wheel", name) : undefined;
    const omiMaxForce = omi ? optNum(omi, "maxForce", "OMI_vehicle_wheel", name) : undefined;
    const omiMaxSteerRaw = omi ? optNum(omi, "maxSteeringAngle", "OMI_vehicle_wheel", name) : undefined;
    const omiStiffness = omi ? optNum(omi, "suspensionStiffness", "OMI_vehicle_wheel", name) : undefined;
    const omiDampComp = omi ? optNum(omi, "suspensionDampingCompression", "OMI_vehicle_wheel", name) : undefined;
    const omiDampReb = omi ? optNum(omi, "suspensionDampingRebound", "OMI_vehicle_wheel", name) : undefined;
    const omiSteer = omi ? optBoolOrNumFlag(omi, "steering", "OMI_vehicle_wheel", name) : undefined;
    const omiProp = omi ? optBoolOrNumFlag(omi, "propulsion", "OMI_vehicle_wheel", name) : undefined;
    const omiSteering = omiSteer?.flag;
    const omiPropulsion = omiProp?.flag;
    const omiMaxSteer = omiMaxSteerRaw ?? (omiSteer?.numeric !== undefined && omiSteer.numeric > 0 ? omiSteer.numeric : undefined);

    const travel = omi ? optNum(ec, "travel", "ecctrlWheel", name) ?? omiTravel ?? 0.25 : reqNum(ec, "travel", "ecctrlWheel", name);
    const sag = omi
        ? optNum(ec, "sag", "ecctrlWheel", name) ?? (travel > 0 ? Math.min(travel * 0.4, 0.1) : 0)
        : reqNum(ec, "sag", "ecctrlWheel", name);
    if (sag > travel) {
        throw new KhrValidationError(`ecctrlWheel "${name}" has invalid sag (${sag}) > travel (${travel})`);
    }
    const steer = omi
        ? optBool(ec, "steer", "ecctrlWheel", name) ?? omiSteering ?? (omiMaxSteer !== undefined ? omiMaxSteer > 0 : false)
        : reqBool(ec, "steer", "ecctrlWheel", name);
    const drive = omi
        ? optBool(ec, "drive", "ecctrlWheel", name) ?? omiPropulsion ?? false
        : reqBool(ec, "drive", "ecctrlWheel", name);
    const brake = omi
        ? optBool(ec, "brake", "ecctrlWheel", name) ?? true
        : reqBool(ec, "brake", "ecctrlWheel", name);

    const omiDamping = omiDampComp !== undefined && omiDampReb !== undefined
        ? 0.5 * (omiDampComp + omiDampReb)
        : (omiDampComp ?? omiDampReb);
    const springK = optNum(ec, "springK", "ecctrlWheel", name) ?? omiStiffness;
    const dampingC = optNum(ec, "dampingC", "ecctrlWheel", name) ?? omiDamping;
    const maxBrakeTorque = optNum(ec, "maxBrakeTorque", "ecctrlWheel", name)
        ?? (omiMaxForce !== undefined ? omiMaxForce * radius : undefined);
    const maxSteeringAngle = optNum(ec, "maxSteeringAngle", "ecctrlWheel", name) ?? omiMaxSteer;

    const box = new THREE.Box3().setFromObject(obj);
    const center = box.isEmpty() ? obj.getWorldPosition(new THREE.Vector3()) : box.getCenter(new THREE.Vector3());
    return {
        nodeName: name,
        object: obj,
        position: [center.x, center.y, center.z],
        quaternion: getRelativeQuatArray(obj, scene),
        radius,
        width,
        steer,
        steerInvert: optBool(ec, "steerInvert", "ecctrlWheel", name),
        drive,
        driveInvert: optBool(ec, "driveInvert", "ecctrlWheel", name),
        brake,
        travel,
        sag,
        springK,
        dampingC,
        maxBrakeTorque,
        ...(maxSteeringAngle === undefined ? {} : { maxSteeringAngle }),
        rollingResistanceCoef: optNum(ec, "rollingResistanceCoef", "ecctrlWheel", name),
        wheelDensity: optNum(ec, "wheelDensity", "ecctrlWheel", name, true),
        driveTorqueWeight: optNum(ec, "driveTorqueWeight", "ecctrlWheel", name),
        tireGripFactor: optNum(ec, "tireGripFactor", "ecctrlWheel", name),
        latFrictionEllipseScale: optNum(ec, "latFrictionEllipseScale", "ecctrlWheel", name),
        relaxLngRate: optNum(ec, "relaxLngRate", "ecctrlWheel", name),
        relaxLatRate: optNum(ec, "relaxLatRate", "ecctrlWheel", name),
        lngSlipRatioCurveData: optCurve(ec.lngSlipRatioCurveData, "lngSlipRatioCurveData", name),
        latSlipRatioCurveData: optCurve(ec.latSlipRatioCurveData, "latSlipRatioCurveData", name),
    };
}

export function decodePropellerTag(
    obj: THREE.Object3D,
    name: string,
    rawPropeller: unknown,
    scene?: THREE.Object3D,
): EcctrlPropellerTag {
    assertIdentityWorldScale(obj, name);
    if (!isObj(rawPropeller))
        throw new KhrValidationError(`ecctrlPropeller "${name}" must be an object`);
    if (!isFiniteNum(rawPropeller.maxThrust) || rawPropeller.maxThrust <= 0) {
        throw new KhrValidationError(`ecctrlPropeller "${name}" is missing positive finite ecctrlPropeller.maxThrust`);
    }
    let torqueRatio: number | undefined;
    if (rawPropeller.torqueRatio !== undefined) {
        if (!isFiniteNum(rawPropeller.torqueRatio)) {
            throw new KhrValidationError(`ecctrlPropeller "${name}" has invalid ecctrlPropeller.torqueRatio`);
        }
        torqueRatio = rawPropeller.torqueRatio;
    }
    const box = new THREE.Box3().setFromObject(obj);
    const center = box.isEmpty() ? obj.getWorldPosition(new THREE.Vector3()) : box.getCenter(new THREE.Vector3());
    return {
        nodeName: name,
        object: obj,
        position: [center.x, center.y, center.z],
        quaternion: getRelativeQuatArray(obj, scene),
        maxThrust: rawPropeller.maxThrust,
        torqueRatio,
        invertThrust: optBool(rawPropeller, "invertThrust", "ecctrlPropeller", name),
        invertTorque: optBool(rawPropeller, "invertTorque", "ecctrlPropeller", name),
    };
}
