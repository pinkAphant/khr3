import type * as THREE from "three";
import type { KhrCurveData } from "./khrConfigTypes.js";

export interface OmiVehicleWheelSpec {
    radius: number;
    width: number;
    maxForce?: number;
    steering?: boolean;
    propulsion?: boolean;
    maxSteeringAngle?: number;
    suspensionStiffness?: number;
    suspensionDampingCompression?: number;
    suspensionDampingRebound?: number;
    suspensionTravel?: number;
}

export interface EcctrlWheelTag {
    nodeName: string;
    object: THREE.Object3D;
    position: [number, number, number];
    quaternion?: [number, number, number, number];
    radius: number;
    width: number;
    steer: boolean;
    steerInvert?: boolean;
    drive: boolean;
    driveInvert?: boolean;
    brake: boolean;
    travel: number;
    sag: number;
    springK?: number;
    dampingC?: number;
    maxBrakeTorque?: number;
    maxSteeringAngle?: number;
    rollingResistanceCoef?: number;
    wheelDensity?: number;
    driveTorqueWeight?: number;
    tireGripFactor?: number;
    latFrictionEllipseScale?: number;
    relaxLngRate?: number;
    relaxLatRate?: number;
    lngSlipRatioCurveData?: KhrCurveData;
    latSlipRatioCurveData?: KhrCurveData;
}

export interface EcctrlPropellerTag {
    nodeName: string;
    object: THREE.Object3D;
    position: [number, number, number];
    quaternion?: [number, number, number, number];
    maxThrust: number;
    torqueRatio?: number;
    invertThrust?: boolean;
    invertTorque?: boolean;
}
