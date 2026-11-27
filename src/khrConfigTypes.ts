import type { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

export class KhrValidationError extends Error {
    constructor(message: string) {
        super(message);
        this.name = "KhrValidationError";
    }
}

export interface KhrCurvePoint {
    x: number;
    y: number;
    r_in?: number;
    r_out?: number;
    w_in?: number;
    w_out?: number;
}

export interface KhrCurveData {
    points: KhrCurvePoint[];
    samples?: number;
}

export interface KhrCarConfig {
    controlMode?: "VELOCITY" | "POSITION";
    engineHorsepower?: number;
    engineMaxRPM?: number;
    gearRatios?: number[];
    finalDriveRatio?: number;
    transmissionMode?: "auto" | "manual";
    shiftUpRPM?: number;
    shiftDownRPM?: number;
    shiftCooldown?: number;
    steerRate?: number;
    maxSteerAngle?: number;
    reverseTorqueScale?: number;
    reverseRPMScale?: number;
    engineTorqueCurveData?: KhrCurveData;
    steerAngleCurveData?: KhrCurveData;
}

export interface KhrDroneConfig {
    controlMode?: "VELOCITY" | "POSITION";
    maxYawRate?: number;
    maxHorizSpeed?: number;
    maxVertSpeed?: number;
    maxTiltAngle?: number;
    airDragFactor?: number;
    TILT_P?: number;
    TILT_D?: number;
    YAW_POS_P?: number;
    YAW_VEL_P?: number;
    VERT_POS_P?: number;
    VERT_POS_D?: number;
    HORIZ_POS_P?: number;
    HORIZ_POS_D?: number;
    HORIZ_VEL_P?: number;
    VERT_VEL_P?: number;
}

export interface KhrPluginOptions {
    extractRole?: (nodeName: string, extras: Record<string, unknown>) => string | null;
    isSocketNode?: (nodeName: string, extras: Record<string, unknown>) => boolean | string;
}

export interface LoadKhrGlbOptions extends KhrPluginOptions {
    loader?: GLTFLoader;
}

export interface UseKhrGlbOptions extends KhrPluginOptions {
    extendLoader?: (loader: GLTFLoader) => void;
    cacheKey?: string;
}
