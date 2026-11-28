import { createContext, type MutableRefObject, type Ref } from "react";
import type * as THREE from "three";
import type RAPIER from "@dimforge/rapier3d-compat";
import type { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

export class KhrValidationError extends Error {
    constructor(message: string) {
        super(message);
        this.name = "KhrValidationError";
    }
}

export interface CurvePoint {
    x: number;
    y: number;
    r_in?: number;
    r_out?: number;
    w_in?: number;
    w_out?: number;
}

export interface CurveData {
    points: CurvePoint[];
    samples?: number;
}

export type KhrCurvePoint = CurvePoint;
export type KhrCurveData = CurveData;

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

export type KhrCombineMode = "average" | "minimum" | "maximum" | "multiply";

export type KhrImplicitShape =
    | { type: "box"; size: [number, number, number] }
    | { type: "sphere"; radius: number }
    | { type: "cylinder"; height: number; radiusTop: number; radiusBottom: number }
    | { type: "capsule"; height: number; radiusTop: number; radiusBottom: number };

export type KhrColliderShape =
    | KhrImplicitShape
    | { type: "convexHull"; vertices: Float32Array; meshNodeName: string }
    | { type: "trimesh"; vertices: Float32Array; indices: Uint32Array; meshNodeName: string };

export interface KhrColliderSpec {
    shape: KhrColliderShape;
    localPosition: [number, number, number];
    localQuaternion: [number, number, number, number];
    friction?: number;
    restitution?: number;
    frictionCombine?: KhrCombineMode;
    restitutionCombine?: KhrCombineMode;
    collisionGroups?: number;
}

export interface KhrJointDriveSpec {
    type?: "linear" | "angular";
    mode: "force" | "acceleration";
    axis?: 0 | 1 | 2;
    positionTarget?: number;
    velocityTarget?: number;
    stiffness?: number;
    damping?: number;
    maxForce?: number;
}

export interface KhrJointSpec {
    index: number;
    nodeName: string;
    connectedNodeName: string;
    kind: "revolute" | "prismatic" | "fixed";
    axis: [number, number, number];
    limits: [number, number] | null;
    limitStiffness?: number;
    limitDamping?: number;
    enableCollision: boolean;
    drive: KhrJointDriveSpec | null;
    drives: KhrJointDriveSpec[];
    anchor1: [number, number, number];
    anchor2: [number, number, number];
    role: string | null;
}

export interface KhrBreakawaySpec {
    latchImpulseNs?: number;
    breakImpulseNs: number;
}

export interface KhrBodySpec {
    nodeName: string;
    isKinematic: boolean;
    hasExplicitMotion?: boolean;
    hasExplicitCom?: boolean;
    hasExplicitInertia?: boolean;
    mass: number;
    centerOfMass: [number, number, number];
    inertiaDiagonal: [number, number, number];
    inertiaOrientation: [number, number, number, number];
    linearVelocity?: [number, number, number];
    angularVelocity?: [number, number, number];
    gravityFactor?: number;
    localPosition: [number, number, number];
    localQuaternion: [number, number, number, number];
    colliders: KhrColliderSpec[];
    joint: KhrJointSpec | null;
    breakaway?: KhrBreakawaySpec;
    object: THREE.Object3D;
    role: string | null;
}

export interface KhrAssetBundle {
    scene: THREE.Group;
    bodies: KhrBodySpec[];
    wheels: EcctrlWheelTag[];
    propellers: EcctrlPropellerTag[];
    sockets: Record<string, [number, number, number]>;
    totalDynamicMass: number;
    centerOfMass: [number, number, number];
    tune?: Partial<KhrCarConfig>;
    droneTune?: Partial<KhrDroneConfig>;
}

export interface KhrExportNodeSpec {
    nodeName: string;
    motion?: {
        isKinematic?: boolean;
        mass?: number;
        centerOfMass?: [number, number, number];
        inertiaDiagonal?: [number, number, number];
        inertiaOrientation?: [number, number, number, number];
        linearVelocity?: [number, number, number];
        angularVelocity?: [number, number, number];
        gravityFactor?: number;
    };
    collider?: {
        shape?: KhrImplicitShape;
        meshNodeName?: string;
        convexHull?: boolean;
        friction?: number;
        restitution?: number;
        frictionCombine?: KhrCombineMode;
        restitutionCombine?: KhrCombineMode;
        collisionSystems?: string[];
        collideWithSystems?: string[];
        notCollideWithSystems?: string[];
    };
    joint?: {
        connectedNodeName: string;
        kind: "revolute" | "prismatic" | "fixed";
        axis?: [number, number, number];
        limits?: [number, number] | null;
        limitStiffness?: number;
        limitDamping?: number;
        enableCollision?: boolean;
        drive?: KhrJointDriveSpec | null;
        drives?: KhrJointDriveSpec[];
    };
    wheel?: OmiVehicleWheelSpec;
    extras?: Record<string, unknown>;
    extensions?: Record<string, unknown>;
}

export interface JointRuntime {
    nodeName: string;
    role: string | null;
    kind: KhrJointSpec["kind"];
    object: THREE.Object3D;
    axis: THREE.Vector3;
    limits: [number, number] | null;
    drive: KhrJointDriveSpec | null;
    positionTarget: number | null;
    velocityTarget: number;
    rapierJoint: RAPIER.ImpulseJoint | null;
    parentBody: RAPIER.RigidBody | null;
    childBody: RAPIER.RigidBody | null;
    unlatched: boolean;
    detached: boolean;
    getCoordinate: () => number;
    applyTarget: (target: number) => void;
    applyImpactImpulse: (impulseNs: number) => "intact" | "unlatched" | "detached";
}

export type JointRegistry = Set<JointRuntime>;
export const JointRegistryContext = createContext<JointRegistry | null>(null);

export const COMBINE_MODES = new Set<KhrCombineMode>(["average", "minimum", "maximum", "multiply"]);
export const isObj = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);
export const isFiniteNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);
export const isPosNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v > 0;
export const isNonNegNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v) && v >= 0;

export function optFiniteNum(v: unknown, label: string, nonNeg = false): number | undefined {
    if (v === undefined)
        return undefined;
    if (!isFiniteNum(v) || (nonNeg && v < 0)) {
        throw new KhrValidationError(`Invalid ${label}: expected finite number${nonNeg ? " >= 0" : ""}`);
    }
    return v;
}

export function assignRef<T>(ref: Ref<T> | undefined, value: T | null): void {
    if (typeof ref === "function")
        ref(value);
    else if (ref && typeof ref === "object")
        (ref as MutableRefObject<T | null>).current = value;
}
