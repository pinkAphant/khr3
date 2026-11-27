import type * as THREE from "three";
import type { KhrCarConfig, KhrDroneConfig } from "./khrConfigTypes.js";
import type { EcctrlPropellerTag, EcctrlWheelTag, OmiVehicleWheelSpec } from "./khrVehicleTypes.js";

export * from "./khrConfigTypes.js";
export * from "./khrVehicleTypes.js";

export type KhrCombineMode = "average" | "minimum" | "maximum" | "multiply";

export type KhrImplicitShape =
    | {
        type: "box";
        size: [number, number, number];
    }
    | {
        type: "sphere";
        radius: number;
    }
    | {
        type: "cylinder";
        height: number;
        radiusTop: number;
        radiusBottom: number;
    }
    | {
        type: "capsule";
        height: number;
        radiusTop: number;
        radiusBottom: number;
    };

export type KhrColliderShape =
    | KhrImplicitShape
    | {
        type: "convexHull";
        vertices: Float32Array;
        meshNodeName: string;
    }
    | {
        type: "trimesh";
        vertices: Float32Array;
        indices: Uint32Array;
        meshNodeName: string;
    };

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
