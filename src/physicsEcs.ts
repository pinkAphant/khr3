import { createContext } from "react";
import type * as THREE from "three";
import type RAPIER from "@dimforge/rapier3d-compat";
import type { KhrJointDriveSpec, KhrJointSpec } from "./khrTypes.js";

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

/** Retained as an optional context export for compatibility; Rapier steps ImpulseJoints natively. */
export const JointRegistryContext = createContext<JointRegistry | null>(null);
