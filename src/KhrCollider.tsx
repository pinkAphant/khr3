import React, { useCallback, useEffect, useRef } from "react";
import {
    BallCollider,
    CapsuleCollider,
    CoefficientCombineRule,
    ConvexHullCollider,
    CuboidCollider,
    CylinderCollider,
    TrimeshCollider,
    type RapierCollider,
} from "@react-three/rapier";
import { assignRef, type KhrColliderSpec, type KhrCombineMode } from "./types.js";

export interface ColliderMass {
    mass: number;
    centerOfMass: { x: number; y: number; z: number };
    principalAngularInertia: { x: number; y: number; z: number };
    angularInertiaLocalFrame: { x: number; y: number; z: number; w: number };
}

const COMBINE_RULES: Record<KhrCombineMode, CoefficientCombineRule> = {
    average: CoefficientCombineRule.Average,
    minimum: CoefficientCombineRule.Min,
    maximum: CoefficientCombineRule.Max,
    multiply: CoefficientCombineRule.Multiply,
};

/** `RAPIER.TriMeshFlags.FIX_INTERNAL_EDGES` (144) — merges duplicate vertices and suppresses internal seam ghost normals. */
export const KHR_TRIMESH_FLAGS = 144;

export interface KhrColliderProps {
    c: KhrColliderSpec;
    position?: [number, number, number];
    quaternion?: [number, number, number, number];
    density?: number;
    mass?: number;
    massProperties?: ColliderMass;
    collisionGroups?: number;
    solverGroups?: number;
    ref?: React.Ref<RapierCollider>;
}

export function KhrCollider({
    c,
    position,
    quaternion,
    density,
    mass,
    massProperties,
    collisionGroups,
    solverGroups,
    ref,
}: KhrColliderProps): React.JSX.Element {
    const innerRef = useRef<RapierCollider | null>(null);
    const setRef = useCallback(
        (node: RapierCollider | null) => {
            innerRef.current = node;
            assignRef(ref, node);
        },
        [ref],
    );

    useEffect(() => {
        innerRef.current?.parent()?.recomputeMassPropertiesFromColliders();
    }, [c, density, mass, massProperties]);

    const effCollisionGroups = c.collisionGroups ?? collisionGroups;
    const effSolverGroups = solverGroups ?? effCollisionGroups;
    const massOption = massProperties
        ? { massProperties }
        : mass !== undefined
            ? { mass }
            : { density: density ?? 0 };

    const common = {
        ref: setRef,
        position: position ?? c.localPosition,
        quaternion: quaternion ?? c.localQuaternion,
        ...massOption,
        ...(c.friction === undefined ? {} : { friction: c.friction }),
        ...(c.restitution === undefined ? {} : { restitution: c.restitution }),
        ...(c.frictionCombine === undefined ? {} : { frictionCombineRule: COMBINE_RULES[c.frictionCombine] }),
        ...(c.restitutionCombine === undefined ? {} : { restitutionCombineRule: COMBINE_RULES[c.restitutionCombine] }),
        ...(effCollisionGroups === undefined ? {} : { collisionGroups: effCollisionGroups }),
        ...(effSolverGroups === undefined ? {} : { solverGroups: effSolverGroups }),
    };

    const { shape } = c;
    if (shape.type === "box") {
        const [sx, sy, sz] = shape.size;
        return <CuboidCollider args={[sx * 0.5, sy * 0.5, sz * 0.5]} {...common} />;
    }
    if (shape.type === "sphere")
        return <BallCollider args={[shape.radius]} {...common} />;
    if (shape.type === "cylinder")
        return <CylinderCollider args={[shape.height * 0.5, shape.radiusTop]} {...common} />;
    if (shape.type === "capsule")
        return <CapsuleCollider args={[shape.height * 0.5, shape.radiusTop]} {...common} />;
    if (shape.type === "convexHull")
        return <ConvexHullCollider args={[shape.vertices]} {...common} />;
    return <TrimeshCollider args={[shape.vertices, shape.indices, KHR_TRIMESH_FLAGS] as unknown as [Float32Array, Uint32Array]} {...common} />;
}
