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
import type { ColliderMass } from "./assemblyPlan.js";
import type { KhrColliderSpec, KhrCombineMode } from "./khrTypes.js";

const COMBINE_RULES: Record<KhrCombineMode, CoefficientCombineRule> = {
    average: CoefficientCombineRule.Average,
    minimum: CoefficientCombineRule.Min,
    maximum: CoefficientCombineRule.Max,
    multiply: CoefficientCombineRule.Multiply,
};

export interface KhrColliderProps {
    c: KhrColliderSpec;
    /** Pose override in the parent RigidBody frame; defaults to the authored KHR collider offset. */
    position?: [number, number, number];
    quaternion?: [number, number, number, number];
    /** Authored body mass carried declaratively by this collider (`density: 0` when omitted). */
    massProperties?: ColliderMass;
    /** Fallback Rapier collision groups when `c.collisionGroups` is omitted. */
    collisionGroups?: number;
    /** Fallback Rapier solver groups when `c.collisionGroups` is omitted. */
    solverGroups?: number;
    ref?: React.Ref<RapierCollider>;
}

export function KhrCollider({ c, position, quaternion, massProperties, collisionGroups, solverGroups, ref }: KhrColliderProps): React.JSX.Element {
    const innerRef = useRef<RapierCollider | null>(null);
    const setRef = useCallback((node: RapierCollider | null) => {
        innerRef.current = node;
        if (typeof ref === "function")
            ref(node);
        else if (ref)
            (ref as React.MutableRefObject<RapierCollider | null>).current = node;
    }, [ref]);
    // Rapier WASM only auto-flushes dirty collider mass flags inside world.step(); flush once after
    // @react-three/rapier's AnyCollider useEffect so body.mass() is already exact before step 0.
    useEffect(() => {
        innerRef.current?.parent()?.recomputeMassPropertiesFromColliders();
    }, [c, massProperties]);
    const effCollisionGroups = c.collisionGroups ?? collisionGroups;
    const effSolverGroups = c.collisionGroups === undefined ? solverGroups : undefined;
    const common = {
        ref: setRef,
        position: position ?? c.localPosition,
        quaternion: quaternion ?? c.localQuaternion,
        ...(massProperties ? { massProperties } : { density: 0 }),
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
    return <TrimeshCollider args={[shape.vertices, shape.indices]} {...common} />;
}
