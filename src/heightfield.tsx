import React, { useEffect, useRef } from "react";
import * as THREE from "three";
import { RigidBody, useRapier, type RapierCollider, type RapierRigidBody, type RigidBodyProps } from "@react-three/rapier";
import { assignRef, KhrValidationError } from "./types.js";

/** `RAPIER.HeightFieldFlags.FIX_INTERNAL_EDGES` (1) — eliminates internal grid-edge ghost contact normals. */
export const KHR_HEIGHTFIELD_FLAGS = 1;

export type HeightfieldLayout = "rapier-column-major" | "row-major";

/**
 * Transposes a row-major `(nrows + 1) * (ncols + 1)` height grid (`heights[zRow * (ncols + 1) + xCol]`,
 * as returned by `@khr3/roads` `sampleHeightfield` and Three.js `PlaneGeometry`) into Rapier WASM's
 * column-major layout (`out[xCol * (nrows + 1) + zRow]`).
 */
export function transposeHeightfieldForRapier(
    rowMajor: ArrayLike<number>,
    nrows: number,
    ncols = nrows,
    out?: Float32Array,
): Float32Array {
    if (!Number.isInteger(nrows) || nrows < 1 || !Number.isInteger(ncols) || ncols < 1) {
        throw new KhrValidationError(`transposeHeightfieldForRapier: nrows (${nrows}) and ncols (${ncols}) must be integers >= 1`);
    }
    const strideX = ncols + 1;
    const strideZ = nrows + 1;
    const expected = strideZ * strideX;
    if (rowMajor.length !== expected) {
        throw new KhrValidationError(
            `transposeHeightfieldForRapier: expected (${nrows} + 1) * (${ncols} + 1) = ${expected} samples, got ${rowMajor.length}`,
        );
    }
    const dst = out && out.length === expected ? out : new Float32Array(expected);
    for (let zRow = 0; zRow <= nrows; zRow++) {
        const rowBase = zRow * strideX;
        for (let xCol = 0; xCol <= ncols; xCol++) {
            dst[xCol * strideZ + zRow] = rowMajor[rowBase + xCol];
        }
    }
    return dst;
}

/**
 * Samples `heightFn(x, z)` directly in Rapier WASM's column-major `(nrows + 1) * (ncols + 1)` layout
 * over `[originX, originX + sizeX] x [originZ, originZ + sizeZ]`.
 */
export function sampleRapierHeightfield(
    heightFn: (x: number, z: number) => number,
    originX: number,
    originZ: number,
    sizeX: number,
    sizeZ: number,
    nrows: number,
    ncols = nrows,
    out?: Float32Array,
): Float32Array {
    if (!Number.isInteger(nrows) || nrows < 1 || !Number.isInteger(ncols) || ncols < 1) {
        throw new KhrValidationError(`sampleRapierHeightfield: nrows (${nrows}) and ncols (${ncols}) must be integers >= 1`);
    }
    if (!(sizeX > 0 && sizeZ > 0 && Number.isFinite(sizeX) && Number.isFinite(sizeZ))) {
        throw new KhrValidationError(`sampleRapierHeightfield: sizeX (${sizeX}) and sizeZ (${sizeZ}) must be finite > 0`);
    }
    const strideZ = nrows + 1;
    const total = strideZ * (ncols + 1);
    const dst = out && out.length === total ? out : new Float32Array(total);
    for (let xCol = 0; xCol <= ncols; xCol++) {
        const x = originX + (xCol / ncols) * sizeX;
        const colBase = xCol * strideZ;
        for (let zRow = 0; zRow <= nrows; zRow++) {
            const z = originZ + (zRow / nrows) * sizeZ;
            dst[colBase + zRow] = heightFn(x, z);
        }
    }
    return dst;
}

/**
 * Builds an indexed Three.js `BufferGeometry` centered at `(0, 0, 0)` spanning
 * `[-sizeX/2, +sizeX/2] x [-sizeZ/2, +sizeZ/2]` whose vertex positions, cell triangulation,
 * and finite-difference normals match Rapier's `HeightField` collider 1:1.
 */
export function createHeightfieldGeometry(
    heights: ArrayLike<number>,
    sizeX: number,
    sizeZ: number,
    nrows: number,
    ncols = nrows,
    layout: HeightfieldLayout = "rapier-column-major",
): THREE.BufferGeometry {
    const strideZ = nrows + 1;
    const strideX = ncols + 1;
    const expected = strideZ * strideX;
    if (heights.length !== expected) {
        throw new KhrValidationError(
            `createHeightfieldGeometry: expected ${expected} height samples for ${nrows}x${ncols} cells, got ${heights.length}`,
        );
    }
    const readH = layout === "rapier-column-major"
        ? (zRow: number, xCol: number): number => heights[xCol * strideZ + zRow]
        : (zRow: number, xCol: number): number => heights[zRow * strideX + xCol];

    const positions = new Float32Array(expected * 3);
    const normals = new Float32Array(expected * 3);
    const uvs = new Float32Array(expected * 2);
    const dx = sizeX / ncols;
    const dz = sizeZ / nrows;
    const halfX = sizeX * 0.5;
    const halfZ = sizeZ * 0.5;

    let vIdx = 0;
    for (let zRow = 0; zRow <= nrows; zRow++) {
        const z = -halfZ + zRow * dz;
        const v = zRow / nrows;
        for (let xCol = 0; xCol <= ncols; xCol++) {
            const x = -halfX + xCol * dx;
            const y = readH(zRow, xCol);
            positions[vIdx * 3] = x;
            positions[vIdx * 3 + 1] = y;
            positions[vIdx * 3 + 2] = z;

            const hL = readH(zRow, Math.max(0, xCol - 1));
            const hR = readH(zRow, Math.min(ncols, xCol + 1));
            const hD = readH(Math.max(0, zRow - 1), xCol);
            const hU = readH(Math.min(nrows, zRow + 1), xCol);
            const spanX = (Math.min(ncols, xCol + 1) - Math.max(0, xCol - 1)) * dx;
            const spanZ = (Math.min(nrows, zRow + 1) - Math.max(0, zRow - 1)) * dz;
            const dhdx = spanX > 0 ? (hR - hL) / spanX : 0;
            const dhdz = spanZ > 0 ? (hU - hD) / spanZ : 0;
            const invLen = 1 / Math.hypot(dhdx, 1, dhdz);
            normals[vIdx * 3] = -dhdx * invLen;
            normals[vIdx * 3 + 1] = invLen;
            normals[vIdx * 3 + 2] = -dhdz * invLen;

            uvs[vIdx * 2] = xCol / ncols;
            uvs[vIdx * 2 + 1] = v;
            vIdx++;
        }
    }

    const triIndices = new Uint32Array(nrows * ncols * 6);
    let iPtr = 0;
    for (let zRow = 0; zRow < nrows; zRow++) {
        for (let xCol = 0; xCol < ncols; xCol++) {
            const i00 = zRow * strideX + xCol;
            const i10 = i00 + 1;
            const i01 = i00 + strideX;
            const i11 = i01 + 1;
            triIndices[iPtr++] = i00;
            triIndices[iPtr++] = i01;
            triIndices[iPtr++] = i10;
            triIndices[iPtr++] = i10;
            triIndices[iPtr++] = i01;
            triIndices[iPtr++] = i11;
        }
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geo.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    geo.setAttribute("uv", new THREE.BufferAttribute(uvs, 2));
    geo.setIndex(new THREE.BufferAttribute(triIndices, 1));
    geo.computeBoundingSphere();
    return geo;
}

export interface KhrHeightfieldProps extends Omit<RigidBodyProps, "type" | "colliders" | "ref"> {
    readonly ref?: React.Ref<RapierRigidBody>;
    readonly colliderRef?: React.Ref<RapierCollider>;
    readonly heights: Float32Array;
    /** Cell subdivisions `[nrows (Z), ncols (X)]` or uniform `n`. `heights.length` must equal `(nrows + 1) * (ncols + 1)`. */
    readonly segments: number | readonly [nrows: number, ncols: number];
    /** World footprint `[sizeX, sizeZ]` in metres, centered on `position`. */
    readonly size: readonly [sizeX: number, sizeZ: number];
    /** Input layout of `heights` (`"rapier-column-major"` default, or `"row-major"` to auto-transpose). */
    readonly layout?: HeightfieldLayout;
    readonly friction?: number;
    readonly restitution?: number;
    readonly collisionGroups?: number;
    readonly solverGroups?: number;
    /** Increment when mutating `heights` in-place to rebuild the Rapier WASM heightfield collider. */
    readonly version?: number;
}

/**
 * Direct Rapier WASM heightfield body & collider (`HeightFieldFlags.FIX_INTERNAL_EDGES`),
 * bypassing `@react-three/rapier`'s broken `<HeightfieldCollider>` `scaleColliderArgs` mutation.
 */
export function KhrHeightfield({
    ref,
    colliderRef,
    heights,
    segments,
    size,
    layout = "rapier-column-major",
    friction = 0.85,
    restitution = 0.05,
    collisionGroups,
    solverGroups,
    version = 0,
    children,
    ...rbProps
}: KhrHeightfieldProps): React.JSX.Element {
    const { rapier, world } = useRapier();
    const rbRef = useRef<RapierRigidBody | null>(null);
    const nrows = typeof segments === "number" ? segments : segments[0];
    const ncols = typeof segments === "number" ? segments : segments[1];
    const [sizeX, sizeZ] = size;

    const assignRb = (rb: RapierRigidBody | null): void => {
        rbRef.current = rb;
        assignRef(ref, rb);
    };

    useEffect(() => {
        const rb = rbRef.current;
        if (!rb || !world.getRigidBody(rb.handle))
            return;
        const colHeights = layout === "row-major"
            ? transposeHeightfieldForRapier(heights, nrows, ncols)
            : heights;
        const expected = (nrows + 1) * (ncols + 1);
        if (colHeights.length !== expected) {
            throw new KhrValidationError(`KhrHeightfield: expected ${expected} samples for ${nrows}x${ncols} cells, got ${colHeights.length}`);
        }
        const desc = (rapier.ColliderDesc as unknown as {
            heightfield: (nr: number, nc: number, h: Float32Array, s: { x: number; y: number; z: number }, flags?: number) => RAPIER_ColliderDesc;
        }).heightfield(nrows, ncols, colHeights, { x: sizeX, y: 1, z: sizeZ }, KHR_HEIGHTFIELD_FLAGS);
        desc.setFriction(friction);
        desc.setRestitution(restitution);
        if (collisionGroups !== undefined)
            desc.setCollisionGroups(collisionGroups);
        if (solverGroups !== undefined || collisionGroups !== undefined)
            desc.setSolverGroups(solverGroups ?? collisionGroups!);

        const col = world.createCollider(desc as never, rb);
        assignRef(colliderRef, col);
        return () => {
            assignRef(colliderRef, null);
            if (world.getCollider(col.handle))
                world.removeCollider(col, false);
        };
    }, [rapier, world, heights, nrows, ncols, sizeX, sizeZ, layout, friction, restitution, collisionGroups, solverGroups, version, colliderRef]);

    return (
        <RigidBody {...rbProps} ref={assignRb} type="fixed" colliders={false}>
            {children}
        </RigidBody>
    );
}

interface RAPIER_ColliderDesc {
    setFriction(f: number): this;
    setRestitution(r: number): this;
    setCollisionGroups(g: number): this;
    setSolverGroups(g: number): this;
}
