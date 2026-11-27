import React, { useLayoutEffect, useRef } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { useRapier } from "@react-three/rapier";
import { useCustomGravity } from "@pinkaphant/ecctrl/gravity";
import type { EcctrlVehicleHandle, PropellerInfoType, WheelInfoType } from "@pinkaphant/ecctrl/vehicle";
import { captureSubtreeOrigins, restoreSubtreeOrigin, type SubtreeOrigin } from "./BundleSceneHost.js";
import { registerMountedSubtree } from "./khrClone.js";
import { computeRotorLocalMatrix } from "./khrVehicleChassis.js";

export type RotorOrigin = SubtreeOrigin;
export const captureRotorOrigins = captureSubtreeOrigins;

const _samplePos = new THREE.Vector3();

export function resetAllWheelHits(handle: EcctrlVehicleHandle | null): void {
    if (!handle)
        return;
    for (const wheelRef of handle.wheelsInfo.values()) {
        const w = wheelRef.current as WheelInfoType | null;
        if (!w)
            continue;
        w.rayHit = null;
        w.rayHitBody = null;
        w.floatImp.set(0, 0, 0);
        w.lngFricImp.set(0, 0, 0);
        w.latFricImp.set(0, 0, 0);
        w.lngSlipRatio = 0;
        w.latSlipRatio = 0;
        w.slipStrength = 0;
        w.isOnPlatform = false;
        w.effInertia = 0;
    }
}

export function resetDisabledWheelHits(handle: EcctrlVehicleHandle | null, isDisabled: (name: string | undefined) => boolean): void {
    if (!handle)
        return;
    for (const wheelRef of handle.wheelsInfo.values()) {
        const w = wheelRef.current as WheelInfoType | null;
        if (!w || !isDisabled(w.name))
            continue;
        w.rayHit = null;
        w.rayHitBody = null;
        w.floatImp.set(0, 0, 0);
        w.lngFricImp.set(0, 0, 0);
        w.latFricImp.set(0, 0, 0);
        w.lngSlipRatio = 0;
        w.latSlipRatio = 0;
        w.slipStrength = 0;
        w.isOnPlatform = false;
        w.effInertia = 0;
    }
}

export function resetAllPropellerOutputs(handle: EcctrlVehicleHandle | null): void {
    if (!handle)
        return;
    for (const propRef of handle.propellersInfo.values()) {
        const p = propRef.current as PropellerInfoType | null;
        if (!p)
            continue;
        p.lx = 0; p.ly = 0; p.lz = 0;
        p.ax = 0; p.ay = 0; p.az = 0;
        p.thrustDir.set(0, 0, 0);
        p.torqueDir.set(0, 0, 0);
        p.thrustImpulse.set(0, 0, 0);
        p.torqueImpulse.set(0, 0, 0);
        p.finalThrottle = 0;
        p.throttle = 0;
        p.setThrottle?.(0);
    }
}

export interface DisabledRotorsFlags {
    wheelsAllOff: boolean;
    frontOff: boolean;
    rearOff: boolean;
    propsOff: boolean;
    frontWheelNames: ReadonlySet<string>;
}

export function resetDisabledRotors(
    handle: EcctrlVehicleHandle | null,
    opts: DisabledRotorsFlags,
): void {
    if (opts.wheelsAllOff) {
        resetAllWheelHits(handle);
    }
    else if (opts.frontOff || opts.rearOff) {
        resetDisabledWheelHits(handle, (name) => (name && opts.frontWheelNames.has(name) ? opts.frontOff : opts.rearOff));
    }
    if (opts.propsOff)
        resetAllPropellerOutputs(handle);
}

/**
 * Enables `<ShapeCastWheel>` exclusively under positive gravity (`gravityMag > 1e-6`), keeping
 * `ShapeCastWheel` internal `smoothedLngImpulse` state finite during 0G flight.
 */
export function ZeroGWheelGate({
    vehicleRef,
    enable,
    enableCustomGravity,
    spawnPos,
    zeroGRef,
    onZeroGChange,
}: {
    vehicleRef: React.RefObject<EcctrlVehicleHandle | null>;
    enable: boolean;
    enableCustomGravity: boolean;
    spawnPos: readonly [number, number, number];
    zeroGRef: React.RefObject<boolean>;
    onZeroGChange: (isZeroG: boolean) => void;
}): null {
    const { world } = useRapier();
    useFrame(() => {
        if (!enable)
            return;
        const handle = vehicleRef.current;
        let gSq: number;
        if (enableCustomGravity) {
            if (handle?.body && !handle.body.isSleeping()) {
                gSq = handle.gravityMag * handle.gravityMag;
            }
            else {
                const t = handle?.body?.translation();
                if (t)
                    _samplePos.set(t.x, t.y, t.z);
                else
                    _samplePos.set(spawnPos[0], spawnPos[1], spawnPos[2]);
                gSq = useCustomGravity.getState().gravityField(_samplePos).lengthSq();
            }
        }
        else {
            const g = world.gravity;
            gSq = g.x * g.x + g.y * g.y + g.z * g.z;
        }
        const isZeroG = gSq <= 1e-12;
        if (isZeroG !== zeroGRef.current) {
            zeroGRef.current = isZeroG;
            onZeroGChange(isZeroG);
        }
        if (isZeroG)
            resetAllWheelHits(handle);
    });
    return null;
}

export function ZeroGWheelPostClear({
    vehicleRef,
    zeroGRef,
    disabledRotorsRef,
}: {
    vehicleRef: React.RefObject<EcctrlVehicleHandle | null>;
    zeroGRef: React.RefObject<boolean>;
    disabledRotorsRef?: React.RefObject<DisabledRotorsFlags>;
}): null {
    useFrame(() => {
        if (zeroGRef.current) {
            resetAllWheelHits(vehicleRef.current);
            return;
        }
        const flags = disabledRotorsRef?.current;
        if (flags && (flags.wheelsAllOff || flags.frontOff || flags.rearOff || flags.propsOff)) {
            resetDisabledRotors(vehicleRef.current, flags);
        }
    });
    return null;
}

export function RotorVisualSlot({
    object,
    center,
    quaternion,
    scene,
    origin,
}: {
    object: THREE.Object3D;
    center: readonly [number, number, number];
    quaternion?: readonly [number, number, number, number];
    scene: THREE.Object3D;
    origin?: RotorOrigin;
}): React.JSX.Element {
    const slotRef = useRef<THREE.Group | null>(null);
    useLayoutEffect(() => {
        const slot = slotRef.current;
        const parent = slot?.parent ?? null;
        if (!slot || !parent)
            return;
        const snap = origin ?? captureSubtreeOrigins([object]).get(object)!;
        scene.updateMatrixWorld(true);
        const localMat = computeRotorLocalMatrix(object, scene, center, quaternion);
        if (snap.origParent)
            snap.origParent.remove(object);
        const pos = new THREE.Vector3();
        const quat = new THREE.Quaternion();
        const scl = new THREE.Vector3();
        localMat.decompose(pos, quat, scl);
        object.position.copy(pos);
        object.quaternion.copy(quat.normalize());
        object.scale.copy(scl);
        slot.add(object);
        const untrack = registerMountedSubtree(() => {
            const p = object.position.clone(), q = object.quaternion.clone(), s = object.scale.clone();
            if (object.parent === slot)
                slot.remove(object);
            restoreSubtreeOrigin(object, snap);
            return () => {
                if (snap.origParent)
                    snap.origParent.remove(object);
                object.position.copy(p);
                object.quaternion.copy(q);
                object.scale.copy(s);
                slot.add(object);
            };
        });
        return () => {
            untrack();
            if (object.parent === slot)
                slot.remove(object);
            restoreSubtreeOrigin(object, snap);
        };
    }, [object, center, quaternion, scene, origin]);
    return <group ref={slotRef} />;
}
