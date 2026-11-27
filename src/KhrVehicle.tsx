import React, { useCallback, useLayoutEffect, useMemo, useRef, useState } from "react";
import { useRapier } from "@react-three/rapier";
import { EcctrlVehicle, type EcctrlVehicleHandle, type EcctrlVehicleProps, type VehicleInput } from "@pinkaphant/ecctrl/vehicle";
import { BundleSceneHost } from "./BundleSceneHost.js";
import { KhrCollider } from "./KhrCollider.js";
import { assertSceneIdentityWorldScale } from "./khrInertia.js";
import { type KhrAssetBundle, type KhrCarConfig, type KhrDroneConfig } from "./khrTypes.js";
import {
    assertVehicleProps,
    buildChassisColliders,
    computeChassisMassProps,
    readPosTuple,
    readSpawnQuat,
} from "./khrVehicleChassis.js";
import { KhrVehicleJoints, type KhrVehicleJointsHandle } from "./khrVehicleJoints.js";
import { type KhrPropellerConfig, KhrVehicleRotors, type KhrWheelConfig } from "./khrVehicleRotors.js";
import { captureRotorOrigins, type DisabledRotorsFlags, resetAllWheelHits, resetDisabledRotors, ZeroGWheelGate, ZeroGWheelPostClear } from "./khrVehicleSlots.js";
import { solveWheelLoads, type SolvedWheel } from "./wheelLoads.js";

export {
    solveWheelLoads,
    type KhrPropellerConfig,
    type KhrVehicleJointsHandle,
    type KhrWheelConfig,
    type SolvedWheel,
};

export interface KhrVehicleProps extends Omit<EcctrlVehicleProps, "ref" | "carConfig" | "droneConfig"> {
    ref?: React.Ref<EcctrlVehicleHandle>;
    bundle: KhrAssetBundle;
    vehicleRef?: React.Ref<EcctrlVehicleHandle | null>;
    jointsRef?: React.Ref<KhrVehicleJointsHandle>;
    carConfig?: Partial<KhrCarConfig>;
    droneConfig?: Partial<KhrDroneConfig>;
    wheelConfig?: KhrWheelConfig;
    frontWheelConfig?: KhrWheelConfig;
    rearWheelConfig?: KhrWheelConfig;
    propellerConfig?: KhrPropellerConfig;
}

const patchedHandles = new WeakSet<EcctrlVehicleHandle>();

function hasActiveMovement(m: VehicleInput): boolean {
    return Boolean(
        m.forward || m.backward || m.steerLeft || m.steerRight || m.brake ||
        m.throttleUp || m.throttleDown || m.yawLeft || m.yawRight ||
        m.pitchForward || m.pitchBackward || m.rollLeft || m.rollRight ||
        (m.joystickL && (Math.abs(m.joystickL.x) > 1e-4 || Math.abs(m.joystickL.y) > 1e-4)) ||
        (m.joystickR && (Math.abs(m.joystickR.x) > 1e-4 || Math.abs(m.joystickR.y) > 1e-4)),
    );
}

export function KhrVehicle({ scale, colliders, bundle, ...rest }: KhrVehicleProps): React.JSX.Element {
    assertVehicleProps(scale, colliders);
    bundle.scene.updateMatrixWorld(true);
    assertSceneIdentityWorldScale(bundle.scene);

    const {
        ref,
        vehicleRef,
        jointsRef,
        position,
        rotation,
        quaternion,
        enable = true,
        enableCustomGravity = false,
        carConfig,
        droneConfig,
        wheelConfig,
        frontWheelConfig,
        rearWheelConfig,
        propellerConfig,
        linearVelocity,
        angularVelocity,
        gravityScale,
        children,
        ...ecctrlRest
    } = rest;

    const { world } = useRapier();
    const [isZeroG, setIsZeroG] = useState(() => {
        if (enableCustomGravity)
            return true;
        const g = world.gravity;
        return g.x * g.x + g.y * g.y + g.z * g.z <= 1e-12;
    });
    const [wheelEpoch, setWheelEpoch] = useState(0);
    const zeroGRef = useRef(isZeroG);
    const poisonedRef = useRef(false);

    const handleZeroGChange = useCallback((nextZeroG: boolean) => {
        if (nextZeroG) {
            poisonedRef.current = true;
            resetAllWheelHits(innerRef.current);
        }
        else if (poisonedRef.current) {
            poisonedRef.current = false;
            setWheelEpoch((e) => e + 1);
        }
        setIsZeroG(nextZeroG);
    }, []);

    const chassis = useMemo(() => computeChassisMassProps(bundle.bodies), [bundle]);
    const collidersList = useMemo(() => buildChassisColliders(bundle.bodies, chassis), [bundle.bodies, chassis]);
    const solvedWheels = useMemo(
        () => solveWheelLoads(bundle.wheels, bundle.totalDynamicMass, bundle.centerOfMass, 0.7),
        [bundle.wheels, bundle.totalDynamicMass, bundle.centerOfMass],
    );
    const rotorOrigins = useMemo(
        () => captureRotorOrigins([...bundle.wheels.map((w) => w.object), ...bundle.propellers.map((p) => p.object)]),
        [bundle.wheels, bundle.propellers],
    );

    const frontWheelNames = useMemo(
        () => new Set(solvedWheels.filter((w) => w.mountPos[2] >= chassis.com[2]).map((w) => w.tag.nodeName)),
        [solvedWheels, chassis.com],
    );
    const wheelsAllOff = !enable || isZeroG || (wheelConfig?.enable === false && frontWheelConfig?.enable !== true && rearWheelConfig?.enable !== true);
    const frontOff = frontWheelConfig?.enable === false || (frontWheelConfig?.enable === undefined && wheelConfig?.enable === false);
    const rearOff = rearWheelConfig?.enable === false || (rearWheelConfig?.enable === undefined && wheelConfig?.enable === false);
    const propsOff = !enable || propellerConfig?.enable === false;

    const disabledRotorsRef = useRef<DisabledRotorsFlags>({ wheelsAllOff, frontOff, rearOff, propsOff, frontWheelNames });
    disabledRotorsRef.current = { wheelsAllOff, frontOff, rearOff, propsOff, frontWheelNames };

    useLayoutEffect(() => {
        resetDisabledRotors(innerRef.current, disabledRotorsRef.current);
    }, [wheelsAllOff, frontOff, rearOff, propsOff, frontWheelNames]);

    const [px, py, pz] = readPosTuple(position);
    const [qx, qy, qz, qw] = readSpawnQuat(rotation, quaternion);
    const bodyPos = useMemo<[number, number, number]>(() => [px, py, pz], [px, py, pz]);
    const bodyQuat = useMemo<[number, number, number, number]>(() => [qx, qy, qz, qw], [qx, qy, qz, qw]);

    const innerRef = useRef<EcctrlVehicleHandle | null>(null);
    const assignRef = useCallback(
        (handle: EcctrlVehicleHandle | null) => {
            if (handle && !patchedHandles.has(handle)) {
                patchedHandles.add(handle);
                const origSetMovement = handle.setMovement;
                handle.setMovement = (state: VehicleInput) => {
                    origSetMovement(state);
                    if (hasActiveMovement(state) && handle.body?.isSleeping())
                        handle.body.wakeUp();
                };
            }
            innerRef.current = handle;
            if (typeof vehicleRef === "function")
                vehicleRef(handle);
            else if (vehicleRef && typeof vehicleRef === "object")
                (vehicleRef as React.MutableRefObject<EcctrlVehicleHandle | null>).current = handle;
            if (typeof ref === "function")
                ref(handle);
            else if (ref && typeof ref === "object")
                (ref as React.MutableRefObject<EcctrlVehicleHandle | null>).current = handle;
        },
        [ref, vehicleRef],
    );

    const omiSteerAngle = useMemo(
        () => bundle.wheels.find((w) => w.steer && w.maxSteeringAngle !== undefined && w.maxSteeringAngle > 0)?.maxSteeringAngle,
        [bundle.wheels],
    );
    const mergedCarConfig = useMemo(
        () => (bundle.tune || carConfig || omiSteerAngle !== undefined
            ? { ...(omiSteerAngle !== undefined ? { maxSteerAngle: omiSteerAngle } : {}), ...bundle.tune, ...carConfig }
            : undefined),
        [omiSteerAngle, bundle.tune, carConfig],
    );
    const mergedDroneConfig = useMemo(
        () => (bundle.droneTune || droneConfig ? { ...bundle.droneTune, ...droneConfig } : undefined),
        [bundle.droneTune, droneConfig],
    );

    const effLinVel = linearVelocity ?? chassis.linearVelocity;
    const effAngVel = angularVelocity ?? chassis.angularVelocity;
    const effGravScale = gravityScale ?? chassis.gravityFactor;

    return (
        <>
            <EcctrlVehicle
                ref={assignRef}
                enable={enable}
                enableCustomGravity={enableCustomGravity}
                position={bodyPos}
                quaternion={bodyQuat}
                {...(effLinVel !== undefined ? { linearVelocity: effLinVel } : {})}
                {...(effAngVel !== undefined ? { angularVelocity: effAngVel } : {})}
                {...(effGravScale !== undefined ? { gravityScale: effGravScale } : {})}
                carConfig={mergedCarConfig}
                droneConfig={mergedDroneConfig}
                {...ecctrlRest}
            >
                <BundleSceneHost bundle={bundle} owner="KhrVehicle" />
                {children}
                {collidersList.map(({ key, spec, massProperties }) => (
                    <KhrCollider key={key} c={spec} massProperties={massProperties} />
                ))}
                <ZeroGWheelGate
                    vehicleRef={innerRef}
                    enable={enable}
                    enableCustomGravity={enableCustomGravity}
                    spawnPos={bodyPos}
                    zeroGRef={zeroGRef}
                    onZeroGChange={handleZeroGChange}
                />
                <KhrVehicleRotors
                    solvedWheels={solvedWheels}
                    propellers={bundle.propellers}
                    scene={bundle.scene}
                    comZ={chassis.com[2]}
                    rotorOrigins={rotorOrigins}
                    enable={enable}
                    wheelEnable={enable && !isZeroG}
                    wheelEpoch={wheelEpoch}
                    wheelConfig={wheelConfig}
                    frontWheelConfig={frontWheelConfig}
                    rearWheelConfig={rearWheelConfig}
                    propellerConfig={propellerConfig}
                />
                <ZeroGWheelPostClear key={`post:${wheelEpoch}`} vehicleRef={innerRef} zeroGRef={zeroGRef} disabledRotorsRef={disabledRotorsRef} />
            </EcctrlVehicle>
            <KhrVehicleJoints
                bodies={bundle.bodies}
                vehicleRef={innerRef}
                spawnPos={bodyPos}
                spawnQuat={bodyQuat}
                enableCustomGravity={enableCustomGravity}
                linearVelocity={effLinVel}
                angularVelocity={effAngVel}
                gravityScale={effGravScale}
                jointsRef={jointsRef}
            />
        </>
    );
}
