import React from "react";
import type * as THREE from "three";
import { ShapeCastWheel, type ShapeCastWheelProps, ThrustPropeller, type ThrustPropellerProps } from "@pinkaphant/ecctrl/vehicle";
import type { EcctrlPropellerTag } from "./khrTypes.js";
import { type RotorOrigin, RotorVisualSlot } from "./khrVehicleSlots.js";
import type { SolvedWheel } from "./wheelLoads.js";

export type KhrWheelConfig = Partial<Omit<ShapeCastWheelProps, "ref" | "key" | "children">>;
export type KhrPropellerConfig = Partial<Omit<ThrustPropellerProps, "ref" | "key" | "children">>;

const G_SI = 9.81;
const UNSPRUNG_CORNER_RATIO = 0.035;

export function KhrVehicleRotors({
    solvedWheels,
    propellers,
    scene,
    comZ,
    rotorOrigins,
    enable,
    wheelEnable,
    wheelEpoch = 0,
    wheelConfig,
    frontWheelConfig,
    rearWheelConfig,
    propellerConfig,
}: {
    solvedWheels: readonly SolvedWheel[];
    propellers: readonly EcctrlPropellerTag[];
    scene: THREE.Group;
    comZ: number;
    rotorOrigins: ReadonlyMap<THREE.Object3D, RotorOrigin>;
    enable: boolean;
    wheelEnable: boolean;
    wheelEpoch?: number;
    wheelConfig?: KhrWheelConfig;
    frontWheelConfig?: KhrWheelConfig;
    rearWheelConfig?: KhrWheelConfig;
    propellerConfig?: KhrPropellerConfig;
}): React.JSX.Element {
    return (
        <>
            {solvedWheels.map(({ tag, cornerMassKg, springK, dampingC, mountPos }) => {
                const axleCfg = mountPos[2] >= comZ ? frontWheelConfig : rearWheelConfig;
                const wheelVol = Math.max(Math.PI * tag.radius * tag.radius * tag.width, 1e-6);
                const wheelModelDensity = tag.wheelDensity ?? (UNSPRUNG_CORNER_RATIO * cornerMassKg) / wheelVol;
                const tagWheelProps: KhrWheelConfig = {
                    rayLength: tag.travel,
                    rayShapeR: tag.radius,
                    rayShapeH: tag.width * 0.5,
                    wheelModelRadius: tag.radius,
                    wheelModelDensity,
                    springK,
                    dampingC,
                    steerWheel: tag.steer,
                    driveWheel: tag.drive,
                    brakeWheel: tag.brake,
                    maxBrakeTorque: tag.maxBrakeTorque ?? cornerMassKg * G_SI * tag.radius,
                    ...(tag.steerInvert !== undefined ? { steerInvert: tag.steerInvert } : {}),
                    ...(tag.driveInvert !== undefined ? { driveInvert: tag.driveInvert } : {}),
                    ...(tag.driveTorqueWeight !== undefined ? { driveTorqueWeight: tag.driveTorqueWeight } : {}),
                    ...(tag.rollingResistanceCoef !== undefined ? { rollingResistanceCoef: tag.rollingResistanceCoef } : {}),
                    ...(tag.tireGripFactor !== undefined ? { tireGripFactor: tag.tireGripFactor } : {}),
                    ...(tag.latFrictionEllipseScale !== undefined ? { latFrictionEllipseScale: tag.latFrictionEllipseScale } : {}),
                    ...(tag.relaxLngRate !== undefined ? { relaxLngRate: tag.relaxLngRate } : {}),
                    ...(tag.relaxLatRate !== undefined ? { relaxLatRate: tag.relaxLatRate } : {}),
                    ...(tag.lngSlipRatioCurveData !== undefined ? { lngSlipRatioCurveData: tag.lngSlipRatioCurveData } : {}),
                    ...(tag.latSlipRatioCurveData !== undefined ? { latSlipRatioCurveData: tag.latSlipRatioCurveData } : {}),
                };
                return (
                    <ShapeCastWheel
                        key={`${tag.nodeName}:${wheelEpoch}`}
                        name={tag.nodeName}
                        position={mountPos}
                        quaternion={tag.quaternion ?? [0, 0, 0, 1]}
                        {...tagWheelProps}
                        {...wheelConfig}
                        {...axleCfg}
                        enable={wheelEnable && (axleCfg?.enable ?? wheelConfig?.enable ?? true)}
                    >
                        <RotorVisualSlot
                            object={tag.object}
                            center={tag.position}
                            quaternion={tag.quaternion}
                            scene={scene}
                            origin={rotorOrigins.get(tag.object)}
                        />
                    </ShapeCastWheel>
                );
            })}
            {propellers.map((tag) => {
                const propEnabled = enable && (propellerConfig?.enable ?? true);
                const visual = (
                    <RotorVisualSlot
                        object={tag.object}
                        center={tag.position}
                        quaternion={tag.quaternion}
                        scene={scene}
                        origin={rotorOrigins.get(tag.object)}
                    />
                );
                if (!propEnabled) {
                    return (
                        <group
                            key={`${tag.nodeName}:off`}
                            name={tag.nodeName}
                            position={[tag.position[0], tag.position[1], tag.position[2]]}
                            quaternion={tag.quaternion ?? [0, 0, 0, 1]}
                        >
                            {visual}
                        </group>
                    );
                }
                return (
                    <ThrustPropeller
                        key={tag.nodeName}
                        name={tag.nodeName}
                        position={[tag.position[0], tag.position[1], tag.position[2]]}
                        quaternion={tag.quaternion ?? [0, 0, 0, 1]}
                        debug={false}
                        maxThrust={tag.maxThrust}
                        {...(tag.torqueRatio !== undefined ? { torqueRatio: tag.torqueRatio } : {})}
                        {...(tag.invertThrust !== undefined ? { invertThrust: tag.invertThrust } : {})}
                        {...(tag.invertTorque !== undefined ? { invertTorque: tag.invertTorque } : {})}
                        {...propellerConfig}
                        enable={true}
                    >
                        {visual}
                    </ThrustPropeller>
                );
            })}
        </>
    );
}
