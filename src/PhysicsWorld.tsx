import React, { useMemo, type ReactNode, type RefObject } from "react";
import { useFrame } from "@react-three/fiber";
import { Physics, useRapier, type PhysicsProps } from "@react-three/rapier";
import { TimeControl } from "@pinkaphant/ecctrl/time";
import { JointRegistryContext, type JointRuntime } from "./types.js";

export type TimeControlValue = number | RefObject<number>;

export interface PhysicsWorldProps extends Omit<PhysicsProps, "paused" | "timeStep" | "gravity" | "children"> {
    paused?: boolean;
    timeScale?: TimeControlValue;
    maxDelta?: TimeControlValue;
    gravity?: [number, number, number];
    children: ReactNode;
}

const readVal = (v: TimeControlValue): number => (typeof v === "number" ? v : v.current);

function TimestepPresync({ paused, timeScale, maxDelta }: { paused: boolean; timeScale: TimeControlValue; maxDelta: TimeControlValue }): null {
    const { world } = useRapier();
    useFrame((_, delta) => {
        const md = Math.max(0, readVal(maxDelta));
        const ts = readVal(timeScale);
        world.timestep = !paused && Number.isFinite(ts) && ts > 0 && md > 0 && delta > 0 ? Math.min(delta, md) * ts : 0;
    });
    return null;
}

export function PhysicsWorld({
    paused = false,
    timeScale = 1,
    maxDelta = 1 / 30,
    gravity = [0, -9.81, 0],
    children,
    ...physicsProps
}: PhysicsWorldProps): React.JSX.Element {
    const jointRegistry = useMemo(() => new Set<JointRuntime>(), []);
    return (
        <JointRegistryContext.Provider value={jointRegistry}>
            <Physics {...physicsProps} paused timeStep="vary" gravity={gravity}>
                <TimestepPresync paused={paused} timeScale={timeScale} maxDelta={maxDelta} />
                {children}
                <TimeControl paused={paused} timeScale={timeScale} maxDelta={maxDelta} />
            </Physics>
        </JointRegistryContext.Provider>
    );
}
