import assert from "node:assert/strict";
import { describe, it } from "node:test";
import * as THREE from "three";
import { ENGINE_TORQUE_CURVES, compilePacejkaCurve } from "./curves.js";
import { exportKhrGlb, loadKhrGlb } from "./index.js";
import { solveWheelLoads } from "./vehicle.js";

describe("@khr3/core", () => {
    it("round-trips KHR_physics_rigid_bodies and OMI_vehicle_wheel and solves wheel loads", async () => {
        const root = new THREE.Group();
        const chassis = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.5, 4.0), new THREE.MeshStandardMaterial());
        chassis.name = "Chassis";
        root.add(chassis);

        const wheels = [
            ["Wheel_FL", 0.85, 1.35, true],
            ["Wheel_FR", -0.85, 1.35, true],
            ["Wheel_RL", 0.85, -1.35, false],
            ["Wheel_RR", -0.85, -1.35, false],
        ] as const;
        for (const [name, x, z] of wheels) {
            const w = new THREE.Mesh(new THREE.CylinderGeometry(0.34, 0.34, 0.24, 12), new THREE.MeshStandardMaterial());
            w.name = name;
            w.position.set(x, -0.2, z);
            chassis.add(w);
        }

        const glb = await exportKhrGlb(
            root,
            [
                { nodeName: "Chassis", motion: { mass: 1200 }, collider: { shape: { type: "box", size: [1.8, 0.5, 4.0] } } },
                ...wheels.map(([nodeName, , , steer]) => ({
                    nodeName,
                    wheel: { radius: 0.34, width: 0.24, steering: steer, propulsion: true, suspensionTravel: 0.22 },
                })),
            ],
            { ecctrl: { engineHorsepower: 260, engineTorqueCurveData: ENGINE_TORQUE_CURVES.naScreamer } },
        );

        const bundle = await loadKhrGlb(new Uint8Array(glb));
        assert.equal(bundle.bodies.length, 1);
        assert.equal(bundle.wheels.length, 4);
        assert.equal(bundle.totalDynamicMass, 1200);
        const solved = solveWheelLoads(bundle.wheels, bundle.totalDynamicMass, bundle.centerOfMass, 0.7);
        assert.equal(solved.length, 4);
        assert.ok(solved.every((w) => w.springK > 0 && w.dampingC > 0));
        assert.ok(compilePacejkaCurve(10, 1.5, -0.2, 16, "lat").points.length >= 2);
    });
});
