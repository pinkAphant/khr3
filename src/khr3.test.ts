import assert from "node:assert/strict";
import { describe, it } from "node:test";
import * as THREE from "three";
import RAPIER from "@dimforge/rapier3d-compat";
import { parseGlbJson } from "./glb.js";
import {
    ATTACHED_PANEL_INTERACTION_GROUPS,
    KHR_TRIMESH_FLAGS,
    applyRapierBodyMass,
    buildBodyMassTarget,
    buildJointRuntime,
    closedTargetForLimits,
    cloneKhrBundle,
    createRapierImpulseJoint,
    createScopedGltfLoaderClass,
    disposeKhrBundle,
    exportKhrGlb,
    isJointOpen,
    loadKhrGlb,
    planAssembly,
    toggleDrivenJoint,
} from "./index.js";
import { registerMountedSubtree } from "./clone.js";
import { mergeVehiclePanelUserData, solveWheelLoads } from "./vehicle.js";

describe("@khr3/core physics, glTF round-trip, and Rapier WASM joints", () => {
    it("round-trips KHR_physics_rigid_bodies + OMI_vehicle_wheel, deduplicates tables, and preserves sceneExtras.tune / ecctrl", async () => {
        const root = new THREE.Group();
        const chassis = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.5, 4.0), new THREE.MeshStandardMaterial());
        chassis.name = "Chassis";
        root.add(chassis);

        // 3-blade asymmetric propeller whose mesh AABB is offset from hub shaft [0, 0.6, -1.8]
        const propGroup = new THREE.Group();
        propGroup.name = "Prop_Rear";
        propGroup.position.set(0, 0.6, -1.8);
        for (let b = 0; b < 3; b++) {
            const blade = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.02, 0.6));
            const ang = (b * 2 * Math.PI) / 3;
            blade.position.set(Math.sin(ang) * 0.3, 0, Math.cos(ang) * 0.3);
            blade.rotation.y = ang;
            propGroup.add(blade);
        }
        chassis.add(propGroup);

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
                {
                    nodeName: "Chassis",
                    motion: { mass: 1200 },
                    collider: {
                        shape: { type: "box", size: [1.8, 0.5, 4.0] },
                        friction: 0.6,
                        collisionSystems: ["vehicle", "vehicle"],
                        collideWithSystems: ["world", "vehicle"],
                    },
                },
                {
                    nodeName: "Prop_Rear",
                    extras: { ecctrlPropeller: { maxThrust: 4500 } },
                },
                ...wheels.map(([nodeName, , , steer]) => ({
                    nodeName,
                    wheel: { radius: 0.34, width: 0.24, steering: steer, propulsion: true, suspensionTravel: 0.22 },
                })),
            ],
            { tune: { engineHorsepower: 260, engineMaxRPM: 7200 } },
        );

        // Verify OMI_vehicle_wheel table deduplicated 4 wheels -> 2 unique specs (steered vs non-steered)
        const rawJson = parseGlbJson(glb);
        assert.equal(rawJson.extensions?.OMI_vehicle_wheel?.wheels?.length, 2);

        const bundle = await loadKhrGlb(glb);
        assert.equal(bundle.bodies.length, 1);
        assert.equal(bundle.wheels.length, 4);
        assert.equal(bundle.propellers.length, 1);
        // Hub shaft stays at exact node pivot [0, 0.6, -1.8], NOT shifted by 3-blade AABB bias
        assert.ok(Math.abs(bundle.propellers[0].position[0]) < 1e-6);
        assert.ok(Math.abs(bundle.propellers[0].position[1] - 0.6) < 1e-6);
        assert.ok(Math.abs(bundle.propellers[0].position[2] - -1.8) < 1e-6);
        assert.equal(bundle.tune?.engineHorsepower, 260);

        const solved = solveWheelLoads(bundle.wheels, bundle.totalDynamicMass, bundle.centerOfMass, 0.7);
        assert.equal(solved.length, 4);
        assert.ok(solved.every((w) => w.springK > 0 && w.dampingC > 0));

        disposeKhrBundle(bundle);
    });

    it("delegates trimesh (FIX_INTERNAL_EDGES), convexHull, and explicit-CoM inertia directly to Rapier WASM", async () => {
        await RAPIER.init();
        const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
        assert.equal(KHR_TRIMESH_FLAGS, RAPIER.TriMeshFlags.FIX_INTERNAL_EDGES);

        const boxGeo = new THREE.BoxGeometry(2, 3, 4);
        const verts = new Float32Array(boxGeo.getAttribute("position").array);
        const indices = new Uint32Array(boxGeo.getIndex()!.array);
        const mass = 24;
        const expectedSorted = [
            (mass * (4 + 9)) / 12,
            (mass * (4 + 16)) / 12,
            (mass * (9 + 16)) / 12,
        ];

        for (const desc of [
            RAPIER.ColliderDesc.trimesh(verts, indices, KHR_TRIMESH_FLAGS),
            RAPIER.ColliderDesc.convexHull(verts)!,
        ]) {
            const rb = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
            world.createCollider(desc, rb);
            applyRapierBodyMass(rb, {
                mass,
                com: [0.25, -0.5, 0],
                hasExplicitCom: true,
                inertiaDiagonal: [0, 0, 0],
                inertiaFrame: [0, 0, 0, 1],
                hasExplicitInertia: false,
            });
            assert.ok(Math.abs(rb.mass() - mass) < 1e-4);
            const pi = rb.principalInertia();
            const gotSorted = [pi.x, pi.y, pi.z].sort((a, b) => a - b);
            for (let k = 0; k < 3; k++) {
                assert.ok(Math.abs(gotSorted[k] - expectedSorted[k]) < 1e-3, `Rapier WASM I[${k}] = ${gotSorted[k]} != ${expectedSorted[k]}`);
            }
            const com = rb.localCom();
            assert.ok(Math.abs(com.x - 0.25) < 1e-5 && Math.abs(com.y - -0.5) < 1e-5);
        }

        world.free();
        boxGeo.dispose();
    });

    it("drives revolute motor targets and detaches breakaway joints in real Rapier WASM", async () => {
        await RAPIER.init();
        const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });

        const root = new THREE.Group();
        const frame = new THREE.Mesh(new THREE.BoxGeometry(0.2, 2.1, 0.2));
        frame.name = "DoorFrame";
        const leaf = new THREE.Mesh(new THREE.BoxGeometry(0.9, 2.1, 0.05));
        leaf.name = "DoorLeaf";
        leaf.position.set(0.45, 0, 0);
        frame.add(leaf);
        root.add(frame);

        const glb = await exportKhrGlb(root, [
            {
                nodeName: "DoorFrame",
                motion: { isKinematic: true },
                collider: { shape: { type: "box", size: [0.2, 2.1, 0.2] } },
            },
            {
                nodeName: "DoorLeaf",
                motion: { mass: 35 },
                collider: { shape: { type: "box", size: [0.9, 2.1, 0.05] } },
                joint: {
                    connectedNodeName: "DoorFrame",
                    kind: "revolute",
                    axis: [0, 1, 0],
                    limits: [0, Math.PI / 2],
                    drive: { mode: "acceleration", positionTarget: 0, stiffness: 180, damping: 18 },
                },
                extras: { role: "door", breakaway: { latchImpulseNs: 40, breakImpulseNs: 180 } },
            },
        ]);

        const bundle = await loadKhrGlb(glb);
        const plans = planAssembly(bundle);
        assert.equal(plans.length, 1);
        assert.equal(plans[0].parts.length, 1);

        const part = plans[0].parts[0];
        const parentBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const childBody = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(part.restRelRootPos[0], part.restRelRootPos[1], part.restRelRootPos[2]),
        );
        const childCol = world.createCollider(
            RAPIER.ColliderDesc.cuboid(0.45, 1.05, 0.025).setMass(35).setCollisionGroups(ATTACHED_PANEL_INTERACTION_GROUPS),
            childBody,
        );
        childBody.userData = mergeVehiclePanelUserData({ kolmoSpawnedId: "veh_42", name: "TestCoupe" });

        const imp = createRapierImpulseJoint(
            RAPIER,
            world,
            parentBody,
            childBody,
            part.joint,
            part.restRelParentPos,
            [0, 0, 0],
            part.axisParent,
            part.axisChild,
            part.restRelParentQuat,
        );
        const rt = buildJointRuntime({
            rapier: RAPIER,
            world,
            nodeName: part.spec.nodeName,
            role: part.joint.role,
            kind: part.joint.kind,
            object: part.spec.object,
            axisParent: part.axisParent,
            anchorParent: part.restRelParentPos,
            limits: [0.6, Math.PI / 2],
            drive: part.joint.drive,
            breakaway: part.spec.breakaway,
            rapierJoint: imp,
            parentBody,
            childBody,
        });

        assert.equal(closedTargetForLimits([0.6, Math.PI / 2]), 0.6);
        rt.applyTarget(0.6);
        assert.equal(isJointOpen(rt), false, "Joint at lower limit 0.6 in [0.6, pi/2] must read closed");
        assert.equal(toggleDrivenJoint(rt), true);
        assert.equal(rt.positionTarget, Math.PI / 2);
        assert.equal(isJointOpen(rt), true);
        assert.equal(toggleDrivenJoint(rt), true);
        assert.equal(rt.positionTarget, 0.6, "Closing joint with positive lower limit must clamp to lower limit 0.6");

        rt.applyTarget(Math.PI / 3);
        for (let i = 0; i < 90; i++)
            world.step();

        const angle = rt.getCoordinate();
        assert.ok(Math.abs(angle - Math.PI / 3) < 0.08, `Expected door angle near pi/3, got ${angle}`);

        assert.equal(rt.applyImpactImpulse(60), "unlatched");
        assert.equal(rt.unlatched, true);
        assert.equal(rt.applyImpactImpulse(240), "detached");
        assert.equal(rt.detached, true);
        assert.equal(childCol.collisionGroups(), 0xffffffff);
        const detachedUd = childBody.userData as { kolmoSpawnedId?: string; ecctrl?: { excludeVehicleRay?: boolean } };
        assert.equal(detachedUd.kolmoSpawnedId, "veh_42");
        assert.equal(detachedUd.ecctrl?.excludeVehicleRay, false);

        world.free();
        disposeKhrBundle(bundle);
    });

    it("scopes subtree restorers per scene during cloneKhrBundle and provides constructible ScopedGLTFLoader", async () => {
        const sceneA = new THREE.Group();
        const boxA = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
        boxA.name = "CrateA";
        sceneA.add(boxA);
        const bundleA = await loadKhrGlb(
            await exportKhrGlb(sceneA, [{ nodeName: "CrateA", motion: { mass: 10 }, collider: { shape: { type: "box", size: [1, 1, 1] } } }]),
        );

        const unrelatedScene = new THREE.Group();
        let unrelatedTouched = 0;
        const untrack = registerMountedSubtree(unrelatedScene, () => {
            unrelatedTouched++;
            return () => {};
        });

        const clonedA = cloneKhrBundle(bundleA);
        assert.equal(unrelatedTouched, 0, "Cloning bundleA must never touch mounted subtrees on unrelated scenes");
        untrack();

        const ScopedProto = createScopedGltfLoaderClass();
        const loaderInstance = new ScopedProto();
        assert.equal(typeof loaderInstance.parse, "function");

        // Verify exporting a scene with temporarily detached mounted subtrees restores them without disturbing live slot
        const slot = new THREE.Group();
        bundleA.scene.remove(bundleA.bodies[0].object);
        slot.add(bundleA.bodies[0].object);
        const untrackMounted = registerMountedSubtree(bundleA.scene, () => {
            slot.remove(bundleA.bodies[0].object);
            bundleA.scene.add(bundleA.bodies[0].object);
            return () => {
                bundleA.scene.remove(bundleA.bodies[0].object);
                slot.add(bundleA.bodies[0].object);
            };
        });
        const liveExported = await exportKhrGlb(bundleA.scene, [
            { nodeName: "CrateA", motion: { mass: 10 }, collider: { shape: { type: "box", size: [1, 1, 1] } } },
        ]);
        assert.equal(bundleA.bodies[0].object.parent, slot, "Live mounted subtree must remain attached to slot");
        assert.ok(buildBodyMassTarget(bundleA.bodies[0]) !== null);
        const reimported = await loadKhrGlb(liveExported);
        assert.equal(reimported.bodies.length, 1);
        untrackMounted();
        disposeKhrBundle(reimported);

        disposeKhrBundle(clonedA, { disposeTemplate: true });
        boxA.geometry.dispose();
    });
});
