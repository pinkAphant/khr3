import assert from "node:assert/strict";
import { describe, it } from "node:test";
import * as THREE from "three";
import RAPIER from "@dimforge/rapier3d-compat";
import { parseGlbJson } from "./glb.js";
import {
    ATTACHED_PANEL_INTERACTION_GROUPS,
    KHR_HEIGHTFIELD_FLAGS,
    KHR_TRIMESH_FLAGS,
    KhrValidationError,
    applyRapierBodyMass,
    buildJointRuntime,
    closedTargetForLimits,
    cloneKhrBundle,
    computeLumpedMassProps,
    computeShapeInertia,
    createHeightfieldGeometry,
    createRapierImpulseJoint,
    createScopedGltfLoaderClass,
    disposeKhrBundle,
    exportKhrGlb,
    isJointOpen,
    jacobiPrincipalInertia,
    loadKhrGlb,
    planAssembly,
    resolveBodyMassFromColliders,
    sampleRapierHeightfield,
    toggleDrivenJoint,
    transposeHeightfieldForRapier,
} from "./index.js";
import { registerMountedSubtree } from "./clone.js";
import { mergeVehiclePanelUserData, solveWheelLoads } from "./vehicle.js";

describe("@khr3/core physics, glTF round-trip, inertia math, and Rapier WASM joints", () => {
    it("round-trips KHR_physics_rigid_bodies + OMI_vehicle_wheel + KOLMO_emitter + role schemas", async () => {
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

        const socketSeat = new THREE.Object3D();
        socketSeat.name = "Socket_Seat";
        socketSeat.position.set(-0.42, 0.38, 0.15);
        chassis.add(socketSeat);

        const socketExit = new THREE.Object3D();
        socketExit.name = "Socket_Exit";
        socketExit.position.set(-1.55, 0.55, 0.15);
        chassis.add(socketExit);

        const plumeL = new THREE.Object3D();
        plumeL.name = "Emitter_Plume_L";
        plumeL.position.set(-0.45, 0.18, -2.05);
        chassis.add(plumeL);

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
                    extras: { vehicleRole: "chassis_main" },
                },
                {
                    nodeName: "Prop_Rear",
                    extras: { ecctrlPropeller: { maxThrust: 4500 } },
                },
                {
                    nodeName: "Socket_Seat",
                    extensions: { KOLMO_emitter: { socket: "seat" } },
                },
                {
                    nodeName: "Socket_Exit",
                    extensions: { KOLMO_emitter: { socket: "exit" } },
                },
                {
                    nodeName: "Emitter_Plume_L",
                    extensions: { KOLMO_emitter: { kind: "plume", drive: "boost", color: [0.22, 0.91, 1], size: 1.35 } },
                },
                ...wheels.map(([nodeName, , , steer]) => ({
                    nodeName,
                    wheel: { radius: 0.34, width: 0.24, steering: steer, propulsion: true, suspensionTravel: 0.22 },
                })),
            ],
            { tune: { engineHorsepower: 260, engineMaxRPM: 7200 } },
        );

        const rawJson = parseGlbJson(glb);
        assert.equal(rawJson.extensions?.OMI_vehicle_wheel?.wheels?.length, 2);

        const bundle = await loadKhrGlb(glb);
        assert.equal(bundle.bodies.length, 1);
        assert.equal(bundle.bodies[0].role, "chassis_main");
        // Offline box inertia populated immediately on loadKhrGlb
        assert.ok(Math.abs(bundle.bodies[0].inertiaDiagonal[0] - (1200 * (0.25 + 16)) / 12) < 1e-4);
        assert.ok(Math.abs(bundle.bodies[0].inertiaDiagonal[1] - (1200 * (3.24 + 16)) / 12) < 1e-4);
        assert.ok(Math.abs(bundle.bodies[0].inertiaDiagonal[2] - (1200 * (3.24 + 0.25)) / 12) < 1e-4);

        assert.equal(bundle.wheels.length, 4);
        assert.equal(bundle.propellers.length, 1);
        assert.ok(Math.abs(bundle.propellers[0].position[0]) < 1e-6);
        assert.ok(Math.abs(bundle.propellers[0].position[1] - 0.6) < 1e-6);
        assert.ok(Math.abs(bundle.propellers[0].position[2] - -1.8) < 1e-6);

        assert.deepEqual(bundle.sockets.seat?.map((v) => Number(v.toFixed(4))), [-0.42, 0.38, 0.15]);
        assert.deepEqual(bundle.sockets.Socket_Seat?.map((v) => Number(v.toFixed(4))), [-0.42, 0.38, 0.15]);
        assert.deepEqual(bundle.sockets.exit?.map((v) => Number(v.toFixed(4))), [-1.55, 0.55, 0.15]);
        assert.equal(bundle.emitters.length, 1);
        assert.equal(bundle.emitters[0].kind, "plume");
        assert.equal(bundle.emitters[0].drive, "boost");
        assert.deepEqual(bundle.emitters[0].color, [0.22, 0.91, 1]);
        assert.equal(bundle.emitters[0].size, 1.35);

        const cloned = cloneKhrBundle(bundle);
        assert.equal(cloned.emitters.length, 1);
        assert.notEqual(cloned.emitters[0].object, bundle.emitters[0].object);
        assert.deepEqual(cloned.sockets.seat, bundle.sockets.seat);

        const solved = solveWheelLoads(bundle.wheels, bundle.totalDynamicMass, bundle.centerOfMass, 0.7);
        assert.equal(solved.length, 4);
        assert.ok(solved.every((w) => w.springK > 0 && w.dampingC > 0));

        disposeKhrBundle(cloned);
        disposeKhrBundle(bundle);
    });

    it("integrates exact closed-mesh divergence, open surface shells, Steiner parallel-axis CoM, and Jacobi principal frames", async () => {
        await RAPIER.init();
        const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
        assert.equal(KHR_TRIMESH_FLAGS, RAPIER.TriMeshFlags.FIX_INTERNAL_EDGES);

        // 1. Closed solid trimesh & convexHull vs exact analytical 2x3x4 box inertia
        const boxGeo = new THREE.BoxGeometry(2, 3, 4);
        const verts = new Float32Array(boxGeo.getAttribute("position").array);
        const indices = new Uint32Array(boxGeo.getIndex()!.array);
        const mass = 24;
        const exactBox = [
            (mass * (9 + 16)) / 12,
            (mass * (4 + 16)) / 12,
            (mass * (4 + 9)) / 12,
        ];

        const triInertia = computeShapeInertia({ type: "trimesh", vertices: verts, indices, meshNodeName: "BoxTri" }, mass);
        const hullInertia = computeShapeInertia({ type: "convexHull", vertices: verts, meshNodeName: "BoxHull" }, mass);
        for (let k = 0; k < 3; k++) {
            assert.ok(Math.abs(triInertia[k] - exactBox[k]) < 1e-4, `triInertia[${k}] = ${triInertia[k]} != ${exactBox[k]}`);
            assert.ok(Math.abs(hullInertia[k] - exactBox[k]) < 1e-4, `hullInertia[${k}] = ${hullInertia[k]} != ${exactBox[k]}`);
        }

        // 2. Rotated collider + offset explicit centerOfMass: Steiner parallel-axis + Jacobi eigensolver
        const rotQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 2, -1).normalize(), 0.73);
        const rotVerts = new Float32Array(verts.length);
        const tmp = new THREE.Vector3();
        for (let i = 0; i < verts.length; i += 3) {
            tmp.set(verts[i], verts[i + 1], verts[i + 2]).applyQuaternion(rotQuat).add(new THREE.Vector3(0.45, 1.05, 0));
            rotVerts[i] = tmp.x;
            rotVerts[i + 1] = tmp.y;
            rotVerts[i + 2] = tmp.z;
        }
        // Authored CoM right at the shifted centroid [0.45, 1.05, 0] -> principal eigenvalues must STILL equal exactBox (sorted)
        const solvedShifted = resolveBodyMassFromColliders(
            [
                {
                    shape: { type: "trimesh", vertices: rotVerts, indices, meshNodeName: "RotDoor" },
                    localPosition: [0, 0, 0],
                    localQuaternion: [0, 0, 0, 1],
                },
            ],
            mass,
            [0.45, 1.05, 0],
            null,
            "RotDoor",
        );
        const gotPrincipal = [...solvedShifted.inertiaDiagonal].sort((a, b) => a - b);
        const expPrincipal = [...exactBox].sort((a, b) => a - b);
        for (let k = 0; k < 3; k++) {
            assert.ok(Math.abs(gotPrincipal[k] - expPrincipal[k]) < 1e-3, `Jacobi principal[${k}] = ${gotPrincipal[k]} != ${expPrincipal[k]}`);
        }

        // Now shift explicit CoM by d = [1, 0, 0] along world X on axis-aligned 2x3x4 box -> Iyy and Izz increase by m * d^2 = 24
        const steinerShifted = resolveBodyMassFromColliders(
            [
                {
                    shape: { type: "box", size: [2, 3, 4] },
                    localPosition: [0, 0, 0],
                    localQuaternion: [0, 0, 0, 1],
                },
            ],
            mass,
            [1, 0, 0],
            null,
            "SteinerBox",
        );
        assert.ok(Math.abs(steinerShifted.inertiaDiagonal[0] - exactBox[0]) < 1e-6);
        assert.ok(Math.abs(steinerShifted.inertiaDiagonal[1] - (exactBox[1] + mass * 1)) < 1e-6);
        assert.ok(Math.abs(steinerShifted.inertiaDiagonal[2] - (exactBox[2] + mass * 1)) < 1e-6);

        // 3. Open 2D thin rectangular lamina (width W=2 along X, height H=3 along Y, zero thickness Z=0):
        // Exact lamina inertia: Ixx = m H^2 / 12, Iyy = m W^2 / 12, Izz = m (W^2 + H^2) / 12
        const quadVerts = new Float32Array([
            -1, -1.5, 0,
            1, -1.5, 0,
            1, 1.5, 0,
            -1, 1.5, 0,
        ]);
        const quadIdx = new Uint32Array([0, 1, 2, 0, 2, 3]);
        const shellI = computeShapeInertia({ type: "trimesh", vertices: quadVerts, indices: quadIdx, meshNodeName: "ThinDoorSkin" }, 12);
        assert.ok(Math.abs(shellI[0] - (12 * 9) / 12) < 1e-5);
        assert.ok(Math.abs(shellI[1] - (12 * 4) / 12) < 1e-5);
        assert.ok(Math.abs(shellI[2] - (12 * 13) / 12) < 1e-5);

        // Collinear zero-area vertices MUST throw KhrValidationError (never fake AABB or 0.04*mass!)
        const collinear = new Float32Array([0, 0, 0, 1, 0, 0, 2, 0, 0]);
        assert.throws(
            () => computeShapeInertia({ type: "trimesh", vertices: collinear, indices: new Uint32Array([0, 1, 2]), meshNodeName: "BadLine" }, 10),
            KhrValidationError,
        );

        // 4. Verify applyRapierBodyMass sets exact mass, CoM, and principal inertia in Rapier WASM with collider density = 0
        const rb = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
        const col = world.createCollider(RAPIER.ColliderDesc.trimesh(verts, indices, KHR_TRIMESH_FLAGS), rb);
        const lumped = computeLumpedMassProps([
            {
                nodeName: "TestBody",
                isKinematic: false,
                mass,
                centerOfMass: steinerShifted.centerOfMass,
                inertiaDiagonal: steinerShifted.inertiaDiagonal,
                inertiaOrientation: steinerShifted.inertiaOrientation,
                localPosition: [0, 0, 0],
                localQuaternion: [0, 0, 0, 1],
                colliders: [],
                joint: null,
                object: new THREE.Object3D(),
                role: null,
            },
        ]);
        applyRapierBodyMass(rb, lumped);
        assert.equal(col.density(), 0);
        assert.ok(Math.abs(rb.mass() - mass) < 1e-4);
        assert.ok(Math.abs(rb.localCom().x - 1) < 1e-5);
        const pi = rb.principalInertia();
        assert.ok(Math.abs(pi.x - exactBox[0]) < 1e-4);
        assert.ok(Math.abs(pi.y - (exactBox[1] + 24)) < 1e-4);
        assert.ok(Math.abs(pi.z - (exactBox[2] + 24)) < 1e-4);

        // Jacobi SO(3) check on arbitrary symmetric positive-definite tensor
        const jac = jacobiPrincipalInertia(14, 19, 23, -3.2, 1.8, -2.5);
        const qNorm = Math.hypot(...jac.inertiaFrame);
        assert.ok(Math.abs(qNorm - 1) < 1e-7);
        assert.ok(jac.inertia.every((v) => v > 0));

        world.free();
        boxGeo.dispose();
    });

    it("aligns Rapier WASM column-major heightfield raycasts and Three.js mesh vertices 1:1 without X<->Z transpose", async () => {
        await RAPIER.init();
        const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
        assert.equal(KHR_HEIGHTFIELD_FLAGS, RAPIER.HeightFieldFlags.FIX_INTERNAL_EDGES);

        // Asymmetric ramp: y(x, z) = 0.5 * x - 0.25 * z + 10 over [-8, +8] x [-8, +8] (sizeX=16, sizeZ=16, n=8)
        const heightFn = (x: number, z: number): number => 0.5 * x - 0.25 * z + 10;
        const nrows = 8, ncols = 8, sizeX = 16, sizeZ = 16;

        // Build row-major grid first (like @khr3/roads sampleHeightfield) and transpose it
        const rowMajor = new Float32Array((nrows + 1) * (ncols + 1));
        for (let zRow = 0; zRow <= nrows; zRow++) {
            for (let xCol = 0; xCol <= ncols; xCol++) {
                const x = -8 + (xCol / ncols) * sizeX;
                const z = -8 + (zRow / nrows) * sizeZ;
                rowMajor[zRow * (ncols + 1) + xCol] = heightFn(x, z);
            }
        }
        const transposed = transposeHeightfieldForRapier(rowMajor, nrows, ncols);
        const directColMajor = sampleRapierHeightfield(heightFn, -8, -8, sizeX, sizeZ, nrows, ncols);
        for (let i = 0; i < transposed.length; i++) {
            assert.ok(Math.abs(transposed[i] - directColMajor[i]) < 1e-6);
        }

        const rb = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const hfDesc = (RAPIER.ColliderDesc as unknown as {
            heightfield: (nr: number, nc: number, h: Float32Array, s: { x: number; y: number; z: number }, flags?: number) => RAPIER.ColliderDesc;
        }).heightfield(nrows, ncols, directColMajor, { x: sizeX, y: 1, z: sizeZ }, KHR_HEIGHTFIELD_FLAGS);
        world.createCollider(hfDesc, rb);
        world.step();

        for (const [qx, qz] of [[4, 0], [0, 4], [-6, 2], [3, -5]] as const) {
            const ray = new RAPIER.Ray({ x: qx, y: 50, z: qz }, { x: 0, y: -1, z: 0 });
            const hit = world.castRay(ray, 100, true);
            assert.ok(hit !== null, `Expected heightfield ray hit at (${qx}, ${qz})`);
            const hitY = 50 - hit.timeOfImpact;
            const expectedY = heightFn(qx, qz);
            assert.ok(
                Math.abs(hitY - expectedY) < 1e-4,
                `At (${qx}, ${qz}): Rapier hitY=${hitY} != expected ${expectedY} (X<->Z transpose check)`,
            );
        }

        // Verify createHeightfieldGeometry matches exact heights at every vertex
        const geo = createHeightfieldGeometry(directColMajor, sizeX, sizeZ, nrows, ncols, "rapier-column-major");
        const posAttr = geo.getAttribute("position");
        for (let i = 0; i < posAttr.count; i++) {
            const vx = posAttr.getX(i), vy = posAttr.getY(i), vz = posAttr.getZ(i);
            assert.ok(Math.abs(vy - heightFn(vx, vz)) < 1e-5);
        }
        geo.dispose();
        world.free();
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
                extras: { buildingRole: "doorframe_leaf", breakaway: { latchImpulseNs: 40, breakImpulseNs: 180 } },
            },
        ]);

        const bundle = await loadKhrGlb(glb);
        assert.equal(bundle.bodies[1].role, "doorframe_leaf");
        const plans = planAssembly(bundle);
        assert.equal(plans.length, 1);
        assert.equal(plans[0].parts.length, 1);

        const part = plans[0].parts[0];
        const parentBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
        const childBody = world.createRigidBody(
            RAPIER.RigidBodyDesc.dynamic().setTranslation(part.restRelRootPos[0], part.restRelRootPos[1], part.restRelRootPos[2]),
        );
        const childCol = world.createCollider(
            RAPIER.ColliderDesc.cuboid(0.45, 1.05, 0.025).setDensity(0).setCollisionGroups(ATTACHED_PANEL_INTERACTION_GROUPS),
            childBody,
        );
        applyRapierBodyMass(childBody, part.lumpedMass);
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
        assert.equal(isJointOpen(rt), false);
        assert.equal(toggleDrivenJoint(rt), true);
        assert.equal(rt.positionTarget, Math.PI / 2);
        assert.equal(isJointOpen(rt), true);
        assert.equal(toggleDrivenJoint(rt), true);
        assert.equal(rt.positionTarget, 0.6);

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
        assert.equal(unrelatedTouched, 0);
        untrack();

        const ScopedProto = createScopedGltfLoaderClass();
        const loaderInstance = new ScopedProto();
        assert.equal(typeof loaderInstance.parse, "function");

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
        assert.equal(bundleA.bodies[0].object.parent, slot);
        assert.ok(computeLumpedMassProps(bundleA.bodies) !== null);
        const reimported = await loadKhrGlb(liveExported);
        assert.equal(reimported.bodies.length, 1);
        untrackMounted();
        disposeKhrBundle(reimported);

        disposeKhrBundle(clonedA, { disposeTemplate: true });
        boxA.geometry.dispose();
    });
});
