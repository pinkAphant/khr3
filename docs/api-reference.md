# `@khr3/core` API Reference

`@khr3/core` loads, validates, clones, exports, and simulates `.glb` assets authored with Khronos `KHR_physics_rigid_bodies`, `KHR_implicit_shapes`, `OMI_vehicle_wheel`, and `KOLMO_emitter` in Three.js, React Three Fiber (`@react-three/fiber`), Rapier WASM (`@react-three/rapier`), and `@pinkaphant/ecctrl`.

---

## Coordinate System & Scale Invariants

- **Right-Handed SI Units**: `1.0` scene unit = `1.0 m`, `+Y` up, `+Z` forward, masses in `kg`, inertia in `kg·m²`, stiffness in `N/m` or `N·m/rad` (or `rad/s²` in acceleration mode), impulses in `N·s`.
- **Unit Scale (`[1, 1, 1]`)**: Every node in `bundle.scene` and every `<KhrAssembly>` / `<KhrVehicle>` wrapper must have scale `1`. Non-unit scales throw `KhrValidationError`.

---

## Import Paths

```ts
import {
  // World & R3F components
  PhysicsWorld,
  KhrAssembly,
  KhrCollider,
  KhrHeightfield,

  // Heightfield utilities (Rapier column-major safe)
  KHR_HEIGHTFIELD_FLAGS,
  sampleRapierHeightfield,
  transposeHeightfieldForRapier,
  createHeightfieldGeometry,

  // GLB loading, cloning, disposal & export
  loadKhrGlb,
  useKhrGlb,
  createKhrGltfPlugin,
  KhrGLTFLoader,
  cloneKhrBundle,
  disposeKhrBundle,
  exportKhrGlb,
  patchGlbJsonChunk,

  // Inertia & mass math
  computeShapeInertia,
  resolveBodyMassFromColliders,
  jacobiPrincipalInertia,
  computeLumpedMassProps,
  applyRapierBodyMass,

  // Impulse joints & breakaway helpers
  JointRegistryContext,
  KHR_TRIMESH_FLAGS,
  ATTACHED_PANEL_INTERACTION_GROUPS,
  DETACHED_PANEL_INTERACTION_GROUPS,
  createRapierImpulseJoint,
  buildJointRuntime,
  toggleDrivenJoint,
  isJointOpen,
  openTargetForLimits,
  closedTargetForLimits,

  // Validation error
  KhrValidationError,
} from "@khr3/core";

import {
  KhrVehicle,
  solveWheelLoads,
  mergeVehiclePanelUserData,
  type KhrVehicleProps,
  type KhrVehicleJointsHandle,
  type SolvedWheel,
} from "@khr3/core/vehicle";
```

---

## Decoded `KhrAssetBundle` Schema

`loadKhrGlb(buffer, options?)` and `useKhrGlb(url, options?)` return a `KhrAssetBundle`:

| Field | Type | Description |
| --- | --- | --- |
| `scene` | `THREE.Group` | Parsed Three.js scene graph (unit scale verified). |
| `bodies` | `KhrBodySpec[]` | Kinematic & dynamic rigid bodies with colliders, exact `centerOfMass`, `inertiaDiagonal`, `inertiaOrientation`, optional `joint`, `breakaway`, and `role`. |
| `wheels` | `EcctrlWheelTag[]` | Decoded `OMI_vehicle_wheel` and/or `extras.ecctrlWheel` wheel tags with exact hub pivots. |
| `propellers` | `EcctrlPropellerTag[]` | Decoded `extras.ecctrlPropeller` rotor tags. |
| `emitters` | `KhrEmitterSpec[]` | Decoded `extensions.KOLMO_emitter` VFX emitters (`kind`, `drive`, `color`, `size`, `position`, `quaternion`, `object`). |
| `sockets` | `Record<string, [number, number, number]>` | Named scene-space attachment points from `extensions.KOLMO_emitter.socket` (both semantic key e.g. `"seat"`, `"exit"`, `"muzzle"` and node name e.g. `"Socket_Seat"`), `extras.socketId`, `extras.isIkAnchor`, `extras.contactDots`, `scene.extras.ikSockets`, `Socket_*`/`IK_*` nodes, and `Seat_L`/`Seat_R` fallbacks. |
| `totalDynamicMass` | `number` | Sum of `mass` across all non-kinematic bodies (`kg`). |
| `centerOfMass` | `[number, number, number]` | Mass-weighted assembly center of mass in scene space (`m`). |
| `tune` | `Partial<KhrCarConfig>` | Optional car powertrain/steering tune from `scene.extras.tune` or `scene.extras.ecctrl`. |
| `droneTune` | `Partial<KhrDroneConfig>` | Optional drone controller tune from `scene.extras.droneTune`. |

### Automatic Role Extraction

Unless overridden via `options.extractRole(nodeName, extras)`, each `KhrBodySpec.role` (and `joint.role`) resolves the first non-empty string among:
`extras.role ?? extras.vehicleRole ?? extras.buildingRole ?? extras.weaponRole ?? null`.

---

## Exact Inertia & Mass Solver

When a dynamic node in `KHR_physics_rigid_bodies` specifies `motion.mass` without an explicit `motion.inertiaDiagonal`:

1. **Closed Solid Meshes (`trimesh` / 3D `convexHull`)**: Integrated analytically via the 3D signed-tetrahedron divergence theorem (`integrateClosedMeshProps`).
2. **Open Thin Sheet-Metal Shells (`trimesh` / coplanar `convexHull` with zero enclosed volume)**: Integrated analytically via exact 2D triangle surface second moments (`integrateSurfaceShellProps`). Degenerate zero-area collinear meshes throw `KhrValidationError`.
3. **Steiner Parallel-Axis + Jacobi Principal Frame**: Multi-collider bodies combine volume-weighted centroids, shift to `motion.centerOfMass` via the full $3\times 3$ parallel-axis tensor $I_c = I_{\text{cm}} + m(\|\mathbf{d}\|^2 I_3 - \mathbf{d}\mathbf{d}^\top)$, and diagonalize off-diagonal products via a 32-sweep Jacobi eigensolver (`jacobiPrincipalInertia`) into `SO(3)` `inertiaOrientation` + positive `inertiaDiagonal`.
4. **Rapier WASM Application (`applyRapierBodyMass`)**: Sets collider densities to `0` and applies `LumpedMassProps` directly via `rb.setAdditionalMassProperties(..., true)`.

---

## Column-Major Rapier Heightfields (`<KhrHeightfield>`)

Rapier 0.19 `ColliderDesc.heightfield(nrows, ncols, heights, scale, flags)` expects `(nrows + 1) * (ncols + 1)` heights indexed in **column-major** order (`heights[xCol * (nrows + 1) + zRow]`), and `@react-three/rapier@2.2.0`'s `<HeightfieldCollider>` has a `scaleColliderArgs` bug that multiplies `s.x` by `scale.x * scale.y * scale.z`.

`<KhrHeightfield>` calls Rapier WASM directly with `KHR_HEIGHTFIELD_FLAGS` (`HeightFieldFlags.FIX_INTERNAL_EDGES = 1`) and `scale = { x: sizeX, y: 1, z: sizeZ }`:

- **`sampleRapierHeightfield(fn, originX, originZ, sizeX, sizeZ, nrows, ncols?)`**: Samples `fn(x, z)` directly in Rapier column-major order.
- **`transposeHeightfieldForRapier(rowMajor, nrows, ncols?)`**: Converts a row-major height array (such as `@khr3/roads` `sampleHeightfield`) to Rapier column-major order, or pass `layout="row-major"` directly to `<KhrHeightfield>` and `createHeightfieldGeometry`.
- **`createHeightfieldGeometry(heights, sizeX, sizeZ, nrows, ncols?, layout?)`**: Builds an indexed `THREE.BufferGeometry` centered at `[0, 0, 0]` whose cell diagonals and vertex heights match Rapier 1:1.

---

## `<KhrAssembly>` & Static Compound Optimization

`<KhrAssembly bundle={bundle} position={[x, y, z]} fixedRoot />` mounts single-body props, multi-body articulated mechanisms (doors, drawers, gun slides, breakaway panels), and static architectural pieces:

- **Articulated Parts**: Driven via `ref.current.setJointTarget(nodeName, radOrMeters)`, `setRoleTarget(role, target)`, or `toggleHitObject(raycastHit.object)`.
- **Static Multi-Root Compounding**: When `fixedRoot` is `true` and every body in `bundle` is kinematic with zero joints (e.g. `city/stairs_3x3.glb` with 16 treads or `city/window_fixed_1800x1200.glb` with 11 bars), `<KhrAssembly>` automatically merges all colliders into **one single `type="fixed"` Rapier `RigidBody`** with zero `MountSubtreeSlot` reparenting overhead.

---

## `<KhrVehicle>` (`@khr3/core/vehicle`)

Wraps `@pinkaphant/ecctrl/vehicle` (`<EcctrlVehicle>`, `<ShapeCastWheel>`, `<ThrustPropeller>`) with automatic static-equilibrium suspension load solving (`solveWheelLoads`) and articulated chassis doors/hoods/trunks (`<KhrVehicleJoints>`):

```tsx
const vehicleRef = useRef<EcctrlVehicleHandle>(null);
const jointsRef = useRef<KhrVehicleJointsHandle>(null);

<KhrVehicle
  ref={vehicleRef}
  jointsRef={jointsRef}
  bundle={carBundle}
  position={[0, 1.5, 0]}
  carConfig={{ transmissionMode: "auto" }}
/>
```

---

## Vite Bundler Configuration (`resolve.dedupe`)

When bundling R3F + Rapier + `@pinkaphant/ecctrl` + `@khr3/core` in Vite, always dedupe singletons so React contexts and Rapier WASM worlds stay unified:

```ts
export default defineConfig({
  resolve: {
    dedupe: ["react", "react-dom", "three", "@react-three/fiber", "@react-three/rapier", "@dimforge/rapier3d-compat"],
  },
});
```
