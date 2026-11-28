# `@khr3/core`

Khronos `KHR_physics_rigid_bodies` + `KHR_implicit_shapes` + `OMI_vehicle_wheel` + `KOLMO_emitter` loader, binary `.glb` exporter, exact solid/surface inertia solver, column-major Rapier WASM heightfields, articulated multi-body assemblies, and `@pinkaphant/ecctrl` vehicle rigs for React Three Fiber.

- **SI Right-Handed Frame**: `1.0 unit === 1.0 metre`, `+Y` up, `+Z` forward. Every node must have baked unit scale `[1, 1, 1]`.
- **Zero Silent Fallbacks**: Malformed `.glb` schemas, non-unit scales, or degenerate zero-area colliders throw `KhrValidationError`.
- **Full API Reference**: [`docs/api-reference.md`](./docs/api-reference.md)

## Quick Start

```tsx
import { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import {
  PhysicsWorld,
  KhrAssembly,
  KhrHeightfield,
  createHeightfieldGeometry,
  sampleRapierHeightfield,
  useKhrGlb,
} from "@khr3/core";
import { KhrVehicle } from "@khr3/core/vehicle";

const VAULT = "https://khr3.optimuspi.workers.dev/assets";
const heights = sampleRapierHeightfield((x, z) => 2 * Math.sin(x * 0.04) * Math.cos(z * 0.04), -100, -100, 200, 200, 128);
const terrainGeo = createHeightfieldGeometry(heights, 200, 200, 128);

function Scene() {
  const car = useKhrGlb(`${VAULT}/cars/nissan_skyline_r34.glb`);
  const door = useKhrGlb(`${VAULT}/city/door_hinged_single_900x2100.glb`);

  return (
    <PhysicsWorld gravity={[0, -9.81, 0]}>
      <KhrHeightfield heights={heights} segments={128} size={[200, 200]}>
        <mesh geometry={terrainGeo} receiveShadow>
          <meshStandardMaterial color="#3c4a3e" roughness={0.92} />
        </mesh>
      </KhrHeightfield>
      <KhrVehicle bundle={car} position={[0, 2.5, 0]} />
      <KhrAssembly bundle={door} position={[4, 0, 4]} fixedRoot />
    </PhysicsWorld>
  );
}

export default function App() {
  return (
    <Canvas shadows camera={{ position: [0, 6, -12], fov: 55 }}>
      <Suspense fallback={null}>
        <Scene />
      </Suspense>
    </Canvas>
  );
}
```

## Entrypoints

- `@khr3/core` — `PhysicsWorld`, `loadKhrGlb`, `useKhrGlb`, `exportKhrGlb`, `cloneKhrBundle`, `disposeKhrBundle`, `KhrAssembly`, `KhrCollider`, `KhrHeightfield`, `sampleRapierHeightfield`, `transposeHeightfieldForRapier`, `createHeightfieldGeometry`, `computeShapeInertia`, `resolveBodyMassFromColliders`, `jacobiPrincipalInertia`, `computeLumpedMassProps`, `applyRapierBodyMass`, `JointRegistryContext`
- `@khr3/core/vehicle` — `KhrVehicle`, `solveWheelLoads`, `mergeVehiclePanelUserData`
