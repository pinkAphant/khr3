export { PhysicsWorld, type PhysicsWorldProps, type TimeControlValue } from "./PhysicsWorld.js";
export {
    JointRegistryContext,
    KhrValidationError,
    assignRef,
    type CurveData,
    type CurvePoint,
    type EcctrlPropellerTag,
    type EcctrlWheelTag,
    type JointRegistry,
    type JointRuntime,
    type KhrAssetBundle,
    type KhrBodySpec,
    type KhrBreakawaySpec,
    type KhrCarConfig,
    type KhrColliderShape,
    type KhrColliderSpec,
    type KhrCombineMode,
    type KhrCurveData,
    type KhrCurvePoint,
    type KhrDroneConfig,
    type KhrEmitterSpec,
    type KhrExportNodeSpec,
    type KhrImplicitShape,
    type KhrJointDriveSpec,
    type KhrJointSpec,
    type KhrPluginOptions,
    type LoadKhrGlbOptions,
    type OmiVehicleWheelSpec,
    type UseKhrGlbOptions,
} from "./types.js";
export { createKhrGltfPlugin, loadKhrGlb } from "./loader.js";
export { exportKhrGlb, patchGlbJsonChunk } from "./exporter.js";
export type { MutableGltfJson, RawGltfJson } from "./glb.js";
export { claimBundleMount, cloneKhrBundle, disposeKhrBundle, releaseBundleMount, snapshotSceneWithMountedSubtrees } from "./clone.js";
export {
    applyRapierBodyMass,
    assertIdentityWorldScale,
    assertSceneIdentityWorldScale,
    assertUnitScale,
    computeLumpedMassProps,
    computeShapeInertia,
    jacobiPrincipalInertia,
    resolveBodyMassFromColliders,
    type LumpedMassProps,
} from "./inertia.js";
export {
    KHR_HEIGHTFIELD_FLAGS,
    KhrHeightfield,
    createHeightfieldGeometry,
    sampleRapierHeightfield,
    transposeHeightfieldForRapier,
    type HeightfieldLayout,
    type KhrHeightfieldProps,
} from "./heightfield.js";
export { KHR_TRIMESH_FLAGS, KhrCollider, type ColliderMass, type KhrColliderProps } from "./KhrCollider.js";
export { KhrAssembly, planAssembly, type KhrAssemblyHandle, type KhrAssemblyProps } from "./KhrAssembly.js";
export { KhrGLTFLoader, createScopedGltfLoaderClass, useKhrGlb } from "./useKhrGlb.js";
export {
    ATTACHED_PANEL_INTERACTION_GROUPS,
    DETACHED_PANEL_INTERACTION_GROUPS,
    buildJointRuntime,
    closedTargetForLimits,
    configureJointDrive,
    createRapierImpulseJoint,
    isJointOpen,
    measureJointCoordinate,
    openTargetForLimits,
    toggleDrivenJoint,
} from "./joints.js";
