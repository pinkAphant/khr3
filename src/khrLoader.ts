import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { validateGlbHeader } from "./khrGlb.js";
import { KHR_BUNDLE_USERDATA_KEY, createHeadlessTexturePlugin, createKhrGltfPlugin, isHeadlessTextureEnv } from "./khrPlugin.js";
import { type KhrAssetBundle, KhrValidationError, type LoadKhrGlbOptions } from "./khrTypes.js";

export * from "./khrTypes.js";
export { KHR_BUNDLE_USERDATA_KEY, createHeadlessTexturePlugin, createKhrGltfPlugin };

const headlessPlugin = createHeadlessTexturePlugin();

export function createDefaultGltfLoader(): GLTFLoader {
    const loader = new GLTFLoader();
    if (isHeadlessTextureEnv()) {
        loader.register(headlessPlugin);
    }
    return loader;
}

export async function loadKhrGlb(source: ArrayBuffer | Uint8Array, options?: LoadKhrGlbOptions): Promise<KhrAssetBundle> {
    const bytes = source instanceof Uint8Array ? source : new Uint8Array(source);
    const buf = new ArrayBuffer(bytes.byteLength);
    new Uint8Array(buf).set(bytes);
    validateGlbHeader(buf);
    const loader = options?.loader ?? createDefaultGltfLoader();
    const plugin = createKhrGltfPlugin(options);
    loader.register(plugin);
    try {
        const toErr = (err: unknown): KhrValidationError =>
            err instanceof KhrValidationError
                ? err
                : new KhrValidationError(err instanceof Error ? err.message : "Failed to parse GLB");
        const gltf = await new Promise<{ userData: Record<string, unknown> }>((resolve, reject) => {
            loader.parse(buf, "", resolve as never, (err) => reject(toErr(err)));
        }).catch((err) => {
            throw toErr(err);
        });
        const bundle = gltf.userData[KHR_BUNDLE_USERDATA_KEY] as KhrAssetBundle | undefined;
        if (!bundle)
            throw new KhrValidationError("Missing parsed KHR bundle on GLTF userData");
        return bundle;
    }
    finally {
        loader.unregister(plugin);
    }
}
