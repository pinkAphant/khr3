import { useMemo } from "react";
import { useLoader } from "@react-three/fiber";
import type { GLTFLoaderPlugin, GLTFParser } from "three/examples/jsm/loaders/GLTFLoader.js";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { cloneKhrBundle } from "./khrClone.js";
import {
    KHR_BUNDLE_USERDATA_KEY,
    applyKhrCustomExtractors,
    createHeadlessTexturePlugin,
    createKhrGltfPlugin,
    isHeadlessTextureEnv,
} from "./khrPlugin.js";
import { type KhrAssetBundle, KhrValidationError, type UseKhrGlbOptions } from "./khrTypes.js";

class KhrGLTFLoader extends GLTFLoader {
    static khrLoaderScope = Symbol("KhrGLTFLoader");
}

const headlessPlugin = createHeadlessTexturePlugin();
const khrPlugin = createKhrGltfPlugin();
const configuredLoaders = new WeakSet<GLTFLoader>();
const customExtendedLoaders = new WeakSet<GLTFLoader>();
const customLoaderRegistry = new Map<string, Map<string, { loader: GLTFLoader; registerExtensions: (loader: GLTFLoader) => void }>>();

function configureKhrLoader(loader: GLTFLoader): void {
    if (configuredLoaders.has(loader))
        return;
    configuredLoaders.add(loader);
    if (isHeadlessTextureEnv()) {
        loader.register(headlessPlugin as (parser: GLTFParser) => GLTFLoaderPlugin);
    }
    loader.register(khrPlugin as (parser: GLTFParser) => GLTFLoaderPlugin);
}

function hasCustomLoaderOpts(options?: UseKhrGlbOptions): options is UseKhrGlbOptions {
    return Boolean(options?.extendLoader || options?.cacheKey !== undefined);
}

function getLoaderCacheKey(options: UseKhrGlbOptions): string {
    return options.cacheKey === undefined
        ? `src:${Function.prototype.toString.call(options.extendLoader)}`
        : `key:${options.cacheKey}`;
}

function getScopedLoaderEntry(url: string, options: UseKhrGlbOptions): { loader: GLTFLoader; registerExtensions: (loader: GLTFLoader) => void } {
    const key = getLoaderCacheKey(options);
    let urlMap = customLoaderRegistry.get(url);
    if (!urlMap) {
        urlMap = new Map();
        customLoaderRegistry.set(url, urlMap);
    }
    let entry = urlMap.get(key);
    if (!entry) {
        const loader = new GLTFLoader();
        const extend = options.extendLoader;
        const created = {
            loader,
            registerExtensions: (target: GLTFLoader) => {
                configureKhrLoader(target);
                if (extend && !customExtendedLoaders.has(target)) {
                    try {
                        extend(target);
                        customExtendedLoaders.add(target);
                    }
                    catch (err) {
                        const map = customLoaderRegistry.get(url);
                        if (map?.get(key) === created) {
                            map.delete(key);
                            if (map.size === 0)
                                customLoaderRegistry.delete(url);
                        }
                        throw err;
                    }
                }
            },
        };
        entry = created;
        urlMap.set(key, entry);
    }
    return entry;
}

export function useKhrGlb(url: string, options?: UseKhrGlbOptions): KhrAssetBundle {
    const extractRole = options?.extractRole;
    const isSocketNode = options?.isSocketNode;
    const scoped = hasCustomLoaderOpts(options) ? getScopedLoaderEntry(url, options) : null;
    const loader = scoped ? scoped.loader : KhrGLTFLoader;
    const reg = scoped ? scoped.registerExtensions : configureKhrLoader;
    const gltf = useLoader(loader as typeof GLTFLoader, url, reg);
    const template = gltf.userData[KHR_BUNDLE_USERDATA_KEY] as KhrAssetBundle | undefined;
    if (!template)
        throw new KhrValidationError(`useKhrGlb: missing "${KHR_BUNDLE_USERDATA_KEY}" on loaded GLTF for "${url}"`);
    return useMemo(() => {
        const cloned = cloneKhrBundle(template);
        if (extractRole || isSocketNode)
            applyKhrCustomExtractors(cloned, template, extractRole, isSocketNode);
        return cloned;
    }, [template, extractRole, isSocketNode]);
}

export namespace useKhrGlb {
    export const preload = (url: string, options?: UseKhrGlbOptions): void => {
        const scoped = hasCustomLoaderOpts(options) ? getScopedLoaderEntry(url, options) : null;
        const loader = scoped ? scoped.loader : KhrGLTFLoader;
        const reg = scoped ? scoped.registerExtensions : configureKhrLoader;
        useLoader.preload(loader as typeof GLTFLoader, url, reg);
    };

    export const clear = (url: string, options?: UseKhrGlbOptions): void => {
        if (hasCustomLoaderOpts(options)) {
            const urlMap = customLoaderRegistry.get(url);
            if (!urlMap)
                return;
            const key = getLoaderCacheKey(options);
            const entry = urlMap.get(key);
            if (entry) {
                useLoader.clear(entry.loader as unknown as typeof GLTFLoader, url);
                urlMap.delete(key);
                if (urlMap.size === 0)
                    customLoaderRegistry.delete(url);
            }
        }
        else {
            useLoader.clear(KhrGLTFLoader as typeof GLTFLoader, url);
            if (options === undefined) {
                const urlMap = customLoaderRegistry.get(url);
                if (urlMap) {
                    for (const entry of urlMap.values())
                        useLoader.clear(entry.loader as unknown as typeof GLTFLoader, url);
                    customLoaderRegistry.delete(url);
                }
            }
        }
    };
}
