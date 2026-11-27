import { useMemo } from "react";
import { useLoader } from "@react-three/fiber";
import { GLTFLoader, type GLTFLoaderPlugin, type GLTFParser } from "three/examples/jsm/loaders/GLTFLoader.js";
import { cloneKhrBundle } from "./clone.js";
import { KHR_BUNDLE_USERDATA_KEY, applyKhrCustomExtractors, createKhrGltfPlugin } from "./loader.js";
import { type KhrAssetBundle, KhrValidationError, type UseKhrGlbOptions } from "./types.js";

export class KhrGLTFLoader extends GLTFLoader {
    static readonly khrLoaderScope = Symbol("KhrGLTFLoader");
}

interface ScopedLoaderEntry {
    readonly Proto: typeof GLTFLoader;
    readonly registerExtensions: (loader: GLTFLoader) => void;
}

const khrPlugin = createKhrGltfPlugin();
const configuredLoaders = new WeakSet<GLTFLoader>();
const customExtendedLoaders = new WeakSet<GLTFLoader>();
const fnIds = new WeakMap<NonNullable<UseKhrGlbOptions["extendLoader"]>, number>();
let nextFnId = 1;
const scopedRegistry = new Map<string, Map<string, ScopedLoaderEntry>>();

function configureKhrLoader(loader: GLTFLoader): void {
    if (configuredLoaders.has(loader))
        return;
    configuredLoaders.add(loader);
    loader.register(khrPlugin as (parser: GLTFParser) => GLTFLoaderPlugin);
}

function hasCustomLoaderOpts(options?: UseKhrGlbOptions): options is UseKhrGlbOptions {
    return Boolean(options?.extendLoader || options?.cacheKey !== undefined);
}

function getLoaderCacheKey(options: UseKhrGlbOptions): string {
    if (options.cacheKey !== undefined)
        return `key:${options.cacheKey}`;
    const fn = options.extendLoader!;
    let id = fnIds.get(fn);
    if (id === undefined) {
        id = nextFnId++;
        fnIds.set(fn, id);
    }
    return `fn:${id}`;
}

export function createScopedGltfLoaderClass(): typeof GLTFLoader {
    return class ScopedKhrGLTFLoader extends GLTFLoader {};
}

function getScopedLoaderEntry(url: string, options: UseKhrGlbOptions): ScopedLoaderEntry {
    const key = getLoaderCacheKey(options);
    let urlMap = scopedRegistry.get(url);
    if (!urlMap) {
        urlMap = new Map();
        scopedRegistry.set(url, urlMap);
    }
    let entry = urlMap.get(key);
    if (!entry) {
        const Proto = createScopedGltfLoaderClass();
        const extend = options.extendLoader;
        const created: ScopedLoaderEntry = {
            Proto,
            registerExtensions: (target: GLTFLoader) => {
                configureKhrLoader(target);
                if (extend && !customExtendedLoaders.has(target)) {
                    try {
                        extend(target);
                        customExtendedLoaders.add(target);
                    }
                    catch (err) {
                        const map = scopedRegistry.get(url);
                        if (map?.get(key) === created) {
                            map.delete(key);
                            if (map.size === 0)
                                scopedRegistry.delete(url);
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
    const Proto = scoped ? scoped.Proto : KhrGLTFLoader;
    const reg = scoped ? scoped.registerExtensions : configureKhrLoader;
    const gltf = useLoader(Proto, url, reg);
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
        const Proto = scoped ? scoped.Proto : KhrGLTFLoader;
        const reg = scoped ? scoped.registerExtensions : configureKhrLoader;
        useLoader.preload(Proto, url, reg);
    };

    export const clear = (url: string, options?: UseKhrGlbOptions): void => {
        if (hasCustomLoaderOpts(options)) {
            const urlMap = scopedRegistry.get(url);
            if (!urlMap)
                return;
            const key = getLoaderCacheKey(options);
            const entry = urlMap.get(key);
            if (entry) {
                useLoader.clear(entry.Proto, url);
                urlMap.delete(key);
                if (urlMap.size === 0)
                    scopedRegistry.delete(url);
            }
        }
        else {
            useLoader.clear(KhrGLTFLoader, url);
            if (options === undefined) {
                const urlMap = scopedRegistry.get(url);
                if (urlMap) {
                    for (const entry of urlMap.values())
                        useLoader.clear(entry.Proto, url);
                    scopedRegistry.delete(url);
                }
            }
        }
    };
}
