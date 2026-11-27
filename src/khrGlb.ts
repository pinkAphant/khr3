import { KhrValidationError } from "./khrTypes.js";

export const GLB_MAGIC = 1179937895;
export const GLB_JSON_CHUNK = 1313821514;
export const GLB_BIN_CHUNK = 5130562;

export interface RawGltfJson {
    scene?: number;
    scenes?: Array<{
        nodes?: number[];
        extras?: Record<string, unknown>;
    }>;
    extras?: Record<string, unknown>;
    extensionsUsed?: string[];
    extensionsRequired?: string[];
    extensions?: {
        KHR_implicit_shapes?: {
            shapes?: Array<Record<string, unknown>>;
        };
        KHR_physics_rigid_bodies?: {
            physicsMaterials?: Array<Record<string, unknown>>;
            collisionFilters?: Array<Record<string, unknown>>;
            physicsJoints?: Array<Record<string, unknown>>;
        };
        OMI_vehicle_wheel?: {
            wheels?: Array<Record<string, unknown>>;
            [k: string]: unknown;
        };
        KOLMO_emitter?: Record<string, unknown>;
        KHR_lights_punctual?: {
            lights?: Array<Record<string, unknown>>;
            [k: string]: unknown;
        };
        [ext: string]: unknown;
    };
    nodes?: Array<{
        name?: string;
        children?: number[];
        extras?: Record<string, unknown>;
        extensions?: {
            KHR_physics_rigid_bodies?: Record<string, unknown>;
            OMI_vehicle_wheel?: { wheel?: number } | Record<string, unknown>;
            KOLMO_emitter?: Record<string, unknown>;
            KHR_lights_punctual?: Record<string, unknown>;
            [ext: string]: unknown;
        };
    }>;
}

export interface MutableGltfJson {
    scene?: number;
    scenes?: Array<{
        nodes?: number[];
        extras?: Record<string, unknown>;
    }>;
    extras?: Record<string, unknown>;
    extensionsUsed?: string[];
    extensionsRequired?: string[];
    extensions?: Record<string, unknown>;
    nodes?: Array<{
        name?: string;
        extras?: Record<string, unknown>;
        extensions?: Record<string, unknown>;
    }>;
}

export function validateGlbHeader(buffer: ArrayBuffer): number {
    if (buffer.byteLength < 20)
        throw new KhrValidationError("GLB buffer shorter than 20-byte header");
    const view = new DataView(buffer);
    if (view.getUint32(0, true) !== GLB_MAGIC)
        throw new KhrValidationError("Invalid GLB magic");
    if (view.getUint32(4, true) !== 2)
        throw new KhrValidationError("Unsupported glTF version");
    const jsonLen = view.getUint32(12, true);
    if (view.getUint32(16, true) !== GLB_JSON_CHUNK)
        throw new KhrValidationError("Missing GLB JSON chunk");
    if (20 + jsonLen > buffer.byteLength)
        throw new KhrValidationError("Truncated GLB JSON chunk");
    return jsonLen;
}

export function parseGlbJson(buffer: ArrayBuffer): RawGltfJson {
    const jsonLen = validateGlbHeader(buffer);
    try {
        return JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 20, jsonLen))) as RawGltfJson;
    }
    catch {
        throw new KhrValidationError("Malformed GLB JSON chunk");
    }
}

export function ensureHeadlessFileReader(): void {
    if (globalThis.FileReader !== undefined)
        return;
    class PolyfillFileReader {
        result: string | ArrayBuffer | null = null;
        error: unknown = null;
        onload: ((ev: { target: PolyfillFileReader }) => void) | null = null;
        onloadend: (() => void) | null = null;
        onerror: ((err: unknown) => void) | null = null;

        readAsArrayBuffer(blob: Blob): void {
            blob.arrayBuffer().then(
                (buf) => {
                    this.result = buf;
                    this.onload?.({ target: this });
                    this.onloadend?.();
                },
                (err) => {
                    this.error = err;
                    this.onerror?.(err);
                    this.onloadend?.();
                },
            );
        }

        readAsDataURL(blob: Blob): void {
            blob.arrayBuffer().then(
                (buf) => {
                    const bytes = new Uint8Array(buf);
                    let bin = "";
                    for (let i = 0; i < bytes.length; i++)
                        bin += String.fromCharCode(bytes[i]);
                    this.result = `data:${blob.type || "application/octet-stream"};base64,${btoa(bin)}`;
                    this.onload?.({ target: this });
                    this.onloadend?.();
                },
                (err) => {
                    this.error = err;
                    this.onerror?.(err);
                    this.onloadend?.();
                },
            );
        }
    }
    (globalThis as unknown as { FileReader: unknown }).FileReader = PolyfillFileReader;
}

export function packJsonToGlb(gltf: Record<string, unknown>): ArrayBuffer {
    const jsonBytes = new TextEncoder().encode(JSON.stringify(gltf));
    const paddedJsonLen = (jsonBytes.byteLength + 3) & ~3;
    const totalLen = 20 + paddedJsonLen;
    const out = new ArrayBuffer(totalLen);
    const dv = new DataView(out);
    const u8 = new Uint8Array(out);
    dv.setUint32(0, GLB_MAGIC, true);
    dv.setUint32(4, 2, true);
    dv.setUint32(8, totalLen, true);
    dv.setUint32(12, paddedJsonLen, true);
    dv.setUint32(16, GLB_JSON_CHUNK, true);
    u8.set(jsonBytes, 20);
    u8.fill(0x20, 20 + jsonBytes.byteLength, 20 + paddedJsonLen);
    return out;
}

export function patchGlbJsonChunk(source: ArrayBuffer | Uint8Array, mutator: (gltf: MutableGltfJson) => void): ArrayBuffer {
    const rawBuf = source instanceof ArrayBuffer
        ? source
        : source.buffer.slice(source.byteOffset, source.byteOffset + source.byteLength) as ArrayBuffer;
    const jsonLen = validateGlbHeader(rawBuf);
    const dv = new DataView(rawBuf);
    let gltf: MutableGltfJson;
    try {
        gltf = JSON.parse(new TextDecoder().decode(new Uint8Array(rawBuf, 20, jsonLen))) as MutableGltfJson;
    }
    catch {
        throw new KhrValidationError("Malformed GLB JSON chunk");
    }
    mutator(gltf);
    const nextJson = new TextEncoder().encode(JSON.stringify(gltf));
    const paddedJsonLen = (nextJson.byteLength + 3) & ~3;
    const binHeaderOff = 20 + jsonLen;
    const hasBin = binHeaderOff + 8 <= rawBuf.byteLength;
    const binLen = hasBin ? Math.min(dv.getUint32(binHeaderOff, true), rawBuf.byteLength - (binHeaderOff + 8)) : 0;
    const binTag = hasBin ? dv.getUint32(binHeaderOff + 4, true) : GLB_BIN_CHUNK;
    const binData = hasBin ? new Uint8Array(rawBuf, binHeaderOff + 8, binLen) : new Uint8Array();
    const paddedBinLen = (binData.byteLength + 3) & ~3;
    const totalLen = 20 + paddedJsonLen + (hasBin ? 8 + paddedBinLen : 0);
    const out = new ArrayBuffer(totalLen);
    const outDv = new DataView(out);
    const outU8 = new Uint8Array(out);
    outDv.setUint32(0, GLB_MAGIC, true);
    outDv.setUint32(4, 2, true);
    outDv.setUint32(8, totalLen, true);
    outDv.setUint32(12, paddedJsonLen, true);
    outDv.setUint32(16, GLB_JSON_CHUNK, true);
    outU8.set(nextJson, 20);
    outU8.fill(0x20, 20 + nextJson.byteLength, 20 + paddedJsonLen);
    if (hasBin) {
        outDv.setUint32(20 + paddedJsonLen, paddedBinLen, true);
        outDv.setUint32(24 + paddedJsonLen, binTag, true);
        outU8.set(binData, 28 + paddedJsonLen);
    }
    return out;
}
