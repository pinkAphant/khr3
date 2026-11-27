import { KhrValidationError } from "./types.js";

/** ASCII "glTF" little-endian fourCC. */
export const GLB_MAGIC = 0x46546c67;
/** ASCII "JSON" little-endian fourCC. */
export const GLB_JSON_CHUNK = 0x4e4f534a;
/** ASCII "BIN\0" little-endian fourCC. */
export const GLB_BIN_CHUNK = 0x004e4942;

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

export function validateGlbHeader(buffer: ArrayBuffer, byteOffset = 0, byteLength = buffer.byteLength - byteOffset): number {
    if (byteLength < 20)
        throw new KhrValidationError("GLB buffer shorter than 20-byte header");
    const view = new DataView(buffer, byteOffset, byteLength);
    if (view.getUint32(0, true) !== GLB_MAGIC)
        throw new KhrValidationError("Invalid GLB magic");
    if (view.getUint32(4, true) !== 2)
        throw new KhrValidationError("Unsupported glTF version");
    const jsonLen = view.getUint32(12, true);
    if (view.getUint32(16, true) !== GLB_JSON_CHUNK)
        throw new KhrValidationError("Missing GLB JSON chunk");
    if (20 + jsonLen > byteLength)
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

function bytesToBase64(bytes: Uint8Array): string {
    if (typeof Buffer !== "undefined")
        return Buffer.from(bytes.buffer, bytes.byteOffset, bytes.byteLength).toString("base64");
    const CHUNK = 0x8000;
    const chunks: string[] = [];
    for (let i = 0; i < bytes.length; i += CHUNK) {
        chunks.push(String.fromCharCode(...bytes.subarray(i, i + CHUNK)));
    }
    return btoa(chunks.join(""));
}

export function ensureHeadlessFileReader(): void {
    if (globalThis.FileReader !== undefined)
        return;
    class HeadlessFileReader extends EventTarget {
        static readonly EMPTY = 0;
        static readonly LOADING = 1;
        static readonly DONE = 2;
        readonly EMPTY = 0;
        readonly LOADING = 1;
        readonly DONE = 2;
        readyState = 0;
        result: string | ArrayBuffer | null = null;
        error: unknown = null;
        onload: ((ev: { target: HeadlessFileReader }) => void) | null = null;
        onloadend: ((ev: { target: HeadlessFileReader }) => void) | null = null;
        onerror: ((err: unknown) => void) | null = null;

        abort(): void {
            this.readyState = 2;
        }

        readAsArrayBuffer(blob: Blob): void {
            this.readyState = 1;
            blob.arrayBuffer().then(
                (buf) => {
                    this.result = buf;
                    this.readyState = 2;
                    this.onload?.({ target: this });
                    this.dispatchEvent(new Event("load"));
                    this.onloadend?.({ target: this });
                    this.dispatchEvent(new Event("loadend"));
                },
                (err) => {
                    this.error = err;
                    this.readyState = 2;
                    this.onerror?.(err);
                    this.dispatchEvent(new Event("error"));
                    this.onloadend?.({ target: this });
                    this.dispatchEvent(new Event("loadend"));
                },
            );
        }

        readAsDataURL(blob: Blob): void {
            this.readyState = 1;
            blob.arrayBuffer().then(
                (buf) => {
                    const b64 = bytesToBase64(new Uint8Array(buf));
                    this.result = `data:${blob.type || "application/octet-stream"};base64,${b64}`;
                    this.readyState = 2;
                    this.onload?.({ target: this });
                    this.dispatchEvent(new Event("load"));
                    this.onloadend?.({ target: this });
                    this.dispatchEvent(new Event("loadend"));
                },
                (err) => {
                    this.error = err;
                    this.readyState = 2;
                    this.onerror?.(err);
                    this.dispatchEvent(new Event("error"));
                    this.onloadend?.({ target: this });
                    this.dispatchEvent(new Event("loadend"));
                },
            );
        }
    }
    (globalThis as unknown as { FileReader: unknown }).FileReader = HeadlessFileReader;
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
    const buf = source instanceof ArrayBuffer ? source : source.buffer;
    const byteOffset = source instanceof ArrayBuffer ? 0 : source.byteOffset;
    const byteLength = source.byteLength;
    const jsonLen = validateGlbHeader(buf as ArrayBuffer, byteOffset, byteLength);
    const dv = new DataView(buf, byteOffset, byteLength);
    let gltf: MutableGltfJson;
    try {
        gltf = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, byteOffset + 20, jsonLen))) as MutableGltfJson;
    }
    catch {
        throw new KhrValidationError("Malformed GLB JSON chunk");
    }
    mutator(gltf);
    const nextJson = new TextEncoder().encode(JSON.stringify(gltf));
    const paddedJsonLen = (nextJson.byteLength + 3) & ~3;
    const binHeaderOff = 20 + jsonLen;
    const hasBin = binHeaderOff < byteLength;
    let binTag = GLB_BIN_CHUNK;
    let binData: Uint8Array = new Uint8Array(0);
    if (hasBin) {
        if (binHeaderOff + 8 > byteLength)
            throw new KhrValidationError("Truncated GLB binary chunk header");
        const binLen = dv.getUint32(binHeaderOff, true);
        binTag = dv.getUint32(binHeaderOff + 4, true);
        if (binHeaderOff + 8 + binLen > byteLength)
            throw new KhrValidationError("Truncated GLB binary chunk payload");
        binData = new Uint8Array(buf, byteOffset + binHeaderOff + 8, binLen);
    }
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
