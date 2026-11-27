import * as THREE from "three";
import { type KhrAssetBundle, type KhrBreakawaySpec, type KhrPluginOptions, KhrValidationError } from "./khrTypes.js";

export { decodePropellerTag, decodeWheelTag } from "./khrRotorDecoders.js";

export function decodeBreakawaySpec(raw: unknown): KhrBreakawaySpec | undefined {
    if (!raw || typeof raw !== "object" || Array.isArray(raw))
        return undefined;
    const rec = raw as Record<string, unknown>;
    if (typeof rec.breakImpulseNs !== "number" || !Number.isFinite(rec.breakImpulseNs) || rec.breakImpulseNs <= 0)
        return undefined;
    const latch = typeof rec.latchImpulseNs === "number" && Number.isFinite(rec.latchImpulseNs) && rec.latchImpulseNs > 0
        ? rec.latchImpulseNs
        : undefined;
    return { ...(latch === undefined ? {} : { latchImpulseNs: latch }), breakImpulseNs: rec.breakImpulseNs };
}

export interface KhrNodeMeta {
    nodeName: string;
    extras: Record<string, unknown>;
    worldPosition: [number, number, number];
    contactDots?: Array<[string, [number, number, number]]>;
}

export interface KhrTemplateMeta {
    nodes: KhrNodeMeta[];
    bodies: Array<{
        nodeName: string;
        extras: Record<string, unknown>;
    }>;
    ikSockets: Array<[string, [number, number, number]]>;
}

const templateMetaMap = new WeakMap<THREE.Object3D, KhrTemplateMeta>();

export function registerKhrTemplateMeta(scene: THREE.Object3D, meta: KhrTemplateMeta): void {
    templateMetaMap.set(scene, meta);
}

export function setSocket(
    sockets: Record<string, [number, number, number]>,
    key: string,
    pos: [number, number, number],
): void {
    Object.defineProperty(sockets, key, {
        value: pos,
        enumerable: true,
        configurable: true,
        writable: true,
    });
}

export function applyKhrCustomExtractors(
    clonedBundle: KhrAssetBundle,
    templateBundle: KhrAssetBundle,
    extractRole?: KhrPluginOptions["extractRole"],
    isSocketNode?: KhrPluginOptions["isSocketNode"],
): void {
    const meta = templateMetaMap.get(templateBundle.scene);
    if (!meta)
        return;
    if (extractRole) {
        for (let i = 0; i < clonedBundle.bodies.length; i++) {
            const rawBody = meta.bodies[i];
            const dstBody = clonedBundle.bodies[i];
            if (!rawBody || !dstBody)
                continue;
            const role = extractRole(rawBody.nodeName, rawBody.extras)
                ?? (typeof rawBody.extras.role === "string" ? rawBody.extras.role : null);
            dstBody.role = role;
            if (dstBody.joint)
                dstBody.joint.role = role;
        }
    }
    if (isSocketNode) {
        const sockets: Record<string, [number, number, number]> = {};
        for (const node of meta.nodes) {
            const custom = isSocketNode(node.nodeName, node.extras);
            if (custom || node.extras.isIkAnchor === true || typeof node.extras.socketId === "string") {
                const key = typeof custom === "string" && custom.length > 0
                    ? custom
                    : (typeof node.extras.socketId === "string" ? node.extras.socketId : node.nodeName);
                setSocket(sockets, key, [...node.worldPosition]);
            }
            if (node.contactDots) {
                for (const [role, pos] of node.contactDots)
                    setSocket(sockets, role, [...pos]);
            }
        }
        for (const [key, pos] of meta.ikSockets)
            setSocket(sockets, key, [...pos]);
        clonedBundle.sockets = sockets;
    }
}

const _dotVec = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const isObj = (v: unknown): v is Record<string, unknown> => typeof v === "object" && !!v && !Array.isArray(v);
const isFiniteNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);

export function collectNodeSockets(
    obj: THREE.Object3D,
    name: string,
    extras: Record<string, unknown>,
    sockets: Record<string, [number, number, number]>,
    isSocketNode?: KhrPluginOptions["isSocketNode"],
): KhrNodeMeta {
    if (extras.socketId !== undefined && (typeof extras.socketId !== "string" || extras.socketId.length === 0)) {
        throw new KhrValidationError(`Node "${name}" has invalid extras.socketId`);
    }
    const wp = obj.getWorldPosition(new THREE.Vector3());
    const worldPosition: [number, number, number] = [wp.x, wp.y, wp.z];
    const custom = isSocketNode?.(name, extras);
    if (custom || extras.isIkAnchor === true || typeof extras.socketId === "string") {
        const key = typeof custom === "string" && custom.length > 0
            ? custom
            : (typeof extras.socketId === "string" ? extras.socketId : name);
        setSocket(sockets, key, [...worldPosition]);
    }
    let contactDots: Array<[string, [number, number, number]]> | undefined;
    if (extras.contactDots !== undefined) {
        if (!Array.isArray(extras.contactDots)) {
            throw new KhrValidationError(`Node "${name}" has invalid extras.contactDots (expected array)`);
        }
        obj.getWorldQuaternion(_quat);
        contactDots = [];
        for (const dot of extras.contactDots) {
            if (
                !isObj(dot) ||
                typeof dot.role !== "string" ||
                dot.role.length === 0 ||
                !Array.isArray(dot.pos) ||
                dot.pos.length !== 3 ||
                !dot.pos.every(isFiniteNum)
            ) {
                throw new KhrValidationError(`Node "${name}" has invalid entry in extras.contactDots`);
            }
            _dotVec.set(dot.pos[0], dot.pos[1], dot.pos[2]).applyQuaternion(_quat).add(wp);
            const dotPos: [number, number, number] = [_dotVec.x, _dotVec.y, _dotVec.z];
            setSocket(sockets, dot.role, dotPos);
            contactDots.push([dot.role, dotPos]);
        }
    }
    return {
        nodeName: name,
        extras,
        worldPosition,
        ...(contactDots ? { contactDots } : {}),
    };
}
