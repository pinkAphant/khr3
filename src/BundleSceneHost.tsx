import React, { useLayoutEffect, useRef } from "react";
import * as THREE from "three";
import { claimBundleMount, registerMountedSubtree, releaseBundleMount } from "./khrClone.js";
import type { KhrAssetBundle } from "./khrTypes.js";

export interface SubtreeOrigin {
    origParent: THREE.Object3D | null;
    origIndex: number;
    origOrder: Map<THREE.Object3D, number>;
    origPos: THREE.Vector3;
    origQuat: THREE.Quaternion;
    origScale: THREE.Vector3;
}

export function captureSubtreeOrigins(objects: readonly THREE.Object3D[]): Map<THREE.Object3D, SubtreeOrigin> {
    const parentOrders = new Map<THREE.Object3D, Map<THREE.Object3D, number>>();
    const origins = new Map<THREE.Object3D, SubtreeOrigin>();
    for (const obj of objects) {
        const origParent = obj.parent;
        let origOrder = new Map<THREE.Object3D, number>();
        let origIndex = -1;
        if (origParent) {
            let order = parentOrders.get(origParent);
            if (!order) {
                order = new Map<THREE.Object3D, number>();
                origParent.children.forEach((child, idx) => order!.set(child, idx));
                parentOrders.set(origParent, order);
            }
            origOrder = order;
            origIndex = order.get(obj) ?? origParent.children.indexOf(obj);
        }
        origins.set(obj, {
            origParent,
            origIndex,
            origOrder,
            origPos: obj.position.clone(),
            origQuat: obj.quaternion.clone(),
            origScale: obj.scale.clone(),
        });
    }
    return origins;
}

export function restoreSubtreeOrigin(object: THREE.Object3D, snap: SubtreeOrigin): void {
    object.position.copy(snap.origPos);
    object.quaternion.copy(snap.origQuat);
    object.scale.copy(snap.origScale);
    const { origParent, origIndex, origOrder } = snap;
    if (origParent) {
        origParent.add(object);
        const curIdx = origParent.children.indexOf(object);
        if (curIdx >= 0)
            origParent.children.splice(curIdx, 1);
        let insertAt = origParent.children.length;
        if (origOrder.size > 0) {
            for (let i = 0; i < origParent.children.length; i++) {
                if ((origOrder.get(origParent.children[i]) ?? Infinity) > origIndex) {
                    insertAt = i;
                    break;
                }
            }
        }
        else if (origIndex >= 0) {
            insertAt = Math.min(origIndex, origParent.children.length);
        }
        origParent.children.splice(insertAt, 0, object);
    }
}

export function BundleSceneHost({ bundle, owner }: { bundle: KhrAssetBundle; owner: string }): React.JSX.Element {
    const hostRef = useRef<THREE.Group | null>(null);
    useLayoutEffect(() => {
        const host = hostRef.current;
        if (!host)
            return;
        claimBundleMount(bundle, owner);
        host.add(bundle.scene);
        return () => {
            if (bundle.scene.parent === host)
                host.remove(bundle.scene);
            releaseBundleMount(bundle);
        };
    }, [bundle, owner]);
    return <group ref={hostRef} />;
}

export function MountSubtreeSlot({
    object,
    position,
    quaternion,
    origin,
}: {
    object: THREE.Object3D;
    position: [number, number, number];
    quaternion: [number, number, number, number];
    origin?: SubtreeOrigin;
}): React.JSX.Element {
    const slotRef = useRef<THREE.Group | null>(null);
    useLayoutEffect(() => {
        const slot = slotRef.current;
        if (!slot)
            return;
        const snap = origin ?? captureSubtreeOrigins([object]).get(object)!;
        if (snap.origParent)
            snap.origParent.remove(object);
        object.position.set(0, 0, 0);
        object.quaternion.set(0, 0, 0, 1);
        object.scale.set(1, 1, 1);
        slot.add(object);
        const untrack = registerMountedSubtree(() => {
            const p = object.position.clone(), q = object.quaternion.clone(), s = object.scale.clone();
            if (object.parent === slot)
                slot.remove(object);
            restoreSubtreeOrigin(object, snap);
            return () => {
                if (snap.origParent)
                    snap.origParent.remove(object);
                object.position.copy(p);
                object.quaternion.copy(q);
                object.scale.copy(s);
                slot.add(object);
            };
        });
        return () => {
            untrack();
            if (object.parent === slot)
                slot.remove(object);
            restoreSubtreeOrigin(object, snap);
        };
    }, [object, origin]);
    return <group ref={slotRef} position={position} quaternion={quaternion} />;
}
