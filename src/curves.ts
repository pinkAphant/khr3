import {
    compilePacejkaCurve,
    engineHorsepowerForPeakTorque,
    tireGripForSurface,
    type CurveData,
    type CurvePoint,
} from "./pacejka.js";

export type { CurveData, CurvePoint };
export { compilePacejkaCurve, engineHorsepowerForPeakTorque, tireGripForSurface };

export interface TireArchetypePreset {
    lng: CurveData;
    lat: CurveData;
    muPeakLng: number;
    latFrictionEllipseScale: number;
    relaxLngRate: number;
    relaxLatRate: number;
}

export const TIRE_ARCHETYPES = {
    hypercarDry: {
        lng: compilePacejkaCurve(11.2, 1.54, -0.45, 100, "lng"),
        lat: compilePacejkaCurve(11.6, 1.5, -0.4, 100, "lat"),
        muPeakLng: 1.38 as const,
        latFrictionEllipseScale: 0.97 as const,
        relaxLngRate: 0.04 as const,
        relaxLatRate: 0.06 as const,
    },
    streetDrift90s: {
        lng: compilePacejkaCurve(8.8, 1.5, -0.15, 80, "lng"),
        lat: compilePacejkaCurve(9.2, 1.44, -0.12, 80, "lat"),
        muPeakLng: 1.08 as const,
        latFrictionEllipseScale: 0.97 as const,
        relaxLngRate: 0.05 as const,
        relaxLatRate: 0.1 as const,
    },
    offroad6x6At: {
        lng: compilePacejkaCurve(6.2, 1.48, -0.35, 80, "lng"),
        lat: compilePacejkaCurve(6.6, 1.48, -0.3, 80, "lat"),
        muPeakLng: 0.85 as const,
        latFrictionEllipseScale: 0.87 as const,
        relaxLngRate: 0.12 as const,
        relaxLatRate: 0.24 as const,
    },
    gravelRally: {
        lng: compilePacejkaCurve(5.8, 1.26, -0.45, 80, "lng"),
        lat: compilePacejkaCurve(6.4, 1.28, -0.4, 80, "lat"),
        muPeakLng: 0.74 as const,
        latFrictionEllipseScale: 0.92 as const,
        relaxLngRate: 0.08 as const,
        relaxLatRate: 0.16 as const,
    },
    wetAsphalt: {
        lng: compilePacejkaCurve(12.8, 1.66, -0.6, 100, "lng"),
        lat: compilePacejkaCurve(12.2, 1.62, -0.55, 100, "lat"),
        muPeakLng: 0.8 as const,
        latFrictionEllipseScale: 0.95 as const,
        relaxLngRate: 0.05 as const,
        relaxLatRate: 0.09 as const,
    },
    snowIce: {
        lng: compilePacejkaCurve(17.5, 1.64, -1.15, 120, "lng"),
        lat: compilePacejkaCurve(16.5, 1.61, -1.05, 120, "lat"),
        muPeakLng: 0.28 as const,
        latFrictionEllipseScale: 0.93 as const,
        relaxLngRate: 0.06 as const,
        relaxLatRate: 0.12 as const,
    },
} as const;

export const TIRE_SLIP_CURVES: {
    readonly streetRadial: CurveData;
    readonly driftProgressive: CurveData;
    readonly offroadLug: CurveData;
} = {
    streetRadial: TIRE_ARCHETYPES.streetDrift90s.lng,
    driftProgressive: TIRE_ARCHETYPES.streetDrift90s.lat,
    offroadLug: TIRE_ARCHETYPES.offroad6x6At.lng,
};

export const ENGINE_TORQUE_CURVES = {
    w16QuadTurbo: {
        samples: 80 as const,
        points: [
            { x: 0 as const, y: 0.62 as const, r_out: 0.85 as const, w_out: 1 as const },
            { x: 0.15 as const, y: 0.84 as const, r_in: 1.05 as const, r_out: 1.05 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.27 as const, y: 1 as const, r_in: 0 as const, r_out: 0 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.84 as const, y: 1 as const, r_in: 0 as const, r_out: 0 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.95 as const, y: 0.962 as const, r_in: -0.68 as const, r_out: -0.68 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 1 as const, y: 0.82 as const, r_in: -1.32 as const, w_in: 1 as const },
        ] as [
            { readonly x: 0; readonly y: 0.62; readonly r_out: 0.85; readonly w_out: 1 },
            { readonly x: 0.15; readonly y: 0.84; readonly r_in: 1.05; readonly r_out: 1.05; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.27; readonly y: 1; readonly r_in: 0; readonly r_out: 0; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.84; readonly y: 1; readonly r_in: 0; readonly r_out: 0; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.95; readonly y: 0.962; readonly r_in: -0.68; readonly r_out: -0.68; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 1; readonly y: 0.82; readonly r_in: -1.32; readonly w_in: 1 }
        ],
    },
    naScreamer: {
        samples: 80 as const,
        points: [
            { x: 0 as const, y: 0.56 as const, r_out: 0.58 as const, w_out: 1 as const },
            { x: 0.25 as const, y: 0.74 as const, r_in: 0.55 as const, r_out: 0.55 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.45 as const, y: 0.84 as const, r_in: 0.62 as const, r_out: 0.62 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.68 as const, y: 1 as const, r_in: 0 as const, r_out: 0 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.93 as const, y: 0.915 as const, r_in: -0.66 as const, r_out: -0.66 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 1 as const, y: 0.8 as const, r_in: -1.12 as const, w_in: 1 as const },
        ] as [
            { readonly x: 0; readonly y: 0.56; readonly r_out: 0.58; readonly w_out: 1 },
            { readonly x: 0.25; readonly y: 0.74; readonly r_in: 0.55; readonly r_out: 0.55; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.45; readonly y: 0.84; readonly r_in: 0.62; readonly r_out: 0.62; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.68; readonly y: 1; readonly r_in: 0; readonly r_out: 0; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.93; readonly y: 0.915; readonly r_in: -0.66; readonly r_out: -0.66; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 1; readonly y: 0.8; readonly r_in: -1.12; readonly w_in: 1 }
        ],
    },
    heavyTurboDiesel: {
        samples: 80 as const,
        points: [
            { x: 0 as const, y: 0.64 as const, r_out: 1.05 as const, w_out: 1 as const },
            { x: 0.14 as const, y: 0.88 as const, r_in: 0.98 as const, r_out: 0.98 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.28 as const, y: 1 as const, r_in: 0 as const, r_out: 0 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.44 as const, y: 0.975 as const, r_in: -0.32 as const, r_out: -0.32 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.72 as const, y: 0.73 as const, r_in: -0.92 as const, r_out: -0.92 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 1 as const, y: 0.28 as const, r_in: -1.08 as const, w_in: 1 as const },
        ] as [
            { readonly x: 0; readonly y: 0.64; readonly r_out: 1.05; readonly w_out: 1 },
            { readonly x: 0.14; readonly y: 0.88; readonly r_in: 0.98; readonly r_out: 0.98; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.28; readonly y: 1; readonly r_in: 0; readonly r_out: 0; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.44; readonly y: 0.975; readonly r_in: -0.32; readonly r_out: -0.32; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.72; readonly y: 0.73; readonly r_in: -0.92; readonly r_out: -0.92; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 1; readonly y: 0.28; readonly r_in: -1.08; readonly w_in: 1 }
        ],
    },
    evDirectDrive: {
        samples: 80 as const,
        points: [
            { x: 0 as const, y: 1 as const, r_out: 0 as const, w_out: 1 as const },
            { x: 0.32 as const, y: 1 as const, r_in: 0 as const, r_out: -1.2611 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.44 as const, y: 0.7273 as const, r_in: -1.0267 as const, r_out: -1.0267 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.6 as const, y: 0.5333 as const, r_in: -0.7266 as const, r_out: -0.7266 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.78 as const, y: 0.4103 as const, r_in: -0.4842 as const, r_out: -0.4842 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 1 as const, y: 0.32 as const, r_in: -0.3097 as const, w_in: 1 as const },
        ] as [
            { readonly x: 0; readonly y: 1; readonly r_out: 0; readonly w_out: 1 },
            { readonly x: 0.32; readonly y: 1; readonly r_in: 0; readonly r_out: -1.2611; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.44; readonly y: 0.7273; readonly r_in: -1.0267; readonly r_out: -1.0267; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.6; readonly y: 0.5333; readonly r_in: -0.7266; readonly r_out: -0.7266; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.78; readonly y: 0.4103; readonly r_in: -0.4842; readonly r_out: -0.4842; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 1; readonly y: 0.32; readonly r_in: -0.3097; readonly w_in: 1 }
        ],
    },
} as const;

export const STEER_SPEED_CURVES = {
    progressiveHighway: {
        samples: 80 as const,
        points: [
            { x: 0 as const, y: 1 as const, r_out: 0 as const, w_out: 1 as const },
            { x: 0.2 as const, y: 1 as const, r_in: 0 as const, r_out: 0 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.5 as const, y: 0.62 as const, r_in: -0.82 as const, r_out: -0.82 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 1 as const, y: 0.3 as const, r_in: -0.35 as const, w_in: 1 as const },
        ] as [
            { readonly x: 0; readonly y: 1; readonly r_out: 0; readonly w_out: 1 },
            { readonly x: 0.2; readonly y: 1; readonly r_in: 0; readonly r_out: 0; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.5; readonly y: 0.62; readonly r_in: -0.82; readonly r_out: -0.82; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 1; readonly y: 0.3; readonly r_in: -0.35; readonly w_in: 1 }
        ],
    },
    driftCountersteer: {
        samples: 80 as const,
        points: [
            { x: 0 as const, y: 1 as const, r_out: 0 as const, w_out: 1 as const },
            { x: 0.14 as const, y: 1 as const, r_in: 0 as const, r_out: 0 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.3 as const, y: 0.72 as const, r_in: -1.02 as const, r_out: -1.02 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.5 as const, y: 0.48 as const, r_in: -0.74 as const, r_out: -0.74 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.75 as const, y: 0.32 as const, r_in: -0.44 as const, r_out: -0.44 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 1 as const, y: 0.24 as const, r_in: -0.2 as const, w_in: 1 as const },
        ] as [
            { readonly x: 0; readonly y: 1; readonly r_out: 0; readonly w_out: 1 },
            { readonly x: 0.14; readonly y: 1; readonly r_in: 0; readonly r_out: 0; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.3; readonly y: 0.72; readonly r_in: -1.02; readonly r_out: -1.02; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.5; readonly y: 0.48; readonly r_in: -0.74; readonly r_out: -0.74; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.75; readonly y: 0.32; readonly r_in: -0.44; readonly r_out: -0.44; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 1; readonly y: 0.24; readonly r_in: -0.2; readonly w_in: 1 }
        ],
    },
    evSingleGearGt: {
        samples: 80 as const,
        points: [
            { x: 0 as const, y: 1 as const, r_out: 0 as const, w_out: 1 as const },
            { x: 0.08 as const, y: 1 as const, r_in: 0 as const, r_out: 0 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.2 as const, y: 0.28 as const, r_in: -1.1 as const, r_out: -1.1 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.5 as const, y: 0.11 as const, r_in: -0.25 as const, r_out: -0.25 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 1 as const, y: 0.07 as const, r_in: 0 as const, w_in: 1 as const },
        ] as [
            { readonly x: 0; readonly y: 1; readonly r_out: 0; readonly w_out: 1 },
            { readonly x: 0.08; readonly y: 1; readonly r_in: 0; readonly r_out: 0; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.2; readonly y: 0.28; readonly r_in: -1.1; readonly r_out: -1.1; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.5; readonly y: 0.11; readonly r_in: -0.25; readonly r_out: -0.25; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 1; readonly y: 0.07; readonly r_in: 0; readonly w_in: 1 }
        ],
    },
    heavy6x6AntiRollover: {
        samples: 80 as const,
        points: [
            { x: 0 as const, y: 1 as const, r_out: 0 as const, w_out: 1 as const },
            { x: 0.1 as const, y: 0.94 as const, r_in: -0.45 as const, r_out: -0.95 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.28 as const, y: 0.42 as const, r_in: -1.12 as const, r_out: -0.85 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 0.55 as const, y: 0.19 as const, r_in: -0.42 as const, r_out: -0.22 as const, w_in: 1 as const, w_out: 1 as const },
            { x: 1 as const, y: 0.11 as const, r_in: -0.1 as const, w_in: 1 as const },
        ] as [
            { readonly x: 0; readonly y: 1; readonly r_out: 0; readonly w_out: 1 },
            { readonly x: 0.1; readonly y: 0.94; readonly r_in: -0.45; readonly r_out: -0.95; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.28; readonly y: 0.42; readonly r_in: -1.12; readonly r_out: -0.85; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 0.55; readonly y: 0.19; readonly r_in: -0.42; readonly r_out: -0.22; readonly w_in: 1; readonly w_out: 1 },
            { readonly x: 1; readonly y: 0.11; readonly r_in: -0.1; readonly w_in: 1 }
        ],
    },
} as const;
