export type CurvePoint = {
    x: number;
    y: number;
    r_in?: number;
    r_out?: number;
    w_in?: number;
    w_out?: number;
};

export type CurveData = {
    points: CurvePoint[];
    samples?: number;
};

const HALF_PI = Math.PI * 0.5;
const MIN_KNOT_SPACING = 0.01;
const MAX_TANGENT_SECANT_RATIO = 2.85;
const MAX_RAD = 1.54;

function clamp(v: number, lo: number, hi: number): number {
    return v < lo ? lo : v > hi ? hi : v;
}

function uVal(x: number, axis: "lng" | "lat"): number {
    return axis === "lat" ? Math.sin(HALF_PI * x) : x;
}

function duVal(x: number, axis: "lng" | "lat"): number {
    return axis === "lat" ? HALF_PI * Math.cos(HALF_PI * x) : 1;
}

function pacejkaPhi(u: number, B: number, E: number): number {
    const bu = B * u;
    return bu - E * (bu - Math.atan(bu));
}

function pacejkaYRaw(u: number, B: number, C: number, E: number): number {
    return Math.sin(C * Math.atan(pacejkaPhi(u, B, E)));
}

function pacejkaSlopeX(x: number, B: number, C: number, E: number, axis: "lng" | "lat"): number {
    const u = uVal(x, axis);
    const bu = B * u;
    const phi = pacejkaPhi(u, B, E);
    const dphiDu = B * ((1 - E) + E / (1 + bu * bu));
    const dyDphi = (C * Math.cos(C * Math.atan(phi))) / (1 + phi * phi);
    return dyDphi * dphiDu * duVal(x, axis);
}

/**
 * Exact 8-knot Fritsch-Carlson monotone Pacejka Magic Formula compiler for `ecctrl/curves`.
 * - `"lng"`: `u = x` matching `ShapeCastWheel` `lngSlipRatio = |w*R - vx| / |vx|`.
 * - `"lat"`: `u = sin((pi/2) * x)` matching `ShapeCastWheel` `latSlipRatio = atan2(|vy|, |vx|) / (pi/2)`.
 */
export function compilePacejkaCurve(
    B: number,
    C: number,
    E: number,
    samples = 100,
    axis: "lng" | "lat" = "lng",
): CurveData {
    let xPeak = 1;
    if (C > 1) {
        const targetPhi = Math.tan(Math.PI / (2 * C));
        if (pacejkaPhi(1, B, E) >= targetPhi) {
            let lo = 0;
            let hi = 1;
            for (let iter = 0; iter < 48; iter++) {
                const mid = 0.5 * (lo + hi);
                if (pacejkaPhi(uVal(mid, axis), B, E) >= targetPhi)
                    hi = mid;
                else
                    lo = mid;
            }
            xPeak = hi;
        }
    }

    const yPeakRaw = pacejkaYRaw(uVal(xPeak, axis), B, C, E);
    const invYMax = yPeakRaw > 1e-6 ? 1 / yPeakRaw : 1;

    const rawKnots = [
        0,
        0.32 * xPeak,
        0.68 * xPeak,
        xPeak,
        xPeak + 0.06 * (1 - xPeak),
        xPeak + 0.18 * (1 - xPeak),
        xPeak + 0.48 * (1 - xPeak),
        1,
    ];

    const knots: number[] = [0];
    for (let i = 1; i < rawKnots.length - 1; i++) {
        const k = rawKnots[i];
        if (k - knots[knots.length - 1] >= MIN_KNOT_SPACING && 1 - k >= MIN_KNOT_SPACING)
            knots.push(k);
    }
    knots.push(1);

    const ys = knots.map((x) => {
        if (x === xPeak && C > 1 && yPeakRaw > 1e-6 && pacejkaPhi(1, B, E) >= Math.tan(Math.PI / (2 * C)))
            return 1;
        return pacejkaYRaw(uVal(x, axis), B, C, E) * invYMax;
    });

    const secants: number[] = [];
    for (let i = 0; i < knots.length - 1; i++) {
        secants.push((ys[i + 1] - ys[i]) / (knots[i + 1] - knots[i]));
    }

    const slopes = knots.map((x, i) => {
        if (x === xPeak && x < 1)
            return 0;
        const mRaw = pacejkaSlopeX(x, B, C, E, axis) * invYMax;
        if (i === 0) {
            const s = secants[0];
            if (Math.abs(s) <= 1e-9)
                return 0;
            return clamp(mRaw / s, 0, MAX_TANGENT_SECANT_RATIO) * s;
        }
        if (i === knots.length - 1) {
            const s = secants[secants.length - 1];
            if (Math.abs(s) <= 1e-9)
                return 0;
            return clamp(mRaw / s, 0, MAX_TANGENT_SECANT_RATIO) * s;
        }
        const sL = secants[i - 1];
        const sR = secants[i];
        if (sL * sR <= 0 || Math.abs(sL) <= 1e-9 || Math.abs(sR) <= 1e-9)
            return 0;
        const minS = Math.abs(sL) <= Math.abs(sR) ? sL : sR;
        return clamp(mRaw / minS, 0, MAX_TANGENT_SECANT_RATIO) * minS;
    });

    const points: CurvePoint[] = knots.map((x, i) => {
        const ang = clamp(Math.atan(slopes[i]), -MAX_RAD, MAX_RAD);
        const pt: CurvePoint = {
            x: Number(x.toFixed(6)),
            y: Number(ys[i].toFixed(6)),
        };
        if (i > 0) {
            pt.r_in = Number(ang.toFixed(6));
            pt.w_in = 1;
        }
        if (i < knots.length - 1) {
            pt.r_out = Number(ang.toFixed(6));
            pt.w_out = 1;
        }
        return pt;
    });

    return { points, samples };
}

/** Inverts `ShapeCastWheel` `(rayHitFriction + tireGripFactor) * 0.5 = muPeak`. */
export function tireGripForSurface(muPeakLng: number, groundColliderFriction = 0.5): number {
    return Number((2 * muPeakLng - groundColliderFriction).toFixed(4));
}

/** Converts physical peak engine torque (N*m) into `carConfig.engineHorsepower` (`T * RPM / 7022`). */
export function engineHorsepowerForPeakTorque(peakTorqueNm: number, engineMaxRPM: number): number {
    return Number(((peakTorqueNm * engineMaxRPM) / 7022).toFixed(1));
}
