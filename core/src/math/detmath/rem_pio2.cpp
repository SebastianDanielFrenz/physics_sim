// core/src/math/detmath/rem_pio2.cpp
// Ported from musl libm v1.2.5:
//   src/math/__rem_pio2.c       (origin: FreeBSD e_rem_pio2.c)
//   src/math/__rem_pio2_large.c (origin: FreeBSD k_rem_pio2.c)
// See common.h for the porting rules and THIRD_PARTY_NOTICES.md for the
// full musl licence text.
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunSoft, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 *
 * Optimized by Bruce D. Evans.
 */

#include "rem_pio2.h"
#include "common.h"

namespace ps::math::detmath {
namespace {

// ---- __rem_pio2_large (k_rem_pio2.c) ----
//
// Table of the (24*i)-th to (24*i+23)-th bit of 2/pi after the binary
// point: ipio2[i]*2^(-24(i+1)). musl ships a longer table for platforms
// where long double has a wider exponent range than double; this port only
// needs the first 66 entries (the portion musl itself uses on double-only
// targets), which comfortably covers every finite double exponent (double
// needs entries up to about index jv+jk ~= 46).
const int32_t ipio2[] = {
    0xA2F983, 0x6E4E44, 0x1529FC, 0x2757D1, 0xF534DD, 0xC0DB62,
    0x95993C, 0x439041, 0xFE5163, 0xABDEBB, 0xC561B7, 0x246E3A,
    0x424DD2, 0xE00649, 0x2EEA09, 0xD1921C, 0xFE1DEB, 0x1CB129,
    0xA73EE8, 0x8235F5, 0x2EBB44, 0x84E99C, 0x7026B4, 0x5F7E41,
    0x3991D6, 0x398353, 0x39F49C, 0x845F8B, 0xBDF928, 0x3B1FF8,
    0x97FFDE, 0x05980F, 0xEF2F11, 0x8B5A0A, 0x6D1F6D, 0x367ECF,
    0x27CB09, 0xB74F46, 0x3F669E, 0x5FEA2D, 0x7527BA, 0xC7EBE5,
    0xF17B3D, 0x0739F7, 0x8A5292, 0xEA6BFB, 0x5FB11F, 0x8D5D08,
    0x560330, 0x46FC7B, 0x6BABF0, 0xCFBC20, 0x9AF436, 0x1DA9E3,
    0x91615E, 0xE61B08, 0x659985, 0x5F14A0, 0x68408D, 0xFFD880,
    0x4D7327, 0x310606, 0x1556CA, 0x73A8C9, 0x60E27B, 0xC08C6B,
};

const double PIo2[] = {
    1.57079625129699707031e+00, /* 0x3FF921FB, 0x40000000 */
    7.54978941586159635335e-08, /* 0x3E74442D, 0x00000000 */
    5.39030252995776476554e-15, /* 0x3CF84698, 0x80000000 */
    3.28200341580791294123e-22, /* 0x3B78CC51, 0x60000000 */
    1.27065575308067607349e-29, /* 0x39F01B83, 0x80000000 */
    1.22933308981111328932e-36, /* 0x387A2520, 0x40000000 */
    2.73370053816464559624e-44, /* 0x36E38222, 0x80000000 */
    2.16741683877804819444e-51, /* 0x3569F31D, 0x00000000 */
};

} // namespace

// prec is always 1 (double) here, but kept as a parameter to mirror musl's
// own signature/structure faithfully.
int rem_pio2_large(double* x, double* y, int e0, int nx, int prec) {
    static const int init_jk[] = {3, 4, 4, 6};

    int32_t jz, jx, jv, jp, jk, carry, n, iq[20], i, j, k, m, q0, ih;
    double z, fw, f[20], fq[20], q[20];

    jk = init_jk[prec];
    jp = jk;

    jx = nx - 1;
    jv = (e0 - 3) / 24;
    if (jv < 0) jv = 0;
    q0 = e0 - 24 * (jv + 1);

    j = jv - jx;
    m = jx + jk;
    for (i = 0; i <= m; i++, j++)
        f[i] = j < 0 ? 0.0 : static_cast<double>(ipio2[j]);

    for (i = 0; i <= jk; i++) {
        for (j = 0, fw = 0.0; j <= jx; j++) fw += x[j] * f[jx + i - j];
        q[i] = fw;
    }

    jz = jk;
recompute:
    for (i = 0, j = jz, z = q[jz]; j > 0; i++, j--) {
        fw = static_cast<double>(static_cast<int32_t>(0x1p-24 * z));
        iq[i] = static_cast<int32_t>(z - 0x1p24 * fw);
        z = q[j - 1] + fw;
    }

    z = db_scalbn(z, q0);
    z -= 8.0 * db_floor(z * 0.125);
    n = static_cast<int32_t>(z);
    z -= static_cast<double>(n);
    ih = 0;
    if (q0 > 0) {
        i = iq[jz - 1] >> (24 - q0);
        n += i;
        iq[jz - 1] -= i << (24 - q0);
        ih = iq[jz - 1] >> (23 - q0);
    } else if (q0 == 0) {
        ih = iq[jz - 1] >> 23;
    } else if (z >= 0.5) {
        ih = 2;
    }

    if (ih > 0) {
        n += 1;
        carry = 0;
        for (i = 0; i < jz; i++) {
            j = iq[i];
            if (carry == 0) {
                if (j != 0) {
                    carry = 1;
                    iq[i] = 0x1000000 - j;
                }
            } else {
                iq[i] = 0xffffff - j;
            }
        }
        if (q0 > 0) {
            switch (q0) {
                case 1: iq[jz - 1] &= 0x7fffff; break;
                case 2: iq[jz - 1] &= 0x3fffff; break;
                default: break;
            }
        }
        if (ih == 2) {
            z = 1.0 - z;
            if (carry != 0) z -= db_scalbn(1.0, q0);
        }
    }

    if (z == 0.0) {
        j = 0;
        for (i = jz - 1; i >= jk; i--) j |= iq[i];
        if (j == 0) {
            for (k = 1; iq[jk - k] == 0; k++) {}
            for (i = jz + 1; i <= jz + k; i++) {
                f[jx + i] = static_cast<double>(ipio2[jv + i]);
                for (j = 0, fw = 0.0; j <= jx; j++) fw += x[j] * f[jx + i - j];
                q[i] = fw;
            }
            jz += k;
            goto recompute;
        }
    }

    if (z == 0.0) {
        jz -= 1;
        q0 -= 24;
        while (iq[jz] == 0) {
            jz--;
            q0 -= 24;
        }
    } else {
        z = db_scalbn(z, -q0);
        if (z >= 0x1p24) {
            fw = static_cast<double>(static_cast<int32_t>(0x1p-24 * z));
            iq[jz] = static_cast<int32_t>(z - 0x1p24 * fw);
            jz += 1;
            q0 += 24;
            iq[jz] = static_cast<int32_t>(fw);
        } else {
            iq[jz] = static_cast<int32_t>(z);
        }
    }

    fw = db_scalbn(1.0, q0);
    for (i = jz; i >= 0; i--) {
        q[i] = fw * static_cast<double>(iq[i]);
        fw *= 0x1p-24;
    }

    for (i = jz; i >= 0; i--) {
        for (fw = 0.0, k = 0; k <= jp && k <= jz - i; k++) fw += PIo2[k] * q[i + k];
        fq[jz - i] = fw;
    }

    // prec == 1 (double) only - the port drops musl's prec==0 (float) and
    // prec==3 (quad) branches, which core never uses.
    fw = 0.0;
    for (i = jz; i >= 0; i--) fw += fq[i];
    fw = static_cast<double>(fw);
    y[0] = ih == 0 ? fw : -fw;
    fw = fq[0] - fw;
    for (i = 1; i <= jz; i++) fw += fq[i];
    y[1] = ih == 0 ? fw : -fw;
    return n & 7;
}

// ---- __rem_pio2 (e_rem_pio2.c) ----

namespace {
const double toint = 1.5 / DBL_EPSILON;
const double pio4 = 0x1.921fb54442d18p-1;
const double invpio2 = 6.36619772367581382433e-01; /* 0x3FE45F30, 0x6DC9C883 */
const double pio2_1 = 1.57079632673412561417e+00;  /* 0x3FF921FB, 0x54400000 */
const double pio2_1t = 6.07710050650619224932e-11; /* 0x3DD0B461, 0x1A626331 */
const double pio2_2 = 6.07710050630396597660e-11;  /* 0x3DD0B461, 0x1A600000 */
const double pio2_2t = 2.02226624879595063154e-21; /* 0x3BA3198A, 0x2E037073 */
const double pio2_3 = 2.02226624871116645580e-21;  /* 0x3BA3198A, 0x2E000000 */
const double pio2_3t = 8.47842766036889956997e-32; /* 0x397B839A, 0x252049C1 */
} // namespace

int rem_pio2(double x, double* y) {
    const uint64_t u = asuint64(x);
    double z, w, t, r, fn;
    double tx[3], ty[2];
    uint32_t ix;
    int sign, n, ex, ey, i;

    sign = static_cast<int>(u >> 63);
    ix = static_cast<uint32_t>((u >> 32) & 0x7fffffff);
    if (ix <= 0x400f6a7a) { /* |x| ~<= 5pi/4 */
        if ((ix & 0xfffff) == 0x921fb) goto medium; /* cancellation */
        if (ix <= 0x4002d97c) {                      /* |x| ~<= 3pi/4 */
            if (!sign) {
                z = x - pio2_1;
                y[0] = z - pio2_1t;
                y[1] = (z - y[0]) - pio2_1t;
                return 1;
            } else {
                z = x + pio2_1;
                y[0] = z + pio2_1t;
                y[1] = (z - y[0]) + pio2_1t;
                return -1;
            }
        } else {
            if (!sign) {
                z = x - 2 * pio2_1;
                y[0] = z - 2 * pio2_1t;
                y[1] = (z - y[0]) - 2 * pio2_1t;
                return 2;
            } else {
                z = x + 2 * pio2_1;
                y[0] = z + 2 * pio2_1t;
                y[1] = (z - y[0]) + 2 * pio2_1t;
                return -2;
            }
        }
    }
    if (ix <= 0x401c463b) { /* |x| ~<= 9pi/4 */
        if (ix <= 0x4015fdbc) { /* |x| ~<= 7pi/4 */
            if (ix == 0x4012d97c) goto medium;
            if (!sign) {
                z = x - 3 * pio2_1;
                y[0] = z - 3 * pio2_1t;
                y[1] = (z - y[0]) - 3 * pio2_1t;
                return 3;
            } else {
                z = x + 3 * pio2_1;
                y[0] = z + 3 * pio2_1t;
                y[1] = (z - y[0]) + 3 * pio2_1t;
                return -3;
            }
        } else {
            if (ix == 0x401921fb) goto medium;
            if (!sign) {
                z = x - 4 * pio2_1;
                y[0] = z - 4 * pio2_1t;
                y[1] = (z - y[0]) - 4 * pio2_1t;
                return 4;
            } else {
                z = x + 4 * pio2_1;
                y[0] = z + 4 * pio2_1t;
                y[1] = (z - y[0]) + 4 * pio2_1t;
                return -4;
            }
        }
    }
    if (ix < 0x413921fb) { /* |x| ~< 2^20*(pi/2), medium size */
    medium:
        fn = x * invpio2 + toint - toint;
        n = static_cast<int32_t>(fn);
        r = x - fn * pio2_1;
        w = fn * pio2_1t;
        if (predict_false(r - w < -pio4)) {
            n--;
            fn--;
            r = x - fn * pio2_1;
            w = fn * pio2_1t;
        } else if (predict_false(r - w > pio4)) {
            n++;
            fn++;
            r = x - fn * pio2_1;
            w = fn * pio2_1t;
        }
        y[0] = r - w;
        ey = static_cast<int>((asuint64(y[0]) >> 52) & 0x7ff);
        ex = static_cast<int>(ix >> 20);
        if (ex - ey > 16) { /* 2nd round, good to 118 bits */
            t = r;
            w = fn * pio2_2;
            r = t - w;
            w = fn * pio2_2t - ((t - r) - w);
            y[0] = r - w;
            ey = static_cast<int>((asuint64(y[0]) >> 52) & 0x7ff);
            if (ex - ey > 49) { /* 3rd round, good to 151 bits */
                t = r;
                w = fn * pio2_3;
                r = t - w;
                w = fn * pio2_3t - ((t - r) - w);
                y[0] = r - w;
            }
        }
        y[1] = (r - y[0]) - w;
        return n;
    }
    if (ix >= 0x7ff00000) { /* x is inf or NaN */
        y[0] = y[1] = x - x;
        return 0;
    }
    // set z = scalbn(|x|, -ilogb(x)+23)
    {
        uint64_t uz = asuint64(x);
        uz &= (~static_cast<uint64_t>(0)) >> 12;
        uz |= static_cast<uint64_t>(0x3ff + 23) << 52;
        z = asdouble(uz);
    }
    for (i = 0; i < 2; i++) {
        tx[i] = static_cast<double>(static_cast<int32_t>(z));
        z = (z - tx[i]) * 0x1p24;
    }
    tx[i] = z;
    while (tx[i] == 0.0) i--;
    n = rem_pio2_large(tx, ty, static_cast<int>(ix >> 20) - (0x3ff + 23), i + 1, 1);
    if (sign) {
        y[0] = -ty[0];
        y[1] = -ty[1];
        return -n;
    }
    y[0] = ty[0];
    y[1] = ty[1];
    return n;
}

} // namespace ps::math::detmath
