// core/src/math/detmath/trig_kernel.cpp
// Ported from musl libm v1.2.5: src/math/__sin.c, __cos.c, __tan.c
// (origin: FreeBSD k_sin.c / k_cos.c / k_tan.c). See common.h for the
// porting rules and THIRD_PARTY_NOTICES.md for the licence.
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 * Copyright 2004 Sun Microsystems, Inc.  All Rights Reserved.
 *
 * Developed at SunSoft, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#include "trig_kernel.h"
#include "common.h"

namespace ps::math::detmath {

double kernel_sin(double x, double y, int iy) {
    static const double S1 = -1.66666666666666324348e-01; /* 0xBFC55555, 0x55555549 */
    static const double S2 = 8.33333333332248946124e-03;  /* 0x3F811111, 0x1110F8A6 */
    static const double S3 = -1.98412698298579493134e-04; /* 0xBF2A01A0, 0x19C161D5 */
    static const double S4 = 2.75573137070700676789e-06;  /* 0x3EC71DE3, 0x57B1FE7D */
    static const double S5 = -2.50507602534068634195e-08; /* 0xBE5AE5E6, 0x8A2B9CEB */
    static const double S6 = 1.58969099521155010221e-10;  /* 0x3DE5D93A, 0x5ACFD57C */

    double z, r, v, w;
    z = x * x;
    w = z * z;
    r = S2 + z * (S3 + z * S4) + z * w * (S5 + z * S6);
    v = z * x;
    if (iy == 0)
        return x + v * (S1 + z * r);
    else
        return x - ((z * (0.5 * y - v * r) - y) - v * S1);
}

double kernel_cos(double x, double y) {
    static const double C1 = 4.16666666666666019037e-02;  /* 0x3FA55555, 0x5555554C */
    static const double C2 = -1.38888888888741095749e-03; /* 0xBF56C16C, 0x16C15177 */
    static const double C3 = 2.48015872894767294178e-05;  /* 0x3EFA01A0, 0x19CB1590 */
    static const double C4 = -2.75573143513906633035e-07; /* 0xBE927E4F, 0x809C52AD */
    static const double C5 = 2.08757232129817482790e-09;  /* 0x3E21EE9E, 0xBDB4B1C4 */
    static const double C6 = -1.13596475577881948265e-11; /* 0xBDA8FAE9, 0xBE8838D4 */

    double hz, z, r, w;
    z = x * x;
    w = z * z;
    r = z * (C1 + z * (C2 + z * C3)) + w * w * (C4 + z * (C5 + z * C6));
    hz = 0.5 * z;
    w = 1.0 - hz;
    return w + (((1.0 - w) - hz) + (z * r - x * y));
}

double kernel_tan(double x, double y, int odd) {
    static const double T[] = {
        3.33333333333334091986e-01,  /* 3FD55555, 55555563 */
        1.33333333333201242699e-01,  /* 3FC11111, 1110FE7A */
        5.39682539762260521377e-02,  /* 3FABA1BA, 1BB341FE */
        2.18694882948595424599e-02,  /* 3F9664F4, 8406D637 */
        8.86323982359930005737e-03,  /* 3F8226E3, E96E8493 */
        3.59207910759131235356e-03,  /* 3F6D6D22, C9560328 */
        1.45620945432529025516e-03,  /* 3F57DBC8, FEE08315 */
        5.88041240820264096874e-04,  /* 3F4344D8, F2F26501 */
        2.46463134818469906812e-04,  /* 3F3026F7, 1A8D1068 */
        7.81794442939557092300e-05,  /* 3F147E88, A03792A6 */
        7.14072491382608190305e-05,  /* 3F12B80F, 32F0A7E9 */
        -1.85586374855275456654e-05, /* BEF375CB, DB605373 */
        2.59073051863633712884e-05,  /* 3EFB2A70, 74BF7AD4 */
    };
    static const double pio4 = 7.85398163397448278999e-01;   /* 3FE921FB, 54442D18 */
    static const double pio4lo = 3.06161699786838301793e-17; /* 3C81A626, 33145C07 */

    double z, r, v, w, s, a;
    double w0, a0;
    uint32_t hx;
    int big, sign = 0; // sign is only read when big is true, when it is always set below;
                        // initialized here only to satisfy cl's (imprecise) flow analysis

    hx = get_high_word(x);
    big = (hx & 0x7fffffff) >= 0x3FE59428; /* |x| >= 0.6744 */
    if (big) {
        sign = static_cast<int>(hx >> 31);
        if (sign) {
            x = -x;
            y = -y;
        }
        x = (pio4 - x) + (pio4lo - y);
        y = 0.0;
    }
    z = x * x;
    w = z * z;
    r = T[1] + w * (T[3] + w * (T[5] + w * (T[7] + w * (T[9] + w * T[11]))));
    v = z * (T[2] + w * (T[4] + w * (T[6] + w * (T[8] + w * (T[10] + w * T[12])))));
    s = z * x;
    r = y + z * (s * (r + v) + y) + s * T[0];
    w = x + r;
    if (big) {
        s = 1 - 2 * odd;
        v = s - 2.0 * (x + (r - w * w / (w + s)));
        return sign ? -v : v;
    }
    if (!odd) return w;
    /* -1.0/(x+r) has up to 2ulp error, so compute it accurately */
    w0 = w;
    w0 = set_low_word(w0, 0);
    v = r - (w0 - x); /* w0+v = r+x */
    a0 = a = -1.0 / w;
    a0 = set_low_word(a0, 0);
    return a0 + a * (1.0 + a0 * w0 + a0 * v);
}

} // namespace ps::math::detmath
