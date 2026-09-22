// core/src/math/detmath/trig.cpp — sin/cos/tan/sincos.
// Ported from musl libm v1.2.5: src/math/sin.c, cos.c, tan.c, sincos.c
// (origin: FreeBSD s_sin.c / s_cos.c / s_tan.c). See common.h for the
// porting rules and THIRD_PARTY_NOTICES.md for the licence.
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#include "detmath.h"
#include "common.h"
#include "rem_pio2.h"
#include "trig_kernel.h"

namespace ps::math::detmath {

// musl's FORCE_EVAL(...) calls in the tiny-|x| fast paths below only exist
// to raise the FE_INEXACT/FE_UNDERFLOW hardware exception flags; this port
// drops them (see common.h's file comment / rule 2 "no compiler builtins")
// since core never reads fenv state and the returned VALUE is identical
// with or without them - they are pure side effects on flags, not on bits.

double sin(double x) {
    double y[2];
    uint32_t ix = get_high_word(x) & 0x7fffffff;

    if (ix <= 0x3fe921fb) { /* |x| ~< pi/4 */
        if (ix < 0x3e500000) return x; /* |x| < 2**-26 */
        return kernel_sin(x, 0.0, 0);
    }
    if (ix >= 0x7ff00000) return x - x; /* sin(Inf or NaN) is NaN */

    const int n = rem_pio2(x, y);
    switch (n & 3) {
        case 0: return kernel_sin(y[0], y[1], 1);
        case 1: return kernel_cos(y[0], y[1]);
        case 2: return -kernel_sin(y[0], y[1], 1);
        default: return -kernel_cos(y[0], y[1]);
    }
}

double cos(double x) {
    double y[2];
    uint32_t ix = get_high_word(x) & 0x7fffffff;

    if (ix <= 0x3fe921fb) { /* |x| ~< pi/4 */
        if (ix < 0x3e46a09e) return 1.0; /* |x| < 2**-27 * sqrt(2) */
        return kernel_cos(x, 0.0);
    }
    if (ix >= 0x7ff00000) return x - x; /* cos(Inf or NaN) is NaN */

    const int n = rem_pio2(x, y);
    switch (n & 3) {
        case 0: return kernel_cos(y[0], y[1]);
        case 1: return -kernel_sin(y[0], y[1], 1);
        case 2: return -kernel_cos(y[0], y[1]);
        default: return kernel_sin(y[0], y[1], 1);
    }
}

double tan(double x) {
    double y[2];
    uint32_t ix = get_high_word(x) & 0x7fffffff;

    if (ix <= 0x3fe921fb) { /* |x| ~< pi/4 */
        if (ix < 0x3e400000) return x; /* |x| < 2**-27 */
        return kernel_tan(x, 0.0, 0);
    }
    if (ix >= 0x7ff00000) return x - x; /* tan(Inf or NaN) is NaN */

    const int n = rem_pio2(x, y);
    return kernel_tan(y[0], y[1], n & 1);
}

void sincos(double x, double* s, double* c) {
    double y[2];
    uint32_t ix = get_high_word(x) & 0x7fffffff;

    if (ix <= 0x3fe921fb) { /* |x| ~< pi/4 */
        if (ix < 0x3e46a09e) {
            *s = x;
            *c = 1.0;
            return;
        }
        *s = kernel_sin(x, 0.0, 0);
        *c = kernel_cos(x, 0.0);
        return;
    }
    if (ix >= 0x7ff00000) {
        *s = *c = x - x;
        return;
    }

    const int n = rem_pio2(x, y);
    const double sn = kernel_sin(y[0], y[1], 1);
    const double cn = kernel_cos(y[0], y[1]);
    switch (n & 3) {
        case 0: *s = sn; *c = cn; break;
        case 1: *s = cn; *c = -sn; break;
        case 2: *s = -sn; *c = -cn; break;
        default: *s = -cn; *c = sn; break;
    }
}

} // namespace ps::math::detmath
