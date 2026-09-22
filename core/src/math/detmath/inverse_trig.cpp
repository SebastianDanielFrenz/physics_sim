// core/src/math/detmath/inverse_trig.cpp — asin/acos/atan/atan2.
// Ported from musl libm v1.2.5: src/math/asin.c, acos.c, atan.c, atan2.c
// (origin: FreeBSD e_asin.c / e_acos.c / s_atan.c / e_atan2.c). See
// common.h for the porting rules and THIRD_PARTY_NOTICES.md for the
// licence.
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro / SunSoft, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#include "detmath.h"
#include "common.h"

#include <cmath> // std::sqrt only (IEEE-exact, see common.h file comment)

namespace ps::math::detmath {
namespace {

// Shared by asin() and acos() (musl duplicates this verbatim in both
// asin.c and acos.c; this port keeps a single copy - same constants, same
// polynomial, same rounding, byte-for-byte).
constexpr double pio2_hi = 1.57079632679489655800e+00; /* 0x3FF921FB, 0x54442D18 */
constexpr double pio2_lo = 6.12323399573676603587e-17; /* 0x3C91A626, 0x33145C07 */
constexpr double pS0 = 1.66666666666666657415e-01;  /* 0x3FC55555, 0x55555555 */
constexpr double pS1 = -3.25565818622400915405e-01; /* 0xBFD4D612, 0x03EB6F7D */
constexpr double pS2 = 2.01212532134862925881e-01;  /* 0x3FC9C155, 0x0E884455 */
constexpr double pS3 = -4.00555345006794114027e-02; /* 0xBFA48228, 0xB5688F3B */
constexpr double pS4 = 7.91534994289814532176e-04;  /* 0x3F49EFE0, 0x7501B288 */
constexpr double pS5 = 3.47933107596021167570e-05;  /* 0x3F023DE1, 0x0DFDF709 */
constexpr double qS1 = -2.40339491173441421878e+00; /* 0xC0033A27, 0x1C8A2D4B */
constexpr double qS2 = 2.02094576023350569471e+00;  /* 0x40002AE5, 0x9C598AC8 */
constexpr double qS3 = -6.88283971605453293030e-01; /* 0xBFE6066C, 0x1B8D0159 */
constexpr double qS4 = 7.70381505559019352791e-02;  /* 0x3FB3B8C5, 0xB12E9282 */

double asin_acos_R(double z) {
    const double p = z * (pS0 + z * (pS1 + z * (pS2 + z * (pS3 + z * (pS4 + z * pS5)))));
    const double q = 1.0 + z * (qS1 + z * (qS2 + z * (qS3 + z * qS4)));
    return p / q;
}

} // namespace

double asin(double x) {
    uint32_t hx = get_high_word(x);
    uint32_t ix = hx & 0x7fffffff;
    if (ix >= 0x3ff00000) { /* |x| >= 1 or nan */
        uint32_t lx = get_low_word(x);
        if ((ix - 0x3ff00000 | lx) == 0) return x * pio2_hi + 0x1p-120; /* asin(1)=+-pi/2 */
        return 0 / (x - x);
    }
    if (ix < 0x3fe00000) { /* |x| < 0.5 */
        if (ix < 0x3e500000 && ix >= 0x00100000) return x;
        return x + x * asin_acos_R(x * x);
    }
    double z = (1 - db_fabs(x)) * 0.5;
    double s = std::sqrt(z);
    double r = asin_acos_R(z);
    double out;
    if (ix >= 0x3fef3333) { /* |x| > 0.975 */
        out = pio2_hi - (2 * (s + s * r) - pio2_lo);
    } else {
        double f = set_low_word(s, 0);
        double c = (z - f * f) / (s + f);
        out = 0.5 * pio2_hi - (2 * s * r - (pio2_lo - 2 * c) - (0.5 * pio2_hi - 2 * f));
    }
    return (hx >> 31) ? -out : out;
}

double acos(double x) {
    uint32_t hx = get_high_word(x);
    uint32_t ix = hx & 0x7fffffff;
    if (ix >= 0x3ff00000) { /* |x| >= 1 or nan */
        uint32_t lx = get_low_word(x);
        if ((ix - 0x3ff00000 | lx) == 0) {
            if (hx >> 31) return 2 * pio2_hi + 0x1p-120; /* acos(-1)=pi */
            return 0;                                    /* acos(1)=0 */
        }
        return 0 / (x - x);
    }
    if (ix < 0x3fe00000) { /* |x| < 0.5 */
        if (ix <= 0x3c600000) return pio2_hi + 0x1p-120; /* |x| < 2**-57 */
        return pio2_hi - (x - (pio2_lo - x * asin_acos_R(x * x)));
    }
    if (hx >> 31) { /* x < -0.5 */
        double z = (1.0 + x) * 0.5;
        double s = std::sqrt(z);
        double w = asin_acos_R(z) * s - pio2_lo;
        return 2 * (pio2_hi - (s + w));
    }
    /* x > 0.5 */
    double z = (1.0 - x) * 0.5;
    double s = std::sqrt(z);
    double df = set_low_word(s, 0);
    double c = (z - df * df) / (s + df);
    double w = asin_acos_R(z) * s + c;
    return 2 * (df + w);
}

double atan(double x) {
    static const double atanhi[] = {
        4.63647609000806093515e-01, /* atan(0.5)hi */
        7.85398163397448278999e-01, /* atan(1.0)hi */
        9.82793723247329054082e-01, /* atan(1.5)hi */
        1.57079632679489655800e+00, /* atan(inf)hi */
    };
    static const double atanlo[] = {
        2.26987774529616870924e-17, /* atan(0.5)lo */
        3.06161699786838301793e-17, /* atan(1.0)lo */
        1.39033110312309984516e-17, /* atan(1.5)lo */
        6.12323399573676603587e-17, /* atan(inf)lo */
    };
    static const double aT[] = {
        3.33333333333329318027e-01,  -1.99999999998764832476e-01,
        1.42857142725034663711e-01,  -1.11111104054623557880e-01,
        9.09088713343650656196e-02,  -7.69187620504482999495e-02,
        6.66107313738753120669e-02,  -5.83357013379057348645e-02,
        4.97687799461593236017e-02,  -3.65315727442169155270e-02,
        1.62858201153657823623e-02,
    };

    double w, s1, s2, z;
    uint32_t ix = get_high_word(x);
    const uint32_t sign = ix >> 31;
    ix &= 0x7fffffff;
    int id;

    if (ix >= 0x44100000) { /* |x| >= 2^66 */
        if (std::isnan(x)) return x;
        z = atanhi[3] + 0x1p-120;
        return sign ? -z : z;
    }
    if (ix < 0x3fdc0000) { /* |x| < 0.4375 */
        if (ix < 0x3e400000) return x; /* |x| < 2^-27 */
        id = -1;
    } else {
        x = db_fabs(x);
        if (ix < 0x3ff30000) { /* |x| < 1.1875 */
            if (ix < 0x3fe60000) {
                id = 0;
                x = (2.0 * x - 1.0) / (2.0 + x);
            } else {
                id = 1;
                x = (x - 1.0) / (x + 1.0);
            }
        } else {
            if (ix < 0x40038000) {
                id = 2;
                x = (x - 1.5) / (1.0 + 1.5 * x);
            } else {
                id = 3;
                x = -1.0 / x;
            }
        }
    }
    z = x * x;
    w = z * z;
    s1 = z * (aT[0] + w * (aT[2] + w * (aT[4] + w * (aT[6] + w * (aT[8] + w * aT[10])))));
    s2 = w * (aT[1] + w * (aT[3] + w * (aT[5] + w * (aT[7] + w * aT[9]))));
    if (id < 0) return x - x * (s1 + s2);
    z = atanhi[id] - (x * (s1 + s2) - atanlo[id] - x);
    return sign ? -z : z;
}

double atan2(double y, double x) {
    constexpr double pi = 3.1415926535897931160E+00;
    constexpr double pi_lo = 1.2246467991473531772E-16;

    if (std::isnan(x) || std::isnan(y)) return x + y;
    uint32_t ix, lx, iy, ly;
    extract_words(ix, lx, x);
    extract_words(iy, ly, y);
    if ((ix - 0x3ff00000 | lx) == 0) return atan(y); /* x = 1.0 */
    const uint32_t m = ((iy >> 31) & 1) | ((ix >> 30) & 2); /* 2*sign(x)+sign(y) */
    ix &= 0x7fffffff;
    iy &= 0x7fffffff;

    if ((iy | ly) == 0) { /* y == 0 */
        switch (m) {
            case 0:
            case 1: return y;
            case 2: return pi;
            default: return -pi;
        }
    }
    if ((ix | lx) == 0) return (m & 1) ? -pi / 2 : pi / 2; /* x == 0 */
    if (ix == 0x7ff00000) {                                /* x is inf */
        if (iy == 0x7ff00000) {
            switch (m) {
                case 0: return pi / 4;
                case 1: return -pi / 4;
                case 2: return 3 * pi / 4;
                default: return -3 * pi / 4;
            }
        } else {
            switch (m) {
                case 0: return 0.0;
                case 1: return -0.0;
                case 2: return pi;
                default: return -pi;
            }
        }
    }
    if (ix + (64u << 20) < iy || iy == 0x7ff00000) return (m & 1) ? -pi / 2 : pi / 2;

    double z;
    if ((m & 2) && iy + (64u << 20) < ix) /* |y/x| < 2^-64, x<0 */
        z = 0;
    else
        z = atan(db_fabs(y / x));
    switch (m) {
        case 0: return z;
        case 1: return -z;
        case 2: return pi - (z - pi_lo);
        default: return (z - pi_lo) - pi;
    }
}

} // namespace ps::math::detmath
