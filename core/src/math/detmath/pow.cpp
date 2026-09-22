// core/src/math/detmath/pow.cpp — pow(x, y).
// Ported from musl libm v1.2.5, src/math/pow.c (Copyright (c) 2018, Arm
// Limited; SPDX-License-Identifier: MIT). See common.h for the porting
// rules and THIRD_PARTY_NOTICES.md for the licence. Always takes
// upstream's "no FMA" path, like exp.cpp/log.cpp - see exp.cpp's comment.

#include "detmath.h"
#include "common.h"
#include "exp_data.h"
#include "pow_data.h"

namespace ps::math::detmath {
namespace {

uint32_t top12(double x) { return static_cast<uint32_t>(asuint64(x) >> 52); }

// Computes y+tail = log(x) where the rounded result is y and tail has
// about 15 extra bits of precision (musl pow.c's log_inline).
double log_inline(uint64_t ix, double* tail) {
    constexpr int N = 1 << kPowLogTableBits;
    constexpr uint64_t OFF = 0x3fe6955500000000ull;
    const double* A = pow_log_data.poly;
    const double Ln2hi = pow_log_data.ln2hi;
    const double Ln2lo = pow_log_data.ln2lo;

    uint64_t tmp = ix - OFF;
    int i = static_cast<int>((tmp >> (52 - kPowLogTableBits)) % N);
    int k = static_cast<int>(static_cast<int64_t>(tmp) >> 52);
    uint64_t iz = ix - (tmp & (0xfffULL << 52));
    double z = asdouble(iz);
    double kd = static_cast<double>(k);

    double invc = pow_log_data.tab[i].invc;
    double logc = pow_log_data.tab[i].logc;
    double logctail = pow_log_data.tab[i].logctail;

    double zhi = asdouble((iz + (1ULL << 31)) & (~0ULL << 32));
    double zlo = z - zhi;
    double rhi = zhi * invc - 1.0;
    double rlo = zlo * invc;
    double r = rhi + rlo;

    double t1 = kd * Ln2hi + logc;
    double t2 = t1 + r;
    double lo1 = kd * Ln2lo + logctail;
    double lo2 = t1 - t2 + r;

    double ar = A[0] * r; /* A[0] == -0.5 */
    double ar2 = r * ar;
    double ar3 = r * ar2;
    double arhi = A[0] * rhi;
    double arhi2 = rhi * arhi;
    double hi = t2 + arhi2;
    double lo3 = rlo * (ar + arhi);
    double lo4 = t2 - hi + arhi2;
    double p = ar3 * (A[1] + r * A[2] + ar2 * (A[3] + r * A[4] + ar2 * (A[5] + r * A[6])));
    double lo = lo1 + lo2 + lo3 + lo4 + p;
    double y = hi + lo;
    *tail = hi - y + lo;
    return y;
}

constexpr int N = 1 << kExpTableBits;
constexpr uint32_t SIGN_BIAS = 0x800u << kExpTableBits;

// Handles overflow/underflow the same way exp.cpp's specialcase does, plus
// the sign bias pow needs for x<0, odd integer y (musl pow.c's
// specialcase, distinct from exp.c's - sign handling differs).
double specialcase(double tmp, uint64_t sbits, uint64_t ki) {
    double scale, y;
    if ((ki & 0x80000000) == 0) {
        sbits -= 1009ull << 52;
        scale = asdouble(sbits);
        y = 0x1p1009 * (scale + scale * tmp);
        return eval_as_double(y);
    }
    sbits += 1022ull << 52;
    scale = asdouble(sbits);
    y = scale + scale * tmp;
    if (db_fabs(y) < 1.0) {
        double one = 1.0;
        if (y < 0.0) one = -1.0;
        double lo = scale - y + scale * tmp;
        double hi = one + y;
        lo = one - hi + y + lo;
        y = eval_as_double(hi + lo) - one;
        if (y == 0.0) y = asdouble(sbits & 0x8000000000000000ull);
        fp_force_eval(fp_barrier(0x1p-1022) * 0x1p-1022);
    }
    y = 0x1p-1022 * y;
    return eval_as_double(y);
}

// Computes sign*exp(x+xtail); sign_bias is SIGN_BIAS or 0 (musl pow.c's
// exp_inline).
double exp_inline(double x, double xtail, uint32_t sign_bias) {
    const double InvLn2N = exp_data.invln2N;
    const double NegLn2hiN = exp_data.negln2hiN;
    const double NegLn2loN = exp_data.negln2loN;
    const double Shift = exp_data.shift;
    const double C2 = exp_data.poly[5 - kExpPolyOrder];
    const double C3 = exp_data.poly[6 - kExpPolyOrder];
    const double C4 = exp_data.poly[7 - kExpPolyOrder];
    const double C5 = exp_data.poly[8 - kExpPolyOrder];

    uint32_t abstop = top12(x) & 0x7ff;
    if (predict_false(abstop - top12(0x1p-54) >= top12(512.0) - top12(0x1p-54))) {
        if (abstop - top12(0x1p-54) >= 0x80000000u) {
            double one = want_rounding ? 1.0 + x : 1.0;
            return sign_bias ? -one : one;
        }
        if (abstop >= top12(1024.0)) {
            if (asuint64(x) >> 63) return math_uflow(sign_bias);
            return math_oflow(sign_bias);
        }
        abstop = 0;
    }

    double z = InvLn2N * x;
    double kd = eval_as_double(z + Shift);
    uint64_t ki = asuint64(kd);
    kd -= Shift;

    double r = x + kd * NegLn2hiN + kd * NegLn2loN;
    r += xtail;
    uint64_t idx = 2 * (ki % N);
    uint64_t top = (ki + sign_bias) << (52 - kExpTableBits);
    double tail = asdouble(exp_data.tab[idx]);
    uint64_t sbits = exp_data.tab[idx + 1] + top;
    double r2 = r * r;
    double tmp = tail + r + r2 * (C2 + r * C3) + r2 * r2 * (C4 + r * C5);
    if (predict_false(abstop == 0)) return specialcase(tmp, sbits, ki);
    double scale = asdouble(sbits);
    return eval_as_double(scale + scale * tmp);
}

// Returns 0 if not int, 1 if odd int, 2 if even int (iy is the bit
// representation of a non-zero finite double).
int checkint(uint64_t iy) {
    int e = static_cast<int>(iy >> 52 & 0x7ff);
    if (e < 0x3ff) return 0;
    if (e > 0x3ff + 52) return 2;
    if (iy & ((1ULL << (0x3ff + 52 - e)) - 1)) return 0;
    if (iy & (1ULL << (0x3ff + 52 - e))) return 1;
    return 2;
}

// True if i is the bit representation of 0, infinity or nan.
bool zeroinfnan(uint64_t i) { return 2 * i - 1 >= 2 * asuint64(HUGE_VAL) - 1; }

} // namespace

double pow(double x, double y) {
    uint32_t sign_bias = 0;
    uint64_t ix = asuint64(x);
    uint64_t iy = asuint64(y);
    uint32_t topx = top12(x);
    uint32_t topy = top12(y);

    if (predict_false(topx - 0x001 >= 0x7ff - 0x001 || (topy & 0x7ff) - 0x3be >= 0x43e - 0x3be)) {
        if (predict_false(zeroinfnan(iy))) {
            if (2 * iy == 0) return issignaling_inline(x) ? x + y : 1.0;
            if (ix == asuint64(1.0)) return issignaling_inline(y) ? x + y : 1.0;
            if (2 * ix > 2 * asuint64(HUGE_VAL) || 2 * iy > 2 * asuint64(HUGE_VAL)) return x + y;
            if (2 * ix == 2 * asuint64(1.0)) return 1.0;
            if ((2 * ix < 2 * asuint64(1.0)) == !(iy >> 63)) return 0.0;
            return y * y;
        }
        if (predict_false(zeroinfnan(ix))) {
            double x2 = x * x;
            if ((ix >> 63) && checkint(iy) == 1) x2 = -x2;
            return (iy >> 63) ? fp_barrier(1 / x2) : x2;
        }
        if (ix >> 63) {
            int yint = checkint(iy);
            if (yint == 0) return math_invalid(x);
            if (yint == 1) sign_bias = SIGN_BIAS;
            ix &= 0x7fffffffffffffffull;
            topx &= 0x7ff;
        }
        if ((topy & 0x7ff) - 0x3be >= 0x43e - 0x3be) {
            if (ix == asuint64(1.0)) return 1.0;
            if ((topy & 0x7ff) < 0x3be) return want_rounding ? (ix > asuint64(1.0) ? 1.0 + y : 1.0 - y) : 1.0;
            return (ix > asuint64(1.0)) == (topy < 0x800) ? math_oflow(0) : math_uflow(0);
        }
        if (topx == 0) {
            ix = asuint64(x * 0x1p52);
            ix &= 0x7fffffffffffffffull;
            ix -= 52ULL << 52;
        }
    }

    double lo;
    double hi = log_inline(ix, &lo);
    double yhi = asdouble(iy & (~0ULL << 27));
    double ylo = y - yhi;
    double lhi = asdouble(asuint64(hi) & (~0ULL << 27));
    double llo = hi - lhi + lo;
    double ehi = yhi * lhi;
    double elo = ylo * lhi + y * llo;
    return exp_inline(ehi, elo, sign_bias);
}

} // namespace ps::math::detmath
