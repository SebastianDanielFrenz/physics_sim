// core/src/math/detmath/log.cpp — log(x).
// Ported from musl libm v1.2.5, src/math/log.c (Copyright (c) 2018, Arm
// Limited; SPDX-License-Identifier: MIT). See common.h for the porting
// rules and THIRD_PARTY_NOTICES.md for the licence. Always takes
// upstream's "no FMA" path, like exp.cpp - see that file's comment.

#include "detmath.h"
#include "common.h"
#include "log_data.h"

namespace ps::math::detmath {
namespace {

constexpr int N = 1 << kLogTableBits;
constexpr uint64_t OFF = 0x3fe6000000000000ull;

uint32_t top16(double x) { return static_cast<uint32_t>(asuint64(x) >> 48); }

} // namespace

double log(double x) {
    const double* A = log_data.poly;
    const double* B = log_data.poly1;
    const double Ln2hi = log_data.ln2hi;
    const double Ln2lo = log_data.ln2lo;

    uint64_t ix = asuint64(x);
    uint32_t top = top16(x);
    const uint64_t LO = asuint64(1.0 - 0x1p-4);
    const uint64_t HI = asuint64(1.0 + 0x1.09p-4);
    if (predict_false(ix - LO < HI - LO)) {
        if (want_rounding && predict_false(ix == asuint64(1.0))) return 0;
        double r = x - 1.0;
        double r2 = r * r;
        double r3 = r * r2;
        double y = r3 * (B[1] + r * B[2] + r2 * B[3] +
                          r3 * (B[4] + r * B[5] + r2 * B[6] +
                                r3 * (B[7] + r * B[8] + r2 * B[9] + r3 * B[10])));
        double w = r * 0x1p27;
        double rhi = r + w - w;
        double rlo = r - rhi;
        w = rhi * rhi * B[0]; /* B[0] == -0.5 */
        double hi = r + w;
        double lo = r - hi + w;
        lo += B[0] * rlo * (rhi + r);
        y += lo;
        y += hi;
        return eval_as_double(y);
    }
    if (predict_false(top - 0x0010 >= 0x7ff0 - 0x0010)) {
        if (ix * 2 == 0) return math_divzero(1);
        if (ix == asuint64(HUGE_VAL)) return x; /* log(inf) == inf */
        if ((top & 0x8000) || (top & 0x7ff0) == 0x7ff0) return math_invalid(x);
        ix = asuint64(x * 0x1p52);
        ix -= 52ULL << 52;
    }

    uint64_t tmp = ix - OFF;
    int i = static_cast<int>((tmp >> (52 - kLogTableBits)) % N);
    int k = static_cast<int>(static_cast<int64_t>(tmp) >> 52);
    uint64_t iz = ix - (tmp & (0xfffULL << 52));
    double invc = log_data.tab[i].invc;
    double logc = log_data.tab[i].logc;
    double z = asdouble(iz);

    double r = (z - log_data.tab2[i].chi - log_data.tab2[i].clo) * invc;
    double kd = static_cast<double>(k);

    double w = kd * Ln2hi + logc;
    double hi = w + r;
    double lo = w - hi + r + kd * Ln2lo;

    double r2 = r * r;
    double y = lo + r2 * A[0] + r * r2 * (A[1] + r * A[2] + r2 * (A[3] + r * A[4])) + hi;
    return eval_as_double(y);
}

} // namespace ps::math::detmath
