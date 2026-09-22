// core/src/math/detmath/exp.cpp — exp(x).
// Ported from musl libm v1.2.5, src/math/exp.c (Copyright (c) 2018, Arm
// Limited; SPDX-License-Identifier: MIT). See common.h for the porting
// rules and THIRD_PARTY_NOTICES.md for the licence.
//
// This port always takes upstream's "no FMA" path (__FP_FAST_FMA == 0):
// no compiler builtin is used anywhere in the port (common.h's rule 2),
// and it keeps the result independent of whether the target happens to
// fuse multiply-adds, which is what PLAN.md D7 requires (core also builds
// with FMA contraction off globally, TOOL-018).

#include "detmath.h"
#include "common.h"
#include "exp_data.h"

namespace ps::math::detmath {
namespace {

constexpr int N = 1 << kExpTableBits;
const double InvLn2N = exp_data.invln2N;
const double NegLn2hiN = exp_data.negln2hiN;
const double NegLn2loN = exp_data.negln2loN;
const double Shift = exp_data.shift;
const double C2 = exp_data.poly[5 - kExpPolyOrder];
const double C3 = exp_data.poly[6 - kExpPolyOrder];
const double C4 = exp_data.poly[7 - kExpPolyOrder];
const double C5 = exp_data.poly[8 - kExpPolyOrder];

uint32_t top12(double x) { return static_cast<uint32_t>(asuint64(x) >> 52); }

// Handles the case that may overflow or underflow when computing
// scale*(1+tmp) without intermediate rounding (musl's exp.c specialcase()).
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
    if (y < 1.0) {
        double hi, lo;
        lo = scale - y + scale * tmp;
        hi = 1.0 + y;
        lo = 1.0 - hi + y + lo;
        y = eval_as_double(hi + lo) - 1.0;
        if (want_rounding && y == 0.0) y = 0.0;
        fp_force_eval(fp_barrier(0x1p-1022) * 0x1p-1022);
    }
    y = 0x1p-1022 * y;
    return eval_as_double(y);
}

} // namespace

double exp(double x) {
    uint32_t abstop = top12(x) & 0x7ff;
    if (predict_false(abstop - top12(0x1p-54) >= top12(512.0) - top12(0x1p-54))) {
        if (abstop - top12(0x1p-54) >= 0x80000000u) return want_rounding ? 1.0 + x : 1.0;
        if (abstop >= top12(1024.0)) {
            if (asuint64(x) == asuint64(-HUGE_VAL)) return 0.0; /* exp(-inf) */
            if (abstop >= top12(HUGE_VAL)) return 1.0 + x;      /* inf or nan */
            if (asuint64(x) >> 63) return math_uflow(0);
            return math_oflow(0);
        }
        abstop = 0;
    }

    double z = InvLn2N * x;
    double kd = eval_as_double(z + Shift);
    uint64_t ki = asuint64(kd);
    kd -= Shift;

    double r = x + kd * NegLn2hiN + kd * NegLn2loN;
    uint64_t idx = 2 * (ki % N);
    uint64_t top = ki << (52 - kExpTableBits);
    double tail = asdouble(exp_data.tab[idx]);
    uint64_t sbits = exp_data.tab[idx + 1] + top;
    double r2 = r * r;
    double tmp = tail + r + r2 * (C2 + r * C3) + r2 * r2 * (C4 + r * C5);
    if (predict_false(abstop == 0)) return specialcase(tmp, sbits, ki);
    double scale = asdouble(sbits);
    return eval_as_double(scale + scale * tmp);
}

} // namespace ps::math::detmath
