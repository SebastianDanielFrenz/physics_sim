// core/src/math/detmath/common.h — internal helpers shared by the ps::math
// transcendental port (PLAN.md D7 / vault PHYS-009).
//
// The algorithms in this directory are ported from musl libm v1.2.5
// (MIT licence, most of them fdlibm-derived; see THIRD_PARTY_NOTICES.md at
// the repo root for the full licence text and exact source commit). Every
// ported file keeps musl's own copyright header. This header re-implements
// musl's small bit-twiddling macros (src/internal/libm.h) as C++ functions:
// - no union type punning: std::bit_cast instead ("EXTRACT_WORDS" etc.)
// - no compiler builtins (__builtin_expect, __builtin_fma): plain code only
// - double-only (ps::real): the long-double and float variants musl also
//   ships are not ported, since core never uses them
//
// double_t: on every platform this project targets (x86-64 with SSE2, no
// x87), FLT_EVAL_METHOD is 0 and double_t is exactly double, so the port
// uses `double` everywhere musl's sources say `double_t`.
//
// floor/scalbn/fabs/copysign are used here the same way sqrt is used
// elsewhere in core (PLAN.md D7): they are exact, correctly-rounded bit
// operations with no library-specific rounding choice (IEEE 754 mandates
// one bit-exact answer for each), unlike sin/cos/exp/log/pow, which is why
// D7's ban on platform transcendentals and the CI grep enforcing it
// (tools/ci.ps1) both exclude them.

#pragma once

#include <bit>
#include <cfloat>
#include <cstdint>
#include <cmath>

namespace ps::math::detmath {

using std::int32_t;
using std::int64_t;
using std::uint32_t;
using std::uint64_t;

// ---- bit-level access (replaces musl's asuint64/asdouble/EXTRACT_WORDS/...) ----

inline uint64_t asuint64(double f) { return std::bit_cast<uint64_t>(f); }
inline double asdouble(uint64_t i) { return std::bit_cast<double>(i); }

inline void extract_words(uint32_t& hi, uint32_t& lo, double d) {
    const uint64_t u = asuint64(d);
    hi = static_cast<uint32_t>(u >> 32);
    lo = static_cast<uint32_t>(u);
}
inline uint32_t get_high_word(double d) { return static_cast<uint32_t>(asuint64(d) >> 32); }
inline uint32_t get_low_word(double d) { return static_cast<uint32_t>(asuint64(d)); }
inline double insert_words(uint32_t hi, uint32_t lo) {
    return asdouble((static_cast<uint64_t>(hi) << 32) | lo);
}
// Returns d with its low 32 bits replaced by lo (musl's SET_LOW_WORD(d,lo)).
inline double set_low_word(double d, uint32_t lo) {
    return insert_words(static_cast<uint32_t>(asuint64(d) >> 32), lo);
}

// ---- fp barriers (replaces musl's fp_barrier/fp_force_eval/eval_as_double) ----
// volatile round-trips, exactly as musl implements them: not a compiler
// builtin, just prevents the optimizer from re-associating/removing an
// operation whose only purpose is a rounding/exception side effect.

inline double eval_as_double(double x) { volatile double y = x; return y; }
inline double fp_barrier(double x) { volatile double y = x; return y; }
inline void fp_force_eval(double x) { volatile double y = x; (void)y; }

// musl's predict_true/predict_false (__builtin_expect) are a perf hint
// only, never a correctness one; the port drops the hint rather than reach
// for a compiler builtin (PLAN.md P1.5a Step 1 rule 2).
constexpr bool predict_true(bool x) { return x; }
constexpr bool predict_false(bool x) { return x; }

// ---- exact bit ops (see file comment: same category as sqrt) ----
inline double db_fabs(double x) { return std::fabs(x); }
inline double db_scalbn(double x, int n) { return std::scalbn(x, n); }
inline double db_floor(double x) { return std::floor(x); }
inline double db_copysign(double x, double y) { return std::copysign(x, y); }

// ---- musl's __math_xflow/__math_oflow/__math_uflow/__math_divzero/__math_invalid
// (src/math/__math_*.c) - overflow/underflow/divide-by-zero/invalid result
// helpers used by exp/log/pow's special cases. ----

inline double math_xflow(uint32_t sign, double y) {
    return eval_as_double(fp_barrier(sign ? -y : y) * y);
}
inline double math_oflow(uint32_t sign) { return math_xflow(sign, 0x1p769); }
inline double math_uflow(uint32_t sign) { return math_xflow(sign, 0x1p-767); }
inline double math_divzero(uint32_t sign) {
    // Deliberate division by zero (produces +-inf, matching musl's
    // __math_divzero.c): route the divisor through fp_barrier so cl's
    // (correct, but here unwanted - this is not a bug) C4723 "possible
    // division by 0" doesn't fire on a literal `/ 0.0`.
    return fp_barrier(sign ? -1.0 : 1.0) / fp_barrier(0.0);
}
inline double math_invalid(double x) { return (x - x) / (x - x); }

// musl WANT_SNAN=0 default (src/internal/libm.h): signalling NaNs are not
// distinguished from quiet ones.
constexpr bool issignaling_inline(double) { return false; }

// musl WANT_ROUNDING=1 default.
constexpr bool want_rounding = true;

} // namespace ps::math::detmath
