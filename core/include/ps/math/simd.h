// ps/math/simd.h — SIMD wrapper skeleton (PLAN.md D5, section 4).
//
// Performance order per D5: correct scalar code -> benchmark -> SoA data
// layout -> AVX2/FMA intrinsics behind these wrappers, with a scalar
// fallback -> assembly (only with a committed benchmark). P1 only needs the
// skeleton below and its scalar-vs-intrinsic equivalence test
// (tests/unit/test_simd.cpp); no hot-path code is routed through it yet.
//
// f64x4 holds 4 lanes of ps::real. PS_SIMD_AVX2 selects the __m256d
// intrinsic backend when the target is built with AVX2 (ps_options: -mavx2
// /arch:AVX2, see CMakeLists.txt); otherwise PS_SIMD_SCALAR is used so the
// same call sites still compile and produce bit-identical-to-scalar results
// on any target. FMA is never used implicitly here (D7: determinism depends
// on never silently fusing a*b+c) - fmadd() below is the only entry point
// that contracts, and only when a caller opts in explicitly.

#pragma once

#include "ps/types.h"

#if defined(__AVX2__) || (defined(_MSC_VER) && defined(__clang__) && defined(__AVX2__))
#define PS_SIMD_AVX2 1
#else
#define PS_SIMD_SCALAR 1
#endif

#if defined(PS_SIMD_AVX2)
#include <immintrin.h>
#endif

#include <array>
#include <cmath>
#include <cstddef>

namespace ps::simd {

// Per-lane comparison result (A9). A lane is "true" when the AVX2 compare left
// all its bits set; the scalar fallback keeps a bool per lane. Only the
// operations the pipe kernel needs: &, |, any(), all(), count(). Never compare
// two Masks for equality.
struct alignas(32) Mask {
#if defined(PS_SIMD_AVX2)
    __m256d m;

    Mask() : m(_mm256_setzero_pd()) {}
    explicit Mask(__m256d val) : m(val) {}

    Mask operator&(const Mask& o) const { return Mask(_mm256_and_pd(m, o.m)); }
    Mask operator|(const Mask& o) const { return Mask(_mm256_or_pd(m, o.m)); }

    bool any() const { return _mm256_movemask_pd(m) != 0; }
    bool all() const { return _mm256_movemask_pd(m) == 0xf; }
    int count() const {
        const int bits = _mm256_movemask_pd(m);
        return ((bits >> 0) & 1) + ((bits >> 1) & 1) + ((bits >> 2) & 1) + ((bits >> 3) & 1);
    }
#else
    std::array<bool, 4> m{false, false, false, false};

    Mask() = default;
    Mask(bool a, bool b, bool c, bool d) : m{a, b, c, d} {}

    Mask operator&(const Mask& o) const { return {m[0] && o.m[0], m[1] && o.m[1], m[2] && o.m[2], m[3] && o.m[3]}; }
    Mask operator|(const Mask& o) const { return {m[0] || o.m[0], m[1] || o.m[1], m[2] || o.m[2], m[3] || o.m[3]}; }

    bool any() const { return m[0] || m[1] || m[2] || m[3]; }
    bool all() const { return m[0] && m[1] && m[2] && m[3]; }
    int count() const { return (m[0] ? 1 : 0) + (m[1] ? 1 : 0) + (m[2] ? 1 : 0) + (m[3] ? 1 : 0); }
#endif
};

// 4 lanes of ps::real (double), 32-byte aligned.
struct alignas(32) f64x4 {
#if defined(PS_SIMD_AVX2)
    __m256d v;

    f64x4() : v(_mm256_setzero_pd()) {}
    explicit f64x4(__m256d val) : v(val) {}
    explicit f64x4(real scalar) : v(_mm256_set1_pd(scalar)) {}
    f64x4(real a, real b, real c, real d) : v(_mm256_set_pd(d, c, b, a)) {}

    static f64x4 load(const real* p) { return f64x4(_mm256_load_pd(p)); }
    static f64x4 loadu(const real* p) { return f64x4(_mm256_loadu_pd(p)); }
    void store(real* p) const { _mm256_store_pd(p, v); }
    void storeu(real* p) const { _mm256_storeu_pd(p, v); }

    real operator[](std::size_t lane) const {
        alignas(32) real tmp[4];
        store(tmp);
        return tmp[lane];
    }

    f64x4 operator+(const f64x4& o) const { return f64x4(_mm256_add_pd(v, o.v)); }
    f64x4 operator-(const f64x4& o) const { return f64x4(_mm256_sub_pd(v, o.v)); }
    f64x4 operator*(const f64x4& o) const { return f64x4(_mm256_mul_pd(v, o.v)); }
    f64x4 operator/(const f64x4& o) const { return f64x4(_mm256_div_pd(v, o.v)); }

    f64x4 sqrt() const { return f64x4(_mm256_sqrt_pd(v)); }
    // Clear the sign bit - exact for every finite value, matching std::fabs.
    f64x4 abs() const { return f64x4(_mm256_andnot_pd(_mm256_set1_pd(-0.0), v)); }

    // Ordered compares (false on NaN, exactly like the scalar operators).
    Mask lt(const f64x4& o) const { return Mask(_mm256_cmp_pd(v, o.v, _CMP_LT_OQ)); }
    Mask le(const f64x4& o) const { return Mask(_mm256_cmp_pd(v, o.v, _CMP_LE_OQ)); }
    Mask gt(const f64x4& o) const { return Mask(_mm256_cmp_pd(v, o.v, _CMP_GT_OQ)); }
    Mask ge(const f64x4& o) const { return Mask(_mm256_cmp_pd(v, o.v, _CMP_GE_OQ)); }
    Mask eq(const f64x4& o) const { return Mask(_mm256_cmp_pd(v, o.v, _CMP_EQ_OQ)); }

    // on_true where m, else on_false, per lane.
    static f64x4 select(const Mask& m, const f64x4& on_true, const f64x4& on_false) {
        return f64x4(_mm256_blendv_pd(on_false.v, on_true.v, m.m));
    }

    real hsum() const {
        alignas(32) real tmp[4];
        store(tmp);
        return (tmp[0] + tmp[1]) + (tmp[2] + tmp[3]);
    }

    // Explicit fused multiply-add. Only used where a caller has decided
    // contraction is acceptable for that computation (D7) - never implicit.
    static f64x4 fmadd(const f64x4& a, const f64x4& b, const f64x4& c) {
        return f64x4(_mm256_fmadd_pd(a.v, b.v, c.v));
    }
#else
    std::array<real, 4> lanes{0, 0, 0, 0};

    f64x4() = default;
    explicit f64x4(real scalar) : lanes{scalar, scalar, scalar, scalar} {}
    f64x4(real a, real b, real c, real d) : lanes{a, b, c, d} {}

    static f64x4 load(const real* p) { return f64x4(p[0], p[1], p[2], p[3]); }
    static f64x4 loadu(const real* p) { return load(p); }
    void store(real* p) const { p[0] = lanes[0]; p[1] = lanes[1]; p[2] = lanes[2]; p[3] = lanes[3]; }
    void storeu(real* p) const { store(p); }

    real operator[](std::size_t lane) const { return lanes[lane]; }

    f64x4 operator+(const f64x4& o) const { return {lanes[0]+o.lanes[0], lanes[1]+o.lanes[1], lanes[2]+o.lanes[2], lanes[3]+o.lanes[3]}; }
    f64x4 operator-(const f64x4& o) const { return {lanes[0]-o.lanes[0], lanes[1]-o.lanes[1], lanes[2]-o.lanes[2], lanes[3]-o.lanes[3]}; }
    f64x4 operator*(const f64x4& o) const { return {lanes[0]*o.lanes[0], lanes[1]*o.lanes[1], lanes[2]*o.lanes[2], lanes[3]*o.lanes[3]}; }
    f64x4 operator/(const f64x4& o) const { return {lanes[0]/o.lanes[0], lanes[1]/o.lanes[1], lanes[2]/o.lanes[2], lanes[3]/o.lanes[3]}; }

    f64x4 sqrt() const { return {std::sqrt(lanes[0]), std::sqrt(lanes[1]), std::sqrt(lanes[2]), std::sqrt(lanes[3])}; }
    f64x4 abs() const { return {std::fabs(lanes[0]), std::fabs(lanes[1]), std::fabs(lanes[2]), std::fabs(lanes[3])}; }

    Mask lt(const f64x4& o) const { return {lanes[0] < o.lanes[0], lanes[1] < o.lanes[1], lanes[2] < o.lanes[2], lanes[3] < o.lanes[3]}; }
    Mask le(const f64x4& o) const { return {lanes[0] <= o.lanes[0], lanes[1] <= o.lanes[1], lanes[2] <= o.lanes[2], lanes[3] <= o.lanes[3]}; }
    Mask gt(const f64x4& o) const { return {lanes[0] > o.lanes[0], lanes[1] > o.lanes[1], lanes[2] > o.lanes[2], lanes[3] > o.lanes[3]}; }
    Mask ge(const f64x4& o) const { return {lanes[0] >= o.lanes[0], lanes[1] >= o.lanes[1], lanes[2] >= o.lanes[2], lanes[3] >= o.lanes[3]}; }
    Mask eq(const f64x4& o) const { return {lanes[0] == o.lanes[0], lanes[1] == o.lanes[1], lanes[2] == o.lanes[2], lanes[3] == o.lanes[3]}; }

    static f64x4 select(const Mask& m, const f64x4& on_true, const f64x4& on_false) {
        return {m.m[0] ? on_true.lanes[0] : on_false.lanes[0], m.m[1] ? on_true.lanes[1] : on_false.lanes[1],
                m.m[2] ? on_true.lanes[2] : on_false.lanes[2], m.m[3] ? on_true.lanes[3] : on_false.lanes[3]};
    }

    real hsum() const { return (lanes[0] + lanes[1]) + (lanes[2] + lanes[3]); }

    // Scalar reference fallback deliberately does NOT fuse (a*b)+c into a
    // hardware FMA - it must stay the bit-for-bit reference the AVX2 path's
    // explicit fmadd() is allowed to diverge from only within a stated
    // per-call tolerance (D5), never silently.
    static f64x4 fmadd(const f64x4& a, const f64x4& b, const f64x4& c) {
        return {a.lanes[0]*b.lanes[0]+c.lanes[0], a.lanes[1]*b.lanes[1]+c.lanes[1],
                a.lanes[2]*b.lanes[2]+c.lanes[2], a.lanes[3]*b.lanes[3]+c.lanes[3]};
    }
#endif
};

constexpr bool has_avx2_backend() {
#if defined(PS_SIMD_AVX2)
    return true;
#else
    return false;
#endif
}

} // namespace ps::simd
