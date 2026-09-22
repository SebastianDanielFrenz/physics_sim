// White-box tests for core/src/math/detmath (internal, PLAN.md D7 / vault
// PHYS-009 Step 1), included via the private include path
// tests/unit/CMakeLists.txt already adds for other white-box tests.
//
// Two independent checks, as required by PLAN.md's D7 amendment:
//   1. Accuracy: ps::math::detmath::X(x) vs the platform libm std::X(x)
//      (the only reference available without a new dependency) over many
//      sampled arguments, including large arguments (sin/cos/tan range
//      reduction), subnormals and specials (+-0, inf, NaN). musl's own
//      accuracy claims for these algorithms are well under 1 ulp from the
//      true value; since ucrtbase and glibc are themselves generally within
//      about 1 ulp of the true value but round differently from each other
//      (that disagreement is PHYS-009's root cause), the ported function
//      can legitimately sit up to about 2 ulp from either single platform's
//      libm while still being correct - so the bound below is 2 ulp, not 1.
//   2. Bit-identity: a small table of golden hex results, run on every CI
//      leg (Windows AND Linux, tools/ci.ps1) - this is what actually
//      catches a future accidental platform divergence (e.g. a stray
//      std:: call slipping back in, or non-portable code in the port).

#include "math/detmath/detmath.h"

#include <catch2/catch_test_macros.hpp>

#include <bit>
#include <cmath>
#include <cstdint>
#include <random>

using namespace ps::math;

namespace {

// Standard monotonic double->int64 mapping (consecutive representable
// doubles map to consecutive integers), so |map(a)-map(b)| is the exact
// ULP distance between a and b, including across the zero crossing.
std::int64_t ordered(double d) {
    std::uint64_t u = std::bit_cast<std::uint64_t>(d);
    if (u & 0x8000000000000000ULL)
        u = ~u;
    else
        u |= 0x8000000000000000ULL;
    return std::bit_cast<std::int64_t>(u);
}

double ulp_distance(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return std::isnan(a) && std::isnan(b) ? 0.0 : 1e300;
    return static_cast<double>(std::llabs(ordered(a) - ordered(b)));
}

constexpr double kUlpBound = 2.0;

} // namespace

TEST_CASE("detmath accuracy vs platform libm - trig", "[math][detmath]") {
    std::mt19937_64 rng(1);
    double worst_sin = 0, worst_cos = 0, worst_tan = 0;

    auto sample = [&](double lo, double hi, int n) {
        std::uniform_real_distribution<double> dist(lo, hi);
        for (int i = 0; i < n; ++i) {
            const double x = dist(rng);
            worst_sin = std::max(worst_sin, ulp_distance(detmath::sin(x), std::sin(x)));
            worst_cos = std::max(worst_cos, ulp_distance(detmath::cos(x), std::cos(x)));
            worst_tan = std::max(worst_tan, ulp_distance(detmath::tan(x), std::tan(x)));
        }
    };
    sample(-4.0, 4.0, 20000);          // no reduction / small-angle fast paths
    sample(-1e3, 1e3, 20000);          // medium reduction (__rem_pio2 fast branches)
    sample(-1e8, 1e8, 20000);          // large reduction
    sample(-1e300, 1e300, 5000);       // __rem_pio2_large path
    sample(-1e-30, 1e-30, 2000);       // tiny / subnormal-adjacent

    CHECK(worst_sin <= kUlpBound);
    CHECK(worst_cos <= kUlpBound);
    CHECK(worst_tan <= kUlpBound);

    // sincos must agree with the separate sin()/cos() bit-for-bit (same
    // kernels, same reduction) - not merely close.
    for (double x : {0.0, 0.7, 1.4, -2.3, 1e6, -1e10, 5e-320}) {
        double s, c;
        detmath::sincos(x, &s, &c);
        CHECK(s == detmath::sin(x));
        CHECK(c == detmath::cos(x));
    }
}

TEST_CASE("detmath accuracy vs platform libm - inverse trig", "[math][detmath]") {
    std::mt19937_64 rng(2);
    double worst_asin = 0, worst_acos = 0, worst_atan = 0, worst_atan2 = 0;

    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    for (int i = 0; i < 40000; ++i) {
        const double x = unit(rng);
        worst_asin = std::max(worst_asin, ulp_distance(detmath::asin(x), std::asin(x)));
        worst_acos = std::max(worst_acos, ulp_distance(detmath::acos(x), std::acos(x)));
    }
    std::uniform_real_distribution<double> wide(-1e6, 1e6);
    for (int i = 0; i < 40000; ++i) {
        const double x = wide(rng);
        worst_atan = std::max(worst_atan, ulp_distance(detmath::atan(x), std::atan(x)));
        const double y = wide(rng);
        worst_atan2 = std::max(worst_atan2, ulp_distance(detmath::atan2(y, x), std::atan2(y, x)));
    }
    CHECK(worst_asin <= kUlpBound);
    CHECK(worst_acos <= kUlpBound);
    CHECK(worst_atan <= kUlpBound);
    CHECK(worst_atan2 <= kUlpBound);
}

TEST_CASE("detmath accuracy vs platform libm - exp/log/pow", "[math][detmath]") {
    std::mt19937_64 rng(3);
    double worst_exp = 0, worst_log = 0, worst_pow = 0;

    std::uniform_real_distribution<double> expdist(-700.0, 700.0);
    for (int i = 0; i < 40000; ++i) {
        const double x = expdist(rng);
        worst_exp = std::max(worst_exp, ulp_distance(detmath::exp(x), std::exp(x)));
    }
    // log-uniform positive samples across a wide range of magnitudes.
    std::uniform_real_distribution<double> logmag(-300.0, 300.0);
    for (int i = 0; i < 40000; ++i) {
        const double x = std::pow(10.0, logmag(rng));
        worst_log = std::max(worst_log, ulp_distance(detmath::log(x), std::log(x)));
    }
    std::uniform_real_distribution<double> base(0.01, 100.0);
    std::uniform_real_distribution<double> expo(-20.0, 20.0);
    for (int i = 0; i < 40000; ++i) {
        const double x = base(rng), y = expo(rng);
        worst_pow = std::max(worst_pow, ulp_distance(detmath::pow(x, y), std::pow(x, y)));
    }
    CHECK(worst_exp <= kUlpBound);
    CHECK(worst_log <= kUlpBound);
    CHECK(worst_pow <= kUlpBound);
}

TEST_CASE("detmath specials", "[math][detmath]") {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double denorm_min = std::numeric_limits<double>::denorm_min(); // smallest positive subnormal
    const double dbl_min = std::numeric_limits<double>::min();          // smallest positive normal

    CHECK(detmath::sin(0.0) == 0.0);
    CHECK(std::signbit(detmath::sin(-0.0)));
    CHECK(detmath::cos(0.0) == 1.0);
    CHECK(std::isnan(detmath::sin(inf)));
    CHECK(std::isnan(detmath::cos(inf)));
    CHECK(std::isnan(detmath::sin(nan)));
    CHECK(detmath::sin(denorm_min) == denorm_min); // sin(x)~=x for tiny x
    CHECK(detmath::cos(denorm_min) == 1.0);
    CHECK(detmath::sin(dbl_min) == dbl_min);

    CHECK(detmath::asin(1.0) > 0.0);
    CHECK(std::isnan(detmath::asin(1.0001)));
    CHECK(std::isnan(detmath::acos(-1.0001)));
    CHECK(detmath::acos(1.0) == 0.0);

    CHECK(detmath::exp(0.0) == 1.0);
    CHECK(detmath::exp(inf) == inf);
    CHECK(detmath::exp(-inf) == 0.0);
    CHECK(std::isnan(detmath::exp(nan)));
    CHECK(detmath::exp(1000.0) == inf); // overflow

    CHECK(detmath::log(1.0) == 0.0);
    CHECK(detmath::log(0.0) == -inf);
    CHECK(std::isnan(detmath::log(-1.0)));
    CHECK(detmath::log(inf) == inf);

    CHECK(detmath::pow(2.0, 10.0) == 1024.0);
    CHECK(detmath::pow(0.0, 0.0) == 1.0);
    CHECK(detmath::pow(1.0, nan) == 1.0);
}

// ---- bit-identity golden table (PLAN.md D7 Step 1 rule 2: "committed
// golden hex results, checked on every CI leg (Windows and Linux must
// produce the same bits)"). Inputs/outputs are the raw IEEE-754 bit
// pattern in hex, produced by this exact port; if a future change (on
// either platform) makes a compiler emit different bits here, this test
// fails on whichever leg diverges. ----

namespace {
std::uint64_t bits(double x) { return std::bit_cast<std::uint64_t>(x); }
double from_bits(std::uint64_t u) { return std::bit_cast<double>(u); }
} // namespace

TEST_CASE("detmath golden bit table", "[math][detmath]") {
    struct Case { std::uint64_t input; std::uint64_t expected; };

    // sin(x)
    for (const Case& c : {
             Case{0x3fe6666666666666ULL, 0x3fe49d6e694619b8ULL}, // sin(0.7)
             Case{0x3ff921fb54442d18ULL, 0x3ff0000000000000ULL}, // sin(pi/2) == 1
             Case{0x4000000000000000ULL, 0x3fed18f6ead1b446ULL}, // sin(2.0)
             Case{0x4130000000000000ULL, 0x3fd526ccb2fc8656ULL}, // sin(1e6), needs __rem_pio2
             Case{0xc130000000000000ULL, 0xbfd526ccb2fc8656ULL}, // sin(-1e6)
             Case{0x4415af1d78b58c40ULL, 0xbfe4a5e605fd6450ULL}, // sin(1e10), __rem_pio2_large
         })
        CHECK(bits(detmath::sin(from_bits(c.input))) == c.expected);

    // cos(x)
    for (const Case& c : {
             Case{0x3fe6666666666666ULL, 0x3fe87996529f9d93ULL}, // cos(0.7)
             Case{0x4000000000000000ULL, 0xbfdaa22657537205ULL}, // cos(2.0)
             Case{0x4415af1d78b58c40ULL, 0x3fe872720fc60d3dULL}, // cos(1e10)
         })
        CHECK(bits(detmath::cos(from_bits(c.input))) == c.expected);

    // tan(x)
    for (const Case& c : {
             Case{0x3fe6666666666666ULL, 0x3feaf406c2fc78aeULL}, // tan(0.7)
             Case{0x4000000000000000ULL, 0xc0017af62e0950f8ULL}, // tan(2.0)
         })
        CHECK(bits(detmath::tan(from_bits(c.input))) == c.expected);

    // asin/acos/atan/atan2
    CHECK(bits(detmath::asin(0.5)) == 0x3fe0c152382d7366ULL);
    CHECK(bits(detmath::acos(0.5)) == 0x3ff0c152382d7366ULL);
    CHECK(bits(detmath::atan(1.5)) == 0x3fef730bd281f69bULL);
    CHECK(bits(detmath::atan2(1.0, 2.0)) == 0x3fddac670561bb4fULL);

    // exp/log/pow
    CHECK(bits(detmath::exp(1.0)) == 0x4005bf0a8b145769ULL);
    CHECK(bits(detmath::exp(-3.5)) == 0x3f9eec1018e4ff66ULL);
    CHECK(bits(detmath::log(2.0)) == 0x3fe62e42fefa39efULL);
    CHECK(bits(detmath::log(1e10)) == 0x4037069e2aa2aa5bULL);
    CHECK(bits(detmath::pow(2.0, 0.5)) == 0x3ff6a09e667f3bcdULL);
    CHECK(bits(detmath::pow(3.7, -2.2)) == 0x3facc9fa1d9340daULL);
}
