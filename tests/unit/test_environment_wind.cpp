// tests/unit/test_environment_wind.cpp - weather plan W4, library level (no ps_core): the
// gust synthesis (ps/environment/wind.h) - Kaimal spectrum per octave, the u/v/w
// standard-deviation ratios, the 3-s/10-min gust factor against the cited open-land
// value, the frozen-turbulence (Taylor) shift, purity/determinism - and the
// Environment's sample_air (atmosphere lift, mean wind vector, gust at a position).
// Tags [environment][wind].
//
// Spectrum reference [C IEC 61400-1 Kaimal model via a secondary source; Kaimal et al.
// 1972]: S_k(f) = sigma_k^2 (4 L_k / U) / (1 + 6 f L_k / U)^(5/3), L_u/L_v/L_w = 8.1/2.7/0.66 * 0.7 z,
// sigma_v/sigma_w = 0.8/0.5 sigma_u. The test integrates this formula numerically (Simpson on a
// log grid) per octave and compares it with the per-octave power of a SYNTHESISED record
// measured by an independent route (Hann-windowed FFT of the time series), not with the
// closed-form band energy the amplitudes are built from.
//
// RECORDED SABOTAGES (performed by hand, reverted):
//  - W4-W1 (frozen-turbulence shift dropped, wind.cpp tau = t_gust): "Taylor shift: a point downwind
//    sees the field delayed" failed (|gust(x = 400 m, t) - gust(0, t - 400/U)| up to 1.2 m/s instead of
//    < 1e-12), the cross-correlation check failed (peak lag 0, not 160 samples) and "two cars see a
//    coherent gust" failed with it.
//  - W4-W2 (wrong sigma ratio, kGustSigmaRatio {1, 0.8, 0.5} -> {1, 1.0, 0.5}): the v octaves of "Kaimal
//    power per octave" failed (measured/theory 1.5625 = 1.25^2 in every octave, bound 0.85-1.15) and
//    "standard deviations u : v : w" failed (sigma_v/sigma_u 1.009 vs 0.8). The test uses literal
//    ratios and length factors, not the library constants (with the constants a changed value
//    passed the octave check - found by this very sabotage).
//  - W4-W3 (variance in the band, a = sqrt(sigma^2 (C_prev - C_next)) without the factor 2): the octaves
//    read 0.5 and failed, "sigma_u / U" failed (0.105 vs 0.151) and so did the gust factor test (label
//    1.35 measured 1.244, label 1.66 measured 1.461).
//  - W4-W4 (crossfade weights, w_new = smoothstep, w_prev = 1 - w_new): the variance-preserving check
//    failed at mid-blend (w_new^2 + w_prev^2 = 0.5 instead of 1; 0.616 and 0.946 at other points).

#include "environment_fixtures.h"

#include "ps/environment/atmosphere.h"
#include "ps/environment/climate_io.h"
#include "ps/environment/environment.h"
#include "ps/environment/weather_io.h"
#include "ps/environment/wind.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::environment;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
namespace fs = std::filesystem;

namespace {

fs::path data(const std::string& rel) { return fs::path(PS_SOURCE_DIR) / "data" / rel; }

constexpr real kPi = 3.14159265358979323846;

// One-window, no-blend description (a single long record sees one phase set).
GustDesc long_window_desc() {
    GustDesc d;
    d.window_s = 86400.0;
    d.blend_s = 0.0;
    return d;
}

struct Record {
    std::vector<real> u, v, w;
};

// A record sampled at fs for n samples at one point, mean wind along +x.
// Lines at or above max_hz are dropped (a record sampled at fs cannot hold them: they would alias into
// the measured octaves).
Record synthesise(const GustField& f, real mean_speed, real sigma_u, real fs, std::size_t n,
                  real max_hz = 1e30) {
    GustField::Amplitudes amps;
    f.amplitudes(mean_speed, sigma_u, amps);
    for (int axis = 0; axis < GustField::kAxes; ++axis)
        for (int k = 0; k < GustField::kComponents; ++k)
            if (f.line_hz(axis, k) >= max_hz) amps.a[axis][k] = 0.0;
    GustField::Phases ph;
    f.fill_phases(0, ph);
    Record r;
    r.u.resize(n);
    r.v.resize(n);
    r.w.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 g = f.evaluate(amps, ph, 1.0, 0.0, mean_speed, static_cast<real>(i) / fs, 0.0, 0.0);
        r.u[i] = g.x;
        r.v[i] = g.y;
        r.w[i] = g.z;
    }
    return r;
}

void fft(std::vector<std::complex<real>>& a) {
    const std::size_t n = a.size();
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (std::size_t len = 2; len <= n; len <<= 1) {
        const real ang = -2.0 * kPi / static_cast<real>(len);
        const std::complex<real> wl(std::cos(ang), std::sin(ang));
        for (std::size_t i = 0; i < n; i += len) {
            std::complex<real> w(1.0, 0.0);
            for (std::size_t k = 0; k < len / 2; ++k) {
                const std::complex<real> x = a[i + k], y = a[i + k + len / 2] * w;
                a[i + k] = x + y;
                a[i + k + len / 2] = x - y;
                w *= wl;
            }
        }
    }
}

// Power [m2/s2] of a record inside [f_lo, f_hi): Hann-windowed FFT, one-sided, Parseval-normalised
// so that a sinusoid of amplitude a contributes a^2/2 however its leakage spreads over bins.
real band_power(const std::vector<real>& x, real fs, real f_lo, real f_hi) {
    const std::size_t n = x.size();
    std::vector<std::complex<real>> a(n);
    real sw2 = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const real w = 0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<real>(i) / static_cast<real>(n));
        a[i] = x[i] * w;
        sw2 += w * w;
    }
    fft(a);
    const real df = fs / static_cast<real>(n);
    const std::size_t k0 = static_cast<std::size_t>(std::ceil(f_lo / df));
    const std::size_t k1 = static_cast<std::size_t>(std::ceil(f_hi / df));
    real sum = 0.0;
    for (std::size_t k = k0; k < k1 && k < n / 2; ++k) sum += std::norm(a[k]);
    return 2.0 * sum / (static_cast<real>(n) * sw2);
}

real variance(const std::vector<real>& x) {
    real m = 0.0;
    for (real v : x) m += v;
    m /= static_cast<real>(x.size());
    real s = 0.0;
    for (real v : x) s += (v - m) * (v - m);
    return s / static_cast<real>(x.size());
}

// Integral of the one-sided Kaimal spectrum over [f_lo, f_hi] (Simpson on a log grid).
real kaimal_band(real sigma, real length_m, real mean_speed, real f_lo, real f_hi) {
    constexpr int kPanels = 4000;
    const real a = std::log(f_lo), b = std::log(f_hi);
    const real h = (b - a) / kPanels;
    const auto integrand = [&](real lf) {  // S(f) * f for d(ln f)
        const real f = std::exp(lf);
        return f * sigma * sigma * (4.0 * length_m / mean_speed) / std::pow(1.0 + 6.0 * f * length_m / mean_speed, 5.0 / 3.0);
    };
    real s = integrand(a) + integrand(b);
    for (int i = 1; i < kPanels; ++i) s += integrand(a + h * i) * (i % 2 ? 4.0 : 2.0);
    return s * h / 3.0;
}

real kaimal_length(int axis, const GustDesc& d) {
    // Literals (IEC 61400-1 values), NOT the library constants: a changed constant must fail here.
    constexpr real kLengthFactor[3] = {8.1, 2.7, 0.66};
    return kLengthFactor[axis] * 0.7 * d.reference_height_m;
}

} // namespace

TEST_CASE("Gusts: Kaimal power per octave (u, v, w)", "[environment][wind]") {
    const GustDesc desc = long_window_desc();
    const GustField f(desc, 0xC0FFEEull);
    const real U = 10.0;
    const real sigma_u = U * gust_sigma_ratio(desc, desc.reference_gust_factor);
    constexpr real fs = 16.0;
    constexpr std::size_t n = 131072;  // 8192 s
    const Record r = synthesise(f, U, sigma_u, fs, n, 0.9 * fs / 2.0);
    const std::vector<real>* series[3] = {&r.u, &r.v, &r.w};
    const char* names[3] = {"u", "v", "w"};
    int checked = 0;
    for (int axis = 0; axis < 3; ++axis) {
        constexpr real kSigmaRatio[3] = {1.0, 0.8, 0.5};   // literals, not the library constants
        const real sigma = kSigmaRatio[axis] * sigma_u;
        const real L = kaimal_length(axis, desc);
        // Octaves j = 0..10: [f_min 2^j, f_min 2^(j+1)), the last upper edge 3.4 Hz < Nyquist 8 Hz.
        for (int j = 0; j <= 10; ++j) {
            const real lo = desc.f_min_hz * std::pow(2.0, j), hi = 2.0 * lo;
            const real theory = kaimal_band(sigma, L, U, lo, hi);
            const real measured = band_power(*series[axis], fs, lo, hi);
            INFO(names[axis] << " octave " << j << " [" << lo << ", " << hi << ") Hz: measured " << measured
                              << " theory " << theory << " ratio " << measured / theory);
            CHECK(measured / theory > 0.85);
            CHECK(measured / theory < 1.15);
            ++checked;
        }
    }
    CHECK(checked == 33);
}

TEST_CASE("Gusts: standard deviations u : v : w and sigma_u / U", "[environment][wind]") {
    const GustDesc desc = long_window_desc();
    const GustField f(desc, 7);
    constexpr real fs = 16.0;
    constexpr std::size_t n = 131072;
    const real I = gust_base_intensity(desc);
    CHECK_THAT(I, WithinRel(2.2 * 0.4 / std::log(10.0 / 0.03), 1e-12));  // WMO eq. 5.A.1, c_u 2.2, kappa 0.4

    // At the reference gust factor sigma_u / U = I (0.1513 at z = 10 m, z0 = 0.03 m).
    const real U = 10.0;
    const real target = U * gust_sigma_ratio(desc, desc.reference_gust_factor);
    CHECK_THAT(target / U, WithinRel(I, 1e-12));
    const Record r = synthesise(f, U, target, fs, n);
    const real su = std::sqrt(variance(r.u)), sv = std::sqrt(variance(r.v)), sw = std::sqrt(variance(r.w));
    INFO("sigma_u " << su << " (target " << target << ") sigma_v " << sv << " sigma_w " << sw);
    CHECK_THAT(su / U, WithinRel(I, 0.05));          // the omitted bands (< f_min, > 109 Hz) cost ~2 % at 10 m/s
    CHECK_THAT(sv / su, WithinRel(0.8, 0.05));
    CHECK_THAT(sw / su, WithinRel(0.5, 0.05));

    // A gust factor of 1 means no turbulence; the ratio scales linearly in (G - 1).
    CHECK(gust_sigma_ratio(desc, 1.0) == 0.0);
    CHECK_THAT(gust_sigma_ratio(desc, 1.175), WithinRel(0.5 * I, 1e-12));
}

namespace {

struct GustFactorStats {
    real mean = 0.0, sd = 0.0;
};

// Ensemble of independent 10-minute records (different seeds, one 600 s window each, sampled at 8 Hz)
// at mean speed U with the along-wind sigma that the PUBLISHED gust factor `label_g` asks for:
// the 3-s gust factor of a record = max(3-s moving mean of U + u') / mean(U + u').
GustFactorStats gust_factor_ensemble(real U, real label_g, int records) {
    GustDesc desc;                          // window 600 s: one record = one window
    desc.blend_s = 0.0;
    const real sigma_u = U * gust_sigma_ratio(desc, label_g);
    constexpr real fs = 8.0;
    constexpr std::size_t n = 4800;         // 600 s
    constexpr std::size_t win = 24;         // 3 s
    real sum_g = 0.0, sum_g2 = 0.0;
    for (int s = 0; s < records; ++s) {
        const GustField f(desc, 1000 + static_cast<std::uint64_t>(s));
        const Record r = synthesise(f, U, sigma_u, fs, n);
        std::vector<real> x(n);
        real mean = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            x[i] = U + r.u[i];
            mean += x[i];
        }
        mean /= static_cast<real>(n);
        real run = 0.0, best = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            run += x[i];
            if (i >= win) run -= x[i - win];
            if (i + 1 >= win) best = std::max(best, run / static_cast<real>(win));
        }
        const real g = best / mean;
        sum_g += g;
        sum_g2 += g * g;
    }
    GustFactorStats st;
    st.mean = sum_g / records;
    st.sd = std::sqrt(std::max(0.0, sum_g2 / records - st.mean * st.mean));
    return st;
}

} // namespace

TEST_CASE("Gusts: 3-s gust factor over 10 minutes, open land", "[environment][wind]") {
    // The WMO intensity I = c_u kappa / ln(z / z0) = 0.1515 (z 10 m, z0 0.03 m) at U = 10 m/s is the
    // reference_gust_factor (default 1.35) label: the field MEASURES a 3-s/10-min factor of 1.346
    // there. The plan's cited open-land value 1.66 is a different number for a Gaussian field: it needs
    // sigma_u/U = I (1.66 - 1)/(1.35 - 1) = 0.286, and the label 1.66 must then MEASURE 1.66.
    const GustDesc desc;
    const GustFactorStats ref = gust_factor_ensemble(10.0, desc.reference_gust_factor, 200);
    const GustFactorStats cited = gust_factor_ensemble(10.0, 1.66, 200);
    const GustFactorStats calm = gust_factor_ensemble(10.0, 1.0, 5);
    const GustFactorStats slow = gust_factor_ensemble(5.0, 1.35, 100);
    const GustFactorStats fast = gust_factor_ensemble(20.0, 1.35, 100);
    std::printf("[W4] 3-s/10-min gust factor, I(z 10 m, z0 0.03 m) = %.4f:\n"
                "  U 10, label 1.35: mean %.4f sd %.4f (200 records)\n"
                "  U 10, label 1.66: mean %.4f sd %.4f (200 records)\n"
                "  U  5, label 1.35: mean %.4f sd %.4f (100 records)\n"
                "  U 20, label 1.35: mean %.4f sd %.4f (100 records)\n",
                gust_base_intensity(desc), ref.mean, ref.sd, cited.mean, cited.sd, slow.mean, slow.sd, fast.mean, fast.sd);
    CHECK_THAT(ref.mean, WithinRel(desc.reference_gust_factor, 0.05));   // the calibration the default stands for
    CHECK_THAT(cited.mean, WithinRel(1.66, 0.10));                       // the plan's cited open-land value as a label
    CHECK_THAT(calm.mean, WithinRel(1.0, 1e-12));                        // no turbulence, no gust
    // The same turbulence averaged over a path of 3 s x U: the factor falls with U (documented limit of
    // one calibration constant), within 10 % of the label across 5..20 m/s.
    CHECK(slow.mean > ref.mean);
    CHECK(fast.mean < ref.mean);
    CHECK_THAT(slow.mean, WithinRel(1.35, 0.10));
    CHECK_THAT(fast.mean, WithinRel(1.35, 0.10));
}

TEST_CASE("Gusts: Taylor shift - a point downwind sees the field delayed", "[environment][wind]") {
    const GustDesc desc = long_window_desc();
    const GustField f(desc, 99);
    const real U = 10.0, sigma_u = 1.5;
    GustField::Amplitudes amps;
    f.amplitudes(U, sigma_u, amps);
    GustField::Phases ph;
    f.fill_phases(0, ph);

    // Wind toward (0.6, 0.8): a point d metres downwind sees the origin's field U-delayed by d / U.
    const real dx = 0.6, dy = 0.8;
    for (const real d : {30.0, 120.0, 400.0}) {
        for (const real t : {50.0, 333.3, 1234.5}) {
            const Vec3 downwind = f.evaluate(amps, ph, dx, dy, U, t, d * dx, d * dy);
            const Vec3 delayed = f.evaluate(amps, ph, dx, dy, U, t - d / U, 0.0, 0.0);
            INFO("d " << d << " t " << t);
            CHECK_THAT(downwind.x, WithinAbs(delayed.x, 1e-12));
            CHECK_THAT(downwind.y, WithinAbs(delayed.y, 1e-12));
            CHECK_THAT(downwind.z, WithinAbs(delayed.z, 1e-12));
        }
    }
    // Across the wind the frozen pattern has no phase difference (a documented limit: no lateral
    // decorrelation in the Taylor model).
    const Vec3 a = f.evaluate(amps, ph, dx, dy, U, 77.0, 0.0, 0.0);
    const Vec3 b = f.evaluate(amps, ph, dx, dy, U, 77.0, -0.8 * 55.0, 0.6 * 55.0);
    CHECK_THAT(a.x, WithinAbs(b.x, 1e-12));
    CHECK_THAT(a.y, WithinAbs(b.y, 1e-12));

    // Cross-correlation of two points 400 m apart along the wind peaks at lag d / U = 40 s with
    // coefficient ~1 (and lag 0 is not the maximum).
    constexpr real fs = 4.0;
    constexpr int n = 4000;
    std::vector<real> x0(n), x1(n);
    for (int i = 0; i < n; ++i) {
        const real t = 100.0 + static_cast<real>(i) / fs;
        x0[i] = f.evaluate(amps, ph, 1.0, 0.0, U, t, 0.0, 0.0).x;
        x1[i] = f.evaluate(amps, ph, 1.0, 0.0, U, t, 400.0, 0.0).x;
    }
    const auto corr = [&](int lag_samples) {  // x1[i + lag] against x0[i]
        real sxy = 0.0, sxx = 0.0, syy = 0.0;
        // Both x0[i] and x1[i + lag] must exist: i in [max(0, -lag), min(n, n - lag)). The old bound
        // (i + lag < n, i >= 0) read x0[i] past its end for every negative lag (ASan/TSan/debug abort).
        for (int i = std::max(0, -lag_samples); i < std::min(n, n - lag_samples); ++i) {
            const real a0 = x0[i], b0 = x1[i + lag_samples];
            sxy += a0 * b0;
            sxx += a0 * a0;
            syy += b0 * b0;
        }
        return sxy / std::sqrt(sxx * syy);
    };
    // x1(t) = x0(t - 40 s): x1 at index i + 160 equals x0 at index i.
    int best_lag = 0;
    real best = -2.0;
    for (int lag = -300; lag <= 300; ++lag) {
        const real c = corr(lag);
        if (c > best) { best = c; best_lag = lag; }
    }
    INFO("peak lag " << best_lag << ", coefficient " << best << ", lag-0 coefficient " << corr(0));
    CHECK(best_lag == 160);
    CHECK(best > 0.999);
    CHECK(corr(0) < best - 1e-3);
}

TEST_CASE("Gusts: purity, determinism and window crossfade", "[environment][wind]") {
    GustDesc desc;
    const GustField f1(desc, 5), f2(desc, 5), f3(desc, 6);
    GustField::Amplitudes a1, a2, a3;
    f1.amplitudes(8.0, 1.0, a1);
    f2.amplitudes(8.0, 1.0, a2);
    f3.amplitudes(8.0, 1.0, a3);
    GustField::Phases p1, p2, p3, p1b;
    f1.fill_phases(3, p1);
    f2.fill_phases(3, p2);
    f3.fill_phases(3, p3);
    f1.fill_phases(4, p1b);
    const Vec3 g1 = f1.evaluate(a1, p1, 1.0, 0.0, 8.0, 1234.5, 10.0, -4.0);
    const Vec3 g2 = f2.evaluate(a2, p2, 1.0, 0.0, 8.0, 1234.5, 10.0, -4.0);
    const Vec3 g1_again = f1.evaluate(a1, p1, 1.0, 0.0, 8.0, 1234.5, 10.0, -4.0);
    CHECK(g1.x == g2.x);
    CHECK(g1.y == g2.y);
    CHECK(g1.z == g2.z);
    CHECK(g1.x == g1_again.x);  // pure: no state moves
    CHECK(f3.evaluate(a3, p3, 1.0, 0.0, 8.0, 1234.5, 10.0, -4.0).x != g1.x);        // another seed
    CHECK(f1.evaluate(a1, p1b, 1.0, 0.0, 8.0, 1234.5, 10.0, -4.0).x != g1.x);       // another window

    // sigma 0 gives exactly nothing.
    GustField::Amplitudes zero;
    f1.amplitudes(8.0, 0.0, zero);
    const Vec3 gz = f1.evaluate(zero, p1, 1.0, 0.0, 8.0, 99.0, 0.0, 0.0);
    CHECK(gz.x == 0.0);
    CHECK(gz.y == 0.0);
    CHECK(gz.z == 0.0);

    // Frequencies: 32 log-spaced bands from f_min; every line strictly inside its band.
    CHECK(f1.band_edge_hz(0) == desc.f_min_hz);
    for (int axis = 0; axis < 3; ++axis) {
        for (int k = 0; k < GustField::kComponents; ++k) {
            const real lo = f1.band_edge_hz(k), hi = f1.band_edge_hz(k + 1);
            const real fr = f1.line_hz(axis, k);
            CHECK(fr > lo + 0.24 * (hi - lo));
            CHECK(fr < lo + 0.76 * (hi - lo));
        }
    }
    CHECK_THAT(f1.band_edge_hz(2) / f1.band_edge_hz(0), WithinRel(2.0, 1e-12));  // 2 bands per octave

    // Crossfade weights: window index, variance-preserving cos/sin blend over blend_s.
    CHECK(GustField::window_index(0.0, 600.0) == 0);
    CHECK(GustField::window_index(599.999, 600.0) == 0);
    CHECK(GustField::window_index(600.0, 600.0) == 1);
    CHECK(GustField::window_index(-0.5, 600.0) == -1);
    for (const real t : {600.0, 603.0, 610.0, 615.0, 629.9, 700.0}) {
        const GustField::Blend b = f1.blend(t);
        INFO("t " << t);
        CHECK(b.window == 1);
        CHECK_THAT(b.w_new * b.w_new + b.w_prev * b.w_prev, WithinAbs(1.0, 1e-12));
    }
    CHECK(f1.blend(600.0).w_new == 0.0);          // boundary: the old window carries fully
    CHECK(f1.blend(600.0).w_prev == 1.0);
    CHECK(f1.blend(630.0).w_new == 1.0);          // blend_s over: the new window alone
    CHECK(f1.blend(630.0).w_prev == 0.0);
    const GustField::Blend mid = f1.blend(615.0);  // smoothstep(0.5) = 0.5 -> sin(pi/4)
    CHECK_THAT(mid.w_new, WithinRel(std::sqrt(0.5), 1e-12));
    CHECK_THAT(mid.w_prev, WithinRel(std::sqrt(0.5), 1e-12));
}

TEST_CASE("Gusts: rejects an invalid description", "[environment][wind]") {
    GustDesc d;
    d.roughness_length_m = 0.0;
    CHECK_THROWS_AS(GustField(d, 1), std::invalid_argument);
    d = GustDesc{};
    d.bands_per_octave = 1;
    CHECK_THROWS_AS(GustField(d, 1), std::invalid_argument);
    d = GustDesc{};
    d.reference_gust_factor = 1.0;
    CHECK_THROWS_AS(GustField(d, 1), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Environment::sample_air
// ---------------------------------------------------------------------------

namespace {

ClimateDesc gust_climate() { return io::load_climate_json(data("climates/test_weather_gusts.json")); }

// A preset with a given wind, everything else benign: wind FROM `from_deg`.
WeatherPresetDesc wind_preset(real speed, real from_deg, real gust_factor) {
    WeatherPresetDesc p = io::load_weather_preset_json(data("weather_presets/clear.json"));
    p.wind_mean_m_s = speed;
    p.wind_dir_deg = from_deg;
    p.gust_factor = gust_factor;
    return p;
}

EnvironmentConfig preset_config(const WeatherPresetDesc& p, std::uint64_t seed = 1) {
    EnvironmentConfig c;
    c.weather = EnvironmentWeather::Preset;
    c.weather_preset = p;
    c.seed = seed;
    c.fixed_season = "summer";
    c.start = EnvironmentStart{0, "summer", 5, 43200.0};
    return c;
}

} // namespace

TEST_CASE("Environment air: atmosphere lift, mean wind vector and density", "[environment][wind][atmosphere]") {
    Environment env(gust_climate(), preset_config(wind_preset(8.0, 270.0, 1.0)));  // westerly 8 m/s, no gusts (G = 1)
    REQUIRE(env.provides_atmosphere());
    env.attach(0, 240.0);
    const EnvironmentSnapshot& snap = env.current();
    REQUIRE(snap.has_weather);
    CHECK(snap.wind_mean_m_s == 8.0);
    CHECK(snap.wind_dir_deg == 270.0);

    const VehicleAir a0 = env.sample_air(Vec3{0, 0, 0});
    REQUIRE(a0.valid);
    // From the west = blowing toward +x (east), north = +y, no vertical mean.
    CHECK_THAT(a0.mean_wind_world.x, WithinAbs(8.0, 1e-9));
    CHECK_THAT(a0.mean_wind_world.y, WithinAbs(0.0, 1e-9));
    CHECK(a0.mean_wind_world.z == 0.0);
    CHECK(a0.gust_world.x == 0.0);  // gust factor 1: steady wind
    CHECK(a0.wind_world.x == a0.mean_wind_world.x);

    // At the station (z 0) the air is the weather's own: temperature and (reduced) pressure.
    CHECK_THAT(a0.temperature_k, WithinAbs(snap.air_temp_c + 273.15, 1e-9));
    CHECK(a0.altitude_m == 150.0);                                            // location.elevation_m of the climate
    CHECK(a0.vapour_pressure_pa > 0.0);
    CHECK_THAT(a0.density_kg_m3, WithinRel(humid_air_density(a0.pressure_pa, a0.temperature_k, a0.vapour_pressure_pa), 1e-12));

    // The station pressure follows ISA from the sea-level pressure the weather publishes.
    const real t0 = a0.temperature_k + 0.0065 * 150.0;
    CHECK_THAT(a0.pressure_pa, WithinRel(isa_pressure_pa(snap.pressure_msl_hpa * 100.0, t0, 150.0), 1e-9));

    // 1000 m higher: ISA lift, density drops, vapour pressure follows the mixing ratio.
    const VehicleAir a1 = env.sample_air(Vec3{500.0, -200.0, 1000.0});
    CHECK_THAT(a1.temperature_k, WithinAbs(a0.temperature_k - 6.5, 1e-9));
    CHECK(a1.pressure_pa < a0.pressure_pa * 0.9);
    CHECK(a1.density_kg_m3 < a0.density_kg_m3 * 0.93);
    CHECK_THAT(a1.vapour_pressure_pa / a1.pressure_pa, WithinRel(a0.vapour_pressure_pa / a0.pressure_pa, 1e-6));

    // The world north yaw rotates the compass: 90 deg ccw puts north on -x, so a southerly
    // (from 180, toward north) blows toward -x.
    EnvironmentConfig cfg = preset_config(wind_preset(5.0, 180.0, 1.0));
    cfg.north_yaw_deg = 90.0;
    Environment rot(gust_climate(), cfg);
    rot.attach(0, 240.0);
    const VehicleAir r = rot.sample_air(Vec3{});
    CHECK_THAT(r.mean_wind_world.x, WithinAbs(-5.0, 1e-9));
    CHECK_THAT(r.mean_wind_world.y, WithinAbs(0.0, 1e-9));
}

TEST_CASE("Environment air: no weather, no atmosphere", "[environment][wind]") {
    EnvironmentConfig cfg;
    cfg.fixed_season = "summer";
    Environment env(gust_climate(), cfg);  // weather Off
    env.attach(0, 240.0);
    CHECK_FALSE(env.provides_atmosphere());
    CHECK_FALSE(env.current().has_weather);
    const VehicleAir a = env.sample_air(Vec3{0, 0, 0});
    CHECK_FALSE(a.valid);
    CHECK(env.current().cloud_cover == 0.0);   // the W2 snapshot values are untouched
}

TEST_CASE("Environment air: weather needs the climate blocks", "[environment][wind]") {
    EnvironmentConfig cfg;
    cfg.weather = EnvironmentWeather::Dynamic;
    CHECK_THROWS_AS(Environment(test_env::astronomical_four_seasons(), cfg), std::invalid_argument);  // no regimes / humidity ...
    EnvironmentConfig bad;
    bad.altitude_offset_m = std::nan("");
    CHECK_THROWS_AS(Environment(gust_climate(), bad), std::invalid_argument);
}

TEST_CASE("Environment air: two cars see a coherent gust", "[environment][wind]") {
    Environment env(gust_climate(), preset_config(wind_preset(10.0, 270.0, 1.66), 42));
    env.attach(0, 240.0);
    REQUIRE(env.gusts() != nullptr);
    const real U = 10.0;
    // Car A at the origin; car B 120 m downwind (+x). At tick t car B sees what car A saw 12 s earlier.
    // The weather is a preset (constant), so the advection speed is exactly U.
    const std::int64_t t_now = 240 * 100;       // 100 s
    const std::int64_t t_then = t_now - 240 * 12;
    env.begin_tick(t_then);
    const Vec3 a_then = env.sample_air(Vec3{0, 0, 0}).gust_world;
    env.begin_tick(t_now);
    const Vec3 b_now = env.sample_air(Vec3{120.0, 0, 0}).gust_world;
    CHECK(a_then.x != 0.0);
    CHECK_THAT(b_now.x, WithinAbs(a_then.x, 1e-9));
    CHECK_THAT(b_now.y, WithinAbs(a_then.y, 1e-9));
    CHECK_THAT(b_now.z, WithinAbs(a_then.z, 1e-9));
    // and it is the gust the car sees on top of the mean: wind = mean + gust.
    const VehicleAir v = env.sample_air(Vec3{120.0, 0, 0});
    CHECK_THAT(v.wind_world.x, WithinAbs(U + v.gust_world.x, 1e-12));
    // The standard deviation scales with U and (G - 1): a different seed gives a different field.
    Environment other(gust_climate(), preset_config(wind_preset(10.0, 270.0, 1.66), 43));
    other.attach(0, 240.0);
    other.begin_tick(t_now);
    CHECK(other.sample_air(Vec3{0, 0, 0}).gust_world.x != env.sample_air(Vec3{0, 0, 0}).gust_world.x);
    (void)U;
}

TEST_CASE("Environment air: gusts run on the sim clock, weather on environment time", "[environment][wind]") {
    // Scale 0 freezes environment time (the weather) but the gusts still move with the sim clock.
    EnvironmentConfig cfg = preset_config(wind_preset(10.0, 270.0, 1.66), 3);
    cfg.time_scale = 0.0;
    Environment env(gust_climate(), cfg);
    env.attach(0, 240.0);
    env.begin_tick(240 * 20);
    const VehicleAir a = env.sample_air(Vec3{});
    env.begin_tick(240 * 21);
    const VehicleAir b = env.sample_air(Vec3{});
    CHECK(a.temperature_k == b.temperature_k);
    CHECK(a.mean_wind_world.x == b.mean_wind_world.x);
    CHECK(a.gust_world.x != b.gust_world.x);
    CHECK(env.gust_time_s() == 21.0);
}

TEST_CASE("Environment air: hash covers the weather only when it is on", "[environment][wind]") {
    EnvironmentConfig off;
    off.fixed_season = "summer";
    Environment e_off(gust_climate(), off);
    e_off.attach(0, 240.0);
    const std::uint64_t h_off = e_off.fold_state_hash(1469598103934665603ULL);

    Environment e1(gust_climate(), preset_config(wind_preset(8.0, 270.0, 1.5), 1));
    Environment e2(gust_climate(), preset_config(wind_preset(8.0, 270.0, 1.5), 2));
    Environment e3(gust_climate(), preset_config(wind_preset(9.0, 270.0, 1.5), 1));
    e1.attach(0, 240.0);
    e2.attach(0, 240.0);
    e3.attach(0, 240.0);
    const std::uint64_t h1 = e1.fold_state_hash(1469598103934665603ULL);
    CHECK(h1 != h_off);
    CHECK(h1 != e2.fold_state_hash(1469598103934665603ULL));   // the seed is hashed
    CHECK(h1 != e3.fold_state_hash(1469598103934665603ULL));   // the preset is hashed

    // The same environment with the weather block removed hashes exactly like a W2 environment: the
    // plain clock fold (nothing else) - checked by two independent constructions agreeing.
    EnvironmentConfig off2;
    off2.fixed_season = "summer";
    Environment e_off2(gust_climate(), off2);
    e_off2.attach(0, 240.0);
    CHECK(e_off2.fold_state_hash(1469598103934665603ULL) == h_off);
}

// Cost of one vehicle's air sample (ISA lift + 3 x 32 gust lines) and of the per-tick environment work
// (begin_tick: weather evaluation + amplitudes), measured, not asserted (hidden: run with "[.][diag]").
TEST_CASE("Environment air: cost per vehicle sample and per tick", "[.][diag][environment][wind]") {
    Environment env(gust_climate(), preset_config(wind_preset(8.0, 270.0, 1.5), 3));
    env.attach(0, 240.0);
    constexpr int kN = 200000;
    real sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kN; ++i) {
        env.begin_tick(i);
        sink += env.current().wind_mean_m_s;
    }
    const auto t1 = std::chrono::steady_clock::now();
    for (int i = 0; i < kN; ++i) sink += env.sample_air(Vec3{static_cast<real>(i % 100), 0.0, 0.5}).gust_world.x;
    const auto t2 = std::chrono::steady_clock::now();
    const double us_tick = std::chrono::duration<double, std::micro>(t1 - t0).count() / kN;
    const double us_air = std::chrono::duration<double, std::micro>(t2 - t1).count() / kN;
    std::printf("[W4] begin_tick %.3f us/tick, sample_air %.3f us/vehicle (sink %.3f)\n", us_tick, us_air, sink);
    CHECK(std::isfinite(sink));
}
