// tests/unit/test_waveguide_network.cpp - ps_audio A3: the linear digital
// waveguide network (ps/engine_cycle/waveguide_network.h). out/audio_plan.md
// section 7 row V3 (WG part: quarter-wave, half-wave and Helmholtz resonances
// within 1 %), the Kirchhoff wall-loss fit and mode decay, the open-end
// radiation filter against the Levine-Schwinger low-ka limit, lossless
// passivity, zero allocation, determinism and the build-time rejections. Each
// case names the defect it catches; the sabotage runs and their failing
// numbers are recorded in out/audio_a3_progress.md.

#include "ps/engine_cycle/waveguide_network.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/math/transcendental.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::engine_cycle;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHFull = 1.0 / 46080.0;  // the cycle model's h_full, 1/(960*48) s

double rel(double a, double b) { return std::fabs(a - b) / std::fabs(b); }

WgNodeDesc node(WgNodeDesc::Kind k, const char* name, double volume = 0.0) {
    WgNodeDesc n;
    n.kind = k;
    n.name = name;
    n.volume_m3 = volume;
    return n;
}

WgNodeDesc open_node(const char* name, bool radiation) {
    WgNodeDesc n = node(WgNodeDesc::Kind::Open, name);
    n.radiation = radiation;
    return n;
}

WgSegmentDesc seg(const char* name, double length, double dia, int a, int b, EndCorrection ea, EndCorrection eb,
                  WgLoss loss = WgLoss::None, GasStateDesc gas = GasStateDesc{}) {
    WgSegmentDesc s;
    s.name = name;
    s.length_m = length;
    s.diameter_m = dia;
    s.node_a = a;
    s.node_b = b;
    s.end_a = ea;
    s.end_b = eb;
    s.loss = loss;
    s.gas = gas;
    return s;
}

// Hann-windowed DFT magnitude of s (sample period dt) at frequency f.
double dft_mag(const std::vector<double>& s, double dt, double f) {
    double re = 0.0, im = 0.0;
    const double w = 2.0 * kPi * f * dt;
    for (std::size_t n = 0; n < s.size(); ++n) {
        re += s[n] * std::cos(w * static_cast<double>(n));
        im -= s[n] * std::sin(w * static_cast<double>(n));
    }
    return std::sqrt(re * re + im * im);
}

std::vector<double> hann(std::vector<double> s) {
    double mean = 0.0;
    for (double v : s) mean += v;
    mean /= static_cast<double>(s.size());
    const std::size_t n = s.size();
    for (std::size_t k = 0; k < n; ++k) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(k) / static_cast<double>(n - 1));
        s[k] = (s[k] - mean) * w;
    }
    return s;
}

// Spectral peak of the (mean-removed, Hann-windowed) signal inside [f_lo, f_hi]:
// a coarse scan, then golden-section refinement around the best bin.
double peak_frequency(const std::vector<double>& raw, double dt, double f_lo, double f_hi) {
    const std::vector<double> s = hann(raw);
    const int scan = 400;
    double best_f = f_lo, best_m = -1.0;
    const double df = (f_hi - f_lo) / scan;
    for (int k = 0; k <= scan; ++k) {
        const double f = f_lo + df * k;
        const double m = dft_mag(s, dt, f);
        if (m > best_m) {
            best_m = m;
            best_f = f;
        }
    }
    double a = best_f - df, b = best_f + df;
    const double gr = 0.5 * (std::sqrt(5.0) - 1.0);
    double c = b - gr * (b - a), d = a + gr * (b - a);
    double mc = dft_mag(s, dt, c), md = dft_mag(s, dt, d);
    for (int it = 0; it < 60; ++it) {
        if (mc > md) {
            b = d;
            d = c;
            md = mc;
            c = b - gr * (b - a);
            mc = dft_mag(s, dt, c);
        } else {
            a = c;
            c = d;
            mc = md;
            d = a + gr * (b - a);
            md = dft_mag(s, dt, d);
        }
    }
    return 0.5 * (a + b);
}

// Unit volume impulse into `in` at step 0, then record node `probe`'s pressure.
std::vector<double> impulse_record(WaveguideNetwork& net, int in, int probe, int steps) {
    std::vector<double> s;
    s.reserve(static_cast<std::size_t>(steps));
    for (int k = 0; k < steps; ++k) {
        if (k == 0) net.set_node_inflow(in, 1.0e-3);
        net.step();
        s.push_back(net.node_pressure(probe));
    }
    return s;
}

// Sound speed of the default gas model at 293.15 K fresh air, via a 1-segment build.
double segment_c(const WaveguideNetwork& net, int s) { return net.segment_sound_speed(s); }

// A lossless 5-segment network: a 3-way junction of unequal pipes, a
// compliance junction, closed and ideal open ends, two temperatures.
WaveguideNetworkDesc lossless_network() {
    WaveguideNetworkDesc d;
    d.nodes.push_back(node(WgNodeDesc::Kind::Closed, "c0"));                // 0
    d.nodes.push_back(node(WgNodeDesc::Kind::Junction, "j3"));              // 1
    d.nodes.push_back(node(WgNodeDesc::Kind::Closed, "c2"));                // 2
    d.nodes.push_back(node(WgNodeDesc::Kind::Junction, "box", 2.0e-3));     // 3
    d.nodes.push_back(open_node("tail", false));                            // 4
    d.nodes.push_back(node(WgNodeDesc::Kind::Junction, "step"));            // 5
    const GasStateDesc hot{101325.0, 700.0, 1.0};
    d.segments.push_back(seg("s0", 0.31, 0.035, 0, 1, EndCorrection::None, EndCorrection::None));
    d.segments.push_back(seg("s1", 0.47, 0.050, 1, 2, EndCorrection::None, EndCorrection::None));
    d.segments.push_back(seg("s2", 0.23, 0.042, 1, 3, EndCorrection::None, EndCorrection::Flanged, WgLoss::None, hot));
    d.segments.push_back(seg("s3", 0.52, 0.060, 3, 5, EndCorrection::Flanged, EndCorrection::None, WgLoss::None, hot));
    d.segments.push_back(seg("s4", 0.39, 0.030, 5, 4, EndCorrection::None, EndCorrection::Unflanged, WgLoss::None, hot));
    return d;
}

// The same topology with every loss on: Kirchhoff walls and a radiating end.
WaveguideNetworkDesc lossy_network() {
    WaveguideNetworkDesc d = lossless_network();
    d.nodes[4].radiation = true;
    for (WgSegmentDesc& s : d.segments) s.loss = WgLoss::Kirchhoff;
    return d;
}

} // namespace

TEST_CASE("W1 waveguide resonances vs analytic quarter-wave, half-wave and Helmholtz (V3 WG)",
          "[drivetrain][cycle][pipe]") {
    // Catches: a missing or wrong end correction, a delay-line length off by
    // one, a wrong fractional-delay (Thiran) coefficient, a radiation filter
    // whose DC group delay is not compensated, a wrong compliance conductance.
    // Tolerance 1 % (plan V3). Lossless segments (the analytic results are).
    SECTION("closed-open quarter wave, 293 K air, ideal and radiating end") {
        for (int rad = 0; rad < 2; ++rad) {
            WaveguideNetworkDesc d;
            const double length = 0.40, dia = 0.05;
            d.nodes.push_back(node(WgNodeDesc::Kind::Closed, "closed"));
            d.nodes.push_back(open_node("open", rad == 1));
            d.segments.push_back(seg("qw", length, dia, 0, 1, EndCorrection::None, EndCorrection::Unflanged));
            WaveguideNetwork net(d, kHFull);
            const std::vector<double> s = impulse_record(net, 0, 0, 11520);
            const double c = segment_c(net, 0);
            const double f_ex = c / (4.0 * (length + kUnflangedEndCorrection * 0.5 * dia));
            const double f = peak_frequency(s, kHFull, 0.8 * f_ex, 1.2 * f_ex);
            std::printf("W1 closed-open (%s): f %.3f Hz vs c/(4(L+0.6133a)) %.3f Hz (%+.4f %%), delay %.4f samples\n",
                        rad ? "radiating" : "ideal", f, f_ex, 100.0 * (f / f_ex - 1.0), net.direction_delay_samples(0, 0));
            CHECK(rel(f, f_ex) < 0.01);
        }
    }
    SECTION("open-open half wave, 800 K burned gas, probe at a mid junction") {
        WaveguideNetworkDesc d;
        const double length = 0.60, dia = 0.05;
        const GasStateDesc hot{101325.0, 800.0, 1.0};
        d.nodes.push_back(open_node("left", true));
        d.nodes.push_back(node(WgNodeDesc::Kind::Junction, "mid"));
        d.nodes.push_back(open_node("right", true));
        d.segments.push_back(seg("l", 0.5 * length, dia, 0, 1, EndCorrection::Unflanged, EndCorrection::None, WgLoss::None, hot));
        d.segments.push_back(seg("r", 0.5 * length, dia, 1, 2, EndCorrection::None, EndCorrection::Unflanged, WgLoss::None, hot));
        WaveguideNetwork net(d, kHFull);
        const std::vector<double> s = impulse_record(net, 1, 1, 11520);
        const double c = segment_c(net, 0);
        const double f_ex = c / (2.0 * (length + 2.0 * kUnflangedEndCorrection * 0.5 * dia));
        const double f = peak_frequency(s, kHFull, 0.8 * f_ex, 1.2 * f_ex);
        std::printf("W1 open-open: f %.3f Hz vs c/(2(L+1.2266a)) %.3f Hz (%+.4f %%), c %.2f m/s\n", f, f_ex,
                    100.0 * (f / f_ex - 1.0), c);
        CHECK(rel(f, f_ex) < 0.01);
    }
    SECTION("Helmholtz resonator: neck (flanged into the cavity, unflanged outside) on a 5 L volume") {
        WaveguideNetworkDesc d;
        const double length = 0.10, dia = 0.04, vol = 5.0e-3;
        d.nodes.push_back(node(WgNodeDesc::Kind::Junction, "cavity", vol));
        d.nodes.push_back(open_node("mouth", true));
        d.segments.push_back(seg("neck", length, dia, 0, 1, EndCorrection::Flanged, EndCorrection::Unflanged));
        WaveguideNetwork net(d, kHFull);
        const std::vector<double> s = impulse_record(net, 0, 0, 23040);
        const double c = segment_c(net, 0);
        const double area = 0.25 * kPi * dia * dia;
        const double l_eff = length + (kUnflangedEndCorrection + kFlangedEndCorrection) * 0.5 * dia;
        const double f_lumped = c / (2.0 * kPi) * std::sqrt(area / (vol * l_eff));
        // Distributed neck: k L tan(k L) = A L / V (as test_pipe_network P4).
        double lo = 0.0, hi = 1.5;
        for (int it = 0; it < 200; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (mid * std::tan(mid) < area * l_eff / vol) lo = mid;
            else hi = mid;
        }
        const double f_ex = 0.5 * (lo + hi) * c / (2.0 * kPi * l_eff);
        const double f = peak_frequency(s, kHFull, 0.7 * f_ex, 1.3 * f_ex);
        std::printf("W1 Helmholtz: f %.3f Hz vs distributed %.3f Hz (%+.4f %%), lumped %.3f Hz (%+.4f %%)\n", f, f_ex,
                    100.0 * (f / f_ex - 1.0), f_lumped, 100.0 * (f / f_lumped - 1.0));
        CHECK(rel(f, f_ex) < 0.01);
    }
}

TEST_CASE("W2 waveguide radiating open end vs the Levine-Schwinger low-ka reflection", "[drivetrain][cycle][pipe]") {
    // Catches: a wrong radiation pole (|R| off), unflanged/flanged swapped, a
    // radiation filter with gain != 1 at DC. Reference: |R| = 1 - K (ka)^2,
    // K = 1/2 unflanged (Levine & Schwinger 1948), K = 1 flanged (Norris &
    // Sheng 1989 low-ka limit). Measured through the network: an Anechoic
    // source injects a unit impulse, the reflection comes back to it. Checked
    // for ka <= 0.2 (the leading-order reference and the one-pole filter part
    // at O((ka)^4); ka 0.3 is printed: flanged 0.9206 vs 0.91).
    for (int fl = 0; fl < 2; ++fl) {
        const double dia = 0.06;
        WaveguideNetworkDesc d;
        d.nodes.push_back(node(WgNodeDesc::Kind::Anechoic, "src"));
        d.nodes.push_back(open_node("end", true));
        d.segments.push_back(seg("p", 0.3, dia, 0, 1, EndCorrection::None,
                                 fl ? EndCorrection::Flanged : EndCorrection::Unflanged));
        WaveguideNetwork net(d, kHFull);
        std::vector<double> back;
        for (int k = 0; k < 8192; ++k) {
            if (k == 0) net.set_node_wave(0, 1.0);
            net.step();
            back.push_back(net.node_arriving_wave(0));
        }
        const double c = net.segment_sound_speed(0);
        const double kk = fl ? 1.0 : 0.5;
        double worst = 0.0, dc = 0.0;
        for (double ka : {0.02, 0.1, 0.2, 0.3}) {
            const double f = ka * c / (2.0 * kPi * 0.5 * dia);
            const double r = dft_mag(back, kHFull, f);
            const double r_ref = 1.0 - kk * ka * ka;
            if (ka <= 0.2) worst = std::fmax(worst, std::fabs(r - r_ref));
            std::printf("W2 %s ka %.2f: |R| %.5f vs 1 - K(ka)^2 %.5f\n", fl ? "flanged" : "unflanged", ka, r, r_ref);
        }
        for (double v : back) dc += v;
        std::printf("W2 %s: pole %.6f, DC gain %.9f, worst |R| error %.5f\n", fl ? "flanged" : "unflanged",
                    net.radiation_pole(1), -dc, worst);
        CHECK(worst < 0.005);
        CHECK(std::fabs(-dc - 1.0) < 1e-6);
    }
}

TEST_CASE("W3 waveguide Kirchhoff wall loss: filter fit and mode decay", "[drivetrain][cycle][pipe]") {
    // Catches: a wrong Kirchhoff coefficient (Pr, gamma, radius), a loss filter
    // not applied or applied twice, a shelf with gain != 1 at DC, a fit that
    // diverged. Reference alpha(f) = sqrt(pi f nu)(1 + (gamma-1)/sqrt(Pr))/(r c)
    // (Kirchhoff 1868; Pierce 1989 eq. 10-6.23 form for a rigid tube). Mode
    // amplitudes of a closed-closed tube decay as exp(-alpha(f) c t).
    WaveguideNetworkDesc d;
    const double length = 1.0, dia = 0.02;
    d.nodes.push_back(node(WgNodeDesc::Kind::Closed, "a"));
    d.nodes.push_back(node(WgNodeDesc::Kind::Closed, "b"));
    d.segments.push_back(seg("tube", length, dia, 0, 1, EndCorrection::None, EndCorrection::None, WgLoss::Kirchhoff));
    WaveguideNetwork net(d, kHFull);

    // White-box: the fitted filter against its target on the fit band.
    double worst_fit = 0.0;
    for (double f = 40.0; f <= 6000.0; f *= 1.25) {
        const double t = net.wall_attenuation_target(0, f);
        const double a = net.wall_attenuation_filter(0, f);
        worst_fit = std::fmax(worst_fit, std::fabs(a / t - 1.0));
    }
    std::printf("W3 fit: max rel error %.4f (build %.4f), alpha(1 kHz)L %.5f Np\n", worst_fit,
                net.loss_fit_max_rel_error(0), net.wall_attenuation_target(0, 1000.0));
    CHECK(worst_fit < 0.01);
    CHECK(net.wall_attenuation_filter(0, 0.0) < 1e-12);

    // Mode decay: two Hann windows 0.3 s apart.
    const int n_win = 9216;  // 0.2 s
    const int gap = 13824;   // 0.3 s
    const std::vector<double> s = impulse_record(net, 0, 0, gap + n_win);
    const std::vector<double> w0 = hann(std::vector<double>(s.begin(), s.begin() + n_win));
    const std::vector<double> w1 = hann(std::vector<double>(s.begin() + gap, s.begin() + gap + n_win));
    const double c = net.segment_sound_speed(0);
    double worst = 0.0;
    for (int m : {1, 3, 6, 9}) {
        const double f0 = m * c / (2.0 * length);
        const double f = peak_frequency(std::vector<double>(s.begin(), s.begin() + n_win), kHFull, 0.97 * f0, 1.03 * f0);
        const double ratio = dft_mag(w1, kHFull, f) / dft_mag(w0, kHFull, f);
        const double rate = -std::log(ratio) / (gap * kHFull);
        const double rate_ex = net.wall_attenuation_target(0, f) / length * c;
        worst = std::fmax(worst, rel(rate, rate_ex));
        std::printf("W3 mode %d: f %.2f Hz, decay %.4f 1/s vs alpha c %.4f 1/s (%+.2f %%)\n", m, f, rate, rate_ex,
                    100.0 * (rate / rate_ex - 1.0));
    }
    CHECK(worst < 0.02);
}

TEST_CASE("W4 waveguide lossless network is passive: wave energy never grows", "[drivetrain][cycle][pipe]") {
    // Catches: a junction whose scattering is not admittance-weighted (energy
    // created or destroyed at an area/impedance step), a compliance port with
    // the wrong conductance or sign, an allpass state excluded from the energy.
    WaveguideNetwork net(lossless_network(), kHFull);
    for (int k = 0; k < 16; ++k) {
        net.set_node_inflow(1, 1.0e-3 * std::sin(0.7 * k));
        net.set_node_inflow(3, -0.6e-3 * std::cos(0.3 * k));
        net.step();
    }
    const double e0 = net.wave_energy();
    double e_max = e0, e_min = e0;
    for (int k = 0; k < 40000; ++k) {
        net.step();
        const double e = net.wave_energy();
        e_max = std::fmax(e_max, e);
        e_min = std::fmin(e_min, e);
    }
    std::printf("W4 lossless: E0 %.6e J, max %+.3e, min %+.3e relative over 40000 steps\n", e0, e_max / e0 - 1.0,
                e_min / e0 - 1.0);
    CHECK(e0 > 0.0);
    CHECK(e_max / e0 - 1.0 < 1e-12);
    CHECK(1.0 - e_min / e0 < 1e-12);

    // With every loss on, the energy of the same excitation decays.
    WaveguideNetwork lossy(lossy_network(), kHFull);
    for (int k = 0; k < 16; ++k) {
        lossy.set_node_inflow(1, 1.0e-3 * std::sin(0.7 * k));
        lossy.set_node_inflow(3, -0.6e-3 * std::cos(0.3 * k));
        lossy.step();
    }
    const double l0 = lossy.wave_energy();
    for (int k = 0; k < 40000; ++k) lossy.step();
    std::printf("W4 lossy: E after 40000 steps %.3e of E0\n", lossy.wave_energy() / l0);
    CHECK(lossy.wave_energy() < 0.05 * l0);
}

TEST_CASE("W5 waveguide build-time rejections", "[drivetrain][cycle][pipe]") {
    // Catches: a segment too short for the minimum delay being accepted (the
    // delay line would need N < 1 or an unstable allpass), bad topology.
    // expect_message(d, "") means "must not throw"; a non-empty expected
    // string means "must throw std::invalid_argument with exactly this
    // message". CHECK_NOTHROW/CHECK_THROWS_WITH, not try/catch + e.what():
    // catching an exception by reference and then reading it is the
    // clang-cl 19.1.5 Windows ASan crash pattern (vault TOOL-019, CLAUDE.md
    // "Windows ASan caveat") - Catch2's own catch is safe (built with
    // -asan-stack=0).
    auto expect_message = [](const WaveguideNetworkDesc& d, const std::string& expected) {
        if (expected.empty()) {
            CHECK_NOTHROW(WaveguideNetwork(d, kHFull));
        } else {
            CHECK_THROWS_WITH(WaveguideNetwork(d, kHFull), Catch::Matchers::Equals(expected));
        }
    };
    WaveguideNetworkDesc d;
    d.nodes.push_back(node(WgNodeDesc::Kind::Closed, "a"));
    d.nodes.push_back(node(WgNodeDesc::Kind::Closed, "b"));
    d.segments.push_back(seg("stub", 0.005, 0.03, 0, 1, EndCorrection::None, EndCorrection::None));
    CHECK_THROWS_WITH(WaveguideNetwork(d, kHFull),
                       Catch::Matchers::StartsWith("WaveguideNetwork: segment 'stub' delay 0.6") &&
                           Catch::Matchers::ContainsSubstring("below the minimum 1.5"));
    d.segments[0].length_m = 0.012;  // 1.61 samples
    expect_message(d, "");

    WaveguideNetworkDesc t = d;
    t.nodes[0].kind = WgNodeDesc::Kind::Open;
    t.nodes[0].radiation = true;
    expect_message(t, "WaveguideNetwork: segment 'stub': a radiating open end needs an unflanged or flanged end "
                       "correction");
    t.segments[0].end_a = EndCorrection::Unflanged;
    t.segments[0].end_b = EndCorrection::Flanged;
    expect_message(t, "WaveguideNetwork: segment 'stub': a closed end cannot carry an end correction");
    WaveguideNetworkDesc u = d;
    u.nodes.push_back(node(WgNodeDesc::Kind::Junction, "dangling"));
    expect_message(u, "WaveguideNetwork: node 'dangling': a junction needs at least one segment");
    u = d;
    u.segments.push_back(seg("extra", 0.3, 0.03, 0, 1, EndCorrection::None, EndCorrection::None));
    expect_message(u, "WaveguideNetwork: node 'a': an end node needs exactly one segment");
    u = d;
    u.nodes[0].kind = WgNodeDesc::Kind::Open;
    u.nodes[0].radiation = false;
    u.nodes[1].kind = WgNodeDesc::Kind::Anechoic;
    u.segments[0].loss = WgLoss::LaminarFriction;
    expect_message(u, "WaveguideNetwork: segment 'stub': laminar friction needs a Junction or Closed node at one end "
                       "at least");
    u.nodes[1].kind = WgNodeDesc::Kind::Closed;
    expect_message(u, "");
}

TEST_CASE("W6 waveguide stepping allocates nothing inside a job", "[drivetrain][cycle][pipe]") {
    // Catches: a heap allocation in begin_step/finish_step/step.
    if (!ps::jobs::debug_alloc::hook_available()) {
        SUCCEED("debug_alloc hook unavailable (sanitizer build)");
        return;
    }
    WaveguideNetwork net(lossy_network(), kHFull);
    ps::jobs::JobSystem js(2);
    auto run = [&](int n) {
        ps::jobs::JobSystem::WaitGroup wg;
        js.submit(
            [&] {
                for (int k = 0; k < n; ++k) {
                    net.begin_step();
                    net.set_node_inflow(1, 1.0e-4 * std::sin(0.01 * k));
                    net.finish_step();
                }
            },
            &wg);
        js.wait(wg);
    };
    run(10);
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    run(5000);
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t v = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    CHECK(v == 0);
    CHECK(std::isfinite(net.wave_energy()));
}

TEST_CASE("W7 waveguide run is bit-identical run to run", "[drivetrain][cycle][pipe]") {
    // Catches: uninitialised state, order-dependent build, hidden statics.
    auto run = [] {
        WaveguideNetwork net(lossy_network(), kHFull);
        for (int k = 0; k < 20000; ++k) {
            // ps::math::sin, not std::sin: the pinned hash below needs bit-identical input on every platform.
            net.set_node_inflow(1, 1.0e-4 * ps::math::sin(0.013 * k));
            net.step();
        }
        return net.fold_state_hash(1469598103934665603ull);
    };
    const std::uint64_t a = run();
    const std::uint64_t b = run();
    std::printf("W7 hash %016llx\n", static_cast<unsigned long long>(a));
    CHECK(a == b);
    CHECK(a == 0xab154e22348af820ull);  // recorded 2026-09-28, clang-cl release and linux-release (ps::math::sin input)
}

TEST_CASE("waveguide diag: micro-benchmark ns per WG sample per segment", "[.][diag][drivetrain][cycle][pipe]") {
    for (int variant = 0; variant < 2; ++variant) {
        WaveguideNetwork net(variant == 0 ? lossy_network() : lossless_network(), kHFull);
        const int steps = 460800;  // 10 s at h_full
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 0; k < steps; ++k) {
            net.set_node_inflow(1, (k & 1023) == 0 ? 1.0e-4 : 0.0);
            net.step();
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
        std::printf("waveguide bench (%s): %d segments, %.2f ns per sample per segment, %.2f ns per step\n",
                    variant == 0 ? "Kirchhoff + radiation" : "lossless", net.segment_count(),
                    ns / (static_cast<double>(steps) * net.segment_count()), ns / steps);
        CHECK(std::isfinite(net.wave_energy()));
    }
}

TEST_CASE("V6 expansion-chamber transmission loss vs the plane-wave formula below the first cross-mode",
          "[drivetrain][cycle][pipe]") {
    // Catches: a junction that does not weight the arriving waves by segment
    // admittance (the area ratio disappears), a wrong chamber delay (plan V6).
    // Anechoic inlet -> 50 mm pipe -> 150 mm x 0.3 m chamber -> 50 mm pipe ->
    // anechoic outlet, lossless, no mean flow. A 1 Pa incident impulse; with
    // equal inlet/outlet pipes TL = -20 log10 |T(f)|, T the DFT of the
    // transmitted wave (the allpass delays are unit-magnitude). Reference
    // (Davis et al., NACA Report 1192, 1954; Munjal, Acoustics of Ducts and
    // Mufflers, 1987, simple expansion chamber):
    //   TL = 10 log10(1 + (m - 1/m)^2 sin^2(kL) / 4),  m = S_ch/S_p,
    // valid below the first cross-mode f_c = 1.841 c/(pi D_ch).
    const double d_p = 0.05, d_ch = 0.15, l_ch = 0.3;
    WaveguideNetworkDesc d;
    d.nodes = {node(WgNodeDesc::Kind::Anechoic, "in"), node(WgNodeDesc::Kind::Junction, "j1"),
               node(WgNodeDesc::Kind::Junction, "j2"), node(WgNodeDesc::Kind::Anechoic, "out")};
    d.segments.push_back(seg("inlet", 0.5, d_p, 0, 1, EndCorrection::None, EndCorrection::None));
    d.segments.push_back(seg("chamber", l_ch, d_ch, 1, 2, EndCorrection::None, EndCorrection::None));
    d.segments.push_back(seg("outlet", 0.5, d_p, 2, 3, EndCorrection::None, EndCorrection::None));
    WaveguideNetwork net(d, kHFull);
    std::vector<double> out;
    const int steps = static_cast<int>(0.25 / kHFull);
    for (int k = 0; k < steps; ++k) {
        if (k == 0) net.set_node_wave(0, 1.0);
        net.step();
        out.push_back(net.node_arriving_wave(3));
    }
    const double c = net.segment_sound_speed(1);
    const double l = net.segment_effective_length(1);
    const double m = (d_ch * d_ch) / (d_p * d_p);
    const double f_c = 1.841 * c / (kPi * d_ch);
    double worst = 0.0, worst_f = 0.0, tl_max = 0.0;
    for (double f = 20.0; f < f_c; f += 10.0) {
        const double s = std::sin(2.0 * kPi * f * l / c);
        const double tl_ref = 10.0 * std::log10(1.0 + 0.25 * (m - 1.0 / m) * (m - 1.0 / m) * s * s);
        const double tl = -20.0 * std::log10(dft_mag(out, kHFull, f));
        tl_max = std::fmax(tl_max, tl);
        if (std::fabs(tl - tl_ref) > worst) {
            worst = std::fabs(tl - tl_ref);
            worst_f = f;
        }
    }
    std::printf("V6: m %.1f, f_c %.1f Hz, max TL %.3f dB, worst |TL - formula| %.3e dB at %.0f Hz\n", m, f_c, tl_max,
                worst, worst_f);
    CHECK(worst < 1.0);
    CHECK(tl_max > 12.0);
}

TEST_CASE("W8 quasi-steady resistance element: mean pressure drop, linearised transmission, dissipation",
          "[drivetrain][cycle][pipe]") {
    // Catches: a wrong mean-drop law (u from the wrong area or without the
    // flow fraction), a linearised resistance that is not d(dp)/dQ, a 2-port
    // that creates energy. Two anechoic lines through one Resistance node; a
    // 1 Pa impulse from the left. Series resistance R between equal
    // impedances Z: transmitted 2Z/(R + 2Z), reflected R/(R + 2Z) (DC sums of
    // the unit-DC-gain allpass responses).
    for (int variant = 0; variant < 2; ++variant) {
        WaveguideNetworkDesc d;
        const double dia = 0.05, area = 0.25 * kPi * dia * dia;
        const double mdot = 0.05;
        d.nodes = {node(WgNodeDesc::Kind::Anechoic, "in"), node(WgNodeDesc::Kind::Resistance, "cat"),
                   node(WgNodeDesc::Kind::Anechoic, "out")};
        d.segments.push_back(seg("a", 0.3, dia, 0, 1, EndCorrection::None, EndCorrection::None));
        d.segments.push_back(seg("b", 0.3, dia, 1, 2, EndCorrection::None, EndCorrection::None));
        const WaveguideNetwork probe(d, kHFull);
        const double rho = probe.segment_density(0);
        const double z = 1.0 / probe.segment_admittance(0);
        double r_ac = 0.0, dp_ref = 0.0;
        if (variant == 0) {
            d.nodes[1].r_linear_pa_s_m = rho * probe.segment_sound_speed(0);  // R = Z
            r_ac = d.nodes[1].r_linear_pa_s_m / area;
            dp_ref = d.nodes[1].r_linear_pa_s_m * mdot / (rho * area);
        } else {
            d.nodes[1].k_loss = 4.0;
            d.nodes[1].flow_fraction = 0.5;
            const double u = 0.5 * mdot / (rho * area);
            dp_ref = 4.0 * rho * u * u / 2.0;
            r_ac = 4.0 * rho * u / area;
        }
        WaveguideNetwork net(d, kHFull);
        net.set_mean_mass_flow(mdot);
        double t_sum = 0.0, r_sum = 0.0, t_sq = 0.0, r_sq = 0.0;
        for (int k = 0; k < 4000; ++k) {
            if (k == 0) net.set_node_wave(0, 1.0);
            net.step();
            const double t = net.node_arriving_wave(2), r = net.node_arriving_wave(0);
            t_sum += t;
            r_sum += r;
            t_sq += t * t;
            r_sq += r * r;
        }
        const double t_ref = 2.0 * z / (r_ac + 2.0 * z), rr_ref = r_ac / (r_ac + 2.0 * z);
        const double dissipated = 1.0 - t_sq - r_sq;
        const double diss_ref = 1.0 - t_ref * t_ref - rr_ref * rr_ref;
        std::printf("W8 %s: T %.12f vs %.12f, R %.12f vs %.12f, dissipated %.9f vs %.9f, dp %.6f vs %.6f Pa\n",
                    variant == 0 ? "linear R = Z" : "k_loss 4, flow fraction 0.5", t_sum, t_ref, r_sum, rr_ref,
                    dissipated, diss_ref, net.mean_pressure_drop(), dp_ref);
        CHECK(std::fabs(t_sum - t_ref) < 1e-9);
        CHECK(std::fabs(r_sum - rr_ref) < 1e-9);
        CHECK(std::fabs(dissipated - diss_ref) < 1e-9);
        CHECK(dissipated > 0.0);
        CHECK(rel(net.mean_pressure_drop(), dp_ref) < 1e-12);
        // The linearised element drops (r_lin + K rho |u|) u for the same DC
        // flow: dp for a linear element, twice dp for a pure K element.
        CHECK(rel(net.linearised_mean_drop(), variant == 0 ? dp_ref : 2.0 * dp_ref) < 1e-12);
    }
}
