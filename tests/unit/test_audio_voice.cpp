// ps_audio A7: the streaming engine voice on the [AS] pipe I4 fixture
// (tests/support/cycle_pipe_fixtures.h). V1 order content, V11 seeded
// determinism, V12 time map in the loop (ramp tracking, hitch), V13 stop,
// the maps <-> audio air tie, zero allocation, the threaded Runner.
// Sabotages and failing numbers: out/audio_a7_progress.md "Sabotages".
#include "cycle_fixtures.h"
#include "cycle_pipe_fixtures.h"

#include "ps/engine_cycle/cycle_engine.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/math/transcendental.h"
#include "ps_audio/runner.h"
#include "ps_audio/voice.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

using namespace ps;
using ps_audio::EngineInput;
using ps_audio::Voice;
using ps_audio::VoiceSettings;

namespace {

constexpr real kTwoPi = 6.283185307179586476925286766559;
constexpr real kPi = 3.141592653589793238462643383280;
constexpr real kFs = 48000.0;

real omega_of(real rpm) { return rpm * kTwoPi / 60.0; }

const drivetrain::CycleModelDesc& fixture() {
    static const drivetrain::CycleModelDesc d = test::make_pipe_i4_cycle_desc();
    return d;
}

// Coarse cells (5 cm) for the tests whose subject is timing, not the pipes.
VoiceSettings coarse_settings(real rpm, real p_im) {
    VoiceSettings vs;
    vs.substeps = 1;
    vs.engine.dx_intake_m = 0.05;
    vs.engine.dx_exhaust_m = 0.05;
    vs.engine.initial_rpm = rpm;
    vs.engine.initial_p_intake_pa = p_im;
    return vs;
}

VoiceSettings audio_settings(real rpm, real p_im) {
    VoiceSettings vs;
    vs.substeps = 1;  // the probe result for the fixture (checked in "A7 S1")
    vs.engine.initial_rpm = rpm;
    vs.engine.initial_p_intake_pa = p_im;
    return vs;
}

EngineInput steady_input(real t, real rpm, real p_im) {
    EngineInput in;
    in.t_sim_s = t;
    in.omega_rad_s = omega_of(rpm);
    in.p_intake_pa = p_im;
    return in;
}

// Renders a voice while a scripted physics thread "produces" one snapshot per
// 1/240 s of simulation time, arriving every (1/240)/clock_ratio of audio time.
struct Driver {
    Voice& v;
    real clock_ratio = 1.0;
    real dt_sim = 1.0 / 240.0;
    real t_sim = 0.0;
    real next_arrival = 0.0;  // audio sample index (fractional)
    std::uint64_t n = 0;      // samples rendered
    std::uint64_t max_frames = 0;  // > 0: cap every render() call at this many frames
    std::vector<float> chunk;

    explicit Driver(Voice& voice, real ratio = 1.0) : v(voice), clock_ratio(ratio) {}

    template <class F>
    void run(real seconds, F&& input_at, std::vector<float>* rec, bool produce = true) {
        const std::uint64_t end = n + static_cast<std::uint64_t>(std::llround(seconds * kFs));
        const std::size_t ns = static_cast<std::size_t>(v.source_count());
        while (n < end) {
            std::uint64_t stop = end;
            if (produce) {
                while (next_arrival <= static_cast<real>(n)) {
                    v.push_input(input_at(t_sim));
                    t_sim += dt_sim;
                    next_arrival += dt_sim / clock_ratio * kFs;
                }
                stop = std::min<std::uint64_t>(end, static_cast<std::uint64_t>(std::ceil(next_arrival)));
                if (stop <= n) stop = n + 1;
            }
            if (max_frames > 0 && stop - n > max_frames) stop = n + max_frames;
            const std::size_t frames = static_cast<std::size_t>(stop - n);
            chunk.resize(frames * ns);
            v.render(chunk.data(), static_cast<int>(frames));
            if (rec != nullptr) rec->insert(rec->end(), chunk.begin(), chunk.end());
            n = stop;
        }
        if (!produce) next_arrival = static_cast<real>(n);
    }
};

// Channel s of an interleaved recording, samples [i0, i0 + len).
std::vector<real> channel(const std::vector<float>& rec, int ns, int s, std::size_t i0, std::size_t len) {
    std::vector<real> x(len);
    for (std::size_t i = 0; i < len; ++i) x[i] = rec[(i0 + i) * static_cast<std::size_t>(ns) + static_cast<std::size_t>(s)];
    return x;
}

// Amplitude of the component at `cycles` periods over the window (Goertzel,
// ps::math only).
real goertzel_amp(const std::vector<real>& x, real cycles) {
    const real w = kTwoPi * cycles / static_cast<real>(x.size());
    const real c = 2.0 * ps::math::cos(w);
    real s1 = 0.0, s2 = 0.0;
    for (real v : x) {
        const real s0 = v + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    const real re = s1 - s2 * ps::math::cos(w);
    const real im = s2 * ps::math::sin(w);
    return 2.0 * std::sqrt(re * re + im * im) / static_cast<real>(x.size());
}

real rms(const std::vector<real>& x, std::size_t i0, std::size_t len) {
    real s = 0.0;
    for (std::size_t i = i0; i < i0 + len; ++i) s += x[i] * x[i];
    return std::sqrt(s / static_cast<real>(len));
}

std::uint64_t pcm_hash(const std::vector<float>& rec) {
    std::uint64_t h = 1469598103934665603ull;
    for (float f : rec) {
        unsigned char b[sizeof(float)];
        std::memcpy(b, &f, sizeof(float));
        for (unsigned char c : b) {
            h ^= c;
            h *= 1099511628211ull;
        }
    }
    return h;
}

bool all_finite(const std::vector<float>& rec) {
    for (float f : rec)
        if (!std::isfinite(f)) return false;
    return true;
}

struct OrderSpectrum {
    real order2 = 0.0;       // amplitude of engine order 2 (firing frequency)
    real worst_other = 0.0;  // largest half or odd order up to order 20
    real worst_other_order = 0.0;
};

// Engine orders k/2 (k = 1..40) over an integer number of cycles.
OrderSpectrum orders(const std::vector<real>& x, int n_cycles) {
    OrderSpectrum o;
    o.order2 = goertzel_amp(x, 4.0 * n_cycles);
    for (int k = 1; k <= 40; ++k) {
        if (k % 4 == 0) continue;  // multiples of order 2
        const real a = goertzel_amp(x, static_cast<real>(k * n_cycles));
        if (a > o.worst_other) {
            o.worst_other = a;
            o.worst_other_order = 0.5 * k;
        }
    }
    return o;
}

real db(real ratio) { return 20.0 * std::log10(ratio); }

// V1 run: steady rpm, fixed throttle (no plenum loop: an exactly periodic
// steady state), no variation.
std::vector<OrderSpectrum> v1_run(const drivetrain::CycleModelDesc& d) {
    const real rpm = 3000.0;
    VoiceSettings vs = audio_settings(rpm, 60000.0);
    vs.engine.variation_scale = 0.0;
    Voice v(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v);
    std::vector<float> rec;
    auto in = [&](real t) {
        EngineInput e = steady_input(t, rpm, 0.0);
        e.p_intake_pa = 0.0;
        e.throttle = 0.25;
        return e;
    };
    const int n_cycles = 8;
    const std::size_t spc = 1920;  // samples per cycle at 3000 rpm, 48 kHz
    drv.run(0.6, in, nullptr);
    drv.run(static_cast<real>(n_cycles * spc) / kFs, in, &rec);
    const int ns = v.source_count();
    std::vector<OrderSpectrum> out;
    for (int s = 0; s < ns; ++s) out.push_back(orders(channel(rec, ns, s, 0, n_cycles * spc), n_cycles));
    return out;
}

}  // namespace

TEST_CASE("A7 S1: the fixture's stability probe", "[audio][cycle][pipe]") {
    // The probe runs two cycles at the stiffest grid corner at h and h/2.
    ps_audio::LiveEngineSettings s;
    const ps_audio::StabilityProbeResult r =
        ps_audio::probe_stability(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, s);
    std::printf("A7 S1: probe at %.0f rpm / %.0f Pa: air h %.6e kg, h/2 %.6e kg, rel %.3e, finite %d -> substeps %d\n",
                r.rpm, r.p_im_pa, r.air_h_kg, r.air_h2_kg, r.rel_diff, r.finite ? 1 : 0, r.substeps);
    CHECK(r.finite);
    CHECK(r.rpm == 1500.0);
    CHECK(r.p_im_pa == 30000.0);
    CHECK(r.air_h_kg > 0.0);
    CHECK(r.substeps == 1);
    // Deterministic: the same probe twice gives the same bits.
    const ps_audio::StabilityProbeResult r2 =
        ps_audio::probe_stability(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, s);
    CHECK(r2.air_h_kg == r.air_h_kg);
    CHECK(r2.air_h2_kg == r.air_h2_kg);
}

TEST_CASE("A7 V1: steady I4 without variation - order 2 and multiples dominate", "[audio][cycle][pipe]") {
    // Catches: cylinders out of phase or unequal (half orders), a firing
    // interval error (odd orders), an aperiodic steady state.
    const std::vector<OrderSpectrum> sp = v1_run(fixture());
    REQUIRE(sp.size() >= 2);
    for (std::size_t s = 0; s < sp.size(); ++s) {
        const OrderSpectrum& o = sp[s];
        std::printf("A7 V1: source %zu: order 2 %.4e Pa (%.1f dB SPL at 1 m), worst half/odd order %.1f at %.1f dB rel\n", s,
                    o.order2, db(o.order2 / 20e-6), o.worst_other_order, db(o.worst_other / o.order2));
        CHECK(o.order2 > 0.0);
        CHECK(db(o.worst_other / o.order2) < -40.0);
    }
}

TEST_CASE("A7 V1 sabotage diag: one firing angle 90 deg off shows half orders", "[.][diag][audio]") {
    drivetrain::CycleModelDesc d = fixture();
    d.firing_order.clear();
    d.firing_angles_deg = {0.0, 540.0, 270.0, 360.0};  // cylinder 3 at 270 instead of 180
    const std::vector<OrderSpectrum> sp = v1_run(d);
    for (std::size_t s = 0; s < sp.size(); ++s) {
        const OrderSpectrum& o = sp[s];
        std::printf("A7 V1 sabotage: source %zu: order 2 %.4e Pa, worst half/odd order %.1f at %.1f dB rel\n", s, o.order2,
                    o.worst_other_order, db(o.worst_other / o.order2));
    }
}

namespace {

// ---- V2 (ps_audio A8) ----------------------------------------------------
// V8 bank order content: see tests/support/cycle_pipe_fixtures.h's own
// header comment on make_v8_bank_cross_plane_cycle_desc/
// make_v8_bank_flat_plane_cycle_desc for why each bank is its own engine,
// and the crank pre-roll (bank B's absolute phase relative to bank A) that
// makes the two banks' events interleave into a full V8's even 90-deg
// firing.

int exhaust_source_index(const Voice& v) {
    for (int s = 0; s < v.source_count(); ++s)
        if (!v.source(s).intake) return s;
    return -1;
}

// Renders `n_cycles` cycles of one bank at a steady rpm after settling the
// crank-phase lock onto `preroll_deg` (the model convention's
// crank_angle_deg, 360 = cylinder 1's firing TDC) for `settle_s` - a preroll
// of 180/90 deg needs several seconds for the capped (1 % of omega) phase
// correction to close (V12d's own bound); 0 deg (bank A) locks near-
// instantly but still gets the same settle so both banks' recordings cover
// identical simulated time. Returns the exhaust channel only.
std::vector<real> render_v8_bank(const drivetrain::CycleModelDesc& d, real rpm, real preroll_deg, int n_cycles,
                                 real settle_s) {
    VoiceSettings vs = audio_settings(rpm, 60000.0);
    vs.engine.variation_scale = 0.0;
    Voice v(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    const int ex = exhaust_source_index(v);
    REQUIRE(ex >= 0);
    Driver drv(v);
    const real deg_per_s = omega_of(rpm) * (360.0 / kTwoPi);  // crank deg/s = omega[rad/s] * 180/pi
    auto in = [&](real t) {
        EngineInput e = steady_input(t, rpm, 0.0);
        e.p_intake_pa = 0.0;
        e.throttle = 0.25;
        real theta = std::fmod(360.0 - preroll_deg + deg_per_s * t, 720.0);
        if (theta < 0.0) theta += 720.0;
        e.crank_angle_deg = theta;
        return e;
    };
    drv.run(settle_s, in, nullptr);
    const std::size_t spc = static_cast<std::size_t>(std::llround(kFs * 120.0 / rpm));  // samples/720 deg at kFs
    std::vector<float> rec;
    drv.run(static_cast<real>(n_cycles) * static_cast<real>(spc) / kFs, in, &rec);
    const int ns = v.source_count();
    return channel(rec, ns, ex, 0, static_cast<std::size_t>(n_cycles) * spc);
}

}  // namespace

TEST_CASE("A8 V2: V8 bank order content - cross-plane half-orders, flat-plane order 2, combined order 4",
          "[audio][cycle][pipe][long]") {
    const real rpm = 3000.0;
    const int n_cycles = 8;
    const real settle_s = 3.0;  // closes a 180 deg preroll at 3000 rpm (V12d's cap: 1 % of omega)

    // Flat-plane bank: identical fixture to the plain I4 (A7 V1) - order 2
    // dominates, half/odd orders suppressed. Reuses v1_run()'s own
    // machinery (unpreroll'd/no crank feed) since it IS the I4 fixture.
    {
        const std::vector<OrderSpectrum> sp = v1_run(test::make_v8_bank_flat_plane_cycle_desc());
        for (const OrderSpectrum& o : sp) {
            std::printf("A8 V2 flat-plane bank: order2 %.4e Pa, worst half/odd order %.1f at %.1f dB rel\n", o.order2,
                        o.worst_other_order, db(o.worst_other / o.order2));
            CHECK(o.order2 > 0.0);
            CHECK(db(o.worst_other / o.order2) < -40.0);  // same bound A7 V1 uses
        }
    }

    // Cross-plane bank (alone, no preroll): the alternating 90/270 deg
    // within-bank pattern breaks the I4's 180-deg sub-symmetry, so energy
    // appears at half/odd orders (k/2, k odd) too - NOT suppressed the way
    // the flat-plane bank's is.
    const std::vector<real> cross_a = render_v8_bank(test::make_v8_bank_cross_plane_cycle_desc(), rpm, 0.0, n_cycles,
                                                      0.6);
    {
        const OrderSpectrum o = orders(cross_a, n_cycles);
        std::printf(
            "A8 V2 cross-plane bank alone: order2 %.4e Pa, worst half/odd order %.1f at %.1f dB rel (was < -40 for "
            "flat-plane)\n",
            o.order2, o.worst_other_order, db(o.worst_other / o.order2));
        CHECK(o.order2 > 0.0);
        CHECK(o.worst_other > 0.0);
        // A half-integer (non-multiple-of-4-in-k, i.e. not a plain even
        // order) is genuinely present and not suppressed the way A7 V1's
        // flat-plane case is - the discriminating check sabotage
        // ("A8 V2 sabotage") flips banks so this goes back below -40 dB.
        CHECK(db(o.worst_other / o.order2) > -20.0);
    }

    // Cross-plane, BOTH banks, bank B pre-rolled 180 deg: the combined
    // 8-cylinder signal should recover order 4 (8 events per 720 deg,
    // evenly spaced) as the dominant component, with the half-orders that
    // dominated a single bank now suppressed (the two banks' odd/half
    // content cancels on summation, the way two mirror-image asymmetric
    // patterns do).
    const std::vector<real> cross_b =
        render_v8_bank(test::make_v8_bank_cross_plane_cycle_desc(), rpm, 180.0, n_cycles, settle_s);
    REQUIRE(cross_a.size() == cross_b.size());
    // cross_a's own recording used a 0.6 s settle, cross_b's used settle_s -
    // re-render bank A with the SAME settle so both windows cover identical
    // simulated time before summing (Driver's t_sim/next_arrival sequence
    // depends only on dt_sim/clock_ratio, not on the crank feed, so this
    // just re-aligns the recorded window, not the phase).
    const std::vector<real> cross_a2 =
        render_v8_bank(test::make_v8_bank_cross_plane_cycle_desc(), rpm, 0.0, n_cycles, settle_s);
    REQUIRE(cross_a2.size() == cross_b.size());
    std::vector<real> summed(cross_a2.size());
    for (std::size_t i = 0; i < summed.size(); ++i) summed[i] = cross_a2[i] + cross_b[i];
    const real order4 = goertzel_amp(summed, 8.0 * n_cycles);
    // Same convention orders() uses: over an n_cycles-cycle (720 deg each)
    // window, only cycles = m * n_cycles (integer m) lands on an actual
    // harmonic bin of a signal periodic every 720 deg (any other C leaks
    // across bins instead of measuring real content) - m corresponds to
    // engine order 0.5*m (order2 is m=4, order4 is m=8).
    //
    // Summing bank A with bank B shifted by a constant crank delta=180 deg
    // scales harmonic m by |1 + exp(i*phi_m)| = 2*|cos(phi_m/2)| with
    // phi_m = 2*pi*m*delta/720 = 90*m degrees: m%4==0 -> cos(180*m/4 ...) =
    // +-1 -> DOUBLES (order 2, 4, 6, 8, 10, ... all fall here, since they
    // are structurally identical content in both banks - a real runner/
    // primary acoustic resonance at, say, order 10 is present in EITHER
    // bank alone and doubles right along with order 4, it is not part of
    // the firing-ASYMMETRY content the 180 deg trick is meant to cancel);
    // m%4==2 -> cos(90 deg)=0 -> CANCELS exactly (order 1, 3, 5, ... - the
    // half-integer content a single cross-plane bank's alternating 90/270
    // deg gaps produce, confirmed dominant above in "cross-plane bank
    // alone"); odd m -> cos(+-45 deg) -> combines to sqrt(2)x, neither
    // cancelling nor doubling. So "dominated by order 4" is a claim about
    // the firing-symmetry content specifically: order 4 (which doubles)
    // must dominate the content that does NOT structurally double, i.e.
    // every m NOT a multiple of 4 - genuine per-bank resonances at other
    // multiples of order 2 (order 2, 6, 10, ...) are excluded from this
    // scan because the preroll trick makes no claim about them (they
    // double regardless of firing symmetry, same as order 4 itself).
    real worst_other = 0.0, worst_other_order = 0.0;
    for (int m = 1; m <= 40; ++m) {
        if (m % 4 == 0) continue;  // structurally doubles regardless of firing symmetry, not part of this claim
        const real a = goertzel_amp(summed, static_cast<real>(m * n_cycles));
        if (a > worst_other) {
            worst_other = a;
            worst_other_order = 0.5 * static_cast<real>(m);
        }
    }
    std::printf("A8 V2 combined dual outlets: order4 %.4e Pa, worst other order %.2f at %.1f dB rel\n", order4,
                worst_other_order, db(worst_other / order4));
    CHECK(order4 > 0.0);
    CHECK(db(worst_other / order4) < -20.0);
}

TEST_CASE("A8 V2 sabotage diag: bank assignment swapped removes the combined order-4 dominance", "[.][diag][audio]") {
    // Sabotage: bank B run WITHOUT its 180 deg preroll (both banks at the
    // SAME absolute phase, as if bank assignment were swapped/lost) -
    // the combined signal is then just 2x a single bank's own half-order
    // content, not the evenly-interleaved 8-cylinder pattern.
    const real rpm = 3000.0;
    const int n_cycles = 8;
    const std::vector<real> a = render_v8_bank(test::make_v8_bank_cross_plane_cycle_desc(), rpm, 0.0, n_cycles, 0.6);
    const std::vector<real> b = render_v8_bank(test::make_v8_bank_cross_plane_cycle_desc(), rpm, 0.0, n_cycles, 0.6);
    REQUIRE(a.size() == b.size());
    std::vector<real> summed(a.size());
    for (std::size_t i = 0; i < summed.size(); ++i) summed[i] = a[i] + b[i];
    const real order4 = goertzel_amp(summed, 8.0 * n_cycles);
    real worst_other = 0.0, worst_other_order = 0.0;
    for (int m = 1; m <= 40; ++m) {
        if (m % 4 == 0) continue;  // structurally doubles regardless of firing symmetry, see the non-sabotage case's comment
        const real amp = goertzel_amp(summed, static_cast<real>(m * n_cycles));
        if (amp > worst_other) {
            worst_other = amp;
            worst_other_order = 0.5 * static_cast<real>(m);
        }
    }
    std::printf("A8 V2 sabotage: order4 %.4e Pa, worst other order %.2f at %.1f dB rel (want NOT < -20)\n", order4,
                worst_other_order, db(worst_other / order4));
}

TEST_CASE("A7 diag: sources and a first look", "[.][diag][audio]") {
    VoiceSettings vs = audio_settings(3000.0, 60000.0);
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    for (int s = 0; s < v.source_count(); ++s) {
        const auto& r = v.source(s);
        std::printf("source %d: %s kind %d intake %d area %.3e\n", s, r.name.c_str(), static_cast<int>(r.kind),
                    r.intake ? 1 : 0, r.area_m2);
    }
    Driver drv(v);
    std::vector<float> rec;
    const auto t0 = std::chrono::steady_clock::now();
    drv.run(1.0, [](real t) { return steady_input(t, 3000.0, 60000.0); }, &rec);
    const real wall = std::chrono::duration<real>(std::chrono::steady_clock::now() - t0).count();
    const int ns = v.source_count();
    for (int s = 0; s < ns; ++s) {
        const std::vector<real> x = channel(rec, ns, s, 0, 48000);
        for (int w = 0; w < 10; ++w)
            std::printf("source %d window %d rms %.4e Pa\n", s, w, rms(x, static_cast<std::size_t>(w) * 4800, 4800));
    }
    std::printf("plenum %.1f Pa (filtered %.1f), cda %.4e, rate %.6f lag %.3f, wall %.3f s for 1 s\n",
                v.engine().plenum_pressure_pa(), v.engine().plenum_filtered_pa(), v.engine().throttle_cd_area(),
                v.time_map().rate(), v.time_map().lag_intervals(), wall);
    for (int c = 0; c < v.engine().cylinders(); ++c)
        std::printf("cyl %d fresh %.6e kg events %llu\n", c, v.engine().last_fresh_air_kg(c),
                    static_cast<unsigned long long>(v.engine().ivc_events(c)));
}

namespace {

// ---- V11 ---------------------------------------------------------------
struct V11Out {
    std::uint64_t pcm = 0;
    std::uint64_t state = 0;
    std::uint64_t cuts = 0;
};

// A short script: a speed and load ramp, a spark change and a fuel cut.
// `split` != 0 renders the same script with differently sized render() calls.
V11Out v11_run(std::uint64_t seed, int split) {
    VoiceSettings vs = coarse_settings(1200.0, 35000.0);
    vs.engine.seed = seed;
    vs.engine.variation_scale = 1.0;
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    auto in = [](real t) {
        const real f = std::min(1.0, t / 0.3);
        EngineInput e = steady_input(t, 1200.0 + 1300.0 * f, 35000.0 + 35000.0 * f);
        e.spark_given = true;
        e.spark_deg_atdc = -25.0 + 10.0 * f;
        if (t >= 0.25 && t < 0.3) e.flags = ps_audio::kInputFuelCut;
        return e;
    };
    std::vector<float> rec;
    Driver drv(v);
    if (split == 0) {
        drv.run(0.4, in, &rec);
    } else {
        drv.max_frames = 37;  // render() calls of <= 37 frames, never aligned to the 32-sample control grid
        drv.run(0.1, in, &rec);
        drv.run(0.1003, in, &rec);
        drv.run(0.1997, in, &rec);
    }
    V11Out o;
    o.pcm = pcm_hash(rec);
    o.state = v.fold_state_hash(1469598103934665603ull);
    o.cuts = v.engine().cut_events();
    return o;
}

// ---- tie -----------------------------------------------------------------
struct TieOut {
    real live_air = 0.0;   // mean fresh air per cylinder-cycle, audio grade (h 1/48000, audio dx)
    real power_air = 0.0;  // CycleEngine at Full h and the power dx (the map generator's engine)
    real live_plenum = 0.0;
    real power_plenum = 0.0;
};

TieOut tie_point(const drivetrain::CycleModelDesc& live_desc, real rpm, real p_im) {
    const drivetrain::CycleModelDesc& d = fixture();
    TieOut t;
    engine_cycle::OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p_im;
    op.t_im_k = 288.15;
    op.p_amb_pa = 101325.0;
    op.lambda = 1.0;
    op.spark_deg_atdc = test::mbt_spark_deg_atdc(d);
    engine_cycle::CycleEngine ce(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op);
    const engine_cycle::CycleEngineResult r = ce.run();
    for (int c = 0; c < r.cylinders; ++c) t.power_air += r.air_kg[static_cast<std::size_t>(c)];
    t.power_air /= r.cylinders;
    t.power_plenum = r.p_plenum_mean_pa;

    ps_audio::LiveEngineSettings s;
    s.initial_rpm = rpm;
    s.initial_p_intake_pa = p_im;
    s.variation_scale = 0.0;
    s.t_amb_k = 288.15;
    ps_audio::LiveEngine eng(live_desc, test::kCycleFixtureLhv, test::kCycleFixtureAfr, s);
    ps_audio::LiveControl c;
    c.omega_rad_s = omega_of(rpm);
    c.p_intake_pa = p_im;
    c.start_of_burn_deg_atdc = eng.mbt_start_of_burn_deg();
    std::array<real, ps_audio::kLiveMaxSources> out{};
    const std::int64_t settle = static_cast<std::int64_t>(1.0 * kFs);
    for (std::int64_t i = 0; i < settle; ++i) eng.step_sample(c, out.data());
    const int nc = eng.cylinders();
    std::vector<std::uint64_t> seen(static_cast<std::size_t>(nc));
    for (int k = 0; k < nc; ++k) seen[static_cast<std::size_t>(k)] = eng.ivc_events(k);
    const std::int64_t meas = static_cast<std::int64_t>(std::llround(4.0 * 120.0 / rpm * kFs));
    real sum = 0.0, psum = 0.0;
    int count = 0;
    for (std::int64_t i = 0; i < meas; ++i) {
        eng.step_sample(c, out.data());
        psum += eng.plenum_pressure_pa();
        for (int k = 0; k < nc; ++k) {
            const std::size_t kk = static_cast<std::size_t>(k);
            if (eng.ivc_events(k) != seen[kk]) {
                seen[kk] = eng.ivc_events(k);
                sum += eng.last_fresh_air_kg(k);
                ++count;
            }
        }
    }
    t.live_air = count > 0 ? sum / count : 0.0;
    t.live_plenum = psum / static_cast<real>(meas);
    return t;
}

}  // namespace

TEST_CASE("A7 V11: same seed and script give identical PCM, another seed differs, maps ignore the seed",
          "[audio][cycle][pipe]") {
    // Catches: non-deterministic state (mutable statics, unordered iteration,
    // render-call-size dependence), a variation that ignores the seed, and
    // (pinned CycleEngine hash) the map generator's engine calling the variation.
    const V11Out a = v11_run(7, 0);
    const V11Out b = v11_run(7, 1);
    const V11Out c = v11_run(8, 0);
    std::printf("A7 V11: seed 7 pcm 0x%016llx state 0x%016llx cuts %llu; split pcm 0x%016llx; seed 8 pcm 0x%016llx\n",
                static_cast<unsigned long long>(a.pcm), static_cast<unsigned long long>(a.state),
                static_cast<unsigned long long>(a.cuts), static_cast<unsigned long long>(b.pcm),
                static_cast<unsigned long long>(c.pcm));
    CHECK(a.pcm == b.pcm);
    CHECK(a.state == b.state);
    CHECK(a.pcm != c.pcm);
    CHECK(a.cuts > 0);
    // Pinned (ps::math only): the voice output and state for seed 7.
    // Re-pinned by ps_audio A8 (F3: the FV Open-end radiation-impedance fix
    // changes every rendered sample at a radiating end - see pipe_network.cpp's
    // reservoir_face()/radiation_pressure() - so this MUST move; the
    // CycleEngine map-generator hash `mh` below is untouched by that fix and
    // stays pinned to its pre-A8 value, confirming the generator's engine
    // never runs through the audio radiation code path). Was
    // 0x4d31ad68a2491739 / 0x5fc7ec81b3fa76e3 before A8. Re-pinned by
    // ps_audio A8b (was 0xf6127b2e2ec390f8 / 0xf42b26f64d538437): the
    // catalyst monolith resistance (F1) and the reservoir face's inflow
    // branch (W2) change the fixture's exhaust and every port/volume face.
    CHECK(a.pcm == 0xe4e8cf0d04b01008ull);
    CHECK(a.state == 0xa798da1cb19bfb03ull);
    // The map generator's engine never sees a seed: a Preview CycleEngine
    // point's state hash is pinned (a generator call of the variation moves it).
    const drivetrain::CycleModelDesc& d = fixture();
    engine_cycle::OperatingPoint op;
    op.rpm = 3000.0;
    op.p_im_pa = 60000.0;
    op.t_im_k = 288.15;
    op.p_amb_pa = 101325.0;
    op.spark_deg_atdc = test::mbt_spark_deg_atdc(d);
    engine_cycle::CycleEngineSettings st;
    st.h_s = engine_cycle::kStepPreview;
    st.settling_cycles = engine_cycle::kEngineSettlingCyclesPreview;
    engine_cycle::CycleEngine ce(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, op, st);
    (void)ce.run();
    const std::uint64_t mh = ce.fold_state_hash(1469598103934665603ull);
    std::printf("A7 V11: CycleEngine Preview 3000 rpm / 60 kPa state 0x%016llx\n", static_cast<unsigned long long>(mh));
    // Re-pinned by ps_audio A8b (was 0x3d50ccb4707a1c36): the generator's
    // CycleEngine runs the same catalyst (F1) and reservoir faces (W2).
    CHECK(mh == 0xaa9bc382a11e1dbeull);
}

TEST_CASE("A7 V12a: an rpm ramp under a 1.5 % clock mismatch - firing frequency follows omega to 0.5 %",
          "[audio][cycle][pipe]") {
    // Catches: pitch taken from the clock / time-map rate instead of omega
    // (1.5 % off here), a wrong crank advance, dropped firing events.
    VoiceSettings vs = coarse_settings(1500.0, 60000.0);
    vs.engine.variation_scale = 0.0;
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v, 1.015);
    auto in = [](real t) {
        const real f = std::clamp((t - 0.2) / 1.0, 0.0, 1.0);
        return steady_input(t, 1500.0 + 3000.0 * f, 60000.0);
    };
    drv.run(1.3, in, nullptr);
    const ps_audio::LiveEngine& e = v.engine();
    const std::uint64_t n = e.firing_count();
    REQUIRE(n > 50);
    REQUIRE(n < static_cast<std::uint64_t>(ps_audio::LiveEngine::kFiringLog));
    // Reference: 180 deg over the interval at the mean of the endpoint omegas -
    // exact while omega is linear in time over the interval, off at the two
    // ramp corners (a kink inside the interval). "Smooth" intervals have the
    // same omega slope as both neighbours.
    auto slope = [&](std::uint64_t i) {
        const ps_audio::FiringEvent& a = e.firing(i - 1);
        const ps_audio::FiringEvent& b = e.firing(i);
        return (b.omega_rad_s - a.omega_rad_s) / (b.t_s - a.t_s);
    };
    real worst = 0.0, worst_smooth = 0.0;
    for (std::uint64_t i = 1; i < n; ++i) {
        const ps_audio::FiringEvent& a = e.firing(i - 1);
        const ps_audio::FiringEvent& b = e.firing(i);
        const real expect_deg_s = 0.5 * (a.omega_rad_s + b.omega_rad_s) * 360.0 / kTwoPi;
        const real err = std::fabs(180.0 / (b.t_s - a.t_s) / expect_deg_s - 1.0);
        worst = std::max(worst, err);
        const bool smooth = i >= 2 && i + 1 < n && std::fabs(slope(i) - slope(i - 1)) < 1.0 &&
                            std::fabs(slope(i) - slope(i + 1)) < 1.0;
        if (smooth) worst_smooth = std::max(worst_smooth, err);
    }
    const real first_omega = e.firing(0).omega_rad_s, last_omega = e.firing(n - 1).omega_rad_s;
    std::printf("A7 V12a: %llu firings, worst |f_fire / f(omega) - 1| %.3e (%.3e away from the ramp corners), "
                "omega %.1f -> %.1f rad/s, rate %.5f\n",
                static_cast<unsigned long long>(n), worst, worst_smooth, first_omega, last_omega, v.time_map().rate());
    CHECK(worst <= 0.005);
    CHECK(worst_smooth <= 0.001);
    CHECK(last_omega > 2.5 * first_omega);  // the ramp was actually rendered
    CHECK(std::fabs(v.time_map().rate() - 1.015) < 0.003);
}

TEST_CASE("A7 V12b: a 200 ms hitch holds the last inputs - no dropout, no step, no catch-up",
          "[audio][cycle][pipe]") {
    // Catches: silence or a restart on a hitch (dropout), a mute / state reset
    // or a catch-up burst on resume (step).
    VoiceSettings vs = coarse_settings(2500.0, 60000.0);
    vs.engine.variation_scale = 0.0;
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v);
    auto in = [](real t) { return steady_input(t, 2500.0, 60000.0); };
    std::vector<float> rec;
    drv.run(1.5, in, nullptr);  // past the start transient of the intake source
    drv.run(0.2, in, &rec);                     // reference
    drv.run(0.2, in, &rec, /*produce=*/false);  // hitch: the simulation stalls
    const std::uint64_t hitches = v.time_map().hitch_count();
    const bool hitched = v.time_map().hitched();
    const real omega_held = v.last_omega_applied();
    drv.run(0.2, in, &rec);  // resume
    const int ns = v.source_count();
    const std::size_t w = 2304;  // one engine cycle at 2500 rpm: steady windows have equal rms
    real worst_drop = 1e9, worst_step_ratio = 0.0;
    for (int s = 0; s < ns; ++s) {
        const std::vector<real> x = channel(rec, ns, s, 0, 28800);
        const real ref_rms = rms(x, 0, 9600);
        for (std::size_t i = 9600 - w / 2; i + w <= 28800; i += 240)
            worst_drop = std::min(worst_drop, rms(x, i, w) / ref_rms);
        real ref_step = 0.0, step = 0.0;
        for (std::size_t i = 1; i < 9600; ++i) ref_step = std::max(ref_step, std::fabs(x[i] - x[i - 1]));
        for (std::size_t i = 9600; i < 28800; ++i) step = std::max(step, std::fabs(x[i] - x[i - 1]));
        worst_step_ratio = std::max(worst_step_ratio, step / ref_step);
    }
    std::printf("A7 V12b: hitches %llu (hitched %d), omega held %.4f rad/s, worst one-cycle rms / reference %.3f, "
                "max sample step / reference max step %.3f\n",
                static_cast<unsigned long long>(hitches), hitched ? 1 : 0, omega_held, worst_drop, worst_step_ratio);
    CHECK(hitches == 1);
    CHECK(hitched);
    CHECK(std::fabs(omega_held - omega_of(2500.0)) < 1e-9);
    CHECK(worst_drop > 0.5);
    CHECK(worst_step_ratio < 1.5);
    CHECK(v.time_map().hitch_count() == 1);
    CHECK_FALSE(v.time_map().hitched());
}

TEST_CASE("A7 V12d: crank-phase correction pulls the crank onto the snapshot angle, capped at 1 % of omega",
          "[audio][cycle][pipe]") {
    // Catches: an uncapped correction (a pitch jump), a correction with the
    // wrong sign (never converges).
    VoiceSettings vs = coarse_settings(2000.0, 60000.0);
    vs.engine.variation_scale = 0.0;
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v);
    const real w = omega_of(2000.0);
    auto in = [w](real t) {
        EngineInput e = steady_input(t, 2000.0, 60000.0);
        const real deg = 90.0 + w * 360.0 / kTwoPi * t;
        e.crank_angle_deg = deg - 720.0 * std::floor(deg / 720.0);
        return e;
    };
    real worst_ratio = 0.0;
    real err = 0.0;
    for (int i = 0; i < 150; ++i) {  // 1.5 s in 10 ms steps
        drv.run(0.01, in, nullptr);
        worst_ratio = std::max(worst_ratio, std::fabs(v.last_omega_applied() / v.last_omega_input() - 1.0));
        err = v.crank_target_deg() - v.engine().crank_deg();
        while (err >= 360.0) err -= 720.0;
        while (err < -360.0) err += 720.0;
    }
    std::printf("A7 V12d: worst |omega_applied / omega - 1| %.3e, final phase error %.4f deg, locked %d\n", worst_ratio,
                err, v.time_map().locked() ? 1 : 0);
    CHECK(worst_ratio <= 0.01 + 1e-12);
    CHECK(worst_ratio > 0.009);  // the correction was active
    CHECK(std::fabs(err) < 1.0);
}

TEST_CASE("A7 V13: omega -> 0 leaves a decaying tail, no NaN", "[audio][cycle][pipe]") {
    // Catches: a division by speed (NaN / inf at rest), a stuck oscillation.
    VoiceSettings vs = coarse_settings(1500.0, 40000.0);
    vs.engine.variation_scale = 0.0;
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v);
    auto in = [](real t) {
        const real f = std::clamp((t - 0.4) / 0.3, 0.0, 1.0);
        return steady_input(t, 1500.0 * (1.0 - f), 40000.0);
    };
    std::vector<float> rec;
    drv.run(1.4, in, &rec);
    const real crank_a = v.engine().crank_deg();
    drv.run(0.05, in, &rec);
    const real crank_b = v.engine().crank_deg();
    const int ns = v.source_count();
    const bool finite = all_finite(rec);
    real run_rms = 0.0, tail_rms = 0.0;
    const std::size_t w = 4800;  // 100 ms
    for (int s = 0; s < ns; ++s) {
        const std::vector<real> x = channel(rec, ns, s, 0, rec.size() / static_cast<std::size_t>(ns));
        run_rms = std::max(run_rms, rms(x, static_cast<std::size_t>(0.2 * kFs), w));
        tail_rms = std::max(tail_rms, rms(x, x.size() - w, w));
    }
    std::printf("A7 V13: finite %d, running rms %.4e Pa, last 100 ms rms %.4e Pa (%.1f dB), crank %.6f -> %.6f deg\n",
                finite ? 1 : 0, run_rms, tail_rms, db(tail_rms / run_rms), crank_a, crank_b);
    CHECK(finite);
    CHECK(crank_a == crank_b);
    // At least 36 dB down 0.7 s after the stop (measured -42.0 dB; the
    // planned A7 bound was 26 dB). ps_audio A8b (F1): the -19.4 dB before
    // had two causes, each now fixed and each failing this bound alone:
    // the catalyst's resistance was purely flow-dependent (K rho |u_mean| / A,
    // 0 at rest), so a 25 Hz exhaust mode rang on (without its laminar
    // substrate resistance, cycle_engine.cpp: -25.4 dB); and the reservoir
    // faces' inflow branch had a velocity deadband/jump wherever the pipe and
    // reservoir temperatures differed, which kept a slowly decaying source
    // under that mode (tau ~22 s; with the old face: -27.0 dB). The exhaust
    // tail now decays with tau 0.6 s to 5e-14 Pa at 5 s.
    CHECK(tail_rms < 0.01585 * run_rms);  // 10^(-36/20)
}

TEST_CASE("A7 V13 diag: the stop tail per second in variants", "[.][diag][audio]") {
    // Variant 0 as V13; 1 catalyst k_loss 0 (no nonlinear waveguide resistance);
    // 2 plenum loop off; 3 audio-grade dx; 4 two substeps.
    for (int variant = 0; variant < 5; ++variant) {
        drivetrain::CycleModelDesc d = fixture();
        if (variant == 1)
            for (auto& el : d.installation->exhaust.elements) el.k_loss = 0.0;
        VoiceSettings vs = variant == 3 ? audio_settings(1500.0, 40000.0) : coarse_settings(1500.0, 40000.0);
        vs.engine.variation_scale = 0.0;
        if (variant == 2) vs.engine.plenum_loop = false;
        if (variant == 4) vs.substeps = 2;
        Voice v(d, test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
        Driver drv(v);
        auto in = [](real t) {
            const real f = std::clamp((t - 0.4) / 0.3, 0.0, 1.0);
            return steady_input(t, 1500.0 * (1.0 - f), 40000.0);
        };
        const int ns = v.source_count();
        std::printf("variant %d:", variant);
        for (int i = 0; i < 6; ++i) {
            std::vector<float> rec;
            drv.run(1.0, in, &rec);
            for (int s = 0; s < ns; ++s)
                std::printf(" %ds src%d %.2e", i + 1, s, rms(channel(rec, ns, s, 0, 48000), 0, 48000));
        }
        std::printf("\n");
    }
}

TEST_CASE("F1 diag: spectral character of the post-stop exhaust floor", "[.][diag][audio]") {
    // ps_audio A8 F1 diagnosis: out/audio_a7_progress.md's open finding
    // ("a residual exhaust floor ~0.06 Pa at 5 s, tau ~18 s", also
    // test_audio_voice.cpp's own "A7 V13 diag" variant sweep above) is
    // characterised here as tonal or broadband by scanning a 1 s window well
    // into the tail (t in [4, 5) s after V13's own stop input) for spectral
    // peaks. A residual RESONANT MODE (physical: the lowest mode of the
    // exhaust runner/primary/collector/waveguide chain has both radiation
    // loss (~ka^2, ka_low_freq is small) and Kirchhoff wall loss (~sqrt(f))
    // scaling DOWN at low frequency, so a low mode can plausibly ring for
    // many seconds) would show a SHARP, narrow, DOMINANT peak far above the
    // rest of the spectrum. Broadband, comparably-sized content across many
    // frequencies would instead point to noise/instability, not a resonance.
    VoiceSettings vs = coarse_settings(1500.0, 40000.0);
    vs.engine.variation_scale = 0.0;
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v);
    auto in = [](real t) {
        const real f = std::clamp((t - 0.4) / 0.3, 0.0, 1.0);
        return steady_input(t, 1500.0 * (1.0 - f), 40000.0);
    };
    std::vector<float> rec;
    drv.run(16.0, in, &rec);  // covers the stop (t~0.4-0.7s) plus a long floor region
    const int ns = v.source_count();
    // Continued-decay check (rules out a plateau / marginal instability that
    // a 1-6 s window alone could mistake for genuine light damping): source
    // 1's per-second RMS out to 15 s, plus a fitted tau from the last 10 s.
    {
        const std::vector<real> x1 = channel(rec, ns, 1, 0, rec.size() / static_cast<std::size_t>(ns));
        std::printf("F1 diag source 1 long tail:");
        std::vector<real> per_s;
        for (int sec = 1; sec <= 15; ++sec) {
            const real r = rms(x1, static_cast<std::size_t>((sec - 1) * kFs), static_cast<std::size_t>(kFs));
            per_s.push_back(r);
            std::printf(" %ds %.4e", sec, r);
        }
        std::printf("\n");
        const real r5 = per_s[4], r15 = per_s[14];
        const real tau = -10.0 / std::log(r15 / r5);
        std::printf("F1 diag source 1: 5s->15s ratio %.4f, fitted tau %.2f s (monotone decay: %d)\n", r15 / r5, tau,
                    r15 < r5 ? 1 : 0);
    }
    const std::size_t win0 = static_cast<std::size_t>(4.0 * kFs), win_len = static_cast<std::size_t>(1.0 * kFs);
    for (int s = 0; s < ns; ++s) {
        const std::vector<real> x = channel(rec, ns, s, win0, win_len);
        const real floor_rms = rms(x, 0, win_len);
        real peak_amp = 0.0, peak_f = 0.0, sum_amp = 0.0;
        int n_bins = 0;
        for (real f = 10.0; f <= 3000.0; f += 5.0) {
            const real cycles = f * static_cast<real>(win_len) / kFs;
            const real a = goertzel_amp(x, cycles);
            sum_amp += a;
            ++n_bins;
            if (a > peak_amp) {
                peak_amp = a;
                peak_f = f;
            }
        }
        const real mean_amp = sum_amp / static_cast<real>(n_bins);
        std::printf(
            "F1 diag source %d: floor rms %.4e Pa, peak %.4e Pa at %.0f Hz, mean-bin %.4e Pa (peak/mean %.1fx, "
            "%.1f dB)\n",
            s, floor_rms, peak_amp, peak_f, mean_amp, peak_amp / mean_amp, db(peak_amp / mean_amp));
    }
}

// ps_audio A8b F1: the lowest exhaust mode's Q per loss element. The
// fixture's own engine network (make_engine_pipe_network_desc, audio-path
// radiation on) with every cylinder port closed (valves shut, engine
// stopped), rung by a +200 Pa pressure step in the collector and read at the
// collector's last FV cell. Frequency: the Goertzel peak over 10..60 Hz of a
// 0.8 s window; decay: the log-amplitude regression over 0.4 s Hann windows.
namespace {
struct ModeFit {
    real f_hz = 0.0;
    real sigma = 0.0;  // amplitude decay rate [1/s]
};

ModeFit ring_down(engine_cycle::PipeNetworkDesc nd, real t_end_s, bool print) {
    using engine_cycle::PipeEndDesc;
    for (engine_cycle::PipeDesc& p : nd.pipes) {
        if (p.start.kind == PipeEndDesc::Kind::Port) p.start = PipeEndDesc{};
        if (p.end.kind == PipeEndDesc::Kind::Port) p.end = PipeEndDesc{};
    }
    nd.ports.clear();
    engine_cycle::PipeNetwork net(nd, 1.0 / kFs);
    int coll = -1;
    for (int k = 0; k < static_cast<int>(nd.pipes.size()); ++k) {
        const engine_cycle::PipeDesc& p = nd.pipes[static_cast<std::size_t>(k)];
        if (p.end.kind == PipeEndDesc::Kind::Waveguide) coll = k;
    }
    REQUIRE(coll >= 0);
    const int nc = net.cell_count(coll);
    for (int i = 0; i < nc; ++i) {
        const engine_cycle::PipeCellState c = net.cell(coll, i);
        net.set_cell(coll, i, c.p + 200.0, c.t, 0.0, c.x_b);
    }
    const std::size_t n = static_cast<std::size_t>(t_end_s * kFs);
    std::vector<real> x(n);
    for (std::size_t s = 0; s < n; ++s) {
        net.step();
        x[s] = net.cell(coll, nc - 1).p;
    }
    real mean = 0.0;
    for (real v : x) mean += v;
    mean /= static_cast<real>(n);
    for (real& v : x) v -= mean;
    ModeFit fit;
    {
        const std::size_t i0 = static_cast<std::size_t>(0.1 * kFs);
        const std::size_t w = static_cast<std::size_t>(0.8 * kFs);
        std::vector<real> seg(x.begin() + static_cast<std::ptrdiff_t>(i0),
                              x.begin() + static_cast<std::ptrdiff_t>(i0 + w));
        real best = 0.0;
        for (real f = 10.0; f <= 60.0; f += 0.05) {
            const real a = goertzel_amp(seg, f * 0.8);
            if (a > best) {
                best = a;
                fit.f_hz = f;
            }
        }
    }
    const std::size_t w = static_cast<std::size_t>(0.4 * kFs);
    real st = 0.0, sy = 0.0, stt = 0.0, sty = 0.0;
    int m = 0;
    for (real t0 = 0.1; t0 + 0.4 <= t_end_s + 1e-9; t0 += 0.2) {
        const std::size_t i0 = static_cast<std::size_t>(t0 * kFs);
        if (i0 + w > n) break;
        std::vector<real> seg(w);
        for (std::size_t k = 0; k < w; ++k)
            seg[k] = x[i0 + k] * 0.5 * (1.0 - ps::math::cos(kTwoPi * static_cast<real>(k) / static_cast<real>(w)));
        // Peak over f0 +- 4 Hz per window: the mean state relaxes after the
        // step, so the mode frequency drifts by ~1 Hz and a fixed-frequency
        // Goertzel read a drift as decay (lossless variant sigma 5.8 1/s).
        real a = 0.0, fw = fit.f_hz;
        for (real ff = fit.f_hz - 4.0; ff <= fit.f_hz + 4.0; ff += 0.05) {
            const real aa = goertzel_amp(seg, ff * 0.4);
            if (aa > a) {
                a = aa;
                fw = ff;
            }
        }
        if (!(a > 1e-7)) break;
        const real t = t0 + 0.2, y = std::log(a);
        if (print) std::printf(" [%.1f s %.2f Hz %.3e]", t, fw, a);
        st += t;
        sy += y;
        stt += t * t;
        sty += t * y;
        ++m;
    }
    if (m >= 2) fit.sigma = -(m * sty - st * sy) / (m * stt - st * st);
    return fit;
}
}  // namespace

TEST_CASE("A8b F1 diag: lowest exhaust mode Q per loss element", "[.][diag][audio]") {
    using engine_cycle::EngineNetworkLayout;
    using engine_cycle::GasStateDesc;
    using engine_cycle::PipeDesc;
    using engine_cycle::PipeNetworkDesc;
    using engine_cycle::WallFriction;
    using engine_cycle::WgLoss;
    using engine_cycle::WgNodeDesc;
    using engine_cycle::WgSegmentDesc;
    const GasStateDesc amb{101325.0, 293.15, 0.0};
    // Every variant but the first sets the catalyst's k_loss to 0: the
    // quadratic term's R = K rho |u_mean| / A follows the mean flow the step
    // excitation drains through the tailpipe and would confound the linear
    // losses (the pre-A8b version of this diag read sigma 5.8 1/s with every
    // loss off for exactly that reason).
    const char* names[] = {"k_loss 1.5, all losses",   "all linear losses",        "monolith off (pre-A8b)",
                           "monolith off, rad off",    "monolith off, Kirchhoff off", "monolith off, FV friction off",
                           "monolith only",            "all losses off"};
    for (int variant = 0; variant < 8; ++variant) {
        EngineNetworkLayout lay;
        drivetrain::CycleModelDesc d = fixture();
        if (variant > 0)
            for (auto& el : d.installation->exhaust.elements) el.k_loss = 0.0;
        const bool rad = !(variant == 3 || variant >= 6);
        PipeNetworkDesc nd = make_engine_pipe_network_desc(d, amb, 40000.0, 0.05, 0.05, &lay, rad);
        if (!rad)
            for (WgNodeDesc& nn : nd.waveguide.nodes) nn.radiation = false;
        if ((variant >= 2 && variant <= 5) || variant == 7)
            for (WgNodeDesc& nn : nd.waveguide.nodes) nn.monolith_length_m = 0.0;
        if (variant == 4 || variant >= 6)
            for (WgSegmentDesc& sg : nd.waveguide.segments) sg.loss = WgLoss::None;
        if (variant == 5 || variant >= 6)
            for (PipeDesc& p : nd.pipes) p.wall.friction = WallFriction::None;
        std::printf("A8b F1 windows %s:", names[variant]);
        const ModeFit f = ring_down(nd, 3.0, true);
        std::printf("\n");
        std::printf("A8b F1 Q: %-32s f %.2f Hz sigma %.4f 1/s Q %.1f\n", names[variant], f.f_hz, f.sigma,
                    f.sigma > 0.0 ? kPi * f.f_hz / f.sigma : -1.0);
    }
}

// ps_audio A8 V9/V10: flow-noise level (spec section on turbulence noise,
// voice.h's own FlowNoiseSettings) and a coarse radiated-power/full-scale
// sanity bound (spec section 8's Calibration). `level_coeff_pa` defaults to
// 0.0 (off). ps_audio A8b: no test sets a level-bearing flow-noise
// coefficient any more - the A8 pass-by constant (1.3e-6 Pa per (m/s)^3,
// chosen so a WOT pass-by landed near 75-80 dB) was a fit, not a physics
// check. The absolute radiated level is now checked physics-only by
// "A8b V9" (the monopole identity at the outlet), and V10 runs with flow
// noise off. The only remaining coefficient, kV9FlowNoiseCoeffPa, is an
// exaggerated [A] value used by the shape-only monotonicity check below
// (and its sabotage diag); it makes no level claim and is not shipped.
namespace {
// [A] exaggerated, Pa per (m/s)^3 at 1 m, shape-only - see above.
constexpr real kV9FlowNoiseCoeffPa = 0.02;

// Runs one settle + 1 s measurement window at (rpm, p_im) with
// `level_coeff_pa` and returns the exhaust channel's RMS over the last 0.8 s
// (0.2 s skip lets the flow-noise filter cascade's own state settle).
real v9_exhaust_rms(real rpm, real p_im, real level_coeff_pa) {
    VoiceSettings vs = coarse_settings(rpm, p_im);
    vs.engine.variation_scale = 0.0;
    vs.flow_noise.level_coeff_pa = level_coeff_pa;
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    const int ex = exhaust_source_index(v);
    REQUIRE(ex >= 0);
    Driver drv(v);
    auto in = [&](real t) { return steady_input(t, rpm, p_im); };
    drv.run(0.3, in, nullptr);
    std::vector<float> rec;
    drv.run(1.0, in, &rec);
    const int ns = v.source_count();
    const std::vector<real> x = channel(rec, ns, ex, 0, rec.size() / static_cast<std::size_t>(ns));
    const std::size_t skip = static_cast<std::size_t>(0.2 * kFs);
    return rms(x, skip, x.size() - skip);
}
}  // namespace

TEST_CASE("A8 V9: flow-noise level increases monotonically with load and rpm (shape only)",
          "[audio][cycle][pipe][long]") {
    // Isolated flow-noise contribution at each point: RMS with the layer on
    // minus RMS with it off, combined in QUADRATURE (the noise is an
    // independent additive term on top of the same periodic exhaust pulse,
    // same seed/state otherwise) - isolates the broadband term the brief
    // asks to be monotone, rather than the whole engine note (whose
    // harmonic content is not guaranteed monotone through pipe resonances).
    struct Point {
        real rpm, p_im;
    };
    const std::vector<Point> pts = {
        {1200.0, 25000.0}, {2500.0, 45000.0}, {4000.0, 65000.0}, {6000.0, 95000.0}};
    std::vector<real> noise_rms;
    for (const auto& pt : pts) {
        const real with_r = v9_exhaust_rms(pt.rpm, pt.p_im, kV9FlowNoiseCoeffPa);
        const real without_r = v9_exhaust_rms(pt.rpm, pt.p_im, 0.0);
        const real noise_power = std::max(0.0, with_r * with_r - without_r * without_r);
        noise_rms.push_back(std::sqrt(noise_power));
        std::printf("A8 V9 rpm %.0f p_im %.0f: with %.4e Pa, without %.4e Pa, isolated flow-noise %.4e Pa\n", pt.rpm,
                    pt.p_im, with_r, without_r, noise_rms.back());
    }
    for (std::size_t i = 1; i < noise_rms.size(); ++i) {
        CHECK(noise_rms[i] > noise_rms[i - 1]);
    }
    // ps_audio A8b: the A8 WOT pass-by estimate that followed here (a
    // 60-90 dB band reached through the fitted 1.3e-6 coefficient) is gone;
    // "A8b V9" checks the level physics-only.
}

TEST_CASE("A8b V9: WOT order-2 outlet pressure equals the monopole estimate from the outlet volume velocity",
          "[audio][cycle][pipe]") {
    // ps_audio A8b point 3: the radiated level validated from physics only,
    // flow noise OFF (FlowNoiseSettings::level_coeff_pa default 0 - no
    // chosen coefficient anywhere in this check). At a steady WOT point the
    // exhaust source's order-2 pressure at 1 m must equal an INDEPENDENT
    // simple-source estimate |p| = rho_amb * omega * |Q| / (4 pi r) (Kinsler
    // et al., Fundamentals of Acoustics, 4th ed. 2000 [A: simple source],
    // r = 1 m, rho_amb = p_amb / (R_air T_amb) with R_air = 287.05 J/(kg K)
    // computed here, not read from the engine) from the outlet's own volume
    // velocity Q (EngineCore::radiating_outflow, the FV/WG pipe physics'
    // output). Catches a calibration that drops rho or 4 pi (sabotage table:
    // measured: 4 pi dropped reads 12.52x, rho dropped 0.816x), a wrong reference
    // distance or a mis-scaled derivative. The absolute WOT level itself is
    // printed (unweighted, 1 m and a 7.5 m 1/r pass-by proxy) - no cited
    // measured tailpipe level for a comparable I4 was found open access, see
    // out/audio_a8b_progress.md.
    const real rpm = 6000.0, p_im = 95000.0;
    VoiceSettings vs = coarse_settings(rpm, p_im);
    vs.engine.variation_scale = 0.0;
    REQUIRE(vs.flow_noise.level_coeff_pa == 0.0);
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    const int ex = exhaust_source_index(v);
    REQUIRE(ex >= 0);
    Driver drv(v);
    auto in = [&](real t) { return steady_input(t, rpm, p_im); };
    drv.run(0.5, in, nullptr);
    const int n_cycles = 25;  // 0.5 s at 6000 rpm
    const std::size_t n = static_cast<std::size_t>(n_cycles * 2 * 60.0 / rpm * kFs + 0.5);
    const int ns = v.source_count();
    std::vector<real> p(n), q(n);
    std::vector<float> rec;
    for (std::size_t k = 0; k < n; ++k) {
        rec.clear();
        drv.run(1.0 / kFs, in, &rec);
        REQUIRE(rec.size() == static_cast<std::size_t>(ns));
        p[k] = rec[static_cast<std::size_t>(ex)];
        q[k] = v.engine().core().radiating_outflow(ex);
    }
    const real omega2 = 2.0 * omega_of(rpm);
    const real rho_amb = vs.engine.p_amb_pa / (287.05 * vs.engine.t_amb_k);
    const real q2 = goertzel_amp(q, 4.0 * n_cycles);
    const real p2 = goertzel_amp(p, 4.0 * n_cycles);
    const real p2_est = rho_amb * omega2 * q2 / (4.0 * kPi * 1.0);
    const real p_rms = rms(p, 0, n);
    std::printf("A8b V9: order-2 |Q| %.4e m^3/s, |p| %.4e Pa vs monopole %.4e Pa (ratio %.5f); rms %.4e Pa = %.1f dB "
                "at 1 m, %.1f dB at 7.5 m (unweighted, 1/r)\n",
                q2, p2, p2_est, p2 / p2_est, p_rms, db(p_rms / 20e-6), db(p_rms / 7.5 / 20e-6));
    CHECK(std::fabs(p2 / p2_est - 1.0) < 0.01);
    CHECK(p2 > 0.0);
}

TEST_CASE("A8b HF diag: WOT octave-band levels at the default audio grid vs a halved grid and step",
          "[.][diag][audio]") {
    // Coordinator question (listening test): almost no energy above 2 kHz.
    // Discriminator: the same steady WOT point rendered at the default audio
    // grid (dx 0.032 / 0.017 m, h = 1/48000) and at half the grid AND half
    // the step (dx 0.016 / 0.0085 m, substeps 2, same CFL). A band that gains
    // many dB on the finer grid is being removed by the FV scheme's numerical
    // dissipation, not by the modelled physics. Engine-cycle harmonics
    // (variation off: an exactly periodic steady state) summed per octave.
    const real rpm = 6000.0, p_im = 95000.0;
    const int n_cycles = 50;  // 1 s
    const real bands[] = {125, 250, 500, 1000, 2000, 4000, 8000};
    for (int grid = 0; grid < 3; ++grid) {
        VoiceSettings vs = coarse_settings(rpm, p_im);
        vs.engine.variation_scale = 0.0;
        vs.engine.plenum_loop = false;
        if (grid == 0) {
            vs.engine.dx_exhaust_m = 0.032;
            vs.engine.dx_intake_m = 0.017;
        } else if (grid == 1) {
            vs.substeps = 2;
            vs.engine.dx_exhaust_m = 0.016;
            vs.engine.dx_intake_m = 0.0085;
        } else {
            vs.substeps = 2;
            vs.engine.dx_exhaust_m = 0.032;
            vs.engine.dx_intake_m = 0.017;
        }
        Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
        Driver drv(v);
        auto in = [&](real t) { return steady_input(t, rpm, p_im); };
        drv.run(0.5, in, nullptr);
        std::vector<float> rec;
        const std::size_t n = static_cast<std::size_t>(n_cycles * 120.0 / rpm * kFs + 0.5);
        drv.run(static_cast<real>(n) / kFs, in, &rec);
        const int ns = v.source_count();
        for (int s = 0; s < ns; ++s) {
            const std::vector<real> x = channel(rec, ns, s, 0, n);
            std::printf("A8b HF grid %d (%s) source %d (%s):", grid,
                        grid == 0 ? "dx 0.032/0.017, h 1/48k" : (grid == 1 ? "dx 0.016/0.0085, h 1/96k" : "dx 0.032/0.017, h 1/96k"),
                        s, v.source(s).intake ? "intake" : "exhaust");
            for (int b = 0; b < 7; ++b) {
                const real lo = bands[b] / std::sqrt(2.0), hi = bands[b] * std::sqrt(2.0);
                real e = 0.0;
                for (int k = 1;; ++k) {
                    const real f = k * rpm / 120.0;
                    if (f >= hi) break;
                    if (f < lo) continue;
                    const real a = goertzel_amp(x, static_cast<real>(k * n_cycles));
                    e += 0.5 * a * a;
                }
                std::printf(" %gHz %.1fdB", bands[b], 10.0 * std::log10(e / (20e-6 * 20e-6) + 1e-30));
            }
            std::printf("\n");
        }
    }
}

TEST_CASE("A8 V9 sabotage diag: calibration off by rho or 4pi", "[.][diag][audio]") {
    // Demonstrates the radiated-power check below (V10) is discriminating:
    // an omitted rho*c in the intensity formula, or an omitted 4*pi*r^2 in
    // the power formula, each move the "radiated power" estimate by a large,
    // easily-distinguished factor rather than a rounding-level difference.
    const real p_rms = 2.0;  // representative WOT exhaust RMS order of magnitude, Pa at 1 m
    const real rho = 1.2, c = 343.0, r = 1.0;
    const real correct_power = 4.0 * kPi * r * r * (p_rms * p_rms) / (rho * c);
    const real missing_rho_c = 4.0 * kPi * r * r * (p_rms * p_rms);       // rho*c dropped entirely
    const real missing_4pi = (p_rms * p_rms) / (rho * c);                 // 4*pi*r^2 dropped entirely
    std::printf(
        "A8 V9 sabotage diag: correct radiated power %.4e W, missing rho*c %.4e W (%.1fx), missing 4pi*r^2 %.4e W "
        "(%.1fx)\n",
        correct_power, missing_rho_c, missing_rho_c / correct_power, missing_4pi, missing_4pi / correct_power);
}

TEST_CASE("A8 V10: radiated power bound, no NaN/Inf, peak below full scale at WOT redline",
          "[audio][cycle][pipe][long]") {
    const real rpm = 6500.0, p_im = 98000.0;  // near-redline WOT for this [AS] fixture
    VoiceSettings vs = coarse_settings(rpm, p_im);
    vs.engine.variation_scale = 0.0;
    // ps_audio A8b: flow noise off (the default) - the A8 version set the
    // fitted pass-by coefficient here; the bounds below now see the modelled
    // pipe physics only.
    REQUIRE(vs.flow_noise.level_coeff_pa == 0.0);
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v);
    auto in = [&](real t) { return steady_input(t, rpm, p_im); };
    drv.run(0.3, in, nullptr);
    std::vector<float> rec;
    drv.run(1.0, in, &rec);

    // No NaN/Inf anywhere in the rendered buffer (every source, every frame).
    bool all_finite = true;
    for (float s : rec) all_finite = all_finite && std::isfinite(s);
    CHECK(all_finite);

    const int ns = v.source_count();
    const int ex = exhaust_source_index(v);
    REQUIRE(ex >= 0);
    const std::vector<real> x = channel(rec, ns, ex, 0, rec.size() / static_cast<std::size_t>(ns));
    const std::size_t skip = static_cast<std::size_t>(0.2 * kFs);
    real peak = 0.0;
    for (std::size_t i = skip; i < x.size(); ++i) peak = std::max(peak, std::abs(x[i]));
    const real p_rms = rms(x, skip, x.size() - skip);

    // Peak below full scale at the default calibration (source.h's
    // Calibration::pa_full_scale = 200 Pa at 1 m).
    const real pa_full_scale = 200.0;  // ps_audio::Calibration{}.pa_full_scale, mirrored here to avoid a new include
    std::printf("A8 V10 WOT redline: peak %.4e Pa (full scale %.1f Pa), rms %.4e Pa\n", peak, pa_full_scale, p_rms);
    CHECK(peak < pa_full_scale);

    // Radiated acoustic power (monopole estimate at the 1 m reference,
    // I = p_rms^2/(rho*c), P = 4*pi*r^2*I [A]) stays under 1e-2 of an
    // ASSUMED [AS] order-of-magnitude exhaust enthalpy flow for a small NA
    // I4 near WOT redline (Heywood-style exhaust energy fraction ~30-40 % of
    // fuel input; fuel input order tens of kW at this size/speed) - 20 kW,
    // deliberately coarse per the brief's own "< 1e-2" wording, not a fitted
    // number for this specific fixture.
    const real rho = 1.2, c = 343.0, r_ref = 1.0;
    const real radiated_power_w = 4.0 * kPi * r_ref * r_ref * (p_rms * p_rms) / (rho * c);
    const real assumed_exhaust_enthalpy_w = 20000.0;  // [AS]
    const real ratio = radiated_power_w / assumed_exhaust_enthalpy_w;
    std::printf("A8 V10 radiated power %.4e W, assumed exhaust enthalpy flow %.1f W, ratio %.4e\n", radiated_power_w,
                assumed_exhaust_enthalpy_w, ratio);
    CHECK(ratio < 1e-2);
}

TEST_CASE("A8 V10 sabotage diag: missing loss term (cranked flow-noise coefficient)", "[.][diag][audio]") {
    // Simulates "a missing loss/clamp term" by switching on a large
    // flow-noise term (V10 itself runs with it off) - demonstrates the
    // peak-below-full-scale and radiated-power checks both catch it.
    const real rpm = 6500.0, p_im = 98000.0;
    VoiceSettings vs = coarse_settings(rpm, p_im);
    vs.engine.variation_scale = 0.0;
    vs.flow_noise.level_coeff_pa = kV9FlowNoiseCoeffPa;  // sabotage: the exaggerated shape-test coefficient
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v);
    auto in = [&](real t) { return steady_input(t, rpm, p_im); };
    drv.run(0.3, in, nullptr);
    std::vector<float> rec;
    drv.run(1.0, in, &rec);
    const int ns = v.source_count();
    const int ex = exhaust_source_index(v);
    REQUIRE(ex >= 0);
    const std::vector<real> x = channel(rec, ns, ex, 0, rec.size() / static_cast<std::size_t>(ns));
    const std::size_t skip = static_cast<std::size_t>(0.2 * kFs);
    real peak = 0.0;
    for (std::size_t i = skip; i < x.size(); ++i) peak = std::max(peak, std::abs(x[i]));
    const real p_rms = rms(x, skip, x.size() - skip);
    const real rho = 1.2, c = 343.0, r_ref = 1.0;
    const real radiated_power_w = 4.0 * kPi * r_ref * r_ref * (p_rms * p_rms) / (rho * c);
    std::printf("A8 V10 sabotage diag: peak %.4e Pa (full scale 200 Pa, %s), radiated power %.4e W (bound 200 W, %s)\n",
                peak, peak < 200.0 ? "OK" : "EXCEEDS", radiated_power_w, radiated_power_w < 200.0 ? "OK" : "EXCEEDS");
}

TEST_CASE("A7 tie: audio-grade cycle-mean fresh air within 3 % of the power-grade engine", "[audio][cycle][pipe]") {
    // Catches: an audio engine that is not the map generator's engine (a
    // different runner, valve or combustion model), a plenum loop that misses
    // its pressure target.
    const real pts[3][2] = {{1500.0, 30000.0}, {3000.0, 60000.0}, {4500.0, 90000.0}};
    for (const auto& p : pts) {
        const TieOut t = tie_point(fixture(), p[0], p[1]);
        const real rel = t.live_air / t.power_air - 1.0;
        std::printf("A7 tie: %.0f rpm / %.0f Pa: audio %.6e kg, power %.6e kg (%+.3f %%); plenum audio %.0f Pa, power %.0f Pa\n",
                    p[0], p[1], t.live_air, t.power_air, 100.0 * rel, t.live_plenum, t.power_plenum);
        CHECK(std::fabs(rel) <= 0.03);
    }
}

TEST_CASE("A7 tie sabotage diag: a longer audio runner breaks the tie", "[.][diag][audio]") {
    const drivetrain::CycleModelDesc live = test::make_pipe_i4_cycle_desc(0.45, 0.60);
    const real pts[3][2] = {{1500.0, 30000.0}, {3000.0, 60000.0}, {4500.0, 90000.0}};
    for (const auto& p : pts) {
        const TieOut t = tie_point(live, p[0], p[1]);
        std::printf("A7 tie sabotage (runner 0.45 m): %.0f rpm / %.0f Pa: %+.3f %%\n", p[0], p[1],
                    100.0 * (t.live_air / t.power_air - 1.0));
    }
}

TEST_CASE("A7 Z1: the render path allocates nothing after construction", "[audio][cycle][pipe]") {
    // Catches: a heap allocation per sample or per block (voice, engine, runner),
    // including ignition cuts and a pause / resume.
    if (!ps::jobs::debug_alloc::hook_available()) {
        SUCCEED("debug_alloc hook unavailable (sanitizer build)");
        return;
    }
    VoiceSettings vs = coarse_settings(2000.0, 50000.0);
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    ps_audio::Runner runner({&v});
    std::vector<float> buf(static_cast<std::size_t>(runner.block_frames() * runner.channel_count()));
    real t = 0.0;
    int tick = 0;
    auto feed = [&] {
        EngineInput e = steady_input(t, 2000.0, 50000.0);
        e.flags = (tick % 50) < 5 ? ps_audio::kInputIgnitionCut : 0u;
        v.push_input(e);
        t += 1.0 / 240.0;
        ++tick;
    };
    auto blocks = [&](int n) {
        for (int i = 0; i < n; ++i) {
            if (i % 2 == 0) feed();  // ~one snapshot per 1/240 s of audio (256 frames = 5.3 ms)
            runner.render_block();
            runner.read(buf.data(), static_cast<std::size_t>(runner.block_frames()));
            if (i == 30) v.set_paused(true);
            if (i == 45) v.set_paused(false);
        }
    };
    blocks(20);  // warm-up (outside a job)
    ps::jobs::JobSystem js(2);
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    {
        ps::jobs::JobSystem::WaitGroup wg;
        js.submit([&] { blocks(60); }, &wg);
        js.wait(wg);
    }
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t n = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    std::printf("A7 Z1: %llu allocations in 60 blocks (ignition cuts, pause and resume included), cuts %llu\n",
                static_cast<unsigned long long>(n), static_cast<unsigned long long>(v.engine().cut_events()));
    CHECK(n == 0);
    CHECK(v.engine().cut_events() > 0);
}

TEST_CASE("A7 P1: pause fades out over 50 ms and freezes, resume re-seats and fades in", "[audio][cycle][pipe]") {
    // Catches: a hard mute (click), physics running while paused, a resume
    // that catches up on the paused simulation time.
    VoiceSettings vs = coarse_settings(2000.0, 50000.0);
    vs.engine.variation_scale = 0.0;
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    Driver drv(v);
    auto in = [](real t) { return steady_input(t, 2000.0, 50000.0); };
    drv.run(0.3, in, nullptr);
    v.set_paused(true);
    std::vector<float> rec;
    drv.run(0.1, in, &rec);
    const real t_engine = v.engine().time_s();
    CHECK(v.frozen());
    drv.run(0.1, in, &rec);
    CHECK(v.engine().time_s() == t_engine);  // physics frozen
    v.set_paused(false);
    drv.run(0.1, in, &rec);
    CHECK_FALSE(v.frozen());
    CHECK(v.gain() == 1.0);
    CHECK(v.time_map().lag_intervals() < 2.0);  // re-seated, no backlog
    const int ns = v.source_count();
    const std::size_t nsu = static_cast<std::size_t>(ns);
    real early = 0.0;
    for (int s = 0; s < ns; ++s) early = std::max(early, rms(channel(rec, ns, s, 0, 480), 0, 480));
    bool zero_after = true;
    for (std::size_t i = static_cast<std::size_t>(0.06 * kFs) * nsu; i < static_cast<std::size_t>(0.2 * kFs) * nsu; ++i)
        zero_after = zero_after && rec[i] == 0.0f;
    real resumed = 0.0;
    for (int s = 0; s < ns; ++s)
        resumed = std::max(resumed, rms(channel(rec, ns, s, static_cast<std::size_t>(0.25 * kFs), 2400), 0, 2400));
    std::printf("A7 P1: first 10 ms after pause rms %.4e Pa, silent 60-200 ms %d, rms after resume %.4e Pa\n", early,
                zero_after ? 1 : 0, resumed);
    CHECK(early > 0.0);
    CHECK(zero_after);
    CHECK(resumed > 0.0);
}

TEST_CASE("A7 R1: the threaded Runner renders while a producer thread pushes snapshots", "[audio][cycle][pipe]") {
    // The TSan leg's subject: the SPSC queue, the float ring, the pause flag,
    // start / stop.
    VoiceSettings vs = coarse_settings(2000.0, 50000.0);
    Voice v(fixture(), test::kCycleFixtureLhv, test::kCycleFixtureAfr, vs);
    ps_audio::Runner runner({&v});
    std::atomic<bool> done{false};
    std::thread producer([&] {
        real t = 0.0;
        while (!done.load(std::memory_order_acquire)) {
            v.push_input(steady_input(t, 2000.0, 50000.0));
            t += 1.0 / 240.0;
            v.set_paused(t > 0.05 && t < 0.1);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    });
    runner.start();
    std::vector<float> buf(static_cast<std::size_t>(512 * runner.channel_count()));
    std::size_t frames = 0;
    bool finite = true;
    // ps_audio A8 (coordinator fix, 2026-09-28): this loop's exit condition
    // used to be a fixed 9600-frame target within a 30 s wall-clock cap - a
    // race against machine speed, not against the code under test. On the
    // linux-asan-ubsan leg running beside linux-tsan (both loaded, ASan/UBSan
    // alone already 5-15x slower per CLAUDE.md's "[long]" policy) that read
    // only 6912/9600 frames in 30 s and failed CHECK(frames >= 9600), while
    // release and TSan alone passed - a genuine wall-clock shortfall, not a
    // defect (the sanitizer legs never assert bench_* wall-clock bounds for
    // the same reason). Fixed by lowering the frame target to a few blocks
    // (still real concurrent SPSC push/read/pause-flag traffic - R1 only
    // checks safety/finiteness/accounting, not audio content, so a smaller
    // sample does not weaken the assertions below) and raising the wall-clock
    // budget to 120 s so a genuinely stalled render thread (the sabotage:
    // render_block() never called, blocks_rendered() stays 0) still reliably
    // fails instead of quietly passing.
    const std::size_t target_frames = static_cast<std::size_t>(runner.block_frames()) * 8;
    const auto t0 = std::chrono::steady_clock::now();
    while (frames < target_frames && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(120)) {
        const std::size_t n = runner.read(buf.data(), 512);
        for (std::size_t i = 0; i < n * static_cast<std::size_t>(runner.channel_count()); ++i)
            finite = finite && std::isfinite(buf[i]);
        frames += n;
        if (n == 0) std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    runner.stop();
    done.store(true, std::memory_order_release);
    producer.join();
    std::printf("A7 R1: %zu frames read (target %zu), %llu blocks rendered\n", frames, target_frames,
                static_cast<unsigned long long>(runner.blocks_rendered()));
    CHECK(frames >= target_frames);
    CHECK(finite);
    CHECK(runner.blocks_rendered() * static_cast<std::uint64_t>(runner.block_frames()) >= frames);
}
