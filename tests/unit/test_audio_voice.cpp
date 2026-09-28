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
    CHECK(a.pcm == 0x4d31ad68a2491739ull);
    CHECK(a.state == 0x5fc7ec81b3fa76e3ull);
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
    CHECK(mh == 0x3d50ccb4707a1c36ull);
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
    // At least 12 dB down 0.7 s after the stop (measured -19.4 dB). It does not
    // ring out to silence: a residual exhaust floor (~0.06 Pa at 5 s, tau ~18 s)
    // is an open finding - see out/audio_a7_progress.md "Findings" and the
    // "A7 V13 diag" case.
    CHECK(tail_rms < 0.25 * run_rms);
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
    const auto t0 = std::chrono::steady_clock::now();
    while (frames < 9600 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(30)) {
        const std::size_t n = runner.read(buf.data(), 512);
        for (std::size_t i = 0; i < n * static_cast<std::size_t>(runner.channel_count()); ++i)
            finite = finite && std::isfinite(buf[i]);
        frames += n;
        if (n == 0) std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    runner.stop();
    done.store(true, std::memory_order_release);
    producer.join();
    std::printf("A7 R1: %zu frames read, %llu blocks rendered\n", frames,
                static_cast<unsigned long long>(runner.blocks_rendered()));
    CHECK(frames >= 9600);
    CHECK(finite);
    CHECK(runner.blocks_rendered() * static_cast<std::uint64_t>(runner.block_frames()) >= frames);
}
