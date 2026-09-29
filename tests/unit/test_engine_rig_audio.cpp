// tests/unit/test_engine_rig_audio.cpp - ps_audio A10 (out/audio_plan.md
// section 6): the public ps::EngineRig against the shipped demo engine
// (data/engines/demo_i4_pipes.json, data/installations/demo_i4_exhaust.json)
// - free rev to the limiter and back, starter -> Running, a stall, the
// EngineRig -> ps_audio::EngineInput -> Voice glue (the same conversion
// tools/ps_engine_audio/main.cpp uses), ledger closure and zero allocation.
// R1 (free rev), R2 (starter), R3 (stall), R4 (WAV-render determinism: same
// seed twice bit-identical, different seed differs, an unseeded-variation
// sabotage), R5 (ledger balance from tick 1, vault PHYS-049 F-T0), R6 (zero
// allocation per step once built) - each with its own sabotage noted inline.
#include "ps/drivetrain/engine_rig.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps_audio/engine_input.h"
#include "ps_audio/voice.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <variant>
#include <vector>

using namespace ps;
using ps_audio::EngineInput;
using ps_audio::Voice;
using ps_audio::VoiceSettings;

namespace {

std::string demo_engine_path() { return std::string(PS_SOURCE_DIR) + "/data/engines/demo_i4_pipes.json"; }

// Same conversion as tools/ps_engine_audio/main.cpp's make_engine_input -
// duplicated here (not shared via a header) since it is glue code that
// belongs in a tool, not a library (see engine_rig.h's own header comment);
// kept in exact lockstep by this test's own R4 case using it to render.
EngineInput make_engine_input(const EngineRig& rig) {
    const auto& ess = rig.engine_sound_state();
    EngineInput in;
    in.t_sim_s = rig.sim_time_s();
    in.omega_rad_s = rig.engine_omega_rad_s();
    constexpr real kRadToDeg = 180.0 / 3.14159265358979323846;
    real deg = std::fmod(ess.crank_angle * kRadToDeg, 720.0);
    if (deg < 0.0) deg += 720.0;
    in.crank_angle_deg = deg;
    in.p_intake_pa = ess.p_intake_pa;
    in.throttle = ess.throttle_position;
    in.lambda = ess.lambda;
    in.spark_deg_atdc = -ess.spark_advance_deg;
    in.spark_given = true;
    std::uint32_t flags = 0;
    if (ess.dfco_active) flags |= ps_audio::kInputDfco;
    if (ess.limiter_active) flags |= ps_audio::kInputFuelCut;
    in.flags = flags;
    return in;
}

const drivetrain::CycleModelDesc& demo_cycle_desc() {
    static const drivetrain::CycleModelDesc d = [] {
        io::EngineMapOptions opts;
        opts.quality = drivetrain::MapQuality::Preview;  // A8b: only the desc is read; Preview maps are cheaper to generate
        jobs::JobSystem pool(10);
        opts.pool = &pool;
        const drivetrain::ComponentParams params = io::load_engine_json(demo_engine_path(), opts, nullptr);
        const auto* sim = std::get_if<drivetrain::SimulatedEngineDesc>(&params);
        REQUIRE(sim != nullptr);
        REQUIRE(sim->cycle.has_value());
        return *sim->cycle;
    }();
    return d;
}

const drivetrain::FuelDesc& demo_fuel_desc() {
    static const drivetrain::FuelDesc f = [] {
        io::EngineMapOptions opts;
        opts.quality = drivetrain::MapQuality::Preview;  // A8b: only the desc is read; Preview maps are cheaper to generate
        jobs::JobSystem pool(10);
        opts.pool = &pool;
        const drivetrain::ComponentParams params = io::load_engine_json(demo_engine_path(), opts, nullptr);
        const auto* sim = std::get_if<drivetrain::SimulatedEngineDesc>(&params);
        REQUIRE(sim != nullptr);
        REQUIRE_FALSE(sim->fuels.empty());
        return sim->fuels.front();
    }();
    return f;
}

} // namespace

TEST_CASE("A10 R1: demo engine free revs from idle toward the limiter and back", "[audio][drivetrain]") {
    EngineRig rig(demo_engine_path());
    rig.set_ignition(1.0);
    rig.set_starter(1.0);
    real peak_rpm = 0.0;
    for (int tick = 0; tick < 240; ++tick) { // 1 s cranking
        rig.step();
        peak_rpm = std::fmax(peak_rpm, rig.engine_rpm());
    }
    rig.set_starter(0.0);
    REQUIRE(rig.engine_state() == drivetrain::EngineState::Running);

    // Blip to WOT for 2 s, tracking the peak rpm reached.
    rig.set_throttle(1.0);
    for (int tick = 0; tick < 480; ++tick) {
        rig.step();
        peak_rpm = std::fmax(peak_rpm, rig.engine_rpm());
    }
    // Back to idle. With no dyno/vehicle load configured, the only
    // deceleration force after a WOT blip is the engine's own internal
    // friction/pumping losses (no engine braking through a drivetrain) - a
    // real free-revving engine on a dyno with the clutch out decelerates
    // slowly too, so this window is several seconds, not one.
    rig.set_throttle(0.02);
    real settle_rpm = 0.0;
    for (int tick = 0; tick < 5 * 240; ++tick) {
        rig.step();
        settle_rpm = rig.engine_rpm();
    }
    std::printf("A10 R1: peak %.0f rpm, settled back to %.0f rpm after 5 s\n", peak_rpm, settle_rpm);
    // demo_i4_pipes.json's grid_rpm tops out at 6500, no explicit limiter
    // (default fuel-cut applies well above that) - a WOT blip from idle
    // should clear 3000 rpm easily but stay bounded (sabotage: a broken
    // limiter/fuel-cut path would run this well past 8000).
    CHECK(peak_rpm > 3000.0);
    CHECK(peak_rpm < 8000.0);
    // Coming off the throttle should bring it back down near idle, not
    // leave it stuck at the peak (sabotage: an engine ignoring `throttle`
    // entirely reads settle_rpm ~= peak_rpm).
    CHECK(settle_rpm < peak_rpm * 0.6);
}

TEST_CASE("A10 R2: the starter takes the demo engine Off -> Cranking -> Running", "[audio][drivetrain]") {
    EngineRig rig(demo_engine_path());
    REQUIRE(rig.engine_state() == drivetrain::EngineState::Off);
    rig.set_ignition(1.0);
    rig.set_starter(1.0);
    bool saw_cranking = false;
    int ticks_to_running = -1;
    for (int tick = 0; tick < 480; ++tick) {
        rig.step();
        if (rig.engine_state() == drivetrain::EngineState::Cranking) saw_cranking = true;
        if (rig.engine_state() == drivetrain::EngineState::Running && ticks_to_running < 0) ticks_to_running = tick;
    }
    // Sabotage: a starter wiring bug that never engages the starter torque
    // leaves the engine in Off/Stalled forever (saw_cranking false,
    // ticks_to_running never set).
    CHECK(saw_cranking);
    REQUIRE(ticks_to_running >= 0);
    std::printf("A10 R2: reached Running at tick %d\n", ticks_to_running);
    rig.set_starter(0.0);
    for (int tick = 0; tick < 60; ++tick) rig.step();
    CHECK(rig.engine_state() == drivetrain::EngineState::Running);
}

TEST_CASE("A10 R3: cutting ignition stalls a running demo engine", "[audio][drivetrain]") {
    EngineRig rig(demo_engine_path());
    rig.set_ignition(1.0);
    rig.set_starter(1.0);
    for (int tick = 0; tick < 240; ++tick) rig.step();
    rig.set_starter(0.0);
    rig.set_throttle(0.1);
    for (int tick = 0; tick < 120; ++tick) rig.step();
    REQUIRE(rig.engine_state() == drivetrain::EngineState::Running);

    rig.set_ignition(0.0); // EngineRig::set_ignition's own documented hook
    bool saw_off = false;
    // engine_simulated.cpp's state machine checks `sensors.ignition <= 0.5`
    // BEFORE the ordinary per-state transitions and, when true, forces
    // EngineState::Off unconditionally - not a rpm-decay-gated Running ->
    // Stalled path. So the transition is immediate (next substep), whatever
    // rpm the engine was turning at; it never passes through Stalled here.
    for (int tick = 0; tick < 240; ++tick) {
        rig.step();
        if (rig.engine_state() == drivetrain::EngineState::Off) saw_off = true;
    }
    // Sabotage: set_ignition wired to a no-op channel never reaches Off
    // within 1 s of a dead-key cut (state holds at Running instead).
    CHECK(saw_off);
    std::printf("A10 R3: engine_state after ignition cut = %d (Off=%d)\n", static_cast<int>(rig.engine_state()),
                static_cast<int>(drivetrain::EngineState::Off));

    // The voice's own tail after the input stream stops must stay finite
    // (no NaN/inf from a divide-by-zero as omega -> 0) - one more second of
    // audio with no further push_input.
    VoiceSettings vs;
    vs.engine.seed = 1;
    Voice voice(demo_cycle_desc(), demo_fuel_desc().lhv_j_kg, demo_fuel_desc().afr_stoich, vs);
    voice.push_input(make_engine_input(rig));
    const int frames = static_cast<int>(voice.sample_rate_hz()); // 1 s
    std::vector<float> buf(static_cast<std::size_t>(frames) * static_cast<std::size_t>(voice.source_count()), 0.0f);
    voice.render(buf.data(), frames);
    bool all_finite = true;
    for (float v : buf) all_finite = all_finite && std::isfinite(v);
    CHECK(all_finite);
}

TEST_CASE("A10 R4: EngineRig -> Voice render is deterministic and seed-sensitive", "[audio][drivetrain]") {
    auto render = [](std::uint64_t seed, int seconds) {
        EngineRig rig(demo_engine_path());
        rig.set_ignition(1.0);
        rig.set_starter(1.0);
        VoiceSettings vs;
        vs.engine.seed = seed;
        Voice voice(demo_cycle_desc(), demo_fuel_desc().lhv_j_kg, demo_fuel_desc().afr_stoich, vs);
        const int frames_per_tick = static_cast<int>(voice.sample_rate_hz() / 240.0);
        const int sources = voice.source_count();
        std::vector<float> per_source(static_cast<std::size_t>(frames_per_tick) * static_cast<std::size_t>(sources));
        std::vector<float> out;
        for (int tick = 0; tick < seconds * 240; ++tick) {
            if (tick == 120) rig.set_starter(0.0);
            if (tick == 150) rig.set_throttle(0.4);
            rig.step();
            voice.push_input(make_engine_input(rig));
            voice.render(per_source.data(), frames_per_tick);
            out.insert(out.end(), per_source.begin(), per_source.end());
        }
        return out;
    };

    const std::vector<float> a1 = render(42, 1);
    const std::vector<float> a2 = render(42, 1);
    const std::vector<float> b = render(43, 1);

    REQUIRE(a1.size() == a2.size());
    CHECK(a1 == a2); // same seed, twice: bit-identical

    REQUIRE(a1.size() == b.size());
    // Sabotage: a build where the seed never reaches LiveEngineSettings
    // (e.g. always constructed with the default VoiceSettings{}) would make
    // a1 == b here too - CHECK(a1 != b) below fails on that sabotaged build,
    // confirming --seed is load-bearing, not a no-op flag.
    CHECK(a1 != b);
}

TEST_CASE("A10 R5: the demo engine's ledger closes from tick 1 (F-T0)", "[audio][drivetrain]") {
    EngineRig rig(demo_engine_path());
    rig.set_ignition(1.0);
    rig.set_starter(1.0);
    real worst_balance = 0.0, scale = 1.0;
    int worst_tick = -1;
    for (int tick = 0; tick < 600; ++tick) {
        if (tick == 200) rig.set_starter(0.0);
        if (tick == 240) rig.set_throttle(0.5);
        rig.step();
        scale = std::fmax(scale, std::fabs(rig.ledger().engine_out));
        // Tick 0 carries the constructor's unbooked rotor seed (vault
        // PHYS-049 F-T0, same convention as Powertrain/test_engine_rig_wheelless.cpp).
        if (tick > 0 && std::fabs(rig.ledger().balance_error) > worst_balance) {
            worst_balance = std::fabs(rig.ledger().balance_error);
            worst_tick = tick;
        }
    }
    std::printf("A10 R5: worst |balance_error| %.3e J at tick %d (scale %.3f)\n", worst_balance, worst_tick, scale);
    CHECK(worst_balance <= 1e-8 * scale);
}

TEST_CASE("A10 R6: EngineRig::step is allocation-free once built", "[audio][drivetrain]") {
    EngineRig rig(demo_engine_path());
    rig.set_ignition(1.0);
    rig.set_starter(1.0);
    for (int tick = 0; tick < 60; ++tick) rig.step(); // warm up (starter transition etc.)

    jobs::debug_alloc::reset();
    jobs::debug_alloc::set_armed(true);
    for (int tick = 0; tick < 120; ++tick) {
        if (tick == 30) rig.set_starter(0.0);
        if (tick == 40) rig.set_throttle(0.6);
        rig.step();
    }
    const std::uint64_t violations = jobs::debug_alloc::violation_count();
    jobs::debug_alloc::set_armed(false);
    std::printf("A10 R6: debug_alloc violation_count = %llu over 120 steps\n",
                static_cast<unsigned long long>(violations));
    CHECK(violations == 0);
}

namespace {
// free_rev.json's event list (data/scripts/engine_audio/free_rev.json),
// driven through an EngineRig exactly as tools/ps_engine_audio does (no
// explicit ignition call), recorded once as the Voice's per-tick inputs.
std::vector<EngineInput> record_free_rev_inputs(real seconds) {
    struct Ev {
        real t;
        int kind;  // 0 starter, 1 throttle
        real v;
    };
    const Ev evs[] = {{0.0, 0, 1.0}, {0.8, 0, 0.0}, {1.2, 1, 0.05}, {3.0, 1, 0.9},
                      {3.8, 1, 0.05}, {5.5, 1, 1.0}, {6.6, 1, 0.05}};
    jobs::JobSystem pool(10);  // Full maps as the tool uses them, generated on 10 workers (A8b: cost)
    io::EngineMapOptions opts;
    opts.pool = &pool;
    EngineRig rig(demo_engine_path(), opts);
    std::vector<EngineInput> in;
    std::size_t next = 0;
    const int ticks = static_cast<int>(seconds * 240.0);
    for (int tick = 0; tick < ticks; ++tick) {
        const real t = tick / 240.0;
        while (next < sizeof(evs) / sizeof(evs[0]) && evs[next].t <= t) {
            if (evs[next].kind == 0) rig.set_starter(evs[next].v);
            else rig.set_throttle(evs[next].v);
            ++next;
        }
        rig.step();
        in.push_back(make_engine_input(rig));
    }
    return in;
}

struct BlowupStats {
    real first_bad_t = -1.0;  // first 0.1 s block with a non-finite or > 1 kPa sample (-1: none)
    real max_abs_pa = 0.0;
    std::uint64_t subcycled = 0;
};

// Renders the recorded inputs through a Voice (default audio dx unless the
// settings override it) and prints per-0.1 s the peak output, plenum
// pressure, filtered plenum target, throttle Cd*A and the network's extreme
// cell state.
BlowupStats render_free_rev(const std::vector<EngineInput>& in, const VoiceSettings& vs, const char* label,
                            bool print, std::size_t first_tick = 0) {
    Voice voice(demo_cycle_desc(), demo_fuel_desc().lhv_j_kg, demo_fuel_desc().afr_stoich, vs);
    const int fpt = static_cast<int>(voice.sample_rate_hz() / 240.0);
    const int ns = voice.source_count();
    std::vector<float> blk(static_cast<std::size_t>(fpt) * static_cast<std::size_t>(ns));
    BlowupStats st;
    real peak = 0.0;
    for (std::size_t tick = first_tick; tick < in.size(); ++tick) {
        voice.push_input(in[tick]);
        voice.render(blk.data(), fpt);
        for (float v : blk) {
            const real a = std::isfinite(v) ? std::fabs(static_cast<real>(v)) : 1e30;
            peak = std::fmax(peak, a);
        }
        if ((tick + 1) % 24 == 0) {
            const real t = (tick + 1) / 240.0;
            const engine_cycle::PipeNetwork* net = voice.engine().core().network();
            real mach = 0.0, pmin = 1e30, pmax = 0.0;
            int wp = -1;
            for (int p = 0; p < net->pipe_count(); ++p)
                for (int i = 0; i < net->cell_count(p); ++i) {
                    const engine_cycle::PipeCellState c = net->cell(p, i);
                    const real m = std::fabs(c.u) / c.a;
                    if (!(m <= mach)) {
                        mach = m;
                        wp = p;
                    }
                    pmin = std::fmin(pmin, c.p);
                    pmax = std::fmax(pmax, c.p);
                }
            if (print)
                std::printf("%s t %.1f: peak %.3e Pa, rpm %.0f p_tgt %.0f plenum %.0f p_f %.0f cda %.3e, max Mach %.3f (pipe %d), "
                            "cell p %.0f..%.0f\n",
                            label, t, peak, in[tick].omega_rad_s * 60.0 / 6.283185307179586, in[tick].p_intake_pa,
                            voice.engine().plenum_pressure_pa(), voice.engine().plenum_filtered_pa(),
                            voice.engine().throttle_cd_area(), mach, wp, pmin, pmax);
            st.max_abs_pa = std::fmax(st.max_abs_pa, peak);
            if (st.first_bad_t < 0.0 && !(peak < 1000.0)) st.first_bad_t = t;
            if (!(peak < 1e20)) break;
            peak = 0.0;
        }
    }
    st.subcycled = voice.engine().core().network()->subcycled_pipe_steps();
    return st;
}
}  // namespace

TEST_CASE("A8b B diag: free_rev at the default audio dx - where the blow-up grows", "[.][diag][audio]") {
    const std::vector<EngineInput> in = record_free_rev_inputs(8.0);
    for (int variant = 0; variant < 3; ++variant) {
        VoiceSettings vs;
        const char* label = "default dx";
        if (variant == 0) {
            vs.engine.dx_exhaust_m = 0.032;
            vs.engine.dx_intake_m = 0.017;
        } else if (variant == 1) {
            vs.engine.dx_exhaust_m = 0.032;
            vs.engine.dx_intake_m = 0.017;
            vs.engine.plenum_loop = false;
            label = "plenum loop off";
        } else {
            vs.engine.dx_exhaust_m = 0.064;  // A10's former workaround (doubled audio dx)
            vs.engine.dx_intake_m = 0.034;
            label = "doubled dx (0.064/0.034)";
        }
        const BlowupStats st = render_free_rev(in, vs, label, true);
        std::printf("%s: first block with |p| >= 1 kPa or non-finite at t %.1f s, max %.3e Pa, subcycled pipe-steps %llu\n", label,
                    st.first_bad_t, st.max_abs_pa, static_cast<unsigned long long>(st.subcycled));
    }
}

TEST_CASE("A8b B diag: sample-level trace into the default-dx blow-up", "[.][diag][audio]") {
    const std::vector<EngineInput> in = record_free_rev_inputs(5.0);
    VoiceSettings vs;
    vs.engine.dx_exhaust_m = 0.032;
    vs.engine.dx_intake_m = 0.017;
    Voice voice(demo_cycle_desc(), demo_fuel_desc().lhv_j_kg, demo_fuel_desc().afr_stoich, vs);
    const int fpt = static_cast<int>(voice.sample_rate_hz() / 240.0);
    const int ns = voice.source_count();
    std::vector<float> blk(static_cast<std::size_t>(fpt) * static_cast<std::size_t>(ns));
    const engine_cycle::PipeNetwork* net = voice.engine().core().network();
    std::printf("pipes:");
    for (int p = 0; p < net->pipe_count(); ++p) std::printf(" %d:%d cells dx %.4f", p, net->cell_count(p), net->cell_dx(p));
    std::printf("\nvolumes %d, ports %d\n", net->volume_count(), net->port_count());
    long long sample = 0;
    real pmin_run = 1e30;
    bool done = false;
    for (std::size_t tick = 0; tick < in.size() && !done; ++tick) {
        voice.push_input(in[tick]);
        const bool fine = tick >= static_cast<std::size_t>(4.3 * 240.0);
        if (!fine) {
            voice.render(blk.data(), fpt);
            sample += fpt;
            continue;
        }
        for (int f = 0; f < fpt && !done; ++f) {
            voice.render(blk.data(), 1);
            ++sample;
            real pmin = 1e30, rmin = 1e30, tmin = 1e30, tmax = 0.0, mmax = 0.0;
            int wp = -1, wi = -1, wm = -1;
            bool bad = false;
            for (int p = 0; p < net->pipe_count(); ++p)
                for (int i = 0; i < net->cell_count(p); ++i) {
                    const engine_cycle::PipeCellState c = net->cell(p, i);
                    if (!std::isfinite(c.p) || !std::isfinite(c.rho) || !std::isfinite(c.u)) bad = true;
                    if (c.p < pmin) {
                        pmin = c.p;
                        wp = p;
                        wi = i;
                    }
                    rmin = std::fmin(rmin, c.rho);
                    tmin = std::fmin(tmin, c.t);
                    tmax = std::fmax(tmax, c.t);
                    if (std::fabs(c.u) / c.a > mmax) {
                        mmax = std::fabs(c.u) / c.a;
                        wm = p;
                    }
                }
            real vmin = 1e30;
            int wv = -1;
            for (int v = 0; v < net->volume_count(); ++v) {
                const real pv = net->volume_state(v).p;
                if (!std::isfinite(pv)) bad = true;
                if (pv < vmin) {
                    vmin = pv;
                    wv = v;
                }
            }
            pmin_run = std::fmin(pmin_run, pmin);
            const bool report = bad || (sample % 480 == 0) || pmin < 2000.0;
            if (report) {
                const engine_cycle::PipeCellState c = (wp >= 0) ? net->cell(wp, wi) : engine_cycle::PipeCellState{};
                std::printf("s %lld t %.5f: pmin %.1f at pipe %d cell %d/%d (rho %.4e u %.1f T %.1f a %.1f) rho_min %.3e "
                            "T %.1f..%.1f Mach max %.3f (pipe %d) vol pmin %.1f (vol %d) fallbacks %d crank %.1f%s\n",
                            sample, sample / 48000.0, pmin, wp, wi, wp >= 0 ? net->cell_count(wp) : 0, c.rho, c.u, c.t,
                            c.a, rmin, tmin, tmax, mmax, wm, vmin, wv, net->last_step_fallback_count(),
                            voice.engine().crank_deg(), bad ? "  NON-FINITE" : "");
            }
            if (bad) done = true;
        }
    }
    std::printf("run min cell pressure %.1f Pa\n", pmin_run);
}

TEST_CASE("A8b B1: WOT hold then close at the default audio dx stays bounded", "[audio][cycle][pipe][long]") {
    // ps_audio A8b point 4: free_rev.json's throttle 0.9 hold (t 3.0-3.8 s)
    // used to grow to NaN / railed output by t ~4.7 s at the default audio
    // dx (0.032/0.017 m). Cause: after the close, under the near-vacuum
    // overrun manifold, valve overlap blows a supersonic backflow jet
    // (Mach ~2, |u| + a ~1260 m/s) into an intake runner - CFL ~1.5 at dx
    // 0.017 m and h 1/48000 s, beyond the build-time check's assumed 500 m/s
    // - and the explicit FV update diverged. Fixed by the runtime CFL
    // sub-cycling in PipeNetwork::step (P9 is the standalone guard). This is
    // the whole-engine guard: the Voice starts at t 2.8 s from the rig's
    // state (saves ~3 s of idle render) and runs past the old blow-up time.
    // Sabotage (step_pipe never sub-cycles): non-finite / > 1 kPa by t ~4.x s.
    const std::vector<EngineInput> in = record_free_rev_inputs(5.0);
    const std::size_t t0 = static_cast<std::size_t>(2.8 * 240.0);
    VoiceSettings vs;
    vs.engine.dx_exhaust_m = 0.032;  // the ResolutionCycleDesc defaults, explicit so the test does not
    vs.engine.dx_intake_m = 0.017;   // follow a later change of the demo file
    vs.engine.initial_rpm = in[t0].omega_rad_s * 60.0 / 6.283185307179586;
    vs.engine.initial_p_intake_pa = in[t0].p_intake_pa;
    const BlowupStats st = render_free_rev(in, vs, "B1", false, t0);
    std::printf("A8b B1: first bad block t %.1f s (-1 none), max |p| %.3e Pa at 1 m, sub-cycled pipe-steps %llu\n",
                st.first_bad_t, st.max_abs_pa, static_cast<unsigned long long>(st.subcycled));
    CHECK(st.first_bad_t < 0.0);
    CHECK(st.max_abs_pa < 1000.0);
    CHECK(st.subcycled > 0);  // scenario sanity: the supersonic backflow does occur
}
