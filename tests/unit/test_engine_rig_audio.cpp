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
