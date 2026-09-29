// ps_audio A11: tools/common/engine_sound_glue.h's ONE shared
// EngineSoundState -> ps_audio::EngineInput mapping (moved out of
// tools/ps_engine_audio/main.cpp so adapters/godot can share it without
// duplicating the mapping - see that header's own comment). Standalone:
// no World, no Powertrain, just hand-built EngineSoundState values.
#include "engine_sound_glue.h"

#include "ps/drivetrain/powertrain_state.h"
#include "ps_audio/engine_input.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using ps::drivetrain::EngineSoundState;
using ps::drivetrain::EngineState;

TEST_CASE("engine sound glue: fields mapped", "[audio]") {
    EngineSoundState ess;
    ess.omega = 314.159;
    ess.crank_angle = 3.14159265358979323846; // pi rad = 180 deg
    ess.engine_state = EngineState::Running;
    ess.throttle_position = 0.42;
    ess.p_intake_pa = 55000.0;
    ess.lambda = 0.98;
    ess.spark_advance_deg = 12.0; // BTDC per docs/data_formats/engine.md
    ess.dfco_active = false;
    ess.limiter_active = false;

    auto in = ps_tools::make_engine_input(ess, 1.5);

    CHECK(in.t_sim_s == 1.5);
    CHECK(in.omega_rad_s == ess.omega);
    CHECK(in.crank_angle_deg == Catch::Approx(180.0).margin(1e-9));
    CHECK(in.p_intake_pa == ess.p_intake_pa);
    CHECK(in.throttle == ess.throttle_position);
    CHECK(in.lambda == ess.lambda);
    // spark_deg_atdc = -spark_advance_deg (BTDC -> ATDC sign flip, see the
    // header's own comment / docs/data_formats/engine.md).
    CHECK(in.spark_deg_atdc == Catch::Approx(-12.0));
    CHECK(in.spark_given);
    CHECK(in.flags == 0u);
}

TEST_CASE("engine sound glue: crank angle wraps into [0, 720)", "[audio]") {
    EngineSoundState ess;
    ess.crank_angle = -0.1; // slightly negative: must wrap up, not go negative
    auto in = ps_tools::make_engine_input(ess, 0.0);
    CHECK(in.crank_angle_deg >= 0.0);
    CHECK(in.crank_angle_deg < 720.0);
}

TEST_CASE("engine sound glue: cut flags", "[audio]") {
    {
        EngineSoundState ess;
        ess.dfco_active = true;
        auto in = ps_tools::make_engine_input(ess, 0.0);
        CHECK((in.flags & ps_audio::kInputDfco) != 0u);
        CHECK((in.flags & ps_audio::kInputFuelCut) == 0u);
    }
    {
        EngineSoundState ess;
        ess.limiter_active = true;
        auto in = ps_tools::make_engine_input(ess, 0.0);
        CHECK((in.flags & ps_audio::kInputFuelCut) != 0u);
        CHECK((in.flags & ps_audio::kInputDfco) == 0u);
    }
    {
        EngineSoundState ess;
        ess.dfco_active = true;
        ess.limiter_active = true;
        auto in = ps_tools::make_engine_input(ess, 0.0);
        CHECK((in.flags & ps_audio::kInputDfco) != 0u);
        CHECK((in.flags & ps_audio::kInputFuelCut) != 0u);
    }
}

TEST_CASE("engine sound glue: engine Off state carries zero omega through", "[audio]") {
    // A freshly-Off (never run) EngineSoundState is default-constructed:
    // omega defaults to 0. The mapping does not special-case engine_state
    // (see the header comment for why: a real vehicle's crank can still
    // turn while Off, e.g. push-started) - it simply passes ess.omega
    // through, so the "Off -> zero omega" case here is the ordinary default
    // state, not a hidden branch.
    EngineSoundState ess;
    ess.engine_state = EngineState::Off;
    REQUIRE(ess.omega == 0.0);
    auto in = ps_tools::make_engine_input(ess, 0.0);
    CHECK(in.omega_rad_s == 0.0);
}
