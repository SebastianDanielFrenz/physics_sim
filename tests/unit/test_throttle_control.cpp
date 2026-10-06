// tests/unit/test_throttle_control.cpp — white-box + loader tests for the
// throttle-command stage between the pedal and an engine (PHYS-033 P4 I7a
// C3, plan section 3.4). core/src is on this target's include path
// (tests/unit/CMakeLists.txt), like the other white-box drivetrain tests -
// ThrottleStage (core/src/drivetrain/throttle_stage.h) and EngineTorqueMap
// (core/src/drivetrain/engine_torque_map.h) are internal, not part of
// ps_core's public API.
//
// P1 and P5 below follow the C3 task briefing's own enumeration and are
// unchanged from the original commit:
//   P1 static_assert: ThrottleCommand/Sensors carry no raw pedal field.
//   P5 the loader's own throttle_control rejection list (plan section
//      3.4.5), each pinned to its exact message (the C3 REVIEW's items 2-5
//      added a further, more complete set of P5 sections to
//      test_vehicle_io_validation.cpp instead - kind-mismatch both
//      directions, hand_lever_channel vs. governor.channel, curve endpoint/
//      monotonicity, engine-ref order-independence and the no-engine
//      rejection - since that is this project's usual home for
//      load_vehicle_json rejection tests).
//
// The C3 REVIEW (a follow-up after the original commit) requires P2/P3/P4/P6
// to each be a Powertrain-level (not ThrottleStage-level) test - see the
// TEST_CASEs below tagged P2/P3/P4/P6 for the real deliverables. The
// ORIGINAL stage-level P2/P3/P4/P6 white-box cases are kept (the review said
// "keep or drop... as you like") but retagged S2/S4/S3/S6 so the P-numbered
// tags refer only to the review's own spec'd tests:
//   S2 white-box: EngineTorqueMap's response is a pure function of
//      ThrottleCommand (+ TcuRequest/Sensors), never of anything else that
//      might smuggle a pedal value through.
//   S3 ThrottleStage's own output (pedal_map/linkage_curve, hand lever,
//      floor/ceiling) reaches EngineTorqueMap unchanged.
//   S4 a mechanical-linkage fixture: linkage_curve applied, floor/ceiling
//      fixed 0/1 regardless of what the TCU/ctx asks for, upshift ignition
//      cut still works (it never goes through the throttle path at all).
//   S6 a tractor-governor-path regression: the ThrottleStage + governed-
//      engine composition reproduces the pre-C3 monolithic max(hand,pedal)
//      formula bit-for-bit.
//
// The review's own P2/P3/P4/P6 (Powertrain-level, appended at the end of
// this file):
//   P2 car_sedan Powertrain, an explicit electronic stage with a flat
//      pedal_map plateau: two different pedal values inside the plateau
//      produce bit-identical PreSolveResult/torque/state hashes.
//   P3 dyno-held rpm: throttle_eff/engine_torque_demand match the pedal_map-
//      reshaped command exactly (1e-12).
//   P4 a mechanical graph-engine fixture (tests/support/fixtures/): the
//      explicit mechanical_linkage stage applies its linkage_curve, the
//      IMPLICIT stage (fixture component erased in code) passes the pedal
//      through unshaped (plan section 3.4: no curve), floor/ceiling reaching
//      the engine are exactly 0/1 even while the TCU requests ceiling 0, and
//      theta_c is 0 during a real TCU upshift cut.
//   P6 master-vs-branch: PowertrainDebug/snapshot fields recorded from
//      master over data/scenarios/vehicle_tractor_i4.json's control events,
//      compared EXACTLY (==) against this branch.

#include "drivetrain/components.h"
#include "drivetrain/engine_torque_map.h"
#include "drivetrain/powertrain.h"
#include "drivetrain/throttle_stage.h"

#include "powertrain_multi_diff_desc.h"
#include "vehicle_test_helpers.h"

#include "ps/io/scenario.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

// --- P1: ThrottleCommand/Sensors carry no raw pedal field -------------------
// SFINAE detection idiom naming the EXACT field names PHYS-033 P4 I7a C3
// removed (engine.h's own header comment: "pedal_throttle/hand_throttle's
// removal") - a compile-time, not run-time, guarantee that a future edit
// cannot silently reintroduce either field on the engine's own input types.
namespace throttle_control_test_detail {
template <typename T, typename = void> struct has_pedal_throttle : std::false_type {};
template <typename T>
struct has_pedal_throttle<T, std::void_t<decltype(std::declval<T>().pedal_throttle)>> : std::true_type {};

template <typename T, typename = void> struct has_hand_throttle : std::false_type {};
template <typename T>
struct has_hand_throttle<T, std::void_t<decltype(std::declval<T>().hand_throttle)>> : std::true_type {};
} // namespace throttle_control_test_detail

static_assert(!throttle_control_test_detail::has_pedal_throttle<Sensors>::value,
              "Sensors must not carry pedal_throttle (I7a C3: moved to ThrottleCommand::command via ThrottleStage)");
static_assert(!throttle_control_test_detail::has_hand_throttle<Sensors>::value,
              "Sensors must not carry hand_throttle (I7a C3: moved to ThrottleStage's own hand lever)");
static_assert(!throttle_control_test_detail::has_pedal_throttle<ThrottleCommand>::value,
              "ThrottleCommand must not carry a raw pedal_throttle field - the pedal/hand-lever combination "
              "happens in ThrottleStage::evaluate() before this struct is ever built");
static_assert(!throttle_control_test_detail::has_hand_throttle<ThrottleCommand>::value,
              "ThrottleCommand must not carry a raw hand_throttle field either, for the same reason");

TEST_CASE("P1: ThrottleCommand/Sensors have no pedal field (compile-time)", "[unit][drivetrain][throttle]") {
    // The static_asserts above are the actual check (they fail the BUILD, not
    // this test, if either field ever comes back) - this TEST_CASE exists
    // only so the guarantee shows up in the test list/CI output.
    SUCCEED("static_assert above compiled: neither Sensors nor ThrottleCommand carries pedal_throttle/hand_throttle");
}

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);
real rpm_to_omega(real rpm) { return rpm / kRadPerSecToRpm; }

// Same hand-computable synthetic engine as test_powertrain_engine.cpp's
// make_test_desc() (kept independent/duplicated rather than shared - that
// file's helper is anonymous-namespace-local to its own translation unit).
EngineTorqueMap::Desc make_test_desc() {
    EngineTorqueMap::Desc d;
    d.wot_torque_nm_vs_rpm.x = {1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0};
    d.wot_torque_nm_vs_rpm.y = {100.0, 140.0, 160.0, 160.0, 140.0, 100.0};
    d.motoring_torque_nm_vs_rpm.x = {0.0, 1000.0, 3000.0, 6000.0};
    d.motoring_torque_nm_vs_rpm.y = {-10.0, -14.0, -22.0, -40.0};
    d.inertia_kgm2 = 0.16;
    d.breakaway_torque_nm = 12.0;
    d.idle_rpm = 900.0;
    d.stall_rpm = 400.0;
    d.fire_rpm = 250.0;
    d.start_complete_rpm = 550.0;
    d.dfco_enable_rpm = 1300.0;
    d.dfco_reenable_rpm = 1000.0;
    d.idle.kp_per_rpm = 0.0018;
    d.idle.ki_per_rpm_s = 0.009;
    d.idle.max_throttle = 0.3;
    d.limiter.type = EngineTorqueMap::LimiterType::FuelCut;
    d.limiter.rpm = 6000.0;
    d.limiter.hysteresis_rpm = 150.0;
    d.starter.stall_torque_nm = 140.0;
    d.starter.free_speed_rpm = 400.0;
    d.starter.interlock = EngineTorqueMap::StarterInterlock::ClutchOrNeutral;
    d.initial_state = EngineTorqueMap::InitialState::Running;
    return d;
}

Sensors running_sensors(real omega) {
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.starter = 0.0;
    s.clutch_engaged_fraction = 1.0;
    s.gearbox_in_neutral = true;
    return s;
}

// A ComponentSubstepCtx with only the fields ThrottleStage::evaluate reads
// set explicitly; everything else stays at ComponentSubstepCtx's own
// defaults (unused by the stage).
ComponentSubstepCtx make_ctx(real throttle, real tcu_floor = 0.0, real tcu_ceiling = 1.0, real h = 1.0 / 960.0) {
    ComponentSubstepCtx ctx;
    ctx.h = h;
    ctx.throttle = throttle;
    ctx.tcu_throttle_floor = tcu_floor;
    ctx.tcu_throttle_ceiling = tcu_ceiling;
    return ctx;
}

} // namespace

// --- P2: EngineTorqueMap never reads anything but ThrottleCommand ----------
//
// Discrimination-demonstration (performed manually this session, then
// reverted - see the C3 handback report for the transcript): the assertion
// below was temporarily broken by editing engine_torque_map.cpp's
// pre_solve() to compute req_input from a HARDCODED 0.0 instead of
// throttle.command (simulating a hypothetical future regression where the
// engine stops reading ThrottleCommand and instead reads nothing / a stale
// value). With that one-line sabotage in place, this SECTION's "raised
// command changes torque_estimate" CHECK failed (torque_estimate stayed
// flat at every throttle level) while every other test in this file that
// does not depend on throttle response kept passing - i.e. this test (and
// only this class of test) is the one that would catch that specific class
// of bug. The sabotage was then reverted and the full suite re-confirmed
// green before this file was committed.
TEST_CASE("S2: EngineTorqueMap's response is a pure function of ThrottleCommand (white-box, stage-level)", "[unit][drivetrain][throttle]") {
    EngineTorqueMap eng(make_test_desc());
    const real omega = rpm_to_omega(2500.0); // well above idle_rpm(900): idle PI clamps to 0, no DFCO, no limiter
    TcuRequest tcu;

    SECTION("raising ThrottleCommand::command strictly raises torque_estimate at fixed rpm") {
        ThrottleCommand low;
        low.command = 0.2;
        low.floor = 0.0;
        low.ceiling = 1.0;
        const auto r_low = eng.pre_solve(running_sensors(omega), low, tcu, 1.0 / 960.0);

        // Fresh engine instance: pre_solve is not purely stateless (idle
        // integrator/limiter latches persist across substeps), but at this
        // rpm/command combination neither latch is active (checked via
        // r_low.limiter_active/dfco below), so a second instance at a
        // different command is directly comparable.
        REQUIRE_FALSE(r_low.limiter_active);
        EngineTorqueMap eng2(make_test_desc());
        ThrottleCommand high;
        high.command = 0.9;
        high.floor = 0.0;
        high.ceiling = 1.0;
        const auto r_high = eng2.pre_solve(running_sensors(omega), high, tcu, 1.0 / 960.0);

        CHECK(r_low.throttle_eff == Catch::Approx(0.2).epsilon(1e-9));
        CHECK(r_high.throttle_eff == Catch::Approx(0.9).epsilon(1e-9));
        CHECK(r_high.torque_estimate > r_low.torque_estimate);
    }

    SECTION("command is clamped to [floor, ceiling] exactly - not read from anywhere else") {
        ThrottleCommand cmd;
        cmd.command = 0.95;
        cmd.floor = 0.10;
        cmd.ceiling = 0.40;
        const auto r = eng.pre_solve(running_sensors(omega), cmd, tcu, 1.0 / 960.0);
        CHECK(r.throttle_eff == Catch::Approx(0.40).epsilon(1e-9)); // clipped to ceiling, not 0.95

        EngineTorqueMap eng2(make_test_desc());
        ThrottleCommand cmd_below_floor;
        cmd_below_floor.command = 0.02;
        cmd_below_floor.floor = 0.10;
        cmd_below_floor.ceiling = 0.40;
        const auto r2 = eng2.pre_solve(running_sensors(omega), cmd_below_floor, tcu, 1.0 / 960.0);
        CHECK(r2.throttle_eff == Catch::Approx(0.10).epsilon(1e-9)); // raised to floor, not 0.02
    }

    SECTION("ignition_cut zeroes the request regardless of ThrottleCommand::command") {
        TcuRequest cut;
        cut.ignition_cut = true;
        ThrottleCommand cmd_a;
        cmd_a.command = 0.2;
        const auto r_a = eng.pre_solve(running_sensors(omega), cmd_a, cut, 1.0 / 960.0);

        EngineTorqueMap eng2(make_test_desc());
        ThrottleCommand cmd_b;
        cmd_b.command = 1.0;
        const auto r_b = eng2.pre_solve(running_sensors(omega), cmd_b, cut, 1.0 / 960.0);

        // theta_req is forced to 0 by ignition_cut independent of command, so
        // theta_eff (== max(0, theta_idle)) and torque_estimate must agree
        // between a 0.2 and a 1.0 command - the ONLY way that can happen is
        // if the engine never reads anything but the (now-irrelevant, cut)
        // ThrottleCommand and TcuRequest::ignition_cut.
        CHECK(r_a.throttle_eff == Catch::Approx(r_b.throttle_eff).epsilon(1e-12));
        CHECK(r_a.torque_estimate == Catch::Approx(r_b.torque_estimate).epsilon(1e-12));
    }
}

// --- P3: ThrottleStage's output reaches EngineTorqueMap correctly -----------
TEST_CASE("S3: ThrottleStage output (curve/hand-lever/floor/ceiling) reaches EngineTorqueMap unchanged (stage-level)",
          "[unit][drivetrain][throttle]") {
    const real omega = rpm_to_omega(2500.0); // idle PI inactive, no DFCO/limiter at this test's throttle levels
    TcuRequest tcu;

    SECTION("no curve, no hand lever: command passes straight through") {
        ThrottleStage stage;
        vehicle::Curve1D no_curve;
        stage.configure(ThrottleInput::Electronic, /*has_curve=*/false, no_curve, /*hand_lever_channel_slot=*/-1);
        REQUIRE_FALSE(stage.has_hand_lever());

        const ComponentSubstepCtx ctx = make_ctx(/*throttle=*/0.6, /*floor=*/0.0, /*ceiling=*/1.0);
        const ThrottleCommand cmd = stage.evaluate(ctx);
        CHECK(cmd.command == Catch::Approx(0.6).epsilon(1e-12));
        CHECK(cmd.floor == Catch::Approx(0.0).epsilon(1e-12));
        CHECK(cmd.ceiling == Catch::Approx(1.0).epsilon(1e-12));

        EngineTorqueMap eng(make_test_desc());
        const auto r = eng.pre_solve(running_sensors(omega), cmd, tcu, 1.0 / 960.0);
        CHECK(r.throttle_eff == Catch::Approx(0.6).epsilon(1e-9));
    }

    SECTION("electronic floor/ceiling from ctx.tcu_* reach and clip the engine") {
        ThrottleStage stage;
        vehicle::Curve1D no_curve;
        stage.configure(ThrottleInput::Electronic, false, no_curve, -1);

        const ComponentSubstepCtx ctx = make_ctx(/*throttle=*/0.95, /*floor=*/0.05, /*ceiling=*/0.90);
        const ThrottleCommand cmd = stage.evaluate(ctx);
        CHECK(cmd.floor == Catch::Approx(0.05).epsilon(1e-12));
        CHECK(cmd.ceiling == Catch::Approx(0.90).epsilon(1e-12));

        EngineTorqueMap eng(make_test_desc());
        const auto r = eng.pre_solve(running_sensors(omega), cmd, tcu, 1.0 / 960.0);
        CHECK(r.throttle_eff == Catch::Approx(0.90).epsilon(1e-9)); // clipped to the TCU's ceiling via the stage
    }

    SECTION("a pedal_map curve is applied before the engine ever sees the command") {
        ThrottleStage stage;
        vehicle::Curve1D half; // halves the pedal position everywhere
        half.x = {0.0, 1.0};
        half.y = {0.0, 0.5};
        stage.configure(ThrottleInput::Electronic, /*has_curve=*/true, half, -1);

        const ComponentSubstepCtx ctx = make_ctx(/*throttle=*/0.8);
        const ThrottleCommand cmd = stage.evaluate(ctx);
        CHECK(cmd.command == Catch::Approx(0.4).epsilon(1e-12)); // 0.8 * 0.5

        EngineTorqueMap eng(make_test_desc());
        const auto r = eng.pre_solve(running_sensors(omega), cmd, tcu, 1.0 / 960.0);
        CHECK(r.throttle_eff == Catch::Approx(0.4).epsilon(1e-9));
    }

    SECTION("a hand lever combines with the pedal as max(), and that max reaches the engine") {
        ThrottleStage stage;
        vehicle::Curve1D no_curve;
        stage.configure(ThrottleInput::Electronic, false, no_curve, /*hand_lever_channel_slot=*/0);
        REQUIRE(stage.has_hand_lever());

        const real channels[1] = {0.7}; // hand lever raw value
        ComponentSubstepCtx ctx = make_ctx(/*throttle=*/0.2);
        ctx.channels = channels;
        ctx.channel_count = 1;

        CHECK(stage.raw_hand_lever(ctx) == Catch::Approx(0.7).epsilon(1e-12));
        const ThrottleCommand cmd = stage.evaluate(ctx);
        CHECK(cmd.command == Catch::Approx(0.7).epsilon(1e-12)); // max(0.7, 0.2)

        EngineTorqueMap eng(make_test_desc());
        const auto r = eng.pre_solve(running_sensors(omega), cmd, tcu, 1.0 / 960.0);
        CHECK(r.throttle_eff == Catch::Approx(0.7).epsilon(1e-9));
    }
}

// --- P4: a mechanical-linkage engine fixture --------------------------------
TEST_CASE("S4: mechanical-linkage stage - linkage_curve applied, floor/ceiling fixed 0/1, upshift cut still works (stage-level)",
          "[unit][drivetrain][throttle]") {
    const real omega = rpm_to_omega(2500.0);

    SECTION("linkage_curve is applied") {
        ThrottleStage stage;
        vehicle::Curve1D linkage; // a nonlinear-ish linkage: compresses the top of the pedal travel
        linkage.x = {0.0, 0.5, 1.0};
        linkage.y = {0.0, 0.35, 0.6};
        stage.configure(ThrottleInput::Mechanical, /*has_curve=*/true, linkage, -1);

        const ComponentSubstepCtx ctx = make_ctx(/*throttle=*/1.0);
        const ThrottleCommand cmd = stage.evaluate(ctx);
        CHECK(cmd.command == Catch::Approx(0.6).epsilon(1e-12));
    }

    SECTION("floor/ceiling are fixed 0/1 regardless of what ctx.tcu_throttle_floor/ceiling ask for") {
        ThrottleStage stage;
        vehicle::Curve1D no_curve;
        stage.configure(ThrottleInput::Mechanical, false, no_curve, -1);

        // Deliberately hostile ctx values that WOULD clip 0.6 down to
        // [0.5, 0.55] if this stage were electronic - a mechanical linkage
        // has no wire back to the pedal, so these must be ignored entirely.
        const ComponentSubstepCtx ctx = make_ctx(/*throttle=*/0.6, /*floor=*/0.5, /*ceiling=*/0.55);
        const ThrottleCommand cmd = stage.evaluate(ctx);
        CHECK(cmd.command == Catch::Approx(0.6).epsilon(1e-12));
        CHECK(cmd.floor == Catch::Approx(0.0).epsilon(1e-12));
        CHECK(cmd.ceiling == Catch::Approx(1.0).epsilon(1e-12));

        EngineTorqueMap eng(make_test_desc());
        TcuRequest tcu;
        const auto r = eng.pre_solve(running_sensors(omega), cmd, tcu, 1.0 / 960.0);
        CHECK(r.throttle_eff == Catch::Approx(0.6).epsilon(1e-9)); // NOT clipped to 0.55
    }

    SECTION("the upshift ignition cut still works on a mechanical linkage (it bypasses the throttle path entirely)") {
        ThrottleStage stage;
        vehicle::Curve1D no_curve;
        stage.configure(ThrottleInput::Mechanical, false, no_curve, -1);

        TcuRequest cut;
        cut.ignition_cut = true;

        EngineTorqueMap eng_a(make_test_desc());
        const ThrottleCommand cmd_low = stage.evaluate(make_ctx(0.2));
        const auto r_a = eng_a.pre_solve(running_sensors(omega), cmd_low, cut, 1.0 / 960.0);

        EngineTorqueMap eng_b(make_test_desc());
        const ThrottleCommand cmd_high = stage.evaluate(make_ctx(1.0));
        const auto r_b = eng_b.pre_solve(running_sensors(omega), cmd_high, cut, 1.0 / 960.0);

        // Same argument as P2's ignition_cut SECTION: with the request
        // forced to 0, a mechanical stage's 0.2 vs 1.0 pedal must produce
        // identical torque - proving the cut acts on TcuRequest, not on
        // anything the mechanical linkage carries.
        CHECK(r_a.throttle_eff == Catch::Approx(r_b.throttle_eff).epsilon(1e-12));
        CHECK(r_a.torque_estimate == Catch::Approx(r_b.torque_estimate).epsilon(1e-12));
    }
}

// --- P5: the loader's own throttle_control rejection list (plan 3.4.5) -----
namespace {

// RAII temp file (same pattern as test_vehicle_io_validation.cpp's
// TempJsonFile - duplicated locally rather than shared across translation
// units).
struct TempJsonFile {
    std::filesystem::path path;
    TempJsonFile(const std::string& contents, const char* tag) {
        path = std::filesystem::temp_directory_path() /
               (std::string("ps_test_throttle_control_") + tag + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json");
        std::ofstream out(path);
        out << contents;
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

// A minimal but otherwise-complete engine component - load_vehicle_json
// never builds a Powertrain (that graph-build-time work is powertrain.cpp's
// job, not the loader's), so "shafts"/gearbox/clutch/diff are NOT required
// to exercise the throttle_control component's own field validation.
std::string minimal_engine_json(const char* throttle_input_field = "") {
    return std::string(R"({ "id": "engine", "type": "engine", "kind": "torque_map",
      "wot_torque_nm_vs_rpm": { "x": [900, 6500], "y": [150, 150] },
      "motoring_torque_nm_vs_rpm": { "x": [0, 6500], "y": [-20, -60] },
      "inertia_kgm2": 0.16, "breakaway_torque_nm": 25.0)") +
           throttle_input_field + " }";
}

std::string minimal_vehicle_json_with_components(const std::string& components_csv) {
    return std::string(R"({
  "format": "physics_sim.vehicle/2",
  "source": "test fixture, not a real vehicle",
  "wheels": [ { "name": "FL" }, { "name": "FR" } ],
  "powertrain": {
    "components": [
      )") + components_csv + R"(
    ]
  }
})";
}

} // namespace

TEST_CASE("P5: load_vehicle_json rejects a malformed throttle_control component", "[unit][vehicle][io][drivetrain][throttle]") {
    SECTION("a second throttle_control component is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "electronic" })";
        const std::string tc2 = R"({ "id": "tc2", "type": "throttle_control", "engine": "engine", "kind": "electronic" })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc + ", " + tc2), "duplicate");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("more than one throttle_control component - at most one is supported"));
    }
    SECTION("missing 'engine' is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "kind": "electronic" })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "no_engine");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("throttle_control 'engine' (a component id reference) is required"));
    }
    SECTION("empty-string 'engine' is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "", "kind": "electronic" })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "empty_engine");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("throttle_control 'engine' (a component id reference) is required"));
    }
    SECTION("'engine' referencing an unknown component id is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "no_such_component", "kind": "electronic" })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "unknown_engine");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("must name a \"torque_map\" engine component"));
    }
    SECTION("missing 'kind' is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine" })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "no_kind");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring(R"(throttle_control 'kind' must be "electronic" or "mechanical_linkage", got "")"));
    }
    SECTION("an unrecognised 'kind' string is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "cable" })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "bad_kind");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring(R"(throttle_control 'kind' must be "electronic" or "mechanical_linkage", got "cable")"));
    }
    SECTION("pedal_map on a mechanical_linkage stage is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "mechanical_linkage",
          "pedal_map": { "x": [0, 1], "y": [0, 1] } })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "pedal_map_on_mechanical");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("throttle_control 'pedal_map' is only valid for kind \"electronic\""));
    }
    SECTION("linkage_curve on an electronic stage is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "electronic",
          "linkage_curve": { "x": [0, 1], "y": [0, 1] } })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "linkage_on_electronic");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("throttle_control 'linkage_curve' is only valid for kind \"mechanical_linkage\""));
    }
    SECTION("pedal_map.x not ending exactly at 1 is rejected") {
        // PHYS-033 P4 I7a C3 review item 4: x.back() must be exactly 1 (this
        // superseded the old generic "x must be within [0, 1]" range check -
        // see test_vehicle_io_validation.cpp's dedicated curve-rules test for
        // the full set of endpoint/monotonicity rejections).
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "electronic",
          "pedal_map": { "x": [0, 1.5], "y": [0, 1] } })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "pedal_map_x_range");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("x.back() must be exactly 1"));
    }
    SECTION("pedal_map.y outside [0, 1] is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "electronic",
          "pedal_map": { "x": [0, 1], "y": [0, 1.2] } })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "pedal_map_y_range");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("pedal_map: y must be within [0, 1]"));
    }
    SECTION("linkage_curve.x not starting exactly at 0 is rejected") {
        // PHYS-033 P4 I7a C3 review item 4: x.front() must be exactly 0 (see
        // the pedal_map SECTION above for the same supersession note).
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "mechanical_linkage",
          "linkage_curve": { "x": [-0.1, 1], "y": [0, 1] } })";
        TempJsonFile f(minimal_vehicle_json_with_components(
                           minimal_engine_json(R"(, "throttle_input": "mechanical")") + ", " + tc),
                       "linkage_x_range");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("x.front() must be exactly 0"));
    }
    SECTION("linkage_curve.y outside [0, 1] is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "mechanical_linkage",
          "linkage_curve": { "x": [0, 1], "y": [-0.1, 1] } })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "linkage_y_range");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("linkage_curve: y must be within [0, 1]"));
    }
    SECTION("a non-ascending pedal_map is rejected") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "electronic",
          "pedal_map": { "x": [0.5, 0.2, 1.0], "y": [0, 0.5, 1] } })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "pedal_map_nonascending");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("pedal_map: x values must be strictly ascending"));
    }
    SECTION("a throttle_input on the engine outside electronic/mechanical is rejected") {
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json(R"(, "throttle_input": "cable")")),
                       "bad_throttle_input");
        REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()),
                             Catch::Matchers::ContainsSubstring("throttle_input must be \"electronic\" or \"mechanical\""));
    }
    SECTION("a well-formed electronic throttle_control with a pedal_map and a hand lever round-trips") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "electronic",
          "pedal_map": { "x": [0, 1], "y": [0, 0.9] }, "hand_lever_channel": "hand_throttle" })";
        TempJsonFile f(minimal_vehicle_json_with_components(minimal_engine_json() + ", " + tc), "well_formed_electronic");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        REQUIRE(d.powertrain.components.size() == 2);
        const auto& tcd = std::get<ThrottleControlDesc>(d.powertrain.components[1].params);
        CHECK(tcd.engine == "engine");
        CHECK(tcd.kind == ThrottleInput::Electronic);
        CHECK(tcd.has_curve);
        REQUIRE(tcd.curve.x.size() == 2);
        CHECK(tcd.curve.y[1] == Catch::Approx(0.9));
        CHECK(tcd.hand_lever_channel == "hand_throttle");
    }
    SECTION("a well-formed mechanical throttle_control with a linkage_curve round-trips") {
        const std::string tc = R"({ "id": "tc", "type": "throttle_control", "engine": "engine", "kind": "mechanical_linkage",
          "linkage_curve": { "x": [0, 1], "y": [0, 0.6] } })";
        // PHYS-033 P4 I7a C3 review item 2: an explicit stage's 'kind' must
        // agree with its engine's throttle_input, so this fixture's engine
        // must declare "mechanical" too (minimal_engine_json() defaults to
        // "electronic").
        TempJsonFile f(minimal_vehicle_json_with_components(
                           minimal_engine_json(R"(, "throttle_input": "mechanical")") + ", " + tc),
                       "well_formed_mechanical");
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        const auto& tcd = std::get<ThrottleControlDesc>(d.powertrain.components[1].params);
        CHECK(tcd.kind == ThrottleInput::Mechanical);
        CHECK(tcd.has_curve);
        CHECK(tcd.hand_lever_channel.empty());
    }
}

// --- P6: tractor governor path regression -----------------------------------
//
// tractor.json's engine (diesel_i4_4l5_turbo.json) has an all-speed governor
// (high_idle_rpm 2400, rated_rpm 2200, droop 0.0909, channel "hand_throttle")
// and no explicit throttle_control component, so Powertrain builds the
// IMPLICIT default stage: kind = the engine's own throttle_input
// (Electronic, tractor.json's default), no pedal_map, and a hand lever
// wired to the governor's own channel (powertrain.cpp's
// `else if (e.has_governor) hand_lever_channel = e.governor.channel;`).
//
// Before C3, EngineTorqueMap::pre_solve computed this internally as a
// single inline `lever = max(hand_throttle, pedal_throttle)` read off
// Sensors, then fed the governor formula. C3 splits that into
// ThrottleStage::evaluate() (the max()) followed by the engine's own
// governor arithmetic (engine_torque_map.cpp, unchanged in shape - see
// this file's own comment: "no separate max()/clamp() left to do"). This
// test pins that the two-stage composition reproduces EXACTLY the same
// numbers the pre-C3 monolithic formula would have, using the SAME
// governor constants as tractor.json's real engine.
//
// The "pre-C3 reference values" below are computed by an INDEPENDENT
// re-derivation of the governor formula (plan section 3.6, quoted in this
// file's header comments and docs/data_formats/engine.md) written directly
// in this test, not by calling into any ThrottleStage/EngineTorqueMap code
// path - i.e. they are what the OLD single-function implementation would
// have produced for the same (pedal, hand_throttle, rpm) triples, verified
// by hand before this test was written. Full end-to-end parity for the real
// tractor.json vehicle over a 10 s drive is separately confirmed by this
// worktree's unchanged vehicle_tractor_i4.json state_hash
// (0xac386af7129b9ffe at 1/2/4/10/24 workers) recorded in this commit's
// handback report.
TEST_CASE("S6: ThrottleStage + governed EngineTorqueMap reproduces the pre-C3 max(hand,pedal) governor formula (stage-level)",
          "[unit][drivetrain][throttle][governor]") {
    // Mirrors tractor.json's diesel_i4_4l5_turbo.json governor constants
    // exactly (high_idle_rpm/rated_rpm/droop), on the synthetic
    // make_governed_desc() engine so the WOT/motoring curves stay
    // hand-computable; only the governor block needs to match the real
    // vehicle for this to be a faithful regression pin.
    EngineTorqueMap::Desc desc = make_test_desc();
    desc.has_governor = true;
    desc.governor.high_idle_rpm = 2400.0;
    desc.governor.rated_rpm = 2200.0;
    desc.governor.droop = 0.0909; // band = 0.0909 * 2200 = 199.98

    struct Case {
        real pedal, hand, rpm;
    };
    const Case cases[] = {
        {0.0, 0.6, 1500.0},  // hand lever wins (tractor's normal operating mode: pedal at rest)
        {0.8, 0.6, 1500.0},  // pedal wins (max(0.8, 0.6))
        {0.3, 0.3, 2350.0},  // tie, near the top of the droop band
        {0.0, 0.0, 900.0},   // both zero: governor still sets idle via n_nl = idle_rpm
    };

    for (const auto& c : cases) {
        INFO("pedal=" << c.pedal << " hand=" << c.hand << " rpm=" << c.rpm);

        // Independent pre-C3 reference: lever = max(hand, pedal); n_nl =
        // idle_rpm + lever*(high_idle_rpm - idle_rpm); band =
        // droop*rated_rpm; theta_gov = clamp((n_nl - rpm)/band, 0, 1).
        const real lever_ref = std::max(c.hand, c.pedal);
        const real n_nl_ref = desc.idle_rpm + lever_ref * (desc.governor.high_idle_rpm - desc.idle_rpm);
        const real band_ref = desc.governor.droop * desc.governor.rated_rpm;
        const real theta_gov_ref = std::clamp((n_nl_ref - c.rpm) / band_ref, real(0.0), real(1.0));

        // C3 composition: ThrottleStage combines pedal/hand, THEN the
        // engine's own (unchanged) governor arithmetic runs on that command.
        ThrottleStage stage;
        vehicle::Curve1D no_curve;
        stage.configure(ThrottleInput::Electronic, false, no_curve, /*hand_lever_channel_slot=*/0);
        const real channels[1] = {c.hand};
        ComponentSubstepCtx ctx = make_ctx(c.pedal);
        ctx.channels = channels;
        ctx.channel_count = 1;
        const ThrottleCommand cmd = stage.evaluate(ctx);
        CHECK(cmd.command == Catch::Approx(lever_ref).epsilon(1e-12));

        EngineTorqueMap eng(desc);
        TcuRequest tcu;
        const auto r = eng.pre_solve(running_sensors(rpm_to_omega(c.rpm)), cmd, tcu, 1.0 / 960.0);

        // At rpm=900==idle_rpm the idle PI could also be active; both paths
        // still agree because theta_eff = max(theta_req, theta_idle) and
        // theta_req here IS theta_gov (no ignition cut, floor/ceiling wide
        // open) - r.throttle_eff is bounded below by theta_idle either way,
        // so comparing throttle_eff against max(theta_gov_ref, 0) alone
        // would only be valid where theta_idle is 0; assert governor_setpoint
        // and the r.throttle_eff >= theta_gov_ref invariant instead to stay
        // correct at every case including the idle one.
        CHECK(r.governor_setpoint_rpm == Catch::Approx(n_nl_ref).epsilon(1e-9));
        CHECK(r.throttle_eff >= Catch::Approx(theta_gov_ref).epsilon(1e-9).margin(1e-9));
        if (c.rpm > desc.idle_rpm + real(50.0)) {
            // Comfortably above idle: the idle PI cannot be raising
            // throttle_eff above theta_gov (e = idle_rpm - rpm is very
            // negative), so this is an exact check.
            CHECK(r.throttle_eff == Catch::Approx(theta_gov_ref).epsilon(1e-9));
        }
    }
}

// ===========================================================================
// C3 REVIEW: Powertrain-level P2/P3/P4/P6, plus review item 13 (a
// "hand_throttle" channel on car_sedan has no effect). Every case below
// drives a real Powertrain (or a real World), never a bare ThrottleStage.
// ===========================================================================
namespace {

constexpr real kSubstepH = 1.0 / 960.0;

Powertrain build_pt(const vehicle::VehicleDesc& d) { return Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name); }

const TorqueMapEngineDesc& engine_desc_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) return *e;
    }
    throw std::runtime_error("engine_desc_of: no torque_map engine component");
}

const ThrottleControlDesc* stage_desc_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* t = std::get_if<ThrottleControlDesc>(&c.params)) return t;
    }
    return nullptr;
}

// car_sedan.json plus an explicit electronic throttle_control stage with the
// given pedal_map (car_sedan.json itself has none - its implicit stage is
// the identity).
vehicle::VehicleDesc sedan_with_pedal_map(std::vector<real> x, std::vector<real> y) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    ThrottleControlDesc t;
    t.engine = "engine";
    t.kind = ThrottleInput::Electronic;
    t.has_curve = true;
    t.curve.x = std::move(x);
    t.curve.y = std::move(y);
    d.powertrain.components.push_back(ComponentDesc{"throttle", t});
    return d;
}

vehicle::VehicleDesc load_mechanical_fixture() {
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicle_mechanical_throttle.json");
}

void reset_running_in_gear(Powertrain& pt, int gear, real hub_omega) {
    vehicle::VehicleResetOptions o;
    o.gear = gear;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    pt.reset(o, std::vector<real>(4, hub_omega));
}

// Same as reset_running_in_gear, but clutch_locked=false: the dog is already
// in `gear` (not Neutral) while ManualTcu's auto-clutch mode is Launch, e_auto
// starting at 0 - i.e. "the driver just selected 1st and is now easing the
// clutch out", the exact state PHYS-033 P4 I7a C5b's launch law (manual_tcu.cpp's
// "full = full_rpm_min + throttle_request * (full_rpm_max - full_rpm_min)")
// acts in. Used instead of a real Neutral->1st shift (which needs a whole
// Selecting/Synchronising gearbox dynamic to reach gearbox_current != 0) to
// isolate the launch law itself.
void reset_launch_in_gear(Powertrain& pt, int gear, real hub_omega) {
    vehicle::VehicleResetOptions o;
    o.gear = gear;
    o.engine_state = EngineState::Running;
    o.clutch_locked = false;
    pt.reset(o, std::vector<real>(4, hub_omega));
}

// Drives a Powertrain one 960 Hz substep at a time with every hub held at
// hub_target by a stiff implicit "dyno" (Assembly::torque semantics
// tau(w') = tau0 - d*(w' - w_old): tau0 = -K*(w_old - target), d = K), so
// the engine sits at a fixed, known rpm whatever torque it makes.
struct DynoDriver {
    Powertrain& pt;
    ComponentSubstepCtx ctx;
    real hub_target;
    real k = 5000.0;
    long substep = 0;

    DynoDriver(Powertrain& p, real target) : pt(p), hub_target(target) {
        ctx.h = kSubstepH;
        ctx.ignition = 1.0;
        ctx.clutch_pedal = 0.0;
        ctx.gear_request = kGearRequestNone;
        ctx.assist_auto_clutch = false;
        ctx.assist_auto_blip = false;
        ctx.assist_auto_shift = false;
    }
    void step() {
        ctx.first_substep_of_tick = (substep % 4) == 0;
        pt.begin_substep();
        pt.pre_solve_components(ctx);
        for (std::size_t i = 0; i < 4; ++i) {
            const real w_old = pt.hub_omega(i);
            pt.add_hub_terms(i, -k * (w_old - hub_target), k);
        }
        pt.solve(ctx.h);
        pt.post_solve_components(ctx);
        ++substep;
    }
};

bool same_bits(real a, real b) { return std::memcmp(&a, &b, sizeof(real)) == 0; }

// Name of the first PreSolveResult field that differs BITWISE, or "".
std::string first_differing_field(const PreSolveResult& a, const PreSolveResult& b) {
    const std::pair<const char*, std::pair<real, real>> reals[] = {
        {"tau0", {a.tau0, b.tau0}},
        {"d", {a.d, b.d}},
        {"friction_cap_static", {a.friction_cap_static, b.friction_cap_static}},
        {"friction_cap_kinetic", {a.friction_cap_kinetic, b.friction_cap_kinetic}},
        {"rpm", {a.rpm, b.rpm}},
        {"throttle_eff", {a.throttle_eff, b.throttle_eff}},
        {"torque_estimate", {a.torque_estimate, b.torque_estimate}},
        {"governor_setpoint_rpm", {a.governor_setpoint_rpm, b.governor_setpoint_rpm}},
        {"torque_demand", {a.torque_demand, b.torque_demand}},
        {"theta_c", {a.theta_c, b.theta_c}},
    };
    for (const auto& [name, v] : reals) {
        if (!same_bits(v.first, v.second)) return name;
    }
    if (a.state != b.state) return "state";
    if (a.limiter_active != b.limiter_active) return "limiter_active";
    if (a.dfco_active != b.dfco_active) return "dfco_active";
    if (a.starter_active != b.starter_active) return "starter_active";
    return {};
}

// A World with a static ground box and a car_sedan-sized chassis (the shape
// test_powertrain_control_channels.cpp's make_minimal_rig uses), one vehicle
// created with the given test-only extra channels, 1 worker, reset Running
// in neutral.
struct WorldRig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
};

WorldRig make_world_rig(const vehicle::VehicleDesc& desc, const std::vector<std::pair<std::string, real>>& extra_channels = {}) {
    WorldRig rig;
    WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    rig.world = std::make_unique<World>(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    rig.world->create_body(ground);

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId chassis_id = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
    rig.vehicle = rig.world->create_vehicle(desc, chassis_id, extra_channels);

    // Every TCU assist off and the clutch pedal released: the ManualTcu
    // reads the RAW pedal (docs/data_formats/vehicle.md, review item 7), and
    // only its auto-clutch launch law acts on it - with the assists off its
    // outputs cannot depend on the pedal.
    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 0;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

} // namespace

TEST_CASE("P2: two pedal values on a flat pedal_map plateau are indistinguishable at the Powertrain level", "[unit][drivetrain][throttle]") {
    // pedal_map plateau: every pedal in [0.4, 0.6] maps to exactly 0.3
    // (Curve1D: y0 + t*(y1 - y0) with y1 == y0 is exactly y0).
    const vehicle::VehicleDesc d = sedan_with_pedal_map({0.0, 0.4, 0.6, 1.0}, {0.0, 0.3, 0.3, 1.0});
    constexpr real kPedalA = 0.45;
    constexpr real kPedalB = 0.55;

    SECTION("white-box: every engine output, the engine's own state hash and the Powertrain hash, per substep") {
        // Gear 1 (3.54 * 4.06), hubs held at 18 rad/s: engine ~2470 rpm.
        constexpr real kHub = 18.0;
        Powertrain pa = build_pt(d);
        Powertrain pb = build_pt(d);
        reset_running_in_gear(pa, 1, kHub);
        reset_running_in_gear(pb, 1, kHub);
        DynoDriver da(pa, kHub);
        DynoDriver db(pb, kHub);
        da.ctx.throttle = kPedalA;
        db.ctx.throttle = kPedalB;

        constexpr int kSubsteps = 1920; // 2 s
        constexpr std::uint64_t kSeed = 14695981039346656037ULL;
        int tcu_not_neutral = 0;
        int mismatches = 0;
        int first_mismatch_substep = -1;
        std::string first_mismatch_field;
        for (int s = 0; s < kSubsteps; ++s) {
            // What the engine sees THIS substep (the TCU output is lagged one
            // substep): no floor, no ceiling, no cut - the stage's own output
            // is the only throttle input the engine gets.
            for (const Powertrain* p : {&pa, &pb}) {
                const TcuThrottleLimits& r = p->tcu_request_debug();
                if (r.throttle_floor != 0.0 || r.throttle_ceiling != 1.0 || r.ignition_cut) ++tcu_not_neutral;
            }
            da.step();
            db.step();
            std::string f = first_differing_field(pa.last_engine_result(), pb.last_engine_result());
            if (f.empty() && !same_bits(pa.throttle_command_debug().command, pb.throttle_command_debug().command)) f = "throttle_command.command";
            if (f.empty() && !same_bits(pa.throttle_eff(), pb.throttle_eff())) f = "throttle_eff()";
            if (f.empty() && !same_bits(pa.engine_torque_demand(), pb.engine_torque_demand())) f = "engine_torque_demand()";
            if (f.empty() && pa.engine().fold_state_hash(kSeed) != pb.engine().fold_state_hash(kSeed)) f = "engine().fold_state_hash";
            if (f.empty() && pa.fold_state_hash(kSeed) != pb.fold_state_hash(kSeed)) f = "Powertrain::fold_state_hash";
            if (!f.empty() && mismatches++ == 0) {
                first_mismatch_substep = s;
                first_mismatch_field = f;
            }
        }
        INFO("first mismatch at substep " << first_mismatch_substep << ": " << first_mismatch_field << " (" << mismatches << " of "
                                          << kSubsteps << " substeps differ)");
        CHECK(tcu_not_neutral == 0);
        CHECK(mismatches == 0);
        // Non-vacuous: both pedals really went through the plateau, and the
        // engine ran on it (not idling, not cut, not on the limiter).
        CHECK(pa.throttle_command_debug().command == 0.3);
        CHECK(pa.throttle_eff() == 0.3);
        CHECK(pa.last_engine_result().state == EngineState::Running);
        CHECK(pa.last_engine_result().rpm > 2000.0);
        CHECK(pa.last_engine_result().rpm < 3000.0);
        CHECK_FALSE(pa.last_engine_result().limiter_active);
    }

    SECTION("whole World: state_hash per tick (neutral, engine free-revving on the plateau)") {
        WorldRig ra = make_world_rig(d);
        WorldRig rb = make_world_rig(d);
        ra.world->set_control("throttle", kPedalA);
        rb.world->set_control("throttle", kPedalB);
        int mismatches = 0;
        int first = -1;
        for (int t = 0; t < 480; ++t) {
            ra.world->step();
            rb.world->step();
            if (ra.world->state_hash() != rb.world->state_hash() && mismatches++ == 0) first = t;
        }
        INFO("first differing tick " << first << " (" << mismatches << " of 480)");
        CHECK(mismatches == 0);
        CHECK(ra.world->powertrain_state(ra.vehicle).engine_rpm > 3000.0); // non-vacuous: the engine revved on the plateau
    }
}

TEST_CASE("P3: dyno-held rpm - throttle_eff and engine_torque_demand follow the pedal_map-reshaped command (1e-12)", "[unit][drivetrain][throttle]") {
    const vehicle::VehicleDesc d = sedan_with_pedal_map({0.0, 0.5, 1.0}, {0.0, 0.2, 1.0});
    const TorqueMapEngineDesc& e = engine_desc_of(d);
    const ThrottleControlDesc* stage = stage_desc_of(d);
    REQUIRE(stage != nullptr);

    // Reference idle PI, from the desc (engine_torque_map.cpp's own formula):
    // theta_idle = clamp(theta_ff + kp*(idle_rpm - rpm) + I, 0, max_throttle).
    const real w_idle = e.wot_torque_nm_vs_rpm.evaluate(e.idle_rpm);
    const real m_idle = e.motoring_torque_nm_vs_rpm.evaluate(e.idle_rpm);
    const real theta_ff = std::clamp(-m_idle / (w_idle - m_idle), real(0.0), e.idle.max_throttle);

    constexpr real kHub = 18.0; // gear 1: ~2470 rpm
    Powertrain pt = build_pt(d);
    reset_running_in_gear(pt, 1, kHub);
    DynoDriver drv(pt, kHub);

    for (const real pedal : {0.25, 0.5, 0.8}) {
        INFO("pedal " << pedal);
        const real mapped = stage->curve.evaluate(pedal);
        REQUIRE(std::abs(mapped - pedal) > 0.05); // the map genuinely reshapes: a raw-pedal leak would show
        drv.ctx.throttle = pedal;
        real worst_eff = 0.0;
        real worst_demand_rel = 0.0;
        for (int s = 0; s < 480; ++s) {
            const real integrator_before = pt.idle_integrator();
            drv.step();
            const PreSolveResult& r = pt.last_engine_result();
            REQUIRE(r.state == EngineState::Running);
            REQUIRE_FALSE(r.limiter_active);
            REQUIRE_FALSE(r.dfco_active);
            const real theta_idle =
                std::clamp(theta_ff + e.idle.kp_per_rpm * (e.idle_rpm - r.rpm) + integrator_before, real(0.0), e.idle.max_throttle);
            const real theta_eff_ref = std::max(mapped, theta_idle); // floor 0, ceiling 1, no cut (assists off)
            const real m = e.motoring_torque_nm_vs_rpm.evaluate(r.rpm);
            const real w = e.wot_torque_nm_vs_rpm.evaluate(r.rpm);
            const real demand_ref = m + theta_eff_ref * (w - m);
            worst_eff = std::max(worst_eff, std::abs(pt.throttle_eff() - theta_eff_ref));
            worst_demand_rel =
                std::max(worst_demand_rel, std::abs(pt.engine_torque_demand() - demand_ref) / std::max(real(1.0), std::abs(demand_ref)));
        }
        CHECK(pt.last_engine_result().rpm > 2000.0);
        CHECK(pt.last_engine_result().rpm < 3000.0);
        CHECK(worst_eff <= 1e-12);
        CHECK(worst_demand_rel <= 1e-12);
        CHECK(pt.throttle_eff() == mapped); // theta_idle is 0 this far above idle_rpm
    }
}

TEST_CASE("P4: mechanical fixture - linkage_curve (explicit) or identity (implicit), fixed 0/1 floor/ceiling, theta_c 0 during a real upshift cut",
          "[unit][drivetrain][throttle]") {
    const vehicle::VehicleDesc explicit_desc = load_mechanical_fixture();
    REQUIRE(engine_desc_of(explicit_desc).throttle_input == ThrottleInput::Mechanical);
    const ThrottleControlDesc* stage = stage_desc_of(explicit_desc);
    REQUIRE(stage != nullptr);
    REQUIRE(stage->kind == ThrottleInput::Mechanical);
    REQUIRE(stage->has_curve);

    // The implicit stage: same fixture with its throttle_control component
    // erased in code (plan section 3.4: a mechanical engine with no stage
    // gets an implicit mechanical stage with NO curve - the identity).
    vehicle::VehicleDesc implicit_desc = explicit_desc;
    auto& comps = implicit_desc.powertrain.components;
    comps.erase(std::remove_if(comps.begin(), comps.end(),
                               [](const ComponentDesc& c) { return std::holds_alternative<ThrottleControlDesc>(c.params); }),
                comps.end());
    REQUIRE(stage_desc_of(implicit_desc) == nullptr);

    struct Variant {
        const char* name;
        const vehicle::VehicleDesc* desc;
        bool linkage;
    };
    for (const Variant& v : {Variant{"explicit mechanical_linkage stage", &explicit_desc, true},
                             Variant{"implicit mechanical stage", &implicit_desc, false}}) {
        INFO(v.name);
        Powertrain pt = build_pt(*v.desc);
        // Gear 1, hubs held at 36 rad/s: ~4940 rpm; 2nd would be ~2860 rpm.
        constexpr real kHub = 36.0;
        reset_running_in_gear(pt, 1, kHub);
        DynoDriver drv(pt, kHub);
        drv.ctx.assist_auto_clutch = true;
        drv.ctx.assist_auto_blip = true;

        int bad_command = 0, bad_floor_ceiling = 0, cut_substeps = 0, tcu_ceiling_below_one = 0, cut_with_combustion = 0;
        for (int s = 0; s < 1920; ++s) {
            drv.ctx.throttle = s < 240 ? 0.3 : 0.65;
            if (s == 240) drv.ctx.gear_request = 2.0; // a real TCU upshift 1 -> 2
            const TcuThrottleLimits req = pt.tcu_request_debug(); // what the engine sees this substep (lagged)
            drv.step();
            const ThrottleCommand& cmd = pt.throttle_command_debug();
            const real expected = v.linkage ? stage->curve.evaluate(drv.ctx.throttle) : drv.ctx.throttle;
            if (cmd.command != expected) ++bad_command;
            if (cmd.floor != 0.0 || cmd.ceiling != 1.0) ++bad_floor_ceiling;
            if (req.throttle_ceiling < 1.0) ++tcu_ceiling_below_one;
            if (req.ignition_cut) {
                ++cut_substeps;
                if (pt.last_engine_result().theta_c != 0.0) ++cut_with_combustion;
            }
        }
        CHECK(bad_command == 0);
        CHECK(bad_floor_ceiling == 0);
        CHECK(cut_substeps > 0);          // the upshift cut really happened
        CHECK(tcu_ceiling_below_one > 0); // ... and the TCU asked for a ceiling < 1 that the mechanical stage ignored
        CHECK(cut_with_combustion == 0);
        CHECK(pt.gearbox().current_position() == 2);
    }
    // The linkage curve genuinely reshapes (a raw-pedal pass-through in the
    // explicit variant would have failed bad_command above).
    CHECK(stage->curve.evaluate(0.3) == 0.12);
    CHECK(std::abs(stage->curve.evaluate(0.65) - 0.65) > 0.05);
}

TEST_CASE("P6: tractor governor path over vehicle_tractor_i4.json's control events matches the pre-C3 master recording exactly",
          "[unit][drivetrain][throttle][vehicle]") {
    // The expected values below were recorded on master ba0f486 (pre-C3) by
    // a throwaway TEST_CASE in a scratch worktree (never committed), running
    // EXACTLY this loop: load data/scenarios/vehicle_tractor_i4.json, 1 job
    // worker, build_scenario, and each tick apply every control event with
    // time_s <= sim_time() (ps_run's own rule) before World::step(); after
    // the step fold the five doubles {snapshot.hand_throttle,
    // snapshot.governor_setpoint_rpm, debug.throttle_eff,
    // debug.engine_torque_demand, debug.idle_integrator} byte-wise into one
    // FNV-1a hash over all fields and one per field, and print ticks
    // 300/1200/2000 as hex floats. Compared with == (hash equality is bit
    // equality of every value on every one of the 2400 ticks).
    // Re-recorded 2026-10-04 (vault PHYS-040): ca118ec made the auto-clutch
    // anti-stall release a locked clutch at once (engine OR signed input rpm
    // below anti_stall_rpm) instead of ramping it at 40/s, which changes the
    // braking-to-a-stop phase only: hand_throttle and governor_setpoint_rpm
    // and the tick 300/1200 spot values are still the ba0f486 recording;
    // throttle_eff/engine_torque_demand/idle_integrator and tick 2000 moved
    // (were 0x6443ff93d68468d5 / 0xd6ab85ab58668bde / 0x3d3d46be580ce397,
    // all 0x5342f8c7ba48dfe9).
    // Re-recorded 2026-10-06 (tyre rolling-resistance work): the tractor tyres
    // now derive qsy3/qsy4 from a rolling_resistance block (rated 65 km/h, crr
    // 1.25x base at 40 km/h, was +5 %), which moves the vehicle dynamics and
    // therefore throttle_eff/engine_torque_demand/idle_integrator at every
    // tick (were 0x4a4dec2dbe22c70d / 0xc00e022ed2f37e06 / 0xcf9faa9e08c73845,
    // all 0x5fd1a1eb03f6e1ab); hand_throttle and governor_setpoint_rpm are
    // control-event driven and unchanged. The spot values below are the new
    // recording; this test is now a frozen regression of the governor path,
    // no longer a comparison against the pre-C3 code.
    auto fnv = [](std::uint64_t h, double v) {
        unsigned char b[8];
        std::memcpy(b, &v, 8);
        for (const unsigned char c : b) {
            h ^= c;
            h *= 1099511628211ULL;
        }
        return h;
    };
    io::Scenario sc = io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/vehicle_tractor_i4.json");
    sc.world_config.job_workers = 1;
    io::BuiltScenario built = io::build_scenario(sc);
    World& world = *built.world;
    const VehicleId vehicle = built.vehicles_by_name.at("tractor");
    const int ticks = static_cast<int>(sc.duration_s * sc.world_config.tick_rate_hz + 0.5);
    std::size_t next_event = 0;
    std::uint64_t h_all = 14695981039346656037ULL;
    std::uint64_t h_field[5] = {h_all, h_all, h_all, h_all, h_all};
    double spot[3][5] = {};
    for (int i = 0; i < ticks; ++i) {
        while (next_event < sc.control_events.size() && sc.control_events[next_event].time_s <= world.sim_time()) {
            world.set_control(sc.control_events[next_event].channel, sc.control_events[next_event].value);
            ++next_event;
        }
        world.step();
        const auto& s = world.powertrain_state(vehicle);
        const auto dbg = world.powertrain_debug(vehicle);
        const double v[5] = {s.hand_throttle, s.governor_setpoint_rpm, dbg.throttle_eff, dbg.engine_torque_demand, dbg.idle_integrator};
        for (int k = 0; k < 5; ++k) {
            h_all = fnv(h_all, v[k]);
            h_field[k] = fnv(h_field[k], v[k]);
        }
        const int slot = i == 300 ? 0 : i == 1200 ? 1 : i == 2000 ? 2 : -1;
        if (slot >= 0) std::memcpy(spot[slot], v, sizeof v);
    }
    REQUIRE(ticks == 2400);
    CHECK(h_field[0] == 0x09c190b1bd78f7f9ULL); // hand_throttle
    CHECK(h_field[1] == 0x1ece9dfdcfa3ddecULL); // governor_setpoint_rpm
    CHECK(h_field[2] == 0xe98a0abededffaf0ULL); // throttle_eff
    CHECK(h_field[3] == 0x0c44a2d22c8a534dULL); // engine_torque_demand
    CHECK(h_field[4] == 0xb405d2355f44b66cULL); // idle_integrator
    CHECK(h_all == 0xc0cbc7fdc6150a48ULL);
    const double expected[3][5] = {
        {0x1.3333333333333p-1, 0x1.bd00000000000p+10, 0x1.ff2a1524a2dddp-3, 0x1.6c7680ec27068p+6, -0x1.b5de0d29ea85bp-4},
        {0x1.3333333333333p-1, 0x1.bd00000000000p+10, 0x1.e4d9f9a92e738p-4, 0x1.028e6a8a8b010p+3, -0x1.b5de0d29ea85bp-4},
        {0x0.0000000000000p+0, 0x1.a900000000000p+9, 0x1.ab9d39d427288p-4, -0x1.2c3be4b3887a0p+0, 0x1.d28ab98965e3cp-9},
    };
    for (int t = 0; t < 3; ++t) {
        for (int k = 0; k < 5; ++k) {
            INFO("spot tick " << (t == 0 ? 300 : t == 1 ? 1200 : 2000) << " field " << k);
            CHECK(spot[t][k] == expected[t][k]);
        }
    }
}

TEST_CASE("car_sedan: a \"hand_throttle\" control channel has no effect (its implicit stage has no hand lever)",
          "[unit][drivetrain][throttle][vehicle]") {
    // The channel is registered as a real component channel (World::
    // create_vehicle's test-only extra-channel hook), so it genuinely reaches
    // ComponentSubstepCtx::channels - only the stage's own configuration (no
    // hand_lever_channel; car_sedan's engine has no governor) keeps it out.
    const vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    REQUIRE(stage_desc_of(d) == nullptr);
    REQUIRE_FALSE(engine_desc_of(d).has_governor);

    struct Trace {
        std::vector<std::uint64_t> hash;
        std::vector<real> eff, demand, integrator;
        int snapshot_hand_nonzero = 0;
        std::vector<real> channel_values;
    };
    auto run = [&d](real hand) {
        WorldRig rig = make_world_rig(d, {{"hand_throttle", 0.0}});
        rig.world->set_control("throttle", 0.2);
        rig.world->set_control("hand_throttle", hand);
        Trace tr;
        for (int t = 0; t < 480; ++t) {
            rig.world->step();
            const auto dbg = rig.world->powertrain_debug(rig.vehicle);
            tr.hash.push_back(rig.world->state_hash());
            tr.eff.push_back(dbg.throttle_eff);
            tr.demand.push_back(dbg.engine_torque_demand);
            tr.integrator.push_back(dbg.idle_integrator);
            if (rig.world->powertrain_state(rig.vehicle).hand_throttle != 0.0) ++tr.snapshot_hand_nonzero;
        }
        tr.channel_values = rig.world->vehicle_component_channel_values(rig.vehicle);
        return tr;
    };
    const Trace base = run(0.0);
    CHECK(base.snapshot_hand_nonzero == 0);
    for (const real hand : {0.5, 1.0}) {
        INFO("hand_throttle " << hand);
        const Trace tr = run(hand);
        REQUIRE(tr.channel_values.size() == 1);
        CHECK(tr.channel_values[0] == hand); // it did reach the component channel buffer
        CHECK(tr.hash == base.hash);
        CHECK(tr.eff == base.eff);
        CHECK(tr.demand == base.demand);
        CHECK(tr.integrator == base.integrator);
        CHECK(tr.snapshot_hand_nonzero == 0);
    }
}

// ===========================================================================
// PHYS-033 P4 I7a C5b (owner ruling 2026-09-26): auto-shift/auto-clutch/
// auto-blip must read the throttle-command stage's REQUEST (ThrottleStage::
// evaluate's ThrottleCommand::command - pedal_map/hand-lever already
// applied), not the raw pedal. The two cases below are the task's required
// discrimination tests: each is set up so the OLD (raw-pedal) and NEW
// (command) values feed a materially different number into the exact same
// TCU formula at an otherwise-identical, steady operating point - so a
// revert of the C5b reorder (Sensors::throttle_request fed ctx.throttle
// again instead of the throttle stage's command) makes the assertions below
// FAIL, not just report a different number. See this commit's handback
// report for the actual sabotage run and its numbers.
// ===========================================================================

TEST_CASE("C5b: a pedal_map changes the auto-shift upshift threshold (assist reads the command, not the raw pedal)",
          "[unit][drivetrain][throttle][tcu]") {
    // An aggressive early-ramp pedal_map: a pedal of 0.2 (raw) reshapes to a
    // command of ~0.667 - the two are far enough apart that
    // lerp(up_zero, up_full, ·) gives two thresholds with a wide gap.
    const vehicle::VehicleDesc d = sedan_with_pedal_map({0.0, 0.3, 1.0}, {0.0, 1.0, 1.0});
    const ThrottleControlDesc* stage = stage_desc_of(d);
    REQUIRE(stage != nullptr);
    const auto& tcu_desc = std::get<ManualTcuDesc>(d.controllers.at(0).params);
    const AutoShiftDesc& as = tcu_desc.auto_shift;

    constexpr real kPedal = 0.2;
    const real command = stage->curve.evaluate(kPedal);
    REQUIRE(command > kPedal + 0.3); // the map genuinely reshapes: a raw-pedal leak would show as command==pedal

    const real up_th_pedal = as.up_rpm_at_zero_throttle + kPedal * (as.up_rpm_at_full_throttle - as.up_rpm_at_zero_throttle);
    const real up_th_command = as.up_rpm_at_zero_throttle + command * (as.up_rpm_at_full_throttle - as.up_rpm_at_zero_throttle);
    REQUIRE(up_th_command > up_th_pedal + 1000.0); // the two thresholds are well separated

    // kHub tuned (gear 1: rpm ~= kHub * 137.27, see reset_running_in_gear's
    // own "kHub=18.0 -> ~2470 rpm" comment/P3's test) so the held engine rpm
    // sits strictly between up_th_pedal and up_th_command - the discriminating
    // operating point: correct (command-based) code must NOT shift here,
    // sabotaged (raw-pedal) code WOULD.
    constexpr real kHub = 29.15;
    Powertrain pt = build_pt(d);
    reset_running_in_gear(pt, 1, kHub);
    DynoDriver drv(pt, kHub);
    drv.ctx.throttle = kPedal;
    drv.ctx.assist_auto_shift = true;

    // Run well past auto_shift.min_time_in_gear_s (1.0 s for car_sedan) so the
    // dwell-time gate cannot be the reason no shift happens.
    REQUIRE(as.min_time_in_gear_s < 2.0);
    for (int s = 0; s < 3 * 960; ++s) drv.step(); // 3 s

    const real rpm = pt.last_engine_result().rpm;
    INFO("engine rpm=" << rpm << " up_th(raw pedal)=" << up_th_pedal << " up_th(command)=" << up_th_command
                        << " gearbox target=" << pt.gearbox().target());
    REQUIRE(rpm > up_th_pedal);   // held ABOVE the (wrong) raw-pedal threshold ...
    REQUIRE(rpm < up_th_command); // ... but BELOW the (correct) command threshold - a genuine discriminator
    CHECK(pt.gearbox().current_position() == 1);
    CHECK(pt.gearbox().target() == 1); // correct code: no upshift requested at this rpm
}

TEST_CASE("C5b: the tractor's hand_throttle reaches the auto-clutch launch law (not the raw pedal)",
          "[unit][drivetrain][throttle][tcu]") {
    const vehicle::VehicleDesc d = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
    REQUIRE(engine_desc_of(d).has_governor);
    REQUIRE(stage_desc_of(d) == nullptr); // no explicit throttle_control component - the implicit stage's hand
                                           // lever is the governor's own "hand_throttle" channel (throttle_stage.h)
    const auto& tcu_desc = std::get<ManualTcuDesc>(d.controllers.at(0).params);
    const AutoClutchDesc& ac = tcu_desc.auto_clutch;
    REQUIRE(ac.enabled);

    // hand_throttle = 0.2: the diesel governor's no-load setpoint is
    // idle_rpm + hand*(high_idle_rpm - idle_rpm) = 850 + 0.2*(2400-850) = 1160
    // rpm (data/engines/diesel_i4_4l5_turbo.json), which sits strictly BELOW
    // BOTH auto_clutch full-engagement thresholds:
    //   full(raw pedal=0)          = full_rpm_min                = 1200
    //   full(command=hand_throttle)= full_rpm_min + 0.2*(1700-1200) = 1300
    // so e_tgt = clamp((rpm-bite_rpm)/(full-bite_rpm), 0, 1) is in the
    // strictly-linear (non-clamped) region for BOTH, giving two cleanly
    // different fractional-engagement targets - not merely two different
    // sides of a clamp.
    constexpr real kHand = 0.2;
    const real full_old = ac.full_rpm_min;                                      // raw pedal (0) - the bug
    const real full_new = ac.full_rpm_min + kHand * (ac.full_rpm_max - ac.full_rpm_min); // stage command - the fix

    Powertrain pt = build_pt(d);
    const auto decls = Powertrain::declare_control_channels(d.powertrain);
    std::vector<real> channels(decls.size(), real(0.0));
    int hand_slot = -1;
    for (std::size_t i = 0; i < decls.size(); ++i) {
        channels[i] = decls[i].default_value;
        if (decls[i].name == "hand_throttle") hand_slot = static_cast<int>(i);
    }
    REQUIRE(hand_slot >= 0);
    channels[static_cast<std::size_t>(hand_slot)] = kHand;

    reset_launch_in_gear(pt, 1, /*hub_omega=*/0.0); // dog already in 1st, clutch open (Launch, e_auto=0), wheel held at rest
    DynoDriver drv(pt, /*target=*/0.0);
    drv.ctx.assist_auto_clutch = true; // DynoDriver defaults this off (see its own comment); the Launch law under
                                        // test lives entirely behind this flag - ManualTcu::step's out.e_auto is
                                        // hard-forced to 1.0 whenever it is false, which would trivially "pass" by
                                        // locking the clutch on substep 1 whatever the rpm/full-rpm math says.
    // DynoDriver's default k=5000 is a stiffness applied AT THE HUB NODE; the
    // rigid gear-1 x rear-diff reduction between gearbox.in and the rear hub
    // is large (>>1:1), and a node-local damping/stiffness term reflects into
    // the gearbox.in-referred reduced coordinate scaled by the SQUARE of the
    // (small) hub/gearbox.in speed ratio - k=5000 turned out to be utterly
    // negligible there once reflected: even a tiny (< 0.2) clutch engagement
    // spun gearbox_in up to within lock_slip_rpm of the engine in well under
    // 0.15 s (measured), tripping Launch -> Locked long before the rpm-gated
    // law could be observed settled. A much stiffer dyno is needed so the
    // held hub - and, through the rigid mesh, gearbox_in - actually stays
    // near 0 while the clutch is only partially engaged.
    drv.k = 1.0e9;
    drv.ctx.throttle = 0.0; // pedal fully released - only the hand lever drives this
    drv.ctx.channels = channels.data();
    drv.ctx.channel_count = static_cast<int>(channels.size());

    // Let the governor settle the engine onto its no-load setpoint and let
    // e_auto (rate-limited at engage_rate_per_s) converge onto e_tgt.
    for (int s = 0; s < 4 * 960; ++s) {
        drv.step();
    }

    REQUIRE(pt.tcu_clutch_mode() == ManualTcu::ClutchMode::Launch); // still slipping in, not locked/aborted
    const real rpm = pt.last_engine_result().rpm;
    // Honest PRECONDITIONS only - these establish the strictly-linear region
    // for both candidate "full" values (see comment above) and do not
    // themselves examine ManualTcu's output. The rpm > bite_rpm + 30 margin
    // keeps us safely clear of the bite-point clamp corner, where a small
    // rpm difference between the correct and sabotaged runs (the auto-clutch
    // engagement feeds back into engine load under the governor's droop -
    // see the sabotage-demo numbers in this test's own history) could
    // otherwise push one run's e_tgt to 0 and mask the discriminating gap.
    REQUIRE(rpm > ac.bite_rpm + 30);
    REQUIRE(rpm < full_old);

    const real e_tgt_old = std::clamp((rpm - ac.bite_rpm) / (full_old - ac.bite_rpm), real(0.0), real(1.0));
    const real e_tgt_new = std::clamp((rpm - ac.bite_rpm) / (full_new - ac.bite_rpm), real(0.0), real(1.0));
    INFO("engine rpm=" << rpm << " e_auto=" << pt.clutch_engagement() << " e_tgt(raw pedal)=" << e_tgt_old
                        << " e_tgt(command)=" << e_tgt_new);
    // The actual discriminators: ManualTcu's own output, not an arithmetic
    // stand-in for it. Both are REQUIREs (not CHECKs) so a sabotaged build
    // (TCU fed ctx.throttle instead of the throttle-command stage's request)
    // fails HERE, not on an unrelated precondition above.
    REQUIRE(pt.clutch_engagement() == Catch::Approx(e_tgt_new).margin(0.05));
    REQUIRE(pt.clutch_engagement() < e_tgt_old - 0.05); // would fail if ManualTcu had instead used the raw pedal (0)
}
