// tests/unit/test_powertrain_gearbox.cpp — white-box tests for
// ps::drivetrain::Gearbox and ::Clutch (PHYS-033 P4 I3 plan section 13.7).
// core/src is on this target's include path, like the other white-box
// drivetrain tests - neither class is part of ps_core's public API, and
// neither is wired into Powertrain yet (commit 6).
//
// The gearbox's phase machine is driven here with a SYNTHETIC omega1/omega2
// trajectory (not a real ShaftNetwork) - phase_step only ever reads
// omega1/omega2/lambda_mesh_prev/h, so this exercises its actual logic
// exactly, without needing a full network solve (which commit 1's own
// topology-switch test already covers at the network level: a state_index
// change from this class, fed into ShaftNetwork::set_topology_state, is
// EXACTLY that momentum-conserving projection - the plan's own section
// 5.3 design, confirmed there).

#include "drivetrain/clutch.h"
#include "drivetrain/gearbox.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

using namespace ps;
using namespace ps::drivetrain;

namespace {

// A small manual box: 3 forward, 1 reverse - r_fd folded into the ratios
// directly here (this class tests the gearbox alone, not the diff).
Gearbox::Desc make_manual_desc() {
    Gearbox::Desc d;
    d.type = GearboxKind::Manual;
    d.forward_ratios = {3.0, 1.6, 1.0};
    d.forward_efficiency = {0.96, 0.97, 0.98};
    d.reverse_ratios = {-3.2};
    d.reverse_efficiency = {0.95};
    d.input_inertia_kgm2 = 0.014;
    d.output_inertia_kgm2 = 0.02;
    d.synchro.forward_capacity_nm = {80.0, 70.0, 60.0};
    d.synchro.reverse_capacity_nm = {0.0}; // no synchro on reverse (dog only)
    d.synchro.static_factor = 1.0;
    d.synchro.max_dog_mismatch_rad_s = 5.0;
    d.neutral_phase_s = 0.06;
    d.dog_release_torque_nm = 60.0;
    d.grind_after_s = 0.25;
    d.abort_after_s = 1.0;
    d.initial_gear = 0;
    return d;
}

} // namespace

TEST_CASE("Gearbox: position <-> state-index mapping matches the sorted R_m..R_1,N,F_1..F_n order",
          "[unit][drivetrain][gearbox]") {
    Gearbox gb(make_manual_desc()); // 1 reverse, 3 forward -> 5 states total
    CHECK(gb.state_index_for_position(-1) == 0); // R1
    CHECK(gb.state_index_for_position(0) == 1);  // N
    CHECK(gb.state_index_for_position(1) == 2);  // F1
    CHECK(gb.state_index_for_position(2) == 3);  // F2
    CHECK(gb.state_index_for_position(3) == 4);  // F3

    CHECK(gb.ratio_for_position(1) == Catch::Approx(3.0));
    CHECK(gb.ratio_for_position(-1) == Catch::Approx(-3.2));
    CHECK(gb.efficiency_for_position(2) == Catch::Approx(0.97));
    CHECK(gb.synchro_capacity_kinetic_for_position(3) == Catch::Approx(60.0));
    CHECK(gb.synchro_capacity_kinetic_for_position(-1) == Catch::Approx(0.0)); // no reverse synchro
}

TEST_CASE("Gearbox: a full N->1st shift cycle (Selecting then Synchronising then engaged)", "[unit][drivetrain][gearbox]") {
    Gearbox gb(make_manual_desc());
    const real h = 1.0 / 960.0;
    REQUIRE(gb.phase() == Gearbox::Phase::Idle);
    REQUIRE(gb.current_position() == 0);

    gb.set_target(1);
    // First substep: Idle -> Selecting (current == 0 already, no Releasing needed).
    Gearbox::StepInput in{0.0, 0.0, 0.0, h};
    auto r = gb.phase_step(in);
    CHECK(r.phase == Gearbox::Phase::Selecting);
    CHECK(r.state_index == gb.state_index_for_position(0)); // still N until the dog is in

    // Hold omega1 == omega2 (mismatch 0, would engage instantly once synchronising).
    int selecting_substeps = 0;
    while (gb.phase() == Gearbox::Phase::Selecting) {
        r = gb.phase_step(in);
        ++selecting_substeps;
        REQUIRE(selecting_substeps < 1000);
    }
    // neutral_phase_s (0.06s) / h (1/960s) = 57.6 -> 57 or 58 substeps.
    CHECK(selecting_substeps == static_cast<int>(std::lround(gb.desc().neutral_phase_s / h)));
    CHECK(r.phase == Gearbox::Phase::Synchronising);

    r = gb.phase_step(in); // omega1==omega2==0, r_target*0==0, delta==0 <= max_mismatch: engages immediately
    CHECK(r.phase == Gearbox::Phase::Idle);
    CHECK(gb.current_position() == 1);
    CHECK(r.state_index == gb.state_index_for_position(1));
}

TEST_CASE("Gearbox: dog release is refused while loaded, then releases once torque drops", "[unit][drivetrain][gearbox]") {
    Gearbox gb(make_manual_desc());
    const real h = 1.0 / 960.0;
    // Start engaged in 1st (bypassing the shift-in sequence: initial_gear).
    Gearbox::Desc d = make_manual_desc();
    d.initial_gear = 1;
    Gearbox gb2(d);
    gb2.set_target(0); // driver wants N
    Gearbox::StepInput in{0.0, 0.0, /*lambda_mesh_prev*/ 80.0, h}; // above dog_release_torque_nm(60): refused
    auto r = gb2.phase_step(in);
    CHECK(r.phase == Gearbox::Phase::Releasing);
    CHECK(gb2.current_position() == 1); // still engaged

    for (int i = 0; i < 50; ++i) {
        r = gb2.phase_step(in); // still loaded
        CHECK(r.phase == Gearbox::Phase::Releasing);
        CHECK(gb2.current_position() == 1);
    }

    in.lambda_mesh_prev = 40.0; // below the threshold now
    r = gb2.phase_step(in);
    CHECK(r.phase == Gearbox::Phase::Selecting);
    CHECK(gb2.current_position() == 0); // dog out
}

TEST_CASE("Gearbox: synchro time matches J_in*|domega|/capacity within 2 substeps", "[unit][drivetrain][gearbox]") {
    // A synthetic CLOSING trajectory (not a real network): omega1 held so
    // that the mismatch shrinks linearly at a rate consistent with a
    // constant synchro torque c acting on J_in alone (omega2 held fixed at
    // some nonzero speed representing the loaded output shaft) - exactly
    // the plan's own section 6.3 "synchro timing" estimate, t = J1*|Domega|/c.
    Gearbox::Desc d = make_manual_desc();
    Gearbox gb(d);
    const real h = 1.0 / 960.0;
    const real omega2 = 70.0; // rad/s, representative
    const real r1 = d.forward_ratios[0];
    const real c = d.synchro.forward_capacity_nm[0];
    const real J_in = d.input_inertia_kgm2;

    gb.set_target(1);
    Gearbox::StepInput in{0.0, omega2, 0.0, h};
    (void)gb.phase_step(in); // -> Selecting
    while (gb.phase() == Gearbox::Phase::Selecting) (void)gb.phase_step(in);
    REQUIRE(gb.phase() == Gearbox::Phase::Synchronising);

    // Start the closing trajectory: initial mismatch large enough that
    // several substeps are needed, closing at the rate a constant torque c
    // would give a J_in-alone rotor: |domega|(t) = domega0 - (c/J_in)*t.
    const real domega0 = real(300.0); // rad/s at the input shaft, well above max_dog_mismatch
    const real closing_rate = c / J_in; // rad/s^2
    const real t_expected = domega0 / closing_rate; // reaches 0 mismatch here (plan's own t = J*Domega/c)
    const int substeps_expected = static_cast<int>(std::lround(t_expected / h));

    int t_count = 0;
    Gearbox::StepResult r;
    while (gb.phase() == Gearbox::Phase::Synchronising && t_count < substeps_expected + 100) {
        const real domega_now = std::max(real(0.0), domega0 - closing_rate * static_cast<real>(t_count) * h);
        in.omega1 = r1 * omega2 + domega_now;
        in.omega2 = omega2;
        r = gb.phase_step(in);
        ++t_count;
    }
    REQUIRE(gb.phase() == Gearbox::Phase::Idle);
    REQUIRE(gb.current_position() == 1);
    CHECK(std::abs(t_count - substeps_expected) <= 2);
}

TEST_CASE("Gearbox: engaging reverse while rolling grinds then aborts to N, never dogs in past the mismatch",
          "[unit][drivetrain][gearbox]") {
    Gearbox::Desc d = make_manual_desc(); // reverse has synchro capacity 0 -> dog only, per the header comment
    Gearbox gb(d);
    const real h = 1.0 / 960.0;
    gb.set_target(-1);
    Gearbox::StepInput in{0.0, real(20.0), 0.0, h}; // rolling forward at omega2 = 20 rad/s: a huge, permanent mismatch for R1
    (void)gb.phase_step(in); // -> Selecting
    while (gb.phase() == Gearbox::Phase::Selecting) (void)gb.phase_step(in);
    REQUIRE(gb.phase() == Gearbox::Phase::Synchronising);

    bool grind_seen = false;
    bool aborted = false;
    std::uint32_t events_at_grind = 0;
    for (int i = 0; i < 2000; ++i) {
        const auto r = gb.phase_step(in);
        // NEVER dogs in with the mismatch this large (the whole point of the test).
        REQUIRE(gb.current_position() == 0);
        if (r.grind_active && !grind_seen) {
            grind_seen = true;
            events_at_grind = gb.grind_events();
            CHECK(std::abs(static_cast<real>(i + 1) * h - d.grind_after_s) <= 2.0 * h);
        }
        if (r.just_refused) {
            aborted = true;
            CHECK(std::abs(static_cast<real>(i + 1) * h - d.abort_after_s) <= 2.0 * h);
            CHECK(gb.grind_events() == events_at_grind); // no new grind between grind_after_s and abort_after_s
            break;
        }
    }
    CHECK(grind_seen);
    CHECK(aborted);
    CHECK(gb.grind_events() == 1);
    CHECK(gb.phase() == Gearbox::Phase::Idle);
    CHECK(gb.target() == 0); // aborted request is cleared to N
}

TEST_CASE("Gearbox: Sequential walks one position at a time toward a far target", "[unit][drivetrain][gearbox]") {
    Gearbox::Desc d = make_manual_desc();
    d.type = GearboxKind::Sequential;
    Gearbox gb(d);
    const real h = 1.0 / 960.0;
    Gearbox::StepInput in{0.0, 0.0, 0.0, h}; // omega1==omega2==0 throughout: every synchronising step engages instantly

    gb.set_target(3); // N -> 3rd directly requested, but must walk 1 -> 2 -> 3
    std::vector<int> current_trace;
    for (int i = 0; i < 2000 && gb.current_position() != 3; ++i) {
        (void)gb.phase_step(in);
        if (current_trace.empty() || current_trace.back() != gb.current_position()) current_trace.push_back(gb.current_position());
    }
    REQUIRE(gb.current_position() == 3);
    // Must have visited every intermediate ENGAGED gear in order - 1, 2, 3 -
    // never jumping straight to 3; each shift also passes back through N
    // (0) as a transient waypoint (dog-out before the next dog-in), which
    // this filters out to check only the engaged sequence.
    std::vector<int> engaged_trace;
    for (int v : current_trace) {
        if (v != 0) engaged_trace.push_back(v);
    }
    REQUIRE(engaged_trace.size() == 3);
    CHECK(engaged_trace[0] == 1);
    CHECK(engaged_trace[1] == 2);
    CHECK(engaged_trace[2] == 3);
}

// --- Clutch (plan section 6.2) ---------------------------------------------

TEST_CASE("Clutch: engagement is min(pedal curve, auto), capacities scale from it", "[unit][drivetrain][clutch]") {
    Clutch::Desc d;
    d.capacity_nm = 300.0;
    d.static_factor = 1.15;
    d.engagement_vs_pedal.x = {0.0, 0.5, 1.0};
    d.engagement_vs_pedal.y = {1.0, 0.5, 0.0};
    Clutch c(d);

    const auto r1 = c.pre_solve(/*pedal*/ 0.0, /*e_auto*/ 1.0);
    CHECK(r1.engagement == Catch::Approx(1.0));
    CHECK(r1.cap_kinetic == Catch::Approx(300.0));
    CHECK(r1.cap_static == Catch::Approx(345.0));

    const auto r2 = c.pre_solve(0.0, /*e_auto*/ 0.3); // auto-clutch overrides down
    CHECK(r2.engagement == Catch::Approx(0.3));
    CHECK(r2.cap_kinetic == Catch::Approx(90.0));

    const auto r3 = c.pre_solve(1.0, /*e_auto*/ 1.0); // pedal fully pressed: released regardless of auto
    CHECK(r3.engagement == Catch::Approx(0.0));
    CHECK(r3.cap_kinetic == Catch::Approx(0.0));
    CHECK(r3.cap_static == Catch::Approx(0.0));
}
