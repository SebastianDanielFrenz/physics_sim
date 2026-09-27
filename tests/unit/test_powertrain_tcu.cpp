// tests/unit/test_powertrain_tcu.cpp — white-box tests for
// ps::drivetrain::ManualTcu (PHYS-033 P4 I3 plan section 13.7). core/src is
// on this target's include path, like the other white-box drivetrain
// tests - ManualTcu is internal, not wired into Powertrain yet (commit 6).

#include "drivetrain/manual_tcu.h"
#include "ps/drivetrain/powertrain_desc.h" // kGearRequestNone

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

using namespace ps;
using namespace ps::drivetrain;

namespace {
constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);
real rpm_to_omega(real rpm) { return rpm / kRadPerSecToRpm; }

ManualTcu::Sensors base_sensors() {
    ManualTcu::Sensors s;
    s.gear_request = kGearRequestNone;
    s.min_gear = -1;
    s.max_gear = 6;
    s.gearbox_current = 0;
    s.gearbox_target = 0;
    s.gearbox_phase = Gearbox::Phase::Idle;
    s.assist_auto_clutch = true;
    s.assist_auto_blip = true;
    s.assist_auto_shift = false;
    return s;
}
} // namespace

TEST_CASE("ManualTcu: counter differences, first-tick baseline, reset, and clamped pending", "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    d.max_pending_steps = 2;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    SECTION("first tick establishes a baseline, no action even with a nonzero starting count") {
        ManualTcu::Sensors s = base_sensors();
        s.shift_up_count = 5.0; // e.g. the adapter seeded from a nonzero persisted counter
        const auto out = tcu.step(s, h, /*first_substep_of_tick*/ true);
        CHECK(out.gearbox_target == 0); // no shift on the very first observation
    }

    SECTION("a single press (delta 1) shifts up by one gear once, from Idle") {
        ManualTcu::Sensors s = base_sensors();
        s.shift_up_count = 0.0;
        (void)tcu.step(s, h, true); // baseline
        s.shift_up_count = 1.0;
        const auto out = tcu.step(s, h, true);
        CHECK(out.gearbox_target == 1);
    }

    SECTION("a counter RESET (new count < last) re-baselines instead of producing a huge negative delta") {
        ManualTcu::Sensors s = base_sensors();
        s.shift_up_count = 100.0;
        (void)tcu.step(s, h, true); // baseline at 100
        s.shift_up_count = 0.0;     // reset (e.g. adapter restart)
        const auto out = tcu.step(s, h, true);
        CHECK(out.gearbox_target == 0); // no shift: d_up = max(0, 0-100) = 0, re-baselines to 0
        s.shift_up_count = 1.0;
        const auto out2 = tcu.step(s, h, true);
        CHECK(out2.gearbox_target == 1); // now counts normally from the new baseline
    }

    SECTION("3 presses in one tick gives pending = +2 (clamped at max_pending_steps)") {
        ManualTcu::Sensors s = base_sensors();
        (void)tcu.step(s, h, true); // baseline at 0
        s.shift_up_count = 3.0;     // 3 presses landed between ticks
        const auto out = tcu.step(s, h, true);
        // One of the 3 is consumed immediately (Idle, pending != 0 -> shift by
        // sgn(pending) and pending -= sgn(pending)); clamp caps pending at
        // max_pending_steps(2) BEFORE that consumption.
        CHECK(out.gearbox_target == 1);
        CHECK(tcu.pending() == 1); // 2 (clamped) - 1 (consumed) = 1
    }
}

TEST_CASE("ManualTcu: gear_request precedence over counters, and the 'none' sentinel", "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    ManualTcu::Sensors s = base_sensors();
    s.shift_up_count = 5.0; // would otherwise clamp-and-walk
    s.gear_request = 4.0;   // absolute request wins outright
    const auto out = tcu.step(s, h, true);
    CHECK(out.gearbox_target == 4);
    CHECK(tcu.pending() == 0); // an absolute request clears any pending counter walk

    // Sentinel: out of range counts as "none" and falls back to counters.
    ManualTcu::Sensors s2 = base_sensors();
    s2.gear_request = kGearRequestNone;
    (void)tcu.step(s2, h, true); // re-baseline counters at their current (0) value first
    s2.shift_up_count = 1.0;
    const auto out2 = tcu.step(s2, h, true);
    CHECK(out2.gearbox_target != 4); // no longer latched at the old absolute request
}

TEST_CASE("ManualTcu: auto-clutch launch law gives e(rpm), then Launch -> Locked -> Launch", "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    d.auto_clutch.bite_rpm = 1100.0;
    d.auto_clutch.full_rpm_min = 1600.0;
    d.auto_clutch.full_rpm_max = 3000.0;
    d.auto_clutch.lock_slip_rpm = 80.0;
    d.auto_clutch.anti_stall_rpm = 950.0;
    d.auto_clutch.engage_rate_per_s = 4.0;
    d.auto_clutch.release_rate_per_s = 12.0;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    ManualTcu::Sensors s = base_sensors();
    s.gearbox_current = 1;
    s.gearbox_target = 1;
    s.throttle_request = 0.0; // full(0) == full_rpm_min == 1600
    s.engine_omega = rpm_to_omega(1100.0);
    s.gearbox_in_omega = 0.0; // large slip: input shaft not yet moving

    // At bite exactly, e_tgt = 0; ramps toward it (release_rate_per_s).
    (void)tcu.step(s, h, true);
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Launch);

    // Halfway to full: e_tgt should trend toward 0.5. Run enough substeps to
    // approach the target at this fixed rpm (rate-limited approach).
    s.engine_omega = rpm_to_omega(1350.0); // (1350-1100)/(1600-1100) = 0.5
    for (int i = 0; i < 2000; ++i) (void)tcu.step(s, h, false);
    CHECK(tcu.e_auto() == Catch::Approx(0.5).margin(0.02));

    // Now close the slip and push rpm above bite: Launch -> Locked.
    s.engine_omega = rpm_to_omega(1200.0);
    s.gearbox_in_omega = rpm_to_omega(1200.0 - 30.0); // |slip| = 30 rpm < lock_slip_rpm(80)
    // Full ramp at engage_rate_per_s(4/s) from ~0.5 to 1.0 needs ~0.125s = 120 substeps.
    for (int i = 0; i < 300; ++i) (void)tcu.step(s, h, false);
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Locked);
    CHECK(tcu.e_auto() == Catch::Approx(1.0).margin(1e-6));

    // Braking to a stop in gear: rpm falls below anti_stall_rpm -> back to Launch.
    s.engine_omega = rpm_to_omega(700.0);
    s.gearbox_in_omega = rpm_to_omega(700.0);
    (void)tcu.step(s, h, false);
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Launch);
}

// PHYS-033 P4 I3 (C8 smoke-test finding): hard braking in gear at moderate
// speed dropped engine rpm from well above anti_stall_rpm to a full stall in
// well under a second, even though the anti-stall transition (Locked ->
// Launch once rpm < anti_stall_rpm) fired immediately - AutoClutchDesc's own
// release_rate_per_s (tuned for an ordinary Launch-law release, ~0.083 s to
// fully open) still leaves enough clutch CAPACITY for several ticks to keep
// dragging a light engine (~0.16 kg*m^2 crank+flywheel) down at the same
// gear-ratio-amplified rate as the decelerating wheel, right through
// stall_rpm, before capacity finally drops low enough to let it slip free.
// AutoClutchDesc::anti_stall_release_rate_per_s (its own comment has the
// full mechanism) is a faster floor applied ONLY while releasing below
// anti_stall_rpm - this pins that it actually takes effect and opens the
// clutch materially faster than release_rate_per_s alone would.
TEST_CASE("ManualTcu: anti-stall release uses the faster panic rate, not the ordinary release_rate_per_s",
          "[unit][drivetrain][tcu]") {
    auto build = [](real anti_stall_release_rate_per_s) {
        ManualTcu::Desc d;
        d.auto_clutch.bite_rpm = 1100.0;
        d.auto_clutch.full_rpm_min = 1600.0;
        d.auto_clutch.full_rpm_max = 3000.0;
        d.auto_clutch.lock_slip_rpm = 80.0;
        d.auto_clutch.anti_stall_rpm = 950.0;
        d.auto_clutch.engage_rate_per_s = 4.0;
        d.auto_clutch.release_rate_per_s = 12.0; // ordinary release: ~0.083s to fully open
        d.auto_clutch.anti_stall_release_rate_per_s = anti_stall_release_rate_per_s;
        return d;
    };
    const real h = 1.0 / 960.0;

    // Drive both variants to Locked at full engagement first (same setup as
    // the launch-law test above), then trigger anti-stall the same way.
    auto lock_up = [&](ManualTcu& tcu) {
        ManualTcu::Sensors s = base_sensors();
        s.gearbox_current = 1;
        s.gearbox_target = 1;
        s.throttle_request = 0.0;
        s.engine_omega = rpm_to_omega(1200.0);
        s.gearbox_in_omega = rpm_to_omega(1200.0 - 30.0); // small slip -> locks quickly
        for (int i = 0; i < 400; ++i) (void)tcu.step(s, h, false);
        REQUIRE(tcu.clutch_mode() == ManualTcu::ClutchMode::Locked);
        REQUIRE(tcu.e_auto() == Catch::Approx(1.0).margin(1e-6));
        return s;
    };

    // Variant A: anti_stall_release_rate_per_s == release_rate_per_s (no
    // panic boost - isolates the ordinary rate's own behaviour).
    ManualTcu tcu_ordinary(build(/*anti_stall_release_rate_per_s=*/12.0));
    ManualTcu::Sensors sa = lock_up(tcu_ordinary);
    sa.engine_omega = rpm_to_omega(700.0); // below anti_stall_rpm(950): triggers the transition
    sa.gearbox_in_omega = rpm_to_omega(700.0);
    (void)tcu_ordinary.step(sa, h, false);
    CHECK(tcu_ordinary.clutch_mode() == ManualTcu::ClutchMode::Launch);
    // One substep at release_rate_per_s(12/s): e_auto drops by 12*h.
    CHECK(tcu_ordinary.e_auto() == Catch::Approx(1.0 - 12.0 * h).margin(1e-6));

    // Variant B: this file's shipped-style panic rate (40/s), well above
    // release_rate_per_s - must open FASTER than variant A on the very same
    // first substep after the anti-stall trigger.
    ManualTcu tcu_panic(build(/*anti_stall_release_rate_per_s=*/40.0));
    ManualTcu::Sensors sb = lock_up(tcu_panic);
    sb.engine_omega = rpm_to_omega(700.0);
    sb.gearbox_in_omega = rpm_to_omega(700.0);
    (void)tcu_panic.step(sb, h, false);
    CHECK(tcu_panic.clutch_mode() == ManualTcu::ClutchMode::Launch);
    CHECK(tcu_panic.e_auto() == Catch::Approx(1.0 - 40.0 * h).margin(1e-6));
    CHECK(tcu_panic.e_auto() < tcu_ordinary.e_auto()); // strictly faster than the ordinary rate

    // Fully open well within the panic rate's own ~0.025s budget (24
    // substeps at 960 Hz), unlike the ordinary rate's ~0.083s (80 substeps).
    for (int i = 0; i < 24; ++i) (void)tcu_panic.step(sb, h, false);
    CHECK(tcu_panic.e_auto() == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("ManualTcu: after an anti-stall release the clutch re-closes while still rolling fast enough",
          "[unit][drivetrain][tcu]") {
    // Owner test drive (2026-09-24): "after braking hard, rpm is stuck at
    // idle" - the Launch law alone kept the clutch open while coasting in
    // gear off the throttle. AutoClutchDesc::rolling_reengage_rpm.
    ManualTcu::Desc d;
    d.auto_clutch.bite_rpm = 1100.0;
    d.auto_clutch.full_rpm_min = 1600.0;
    d.auto_clutch.full_rpm_max = 3000.0;
    d.auto_clutch.lock_slip_rpm = 80.0;
    d.auto_clutch.anti_stall_rpm = 950.0;
    d.auto_clutch.engage_rate_per_s = 4.0;
    d.auto_clutch.release_rate_per_s = 12.0;
    d.auto_clutch.rolling_reengage_rpm = 1300.0;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    ManualTcu::Sensors s = base_sensors();
    s.gearbox_current = 2;
    s.gearbox_target = 2;
    s.throttle_request = 0.0;
    s.engine_omega = rpm_to_omega(1200.0);
    s.gearbox_in_omega = rpm_to_omega(1170.0);
    for (int i = 0; i < 400; ++i) (void)tcu.step(s, h, false);
    REQUIRE(tcu.clutch_mode() == ManualTcu::ClutchMode::Locked);

    // Hard braking drags the engine below anti_stall_rpm: the clutch opens.
    s.engine_omega = rpm_to_omega(900.0);
    s.gearbox_in_omega = rpm_to_omega(900.0);
    for (int i = 0; i < 200; ++i) (void)tcu.step(s, h, false);
    REQUIRE(tcu.clutch_mode() == ManualTcu::ClutchMode::Launch);
    REQUIRE(tcu.e_auto() == Catch::Approx(0.0).margin(1e-6));

    // Engine back at idle, input shaft below rolling_reengage_rpm: stays open
    // (closing now could stall the engine).
    s.engine_omega = rpm_to_omega(800.0);
    s.gearbox_in_omega = rpm_to_omega(1200.0);
    for (int i = 0; i < 200; ++i) (void)tcu.step(s, h, false);
    CHECK(tcu.e_auto() == Catch::Approx(0.0).margin(1e-6));

    // Still rolling with the input shaft above rolling_reengage_rpm, driver
    // off the throttle: the clutch closes (engage_rate 4/s -> 0.25 s).
    s.gearbox_in_omega = rpm_to_omega(1800.0);
    for (int i = 0; i < 300; ++i) (void)tcu.step(s, h, false);
    CHECK(tcu.e_auto() == Catch::Approx(1.0).margin(1e-6));
    // Once the engine is dragged up to the input shaft, it locks.
    s.engine_omega = rpm_to_omega(1790.0);
    (void)tcu.step(s, h, false);
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Locked);
}

TEST_CASE("ManualTcu: blip and ignition-cut flags follow the shift phase", "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    d.auto_blip.enabled = true;
    d.auto_blip.upshift_cut = true;
    d.auto_blip.kp_per_rpm = 0.002;
    d.auto_blip.max_throttle = 1.0;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    SECTION("upshift: ignition_cut is set while the gearbox is mid-shift, cleared once re-idle") {
        ManualTcu::Sensors s = base_sensors();
        s.gearbox_current = 1;
        s.gearbox_target = 2; // already commanded before this call (as if requested last tick)
        s.gearbox_phase = Gearbox::Phase::Selecting;
        const auto out = tcu.step(s, h, true);
        CHECK(out.ignition_cut);

        s.gearbox_phase = Gearbox::Phase::Idle;
        s.gearbox_current = 2; // dog engaged
        const auto out2 = tcu.step(s, h, false);
        CHECK_FALSE(out2.ignition_cut);
    }

    SECTION("downshift: throttle_floor blips toward the synchro target speed while Synchronising") {
        ManualTcu::Sensors s = base_sensors();
        s.gearbox_current = 3;
        s.gearbox_target = 2;
        s.gearbox_phase = Gearbox::Phase::Synchronising;
        s.synchro_target_ratio = 1.6; // 2nd gear's ratio
        s.gearbox_out_omega = 100.0;  // -> target input speed 160 rad/s
        s.engine_omega = 120.0;       // below target: throttle should open
        const auto out = tcu.step(s, h, true);
        CHECK(out.throttle_floor > real(0.0));
        CHECK_FALSE(out.ignition_cut); // downshifts do not cut ignition
    }

    SECTION("no shift in progress: neither flag fires") {
        ManualTcu::Sensors s = base_sensors();
        s.gearbox_current = 3;
        s.gearbox_target = 3;
        s.gearbox_phase = Gearbox::Phase::Idle;
        const auto out = tcu.step(s, h, true);
        CHECK(out.throttle_floor == real(0.0));
        CHECK_FALSE(out.ignition_cut);
    }
}

// Speed-matched upshift (owner test drive 2026-09-25: "I still see the
// speed ticking throughout the entire shift"): with the auto-clutch on, the
// upshift cut outlives the gearbox phase until the engine is within
// upshift_cut_release_slip_rpm of the new gear's input shaft (or
// upshift_cut_max_hold_s after dog-in), then the pedal comes back through a
// ceiling ramped over upshift_torque_ramp_s.
TEST_CASE("ManualTcu: the upshift cut is held through the post-shift close until the engine is speed-matched",
          "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    d.auto_blip.upshift_cut = true;
    d.auto_blip.upshift_cut_release_slip_rpm = 300.0;
    d.auto_blip.upshift_cut_max_hold_s = 0.5;
    d.auto_blip.upshift_torque_ramp_s = 0.1;
    d.engine_inertia = 0.16; // torque-following close (as wired by Powertrain)
    d.clutch_capacity_nm = 300.0;
    const real h = 1.0 / 960.0;

    // 2nd at WOT, 1->2 already commanded, gearbox mid-shift.
    auto mid_shift = [] {
        ManualTcu::Sensors s = base_sensors();
        s.throttle_request = 1.0;
        s.gearbox_current = 1;
        s.gearbox_target = 2;
        s.gearbox_phase = Gearbox::Phase::Selecting;
        s.engine_omega = rpm_to_omega(6000.0);
        s.gearbox_in_omega = rpm_to_omega(6000.0);
        return s;
    };
    // Dog in (Idle in 2nd), engine flaring above the 2nd-gear input shaft.
    auto dog_in = [](ManualTcu::Sensors s, real engine_rpm) {
        s.gearbox_current = 2;
        s.gearbox_target = 2;
        s.gearbox_phase = Gearbox::Phase::Idle;
        s.engine_omega = rpm_to_omega(engine_rpm);
        s.gearbox_in_omega = rpm_to_omega(3500.0);
        return s;
    };

    SECTION("held while the flare exceeds the release slip, released below it, then the ceiling ramps 0 -> 1") {
        ManualTcu tcu(d);
        auto out = tcu.step(mid_shift(), h, true);
        CHECK(out.ignition_cut);
        CHECK(out.throttle_ceiling == real(0.0));
        out = tcu.step(dog_in(mid_shift(), 5000.0), h, false);
        CHECK(out.ignition_cut); // 1500 rpm flare: still cut
        out = tcu.step(dog_in(mid_shift(), 3900.0), h, false);
        CHECK(out.ignition_cut); // 400 rpm
        out = tcu.step(dog_in(mid_shift(), 3750.0), h, false);
        CHECK_FALSE(out.ignition_cut); // 250 rpm < 300: released
        CHECK(out.throttle_ceiling == Catch::Approx(h / 0.1));
        // Never re-arms within the same shift, even if the engine flares again.
        out = tcu.step(dog_in(mid_shift(), 4500.0), h, false);
        CHECK_FALSE(out.ignition_cut);
        for (int i = 0; i < 200; ++i) out = tcu.step(dog_in(mid_shift(), 3600.0), h, false);
        CHECK(out.throttle_ceiling == real(1.0));
    }

    SECTION("a close that never synchronises releases the cut after upshift_cut_max_hold_s") {
        ManualTcu tcu(d);
        (void)tcu.step(mid_shift(), h, true);
        ManualTcu::Outputs out;
        int n_cut = 0;
        for (int i = 0; i < 960; ++i) {
            out = tcu.step(dog_in(mid_shift(), 5000.0), h, false);
            if (out.ignition_cut) ++n_cut;
        }
        CHECK_FALSE(out.ignition_cut);
        CHECK(n_cut >= 479); // 0.5 s at 960 Hz
        CHECK(n_cut <= 481);
    }

    SECTION("manual clutch (assist.auto_clutch off): the cut still ends with the gearbox phase") {
        ManualTcu tcu(d);
        ManualTcu::Sensors s = mid_shift();
        s.assist_auto_clutch = false;
        CHECK(tcu.step(s, h, true).ignition_cut);
        ManualTcu::Sensors s2 = dog_in(s, 5000.0);
        const auto out = tcu.step(s2, h, false);
        CHECK_FALSE(out.ignition_cut);
        CHECK(out.throttle_ceiling < real(0.05)); // the ramp still applies
    }

    SECTION("assist.auto_blip off: no cut and no ceiling reach the engine") {
        ManualTcu tcu(d);
        ManualTcu::Sensors s = mid_shift();
        s.assist_auto_blip = false;
        const auto out = tcu.step(s, h, true);
        CHECK_FALSE(out.ignition_cut);
        CHECK(out.throttle_ceiling == real(1.0));
    }
}

// Owner ruling (PHYS-033 P4 I3, post-C6): a Neutral -> gear engagement at
// low speed is a LAUNCH, not a mid-drive shift - it must follow the
// rpm-gated Launch law, not the fast forced Shift-mode ClutchClosing
// reclose (which locks the clutch in close_time_s regardless of engine rpm
// and produced a ~1.7 peak slip wheelspin on every drive-off, measured in
// test_vehicle_accel_standing_start.cpp before this fix).
TEST_CASE("ManualTcu: launching from Neutral at low speed uses the Launch law, not a forced Shift reclose",
          "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    d.auto_clutch.bite_rpm = 1100.0;
    d.auto_clutch.full_rpm_min = 1600.0;
    d.auto_clutch.full_rpm_max = 3000.0;
    d.auto_clutch.lock_slip_rpm = 80.0;
    d.auto_clutch.anti_stall_rpm = 950.0;
    d.auto_clutch.engage_rate_per_s = 4.0;
    d.auto_clutch.release_rate_per_s = 12.0;
    // Deliberately fast: if launching from Neutral wrongly went through
    // Shift mode, e_auto would snap most of the way to 1 within this test's
    // own ~0.1 s fast-forward below, rpm be damned - that is exactly the
    // failure this test guards against.
    d.auto_clutch.open_time_s = 0.08;
    d.auto_clutch.close_time_s = 0.20;
    d.auto_clutch.launch_speed_threshold_mps = 1.0;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    ManualTcu::Sensors s = base_sensors();
    s.gearbox_current = 0;
    s.gearbox_target = 0;
    s.gearbox_phase = Gearbox::Phase::Idle;
    s.approx_speed_mps = 0.2; // near standstill
    s.engine_omega = rpm_to_omega(800.0); // idle, below bite_rpm(1100)
    (void)tcu.step(s, h, true); // baseline at rest: clutch closed in N (Launch law's own N rule)
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Launch);
    CHECK(tcu.e_auto() == Catch::Approx(1.0).margin(1e-3));

    // Request 1st gear at near-standstill.
    s.gear_request = 1.0;
    const auto out = tcu.step(s, h, true);
    CHECK(out.gearbox_target == 1);
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Launch); // NOT Shift

    // The gearbox itself now chases the new target (Powertrain's own
    // set_target/phase_step, outside the TCU) - simulate it having started
    // (phase no longer Idle, current still 0 until dog-in) while rpm stays
    // at idle (well below bite_rpm): the Launch law's e_tgt is 0 here, so
    // e_auto must RELEASE toward 0, not snap toward 1 via a forced reclose.
    s.gear_request = kGearRequestNone;
    s.gearbox_phase = Gearbox::Phase::Synchronising;
    for (int i = 0; i < 100; ++i) (void)tcu.step(s, h, false); // ~0.1s - well along a 0.2s forced reclose, if it fired
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Launch);
    CHECK(tcu.e_auto() < real(0.5));

    // Dog engages (current -> 1, phase -> Idle), rpm rises with throttle:
    // the Launch law now ramps e_auto up as rpm climbs past bite, exactly
    // as it would after any OTHER Launch-mode engagement.
    s.gearbox_current = 1;
    s.gearbox_target = 1;
    s.gearbox_phase = Gearbox::Phase::Idle;
    s.engine_omega = rpm_to_omega(1350.0); // (1350-1100)/(1600-1100) = 0.5
    s.gearbox_in_omega = 0.0;              // large slip: input shaft not yet synced, stays Launch (not Locked)
    for (int i = 0; i < 2000; ++i) (void)tcu.step(s, h, false);
    CHECK(tcu.e_auto() == Catch::Approx(0.5).margin(0.02));
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Launch);
}

// Coordinator/haptics report 2026-09-25: N->1 requested together with full
// throttle. The engine flares past bite_rpm while the gearbox is still
// Selecting/Synchronising; the clutch must stay OPEN until the dog is in
// (the synchro cannot match speeds against a loaded input shaft), then the
// ordinary Launch law takes over.
TEST_CASE("ManualTcu: launching from Neutral holds the clutch open while the gearbox engages, whatever the rpm",
          "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    d.auto_clutch.bite_rpm = 1100.0;
    d.auto_clutch.full_rpm_min = 1600.0;
    d.auto_clutch.full_rpm_max = 3000.0;
    d.auto_clutch.lock_slip_rpm = 80.0;
    d.auto_clutch.anti_stall_rpm = 750.0;
    d.auto_clutch.engage_rate_per_s = 4.0;
    d.auto_clutch.release_rate_per_s = 12.0;
    d.auto_clutch.open_time_s = 0.08;
    d.auto_clutch.launch_speed_threshold_mps = 1.0;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    ManualTcu::Sensors s = base_sensors();
    s.approx_speed_mps = 0.0;
    s.engine_omega = rpm_to_omega(800.0);
    (void)tcu.step(s, h, true);
    REQUIRE(tcu.e_auto() == Catch::Approx(1.0).margin(1e-3)); // closed in N

    s.gear_request = 1.0;
    s.throttle_request = 1.0;
    (void)tcu.step(s, h, true);
    s.gear_request = kGearRequestNone;
    s.gearbox_target = 1;
    s.engine_omega = rpm_to_omega(4000.0); // flared well past full_rpm_max: the Launch law alone would say e = 1
    for (Gearbox::Phase ph : {Gearbox::Phase::Selecting, Gearbox::Phase::Synchronising}) {
        s.gearbox_phase = ph;
        for (int i = 0; i < 96; ++i) (void)tcu.step(s, h, false); // 0.1 s each
        CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Launch);
        CHECK(tcu.e_auto() == Catch::Approx(0.0).margin(1e-9)); // fully open within open_time_s
    }

    // Dog in: the Launch law closes on rpm again.
    s.gearbox_current = 1;
    s.gearbox_phase = Gearbox::Phase::Idle;
    s.gearbox_in_omega = 0.0;
    for (int i = 0; i < 96; ++i) (void)tcu.step(s, h, false);
    CHECK(tcu.e_auto() == Catch::Approx(0.4).margin(0.01)); // engage_rate_per_s 4 x 0.1 s
}

TEST_CASE("ManualTcu: launching from Neutral ABOVE the speed threshold still uses the ordinary Shift-mode reclose",
          "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    d.auto_clutch.launch_speed_threshold_mps = 1.0;
    d.auto_clutch.open_time_s = 0.08;
    d.auto_clutch.close_time_s = 0.20;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    ManualTcu::Sensors s = base_sensors();
    s.gearbox_current = 0;
    s.gearbox_target = 0;
    s.gearbox_phase = Gearbox::Phase::Idle;
    s.approx_speed_mps = 5.0; // well above the launch threshold - e.g. coasting in N, then selecting a gear
    s.engine_omega = rpm_to_omega(800.0);
    (void)tcu.step(s, h, true); // baseline

    s.gear_request = 1.0;
    const auto out = tcu.step(s, h, true);
    CHECK(out.gearbox_target == 1);
    CHECK(tcu.clutch_mode() == ManualTcu::ClutchMode::Shift); // unchanged behaviour above the threshold
}

TEST_CASE("ManualTcu: auto-shift thresholds and hysteresis", "[unit][drivetrain][tcu]") {
    ManualTcu::Desc d;
    d.auto_shift.enabled = true;
    d.auto_shift.up_rpm_at_zero_throttle = 2500.0;
    d.auto_shift.up_rpm_at_full_throttle = 6300.0;
    d.auto_shift.down_rpm_at_zero_throttle = 1300.0;
    d.auto_shift.down_rpm_at_full_throttle = 2800.0;
    d.auto_shift.min_time_in_gear_s = 1.0;
    ManualTcu tcu(d);
    const real h = 1.0 / 960.0;

    ManualTcu::Sensors s = base_sensors();
    s.assist_auto_shift = true;
    s.gearbox_current = 1;
    s.gearbox_target = 1;
    s.throttle_request = 0.0;
    s.engine_omega = rpm_to_omega(2000.0); // below up(0)=2500: no upshift yet
    s.gearbox_out_omega = rpm_to_omega(1000.0);

    // Not enough time in gear yet: no shift even if rpm is high.
    s.engine_omega = rpm_to_omega(3000.0);
    auto out = tcu.step(s, h, true);
    CHECK(out.gearbox_target == 1);

    // Advance past min_time_in_gear_s.
    for (int i = 0; i < 1000; ++i) (void)tcu.step(s, h, false); // ~1.04s at h=1/960
    out = tcu.step(s, h, true);
    CHECK(out.gearbox_target == 2); // rpm(3000) > up(0)=2500

    // Simulate the shift completing (gearbox now reports gear 2), then
    // check the downshift hysteresis at a mid rpm that is below down(0)
    // (1300) - should downshift back.
    s.gearbox_current = 2;
    s.gearbox_target = 2;
    s.engine_omega = rpm_to_omega(1200.0);
    for (int i = 0; i < 1000; ++i) (void)tcu.step(s, h, false);
    out = tcu.step(s, h, true);
    CHECK(out.gearbox_target == 1);
}
