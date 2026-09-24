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
    s.throttle_pedal = 0.0; // full(0) == full_rpm_min == 1600
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
    s.throttle_pedal = 0.0;
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
