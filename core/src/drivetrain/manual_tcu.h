// core/src/drivetrain/manual_tcu.h — the I3 manual transmission control
// unit (PHYS-033 P4 I3 plan section 6.5): request arbitration (counters +
// absolute gear_request), auto-clutch, auto-blip/upshift-cut, and an
// optional auto-shift driver aid. A new class, framework-agnostic like
// engine_torque_map.h/clutch.h/gearbox.h - not yet wired into Powertrain
// (commit 6). Deliberately does not read Gearbox/Clutch directly (spec's
// "controllers get sensors only"): callers translate live state into
// Sensors and apply Outputs to the gearbox/clutch/engine themselves.

#pragma once

#include "gearbox.h" // Gearbox::Phase only

#include "ps/drivetrain/controller_desc.h" // AutoClutchDesc/AutoBlipDesc/AutoShiftDesc (public, plan section 4.2)
#include "ps/types.h"

#include <cstdint>
#include <limits>

namespace ps::drivetrain {

class ManualTcu {
public:
    struct Desc {
        AutoClutchDesc auto_clutch;
        AutoBlipDesc auto_blip;
        AutoShiftDesc auto_shift;
        int max_pending_steps = 2;
        // Filled by Powertrain from the network (not from the loader): the
        // engine-side clutch node's inertia (kg*m^2) and the clutch's
        // kinetic capacity at engagement 1 (Nm). The torque-following close
        // (AutoClutchDesc::sync_time_s) needs both; if either is <= 0 (a
        // bare ManualTcu, e.g. a unit test) the TCU falls back to the
        // legacy fixed-rate close (e_auto -> 1 at 1/close_time_s).
        real engine_inertia = 0.0;
        real clutch_capacity_nm = 0.0;
    };

    enum class ClutchMode : std::uint8_t { Launch, Locked, Shift };

    // By value, one substep's worth of live state (plan section 6.5's
    // "controllers get sensors only"; gear_request/counters are TICK-
    // granularity but harmlessly re-read every substep - see step()'s
    // first_substep_of_tick parameter, which gates the arbitration itself).
    struct Sensors {
        real engine_omega = 0.0, gearbox_in_omega = 0.0, gearbox_out_omega = 0.0;
        real approx_speed_mps = 0.0; // vehicle road speed proxy (gearbox_out_omega/|r_fd|*R_mean) - launch-vs-shift detection
        // PHYS-033 P4 I7a C5b (owner ruling 2026-09-26): the throttle-command
        // stage's TORQUE REQUEST (throttle_stage.h's ThrottleCommand::command
        // - pedal_map/hand-lever already applied), NOT the raw pedal - so a
        // vehicle with an electronic pedal_map or a hand throttle (the
        // tractor's governor) has its launch law / auto-shift thresholds /
        // auto-blip act on the same signal the engine itself sees. Was
        // `throttle_pedal` (raw ctx.throttle) before this commit; for a
        // vehicle with no pedal_map and no hand lever the stage's command
        // equals the raw pedal exactly, so this is a no-op for those (every
        // shipped sedan).
        real throttle_request = 0.0;
        real gear_request = real(-100.0); // compare against ps::drivetrain::kGearRequestNone
        real shift_up_count = 0.0, shift_down_count = 0.0;
        bool assist_auto_clutch = true, assist_auto_blip = true, assist_auto_shift = false;
        int gearbox_current = 0, gearbox_target = 0;
        Gearbox::Phase gearbox_phase = Gearbox::Phase::Idle;
        real synchro_target_ratio = 0.0; // Gearbox::StepResult::synchro_target_ratio, 0 if not Synchronising
        int min_gear = -1, max_gear = 1; // this vehicle's gearbox bounds (-m..n)
        // Net crank torque the engine's throttle demand asks for, excluding
        // the clutch and ignoring the rev limiter (EngineTorqueMap::
        // PreSolveResult::torque_demand; Nm, positive = driving), from the
        // engine's PREVIOUS substep (one substep lagged, like every other
        // engine<->TCU exchange - plan section 7.2).
        real engine_torque_nm = 0.0;
        // Assists I2 (PHYS-074 D-LS2): the vehicle's ground-speed reference (m/s, signed along the chassis forward
        // axis; NaN = none - only set while traction control's effective mode is not Off) and the factor that turns
        // it into the gearbox-input omega of the CURRENT gear at no tyre slip (ratio_gear * speedo ratio / speedo
        // radius; 0 in neutral). Below anti_stall_rpm the Launch law does not take the Locked transition: a lock
        // there would be onto spinning wheels. Unhashed inputs.
        real ground_speed_mps = std::numeric_limits<real>::quiet_NaN();
        real ground_speed_to_input_omega = 0.0;
        // Assists I2 (PHYS-074 D-LS1): traction control acting through the auto-clutch at crawl speed - a cap on the
        // clutch's kinetic capacity (N*m) applied to the auto-clutch output e_auto (+inf = none). The TCU's own
        // e_auto state is not changed by it.
        real clutch_capacity_cap_nm = std::numeric_limits<real>::infinity();
        // Assists I7: the effective drive mode's overrides of AutoShiftDesc's four rpm points and of AutoClutchDesc::
        // close_time_s (NaN = the car's own value; a NaN compares false and is never used as a threshold).
        real up_rpm_at_zero_throttle = std::numeric_limits<real>::quiet_NaN(), up_rpm_at_full_throttle = std::numeric_limits<real>::quiet_NaN();
        real down_rpm_at_zero_throttle = std::numeric_limits<real>::quiet_NaN(), down_rpm_at_full_throttle = std::numeric_limits<real>::quiet_NaN();
        real close_time_s = std::numeric_limits<real>::quiet_NaN();
    };

    struct Outputs {
        int gearbox_target = 0;   // feed into Gearbox::set_target (unchanged from Sensors::gearbox_target if no new request)
        real e_auto = 1.0;        // feed into Clutch::pre_solve's e_auto
        real throttle_floor = 0.0;
        bool ignition_cut = false;
        // Cap on the driver's pedal (the auto-blip floor is not capped),
        // ramped 0 -> 1 over AutoBlipDesc::upshift_torque_ramp_s after an
        // upshift cut ends; 1 otherwise.
        real throttle_ceiling = 1.0;
    };

    explicit ManualTcu(Desc desc) : desc_(desc) {}

    // first_substep_of_tick: request arbitration (gear_request/counters) is
    // consumed only once per TICK (plan section 6.5: "Counters and requests
    // are consumed once per tick, at the first substep") - every other
    // substep reuses the tick's already-decided gearbox_target unchanged.
    [[nodiscard]] Outputs step(const Sensors& s, real h, bool first_substep_of_tick);

    [[nodiscard]] ClutchMode clutch_mode() const { return mode_; }
    [[nodiscard]] const Desc& desc() const { return desc_; }
    [[nodiscard]] real e_auto() const { return e_auto_; }
    [[nodiscard]] int pending() const { return pending_; }

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    // World::reset_vehicle (plan section 4.5): puts the TCU in Idle (no
    // pending shift/request; re-baselines the counter arbitration so a
    // stale shift_up_count/shift_down_count delta is not replayed on the
    // next step) and sets the auto-clutch mode to Locked (clutch_locked)
    // or Launch, matching VehicleResetOptions::clutch_locked. e_auto is set
    // to match (1 when Locked, 0 when Launch - the caller has already set
    // every network node's omega directly, so there is no slip transient
    // for the Launch law to chase on the very next step).
    void reset(bool clutch_locked) {
        seeded_ = false;
        last_up_ = 0;
        last_dn_ = 0;
        pending_ = 0;
        last_target_seen_valid_ = false;
        time_in_gear_ = real(0.0);
        upshift_cut_armed_ = false;
        upshift_cut_hold_t_ = real(0.0);
        throttle_ceiling_ = real(1.0);
        mode_ = clutch_locked ? ClutchMode::Locked : ClutchMode::Launch;
        e_auto_ = clutch_locked ? real(1.0) : real(0.0);
        reverse_hold_ = false;
        anti_stall_released_ = false;
    }

private:
    // The torque-following close's engagement target (AutoClutchDesc::
    // sync_time_s), in [0, 1]. Pure function of the sensors.
    [[nodiscard]] real sync_close_target(const Sensors& s) const;
    [[nodiscard]] bool torque_following_close() const {
        return desc_.engine_inertia > real(0.0) && desc_.clutch_capacity_nm > real(0.0);
    }

    Desc desc_;

    // Request arbitration state (plan section 6.5.1).
    bool seeded_ = false;
    std::int64_t last_up_ = 0, last_dn_ = 0;
    int pending_ = 0;
    int last_target_seen_ = 0;
    bool last_target_seen_valid_ = false;

    // Auto-clutch state (plan section 6.5.2).
    ClutchMode mode_ = ClutchMode::Launch;
    real e_auto_ = 1.0;
    // The gearbox input turns backwards in a forward gear: the clutch is held
    // open. Latched on a clear reversal (below -anti_stall_rpm, or an
    // emergency release with the input already below -lock_slip_rpm) and
    // released once the input is above -lock_slip_rpm again, so short
    // negative transients (suspension settling, locked-diff wind-up) do not
    // dump the clutch. Applies only without driver throttle at hill-start pace
    // (throttle_request above kDriverThrottleThreshold and road speed below
    // kReverseLaunchMaxSpeedMps, manual_tcu.cpp, clear it): with the throttle
    // pressed - a hill start rolling backwards in a forward gear - the
    // ordinary Launch law engages the clutch gradually instead.
    bool reverse_hold_ = false;
    // The anti-stall last released a locked clutch and it has not locked
    // again since: from this state the clutch may not start closing onto a
    // backwards-turning input at all (latches reverse_hold_ at any input
    // below -lock_slip_rpm), unlike a launch from rest, whose input can dip
    // backwards briefly while 1st goes in.
    bool anti_stall_released_ = false;

    // Auto-shift state (plan section 6.5.4).
    real time_in_gear_ = 0.0;

    // The shift in progress started as an upshift (6.5.3's upshift cut).
    bool upshift_cut_armed_ = false;
    // Time since the dog went in while the cut is held through the
    // post-shift close (AutoBlipDesc::upshift_cut_max_hold_s).
    real upshift_cut_hold_t_ = 0.0;
    // Pedal ceiling after an upshift cut (Outputs::throttle_ceiling).
    real throttle_ceiling_ = 1.0;
};

} // namespace ps::drivetrain
