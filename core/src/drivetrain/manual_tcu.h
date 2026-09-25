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
        real throttle_pedal = 0.0;
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
    };

    struct Outputs {
        int gearbox_target = 0;   // feed into Gearbox::set_target (unchanged from Sensors::gearbox_target if no new request)
        real e_auto = 1.0;        // feed into Clutch::pre_solve's e_auto
        real throttle_floor = 0.0;
        bool ignition_cut = false;
    };

    explicit ManualTcu(Desc desc) : desc_(desc) {}

    // first_substep_of_tick: request arbitration (gear_request/counters) is
    // consumed only once per TICK (plan section 6.5: "Counters and requests
    // are consumed once per tick, at the first substep") - every other
    // substep reuses the tick's already-decided gearbox_target unchanged.
    [[nodiscard]] Outputs step(const Sensors& s, real h, bool first_substep_of_tick);

    [[nodiscard]] ClutchMode clutch_mode() const { return mode_; }
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
        mode_ = clutch_locked ? ClutchMode::Locked : ClutchMode::Launch;
        e_auto_ = clutch_locked ? real(1.0) : real(0.0);
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

    // Auto-shift state (plan section 6.5.4).
    real time_in_gear_ = 0.0;

    // The shift in progress started as an upshift (6.5.3's upshift cut).
    bool upshift_cut_armed_ = false;
};

} // namespace ps::drivetrain
