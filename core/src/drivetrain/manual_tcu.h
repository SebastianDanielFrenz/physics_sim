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

#include "ps/types.h"

#include <cstdint>

namespace ps::drivetrain {

struct AutoClutchDesc {
    bool enabled = true;
    real bite_rpm = 1100.0;      // engagement 0 at or below this engine speed (launch law)
    real full_rpm_min = 1600.0;  // engagement 1 at this rpm with throttle 0 ...
    real full_rpm_max = 3000.0;  // ... and at this rpm with throttle 1
    real lock_slip_rpm = 80.0;   // |engine - input| below this (and rpm > bite) -> Locked
    real anti_stall_rpm = 950.0; // Locked and rpm below this -> back to the launch law
    real engage_rate_per_s = 4.0, release_rate_per_s = 12.0;
    real open_time_s = 0.08, close_time_s = 0.20; // shift ramps
};
struct AutoBlipDesc {
    bool enabled = true;
    real kp_per_rpm = 0.002;
    real max_throttle = 1.0;
    bool upshift_cut = true;
};
struct AutoShiftDesc {
    bool enabled = false;
    real up_rpm_at_zero_throttle = 2500.0, up_rpm_at_full_throttle = 6300.0;
    real down_rpm_at_zero_throttle = 1300.0, down_rpm_at_full_throttle = 2800.0;
    real min_time_in_gear_s = 1.0;
};

class ManualTcu {
public:
    struct Desc {
        AutoClutchDesc auto_clutch;
        AutoBlipDesc auto_blip;
        AutoShiftDesc auto_shift;
        int max_pending_steps = 2;
    };

    enum class ClutchMode : std::uint8_t { Launch, Locked, Shift };

    // By value, one substep's worth of live state (plan section 6.5's
    // "controllers get sensors only"; gear_request/counters are TICK-
    // granularity but harmlessly re-read every substep - see step()'s
    // first_substep_of_tick parameter, which gates the arbitration itself).
    struct Sensors {
        real engine_omega = 0.0, gearbox_in_omega = 0.0, gearbox_out_omega = 0.0;
        real throttle_pedal = 0.0;
        real gear_request = real(-100.0); // compare against ps::drivetrain::kGearRequestNone
        real shift_up_count = 0.0, shift_down_count = 0.0;
        bool assist_auto_clutch = true, assist_auto_blip = true, assist_auto_shift = false;
        int gearbox_current = 0, gearbox_target = 0;
        Gearbox::Phase gearbox_phase = Gearbox::Phase::Idle;
        real synchro_target_ratio = 0.0; // Gearbox::StepResult::synchro_target_ratio, 0 if not Synchronising
        int min_gear = -1, max_gear = 1; // this vehicle's gearbox bounds (-m..n)
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

private:
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
};

} // namespace ps::drivetrain
