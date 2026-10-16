// ps/drivetrain/controller_desc.h — public, additive descriptors for
// PHYS-033 P4 I3's per-vehicle controllers (plan section 4.2). Pure data;
// no algorithm code (that lives in core/src/drivetrain/manual_tcu.h/.cpp,
// internal - same D2/D10 pattern as powertrain_desc.h). I5 extends
// ControllerParams's variant and adds default_enabled/rate_divider fields
// to these structs (not needed until then).

#pragma once

#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <optional>
#include <string>
#include <variant>

namespace ps::drivetrain {

struct AutoClutchDesc {
    bool enabled = true; // default of the runtime assist.* channel (World::create_vehicle); the channel is the switch
    // Engine garage P8 (A15): hardware fitment. false = the car does not have this assist: its channel is still
    // registered (stable channel list, default 0) but ignored - the assist stays off whatever the channel says.
    bool fitted = true;
    real bite_rpm = 1100.0;      // engagement 0 at or below this engine speed (launch law)
    real full_rpm_min = 1600.0;  // engagement 1 at this rpm with throttle 0 ...
    real full_rpm_max = 3000.0;  // ... and at this rpm with throttle 1
    real lock_slip_rpm = 80.0;   // |engine - input| below this (and rpm > bite) -> Locked
    real anti_stall_rpm = 950.0; // signed crank/coupled RPM below this: release a locked clutch
    real engage_rate_per_s = 4.0, release_rate_per_s = 12.0;
    // PHYS-033 P4 I3 (C8 smoke-test finding): while releasing BELOW
    // anti_stall_rpm specifically, the clutch opens at at least this rate
    // instead of the ordinary release_rate_per_s. Found via the M1 smoke
    // test's scripted hard-brake-in-gear phase (data/vehicles/car_sedan.json,
    // 2nd gear, full brake from ~20 km/h): at release_rate_per_s alone, the
    // clutch's kinetic capacity (curve capacity_nm * e_auto) stays well above
    // the small torque needed to keep a light crank+flywheel (~0.16 kg*m^2)
    // decelerating in lockstep with the gear-ratio-amplified wheel deceleration
    // for most of the release ramp - by the time capacity finally drops low
    // enough to let the engine slip free, hard braking has already dragged it
    // from anti_stall_rpm past stall_rpm. A real automated-clutch anti-stall
    // system dumps line pressure near-instantly (a fast solenoid, not the
    // smooth modulated release used for an ordinary gear shift's ClutchClosing
    // or Launch-law bite ramp) specifically because it exists to catch this
    // same "still connected while decelerating hard" case - this field is
    // that same distinction. Applies ONLY below anti_stall_rpm (the ordinary,
    // smoother release_rate_per_s still governs the bite_rpm..anti_stall_rpm
    // band and every other release), so normal launch/shift feel is
    // unaffected. Default is intentionally aggressive (~0.025 s to fully
    // open) relative to release_rate_per_s's ~0.083 s.
    real anti_stall_release_rate_per_s = 40.0;
    // Rolling re-engagement (owner test drive 2026-09-24: "after braking
    // hard, rpm is stuck at idle"): in Launch mode, in gear with the gearbox
    // idle, the clutch closes once the gearbox INPUT shaft turns faster than
    // this (rpm, SIGNED - a backwards-turning input never qualifies). Also
    // the post-shift launch-vs-shift decision (owner test drive 2026-09-25):
    // below it the post-shift engagement is a launch, not a shift reclose.
    // Must be > anti_stall_rpm. ca118ec (2026-10-04) briefly replaced it with
    // anti_stall_rpm; that closed every launch from rest at ~6 km/h onto an
    // engine near idle (failing the unsettled-start, compliant-launch and
    // split-mu validation tests), so it governs again. The release side
    // (Locked below anti_stall_rpm on engine OR signed input rpm, instant
    // release, reverse-rotation hold) stays as ca118ec made it.
    real rolling_reengage_rpm = 1300.0;
    real open_time_s = 0.08, close_time_s = 0.20; // shift ramps
    // Torque-following close onto a MOVING driveline (owner test drive
    // 2026-09-25, "a lot of wheel spin in 2nd"): after an upshift's dog is
    // in, and for the Launch law's rolling re-engagement, the clutch no
    // longer ramps to full capacity at a fixed rate whatever the engine
    // does (that dumped up to capacity_nm - 300 Nm on car_sedan.json, 1.5x
    // the engine's own peak - plus the flywheel's slip energy into the
    // driven wheels). Instead the clutch's kinetic capacity follows
    //   T = max(0, sign(slip) * T_engine + min(J_engine * |slip| / sync_time_s, max_sync_torque_nm))
    // so it carries the engine's own torque plus a bounded synchronising
    // torque that pulls the engine onto the input shaft's speed with time
    // constant sync_time_s; once |slip| < lock_slip_rpm the clutch locks
    // (Locked mode, engage_rate_per_s up to full capacity, as before). Both
    // must be > 0 (vehicle_io.cpp validates). manual_tcu.cpp has the full
    // derivation and the rejected alternatives.
    real sync_time_s = 0.30;
    real max_sync_torque_nm = 50.0;
    // Owner ruling (PHYS-033 P4 I3, post-C6): a Neutral -> gear engagement
    // with the vehicle at or below this speed is a LAUNCH, not a mid-drive
    // shift - it uses the rpm-gated Launch law (bite_rpm/full_rpm_*) instead
    // of the fast Shift-mode ClutchClosing reclose (open_time_s/
    // close_time_s), which otherwise locks the clutch in a few tenths of a
    // second regardless of engine rpm and produces a hard, driver-unintended
    // wheelspin on every drive-off (manual_tcu.cpp's own comment has the
    // full mechanism). Manual clutch (auto_clutch off) is unaffected - the
    // driver's own pedal input decides.
    real launch_speed_threshold_mps = 1.0;
};

struct AutoBlipDesc {
    bool enabled = true; // default of the runtime assist.* channel (World::create_vehicle); the channel is the switch
    bool fitted = true;  // engine garage P8 (A15): see AutoClutchDesc::fitted
    real kp_per_rpm = 0.002;
    real max_throttle = 1.0;
    bool upshift_cut = true;
    // Speed-matched upshift (owner test drive 2026-09-25; manual_tcu.cpp has
    // the mechanism). With the auto-clutch on, the upshift cut is held
    // through the post-shift clutch close until the engine is within
    // upshift_cut_release_slip_rpm of the new gear's input shaft (engine
    // faster), so the car coasts through the shift instead of being pushed
    // by a flaring engine through the slipping clutch; at most
    // upshift_cut_max_hold_s after the dog is in (a stuck close never holds
    // the cut forever). Afterwards the pedal comes back through a ceiling
    // ramped 0 -> 1 over upshift_torque_ramp_s (0 = step). vehicle_io.cpp
    // validates: release slip and max hold > 0, ramp >= 0.
    real upshift_cut_release_slip_rpm = 300.0;
    real upshift_cut_max_hold_s = 0.6;
    real upshift_torque_ramp_s = 0.10;
};

struct AutoShiftDesc {
    bool enabled = false; // default of the runtime assist.* channel (World::create_vehicle); the channel is the switch
    bool fitted = true;   // engine garage P8 (A15): see AutoClutchDesc::fitted
    real up_rpm_at_zero_throttle = 2500.0, up_rpm_at_full_throttle = 6300.0;
    real down_rpm_at_zero_throttle = 1300.0, down_rpm_at_full_throttle = 2800.0;
    real min_time_in_gear_s = 1.0;
    // Assists I7 (owner 2026-10-13): kickdown bypass. A downshift request (engine rpm below the downshift point) made
    // while the throttle COMMAND (the throttle stage's output: car pedal curve and drive-mode bend applied) is at or
    // above this value ignores min_time_in_gear_s. 0 = no bypass (every car that does not set it is bit-identical).
    // [AS] 0.95 on car_hyper_n2o.
    real kickdown_throttle = 0.0;
    // G1 (ps/drivetrain/limiter_range.h shift_point_scale): when the higher upshift point (the car's own or a drive mode's
    // override) lies above this fraction of the engine's effective limiter (effective_limiter_rpm: a low garage limiter, a
    // soft cut's taper, a governor or an armed roll-off), all four points are scaled down by one factor. Key
    // "max_limiter_fraction" in the auto_shift block, in (0, 1]. [AS] 0.985 (kDefaultShiftLimiterFraction).
    real max_limiter_fraction = 0.985;
};

// The loader-facing descriptor for a "manual_tcu" controller (plan section
// 9.1): `gearbox`/`clutch`/`engine` name the PowertrainDesc component ids
// this controller drives - validated (existence + type) by the loader, not
// by ManualTcu itself (core/src/drivetrain/manual_tcu.h's ManualTcu::Desc
// has no notion of component ids at all - it operates purely on Sensors).
struct ManualTcuDesc {
    std::string gearbox, clutch, engine;
    AutoClutchDesc auto_clutch;
    AutoBlipDesc auto_blip;
    AutoShiftDesc auto_shift;
    int max_pending_steps = 2;
};

// Engine garage P8 (A16, docs/data_formats/electronics.md): launch control (two-step). A second rev limit
// `rpm` that acts through the engine's own limiter (fuel cut or soft cut per the engine's limiter type) while it
// is active. Active from the moment the channel is on, the car is (nearly) stationary (speedo below
// max_speed_mps) and the clutch is effectively in (engagement < kClutchInEngagement), until the channel goes
// off or the speed exceeds max_speed_mps; the latch is what lets the car launch with the limit still held.
// Hashed state only while the component exists.
struct LaunchControlDesc {
    real rpm = 0.0;                                   // second limiter speed; > engine idle, < the engine's own limiter
    real max_speed_mps = 2.0;                         // active only below this speedo speed; releases above it
    std::string channel = "assist.launch_control";   // runtime arm switch (plain 0/1 level)
    bool fitted = true;                               // false: the channel is ignored (A15)
    // G1 (ps/drivetrain/limiter_range.h fit_launch_rpm): an rpm at or above this fraction of the engine's limiter (a low
    // garage limiter) is clamped to it with a load warning instead of being a load error. Part key "max_limiter_fraction",
    // in (0, 1). [AS] 0.8 (kDefaultLaunchLimiterFraction).
    real max_limiter_fraction = 0.8;
};

// Automatic gearbox plan A1: the shift schedule of an automatic_tcu, in gearbox INPUT rpm (the turbine of an
// Automatic, the engine of a DualClutch) so it moves with the engine, not with the final drive or the tyre size.
// Upshift above lerp(up_zero, up_full, throttle), downshift below lerp(down_zero, down_full, throttle).
struct ShiftScheduleDesc {
    real up_rpm_at_zero_throttle = 2000.0, up_rpm_at_full_throttle = 6000.0;
    real down_rpm_at_zero_throttle = 1100.0, down_rpm_at_full_throttle = 3200.0;
};

// Automatic gearbox plan A1: the TCU of an Automatic or DualClutch gearbox.
// Selector through the existing gear_request channel (kGearRequestNone = no change): 0 = N, any negative value = R,
// a positive value k = D with gears up to min(k, forward gears) (so a large value is plain D, 1..n hold the box at or
// below that gear like an L/2/3 position). Park is N plus the park_lock component (channel park_lock), as before.
// shift_up_count / shift_down_count in D step the box one gear (manual mode, "tiptronic"/paddles); manual mode ends
// after manual_timeout_s without a press (0 = never) or when gear_request changes. The channel
// kTransmissionModeChannel (declared with default 0 = D by every Automatic/DualClutch gearbox; 1 = S) selects the sport
// schedule. Driver demand = the throttle stage's command (pedal map, drive mode applied).
inline constexpr const char* kTransmissionModeChannel = "transmission.mode";

// CVT plan C1: the ratio law of an automatic_tcu driving a Cvt gearbox ("cvt" block, required there, rejected
// elsewhere). Continuous mode: the variator aims at the input rpm (turbine of a converter CVT, engine otherwise)
//   n_t = max(target_rpm_vs_throttle(throttle), min_rpm_vs_output_rpm(gearbox output rpm)),  r_t = n_t / output rpm
// clamped to the variator's range (ratio_low at a standstill). In S (transmission.mode >= 0.5) the TCU either aims at
// sport_target_rpm_vs_throttle (stepped_sport false) or runs the variator through its virtual steps on the
// sport_schedule's shift lines (stepped_sport true, default); paddles / shift keys always select the steps (manual
// mode, timeout and forced shifts as for a stepped box). Curves: x ascending; throttle x in [0, 1]; rpm y > 0.
struct CvtTcuDesc {
    vehicle::Curve1D target_rpm_vs_throttle;
    vehicle::Curve1D sport_target_rpm_vs_throttle; // empty = target_rpm_vs_throttle
    vehicle::Curve1D min_rpm_vs_output_rpm;        // empty = none
    bool stepped_sport = true;
};

struct AutomaticTcuDesc {
    std::string gearbox, engine;
    ShiftScheduleDesc schedule;       // D
    ShiftScheduleDesc sport_schedule; // S (mode channel >= 0.5)
    real min_time_in_gear_s = 0.8;    // > = 0, upshift dwell (a downshift by kickdown or the stall guard ignores it)
    real kickdown_throttle = 0.9;     // command at or above: skip-downshift to the lowest gear whose input rpm stays
    real kickdown_max_rpm = 5500.0;   // ... at or below this (and above the downshift line); 0..1 / > 0
    real manual_timeout_s = 0.0;      // >= 0, see above
    real manual_up_rpm = 0.0;         // manual mode: forced upshift at this input rpm (0 = the full-throttle up line)
    // Torque reduction while the on-coming clutch pulls the engine down in a power-on upshift (inertia phase): the
    // engine's throttle is capped at this fraction (1 = none).
    real upshift_torque_fraction = 0.7;
    // Automatic only: lock-up policy. Engaged in gear >= min_gear with the turbine above engage_rpm and the throttle
    // command at or below max_throttle; released below release_rpm, above max_throttle + 0.1, and during every shift.
    bool lockup_enabled = true;
    int lockup_min_gear = 3;
    real lockup_engage_rpm = 1400.0, lockup_release_rpm = 1100.0;
    real lockup_max_throttle = 0.8;
    // DualClutch only: launch on clutch slip. The starting clutch's engagement e goes 0 at bite_rpm -> 1 at
    // lerp(full_rpm_min, full_rpm_max, throttle) of ENGINE rpm; once |slip| < lock_slip_rpm it locks (e = 1) until the
    // engine falls below anti_stall_rpm. creep_torque_nm = the capacity the clutch keeps at idle without throttle in D/R
    // (the creep), 0 = none.
    real launch_bite_rpm = 1000.0, launch_full_rpm_min = 1500.0, launch_full_rpm_max = 3500.0;
    real launch_lock_slip_rpm = 100.0, launch_anti_stall_rpm = 600.0;
    real creep_torque_nm = 40.0;
    // CVT plan C1: Cvt only (see CvtTcuDesc). A Cvt with a converter uses the lock-up policy above (lockup_min_gear
    // counts D as gear 1), one without launches on the launch_* law above.
    std::optional<CvtTcuDesc> cvt;
};

// Closed, visited exhaustively - at most one transmission controller (manual_tcu or automatic_tcu) and at most one
// launch_control per vehicle. AutomaticTcuDesc appended last (automatic gearbox plan A1).
using ControllerParams = std::variant<ManualTcuDesc, LaunchControlDesc, AutomaticTcuDesc>;

struct ControllerDesc {
    std::string id;
    ControllerParams params;
};

} // namespace ps::drivetrain
