// ps/drivetrain/controller_desc.h — public, additive descriptors for
// PHYS-033 P4 I3's per-vehicle controllers (plan section 4.2). Pure data;
// no algorithm code (that lives in core/src/drivetrain/manual_tcu.h/.cpp,
// internal - same D2/D10 pattern as powertrain_desc.h). I5 extends
// ControllerParams's variant and adds default_enabled/rate_divider fields
// to these structs (not needed until then).

#pragma once

#include "ps/types.h"

#include <string>
#include <variant>

namespace ps::drivetrain {

struct AutoClutchDesc {
    bool enabled = true; // default of the runtime assist.* channel (World::create_vehicle); the channel is the switch
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
    // Legacy configuration field retained for source/JSON compatibility.
    // Since 2026-10-04 the controller uses signed gearbox-input RPM and
    // anti_stall_rpm for rolling safety, rather than this separate threshold.
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
    real up_rpm_at_zero_throttle = 2500.0, up_rpm_at_full_throttle = 6300.0;
    real down_rpm_at_zero_throttle = 1300.0, down_rpm_at_full_throttle = 2800.0;
    real min_time_in_gear_s = 1.0;
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

// Closed, visited exhaustively - I3 has exactly one controller kind (at
// most one manual_tcu per vehicle, plan section 9.1); I5 adds more.
using ControllerParams = std::variant<ManualTcuDesc>;

struct ControllerDesc {
    std::string id;
    ControllerParams params;
};

} // namespace ps::drivetrain
