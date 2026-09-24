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
    bool enabled = true;
    real bite_rpm = 1100.0;      // engagement 0 at or below this engine speed (launch law)
    real full_rpm_min = 1600.0;  // engagement 1 at this rpm with throttle 0 ...
    real full_rpm_max = 3000.0;  // ... and at this rpm with throttle 1
    real lock_slip_rpm = 80.0;   // |engine - input| below this (and rpm > bite) -> Locked
    real anti_stall_rpm = 950.0; // Locked and rpm below this -> back to the launch law
    real engage_rate_per_s = 4.0, release_rate_per_s = 12.0;
    real open_time_s = 0.08, close_time_s = 0.20; // shift ramps
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
