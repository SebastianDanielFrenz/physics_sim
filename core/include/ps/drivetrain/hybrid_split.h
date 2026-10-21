// ps/drivetrain/hybrid_split.h - the hybrid supervisor's power split as a public, pure function (powertrain dyno,
// 2026-10-19). Powertrain::pre_solve_components (core/src/drivetrain/powertrain.cpp) calls exactly this every substep,
// so a garage that draws the split from it sees the car's own rule. docs/electric_drive.md "Hydrogen and hybrid
// development packages" describes the rule; the state the supervisor keeps (selector latch, stop delay, crank timeout,
// the low-SOC charging throttle) stays in the supervisor.
//
//   requested = driver command * requested_power_w
//   extra     = drive_mode != Eco && requested > available
//   coupled   = selector != N && drive_mode != Eco && (extra || drive_mode >= Sport)   (the ICE drives the wheels)
//   wanted    = ready && (charge latched || drive_mode >= Sport || extra)                (the ICE should run)
//   electric  = ready ? clamp(requested / max(1, available), 0, 1) : 0                    (the traction motors' pedal)
//   engine    = coupled ? clamp((requested - available) / max(1, requested_power_w - available), 0, 1) : 0
// The charge latch sets at soc <= low_soc and clears at soc >= charge_target_soc - 1e-6 (set first, then clear).
// Drive modes (channel assist.drive_mode, rounded and clamped): 0 Eco, 1 Normal (default), 2 Sport, 3 Racing.
#pragma once

#include "ps/drivetrain/electric_drive_desc.h"
#include "ps/types.h"

namespace ps::drivetrain {

struct HybridSplitInput {
    real driver_command = 1.0; // the driver's pedal command (pedal curve and drive-mode bend applied), 0..1
    real available_w = 0.0;    // electric potential: min(rated traction motors, pack discharge + fuel-cell rating)
    int drive_mode = 1;        // 0..3, see the header
    real battery_soc = 0.0;    // energy-weighted over every pack
    bool charge_latched = false; // the latch BEFORE this update (false for a fresh car)
    bool ready = true;           // channel electric.ready
    real selector = 1.0;         // D/N/R latch: > 0 D, 0 N, < 0 R
};

struct HybridSplit {
    real requested_w = 0.0;
    real electric_command = 0.0; // traction-motor pedal
    real engine_command = 0.0;   // ICE command while coupled (the low-SOC charging throttle is the supervisor's, not this)
    bool extra_power = false;    // the request exceeds the electric potential (Normal and above)
    bool engine_coupled = false; // the ICE should drive the wheels (gearbox in gear) - once it runs
    bool engine_wanted = false;  // the ICE should run (start / keep running)
    bool charge_latched = false; // the latch AFTER this update
};

[[nodiscard]] bool hybrid_update_charge_latch(const HybridControlDesc& c, real battery_soc, bool latched);
[[nodiscard]] HybridSplit hybrid_power_split(const HybridControlDesc& c, const HybridSplitInput& in);

// Full pedal convenience (the dyno chart): driver_command = pedal, D selected, ready.
[[nodiscard]] HybridSplit hybrid_full_pedal_split(const HybridControlDesc& c, real available_w, real pedal, int drive_mode,
                                                  real battery_soc, bool charge_latched = false);

} // namespace ps::drivetrain
