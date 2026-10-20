// core/src/drivetrain/gearbox_rating.h - transmission plan T0: the ONE grading of a gearbox input-torque rating, shared by
// check_gearbox_rating (BuildRule::GearboxRating, engine_rules.cpp) and check_driveline_limits (LimitKind::GearboxInputTorque,
// engine_dyno.cpp), and (owner 2026-10-20) the grading of a clutch's static capacity (LimitKind::ClutchCapacity), which reuses
// the gearbox's Exceeded band. INTERNAL. The grades come from the LimitPolicy (gearbox_rating_over_limiting / _exceeded,
// driveline_torque_margin; defaults rules::kGearboxOver* in engine_rules_table.h, kDrivelineMargin in limit_thresholds.h, L0).
#pragma once

#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/limit_policy.h"
#include "ps/types.h"

namespace ps::drivetrain {

// Peak over rating -> severity. Returns false when the peak is not over the rating (or there is no rating): nothing to report.
[[nodiscard]] inline bool gearbox_rating_severity(real peak_torque_nm, real rating_nm, const LimitPolicy& pol, LimitSeverity& out) {
    if (!(rating_nm > real(0.0)) || !(peak_torque_nm > rating_nm)) return false;
    const real rel = (peak_torque_nm - rating_nm) / rating_nm;
    out = rel > pol.gearbox_rating_over_exceeded   ? LimitSeverity::Exceeded
          : rel > pol.gearbox_rating_over_limiting ? LimitSeverity::Limiting
                                                     : LimitSeverity::Near;
    return true;
}

// Peak against a clutch's STATIC capacity (capacity_nm * static_factor) -> severity. Near while it holds the peak but not the
// peak x driveline_torque_margin; Limiting from the static capacity up to gearbox_rating_over_exceeded over it; Exceeded beyond
// (owner 2026-10-20: the gearbox's band, no clutch key of its own). Returns false when the clutch holds the margin: nothing to report.
[[nodiscard]] inline bool clutch_capacity_severity(real peak_torque_nm, real static_capacity_nm, const LimitPolicy& pol, LimitSeverity& out) {
    if (!(static_capacity_nm > real(0.0))) return false;
    if (peak_torque_nm >= static_capacity_nm) {
        const real rel = (peak_torque_nm - static_capacity_nm) / static_capacity_nm;
        out = rel > pol.gearbox_rating_over_exceeded ? LimitSeverity::Exceeded : LimitSeverity::Limiting;
        return true;
    }
    if (peak_torque_nm * pol.driveline_torque_margin >= static_capacity_nm) {
        out = LimitSeverity::Near;
        return true;
    }
    return false;
}

} // namespace ps::drivetrain
