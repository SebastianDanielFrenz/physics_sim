// core/src/drivetrain/gearbox_rating.h - transmission plan T0: the ONE grading of a gearbox input-torque rating, shared by
// check_gearbox_rating (BuildRule::GearboxRating, engine_rules.cpp) and check_driveline_limits (LimitKind::GearboxInputTorque,
// engine_dyno.cpp). INTERNAL. The grades come from the LimitPolicy (gearbox_rating_over_limiting / _exceeded; defaults
// rules::kGearboxOver* in engine_rules_table.h, L0).
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

} // namespace ps::drivetrain
