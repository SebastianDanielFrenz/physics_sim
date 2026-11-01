// ps/machine/hitch_limits.h — a towing point's ratings as limit findings
// (LimitKind::TowedMass, TongueLoad, LiftCapacity; the thresholds are LimitPolicy entries,
// data/limit_policies/default.json group "hitch"). A rating never refuses a hitch
// by itself: the findings are for whoever advises the player.

#pragma once

#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/limit_policy.h"
#include "ps/machine/hitch_point.h"
#include "ps/types.h"

#include <vector>

namespace ps::machine {

struct HitchLoads {
    real towed_mass_kg = 0.0; // everything behind the hitch, a chain's further links included
    real tongue_load_n = 0.0; // down on the towing socket
    real lift_force_n = 0.0;  // a linkage's force at the lower pins
    bool lift_saturated = false; // and it asked for more than the lift has
};

[[nodiscard]] std::vector<drivetrain::LimitFinding> check_hitch_limits(const HitchPointDesc& point, const HitchLoads& loads,
                                                                       const drivetrain::LimitPolicy* policy = nullptr);

} // namespace ps::machine
