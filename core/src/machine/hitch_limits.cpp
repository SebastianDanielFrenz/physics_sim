#include "ps/machine/hitch_limits.h"

namespace ps::machine {

std::vector<drivetrain::LimitFinding> check_hitch_limits(const HitchPointDesc& point, const HitchLoads& loads, const drivetrain::LimitPolicy* policy) {
    using namespace drivetrain;
    const LimitPolicy& pol = resolve_limit_policy(policy);
    std::vector<LimitFinding> out;
    const auto grade = [&](LimitKind kind, real value, real rating, LimitUnit unit, const char* field) {
        if (rating <= 0.0) return;
        const real share = value / rating;
        LimitFinding f;
        if (share > pol.hitch_exceeded_share) {
            f.severity = LimitSeverity::Exceeded;
        } else if (share > pol.hitch_limiting_share) {
            f.severity = LimitSeverity::Limiting;
        } else if (share > pol.hitch_near_share) {
            f.severity = LimitSeverity::Near;
        } else {
            return;
        }
        f.kind = kind;
        f.value = value;
        f.limit = rating;
        f.unit = unit;
        f.part = PartRef{point.name, "", field};
        out.push_back(f);
    };
    grade(LimitKind::TowedMass, loads.towed_mass_kg, point.max_towed_mass_kg, LimitUnit::Kilogram, "max_towed_mass_kg");
    grade(LimitKind::TongueLoad, loads.tongue_load_n, point.max_tongue_load_n, LimitUnit::Newton, "max_tongue_load_n");
    return out;
}

} // namespace ps::machine
