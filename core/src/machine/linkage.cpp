#include "ps/machine/linkage.h"

#include "ps/math/transcendental.h"

#include <cmath>
#include <stdexcept>

namespace ps::machine {

void validate_linkage(const LinkageDesc& d) {
    const auto fail = [](const char* what) { throw std::invalid_argument(std::string("linkage: ") + what); };
    const auto tagged = [](const std::string& s) {
        for (const char* tag : {"[C]", "[D]", "[M]", "[AS]"}) {
            if (s.rfind(tag, 0) == 0) return true;
        }
        return false;
    };
    if (!tagged(d.source)) fail("'source' must start with [C], [D], [M] or [AS]");
    if (!(d.max_angle_rad > d.min_angle_rad)) fail("max_deg must lie above min_deg");
    if (!(d.min_angle_rad <= 0.0 && d.max_angle_rad >= 0.0)) fail("the hitch point's own pose must lie between the stops");
    if (!(d.max_lift_force_n > 0.0)) fail("lift.max_force_n must be > 0");
    if (!(d.full_stroke_s > 0.0)) fail("lift.full_stroke_s must be > 0");
    if (!(d.follow_slack_rad >= 0.0)) fail("lift.follow_slack_deg must be >= 0");
    if (!(d.draft_gain_per_s > 0.0)) fail("draft.gain_per_s must be > 0");
    if (!(d.draft_filter_s >= 0.0)) fail("draft.filter_s must be >= 0");
    if (!(d.draft_band >= 0.0 && d.draft_band < 1.0)) fail("draft.band must lie in [0, 1)");
    if (d.position_channel.empty() || d.mode_channel.empty() || d.draft_channel.empty()) fail("a channel name is empty");
}

real linkage_angle(const Vec3& link_local) { return math::atan2(link_local.z, -link_local.x); }

} // namespace ps::machine
