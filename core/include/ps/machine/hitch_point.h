// ps/machine/hitch_point.h — a place on a vehicle where a hitch goes (the
// "hitch_points" list of a vehicle file, docs/data_formats/vehicle.md).
//
// A point carries the name of the socket in the vehicle's rig
// (data/models/*/*.rig.json) and the class it takes. Two points mate when one tows,
// the other is towed and both name the same class; a linkage that stands on several
// rig sockets, the three of a three-point hitch, is one point at their centroid.

#pragma once

#include "ps/machine/hitch_class.h"
#include "ps/math/vec3.h"
#include "ps/types.h"

#include <string>
#include <string_view>
#include <vector>

namespace ps::machine {

enum class HitchRole : unsigned char { Towing, Towed };

struct HitchPointDesc {
    std::string name;
    HitchRole role = HitchRole::Towing;
    HitchClassDesc hitch;
    Vec3 position_local{};                // chassis-local, like a wheel's attachment
    std::vector<std::string> rig_sockets; // the rig's names; one, unless the point stands on several

    // Ratings of a towing point, read by check_hitch_limits. 0: not rated.
    real max_towed_mass_kg = 0.0;
    real max_tongue_load_n = 0.0;
};

[[nodiscard]] inline const HitchPointDesc* find_hitch_point(const std::vector<HitchPointDesc>& points, std::string_view name) {
    for (const HitchPointDesc& p : points) {
        if (p.name == name) return &p;
    }
    return nullptr;
}

} // namespace ps::machine
