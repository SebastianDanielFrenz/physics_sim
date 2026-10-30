// ps/world/ids.h — opaque handles into the backend/World (PLAN.md section
// 5: "Bodies and joints are handles into the backend"). No backend type is
// ever exposed; an id is just an index + generation so a stale handle to a
// destroyed/recycled slot is detectable rather than silently valid.

#pragma once

#include "ps/types.h"

namespace ps {

template <typename Tag>
struct Id {
    id_index_t index = 0;
    id_generation_t generation = 0; // 0 == never assigned / invalid

    [[nodiscard]] constexpr bool valid() const { return generation != 0; }
    constexpr bool operator==(const Id&) const = default;
};

struct BodyIdTag {};
struct JointIdTag {};
struct ForceElementIdTag {};
struct ArticulationIdTag {}; // PLAN.md D10/P1.5a: ps::World::create_articulation
struct VehicleIdTag {};      // PLAN.md 5.1/P3: ps::World::create_vehicle
struct HitchIdTag {};        // attachment spike A0: ps::World::create_coupling

using BodyId = Id<BodyIdTag>;
using JointId = Id<JointIdTag>;
using ForceElementId = Id<ForceElementIdTag>;
using ArticulationId = Id<ArticulationIdTag>;
using VehicleId = Id<VehicleIdTag>;
using HitchId = Id<HitchIdTag>;

} // namespace ps
