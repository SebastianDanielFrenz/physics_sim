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

using BodyId = Id<BodyIdTag>;
using JointId = Id<JointIdTag>;
using ForceElementId = Id<ForceElementIdTag>;

} // namespace ps
