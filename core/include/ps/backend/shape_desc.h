// ps/backend/shape_desc.h — collision shape descriptors (PLAN.md D2: our
// own handle/descriptor types, never a Jolt shape crossing the backend
// seam).
//
// Split out of i_rigid_backend.h (P1.5b) so a header that only needs a
// shape (e.g. ps/articulation/articulation_desc.h's per-link contact
// shape) does not have to pull in JointType, BodyDesc and the rest of the
// rigid-body backend interface - JointType in particular collides by name
// with ps::articulation::JointType, and both used to be visible unqualified
// wherever a caller had both `using namespace ps;` and
// `using namespace ps::articulation;` in scope.

#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"

#include <variant>

namespace ps {

struct BoxShape { Vec3 half_extents{0.5, 0.5, 0.5}; };
struct SphereShape { real radius = 0.5; };
struct CapsuleShape { real half_height = 0.5; real radius = 0.25; };
using ShapeDesc = std::variant<BoxShape, SphereShape, CapsuleShape>;

} // namespace ps
