// ps/types.h — fundamental scalar type for the whole core.
//
// PLAN.md D4: ps::real = double everywhere for poses and world positions.
// Hot per-wheel/per-element math may use float only after a benchmark
// justifies it (not done in P1); such code would introduce its own local
// float types rather than redefining ps::real.

#pragma once

#include <cstdint>

namespace ps {

using real = double;

// Handle generation/index width, shared by BodyId/JointId/ShapeId (world/ids.h).
using id_index_t = std::uint32_t;
using id_generation_t = std::uint32_t;

} // namespace ps
