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

// Opaque physical-surface identifier (PLAN.md section 6: "the surface table
// is data ... shared by all sources"). ps_core assigns no meaning to the
// value itself - a caller (terrain tiles, the test-ground loader, later tyre
// code) maps it to physical parameters via ps::io::SurfaceTable
// (ps/io/surface_table.h); the backend's own job is only to carry it
// untouched through collision shapes, collide_shape_into results and
// contact events. 16 bits is a deliberate seam-tightener, not a real limit
// on the number of surfaces: Jolt's own per-cell/per-triangle material index
// is 8 bits (see core/src/backend/jolt/jolt_backend.cpp), so any single mesh
// or heightfield shape is already capped at 256 distinct surfaces well below
// this type's range.
using SurfaceId = std::uint16_t;
constexpr SurfaceId kInvalidSurfaceId = 0xFFFFu;

} // namespace ps
