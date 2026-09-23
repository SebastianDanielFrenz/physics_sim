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

#include <cstdint>
#include <limits>
#include <variant>
#include <vector>

namespace ps {

struct BoxShape { Vec3 half_extents{0.5, 0.5, 0.5}; };
struct SphereShape { real radius = 0.5; };
struct CapsuleShape { real half_height = 0.5; real radius = 0.25; };

// PLAN.md 5.1/P3: wheel contact is a cylinder shape cast, not a single ray
// or a capsule (a capsule's rounded ends bias the contact point/normal
// versus a real tyre's flat-ish tread face at small camber). Same axis
// convention as CapsuleShape above: local Y (JoltPhysics' own
// JPH::CylinderShape axis, passed through unrotated - no wrapper shape is
// added, exactly like CapsuleShape's own mapping in jolt_backend.cpp), so a
// caller orients the wheel's own query pose to put local Y along the wheel
// spin axis.
struct CylinderShape { real half_height = 0.5; real radius = 0.3; };

// Static triangle-mesh collision (PLAN.md M1 "static test-ground collision":
// data/tracks/test_ground's own binary format is triangles grouped by
// physical surface - see core/src/io/test_ground_loader.cpp). One surface id
// per triangle: triangle_surface_ids[i] applies to indices[3*i..3*i+2], or
// the whole vector may be left empty for kInvalidSurfaceId everywhere.
// Triangles must be counter-clockwise (viewed from the side that should
// collide - Jolt's own MeshShapeSettings convention); this shape is for
// static geometry only (IRigidBackend does not require Static motion here,
// but JoltPhysics itself refuses a mesh shape on a non-static body - see
// JPH::MeshShape::MustBeStatic()).
struct MeshShape {
    std::vector<Vec3> vertices;
    std::vector<std::uint32_t> indices; // triples, length a multiple of 3
    std::vector<SurfaceId> triangle_surface_ids; // size == indices.size()/3, or empty
};

// Static heightfield collision (PLAN.md section 6 ITerrainSource / M1 tile
// system, core/include/ps/terrain/terrain_source.h). A square grid of
// sample_count x sample_count height samples, row-major with y the slower-
// varying index (heights[y*sample_count + x]), at body-local positions
// origin + (x, y) * sample_spacing - height is along body-local +z (our own
// Z-up convention; the Jolt backend does the axis fix-up to Jolt's own
// Y-up-native heightfield internally, never exposed here). One surface id
// per (sample_count-1)^2 cell, same row-major convention
// (cell_surface_ids[y*(sample_count-1) + x]), or empty for kInvalidSurfaceId
// everywhere. Static geometry only, same reason as MeshShape above.
//
// sample_count must be EVEN (a multiple of 2): Jolt's own HeightFieldShape
// rounds its internal sample count up to a multiple of its block size
// (default 2, JoltPhysics HeightFieldShape.cpp) at construction time, and
// IRigidBackend::update_heightfield_body requires the caller's height/
// surface-id arrays to match that (possibly-rounded) count exactly - using
// an even sample_count here keeps the two always equal, with no rounding
// surprise to reconcile at update time.
struct HeightfieldShape {
    Vec3 origin = Vec3::zero();
    real sample_spacing = 1.0;
    std::uint32_t sample_count = 0; // samples per side; must be even and >= 2
    std::vector<real> heights;              // sample_count^2
    std::vector<SurfaceId> cell_surface_ids; // (sample_count-1)^2, or empty
};

// A `HeightfieldShape::heights` sample exactly equal to this value has NO
// collision at that grid vertex - every triangle touching it is dropped
// (JoltPhysics' own JPH::HeightFieldShapeConstants::cNoCollisionValue,
// FLT_MAX, and this is exactly that value widened to `real` - chosen so the
// existing plain real->float sample cast in jolt_backend.cpp
// (reverse_heightfield_rows_into, used by both initial shape construction
// and the hot update_heightfield_body/SetHeights path) carries it through
// byte-identically with no special-case code anywhere in the backend).
// PLAN.md M1: ps::terrain::TileManager writes this into a tile's samples
// wherever it detects (by an actual physics query, not a hardcoded rule)
// that another STATIC body already occupies the exact same footprint - see
// tile_manager.h's header comment and data/tracks/README.md's "a meshed
// feature's own footprint always wins locally" - so the terrain and a
// static feature mesh never both claim the same point at once.
inline constexpr real kHeightfieldNoCollision = static_cast<real>(std::numeric_limits<float>::max());

using ShapeDesc = std::variant<BoxShape, SphereShape, CapsuleShape, CylinderShape, MeshShape, HeightfieldShape>;

} // namespace ps
