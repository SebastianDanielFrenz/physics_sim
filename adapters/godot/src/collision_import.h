// adapters/godot/src/collision_import.h — static collision geometry from
// Godot into the sim (PLAN.md P2: "hand static collision geometry from
// Godot to the sim (mesh -> backend shapes), and load the test ground's
// collision data ... into the sim").

#pragma once

#include "ps/world/world.h"

#include <godot_cpp/classes/node.hpp>

#include <string>

namespace ps_godot {

// Walks every descendant of `root`, and for each godot::StaticBody3D finds
// its godot::CollisionShape3D children whose shape is a BoxShape3D,
// SphereShape3D or CapsuleShape3D - the exact three shapes ps::ShapeDesc
// supports (ps/backend/shape_desc.h, PLAN.md D2's own-descriptor seam), so
// this is a direct 1:1 mapping, not a mesh-decomposition heuristic. Any
// other collision shape type (mesh, concave polygon, height map, ...) is
// skipped and counted in `out_skipped` if non-null, since core has nowhere
// to put it yet (see this adapter's report for the requested ShapeDesc
// addition). Each created body's pose is the node's global transform,
// converted through frame_convert.h's single conversion point using
// `origin_m` (pass ps::Vec3::zero() at load time before any origin rebase
// has happened, or the SimBridge's current origin if importing later).
// Returns the number of bodies created.
int import_static_collision_from_scene(godot::Node* root, ps::World& world, const ps::Vec3& origin_m, int* out_skipped = nullptr);

// PLAN.md P2's test-ground collision loader. KNOWN LIMITATION - see the
// .cpp file's header comment and this adapter's final report: without a
// mesh/heightfield ShapeDesc in core, this loads only a single large flat
// static box at z=0 sized from `track_dir`/test_ground.json's
// world.half_extent_m (exact over the flat zones every M1 test feature
// sits on; not exact for ramps, the skidpad, kerb height, or the
// procedural terrain outside the flat zones - PLAN.md's terrain-tile
// system, P8/M1, is what eventually replaces this). Returns an invalid
// BodyId if track_dir/test_ground.json could not be read.
ps::BodyId import_test_ground_flat_plane(const std::string& track_dir, ps::World& world);

} // namespace ps_godot
