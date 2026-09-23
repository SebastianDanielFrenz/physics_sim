// adapters/godot/src/collision_import.h — static collision geometry from
// Godot into the sim (PLAN.md P2: "hand static collision geometry from
// Godot to the sim (mesh -> backend shapes), and load the test ground's
// collision data ... into the sim").

#pragma once

#include "ps/io/surface_table.h"
#include "ps/terrain/terrain_config.h"
#include "ps/world/world.h"

#include <godot_cpp/classes/node.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ps_godot {

// Walks every descendant of `root`, and for each godot::StaticBody3D finds
// its godot::CollisionShape3D children whose shape is a BoxShape3D,
// SphereShape3D or CapsuleShape3D - the exact three shapes ps::ShapeDesc
// supports (ps/backend/shape_desc.h, PLAN.md D2's own-descriptor seam), so
// this is a direct 1:1 mapping, not a mesh-decomposition heuristic. Any
// other collision shape type (mesh, concave polygon, height map, ...) is
// skipped and counted in `out_skipped` if non-null. Each created body's
// pose is the node's global transform, converted through frame_convert.h's
// single conversion point using `origin_m` (pass ps::Vec3::zero() at load
// time before any origin rebase has happened, or the SimBridge's current
// origin if importing later). Returns the number of bodies created.
int import_static_collision_from_scene(godot::Node* root, ps::World& world, const ps::Vec3& origin_m, int* out_skipped = nullptr);

// PLAN.md M1 step 1: the real test-ground loader, superseding the earlier
// P2 flat-plane placeholder (core had no mesh/heightfield ShapeDesc at the
// time - it now has MeshShape/HeightfieldShape, ps::io::load_test_ground_
// collision and ps::terrain::load_procedural_terrain_source, PLAN.md M1).
// Loads:
//  1) `track_dir`'s SurfaceTable (data/surfaces/surfaces.json, resolved as
//     `track_dir + "/../../surfaces/surfaces.json"` - the same relation
//     data/tracks/README.md documents between a track dir and the shared
//     surface table) and installs it on `world` (World::set_surface_table),
//     so every vehicle's tyre model sees real lambda_mu/crr values.
//  2) the exact static feature-mesh collision
//     (`ps::io::load_test_ground_collision`, one MeshShape body per surface
//     group) - replaces the old single flat slab.
//  3) the procedural terrain source (`ps::terrain::load_procedural_terrain_
//     source`, `track_dir + "/terrain.json"`) via World::set_terrain_source,
//     with `terrain_config` (tile pool sizing/fill budget - see terrain_
//     config.h) and `samples_per_tile` resolution.
// Must be called before World::step() ever runs for `world` (setup time,
// like every other set_terrain_source/create_body call in this codebase).
// Returns false (and leaves world/out_surfaces untouched) if any file could
// not be read/parsed; check `out_error` for the reason.
struct TestGroundLoadResult {
    bool ok = false;
    int collision_bodies = 0;
    std::string error;
    // The installed terrain source (null unless ok) - kept by PsSimulation
    // for the fall detector's "where should the ground be" height lookup.
    std::shared_ptr<const ps::terrain::ITerrainSource> terrain_source;
};
TestGroundLoadResult import_test_ground_real(const std::string& track_dir,
                                              ps::World& world,
                                              std::shared_ptr<const ps::io::SurfaceTable>& out_surfaces,
                                              const ps::terrain::TerrainConfig& terrain_config = {},
                                              std::uint32_t samples_per_tile = 64);

} // namespace ps_godot
