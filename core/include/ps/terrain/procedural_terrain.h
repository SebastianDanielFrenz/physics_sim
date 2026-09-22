// ps/terrain/procedural_terrain.h — factory for the procedural
// ITerrainSource (PLAN.md M1: "a procedural implementation reading the
// terrain.json spec format"). Implementation (TerrainSpec parsing, the
// value-noise height function) stays internal
// (core/src/terrain/procedural_terrain_source.h/.cpp) - this header only
// exposes the abstract ITerrainSource, same pattern as
// ps/backend/i_rigid_backend.h's create_jolt_backend().

#pragma once

#include "ps/terrain/terrain_source.h"

#include <memory>
#include <string>

namespace ps::terrain {

// Parses a terrain.json file (format "physics_sim.terrain/1" - seed,
// height_noise octaves, flat_zones, surface_rules, tiling.tile_size_m; see
// data/tracks/test_ground/terrain.json for an example and
// tools/trackgen/trackgen/terrain.py for the Python reference this must
// match bit-for-bit) and returns a source that evaluates it exactly.
// Throws std::runtime_error on a malformed or unreadable file.
std::shared_ptr<ITerrainSource> load_procedural_terrain_source(const std::string& terrain_json_path);

} // namespace ps::terrain
