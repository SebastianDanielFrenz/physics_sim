// ps/terrain/procedural_terrain.h — factory for the procedural
// ITerrainSource (PLAN.md M1: "a procedural implementation reading the
// terrain.json spec format"). Implementation (TerrainSpec parsing, the
// value-noise height function) stays internal
// (core/src/terrain/procedural_terrain_source.h/.cpp) - this header only
// exposes the abstract ITerrainSource, same pattern as
// ps/backend/i_rigid_backend.h's create_jolt_backend().

#pragma once

#include "ps/terrain/terrain_source.h"

#include <cstdint>
#include <memory>
#include <string>

namespace ps::terrain {

// Parses a terrain.json file (format "physics_sim.terrain/1" - seed,
// height_noise octaves, flat_zones, surface_rules, tiling.tile_size_m; see
// data/tracks/test_ground/terrain.json for an example and
// tools/trackgen/trackgen/terrain.py for the Python reference this must
// match bit-for-bit) and returns a source that evaluates it exactly.
// samples_per_tile fixes the returned source's grid resolution for its
// whole lifetime (ITerrainSource::samples_per_tile()) - the caller (e.g.
// ps::io::build_scenario) must be the one deciding it, since TileManager
// sizes its entire pool from this same source (see
// ps/terrain/terrain_config.h's header comment for the bug a decoupled,
// separately-configured resolution caused).
// Throws std::runtime_error on a malformed or unreadable file.
std::shared_ptr<ITerrainSource> load_procedural_terrain_source(const std::string& terrain_json_path, std::uint32_t samples_per_tile = 64);

} // namespace ps::terrain
