// ps/terrain/terrain_config.h — TileManager configuration (PLAN.md M1 tile
// system). Split out so ps/world/world.h does not need the internal tile
// manager header to expose World::set_terrain_source's parameter type.
//
// Grid resolution (samples per tile edge) is NOT here: it is a property of
// the ITerrainSource itself (ITerrainSource::samples_per_tile(), fixed for
// that source's whole lifetime - PLAN.md M1: single fixed resolution, no
// LOD, see tile_manager.h's header comment for why). TileManager always
// sizes its pool from the source it was given, so a mismatch between "how
// many samples the manager allocates" and "how many samples fill_tile
// writes" cannot happen by construction - found the hard way (PHYS-014-ish:
// a scenario-loaded source built with the ProceduralTerrainSource
// constructor's OWN default 64, while the scenario's TerrainConfig used to
// separately carry its own samples_per_tile, e.g. 8, for the manager's pool
// - fill_tile then wrote up to 64x64 samples into an 8x8-sized buffer, a
// silent heap overflow that only an MSVC STL debug-iterator build's
// "vector subscript out of range" caught).

#pragma once

#include <cstddef>
#include <cstdint>

namespace ps::terrain {

struct TerrainConfig {
    // Fixed pool size: how many tiles can be resident (have a live
    // heightfield collision body) at once. Must be large enough to cover
    // every interest point's own stream radius simultaneously (sum over all
    // interest points of the tile count within radius) - an undersized
    // pool means some needed tiles never get a body and produce no
    // collision, reported via TileManager::starved_tile_count().
    std::size_t max_resident_tiles = 256;

    // Upper bound on how many tiles are (re)assigned and filled in a single
    // World::step() call - the budget that keeps a sudden burst of newly-
    // needed tiles (e.g. a vehicle teleported 50 km) from stalling one
    // tick; the rest queue up and stream in over the following ticks
    // instead. A steady-state drive needs far fewer than this per tick
    // (roughly the stream boundary's perimeter / tile size).
    std::size_t max_tile_fills_per_tick = 4;
};

} // namespace ps::terrain
