// ps/terrain/terrain_config.h — TileManager configuration (PLAN.md M1 tile
// system). Split out so ps/world/world.h does not need the internal tile
// manager header to expose World::set_terrain_source's parameter type.

#pragma once

#include <cstddef>
#include <cstdint>

namespace ps::terrain {

struct TerrainConfig {
    // Samples per tile edge (grid is samples_per_tile x samples_per_tile).
    // Must be EVEN (matches ps::HeightfieldShape's own constraint - Jolt's
    // HeightFieldShape rounds its sample count up to a multiple of its
    // block size, default 2). PLAN.md M1: single fixed resolution, no LOD -
    // see tile_manager.h's header comment for why that scope was chosen.
    std::uint32_t samples_per_tile = 64;

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
