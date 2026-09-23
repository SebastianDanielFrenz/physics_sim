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

    // PLAN.md M1 / PHYS-021: whenever a newly-filled tile has a sample at
    // exactly height 0 (terrain.json's own documented invariant - height is
    // EXACTLY 0 inside a flat zone's interior, nowhere else), TileManager
    // probes the backend for another STATIC body already occupying that
    // exact point (e.g. a track's static feature mesh, which - by the same
    // spec - only ever sits on a flat zone, at height 0) and, if found,
    // marks that sample as HeightfieldShape's no-collision sentinel
    // (ps::kHeightfieldNoCollision) instead of writing the terrain's own
    // height there. Without this, a wheel/ray query can hit either the
    // terrain heightfield or the feature mesh at the same point depending
    // on tiny, tick-to-tick Jolt-internal tie-breaking - flickering the
    // reported surface (and its friction) every tick. Detected by an actual
    // physics query against whatever static geometry happens to be loaded,
    // not by any hardcoded knowledge of a specific track, so it stays
    // correct for any future static feature set on the same terrain source.
    // A scenario with no flat zones (or no other static bodies near them)
    // pays for exactly one extra broad-phase probe per newly-filled tile
    // and nothing else (see tile_manager.cpp's suppress_covered_samples).
    // Default on; turn off only to measure pure tile-fill cost with zero
    // query overhead.
    bool suppress_terrain_under_static_geometry = true;
};

} // namespace ps::terrain
