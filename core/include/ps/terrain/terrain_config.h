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

    // PHYS-021 Task B: compute the suppression result for every tile that
    // touches foreign static geometry ONCE, inside World::set_terrain_source
    // (load time, parallel across tiles), instead of on the tick path the
    // first time each tile streams in. A tile not precomputed still takes
    // the lazy per-fill path. The result is bit-identical either way; this
    // only moves where the cost is paid. Contract: static geometry should be
    // complete before set_terrain_source (a Static body created or destroyed
    // later through World drops every cached result, see
    // TileManager::on_static_geometry_changed). Ignored when
    // suppress_terrain_under_static_geometry is off.
    bool precompute_suppression_at_load = true;

    // Terrain prefetch ring (owner-approved 2026-09-27): how many tiles
    // beyond the needed square TileManager builds ahead of time on a
    // background lane, so the tile a moving interest point is about to need
    // is already built (fill_tile + datum, and - gated on the C0 gate's
    // Option A/B outcome - the Jolt-side shape upload too) by the
    // DETERMINISTIC tick it actually enters the needed set, instead of
    // being built synchronously in that tick (today's only path, and the
    // one that costs several 240 Hz ticks at a 256^2-sample row crossing -
    // see bench_terrain_tiles/the session report's before/after numbers).
    // 0 (default) == prefetch OFF: TileManager's behaviour, timing and
    // every existing state_hash are exactly today's - this field does not
    // exist as far as any pre-existing scenario/test is concerned. A
    // nonzero value only ever WIDENS the set of keys built ahead of time;
    // it never changes WHEN a tile is installed (still the deterministic
    // tick the key enters the needed set, never "whenever the build
    // finishes" - see TileManager's own header comment on the ticket
    // install protocol) or which pool BodyId a key lands on, so prefetch
    // on vs off must hash identically at every worker count.
    std::uint32_t prefetch_margin_tiles = 0;

    // Fixed size of the background-build ticket pool (one ticket can be
    // in flight per prefetch-ring key at a time). 0 (default) == prefetch
    // OFF, same as prefetch_margin_tiles == 0 - both must be nonzero to
    // enable the background lane; TileManager rejects (clamps to 0/off) a
    // nonzero margin with a zero ticket pool rather than guessing a size.
    std::uint32_t prefetch_max_tiles = 0;
};

// Terrain prefetch ring counters (TileManager::prefetch_stats(), World::
// terrain_prefetch_stats()), lifetime totals. Diagnostics only - none of
// them feeds back into the simulation, and every one stays 0 with prefetch
// off. "Late" means a key reached its install tick while its background
// build had not finished: late_sync = the build had not started yet, so it
// was cancelled and the tile filled synchronously exactly as with prefetch
// off; late_wait = the build was running, so the tick waited for it
// (late_wait_ns_total/max = how long). stale_inputs = a Snapshot source's
// inputs changed after capture, so the background result was discarded and
// the tile filled synchronously. skipped_suppression = enqueue attempts
// skipped because the key's suppression holes were neither cached nor
// provably empty (see TileManager's enqueue rule).
struct PrefetchStats {
    std::uint64_t enqueued = 0;
    std::uint64_t installed = 0; // tiles installed from a background build (a spare swap)
    std::uint64_t late_sync = 0;
    std::uint64_t late_wait = 0;
    std::uint64_t late_wait_ns_total = 0;
    std::uint64_t late_wait_ns_max = 0;
    std::uint64_t stale_inputs = 0;
    std::uint64_t cancelled = 0;
    std::uint64_t skipped_suppression = 0;
};

} // namespace ps::terrain
