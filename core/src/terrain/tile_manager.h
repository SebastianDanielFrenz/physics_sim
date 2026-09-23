// core/src/terrain/tile_manager.h — internal: streams ITerrainSource tiles
// around registered interest points, creates/destroys/updates heightfield
// collision bodies through IRigidBackend, runs generation as jobs (PLAN.md
// D8). Owned by World (core/src/world/world.cpp), forward-declared from
// world.h exactly like articulation::System.
//
// Scope decision (PLAN.md M1 explicitly allows this: "keep one resolution
// and say so"): SINGLE FIXED RESOLUTION, no LOD/mip system. Every resident
// tile is ITerrainSource::samples_per_tile() x samples_per_tile() regardless of
// distance from an interest point. Rationale: a correct LOD scheme needs
// seam-free stitching between differently-sampled neighbour tiles (skirts,
// T-junction fixes, or a quadtree with matching-resolution borders) - real
// engineering with real failure modes (that is exactly the "seams" PLAN.md
// M1's acceptance test worries about), and getting it provably right within
// this task's budget was not achievable alongside everything else in the
// brief. A single resolution has NO seam risk at all: every tile's edge
// samples are the literal same world-coordinate evaluation as its
// neighbour's (see procedural_terrain_source.cpp's fill_tile - the shared
// boundary column/row lands on the same world (x, y) on both sides by
// construction), which tests/unit/test_terrain_tile_manager.cpp checks with
// exact equality, not a tolerance. The cost this defers: memory/generation
// cost scales with (stream_radius / tile_size)^2 at FULL resolution even
// far from the interest point, rather than coarsening with distance -
// acceptable at the pool sizes and stream radii this task measures (see
// the bench and this header's own numbers), flagged here for a future P8+
// session if a much larger radius or much finer near-field resolution is
// ever needed simultaneously.
//
// Zero-allocation-in-jobs scope (PLAN.md D8): TileSample generation
// (ITerrainSource::fill_tile) runs inside ps::jobs::parallel_for, into a
// slot's OWN preallocated, never-resized heights/cell_surface_ids buffers -
// that part is genuinely allocation-free every tick, steady-state and
// otherwise (nothing here depends on a warm-up). Handing a freshly filled
// tile's content to the backend (IRigidBackend::update_heightfield_body,
// IRigidBackend::set_pose) happens sequentially, still inside this stage's
// own job, and is zero-allocation ONLY in the STEADY STATE the way every
// other backend scratch buffer in this codebase is (see
// update_heightfield_body's own comment in jolt_backend.cpp) - the FIRST
// tile ever assigned to a given pool slot is not claimed allocation-free
// (Jolt's own SetShape/body bookkeeping may still touch the heap the very
// first time), which is why this pool is created once, up front, at
// World::set_terrain_source time (setup, not a job) - see TileManager's
// constructor.

#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/jobs/job_system.h"
#include "ps/terrain/terrain_config.h"
#include "ps/terrain/terrain_source.h"

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace ps::terrain {

class TileManager {
public:
    TileManager(std::shared_ptr<const ITerrainSource> source, const TerrainConfig& config, IRigidBackend& backend);
    ~TileManager();
    TileManager(const TileManager&) = delete;
    TileManager& operator=(const TileManager&) = delete;

    // Creates or moves a named interest point (e.g. one per vehicle/camera -
    // PLAN.md section 5.4/M1: "streamed around active vehicles").
    void set_interest_point(std::uint64_t id, const Vec3& world_xy, real stream_radius_m);
    void remove_interest_point(std::uint64_t id);

    // Called once per tick from World's terrain_tiles stage. Fills up to
    // config().max_tile_fills_per_tick newly-needed tiles (parallel_for
    // across this tick's own batch, then a sequential backend-update pass -
    // see this file's header comment).
    void update(jobs::JobSystem& jobs);

    [[nodiscard]] std::size_t resident_tile_count() const { return resident_count_; }
    // Tiles that are currently needed by some interest point but have no
    // free/reusable pool slot to occupy - PLAN.md M1 wants this reported,
    // not silently dropped (an undersized pool relative to the requested
    // stream radius/interest-point count).
    [[nodiscard]] std::size_t starved_tile_count() const { return last_starved_count_; }
    [[nodiscard]] const TerrainConfig& config() const { return config_; }

private:
    struct InterestPoint {
        Vec3 world_xy;
        real radius_m = 0.0;
    };

    struct Slot {
        bool ever_assigned = false; // false until this slot's body exists with real content
        bool assigned = false;      // true while `key` is currently needed
        TileKey key{};
        BodyId body{};
        TileSample sample; // preallocated once (ensure_slot_storage), never resized after
    };

    void ensure_slot_storage(Slot& slot);
    // Returns the slot index newly holding `key` (reusing a free or stale
    // slot), or SIZE_MAX if the pool has no slot to give it.
    std::size_t acquire_slot_for(TileKey key);

    std::shared_ptr<const ITerrainSource> source_;
    TerrainConfig config_;
    IRigidBackend& backend_;

    std::vector<Slot> slots_;
    std::unordered_map<std::int64_t, std::size_t> key_to_slot_; // TileKey::packed -> slots_ index
    std::vector<std::size_t> free_slots_;                       // never-yet-assigned slot indices

    std::unordered_map<std::uint64_t, InterestPoint> interest_points_;

    std::size_t resident_count_ = 0;
    std::size_t last_starved_count_ = 0;

    // Scratch reused across update() calls (PLAN.md D8: no per-tick
    // allocation once warmed up) - the set of keys needed this tick and the
    // subset of those that are newly-needed (not already resident), capped
    // per tick by max_tile_fills_per_tick and carried over across ticks so
    // a burst spreads out instead of being dropped.
    std::vector<TileKey> needed_keys_scratch_;
    std::vector<TileKey> pending_fill_queue_;
    std::vector<std::size_t> this_tick_slots_scratch_;
};

} // namespace ps::terrain
