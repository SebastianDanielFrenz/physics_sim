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
// Per-tile vertical datum (R1, PLAN.md M1 follow-up): ITerrainSource::
// fill_tile always produces ABSOLUTE heights (real-world terrain can sit
// anywhere from ~90-880 m ASL); TileManager subtracts each tile's OWN
// snapped-to-128 m datum before uploading it to the backend and adds the
// datum back via the tile body's pose z, so Jolt's fixed +-200 m
// quantisation range (see kPlaceholderHeightRangeM's comment in
// tile_manager.cpp) only ever has to cover one tile's local relief, never
// the source's whole elevation range. See update()'s own comment for the
// computation and relief_overflow_count() above for what happens when a
// tile's relief alone exceeds that range.
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

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
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

    // R1 (real-world terrain, 90-880 m ASL): how many times a newly-filled
    // tile's own local relief (post-datum, see tile_manager.cpp's update())
    // exceeded the pool's fixed +-kPlaceholderHeightRangeM quantisation
    // range minus a small margin - see tile_manager.cpp for the exact bound.
    // Such a tile is still uploaded (clamped by Jolt's own SetHeights, see
    // shape_desc.h's HeightfieldShape doc comment) so the car has SOME
    // ground rather than none; this counter is how a caller notices instead
    // of it happening silently. Lifetime total, like lazy_suppression_count().
    [[nodiscard]] std::size_t relief_overflow_count() const { return relief_overflow_count_; }
    // White-box test hook: the datum currently applied to `key`'s resident
    // tile (0.0 if `key` is not resident) - see tile_manager.cpp's update()
    // for how it is computed and applied.
    [[nodiscard]] real resident_tile_datum(TileKey key) const;

    // PLAN.md M1 / PHYS-021 Task B - load-time suppression precompute.
    // Enumerates every tile key touched by a foreign static body's actual
    // triangles (IRigidBackend::body_triangle_xy_bounds_into, binned to
    // tile keys - never a body's overall AABB) and computes each one's
    // suppression hole list with the SAME function the lazy per-fill path
    // uses (compute_suppression_holes), in parallel across tiles
    // (jobs.parallel_for, one private scratch per batch), then stores the
    // results in suppression_cache_ in ascending key order. A runtime fill
    // of such a tile is then a cache hit: no probes on the tick path. Any
    // tile NOT precomputed still takes the lazy path. Called by
    // World::set_terrain_source right after construction; allocates (load
    // time only). No-op when suppress_terrain_under_static_geometry or
    // precompute_suppression_at_load is off.
    void precompute_suppression(jobs::JobSystem& jobs);
    // Static geometry changed after construction (World::create_body/
    // destroy_body of a Static body while a terrain source is set): drops
    // every cached suppression result, precomputed or lazily computed, so
    // tiles filled from now on are probed against the new geometry.
    // Already-resident tiles keep the holes they were filled with. Setup-
    // time only; no current flow creates static bodies after
    // set_terrain_source.
    void on_static_geometry_changed();
    [[nodiscard]] std::size_t precomputed_tile_count() const { return precomputed_keys_.size(); }
    [[nodiscard]] const std::vector<TileKey>& precomputed_keys() const { return precomputed_keys_; }
    // How many tile fills took the lazy (probing) suppression path rather
    // than a cache hit, over this manager's lifetime.
    [[nodiscard]] std::size_t lazy_suppression_count() const { return lazy_suppression_count_; }
    // White-box test hooks (tests/unit/test_terrain_suppression_precompute.cpp):
    // the cached hole list for `key` (nullptr if none), and the lazy path's
    // result for `key` computed from scratch on a freshly filled sample,
    // bypassing and not touching the cache.
    [[nodiscard]] const std::vector<std::uint32_t>* cached_suppression(TileKey key) const;
    [[nodiscard]] std::vector<std::uint32_t> compute_suppression_uncached(TileKey key) const;

    // --- Terrain prefetch ring (owner-approved 2026-09-27) -----------------
    // Setup-time: enables the prefetch ring when config().prefetch_margin_tiles
    // and prefetch_max_tiles are both nonzero and the source's
    // prefetch_support() is not TickOnly; otherwise a no-op and update()
    // stays exactly the pre-prefetch path. Starts `jobs`' background lane
    // (JobSystem::start_background_lane), creates one build context +
    // TileSample per lane thread and one heightfield spare per ticket
    // (allocates). `jobs` must outlive this TileManager (the destructor
    // drains its lane). World::set_terrain_source calls this right after
    // precompute_suppression; call precompute_suppression FIRST - the
    // enqueue rule below relies on it (precompute_complete_).
    //
    // Ticket protocol (see update()): at the end of every update() the ring
    // keys (every interest point's square widened by prefetch_margin_tiles,
    // pending-fill keys first) that are not resident and not ticketed get a
    // Free ticket and a lane job that fills the tile, applies its cached
    // suppression holes, computes its datum and builds the ticket's spare.
    // A key is installed ONLY at the tick today's path would install it
    // (steps 1-3 are unchanged, so slot, BodyId and install tick per key
    // are identical with prefetch on or off): a Done ticket is installed by
    // swap_heightfield_spare + set_pose; a still-Queued one is cancelled and
    // the tile filled synchronously (late_sync); a Running one is waited for
    // (late_wait). Right after that partition, tickets for keys that left
    // both the ring and the pending queue are cancelled.
    //
    // Suppression rule at enqueue: suppression off -> no holes; a cached
    // hole list -> the ticket points at it (unordered_map values never move;
    // entries are erased only by on_static_geometry_changed, which drains
    // every ticket first); no cache entry but precompute_complete_ -> no
    // holes, because the load-time precompute covers EVERY tile a foreign
    // static triangle touches (1 m binning margin) and compute_suppression_
    // holes returns nothing for any other tile (its whole-tile probe finds
    // no foreign static body); otherwise the key is skipped
    // (skipped_suppression) and filled synchronously when needed.
    void start_prefetch(jobs::JobSystem& jobs);
    [[nodiscard]] bool prefetch_enabled() const { return lane_jobs_ != nullptr; }
    [[nodiscard]] const PrefetchStats& prefetch_stats() const { return prefetch_stats_; }
    // White-box test hooks: the state of `key`'s live ticket (1 Queued,
    // 2 Running, 3 Done; -1 if none), the number of Free tickets, and the
    // pool body currently holding `key` (invalid if not resident).
    [[nodiscard]] int prefetch_ticket_state(TileKey key) const;
    [[nodiscard]] std::size_t prefetch_free_ticket_count() const;
    [[nodiscard]] BodyId resident_body(TileKey key) const;

private:
    struct InterestPoint {
        Vec3 world_xy;
        real radius_m = 0.0;
    };

    // TileKey::packed -> slots_ index, as a vector kept sorted by key.
    // Holds at most one entry per ASSIGNED slot, so reserve(pool size) at
    // construction makes every insert/erase allocation-free for the
    // manager's whole life - the unordered_map it replaced allocated a node
    // on every slot (re)assignment, i.e. on every tile-boundary crossing.
    // O(log n) lookup, O(n) insert/erase (a memmove over at most the pool
    // size, a few hundred 16-byte entries). Never iterated to decide
    // anything, although iteration would be in key order anyway.
    class KeySlotIndex {
    public:
        void reserve(std::size_t n) { entries_.reserve(n); }
        [[nodiscard]] std::size_t size() const { return entries_.size(); }
        // slots_ index for `packed`, or kAbsent.
        [[nodiscard]] std::size_t find(std::int64_t packed) const {
            const auto it = lower(packed);
            return (it != entries_.end() && it->first == packed) ? it->second : kAbsent;
        }
        [[nodiscard]] bool contains(std::int64_t packed) const { return find(packed) != kAbsent; }
        void insert_or_assign(std::int64_t packed, std::size_t slot) {
            const auto it = lower(packed);
            if (it != entries_.end() && it->first == packed) {
                entries_[static_cast<std::size_t>(it - entries_.begin())].second = slot;
            } else {
                entries_.insert(it, {packed, slot});
            }
        }
        void erase(std::int64_t packed) {
            const auto it = lower(packed);
            if (it != entries_.end() && it->first == packed) entries_.erase(it);
        }
        static constexpr std::size_t kAbsent = static_cast<std::size_t>(-1);

    private:
        using Entry = std::pair<std::int64_t, std::size_t>;
        [[nodiscard]] std::vector<Entry>::const_iterator lower(std::int64_t packed) const {
            return std::lower_bound(entries_.begin(), entries_.end(), packed,
                                    [](const Entry& e, std::int64_t k) { return e.first < k; });
        }
        std::vector<Entry> entries_;
    };

    struct Slot {
        bool ever_assigned = false; // false until this slot's body exists with real content
        bool assigned = false;      // true while `key` is currently needed
        TileKey key{};
        BodyId body{};
        TileSample sample; // preallocated once (ensure_slot_storage), never resized after
        // Per-tile vertical datum (R1) currently baked into `sample.heights`
        // (subtracted) and `body`'s own pose z (added back) - see
        // tile_manager.cpp's update(). 0.0 for every slot until first filled.
        real datum = 0.0;
    };

    void ensure_slot_storage(Slot& slot);
    // Returns the slot index newly holding `key` (reusing a free or stale
    // slot), or SIZE_MAX if the pool has no slot to give it.
    std::size_t acquire_slot_for(TileKey key);

    // PLAN.md M1 / PHYS-021 (see TerrainConfig::suppress_terrain_under_static_geometry's
    // doc comment): rewrites any exactly-height-0 sample of `slot.sample`
    // that another STATIC body already covers to ps::kHeightfieldNoCollision,
    // in place, right before the tile's content is handed to the backend.
    // Called once per newly-filled tile (never for an already-resident
    // one), from update()'s step 5, before update_heightfield_body - see
    // this .cpp file's own header comment for the cost fix (a per-TileKey
    // cache and a coarse-to-fine probe) and for two alternatives that were
    // tried and measured not to help on this workload.
    void suppress_covered_samples(Slot& slot);
    // update()'s step 5 for one synchronously filled slot: suppression,
    // datum, update_heightfield_body, set_pose.
    void install_sync_slot(Slot& slot);
    // update()'s tail (after steps 1-3) when the prefetch ring is on.
    void update_with_prefetch(jobs::JobSystem& jobs);
    // `in` without repeated slot indices, first-occurrence order.
    static void unique_slots_into(const std::vector<std::size_t>& in, std::vector<std::size_t>& out);

    // Per-caller scratch for compute_suppression_holes: the lazy path owns
    // one (lazy_scratch_, reused forever); the precompute gives each
    // parallel batch its own.
    struct SuppressionScratch {
        std::vector<ShapeContactHit> hits;
        std::vector<std::uint8_t> quad_covered;
        std::vector<std::uint8_t> vertex_state;
    };
    // The suppression computation itself (see tile_manager.cpp's header
    // comment on suppress_covered_samples): appends to `holes` (cleared
    // first) the sample indices of `sample` - a tile's UNMODIFIED
    // ITerrainSource::fill_tile output - to mark kHeightfieldNoCollision, in
    // ascending index order. Reads only backend queries, `sample` and
    // `scratch`; safe to run concurrently with distinct scratch while the
    // backend is not stepping and no body is created/destroyed/moved.
    void compute_suppression_holes(const TileSample& sample, SuppressionScratch& scratch, std::vector<std::uint32_t>& holes) const;
    // A TileSample sized for this source (heights/cell ids allocated).
    [[nodiscard]] TileSample make_sample_storage() const;
    // Pure per-tile datum computation (R1, PLAN.md M1 follow-up; extracted
    // from update()'s step 5 unchanged - terrain-prefetch-ring plan commit
    // C1, no behaviour change). Scans `sample.heights` (already suppression-
    // holed), picks the datum (0 for an all-holes tile, else the mid-range
    // snapped to kTileDatumStepM), subtracts it from every non-hole sample
    // IN PLACE (skipped entirely when the datum is exactly 0.0 - see
    // update()'s own comment for why that keeps every pre-existing
    // scenario's upload bit-for-bit unchanged), and increments
    // *relief_overflow_count when this tile's own relief alone exceeds the
    // placeholder's quantisation range. Reads/writes only `sample` and the
    // counter it is given - no backend/member-state access - so a future
    // caller (the prefetch ring's background lane) can run this on a
    // fill_tile'd sample that is not yet attached to any pool slot, exactly
    // as update() itself now does inline. Returns the computed datum.
    [[nodiscard]] static real compute_tile_datum(TileSample& sample, std::size_t* relief_overflow_count);
    // True for any BodyId belonging to THIS pool - every slot's body,
    // whether currently assigned to a real tile or still an unused
    // placeholder. Found the hard way (PHYS-021): an unassigned slot's body
    // used to sit at its construction-time default pose (world origin)
    // until first assigned a real tile, so a probe for a tile NEAR the
    // world origin (exactly where a scenario's main spawn usually sits)
    // could otherwise see dozens of stacked, content-irrelevant sibling
    // tile bodies and mistake them for a real static feature -
    // suppress_covered_samples must only ever suppress terrain in favour of
    // FOREIGN static geometry, never a sibling tile. tile_manager.cpp's
    // constructor now additionally PARKS an unassigned slot's body far from
    // world origin (kUnassignedSlotParkingZ) - a second, independent fix for
    // the same root observation, since the stacked-siblings hazard turned
    // out to be a real GAMEPLAY collision hazard (a bottomless hole exactly
    // at world origin, not just a suppression-probe misclassification risk)
    // - but this function still guards the classification regardless of
    // where an unassigned body happens to sit, so it is kept as-is.
    [[nodiscard]] bool is_own_pool_body(BodyId id) const;

    // The placeholder heightfield every pool body (and every prefetch spare)
    // is created from - see kPlaceholderHeightRangeM in tile_manager.cpp.
    [[nodiscard]] HeightfieldShape make_placeholder_shape() const;

    // Terrain prefetch ring (see start_prefetch's doc comment).
    enum TicketState : std::uint8_t { kTicketFree = 0, kTicketQueued = 1, kTicketRunning = 2, kTicketDone = 3, kTicketCancelled = 4 };
    struct PrefetchTicket {
        // Free -> Queued (tick, then submit) -> Running (lane CAS) -> Done
        // (lane) -> Free (tick, after install or cancel). Queued ->
        // Cancelled (tick CAS) -> Free (the lane, when it pops the job).
        // The tick only ever writes the fields below while the ticket is
        // Free (inputs: also right after its own Queued->Cancelled CAS,
        // after which the lane never reads it); the lane writes the result
        // fields only while Running. The release/acquire pairs on `state`
        // publish them.
        std::atomic<std::uint8_t> state{kTicketFree};
        TileKey key{};
        HeightfieldSpareId spare{};                      // owned for this TileManager's lifetime
        std::shared_ptr<const void> inputs;              // Snapshot sources only; tick thread only
        bool snapshot = false;                           // fill via fill_tile_from(inputs)
        const std::vector<std::uint32_t>* holes = nullptr; // into suppression_cache_; nullptr = none
        // Results (lane, published by Done).
        Vec3 origin{};
        real datum = 0.0;
        bool relief_overflow = false;
    };
    struct PrefetchBuilder {
        std::atomic<bool> busy{false};
        TileSample sample;
        std::unique_ptr<IHeightfieldBuildContext> ctx;
    };
    static constexpr std::size_t kNoTicket = static_cast<std::size_t>(-1);
    // Lane side of one ticket's build.
    void run_prefetch_job(std::size_t ticket_index);
    // Index of the Queued/Running/Done ticket for `key`, or kNoTicket.
    [[nodiscard]] std::size_t find_live_ticket(TileKey key) const;
    [[nodiscard]] bool key_in_ring_or_pending(TileKey key) const;
    // Cancels/frees Queued/Done tickets whose key left both the ring and the
    // pending-fill queue, except the ones in `claimed` (installing this tick).
    void prefetch_cancel_stale(const std::vector<std::size_t>& claimed);
    // Install-tick claim: kNoTicket -> fill synchronously; else a Done
    // ticket to install from.
    std::size_t prefetch_claim_for_install(TileKey key);
    void prefetch_enqueue();
    void release_ticket(PrefetchTicket& t);
    // Cancels every Queued ticket, drains the lane, frees every Done one.
    void prefetch_quiesce();

    std::shared_ptr<const ITerrainSource> source_;
    TerrainConfig config_;
    IRigidBackend& backend_;

    // ORDERING RULE (PLAN.md D7, vault PHYS-026): nothing that decides
    // simulation state, slot assignment or the order of backend calls may
    // iterate an unordered container - MSVC's and libstdc++'s iteration
    // orders differ, so Windows and Linux would diverge. The unordered
    // containers below are lookup-only (find/count/insert/erase/size) and
    // must stay that way; anything iterated is a vector, a std::map, or is
    // sorted first. key_to_slot_ is a sorted vector (KeySlotIndex), not an
    // unordered_map, for allocation-freedom rather than ordering; it is
    // lookup-only all the same.
    std::vector<Slot> slots_;
    KeySlotIndex key_to_slot_; // TileKey::packed -> slots_ index (lookup only), reserved to the pool size
    std::vector<std::size_t> free_slots_;                       // never-yet-assigned slot indices
    // Every slot's BodyId (index, generation), packed - fixed for this
    // TileManager's whole lifetime (see is_own_pool_body's doc comment).
    // Built once in the constructor, right after every slot's body exists.
    std::unordered_set<std::uint64_t> own_pool_body_ids_; // lookup only

    // Iterated every update() to build the needed-key list, which decides
    // fill-queue order and therefore which pool slot (BodyId) gets which
    // tile: ordered by interest-point id, identical on every platform.
    // Updating an existing id (e.g. a vehicle's point every tick) does not
    // allocate; only adding a new id does (setup time).
    std::map<std::uint64_t, InterestPoint> interest_points_;

    std::size_t resident_count_ = 0;
    std::size_t last_starved_count_ = 0;
    std::size_t relief_overflow_count_ = 0;

    // Scratch reused across update() calls (PLAN.md D8: no per-tick
    // allocation once warmed up) - the set of keys needed this tick and the
    // subset of those that are newly-needed (not already resident), capped
    // per tick by max_tile_fills_per_tick and carried over across ticks so
    // a burst spreads out instead of being dropped.
    std::vector<TileKey> needed_keys_scratch_;
    std::vector<TileKey> pending_fill_queue_;
    std::vector<std::size_t> this_tick_slots_scratch_;

    // suppress_covered_samples' own per-TileKey cache (PLAN.md M1/PHYS-021
    // tile-suppression cost fix): the world's static geometry never changes
    // at runtime, so a TileKey's suppression result (which sample indices
    // get holed) is a pure function of it and is computed at most once per
    // key for this TileManager's whole lifetime, however many times that
    // key's tile is streamed in and back out again. Populated only from the
    // SEQUENTIAL part of update() (step 5) - never touched from inside
    // jobs.parallel_batches - so an ordinary std::unordered_map is safe with
    // no lock. A tile with nothing to suppress is cached too (an empty
    // vector), so a revisit skips even the cheap level-1 broad probe.
    // Also filled at load time by precompute_suppression (PHYS-021 Task B),
    // so a precomputed tile's first fill is already a hit.
    std::unordered_map<std::int64_t, std::vector<std::uint32_t>> suppression_cache_; // lookup only
    std::vector<TileKey> precomputed_keys_; // ascending by TileKey::packed
    std::size_t lazy_suppression_count_ = 0;
    // The lazy path's scratch - TileManager-wide (not per-slot):
    // suppress_covered_samples only ever runs from update()'s sequential
    // step 5, so there is exactly one call in flight at a time. Every
    // buffer grows once to the largest tile seen and is reused forever
    // after. vertex_state is the per-VERTEX probe cache for the 5-point
    // "fully covered" test (each interior vertex is shared by up to 4
    // quads; reset per call) and is tri-state because "never probed"
    // (height != 0) must differ from "probed, not covered".
    SuppressionScratch lazy_scratch_;
    std::vector<std::uint32_t> lazy_holes_scratch_;
    // True once precompute_suppression ran with suppression + precompute on
    // and no static geometry changed since (see start_prefetch's suppression
    // rule).
    bool precompute_complete_ = false;

    // Terrain prefetch ring state (all empty/null with prefetch off).
    jobs::JobSystem* lane_jobs_ = nullptr;
    std::unique_ptr<PrefetchTicket[]> tickets_;
    std::size_t ticket_count_ = 0;
    std::unique_ptr<PrefetchBuilder[]> builders_;
    std::size_t builder_count_ = 0;
    PrefetchStats prefetch_stats_;
    std::vector<TileKey> ring_keys_scratch_;        // step 1's order (interest id, dy, dx), margin-widened
    std::vector<std::size_t> sync_slots_scratch_;   // this tick's slots filled synchronously
    std::vector<std::size_t> slot_ticket_scratch_;  // per this_tick_slots_scratch_ entry: ticket or kNoTicket
};

} // namespace ps::terrain
