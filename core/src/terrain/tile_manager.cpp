#include "tile_manager.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <utility>

namespace ps::terrain {

namespace {
// A pool slot's Jolt HeightFieldShape is created ONCE (this constructor)
// and its samples are mutated in place forever after via
// IRigidBackend::update_heightfield_body (JPH::HeightFieldShape::
// SetHeights) - never recreated. JoltPhysics's own HeightFieldShapeSettings
// ::Create() bakes in a FIXED 16-bit quantization scale/offset derived from
// the height data's min/max range AT CONSTRUCTION TIME (see
// HeightFieldShapeSettings::DetermineMinAndMaxSample,
// Jolt/Physics/Collision/Shape/HeightFieldShape.cpp) - SetHeights later
// only rewrites the quantized SAMPLES using that already-fixed scale, it
// does not re-derive it. A perfectly flat (all-equal-height) placeholder
// therefore bakes in a DEGENERATE range: Jolt's own DetermineMinAndMaxSample
// falls back to a 1e-6 epsilon height difference to avoid dividing by zero,
// producing a quantization scale about 1e5 times finer than intended. Any
// REAL terrain height set afterward via SetHeights (even an ordinary few
// metres) then divides by that near-zero scale and overflows `int` on the
// unclamped cast at HeightFieldShape.cpp's per-block min/max update - UB,
// caught by UBSan (linux-asan-ubsan) as "outside the range of representable
// values of type 'int'"; in a non-sanitized build this silently produces
// garbage-but-clamped collision geometry instead of a crash, so this is a
// real correctness bug, not merely a sanitizer nuisance.
//
// Fix: seed the placeholder with two sentinel corner heights spanning a
// generous, fixed envelope (kPlaceholderHeightRangeM below) instead of a
// literal flat 0, so Jolt's construction-time quantization is fitted to a
// SANE, non-degenerate range from the start - one that then stays valid for
// any real tile content this slot is ever assigned over its lifetime (see
// this file's header comment on the zero-alloc/no-shape-rebuild design).
// +-200 m is comfortably wider than any single tile's local relief this
// project's terrain sources produce (data/tracks/test_ground's real spec's
// steepest area varies by roughly 14 m over its ENTIRE ~13 km extent, not
// per 256 m tile); the resulting quantization step is
// 400 m / 65535 =~ 6 mm, well under every existing resting-contact
// acceptance test's <1 cm penetration bound. If a future terrain source
// ever needs more local relief than this within one tile, this constant is
// the one place to revisit.
constexpr real kPlaceholderHeightRangeM = 200.0;

// R1 (real-world terrain, 90-880 m ASL fed through ITerrainSource): a tile's
// own datum (see update()'s comment below) is the tile's own absolute
// mid-height snapped to the nearest multiple of this. 128 m is arbitrary
// but convenient (a power of two, so the snap is exact in binary floating
// point) and comfortably coarser than any real tile's local relief at this
// project's sample spacing, so neighbouring tiles usually share one datum.
constexpr real kTileDatumStepM = 128.0;

// relief_overflow_count() fires once a tile's own POST-DATUM relief (see
// update()) gets within this margin of kPlaceholderHeightRangeM - not only
// once it actually exceeds it. JoltPhysics' own HeightFieldShape::
// SetHeights silently CLAMPS any value outside [GetMinHeightValue(),
// GetMaxHeightValue()] (HeightFieldShape.h's own doc comment) rather than
// erroring, so a tile sitting exactly at the edge is one float rounding
// away from a silent clamp; this margin catches it before that happens
// rather than only after.
constexpr real kReliefOverflowMarginM = 2.0;

// PLAN.md M1 / PHYS-021 (owner-reported "fell through the floor", second of
// two bugs this fix addresses - see this file's git history/PR description
// for the full investigation): every one of this pool's bodies is created
// UP FRONT (this constructor), and is_own_pool_body's own doc comment
// already recorded that an unassigned slot's body "is never repositioned
// away from its construction-time default pose ... until it is first
// assigned a real tile" - but that comment only ever protected
// suppress_covered_samples' own classification (a never-assigned body must
// not be mistaken for a real static feature there). It did NOT stop that
// body from being a REAL, live Jolt collider sitting at whatever pose
// BodyDesc::pose defaulted to. That default is world (0,0,0) - and the
// placeholder heightfield's own corner sample (index 0, see
// kPlaceholderHeightRangeM above) is deliberately non-degenerate, i.e. it
// sits at exactly -200 m directly under that same point. Reproduced
// directly, before this fix, with the interest point kept away from world
// origin so most of the 512-slot pool stays unassigned (this project's own
// demo config): a ray straight down at world (0,0,0) itself misses
// entirely (the -200 m corner sample sits outside a +-50 m query's range) -
// but a ray at as little as 2 m away hits a REAL surface at z=-3.50 m,
// climbing back to z=0.00 by about 6.5 m out (the placeholder's own bilinear
// interpolation between its one extreme corner and its otherwise-flat
// interior) - i.e. NOT simply empty space the way any other never-streamed
// area of the world honestly is, but a hidden, out-of-place canyon exactly
// at world origin with a seemingly normal z=0 "floor" immediately
// surrounding it: a body could ride along completely normally and then, a
// couple of metres later with no visible cause, find nothing underneath it -
// exactly the owner's own "I fell through the floor" report. A genuine,
// standing hazard for the ENTIRE session wherever a slot happens to never
// be touched, not just at startup: only the ONE tile whose real content
// happens to cover that spot is ever repositioned there; every OTHER
// still-unassigned slot (typically most of the pool - only ~49-98 of this
// project's own 512-slot demo config are ever resident at once) stays
// stacked at world origin regardless of how long the session runs, and
// PLAN.md's own README says world origin is deliberately "the main spawn
// point, at the centre of terrain.json's main flat disc" - i.e. exactly
// where a player is most likely to end up.
//
// Fix: park an unassigned slot's body far below any reachable content
// instead of leaving it at the pose default. A real position (still a live
// static body, nothing here disables collision or changes any other
// behaviour), just one nothing in this project's world (README: "100 x
// 100 km (+-50 km)", terrain amplitude a few metres, feature heights
// modest) can ever reach - set_pose moves the body to its real tile's
// position the moment it is actually assigned (update()'s step 5), exactly
// as before; this constant only matters for however long a slot stays
// unassigned, which can be the bulk of a long session for most of a large
// pool.
constexpr real kUnassignedSlotParkingZ = -1.0e6;
} // namespace

TileManager::TileManager(std::shared_ptr<const ITerrainSource> source, const TerrainConfig& config, IRigidBackend& backend)
    : source_(std::move(source)), config_(config), backend_(backend) {
    const std::uint32_t n = source_->samples_per_tile();
    assert(n % 2 == 0 && n >= 2 && "ITerrainSource::samples_per_tile() must be even (see ps::HeightfieldShape's own doc comment)");

    slots_.resize(config_.max_resident_tiles);
    free_slots_.reserve(config_.max_resident_tiles);

    const real spacing = source_->tile_size_m() / static_cast<real>(n - 1);
    for (std::size_t i = 0; i < slots_.size(); ++i) {
        Slot& slot = slots_[i];
        ensure_slot_storage(slot);

        HeightfieldShape hf;
        hf.origin = Vec3::zero(); // repositioned per tile via backend_.set_pose, never via the shape's own origin
        hf.sample_spacing = spacing;
        hf.sample_count = n;
        // Non-degenerate placeholder range (see the namespace-scope comment
        // above) - not the real content, which every slot gets via
        // update_heightfield_body before it is ever positioned somewhere a
        // body could reach it (a never-assigned slot sits parked at
        // kUnassignedSlotParkingZ, far below anything reachable, until
        // then - see that constant's own comment for why the ORIGIN default
        // this project used before was a real, reproduced hazard, not just
        // a suppression-probe classification concern).
        hf.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
        hf.heights.front() = -kPlaceholderHeightRangeM;
        hf.heights.back() = kPlaceholderHeightRangeM;
        hf.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), kInvalidSurfaceId);

        BodyDesc desc;
        desc.shape = hf;
        desc.motion = BodyMotionType::Static;
        desc.friction = 0.8; // overridden per-cell by the tyre/contact code's own surface lookup later (PLAN.md section 6)
        desc.pose.position = Vec3{0.0, 0.0, kUnassignedSlotParkingZ}; // see kUnassignedSlotParkingZ's own comment above
        slot.body = backend_.create_body(desc);
        free_slots_.push_back(i);
        own_pool_body_ids_.insert((static_cast<std::uint64_t>(slot.body.generation) << 32) | slot.body.index);
    }
}

TileManager::~TileManager() {
    for (Slot& slot : slots_) {
        if (slot.body.valid()) backend_.destroy_body(slot.body);
    }
}

bool TileManager::is_own_pool_body(BodyId id) const {
    return own_pool_body_ids_.count((static_cast<std::uint64_t>(id.generation) << 32) | id.index) != 0;
}

void TileManager::ensure_slot_storage(Slot& slot) {
    const std::size_t n = source_->samples_per_tile();
    slot.sample.heights.assign(n * n, 0.0);
    slot.sample.cell_surface_ids.assign((n - 1) * (n - 1), kInvalidSurfaceId);
    slot.sample.samples_per_tile = static_cast<std::uint32_t>(n);
}

void TileManager::set_interest_point(std::uint64_t id, const Vec3& world_xy, real stream_radius_m) {
    interest_points_[id] = InterestPoint{world_xy, stream_radius_m};
}

void TileManager::remove_interest_point(std::uint64_t id) { interest_points_.erase(id); }

std::size_t TileManager::acquire_slot_for([[maybe_unused]] TileKey key) {
    if (!free_slots_.empty()) {
        const std::size_t idx = free_slots_.back();
        free_slots_.pop_back();
        return idx;
    }
    // No never-used slot left - look for a STALE one (currently assigned to
    // a key no interest point needs this tick) to repurpose. Linear scan:
    // pool sizes here (hundreds, not millions) make this cheap relative to
    // the tile fill itself, and keeps this bookkeeping free of a hash-set's
    // per-insertion node allocation (see tile_manager.h's header comment on
    // this file's zero-allocation scope).
    for (std::size_t i = 0; i < slots_.size(); ++i) {
        if (!slots_[i].assigned) continue;
        bool still_needed = false;
        for (const TileKey& k : needed_keys_scratch_) {
            if (k.packed == slots_[i].key.packed) {
                still_needed = true;
                break;
            }
        }
        if (!still_needed) {
            key_to_slot_.erase(slots_[i].key.packed);
            return i;
        }
    }
    return static_cast<std::size_t>(-1); // pool saturated with still-needed tiles - starved
}

void TileManager::update(jobs::JobSystem& jobs) {
    // 1. This tick's needed key set (PLAN.md M1: streamed around active
    // vehicles/camera - one square of tiles per interest point).
    needed_keys_scratch_.clear();
    const real ts = source_->tile_size_m();
    for (const auto& [id, ip] : interest_points_) {
        const TileKey center = source_->key_for_world_xy(ip.world_xy.x, ip.world_xy.y);
        const auto radius_tiles = static_cast<std::int32_t>(std::ceil(ip.radius_m / ts));
        for (std::int32_t dy = -radius_tiles; dy <= radius_tiles; ++dy) {
            for (std::int32_t dx = -radius_tiles; dx <= radius_tiles; ++dx) {
                needed_keys_scratch_.push_back(source_->key_offset(center, dx, dy));
            }
        }
    }

    // 2. Queue newly-needed keys that are not yet resident and not already
    // queued from an earlier tick.
    for (const TileKey& k : needed_keys_scratch_) {
        if (key_to_slot_.count(k.packed)) continue;
        bool already_queued = false;
        for (const TileKey& q : pending_fill_queue_) {
            if (q.packed == k.packed) {
                already_queued = true;
                break;
            }
        }
        if (!already_queued) pending_fill_queue_.push_back(k);
    }

    // 3. Process up to max_tile_fills_per_tick from the queue - the rest
    // (budget-limited or starved) stay queued for a later tick (PLAN.md M1:
    // "tiles stream ... without hitches" - a bounded per-tick cost, not an
    // instant catch-up).
    this_tick_slots_scratch_.clear();
    std::size_t starved_this_tick = 0;
    std::size_t processed = 0;
    std::size_t write = 0;
    for (std::size_t i = 0; i < pending_fill_queue_.size(); ++i) {
        const TileKey key = pending_fill_queue_[i];
        if (processed >= config_.max_tile_fills_per_tick) {
            pending_fill_queue_[write++] = key;
            continue;
        }
        const std::size_t slot_idx = acquire_slot_for(key);
        if (slot_idx == static_cast<std::size_t>(-1)) {
            ++starved_this_tick;
            pending_fill_queue_[write++] = key; // retry once the pool frees up
            continue;
        }
        Slot& slot = slots_[slot_idx];
        slot.key = key;
        slot.assigned = true;
        key_to_slot_[key.packed] = slot_idx;
        this_tick_slots_scratch_.push_back(slot_idx);
        ++processed;
    }
    pending_fill_queue_.resize(write);
    last_starved_count_ = starved_this_tick;
    resident_count_ = key_to_slot_.size();

    if (this_tick_slots_scratch_.empty()) return;

    // 4. Generate content (scatter: each job writes only to its own slot's
    // preallocated TileSample - PLAN.md D8; ITerrainSource::fill_tile does
    // not allocate, see terrain_source.h's contract).
    jobs.parallel_batches(this_tick_slots_scratch_.size(), [this](std::size_t i) {
        Slot& slot = slots_[this_tick_slots_scratch_[i]];
        source_->fill_tile(slot.key, slot.sample);
    });

    // 5. Ordered, sequential hand-off to the backend (PLAN.md D8's gather
    // convention) - see jolt_backend.cpp's update_heightfield_body for why
    // this is allocation-free in steady state. suppress_covered_samples
    // (PLAN.md M1/PHYS-021, see its own header comment for the cost fix and
    // for two rejected alternatives) runs here too.
    for (std::size_t idx : this_tick_slots_scratch_) {
        Slot& slot = slots_[idx];
        suppress_covered_samples(slot);

        // R1 (real-world terrain, 90-880 m ASL): ITerrainSource::fill_tile
        // always produces ABSOLUTE heights, but the pool's placeholder
        // seeding (kPlaceholderHeightRangeM above) fixes Jolt's 16-bit
        // quantisation range at +-200 m for this slot's whole lifetime -
        // real-world elevation does not fit that on its own. Fix: pick a
        // per-tile DATUM from this tile's own absolute sample range
        // (computed AFTER suppression, so a holed sample is excluded the
        // same way it is excluded from collision), snapped to a multiple of
        // kTileDatumStepM; subtract it from every non-hole sample before
        // upload and add it back via the tile body's own pose z. Collision
        // then lands at the same absolute height as before, and Jolt's
        // fixed range only ever has to cover this one tile's local relief.
        // An all-holes tile (any_non_hole false) gets datum 0 - there is no
        // real geometry left to place, and 0 is as good as anything else.
        // datum == 0.0 skips the subtraction loop entirely so a source
        // whose every tile already snaps to 0 (this project's own
        // data/tracks/test_ground and terrain_spec_small.json, both well
        // under +-64 m) uploads the EXACT SAME bytes as before this feature
        // existed - h - 0.0 is an exact no-op in IEEE 754, but skipping it
        // outright also skips the loop's own float compare/branch per
        // sample, and keeps this change bit-for-bit invisible to every
        // existing scenario's state_hash/hash-compare.
        const real datum = compute_tile_datum(slot.sample, &relief_overflow_count_);
        slot.datum = datum;

        backend_.update_heightfield_body(slot.body, slot.sample.heights, slot.sample.cell_surface_ids);
        backend_.set_pose(slot.body, Pose{Vec3{slot.sample.tile_origin_xy.x, slot.sample.tile_origin_xy.y, datum}, Quat::identity()});
        slot.ever_assigned = true;
    }
}

// Terrain-prefetch-ring plan commit C1: pure extraction of update()'s step
// 5 datum/relief computation (see tile_manager.h's own doc comment on this
// declaration) - byte-for-byte the same arithmetic, in the same order, as
// before this function existed.
real TileManager::compute_tile_datum(TileSample& sample, std::size_t* relief_overflow_count) {
    real min_h = std::numeric_limits<real>::max();
    real max_h = std::numeric_limits<real>::lowest();
    bool any_non_hole = false;
    for (const real h : sample.heights) {
        if (h == kHeightfieldNoCollision) continue;
        any_non_hole = true;
        if (h < min_h) min_h = h;
        if (h > max_h) max_h = h;
    }
    real datum = 0.0;
    if (any_non_hole) {
        const real mid = (min_h + max_h) * real(0.5);
        datum = std::round(mid / kTileDatumStepM) * kTileDatumStepM;
        const real relief = std::max(max_h - datum, datum - min_h);
        if (relief > kPlaceholderHeightRangeM - kReliefOverflowMarginM) ++(*relief_overflow_count);
    }
    if (datum != real(0.0)) {
        for (real& h : sample.heights) {
            if (h == kHeightfieldNoCollision) continue;
            h -= datum;
        }
    }
    return datum;
}

real TileManager::resident_tile_datum(TileKey key) const {
    const auto it = key_to_slot_.find(key.packed);
    return (it != key_to_slot_.end()) ? slots_[it->second].datum : real(0.0);
}

// PLAN.md M1 / PHYS-021 - see terrain_config.h's
// suppress_terrain_under_static_geometry doc comment for the "why" (the
// terrain and a static feature mesh both sitting at height 0 over the same
// footprint made a wheel's shape_cast hit either one depending on tiny
// Jolt-internal tie-breaking, flickering the reported surface/friction).
//
// MARKING RULE (owner decision, PHYS-021 follow-up): a terrain quad may
// lose collision ONLY if it is judged ENTIRELY covered by foreign static
// geometry, and (Jolt's own per-triangle dispatch, HeightFieldShape.cpp's
// ProjectOntoSurface/narrowphase walk - a triangle is dropped if any ONE of
// its 3 corners is a hole) a vertex may be marked kHeightfieldNoCollision
// only if EVERY quad touching it is itself judged fully covered. This is
// the conservative direction on purpose, chosen over the cheaper
// centre-sample-only rule this function used to use, and the two directions
// are NOT symmetric:
//   - under-drop (a covered quad kept, both terrain and mesh present) =>
//     the original PHYS-021 flicker: cosmetic-to-annoying surface/friction
//     tie-breaking, nobody leaves the world.
//   - over-drop (a partially-covered quad holed) => a REAL gap: no mesh
//     (its true edge does not reach this far) and no terrain (holed) under
//     that part of the quad. A body crossing it finds nothing at all.
// The centre-sample-only rule over-dropped: measured directly (a
// downward-ray sweep of the real test_ground skidpad ring's outer/inner
// edges at 0.5 deg / 0.05 m resolution, see PHYS-021 vault follow-up),
// 36-40% of the ring's circumference had a genuine floorless strip up to
// ~2.05 m wide (predicted analytically as half a quad's spacing,
// (256/63)/2 =~ 2.03 m at this source's samples_per_tile=64 - the two match
// to the sweep's own 0.05 m resolution) - reproducing the owner's "fell
// through the floor" report on a CURVED edge specifically (see this
// function's own comment on why a straight edge's existing regression test,
// tests/unit/test_terrain_static_overlap.cpp, could never have caught this -
// a straight edge's quad-grid phase is constant along its whole length, so
// every point on it gets the SAME residual; only a curved edge sweeps
// through every possible phase and is guaranteed to hit both). Given that
// asymmetry, the fix always resolves the per-quad granularity mismatch
// (a 4.06 m quad against an arbitrary mesh edge is ALWAYS wrong by up to a
// quad width somewhere - no binary per-quad rule avoids that) toward more
// flicker, never toward a hole a body can fall through.
//
// "Fully covered" is tested by probing a quad's 4 corners AND its centre (5
// points) and requiring all 5 to hit foreign static geometry - corners are
// shared by up to 4 quads, so each vertex's ray is cached/computed at most
// once per this function's call (SuppressionScratch::vertex_state), not up to 4x
// over; the extra cost vs. the old centre-only rule is therefore roughly
// (n^2 vertex probes + (n-1)^2 quad-centre probes) vs. the old (n-1)^2 alone -
// about 2x the level-3 ray count, still gated behind the same level-1/2
// broad/coarse probes below so it costs nothing for the overwhelming
// majority of tiles that touch no static feature at all. Be honest about
// what 5 points proves: this is an APPROXIMATION of "fully covered", not a
// proof - a sufficiently concave mesh boundary could still thread between
// the 5 samples and leave a quad wrongly marked fully covered (an over-drop
// this rule would then miss). It is conservative in the SAFE direction
// relative to the rule it replaces (which sampled only 1 of these 5 points),
// not an absolute guarantee; Task B (PLAN.md, load-time precompute) does not
// change this trade-off, only where the cost is paid.
//
// Deferred, NOT done here (owner decision - fix the dangerous direction
// first, then measure): narrowing the flicker band by biasing the
// closest-hit query so the mesh deterministically wins ties, e.g. dropping
// the boundary height check's tolerance a few cm. Left for a follow-up once
// it is clear the wider flicker footprint this rule accepts does not itself
// degrade driving.
//
// Cost fix (owner-reported: teleporting near a track feature stalled the
// sim below 100 Hz for several seconds - a fresh teleport's whole streamed
// set is newly-filled at once, and the original version of this function
// ran up to (samples_per_tile-1)^2 sequential ray casts - 3969 at this
// project's samples_per_tile=64 - for EVERY one of those tiles, entirely on
// update()'s sequential step 5). Two changes below (a, b) - the coarse-to-
// fine structure they establish is unchanged by the marking-rule fix above,
// only what level 3 computes inside a covered block changed - plus two
// alternatives that were tried, measured and
// REJECTED (c, d), kept here so a future session does not re-try them
// without first reading why they did not work:
//   a. Per-TileKey cache (suppression_cache_, owned by TileManager, read
//      here / written by update()'s step 5): the world's static geometry
//      never changes at runtime, so a TileKey's result is a pure function
//      of it, computed at most once for this TileManager's whole lifetime
//      however many times that tile streams in and back out again (a
//      player revisiting the same area, or a slot getting reused, common
//      over a long session). Does not help a fresh teleport's FIRST burst
//      (every key is novel), only repeat visits/reuse - (b) is what
//      actually cuts the first-burst cost.
//   b. Coarse-to-fine probe instead of a uniform per-quad sweep (see the
//      3-level description below) - "most tiles touching a feature are
//      touched by a small part of it" (task brief). Measured on this
//      machine, this project's own teleport-to-split_mu repro (see the
//      task's own report for the harness): the worst tick in the burst
//      dropped from ~980 ms to ~470 ms with this alone - real, but far
//      short of a full fix; see (c) for why the "small part" is often not
//      actually small at the densest feature clusters (main_disc), which
//      is exactly where a teleport is most likely to land.
//   c. REJECTED: restricting the coarse probe further using the level-1
//      broad probe's own HIT BODIES' world AABBs (a body's real triangles
//      can never be outside Jolt's own already-maintained broadphase
//      bounds for it) instead of an extra grid-probe level. This project's
//      collision meshes are grouped by SURFACE, not by feature
//      (data/tracks/README.md), so the single "asphalt" body's own AABB
//      spans nearly every paved feature in the whole world (the straight,
//      skidpad, split-mu strip, ramps, circuit, the 10 km straight, the
//      airstrip - tens of km apart) - a body-AABB restriction is a no-op
//      for exactly the tiles this fix most needs to help. The coarse GRID
//      probe (b. above) queries the real local geometry per block instead
//      of a body's overall bounds, so it stays effective regardless of how
//      a mesh is grouped - this is why (b) is a grid of collide_shape_into
//      calls, not an AABB-math shortcut.
//   d. REJECTED: running this function from INSIDE update()'s step 4
//      jobs.parallel_batches fill job instead of sequentially in step 5,
//      spreading this tick's suppression work across workers the same way
//      ITerrainSource::fill_tile already is. Implemented and measured: (a)
//      it would have been safe (every backend call here - collide_shape_into,
//      ray_cast, is_static - is documented read-only/concurrent-safe as long
//      as the backend is not concurrently stepping and no body is being
//      created/destroyed/moved, i_rigid_backend.h, true here since
//      terrain_tiles runs strictly after backend_step and every body
//      mutation happens strictly after this phase); (b) it would have been
//      bit-identical regardless of worker count (PLAN.md D7), same argument
//      as fill_tile's own existing parallel_batches use. But A/B measurement
//      on the SAME teleport-to-split_mu burst (worst tick, forcing a single
//      whole-tile "block" so only the threading change is isolated) showed
//      NO improvement (~1025 ms parallel vs. ~980 ms sequential, both a
//      single, unparallelized full sweep's worth of Jolt queries against the
//      same giant "asphalt" MeshShape) - i.e. concurrent Jolt narrowphase
//      queries against one large mesh do not scale on this workload, most
//      plausibly internal query-path contention/shared-cache traffic across
//      threads hammering the same mesh's BVH at once, not anything this
//      project's own code controls. Left sequential; a future session
//      revisiting this should re-measure rather than assume either result
//      still holds after a Jolt version bump.
//      Reconciled (Task B, below): concurrent queries DO scale - the
//      load-time precompute parallelises across tiles and goes from ~700 ms
//      on 1 worker to ~185 ms on 4-10 workers (release, 78 test_ground
//      tiles). What does not scale is the cost DISTRIBUTION: the single
//      most expensive tile alone costs ~138 ms of the ~578 ms total, so
//      wall time floors at that one tile no matter how many workers run.
//      A teleport burst is dominated by exactly such a tile, which is why
//      parallelising inside one tick showed no gain.
//   e. Load-time precompute (PHYS-021 Task B, precompute_suppression):
//      every tile touching static geometry is computed once inside
//      World::set_terrain_source, so on the tick path it is always (a).
//      The lazy path below stays as the fallback for any other tile.
//
// Three-level probe so this costs almost nothing for a tile with no static
// feature anywhere near it (the overwhelming majority, in every scenario
// except the handful of tiles actually under a track's meshed features),
// and stays cheap even for a tile that DOES touch one, as long as the
// touch is only partial (also the overwhelming majority - a 256 m tile is
// much bigger than most single features are wide):
//   1. One collide_shape_into with a box sized to the WHOLE tile footprint
//      (generous vertical extent) - if that finds no STATIC body other
//      than this tile's own pool (is_own_pool_body), there is nothing to
//      suppress anywhere in this tile and every pass below never runs.
//   2. Only once (1) found something: subdivide the tile into
//      kCoarseBlockQuads x kCoarseBlockQuads-quad blocks and, for each
//      block, one more collide_shape_into sized to just that block's own
//      footprint - a REAL local narrowphase query against the actual mesh
//      triangles near that block, correct regardless of how large the
//      overall body's own bounding box is (see b. above). A block with no
//      hit skips level 3 entirely for every quad in it.
//   3. Only for a block level 2 found covered: for every quad in it, probe
//      its 4 corners AND its centre (5 point probes) and require ALL 5 to hit
//      foreign static geometry before calling the quad "fully covered" -
//      see the marking-rule comment above for why 5 points and why every
//      one must hit. A corner probe is only ever made if that vertex's own
//      height sample is EXACTLY 0.0 (terrain.json's own documented
//      invariant - only a flat zone's interior is ever exactly 0, and a
//      feature mesh only ever sits on a flat zone; any other vertex can
//      never coincide with a feature mesh, so it is "not covered" with no
//      probe at all - vertex_covered() below). A vertex is then marked
//      ps::kHeightfieldNoCollision only if EVERY quad touching it (that
//      exists within this tile) was itself found fully covered.
void TileManager::suppress_covered_samples(Slot& slot) {
    if (!config_.suppress_terrain_under_static_geometry) return;

    // a. Cache lookup - a pure function of TileKey, see header comment. A
    // tile precomputed at load time (precompute_suppression) is always a
    // hit here; applying it is allocation-free.
    if (const auto it = suppression_cache_.find(slot.key.packed); it != suppression_cache_.end()) {
        for (const std::uint32_t idx : it->second) slot.sample.heights[idx] = kHeightfieldNoCollision;
        return;
    }

    // b. Lazy fallback (any tile not precomputed): the same computation the
    // precompute runs, then cached.
    ++lazy_suppression_count_;
    compute_suppression_holes(slot.sample, lazy_scratch_, lazy_holes_scratch_);
    for (const std::uint32_t idx : lazy_holes_scratch_) slot.sample.heights[idx] = kHeightfieldNoCollision;
    suppression_cache_.emplace(slot.key.packed, lazy_holes_scratch_);
}

void TileManager::compute_suppression_holes(const TileSample& sample, SuppressionScratch& scratch, std::vector<std::uint32_t>& holes) const {
    holes.clear();
    const std::uint32_t n = source_->samples_per_tile();
    const std::uint32_t quads = (n >= 1) ? n - 1 : 0;
    const real ts = source_->tile_size_m();
    const Vec3& origin = sample.tile_origin_xy;
    const real spacing = ts / static_cast<real>(n > 0 ? n - 1 : 1);
    constexpr real kVerticalHalfExtentM = 50.0; // features sit within a few metres of height 0, not near this

    // 1. Broad, whole-tile probe.
    const BoxShape probe_box{Vec3{ts * real(0.5), ts * real(0.5), kVerticalHalfExtentM}};
    Pose probe_pose;
    probe_pose.position = origin + Vec3{ts * real(0.5), ts * real(0.5), real(0.0)};
    backend_.collide_shape_into(probe_box, probe_pose, scratch.hits);
    bool any_other_static = false;
    for (const ShapeContactHit& hit : scratch.hits) {
        if (!is_own_pool_body(hit.body) && backend_.is_static(hit.body)) {
            any_other_static = true;
            break;
        }
    }
    if (!any_other_static) return; // nothing to suppress anywhere in this tile

    const std::size_t quad_count = static_cast<std::size_t>(quads) * quads;
    const std::size_t vertex_count = static_cast<std::size_t>(n) * n;
    if (scratch.quad_covered.size() < quad_count) scratch.quad_covered.resize(quad_count);
    std::fill(scratch.quad_covered.begin(), scratch.quad_covered.begin() + static_cast<std::ptrdiff_t>(quad_count), std::uint8_t{0});

    // Per-VERTEX probe cache for the "fully covered" test's 4 corners -
    // reset every call. Tri-state: kVertexUnknown must be distinguishable
    // from kVertexNotCovered so a vertex whose probe was never made (height
    // != 0, see vertex_covered() below) is not confused with one that WAS
    // probed and missed.
    constexpr std::uint8_t kVertexUnknown = 0;
    constexpr std::uint8_t kVertexCovered = 1;
    constexpr std::uint8_t kVertexNotCovered = 2;
    if (scratch.vertex_state.size() < vertex_count) scratch.vertex_state.resize(vertex_count);
    std::fill(scratch.vertex_state.begin(), scratch.vertex_state.begin() + static_cast<std::ptrdiff_t>(vertex_count), kVertexUnknown);

    // Point-covered probe: collide_shape_into with a small vertical box, NOT
    // ray_cast (a plain ray_cast measurably misses this project's static
    // feature MeshShapes at some interior points - vault PHYS-025 Part 3).
    // Back-face caveat (PHYS-025 Part 4): Jolt's CollideShape ignores
    // triangles whose plane lies beyond the query box's CENTRE, so this box
    // (centred at z=0) only reports surfaces at or below z=0. It detects a
    // flat z=0 feature mesh only because the centre lies exactly on that
    // plane. Every miss this causes is in the SAFE direction (not covered
    // -> no hole): a feature slightly above 0, or an elevated deck such as a
    // ramp, is never treated as covering the ground below it.
    constexpr real kPointProbeHalfExtentM = 0.05;
    auto point_covered = [&](real wx, real wy) -> bool {
        const BoxShape point_box{Vec3{kPointProbeHalfExtentM, kPointProbeHalfExtentM, kVerticalHalfExtentM}};
        Pose point_pose;
        point_pose.position = Vec3{wx, wy, real(0.0)};
        backend_.collide_shape_into(point_box, point_pose, scratch.hits);
        for (const ShapeContactHit& hit : scratch.hits) {
            if (!is_own_pool_body(hit.body) && backend_.is_static(hit.body)) return true;
        }
        return false;
    };

    // Probes (and caches) a single vertex's coverage on first use; every
    // subsequent call for the same (iy, ix) - up to 4x, once per quad that
    // shares this corner - is an array lookup.
    auto vertex_covered = [&](std::uint32_t iy, std::uint32_t ix) -> bool {
        const std::size_t idx = static_cast<std::size_t>(iy) * n + ix;
        std::uint8_t& state = scratch.vertex_state[idx];
        if (state == kVertexUnknown) {
            if (sample.heights[idx] != real(0.0)) {
                // Only a flat zone's interior is ever exactly 0.0 (see the
                // function-level comment) - a non-zero vertex can never
                // coincide with a feature mesh, so it is never covered and
                // no probe is made for it at all.
                state = kVertexNotCovered;
            } else {
                const real wx = origin.x + static_cast<real>(ix) * spacing;
                const real wy = origin.y + static_cast<real>(iy) * spacing;
                state = point_covered(wx, wy) ? kVertexCovered : kVertexNotCovered;
            }
        }
        return state == kVertexCovered;
    };

    // 2 + 3. Coarse block grid, refined to a per-quad "fully covered" probe
    // only inside a block level 2 actually found covered.
    constexpr std::uint32_t kCoarseBlockQuads = 8; // 63 quads -> 8x8 = 64 coarse blocks at this project's samples_per_tile=64; measured against 4 (tuning sweep) - 4 was slightly WORSE (more per-block collide_shape_into overhead, no extra quads skipped) at this project's own feature density near its most-covered tiles
    for (std::uint32_t by = 0; by * kCoarseBlockQuads < quads; ++by) {
        const std::uint32_t qy0 = by * kCoarseBlockQuads;
        const std::uint32_t qy1 = std::min(qy0 + kCoarseBlockQuads, quads); // exclusive
        for (std::uint32_t bx = 0; bx * kCoarseBlockQuads < quads; ++bx) {
            const std::uint32_t qx0 = bx * kCoarseBlockQuads;
            const std::uint32_t qx1 = std::min(qx0 + kCoarseBlockQuads, quads); // exclusive

            const real block_w = static_cast<real>(qx1 - qx0) * spacing;
            const real block_h = static_cast<real>(qy1 - qy0) * spacing;
            const BoxShape block_box{Vec3{block_w * real(0.5), block_h * real(0.5), kVerticalHalfExtentM}};
            Pose block_pose;
            block_pose.position =
                origin + Vec3{static_cast<real>(qx0) * spacing + block_w * real(0.5), static_cast<real>(qy0) * spacing + block_h * real(0.5), real(0.0)};
            backend_.collide_shape_into(block_box, block_pose, scratch.hits);
            bool block_covered = false;
            for (const ShapeContactHit& hit : scratch.hits) {
                if (!is_own_pool_body(hit.body) && backend_.is_static(hit.body)) {
                    block_covered = true;
                    break;
                }
            }
            if (!block_covered) continue; // level 3 never runs for this block - every quad in it stays "not covered"

            for (std::uint32_t qy = qy0; qy < qy1; ++qy) {
                for (std::uint32_t qx = qx0; qx < qx1; ++qx) {
                    const std::size_t qi = static_cast<std::size_t>(qy) * quads + qx;
                    // 4 corners first (cheap array-lookup on a cache hit,
                    // short-circuits before ever reaching the centre probe).
                    if (!vertex_covered(qy, qx) || !vertex_covered(qy, qx + 1) || !vertex_covered(qy + 1, qx) ||
                        !vertex_covered(qy + 1, qx + 1)) {
                        continue; // already false
                    }
                    const real wx = origin.x + (static_cast<real>(qx) + real(0.5)) * spacing;
                    const real wy = origin.y + (static_cast<real>(qy) + real(0.5)) * spacing;
                    scratch.quad_covered[qi] = point_covered(wx, wy) ? 1 : 0;
                }
            }
        }
    }

    // 4. A vertex becomes a hole only if EVERY quad touching it that
    // exists within this tile is fully covered (see the marking-rule
    // comment above) - a tile's own edge vertices simply have fewer
    // existing neighbours to require.
    for (std::uint32_t iy = 0; iy < n; ++iy) {
        for (std::uint32_t ix = 0; ix < n; ++ix) {
            bool all_covered = true;
            bool any_quad = false;
            for (std::int32_t dqy = -1; dqy <= 0 && all_covered; ++dqy) {
                const std::int32_t qy = static_cast<std::int32_t>(iy) + dqy;
                if (qy < 0 || qy >= static_cast<std::int32_t>(quads)) continue;
                for (std::int32_t dqx = -1; dqx <= 0; ++dqx) {
                    const std::int32_t qx = static_cast<std::int32_t>(ix) + dqx;
                    if (qx < 0 || qx >= static_cast<std::int32_t>(quads)) continue;
                    any_quad = true;
                    if (scratch.quad_covered[static_cast<std::size_t>(qy) * quads + static_cast<std::size_t>(qx)] == 0) {
                        all_covered = false;
                        break;
                    }
                }
            }
            if (any_quad && all_covered) holes.push_back(static_cast<std::uint32_t>(static_cast<std::size_t>(iy) * n + ix));
        }
    }
}

TileSample TileManager::make_sample_storage() const {
    const std::size_t n = source_->samples_per_tile();
    TileSample s;
    s.heights.assign(n * n, 0.0);
    s.cell_surface_ids.assign((n - 1) * (n - 1), kInvalidSurfaceId);
    s.samples_per_tile = static_cast<std::uint32_t>(n);
    return s;
}

// PHYS-021 Task B - see tile_manager.h. Tiles are enumerated from the
// actual triangles of every foreign STATIC body (a surface-grouped mesh
// body's own AABB spans nearly the whole world, so it cannot be used),
// each triangle's XY bounds grown by kBinMarginM so a triangle lying
// exactly on a tile boundary still bins into both neighbours. Precomputing
// an extra tile is harmless (same function, same result); missing one
// would only mean that tile takes the lazy path.
void TileManager::precompute_suppression(jobs::JobSystem& jobs) {
    if (!config_.suppress_terrain_under_static_geometry || !config_.precompute_suppression_at_load) return;

    constexpr real kBinMarginM = 1.0;
    const real ts = source_->tile_size_m();
    std::vector<BodyId> ids;
    backend_.live_body_ids_into(ids);
    std::vector<XyBounds> tris;
    std::unordered_map<std::int64_t, TileKey> keys;
    for (const BodyId id : ids) {
        if (is_own_pool_body(id) || !backend_.is_static(id)) continue;
        backend_.body_triangle_xy_bounds_into(id, tris);
        for (const XyBounds& b : tris) {
            const TileKey k0 = source_->key_for_world_xy(b.min_x - kBinMarginM, b.min_y - kBinMarginM);
            const Vec3 o0 = source_->tile_origin_xy(k0);
            const auto nx = static_cast<std::int32_t>(std::floor((b.max_x + kBinMarginM - o0.x) / ts));
            const auto ny = static_cast<std::int32_t>(std::floor((b.max_y + kBinMarginM - o0.y) / ts));
            for (std::int32_t dy = 0; dy <= ny; ++dy) {
                for (std::int32_t dx = 0; dx <= nx; ++dx) {
                    const TileKey k = source_->key_offset(k0, dx, dy);
                    keys.emplace(k.packed, k);
                }
            }
        }
    }

    // `keys` is iterated in hash order, but only to collect; the sort below
    // (by the unique TileKey::packed) fixes the order before anything
    // depends on it (ordering rule, tile_manager.h / vault PHYS-026).
    std::vector<TileKey> todo;
    todo.reserve(keys.size());
    for (const auto& [packed, k] : keys) {
        if (!suppression_cache_.count(packed)) todo.push_back(k);
    }
    std::sort(todo.begin(), todo.end(), [](const TileKey& a, const TileKey& b) { return a.packed < b.packed; });

    // Parallel across tiles; each batch owns its scratch and sample buffer
    // and writes only its own result slots (PLAN.md D8 scatter). The
    // backend is idle here (setup time, nothing stepping), which is the
    // documented condition for concurrent read-only queries.
    std::vector<std::vector<std::uint32_t>> results(todo.size());
    jobs.parallel_for(todo.size(), 1, [&](std::size_t begin, std::size_t end) {
        SuppressionScratch scratch;
        TileSample sample = make_sample_storage();
        for (std::size_t i = begin; i < end; ++i) {
            source_->fill_tile(todo[i], sample);
            compute_suppression_holes(sample, scratch, results[i]);
        }
    });

    // Sequential, ascending-key gather into the cache.
    for (std::size_t i = 0; i < todo.size(); ++i) {
        suppression_cache_.emplace(todo[i].packed, std::move(results[i]));
        precomputed_keys_.push_back(todo[i]);
    }
}

void TileManager::on_static_geometry_changed() {
    suppression_cache_.clear();
    precomputed_keys_.clear();
}

const std::vector<std::uint32_t>* TileManager::cached_suppression(TileKey key) const {
    const auto it = suppression_cache_.find(key.packed);
    return it == suppression_cache_.end() ? nullptr : &it->second;
}

std::vector<std::uint32_t> TileManager::compute_suppression_uncached(TileKey key) const {
    SuppressionScratch scratch;
    TileSample sample = make_sample_storage();
    source_->fill_tile(key, sample);
    std::vector<std::uint32_t> holes;
    compute_suppression_holes(sample, scratch, holes);
    return holes;
}

} // namespace ps::terrain
