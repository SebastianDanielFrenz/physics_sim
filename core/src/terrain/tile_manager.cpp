#include "tile_manager.h"

#include <algorithm>
#include <cassert>
#include <cmath>
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
        backend_.update_heightfield_body(slot.body, slot.sample.heights, slot.sample.cell_surface_ids);
        backend_.set_pose(slot.body, Pose{slot.sample.tile_origin_xy, Quat::identity()});
        slot.ever_assigned = true;
    }
}

// PLAN.md M1 / PHYS-021 - see terrain_config.h's
// suppress_terrain_under_static_geometry doc comment for the "why" (the
// terrain and a static feature mesh both sitting at height 0 over the same
// footprint made a wheel's shape_cast hit either one depending on tiny
// Jolt-internal tie-breaking, flickering the reported surface/friction).
//
// Cost fix (owner-reported: teleporting near a track feature stalled the
// sim below 100 Hz for several seconds - a fresh teleport's whole streamed
// set is newly-filled at once, and the original version of this function
// ran up to (samples_per_tile-1)^2 sequential ray casts - 3969 at this
// project's samples_per_tile=64 - for EVERY one of those tiles, entirely on
// update()'s sequential step 5). Two changes below (a, b), both independent
// and both still producing the EXACT SAME hole set as before (see
// tests/unit/test_terrain_static_overlap.cpp, which must keep passing
// unchanged) - plus two alternatives that were tried, measured and
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
//   3. Only for a block level 2 found covered: one ray straight down
//      through every QUAD's centre in that block (not its corners - see
//      below) whose 4 corner samples are all EXACTLY height 0.0
//      (terrain.json's own documented invariant - only a flat zone's
//      interior is ever exactly 0, and a feature mesh only ever sits on a
//      flat zone). A vertex is then marked ps::kHeightfieldNoCollision only
//      if EVERY quad touching it (that exists within this tile) was itself
//      found covered.
//
// Sampling at each QUAD's centre and requiring ALL of a vertex's
// surrounding quads to be covered (an AND, not an OR) - rather than
// raycasting the vertices themselves and dropping a quad if ANY corner
// hit, the simpler thing to write - matters at the mesh's own edge: a quad
// straddling the true boundary has exactly one of its two possible states
// (covered by its centre sample, or not) and either way stays consistent
// with its neighbours' own vertices, so a hole never extends into a quad
// whose own centre sample found nothing. Marking based on corner hits
// instead over-extended the hole by up to one whole quad width on the
// OUTSIDE of a feature's leading edge (found the hard way, PHYS-021: a
// hit right at/after the straight's own start line marked that vertex,
// which - being a shared corner - also dropped the quad BEFORE the
// straight, where neither the mesh nor the now-holed terrain collided any
// more, and a body crossing there fell through). The trade-off this
// accepts: a vertex just outside the true covered region can still end up
// kept (not holed) even though its one covered neighbour quad's own
// far corner is already on the mesh - i.e. a residual sliver up to one
// quad wide right at a feature's edge can still see the original
// coincident-collision flicker, never the whole footprint. Level 2's own
// block boundaries add no further slop to this: a block is only ever used
// to SKIP a quad range (never to mark one covered by itself), so the same
// per-quad ray + per-vertex AND rule from the original single-level probe
// still decides every hole exactly as before.
void TileManager::suppress_covered_samples(Slot& slot) {
    if (!config_.suppress_terrain_under_static_geometry) return;

    // a. Cache lookup - a pure function of TileKey, see header comment.
    if (const auto it = suppression_cache_.find(slot.key.packed); it != suppression_cache_.end()) {
        for (const std::uint32_t idx : it->second) slot.sample.heights[idx] = kHeightfieldNoCollision;
        return;
    }

    const std::uint32_t n = source_->samples_per_tile();
    const std::uint32_t quads = (n >= 1) ? n - 1 : 0;
    const real ts = source_->tile_size_m();
    const Vec3& origin = slot.sample.tile_origin_xy;
    const real spacing = ts / static_cast<real>(n > 0 ? n - 1 : 1);
    constexpr real kVerticalHalfExtentM = 50.0; // features sit within a few metres of height 0, not near this
    constexpr real kRayHalfLengthM = 50.0;      // matches the broad/block probes' own vertical extent

    // 1. Broad, whole-tile probe.
    const BoxShape probe_box{Vec3{ts * real(0.5), ts * real(0.5), kVerticalHalfExtentM}};
    Pose probe_pose;
    probe_pose.position = origin + Vec3{ts * real(0.5), ts * real(0.5), real(0.0)};
    backend_.collide_shape_into(probe_box, probe_pose, coverage_probe_scratch_);
    bool any_other_static = false;
    for (const ShapeContactHit& hit : coverage_probe_scratch_) {
        if (!is_own_pool_body(hit.body) && backend_.is_static(hit.body)) {
            any_other_static = true;
            break;
        }
    }
    if (!any_other_static) {
        suppression_cache_.emplace(slot.key.packed, std::vector<std::uint32_t>{}); // cache the empty result too - skips even this broad probe on revisit
        return;
    }

    if (quad_covered_scratch_.size() < static_cast<std::size_t>(quads) * quads) quad_covered_scratch_.resize(static_cast<std::size_t>(quads) * quads);
    std::fill(quad_covered_scratch_.begin(), quad_covered_scratch_.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(quads) * quads), false);

    // 2 + 3. Coarse block grid, refined to a per-quad ray probe only inside
    // a block level 2 actually found covered.
    constexpr std::uint32_t kCoarseBlockQuads = 8; // 63 quads -> 8x8 = 64 coarse blocks at this project's samples_per_tile=64; measured against 4 (tuning sweep, tile_manager.cpp's own report/header comment) - 4 was slightly WORSE (more per-block collide_shape_into overhead, no extra quads skipped) at this project's own feature density near its most-covered tiles
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
            backend_.collide_shape_into(block_box, block_pose, coverage_probe_scratch_);
            bool block_covered = false;
            for (const ShapeContactHit& hit : coverage_probe_scratch_) {
                if (!is_own_pool_body(hit.body) && backend_.is_static(hit.body)) {
                    block_covered = true;
                    break;
                }
            }
            if (!block_covered) continue; // level 3 never runs for this block - every quad in it stays "not covered"

            for (std::uint32_t qy = qy0; qy < qy1; ++qy) {
                for (std::uint32_t qx = qx0; qx < qx1; ++qx) {
                    const std::size_t qi = static_cast<std::size_t>(qy) * quads + qx;
                    const std::size_t v00 = static_cast<std::size_t>(qy) * n + qx;
                    const std::size_t v11 = static_cast<std::size_t>(qy + 1) * n + (qx + 1);
                    // Only a quad fully inside a flat zone's interior can
                    // possibly coincide with a feature mesh (see this
                    // function's own header comment) - checking two
                    // opposite corners is enough given fill_tile's height
                    // field is continuous (D7's smoothstep blending), never
                    // exactly 0 on one side of a quad and far from 0 on the
                    // other without also being non-zero at these corners.
                    if (slot.sample.heights[v00] != real(0.0) || slot.sample.heights[v11] != real(0.0)) continue; // already false
                    const real wx = origin.x + (static_cast<real>(qx) + real(0.5)) * spacing;
                    const real wy = origin.y + (static_cast<real>(qy) + real(0.5)) * spacing;
                    const RayCastHit hit = backend_.ray_cast(Vec3{wx, wy, kRayHalfLengthM}, Vec3{0.0, 0.0, -1.0}, kRayHalfLengthM * real(2.0));
                    quad_covered_scratch_[qi] = hit.hit && !is_own_pool_body(hit.body) && backend_.is_static(hit.body);
                }
            }
        }
    }

    // 4. A vertex becomes a hole only if EVERY quad touching it that
    // exists within this tile is covered (see header comment) - a tile's
    // own edge vertices simply have fewer existing neighbours to require.
    std::vector<std::uint32_t> holes;
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
                    if (!quad_covered_scratch_[static_cast<std::size_t>(qy) * quads + static_cast<std::size_t>(qx)]) {
                        all_covered = false;
                        break;
                    }
                }
            }
            if (any_quad && all_covered) {
                const std::size_t idx = static_cast<std::size_t>(iy) * n + ix;
                slot.sample.heights[idx] = kHeightfieldNoCollision;
                holes.push_back(static_cast<std::uint32_t>(idx));
            }
        }
    }
    suppression_cache_.emplace(slot.key.packed, std::move(holes));
}

} // namespace ps::terrain
