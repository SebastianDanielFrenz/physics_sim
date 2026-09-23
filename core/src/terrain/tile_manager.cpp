#include "tile_manager.h"

#include <cassert>
#include <cmath>

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
        // body could reach it (World::stage_terrain_tiles runs before any
        // dynamic body's first tick can matter, and a never-assigned slot
        // sits inert at the origin exactly as it always has).
        hf.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
        hf.heights.front() = -kPlaceholderHeightRangeM;
        hf.heights.back() = kPlaceholderHeightRangeM;
        hf.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), kInvalidSurfaceId);

        BodyDesc desc;
        desc.shape = hf;
        desc.motion = BodyMotionType::Static;
        desc.friction = 0.8; // overridden per-cell by the tyre/contact code's own surface lookup later (PLAN.md section 6)
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
    // this is allocation-free in steady state.
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
// Two-level probe so this costs almost nothing for a tile with no static
// feature anywhere near it (the overwhelming majority, in every scenario
// except the handful of tiles actually under a track's meshed features):
//   1. One collide_shape_into with a box sized to the WHOLE tile footprint
//      (generous vertical extent) - if that finds no STATIC body other
//      than this tile's own pool (is_own_pool_body), there is nothing to
//      suppress anywhere in this tile and the fine pass below never runs.
//   2. Only once (1) found something: one ray straight down through every
//      QUAD's centre (not its corners - see below) whose 4 corner samples
//      are all EXACTLY height 0.0 (terrain.json's own documented invariant
//      - only a flat zone's interior is ever exactly 0, and a feature mesh
//      only ever sits on a flat zone). A vertex is then marked
//      ps::kHeightfieldNoCollision only if EVERY quad touching it (that
//      exists within this tile) was itself found covered.
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
// coincident-collision flicker, never the whole footprint.
void TileManager::suppress_covered_samples(Slot& slot) {
    if (!config_.suppress_terrain_under_static_geometry) return;

    const std::uint32_t n = source_->samples_per_tile();
    const std::uint32_t quads = (n >= 1) ? n - 1 : 0;
    const real ts = source_->tile_size_m();
    const Vec3& origin = slot.sample.tile_origin_xy;

    // 1. Broad, whole-tile probe.
    constexpr real kVerticalHalfExtentM = 50.0; // features sit within a few metres of height 0, not near this
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
    if (!any_other_static) return;

    // 2. Per-quad centre probe.
    const real spacing = ts / static_cast<real>(n - 1);
    constexpr real kRayHalfLengthM = 50.0; // matches the broad probe's own vertical extent
    if (quad_covered_scratch_.size() < static_cast<std::size_t>(quads) * quads) quad_covered_scratch_.resize(static_cast<std::size_t>(quads) * quads);
    for (std::uint32_t qy = 0; qy < quads; ++qy) {
        for (std::uint32_t qx = 0; qx < quads; ++qx) {
            const std::size_t qi = static_cast<std::size_t>(qy) * quads + qx;
            const std::size_t v00 = static_cast<std::size_t>(qy) * n + qx;
            const std::size_t v11 = static_cast<std::size_t>(qy + 1) * n + (qx + 1);
            // Only a quad fully inside a flat zone's interior can possibly
            // coincide with a feature mesh (see this function's own header
            // comment) - checking two opposite corners is enough given
            // fill_tile's height field is continuous (D7's smoothstep
            // blending), never exactly 0 on one side of a quad and far from
            // 0 on the other without also being non-zero at these corners.
            if (slot.sample.heights[v00] != real(0.0) || slot.sample.heights[v11] != real(0.0)) {
                quad_covered_scratch_[qi] = false;
                continue;
            }
            const real wx = origin.x + (static_cast<real>(qx) + real(0.5)) * spacing;
            const real wy = origin.y + (static_cast<real>(qy) + real(0.5)) * spacing;
            const RayCastHit hit = backend_.ray_cast(Vec3{wx, wy, kRayHalfLengthM}, Vec3{0.0, 0.0, -1.0}, kRayHalfLengthM * real(2.0));
            quad_covered_scratch_[qi] = hit.hit && !is_own_pool_body(hit.body) && backend_.is_static(hit.body);
        }
    }

    // 3. A vertex becomes a hole only if EVERY quad touching it that
    // exists within this tile is covered (see header comment) - a tile's
    // own edge vertices simply have fewer existing neighbours to require.
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
                slot.sample.heights[static_cast<std::size_t>(iy) * n + ix] = kHeightfieldNoCollision;
            }
        }
    }
}

} // namespace ps::terrain
