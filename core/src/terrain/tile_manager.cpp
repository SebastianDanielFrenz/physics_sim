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
    }
}

TileManager::~TileManager() {
    for (Slot& slot : slots_) {
        if (slot.body.valid()) backend_.destroy_body(slot.body);
    }
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
        backend_.update_heightfield_body(slot.body, slot.sample.heights, slot.sample.cell_surface_ids);
        backend_.set_pose(slot.body, Pose{slot.sample.tile_origin_xy, Quat::identity()});
        slot.ever_assigned = true;
    }
}

} // namespace ps::terrain
