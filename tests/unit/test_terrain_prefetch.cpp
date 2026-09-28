// tests/unit/test_terrain_prefetch.cpp - terrain prefetch ring, commit C5:
// TileManager's background-lane tickets (TileManager::start_prefetch).
//
// The contract checked here (tile_manager.h's start_prefetch doc comment):
// prefetch ON changes WHEN a tile's content is computed, never WHAT lands
// WHERE or WHEN it is installed. Every case therefore runs a prefetch-OFF
// manager in lock-step with a prefetch-ON one (each on its own World's
// backend, so both pools get the same BodyIds) and after every update()
// compares, per tile key in a window around the interest point: the
// resident pool BodyId, the backend's heightfield digest, the body pose and
// the tile datum - plus resident/starved/relief-overflow counts. On top of
// that each case forces one ticket path and checks its counter:
//   - drained lane (every ticket Done): installed by swap;
//   - blocked lane (tickets still Queued at the install tick): late_sync;
//   - a ticket Running at the install tick: late_wait;
//   - tickets for keys that left the ring: cancelled, and every ticket is
//     Free again after the lane drains (no leak);
//   - a Snapshot source whose data changed after capture: stale_inputs;
//   - no precompute: skipped_suppression, nothing enqueued;
//   - a TickOnly source: prefetch never starts.
// A zero-allocation case (C7) drives tile-row crossings with prefetch on,
// update() inside a job and debug_alloc armed, and requires no allocation of
// our own on the tick side or in the lane builder.
// A last case uses the real test_ground static collision: a prefetched tile
// with suppression holes is bit-identical to the synchronous path, and the
// enqueue rule's premise holds (every tile the precompute did NOT cover has
// no holes).
//
// White-box (core/src on the include path): ps::terrain::TileManager is
// internal.

#include "terrain/procedural_terrain_source.h"
#include "terrain/tile_manager.h"

#include "ps/io/surface_table.h"
#include "ps/io/test_ground_loader.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/terrain/procedural_terrain.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace ps;
using namespace ps::terrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

// Same small hand-built spec as test_terrain_tile_manager.cpp: 32 m tiles.
TerrainSpec make_test_spec() {
    TerrainSpec spec;
    spec.seed = 12345;
    spec.rolling_octaves.push_back(Octave{50.0, 3.0, 1});
    spec.rolling_octaves.push_back(Octave{17.0, 0.8, 2});
    spec.hill_octave = Octave{200.0, 10.0, 5};
    spec.roughness_near_m = 1.0e9;
    spec.roughness_far_m = 2.0e9;
    spec.default_surface = "grass";
    spec.surface_rules.push_back(SurfaceRule{"dirt", 40.0, 101, 0.5});
    spec.surface_names = {"grass", "dirt"};
    spec.tile_size_m = 32.0;
    return spec;
}

// A ProceduralTerrainSource with a selectable prefetch_support(), an
// optional gate that holds fills running on `lane_owner`'s background lane,
// and (Snapshot) a mutable version added to every height - so a snapshot
// captured before a version bump is genuinely stale.
class TestSource final : public ITerrainSource {
public:
    TestSource(PrefetchSupportKind kind, std::uint32_t samples) : inner_(make_test_spec(), samples), kind_(kind) {}

    real tile_size_m() const override { return inner_.tile_size_m(); }
    std::uint32_t samples_per_tile() const override { return inner_.samples_per_tile(); }
    TileKey key_for_world_xy(real x, real y) const override { return inner_.key_for_world_xy(x, y); }
    TileKey key_offset(TileKey key, std::int32_t dx, std::int32_t dy) const override { return inner_.key_offset(key, dx, dy); }
    Vec3 tile_origin_xy(TileKey key) const override { return inner_.tile_origin_xy(key); }
    PrefetchSupportKind prefetch_support() const override { return kind_; }

    void fill_tile(TileKey key, TileSample& out) const override {
        maybe_gate(key);
        fill_versioned(key, version_.load(std::memory_order_acquire), out);
    }
    bool acquire_prefetch_inputs(TileKey /*key*/, std::shared_ptr<const void>& out) const override {
        if (kind_ != PrefetchSupportKind::Snapshot) return false;
        out = std::make_shared<const int>(version_.load(std::memory_order_acquire));
        return true;
    }
    void fill_tile_from(TileKey key, const void* inputs, TileSample& out) const override {
        maybe_gate(key);
        fill_versioned(key, *static_cast<const int*>(inputs), out);
    }
    bool prefetch_inputs_current(TileKey /*key*/, const void* inputs) const override {
        return *static_cast<const int*>(inputs) == version_.load(std::memory_order_acquire);
    }

    // Only called between updates (tick thread) or while the lane is idle.
    void bump_version() { version_.fetch_add(1, std::memory_order_acq_rel); }

    // Gate: while closed, a fill running on lane_owner's background lane
    // records its key (first one only) and blocks until opened.
    void close_gate(const jobs::JobSystem* lane_owner) {
        lane_owner_ = lane_owner;
        gate_open_.store(false, std::memory_order_release);
    }
    void open_gate() { gate_open_.store(true, std::memory_order_release); }
    bool gate_entered() const { return entered_.load(std::memory_order_acquire); }
    std::int64_t entered_key() const { return entered_key_.load(std::memory_order_acquire); }

private:
    void maybe_gate(TileKey key) const {
        if (lane_owner_ == nullptr || !lane_owner_->on_background_lane()) return;
        if (gate_open_.load(std::memory_order_acquire)) return;
        bool expected = false;
        if (entered_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            entered_key_.store(key.packed, std::memory_order_release);
        }
        while (!gate_open_.load(std::memory_order_acquire)) std::this_thread::yield();
    }
    void fill_versioned(TileKey key, int version, TileSample& out) const {
        inner_.fill_tile(key, out);
        if (version == 0) return;
        const real dh = static_cast<real>(version) * 0.5;
        for (real& h : out.heights) h += dh;
    }

    ProceduralTerrainSource inner_;
    PrefetchSupportKind kind_;
    std::atomic<int> version_{0};
    const jobs::JobSystem* lane_owner_ = nullptr;
    std::atomic<bool> gate_open_{true};
    mutable std::atomic<bool> entered_{false};
    mutable std::atomic<std::int64_t> entered_key_{0};
};

// Blocks the (single-thread) lane until release is set.
struct Latch {
    std::atomic<bool> started{false};
    std::atomic<bool> release{false};
};

void block_lane(jobs::JobSystem& js, Latch& latch) {
    Latch* l = &latch;
    REQUIRE(js.submit_background(jobs::JobFn([l] {
        l->started.store(true, std::memory_order_release);
        while (!l->release.load(std::memory_order_acquire)) std::this_thread::yield();
    })));
    while (!latch.started.load(std::memory_order_acquire)) std::this_thread::yield();
}

TerrainConfig small_config() {
    TerrainConfig tc;
    tc.max_resident_tiles = 64;
    tc.max_tile_fills_per_tick = 4;
    tc.prefetch_margin_tiles = 1;
    tc.prefetch_max_tiles = 16;
    return tc;
}

constexpr real kRadius = 40.0; // ceil(40/32) = 2 -> a 5x5 needed square, 7x7 ring with margin 1

// One prefetch-off and one prefetch-on manager, each on its own World (same
// body-creation sequence, so the same pool BodyIds), driven in lock-step.
struct Pair {
    std::shared_ptr<TestSource> source;
    World world_off{make_world_config()};
    World world_on{make_world_config()};
    jobs::JobSystem jobs_off{2};
    jobs::JobSystem jobs_on{2};
    std::unique_ptr<TileManager> off;
    std::unique_ptr<TileManager> on;

    static WorldConfig make_world_config() {
        WorldConfig cfg;
        cfg.job_workers = 1;
        return cfg;
    }

    Pair(PrefetchSupportKind kind, const TerrainConfig& tc, bool precompute = true)
        : source(std::make_shared<TestSource>(kind, 8)) {
        TerrainConfig tc_off = tc;
        tc_off.prefetch_margin_tiles = 0;
        tc_off.prefetch_max_tiles = 0;
        off = std::make_unique<TileManager>(source, tc_off, world_off.backend());
        on = std::make_unique<TileManager>(source, tc, world_on.backend());
        if (precompute) {
            off->precompute_suppression(jobs_off);
            on->precompute_suppression(jobs_on);
        }
        off->start_prefetch(jobs_off); // a no-op: prefetch disabled in tc_off
        on->start_prefetch(jobs_on);
        REQUIRE_FALSE(off->prefetch_enabled());
    }
    ~Pair() {
        // Managers before their JobSystems (the on-manager drains its lane).
        on.reset();
        off.reset();
    }

    void set_point(const Vec3& p) {
        off->set_interest_point(1, p, kRadius);
        on->set_interest_point(1, p, kRadius);
    }
    void update() {
        off->update(jobs_off);
        on->update(jobs_on);
    }

    // Everything prefetch must not change, over a window wide enough to
    // cover the needed square, the ring and some tiles beyond.
    void require_same(const Vec3& p) const {
        REQUIRE(off->resident_tile_count() == on->resident_tile_count());
        REQUIRE(off->starved_tile_count() == on->starved_tile_count());
        REQUIRE(off->relief_overflow_count() == on->relief_overflow_count());
        const TileKey c = source->key_for_world_xy(p.x, p.y);
        for (std::int32_t dy = -5; dy <= 5; ++dy) {
            for (std::int32_t dx = -5; dx <= 5; ++dx) {
                const TileKey k = source->key_offset(c, dx, dy);
                const BodyId a = off->resident_body(k);
                const BodyId b = on->resident_body(k);
                INFO("key " << k.packed << " dx " << dx << " dy " << dy);
                REQUIRE(a == b);
                if (!a.valid()) continue;
                REQUIRE(world_off.backend().debug_heightfield_digest(a) == world_on.backend().debug_heightfield_digest(b));
                const Pose pa = world_off.backend().get_pose(a);
                const Pose pb = world_on.backend().get_pose(b);
                REQUIRE(pa.position.x == pb.position.x);
                REQUIRE(pa.position.y == pb.position.y);
                REQUIRE(pa.position.z == pb.position.z);
                REQUIRE(off->resident_tile_datum(k) == on->resident_tile_datum(k));
            }
        }
    }
};

} // namespace

TEST_CASE("Prefetch never starts for a TickOnly source or with the ring configured off", "[terrain][tiles][prefetch]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    SECTION("TickOnly source, ring configured on") {
        jobs::JobSystem jobs(1);
        auto src = std::make_shared<TestSource>(PrefetchSupportKind::TickOnly, 8);
        TileManager mgr(src, small_config(), world.backend());
        mgr.precompute_suppression(jobs);
        mgr.start_prefetch(jobs);
        REQUIRE_FALSE(mgr.prefetch_enabled());
        REQUIRE_FALSE(jobs.background_lane_started());
    }
    SECTION("Immutable source, margin 0 or max 0") {
        for (int which = 0; which < 2; ++which) {
            jobs::JobSystem jobs(1);
            auto src = std::make_shared<TestSource>(PrefetchSupportKind::Immutable, 8);
            TerrainConfig tc = small_config();
            if (which == 0) tc.prefetch_margin_tiles = 0;
            else tc.prefetch_max_tiles = 0;
            TileManager mgr(src, tc, world.backend());
            mgr.precompute_suppression(jobs);
            mgr.start_prefetch(jobs);
            REQUIRE_FALSE(mgr.prefetch_enabled());
            REQUIRE_FALSE(jobs.background_lane_started());
            mgr.set_interest_point(1, Vec3{0.0, 0.0, 0.0}, kRadius);
            mgr.update(jobs);
            REQUIRE(mgr.prefetch_stats().enqueued == 0);
        }
    }
    SECTION("the procedural source reports Immutable") {
        ProceduralTerrainSource p(make_test_spec(), 8);
        REQUIRE(p.prefetch_support() == PrefetchSupportKind::Immutable);
    }
}

TEST_CASE("Prefetch on and off install the same content into the same body at the same tick", "[terrain][tiles][prefetch]") {
    for (const PrefetchSupportKind kind : {PrefetchSupportKind::Immutable, PrefetchSupportKind::Snapshot}) {
        for (const bool drain : {true, false}) {
            INFO("kind " << static_cast<int>(kind) << " drain " << drain);
            Pair pair(kind, small_config());
            REQUIRE(pair.on->prefetch_enabled());
            // Drive diagonally across several tile rows and columns, 7 m per
            // tick (a tile boundary every ~4-5 ticks), then back.
            Vec3 p{5.0, 3.0, 0.0};
            for (int tick = 0; tick < 120; ++tick) {
                p.x += (tick < 80) ? 7.0 : -9.0;
                p.y += (tick < 80) ? 3.0 : 4.0;
                pair.set_point(p);
                if (drain) pair.jobs_on.drain_background();
                pair.update();
                pair.require_same(p);
            }
            const PrefetchStats& s = pair.on->prefetch_stats();
            INFO("enqueued " << s.enqueued << " installed " << s.installed << " late_sync " << s.late_sync << " late_wait " << s.late_wait
                             << " cancelled " << s.cancelled);
            REQUIRE(s.enqueued > 0);
            if (drain) {
                REQUIRE(s.installed > 0);
                REQUIRE(s.late_sync == 0);
                REQUIRE(s.late_wait == 0);
            }
            REQUIRE(s.stale_inputs == 0);
            REQUIRE(s.skipped_suppression == 0);
            REQUIRE(pair.off->prefetch_stats().enqueued == 0);
        }
    }
}

TEST_CASE("A ticket still Queued at its install tick is cancelled and filled synchronously (late_sync)", "[terrain][tiles][prefetch]") {
    Pair pair(PrefetchSupportKind::Immutable, small_config());
    Latch latch;
    block_lane(pair.jobs_on, latch);

    // Tick 1 installs 4 of the 25 needed tiles and tickets the pending ones;
    // the lane is blocked, so at tick 2 those tickets are still Queued.
    const Vec3 p{0.0, 0.0, 0.0};
    pair.set_point(p);
    pair.update();
    pair.require_same(p);
    REQUIRE(pair.on->prefetch_stats().enqueued > 0);
    pair.update();
    pair.require_same(p);
    REQUIRE(pair.on->prefetch_stats().late_sync > 0);
    REQUIRE(pair.on->prefetch_stats().installed == 0);

    latch.release.store(true, std::memory_order_release);
    pair.jobs_on.drain_background();
    for (int tick = 0; tick < 12; ++tick) {
        pair.update();
        pair.require_same(p);
        pair.jobs_on.drain_background();
    }
    REQUIRE(pair.on->prefetch_stats().installed > 0);
}

TEST_CASE("A ticket Running at its install tick is waited for (late_wait)", "[terrain][tiles][prefetch]") {
    Pair pair(PrefetchSupportKind::Immutable, small_config());
    pair.source->close_gate(&pair.jobs_on);

    // Tick 1 tickets the pending keys, pending order first; the lane starts
    // the first one and blocks inside its fill. That key is the head of the
    // pending queue, so tick 2 installs it and must wait for the lane.
    const Vec3 p{0.0, 0.0, 0.0};
    pair.set_point(p);
    pair.update();
    pair.require_same(p);
    while (!pair.source->gate_entered()) std::this_thread::yield();
    const TileKey running{pair.source->entered_key()};
    REQUIRE(pair.on->prefetch_ticket_state(running) == 2);

    std::thread opener([&pair] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pair.source->open_gate();
    });
    pair.update();
    opener.join();
    pair.require_same(p);
    const PrefetchStats& s = pair.on->prefetch_stats();
    REQUIRE(s.late_wait == 1);
    REQUIRE(s.late_wait_ns_max > 0);
    REQUIRE(s.late_wait_ns_total >= s.late_wait_ns_max);
    REQUIRE(pair.on->resident_body(running).valid());

    for (int tick = 0; tick < 8; ++tick) {
        pair.update();
        pair.require_same(p);
    }
}

TEST_CASE("Tickets for keys that leave the ring are cancelled and every ticket comes back", "[terrain][tiles][prefetch]") {
    TerrainConfig tc = small_config();
    tc.max_tile_fills_per_tick = 64; // no pending keys: every ticket is a margin key
    Pair pair(PrefetchSupportKind::Immutable, tc);
    Latch latch;
    block_lane(pair.jobs_on, latch);

    Vec3 p{0.0, 0.0, 0.0};
    pair.set_point(p);
    pair.update();
    pair.require_same(p);
    REQUIRE(pair.on->prefetch_stats().enqueued == tc.prefetch_max_tiles);
    REQUIRE(pair.on->prefetch_free_ticket_count() == 0);

    // Teleport: every ticketed margin key leaves the ring.
    p = Vec3{10000.0, -5000.0, 0.0};
    pair.set_point(p);
    pair.update();
    pair.require_same(p);
    REQUIRE(pair.on->prefetch_stats().cancelled == tc.prefetch_max_tiles);
    REQUIRE(pair.on->prefetch_stats().installed == 0);

    pair.off->remove_interest_point(1);
    pair.on->remove_interest_point(1);
    pair.update();
    latch.release.store(true, std::memory_order_release);
    pair.jobs_on.drain_background();
    pair.update();
    REQUIRE(pair.on->prefetch_free_ticket_count() == tc.prefetch_max_tiles);
}

TEST_CASE("Destroying a prefetching TileManager with live tickets is clean", "[terrain][tiles][prefetch]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    jobs::JobSystem jobs(2);
    auto src = std::make_shared<TestSource>(PrefetchSupportKind::Snapshot, 8);
    TerrainConfig tc = small_config();

    SECTION("lane free: Queued/Running/Done tickets") {
        for (int round = 0; round < 3; ++round) {
            auto mgr = std::make_unique<TileManager>(src, tc, world.backend());
            mgr->precompute_suppression(jobs);
            mgr->start_prefetch(jobs);
            mgr->set_interest_point(1, Vec3{0.0, 0.0, 0.0}, kRadius);
            mgr->update(jobs);
            REQUIRE(mgr->prefetch_stats().enqueued > 0);
            if (round == 1) jobs.drain_background();
            mgr.reset();
        }
    }
    SECTION("lane blocked: the destructor waits for it") {
        auto mgr = std::make_unique<TileManager>(src, tc, world.backend());
        mgr->precompute_suppression(jobs);
        mgr->start_prefetch(jobs);
        Latch latch;
        block_lane(jobs, latch);
        mgr->set_interest_point(1, Vec3{0.0, 0.0, 0.0}, kRadius);
        mgr->update(jobs);
        REQUIRE(mgr->prefetch_stats().enqueued > 0);
        std::thread releaser([&latch] {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            latch.release.store(true, std::memory_order_release);
        });
        mgr.reset();
        releaser.join();
    }
    // The lane is idle and nothing left in it references a destroyed manager.
    jobs.drain_background();
}

TEST_CASE("A Snapshot ticket whose source changed is discarded and filled synchronously (stale_inputs)", "[terrain][tiles][prefetch]") {
    TerrainConfig tc = small_config();
    tc.max_tile_fills_per_tick = 64; // tick 1 installs the whole needed square; tickets go to margin keys
    Pair pair(PrefetchSupportKind::Snapshot, tc);

    Vec3 p{16.0, 16.0, 0.0};
    pair.set_point(p);
    pair.update();
    pair.require_same(p);
    pair.jobs_on.drain_background(); // every ticket Done, built from version 0
    REQUIRE(pair.on->prefetch_stats().enqueued > 0);

    pair.source->bump_version(); // the source's live data changes (both managers see it)
    p.x += 32.0;                 // one tile east: the east margin column becomes needed
    pair.set_point(p);
    pair.update();
    pair.require_same(p);
    REQUIRE(pair.on->prefetch_stats().stale_inputs > 0);
    REQUIRE(pair.on->prefetch_stats().installed == 0);

    // Fresh tickets (captured at version 1) install normally.
    pair.jobs_on.drain_background();
    p.y += 32.0;
    pair.set_point(p);
    pair.update();
    pair.require_same(p);
    REQUIRE(pair.on->prefetch_stats().installed > 0);
}

TEST_CASE("Without a complete suppression precompute nothing is enqueued (skipped_suppression)", "[terrain][tiles][prefetch]") {
    TerrainConfig tc = small_config();
    tc.precompute_suppression_at_load = false; // precompute_suppression becomes a no-op
    Pair pair(PrefetchSupportKind::Immutable, tc);
    REQUIRE(pair.on->prefetch_enabled());
    const Vec3 p{0.0, 0.0, 0.0};
    pair.set_point(p);
    for (int tick = 0; tick < 3; ++tick) {
        pair.update();
        pair.require_same(p);
    }
    REQUIRE(pair.on->prefetch_stats().enqueued == 0);
    REQUIRE(pair.on->prefetch_stats().skipped_suppression > 0);

    // With suppression off there is nothing to wait for: tickets flow.
    TerrainConfig tc2 = tc;
    tc2.suppress_terrain_under_static_geometry = false;
    Pair pair2(PrefetchSupportKind::Immutable, tc2);
    pair2.set_point(p);
    pair2.update();
    pair2.require_same(p);
    REQUIRE(pair2.on->prefetch_stats().enqueued > 0);
    REQUIRE(pair2.on->prefetch_stats().skipped_suppression == 0);
}

TEST_CASE("test_ground: a prefetched tile with suppression holes matches the synchronous path; uncovered tiles have no holes",
          "[terrain][tiles][prefetch][PHYS-021]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    const std::string track_dir = std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground";
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    io::load_test_ground_collision(world, track_dir, *surfaces);
    std::shared_ptr<const ITerrainSource> source = load_procedural_terrain_source(track_dir + "/terrain.json", 64);
    const real ts = source->tile_size_m();

    TerrainConfig tc;
    tc.max_resident_tiles = 32;
    tc.max_tile_fills_per_tick = 16;
    tc.prefetch_margin_tiles = 1;
    tc.prefetch_max_tiles = 16;
    TerrainConfig tc_off = tc;
    tc_off.prefetch_margin_tiles = 0;
    tc_off.prefetch_max_tiles = 0;
    constexpr real kR = 200.0; // ceil(200/256) = 1 -> 3x3 needed, 5x5 ring

    struct Record {
        std::uint64_t digest = 0;
        Pose pose{};
        real datum = 0.0;
    };
    jobs::JobSystem jobs(4);

    // The two pools must never be alive together: each would see the other's
    // bodies as foreign static geometry. OFF first.
    TileKey target{};
    std::vector<Vec3> path;
    std::map<std::int64_t, Record> off_records;
    {
        TileManager off(source, tc_off, world.backend());
        off.precompute_suppression(jobs);
        REQUIRE(off.precomputed_tile_count() > 0);
        bool found = false;
        for (const TileKey& k : off.precomputed_keys()) {
            const std::vector<std::uint32_t>* holes = off.cached_suppression(k);
            REQUIRE(holes != nullptr);
            if (!holes->empty()) {
                target = k;
                found = true;
                break;
            }
        }
        REQUIRE(found);

        // Exhaustiveness (the enqueue rule's premise): within two tiles of
        // every precomputed tile, a tile NOT precomputed has no holes.
        std::set<std::int64_t> pre;
        for (const TileKey& k : off.precomputed_keys()) pre.insert(k.packed);
        std::set<std::int64_t> checked;
        for (const TileKey& k : off.precomputed_keys()) {
            for (std::int32_t dy = -2; dy <= 2; ++dy) {
                for (std::int32_t dx = -2; dx <= 2; ++dx) {
                    const TileKey nb = source->key_offset(k, dx, dy);
                    if (pre.count(nb.packed) || !checked.insert(nb.packed).second) continue;
                    INFO("uncovered key " << nb.packed);
                    REQUIRE(off.compute_suppression_uncached(nb).empty());
                }
            }
        }
        REQUIRE(!checked.empty());

        // Start two tiles east of the target (target in the margin ring,
        // not needed), then move onto it.
        const Vec3 o = source->tile_origin_xy(target);
        const Vec3 at_target{o.x + 0.5 * ts, o.y + 0.5 * ts, 0.0};
        REQUIRE(source->key_for_world_xy(at_target.x, at_target.y).packed == target.packed);
        path = {Vec3{at_target.x + 2.0 * ts, at_target.y, 0.0}, Vec3{at_target.x + 2.0 * ts, at_target.y, 0.0}, at_target};
        for (const Vec3& p : path) {
            off.set_interest_point(1, p, kR);
            off.update(jobs);
        }
        for (std::int32_t dy = -2; dy <= 2; ++dy) {
            for (std::int32_t dx = -2; dx <= 2; ++dx) {
                const TileKey k = source->key_offset(target, dx, dy);
                const BodyId b = off.resident_body(k);
                if (!b.valid()) continue;
                off_records[k.packed] = Record{world.backend().debug_heightfield_digest(b), world.backend().get_pose(b), off.resident_tile_datum(k)};
            }
        }
        REQUIRE(off_records.count(target.packed) == 1);
    }

    TileManager on(source, tc, world.backend());
    on.precompute_suppression(jobs);
    on.start_prefetch(jobs);
    REQUIRE(on.prefetch_enabled());
    for (std::size_t i = 0; i < path.size(); ++i) {
        on.set_interest_point(1, path[i], kR);
        on.update(jobs);
        jobs.drain_background();
        if (i + 1 == path.size() - 1) REQUIRE(on.prefetch_ticket_state(target) == 3); // Done before its install tick
    }
    REQUIRE(on.prefetch_stats().installed > 0);
    REQUIRE(on.prefetch_stats().skipped_suppression == 0);
    std::size_t compared = 0;
    for (std::int32_t dy = -2; dy <= 2; ++dy) {
        for (std::int32_t dx = -2; dx <= 2; ++dx) {
            const TileKey k = source->key_offset(target, dx, dy);
            const BodyId b = on.resident_body(k);
            const auto it = off_records.find(k.packed);
            REQUIRE(b.valid() == (it != off_records.end()));
            if (!b.valid()) continue;
            INFO("key " << k.packed);
            REQUIRE(world.backend().debug_heightfield_digest(b) == it->second.digest);
            const Pose pose = world.backend().get_pose(b);
            REQUIRE(pose.position.x == it->second.pose.position.x);
            REQUIRE(pose.position.y == it->second.pose.position.y);
            REQUIRE(pose.position.z == it->second.pose.position.z);
            REQUIRE(on.resident_tile_datum(k) == it->second.datum);
            ++compared;
        }
    }
    REQUIRE(compared == off_records.size());
}

// C7: a tile-row crossing with prefetch on is allocation-free in our own
// code - on the tick side (update() runs inside a job, as World's
// terrain_tiles stage calls it, so debug_alloc::in_job() holds) and in the
// lane builder (lane jobs run through the same job accounting). A crossing
// reassigns pool slots (key_to_slot_ insert/erase - a sorted vector reserved
// to the pool size since C7; the unordered_map before it allocated a node
// per assignment), claims and releases tickets, enqueues lane jobs and swaps
// spares. Jolt's own allocations (external_violation_count) are reported,
// not gated, like every other zero-alloc test here.
//
// Suppression is switched off: an install that falls back to the
// synchronous path on a tile no precompute covered still runs the lazy
// suppression computation and inserts its result into suppression_cache_
// (an allocation, pre-existing and independent of the ring - see
// suppress_covered_samples), which would hide what this case measures.
TEST_CASE("Prefetch on: tile-row crossings allocate nothing in update() or the lane builder", "[terrain][tiles][prefetch][alloc]") {
    if (!jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hook unavailable under this build (ASan/TSan) - skipping");
        return;
    }
    auto source = std::make_shared<ProceduralTerrainSource>(make_test_spec(), 8);
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    jobs::JobSystem js(2);

    TerrainConfig tc = small_config(); // 64-slot pool, 4 fills/tick, margin 1
    tc.prefetch_max_tiles = 40;        // the 24 margin keys of a 5x5 square hold a ticket each, plus the leading edge
    tc.suppress_terrain_under_static_geometry = false;
    TileManager mgr(source, tc, world.backend());
    mgr.start_prefetch(js);
    REQUIRE(mgr.prefetch_enabled());

    // Tile centre (x, y) of the 32 m tile column/row (cx, cy).
    const auto centre = [](int cx, int cy) { return Vec3{32.0 * cx + 16.0, 32.0 * cy + 16.0, 0.0}; };
    // Moves to `p`, lets the lane finish last tick's tickets (outside any
    // job), then runs update() inside a job, `ticks` times.
    const auto drive_to = [&](const Vec3& p, int ticks) {
        mgr.set_interest_point(1, p, kRadius);
        for (int i = 0; i < ticks; ++i) {
            js.drain_background();
            jobs::JobSystem::WaitGroup group;
            js.submit([&] { mgr.update(js); }, &group);
            js.wait(group);
        }
    };

    // Every tile of the 5x5 needed square around `p` is resident
    // (resident_tile_count() also counts stale slots still holding a key).
    const auto needed_resident = [&](const Vec3& p) {
        const TileKey c = source->key_for_world_xy(p.x, p.y);
        int n = 0;
        for (std::int32_t dy = -2; dy <= 2; ++dy) {
            for (std::int32_t dx = -2; dx <= 2; ++dx) n += mgr.resident_body(source->key_offset(c, dx, dy)).valid() ? 1 : 0;
        }
        return n;
    };

    // Warm-up (allocation expected: first use of each pool slot and spare
    // in Jolt, scratch vectors growing to their high-water mark): stream in,
    // then the same pattern of crossings the measurement repeats, so every
    // pool slot has been assigned at least once.
    drive_to(centre(0, 0), 12);
    for (int step = 1; step <= 8; ++step) drive_to(centre(step, step / 2), 8);
    for (int step = 7; step >= 0; --step) drive_to(centre(step, 4 + step / 2), 8);
    REQUIRE(mgr.resident_tile_count() == 64); // every pool slot used at least once
    REQUIRE(needed_resident(centre(0, 4)) == 25);

    const PrefetchStats before = mgr.prefetch_stats();
    jobs::debug_alloc::reset();
    jobs::debug_alloc::set_armed(true);
    // Measured: east across 8 columns (every second step also a row), then
    // back west one row further north - every step a crossing that
    // reassigns 5-9 slots, installed from tickets.
    for (int step = 1; step <= 8; ++step) drive_to(centre(step, 8 + step / 2), 8);
    for (int step = 7; step >= 0; --step) drive_to(centre(step, 13 + step / 2), 8);
    // And without draining the lane: some installs find their ticket still
    // Queued or Running (late_sync / late_wait paths).
    mgr.set_interest_point(1, centre(20, 20), kRadius);
    for (int i = 0; i < 12; ++i) {
        jobs::JobSystem::WaitGroup group;
        js.submit([&] { mgr.update(js); }, &group);
        js.wait(group);
    }
    js.drain_background();
    jobs::debug_alloc::set_armed(false);
    const std::uint64_t own = jobs::debug_alloc::own_violation_count();
    const std::uint64_t external = jobs::debug_alloc::external_violation_count();
    jobs::debug_alloc::reset();

    const PrefetchStats& after = mgr.prefetch_stats();
    INFO("own " << own << " external (Jolt, not gated) " << external << " installed " << (after.installed - before.installed)
                << " late_sync " << (after.late_sync - before.late_sync) << " late_wait " << (after.late_wait - before.late_wait)
                << " cancelled " << (after.cancelled - before.cancelled));
    CHECK(after.installed - before.installed >= 50); // the crossings really went through the lane
    CHECK(needed_resident(centre(20, 20)) == 25);
    REQUIRE(own == 0);
}
