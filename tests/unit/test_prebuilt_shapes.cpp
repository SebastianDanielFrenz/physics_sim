// tests/unit/test_prebuilt_shapes.cpp - racing_game request R2: prebuilt
// collision shapes (ps/backend/shape_handle.h). World::create_shape builds a
// shape (for a MeshShape, Jolt's whole BVH) on any thread, also while
// World::step runs; World::create_body(desc, handle) installs it on the sim
// thread at O(1) cost. These tests pin:
//
//  1. Equivalence: a 300 m x 8 m deck at 1 m stations installed via desc vs
//     via handle gives equal BodyIds, an equal shape digest and an equal
//     state_hash every tick (1 and 10 workers) with a sphere, a box and
//     car_sedan running over it, next to a heightfield "abutment" so the
//     terrain seam-welding path (jolt_backend.cpp collide_body_vs_body) runs.
//     SABOTAGE (recorded): the handle overload of create_body passed
//     is_mesh = false to add_body (the deck missing from the seam-welding
//     lists). Measured: "1 worker(s)" failed at
//     REQUIRE(diff == desc.hashes.size()) with "44 == 960" - the first
//     divergent state_hash is tick 44, the ramp ball crossing the deck's
//     start edge over the heightfield (the seam-welding path).
//  2. Order independence: 8 decks built concurrently on 8 threads, published
//     in 4 different completion orders, installed in a fixed order -> equal
//     BodyIds, ray hits and state_hash, equal to the desc path. The decks are
//     deck-local meshes placed by the body pose, so a shape installed into
//     the wrong body is visible. SABOTAGE (recorded): install in completion
//     order instead of index order. Measured: failed on the reversed
//     completion order at REQUIRE(run.ray_hits == reference.ray_hits) (the
//     BodyId sequence itself still matched - install order alone decides it).
//  3. Concurrency (TSan leg, not [long]): shapes built on 3 threads and
//     handle copies released on a 4th thread WHILE World::step drives
//     car_sedan; installs and destroys between steps on a fixed schedule
//     (waiting for a shape that is not built yet - racing_game's clock
//     freeze); final state_hash equals a reference run on the desc path. A
//     separate case checks that create_shape on an ordinary std::thread is
//     not a debug_alloc in-job violation while the World's jobs are armed.
//     SABOTAGE (recorded): a plain (non-atomic) static counter incremented
//     inside create_jolt_shape_handle. linux-tsan reported two
//     "WARNING: ThreadSanitizer: data race" (builder thread vs builder
//     thread, jolt_backend.cpp create_jolt_shape_handle) and exited 66 while
//     the case's own assertions passed. Alloc-case sabotage: the builder
//     wrapped create_shape in debug_alloc::enter_job()/exit_job() (= calling
//     it from a job body): own violations 0 -> 1276, the case failed.
//  4. Lifetime: releasing the handle right after create_body keeps the shape
//     (the body still collides), destroy_body after the last release frees
//     it (debug_live_prepared_shape_count / debug_prepared_shape_backend_frees),
//     the reverse order frees on release (also from another thread), and a
//     handle outlives its World and installs into a second World. SABOTAGE
//     (recorded): free_slot did not reset Slot::prepared. Measured: failed at
//     the live-prepared-shape check after destroy_body with "1 == 0" (the
//     "release then destroy" and "outlive the World" sections).
//  5. Suppression: a handle-created static mesh on a flat terrain punches the
//     same holes as the desc path (TileManager precompute and lazy path),
//     and World::create_body(desc, handle) fires the terrain hook for a
//     static body only. SABOTAGE (recorded): the World handle overload
//     skipped on_static_geometry_changed. Measured: failed at
//     REQUIRE(world.terrain_precomputed_tile_count() == 0) (stayed 1).
//  6. Cost: [.][diag] create_body(desc) vs create_shape + create_body(handle)
//     for 300/1000/3000 m decks (numbers in out/prebuilt_shapes_progress.md).

#include "terrain/tile_manager.h"

#include "ps/backend/shape_handle.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/terrain/terrain_source.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;

namespace {

// A deck-like strip: 1 m stations along +x, 1 m columns across an 8 m width,
// a longitudinal grade and a 2 % crown. Triangle edges <= 1.42 m (PHYS-047).
real deck_z(real x, real y, real x0, real y0, real z0, real grade) {
    return z0 + grade * (x - x0) - 0.02 * std::abs(y - y0);
}

MeshShape deck_mesh(real length, real x0, real y0, real z0, real grade, SurfaceId surface) {
    const int stations = static_cast<int>(length);
    constexpr int kCols = 8;
    MeshShape m;
    for (int i = 0; i <= stations; ++i) {
        for (int j = 0; j <= kCols; ++j) {
            const real x = x0 + i;
            const real y = y0 - 4.0 + j;
            m.vertices.push_back(Vec3{x, y, deck_z(x, y, x0, y0, z0, grade)});
        }
    }
    const auto nv = static_cast<std::uint32_t>(kCols + 1);
    for (int i = 0; i < stations; ++i) {
        for (int j = 0; j < kCols; ++j) {
            const auto a = static_cast<std::uint32_t>(i) * nv + static_cast<std::uint32_t>(j);
            const std::uint32_t b = a + nv;     // (x+1, y)
            const std::uint32_t c = a + nv + 1; // (x+1, y+1)
            const std::uint32_t d = a + 1;      // (x, y+1)
            m.indices.insert(m.indices.end(), {a, b, c, a, c, d}); // CCW viewed from +z
            m.triangle_surface_ids.push_back(surface);
            m.triangle_surface_ids.push_back(surface);
        }
    }
    return m;
}

std::shared_ptr<const io::SurfaceTable> surface_table() {
    return std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
}

BodyDesc static_desc(ShapeDesc shape) {
    BodyDesc bd;
    bd.shape = std::move(shape);
    bd.motion = BodyMotionType::Static;
    bd.friction = 1.0;
    return bd;
}

struct Trace {
    std::vector<BodyId> ids;
    std::vector<std::uint64_t> hashes;
    std::uint64_t deck_digest = 0;
    Vec3 car_end{};
    Vec3 ramp_ball_end{};
};

std::size_t first_difference(const std::vector<std::uint64_t>& a, const std::vector<std::uint64_t>& b) {
    for (std::size_t i = 0; i < std::min(a.size(), b.size()); ++i)
        if (a[i] != b[i]) return i;
    return a.size() == b.size() ? a.size() : std::min(a.size(), b.size());
}

// ---------------------------------------------------------------------------
// Test 1 scene.
constexpr real kX0 = 0.0, kY0 = 0.0, kZ0 = 5.0, kGrade = 0.01;

Trace run_equivalence(bool use_handle, int workers, int ticks) {
    WorldConfig cfg;
    cfg.job_workers = workers;
    World world(cfg);
    auto surfaces = surface_table();
    world.set_surface_table(surfaces);
    Trace t;

    // Heightfield "abutment" 1 cm under the first ~120 m of the deck, one far
    // corner raised so its AABB contains every body on the deck: every
    // (body, deck) pair then runs the seam-welding narrow phase.
    HeightfieldShape hf;
    hf.sample_count = 32;
    hf.sample_spacing = 4.0;
    hf.origin = Vec3{kX0 - 4.0, kY0 - 62.0, 0.0};
    hf.heights.resize(32 * 32);
    for (std::uint32_t iy = 0; iy < 32; ++iy)
        for (std::uint32_t ix = 0; ix < 32; ++ix) {
            const real x = hf.origin.x + ix * hf.sample_spacing;
            const real y = hf.origin.y + iy * hf.sample_spacing;
            hf.heights[iy * 32 + ix] = deck_z(x, y, kX0, kY0, kZ0, kGrade) - 0.005; // 5 mm under the deck, crown included
        }
    hf.heights[31] = kZ0 + 3.0; // far corner (iy = 0, ix = 31: y = -62)
    t.ids.push_back(world.create_body(static_desc(hf)));

    const MeshShape deck = deck_mesh(300.0, kX0, kY0, kZ0, kGrade, surfaces->id_for("dirt"));
    if (use_handle) {
        ShapeHandle h = world.create_shape(deck);
        t.ids.push_back(world.create_body(static_desc(SphereShape{}), h)); // desc.shape ignored
    } else {
        t.ids.push_back(world.create_body(static_desc(deck)));
    }
    t.deck_digest = world.backend().debug_body_shape_digest(t.ids.back());

    BodyDesc sphere;
    sphere.shape = SphereShape{0.4};
    sphere.mass = 20.0;
    sphere.pose.position = Vec3{kX0 + 20.0, kY0 + 1.5, deck_z(kX0 + 20.0, 1.5, kX0, kY0, kZ0, kGrade) + 0.6};
    sphere.velocity.linear = Vec3{6.0, 0.0, 0.0};
    t.ids.push_back(world.create_body(sphere));

    // A second ball rolls off the heightfield (5 mm below the deck) up onto
    // the deck's start edge, right over a deck vertex that coincides with a
    // heightfield sample (x0, 2) - the seam-welding case racing_game's
    // abutments produce.
    BodyDesc ramp_ball = sphere;
    ramp_ball.pose.position = Vec3{kX0 - 3.0, kY0 + 2.0, deck_z(kX0 - 3.0, 2.0, kX0, kY0, kZ0, kGrade) + 0.4};
    ramp_ball.velocity.linear = Vec3{4.0, 0.0, 0.0};
    t.ids.push_back(world.create_body(ramp_ball));

    BodyDesc box;
    box.shape = BoxShape{Vec3{0.5, 0.4, 0.3}};
    box.mass = 50.0;
    box.pose.position = Vec3{kX0 + 40.0, kY0 - 2.0, deck_z(kX0 + 40.0, -2.0, kX0, kY0, kZ0, kGrade) + 0.5};
    box.velocity.linear = Vec3{8.0, 0.3, 0.0};
    t.ids.push_back(world.create_body(box));

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.pose.position = Vec3{kX0 + 80.0, kY0, deck_z(kX0 + 80.0, 0.0, kX0, kY0, kZ0, kGrade) + 0.6};
    const BodyId chassis_id = world.create_body(chassis);
    t.ids.push_back(chassis_id);
    world.create_vehicle(io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json"), chassis_id);
    world.set_control("gear_request", 1.0);
    world.set_control("throttle", 0.5);

    for (int i = 0; i < ticks; ++i) {
        world.step();
        t.hashes.push_back(world.state_hash());
    }
    t.car_end = world.get_pose(chassis_id).position;
    t.ramp_ball_end = world.get_pose(t.ids[3]).position;
    return t;
}

} // namespace

TEST_CASE("Prebuilt deck mesh: handle path is bit-identical to the desc path (BodyIds, shape, state_hash)", "[backend][world][vehicle]") {
    constexpr int kTicks = 960; // 4 s at 240 Hz
    for (int workers : {1, 10}) {
        INFO(workers << " worker(s)");
        const Trace desc = run_equivalence(false, workers, kTicks);
        const Trace handle = run_equivalence(true, workers, kTicks);
        REQUIRE(desc.ids == handle.ids);
        REQUIRE(desc.deck_digest == handle.deck_digest);
        REQUIRE(desc.hashes.size() == handle.hashes.size());
        const std::size_t diff = first_difference(desc.hashes, handle.hashes);
        if (diff != desc.hashes.size()) std::printf("equivalence: %d workers, first differing tick %zu\n", workers, diff);
        REQUIRE(diff == desc.hashes.size()); // every tick's state_hash equal
        // The scene did what it is meant to: the car drove up the deck and is still on it.
        const real deck_top = deck_z(desc.car_end.x, desc.car_end.y, kX0, kY0, kZ0, kGrade);
        CHECK(desc.car_end.x > kX0 + 85.0); // measured 90.69 (started at 80, 1st gear, throttle 0.5, 1 % grade)
        CHECK(desc.car_end.z > deck_top + 0.2);
        CHECK(desc.car_end.z < deck_top + 1.0);
        // The ramp ball crossed from the heightfield onto the deck.
        CHECK(desc.ramp_ball_end.x > kX0 + 2.0);
        std::printf("equivalence: %d workers, car at x=%.3f z-deck=%.3f, ramp ball at x=%.3f, final hash 0x%016llx\n", workers,
                    desc.car_end.x, desc.car_end.z - deck_top, desc.ramp_ball_end.x, static_cast<unsigned long long>(desc.hashes.back()));
    }
    // Worker-count invariance of the handle path itself.
    const Trace w1 = run_equivalence(true, 1, 240);
    const Trace w10 = run_equivalence(true, 10, 240);
    REQUIRE(first_difference(w1.hashes, w10.hashes) == w1.hashes.size());
}

namespace {

// ---------------------------------------------------------------------------
// Test 2: N decks, concurrent builds, permuted completion order.
constexpr int kDecks = 8;

// Built in deck-local coordinates and placed by the body pose, so a deck
// installed into the wrong body moves that body's pose (state_hash) and what
// a ray at deck i hits.
real order_deck_length(int i) { return 40.0 + 30.0 * i; }
MeshShape order_deck(int i) { return deck_mesh(order_deck_length(i), 0.0, 0.0, 0.0, 0.0, static_cast<SurfaceId>(i % 3)); }
Vec3 order_deck_pos(int i) { return Vec3{-100.0 + 7.0 * i, -60.0 + 15.0 * i, 0.0}; }

struct OrderRun {
    std::vector<BodyId> ids;
    std::vector<BodyId> ray_hits; // body under the far end of deck i
    std::uint64_t hash = 0;
};

OrderRun run_order(const std::vector<ShapeHandle>* handles) {
    WorldConfig cfg;
    cfg.job_workers = 4;
    World world(cfg);
    OrderRun r;
    for (int i = 0; i < kDecks; ++i) {
        BodyDesc bd = static_desc(SphereShape{});
        bd.pose.position = order_deck_pos(i);
        if (handles) {
            r.ids.push_back(world.create_body(bd, (*handles)[static_cast<std::size_t>(i)]));
        } else {
            bd.shape = order_deck(i);
            r.ids.push_back(world.create_body(bd));
        }
    }
    for (int i = 0; i < kDecks; ++i) {
        const Vec3 p = order_deck_pos(i) + Vec3{order_deck_length(i) - 1.0, 0.5, 5.0};
        r.ray_hits.push_back(world.backend().ray_cast(p, Vec3{0.0, 0.0, -1.0}, 10.0).body);
    }
    for (int i = 0; i < kDecks; ++i) {
        BodyDesc s;
        s.shape = SphereShape{0.5};
        s.mass = 10.0;
        s.pose.position = order_deck_pos(i) + Vec3{10.0 + 3.0 * i, 0.5, 1.5};
        s.velocity.linear = Vec3{4.0, 0.0, 0.0};
        r.ids.push_back(world.create_body(s));
    }
    for (int k = 0; k < 360; ++k) world.step();
    r.hash = world.state_hash();
    return r;
}

// Builds every deck concurrently on its own thread; thread i publishes its
// handle only when its turn in `completion` comes up, so the publish
// (completion) order is exactly the permutation. Returns the handles by
// deck index and the completion order observed.
std::vector<ShapeHandle> build_concurrently(const World& world, const std::vector<int>& completion, std::vector<int>& observed) {
    std::vector<ShapeHandle> by_index(kDecks);
    std::mutex m;
    std::condition_variable cv;
    std::size_t turn = 0;
    observed.clear();
    std::vector<std::thread> threads;
    for (int i = 0; i < kDecks; ++i) {
        threads.emplace_back([&, i] {
            ShapeHandle h = world.create_shape(order_deck(i)); // all builds overlap
            std::unique_lock<std::mutex> lock(m);
            cv.wait(lock, [&] { return completion[turn] == i; });
            by_index[static_cast<std::size_t>(i)] = std::move(h);
            observed.push_back(i);
            ++turn;
            cv.notify_all();
        });
    }
    for (auto& th : threads) th.join();
    return by_index;
}

} // namespace

TEST_CASE("Prebuilt shapes: BodyIds and state_hash do not depend on build threads or completion order", "[backend][world]") {
    const OrderRun reference = run_order(nullptr);
    for (int i = 0; i < kDecks; ++i) REQUIRE(reference.ray_hits[static_cast<std::size_t>(i)] == reference.ids[static_cast<std::size_t>(i)]);

    WorldConfig cfg;
    cfg.job_workers = 1;
    const World builder(cfg); // create_shape is const; any World (or none of the installing ones) may build

    std::vector<std::vector<int>> perms;
    std::vector<int> p(kDecks);
    for (int i = 0; i < kDecks; ++i) p[static_cast<std::size_t>(i)] = i;
    perms.push_back(p);
    std::reverse(p.begin(), p.end());
    perms.push_back(p);
    std::mt19937 rng(12345);
    for (int k = 0; k < 2; ++k) {
        std::shuffle(p.begin(), p.end(), rng);
        perms.push_back(p);
    }

    for (const auto& perm : perms) {
        std::vector<int> observed;
        const std::vector<ShapeHandle> handles = build_concurrently(builder, perm, observed);
        REQUIRE(observed == perm);
        const OrderRun run = run_order(&handles); // installs in index order
        REQUIRE(run.ids == reference.ids);
        REQUIRE(run.ray_hits == reference.ray_hits);
        REQUIRE(run.hash == reference.hash);
    }
    std::printf("order independence: %zu completion orders, hash 0x%016llx\n", perms.size(), static_cast<unsigned long long>(reference.hash));
}

namespace {

// ---------------------------------------------------------------------------
// Test 3: shapes built and released on other threads while World::step runs.
constexpr int kBuilders = 3;
constexpr int kPerBuilder = 6;
constexpr int kShapes = kBuilders * kPerBuilder;

ShapeDesc concurrency_shape(int j) {
    switch (j % 4) {
        case 0: return deck_mesh(60.0 + 10.0 * j, 200.0, -200.0 + 12.0 * j, 0.0, 0.0, kInvalidSurfaceId);
        case 1: return BoxShape{Vec3{0.4, 0.3, 0.2 + 0.01 * j}};
        case 2: return SphereShape{0.3 + 0.01 * j};
        default: return CapsuleShape{0.3, 0.2};
    }
}

BodyDesc concurrency_body(int j) {
    BodyDesc bd;
    if (j % 4 == 0) {
        bd.motion = BodyMotionType::Static;
        return bd;
    }
    bd.motion = BodyMotionType::Dynamic;
    bd.mass = 5.0 + j;
    bd.pose.position = Vec3{-50.0 - 3.0 * j, 20.0 + 2.0 * (j % 5), 1.0};
    bd.velocity.linear = Vec3{1.0, 0.0, 0.0};
    return bd;
}

int install_tick(int j) { return 8 + 12 * j; }
int destroy_tick(int j) { return install_tick(j) + 60; } // only odd j

struct ConcurrencyResult {
    std::uint64_t hash = 0;
    std::vector<BodyId> ids;
    std::uint64_t builds = 0;
};

ConcurrencyResult run_concurrency(bool use_handles) {
    WorldConfig cfg;
    cfg.job_workers = 4;
    World world(cfg);
    world.set_surface_table(surface_table());

    BodyDesc ground = static_desc(BoxShape{Vec3{400.0, 400.0, 0.5}});
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    ConcurrencyResult r;
    r.ids.push_back(world.create_body(ground));
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId chassis_id = world.create_body(chassis);
    r.ids.push_back(chassis_id);
    world.create_vehicle(io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json"), chassis_id);
    world.set_control("gear_request", 1.0);
    world.set_control("throttle", 0.4);
    world.set_control("steer", 0.1);

    // Shared between the sim thread, the builders and the releaser.
    std::mutex m;
    std::condition_variable cv;
    std::map<int, ShapeHandle> ready;         // built, waiting to be installed
    std::vector<ShapeHandle> to_release;      // copies handed to the releaser
    bool done = false;
    std::atomic<std::uint64_t> builds{0};

    std::vector<std::thread> threads;
    if (use_handles) {
        for (int b = 0; b < kBuilders; ++b) {
            threads.emplace_back([&, b] {
                for (int k = 0; k < kPerBuilder; ++k) {
                    const int j = b + kBuilders * k; // interleaved, so builds overlap in time
                    ShapeHandle h = world.create_shape(concurrency_shape(j));
                    // A throwaway build released right here, on this thread,
                    // while the sim may be stepping.
                    { ShapeHandle extra = world.create_shape(concurrency_shape(j)); }
                    builds.fetch_add(2, std::memory_order_relaxed);
                    std::lock_guard<std::mutex> lock(m);
                    ready.emplace(j, std::move(h));
                    cv.notify_all();
                }
            });
        }
        threads.emplace_back([&] { // releaser: drops handle copies while the sim steps
            for (;;) {
                std::vector<ShapeHandle> batch;
                {
                    std::unique_lock<std::mutex> lock(m);
                    cv.wait(lock, [&] { return done || !to_release.empty(); });
                    if (to_release.empty() && done) return;
                    batch.swap(to_release);
                }
                batch.clear(); // last copies released off the sim thread
            }
        });
    }

    std::vector<BodyId> installed(kShapes);
    const int last_tick = destroy_tick(kShapes - 1) + 20;
    for (int tick = 0; tick <= last_tick; ++tick) {
        for (int j = 0; j < kShapes; ++j) {
            if (install_tick(j) == tick) {
                if (use_handles) {
                    ShapeHandle h;
                    {
                        std::unique_lock<std::mutex> lock(m);
                        cv.wait(lock, [&] { return ready.count(j) != 0; }); // the "frozen clock" wait
                        h = std::move(ready.at(j));
                        ready.erase(j);
                    }
                    installed[static_cast<std::size_t>(j)] = world.create_body(concurrency_body(j), h);
                    std::lock_guard<std::mutex> lock(m);
                    to_release.push_back(std::move(h));
                    cv.notify_all();
                } else {
                    BodyDesc bd = concurrency_body(j);
                    bd.shape = concurrency_shape(j);
                    installed[static_cast<std::size_t>(j)] = world.create_body(bd);
                }
                r.ids.push_back(installed[static_cast<std::size_t>(j)]);
            }
            if (j % 2 == 1 && destroy_tick(j) == tick) world.destroy_body(installed[static_cast<std::size_t>(j)]);
        }
        world.step();
    }
    {
        std::lock_guard<std::mutex> lock(m);
        done = true;
        cv.notify_all();
    }
    for (auto& th : threads) th.join();
    r.hash = world.state_hash();
    r.builds = builds.load();
    return r;
}

} // namespace

TEST_CASE("Prebuilt shapes: create_shape and handle releases on other threads while World::step drives a car", "[backend][world][vehicle]") {
    const ConcurrencyResult reference = run_concurrency(false);
    const ConcurrencyResult threaded = run_concurrency(true);
    REQUIRE(threaded.builds == 2 * kShapes);
    REQUIRE(threaded.ids == reference.ids);
    REQUIRE(threaded.hash == reference.hash);
    std::printf("concurrency: %llu builds off the sim thread, hash 0x%016llx\n", static_cast<unsigned long long>(threaded.builds),
                static_cast<unsigned long long>(threaded.hash));
}

TEST_CASE("Prebuilt shapes: create_shape on a std::thread is not a debug_alloc in-job violation while the World's jobs are armed",
          "[backend][world][debug_alloc]") {
    namespace da = jobs::debug_alloc;
    if (!da::hook_available()) {
        WARN("debug_alloc's hooks are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    WorldConfig cfg;
    cfg.job_workers = 4;
    World world(cfg);
    BodyDesc ground = static_desc(BoxShape{Vec3{100.0, 100.0, 0.5}});
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    world.create_body(ground);
    BodyDesc ball;
    ball.shape = SphereShape{0.5};
    ball.pose.position = Vec3{0.0, 0.0, 0.5};
    world.create_body(ball);
    for (int i = 0; i < 120; ++i) world.step(); // warm-up: every scratch buffer at its steady size

    // Control window: the World's own steps are own-allocation-free.
    da::reset();
    da::set_armed(true);
    for (int i = 0; i < 60; ++i) world.step();
    da::set_armed(false);
    REQUIRE(da::own_violation_count() == 0);

    std::atomic<bool> stop{false};
    std::atomic<int> built{0};
    da::reset();
    da::set_armed(true);
    std::thread builder([&] {
        while (!stop.load()) {
            ShapeHandle h = world.create_shape(deck_mesh(100.0, 300.0, 300.0, 0.0, 0.0, kInvalidSurfaceId));
            ShapeHandle b = world.create_shape(BoxShape{Vec3{1.0, 1.0, 1.0}});
            built.fetch_add(1);
        } // released on this thread too
    });
    for (int i = 0; i < 240 || built.load() < 3; ++i) world.step();
    stop.store(true);
    builder.join();
    da::set_armed(false);
    const std::uint64_t own = da::own_violation_count();
    std::printf("debug_alloc: %d builds on a std::thread while 240+ armed ticks ran, own violations %llu\n", built.load(),
                static_cast<unsigned long long>(own));
    REQUIRE(built.load() >= 3);
    REQUIRE(own == 0);
}

namespace {

std::uint64_t live() { return debug_live_prepared_shape_count(); }
std::uint64_t frees() { return debug_prepared_shape_backend_frees(); }

// Drops a ball onto the deck at (x, 0) and returns its height after 1 s.
real ball_rest_height(World& world, real x) {
    BodyDesc s;
    s.shape = SphereShape{0.5};
    s.mass = 10.0;
    s.pose.position = Vec3{x, 0.0, 2.0};
    const BodyId b = world.create_body(s);
    for (int i = 0; i < 240; ++i) world.step();
    const real z = world.get_pose(b).position.z;
    world.destroy_body(b);
    return z;
}

} // namespace

TEST_CASE("Prebuilt shapes: lifetime - a body keeps its shape, the last release frees it, a handle may outlive its World",
          "[backend][world]") {
    const MeshShape deck = deck_mesh(50.0, 0.0, 0.0, 0.0, 0.0, kInvalidSurfaceId);
    WorldConfig cfg;
    cfg.job_workers = 2;

    SECTION("release right after create_body; destroy_body frees") {
        World world(cfg);
        const std::uint64_t live0 = live();
        const std::uint64_t frees0 = frees();
        ShapeHandle h = world.create_shape(deck);
        REQUIRE(live() == live0 + 1);
        const BodyId body = world.create_body(static_desc(SphereShape{}), h);
        REQUIRE(h.use_count() == 2); // the handle + the body
        h.reset();
        REQUIRE(live() == live0 + 1); // the body keeps it
        REQUIRE(frees() == frees0);
        CHECK(std::abs(ball_rest_height(world, 25.0) - 0.5) < 0.05); // still collides
        const RayCastHit hit = world.backend().ray_cast(Vec3{10.0, 0.5, 5.0}, Vec3{0.0, 0.0, -1.0}, 10.0);
        REQUIRE(hit.hit);
        REQUIRE(hit.body == body);
        world.destroy_body(body);
        REQUIRE(live() == live0);
        REQUIRE(frees() == frees0 + 1);
    }

    SECTION("destroy_body first; the last handle release (on another thread) frees") {
        World world(cfg);
        const std::uint64_t live0 = live();
        const std::uint64_t frees0 = frees();
        ShapeHandle h = world.create_shape(deck);
        ShapeHandle copy = h;
        const BodyId a = world.create_body(static_desc(SphereShape{}), h);
        const BodyId b = world.create_body(static_desc(SphereShape{}), copy); // two bodies share one shape
        REQUIRE(h.use_count() == 4);
        world.destroy_body(a);
        world.destroy_body(b);
        REQUIRE(live() == live0 + 1);
        REQUIRE(frees() == frees0);
        h.reset();
        REQUIRE(live() == live0 + 1);
        std::thread([c = std::move(copy)]() mutable { c.reset(); }).join();
        REQUIRE(live() == live0);
        REQUIRE(frees() == frees0 + 1);
    }

    SECTION("a handle outlives its World and installs into another World") {
        const std::uint64_t live0 = live();
        const std::uint64_t frees0 = frees();
        ShapeHandle h;
        {
            World first(cfg);
            h = first.create_shape(deck);
            first.create_body(static_desc(SphereShape{}), h);
            REQUIRE(h.use_count() == 2);
        } // the World and its body are gone; the shape is not
        REQUIRE(h.valid());
        REQUIRE(h.use_count() == 1);
        REQUIRE(live() == live0 + 1);
        REQUIRE(frees() == frees0);
        {
            World second(cfg);
            second.create_body(static_desc(SphereShape{}), h);
            CHECK(std::abs(ball_rest_height(second, 30.0) - 0.5) < 0.05);
            h.reset(); // the second World's body is now the only owner
            REQUIRE(live() == live0 + 1);
        } // ~World destroys the body -> freed
        REQUIRE(live() == live0);
        REQUIRE(frees() == frees0 + 1);
    }

    SECTION("rejections") {
        World world(cfg);
        CHECK_THROWS_AS(world.create_body(static_desc(SphereShape{}), ShapeHandle{}), std::invalid_argument);
        HeightfieldShape hf;
        hf.sample_count = 4;
        hf.heights.assign(16, 0.0);
        CHECK_THROWS_AS(world.create_shape(hf), std::invalid_argument);
        CHECK_THROWS_AS(world.create_shape(MeshShape{}), std::runtime_error); // no triangles
        ShapeHandle mesh = world.create_shape(deck);
        BodyDesc dynamic;
        dynamic.motion = BodyMotionType::Dynamic;
        CHECK_THROWS_AS(world.create_body(dynamic, mesh), std::invalid_argument);
        REQUIRE(world.backend().body_count() == 0);
        // Convex handles work for dynamic bodies and report their kind.
        const ShapeHandle box = world.create_shape(BoxShape{Vec3{0.5, 0.5, 0.5}});
        REQUIRE(box.kind() == ShapeKind::Box);
        REQUIRE(mesh.kind() == ShapeKind::Mesh);
        REQUIRE(world.create_body(dynamic, box).valid());
    }
}

namespace {

// ---------------------------------------------------------------------------
// Test 5: flat-zero analytic terrain (tile 32 m, 16 samples).
class FlatTerrain final : public terrain::ITerrainSource {
public:
    [[nodiscard]] real tile_size_m() const override { return 32.0; }
    [[nodiscard]] std::uint32_t samples_per_tile() const override { return 16; }
    [[nodiscard]] terrain::TileKey key_for_world_xy(real x, real y) const override {
        return pack(static_cast<std::int32_t>(std::floor(x / 32.0)), static_cast<std::int32_t>(std::floor(y / 32.0)));
    }
    [[nodiscard]] terrain::TileKey key_offset(terrain::TileKey key, std::int32_t dx, std::int32_t dy) const override {
        return pack(tx(key) + dx, ty(key) + dy);
    }
    [[nodiscard]] Vec3 tile_origin_xy(terrain::TileKey key) const override { return Vec3{tx(key) * 32.0, ty(key) * 32.0, 0.0}; }
    void fill_tile(terrain::TileKey key, terrain::TileSample& out) const override {
        std::fill(out.heights.begin(), out.heights.end(), 0.0);
        std::fill(out.cell_surface_ids.begin(), out.cell_surface_ids.end(), kInvalidSurfaceId);
        out.samples_per_tile = 16;
        out.tile_size_m = 32.0;
        out.tile_origin_xy = tile_origin_xy(key);
    }

private:
    static terrain::TileKey pack(std::int32_t x, std::int32_t y) {
        return terrain::TileKey{(static_cast<std::int64_t>(static_cast<std::uint32_t>(x)) << 32) |
                                static_cast<std::int64_t>(static_cast<std::uint32_t>(y))};
    }
    static std::int32_t tx(terrain::TileKey k) { return static_cast<std::int32_t>(static_cast<std::uint64_t>(k.packed) >> 32); }
    static std::int32_t ty(terrain::TileKey k) { return static_cast<std::int32_t>(static_cast<std::uint64_t>(k.packed) & 0xffffffffu); }
};

std::map<std::int64_t, std::vector<std::uint32_t>> precomputed_holes(World& world, const std::shared_ptr<const terrain::ITerrainSource>& src,
                                                                     std::size_t& tiles_with_holes) {
    terrain::TerrainConfig tc;
    tc.max_resident_tiles = 32;
    tc.max_tile_fills_per_tick = 32;
    jobs::JobSystem jobs(2);
    terrain::TileManager mgr(src, tc, world.backend());
    mgr.precompute_suppression(jobs);
    std::map<std::int64_t, std::vector<std::uint32_t>> out;
    tiles_with_holes = 0;
    for (const terrain::TileKey& k : mgr.precomputed_keys()) {
        const std::vector<std::uint32_t>* cached = mgr.cached_suppression(k);
        REQUIRE(cached != nullptr);
        REQUIRE(*cached == mgr.compute_suppression_uncached(k)); // lazy path agrees
        if (!cached->empty()) ++tiles_with_holes;
        out.emplace(k.packed, *cached);
    }
    return out;
}

} // namespace

TEST_CASE("Prebuilt shapes: a handle-created static mesh punches the same terrain holes and fires the suppression hook",
          "[backend][world][terrain]") {
    const auto src = std::make_shared<const FlatTerrain>();
    // A flat deck lying exactly on the z = 0 terrain, across a tile seam.
    const MeshShape deck = deck_mesh(60.0, 10.0, 20.0, 0.0, 0.0, kInvalidSurfaceId);
    MeshShape flat = deck;
    for (Vec3& v : flat.vertices) v.z = 0.0; // the hole rule needs vertices exactly at z = 0

    WorldConfig cfg;
    cfg.job_workers = 1;
    World desc_world(cfg);
    World handle_world(cfg);
    desc_world.create_body(static_desc(flat));
    const ShapeHandle h = handle_world.create_shape(flat);
    handle_world.create_body(static_desc(SphereShape{}), h);

    std::size_t desc_holes = 0;
    std::size_t handle_holes = 0;
    const auto a = precomputed_holes(desc_world, src, desc_holes);
    const auto b = precomputed_holes(handle_world, src, handle_holes);
    REQUIRE(desc_holes > 0);
    REQUIRE(a == b);
    std::size_t total = 0;
    for (const auto& [k, v] : a) total += v.size();
    std::printf("suppression: %zu tiles precomputed, %zu with holes, %zu holes - desc == handle\n", a.size(), desc_holes, total);

    // The World hook: a static body from a handle drops the precomputed
    // suppression cache exactly like a desc-created one; a dynamic one does not.
    World world(cfg);
    BodyDesc pad = static_desc(BoxShape{Vec3{4.0, 4.0, 0.5}});
    pad.pose.position = Vec3{-40.0, -40.0, -0.5};
    world.create_body(pad);
    terrain::TerrainConfig tc;
    tc.max_resident_tiles = 32;
    world.set_terrain_source(src, tc);
    const std::size_t precomputed = world.terrain_precomputed_tile_count();
    REQUIRE(precomputed > 0);
    BodyDesc ball;
    ball.shape = SphereShape{0.5};
    ball.pose.position = Vec3{0.0, 0.0, 5.0};
    world.create_body(ball, world.create_shape(SphereShape{0.5}));
    REQUIRE(world.terrain_precomputed_tile_count() == precomputed);
    world.create_body(static_desc(SphereShape{}), h); // the same handle, a third body in another World
    REQUIRE(world.terrain_precomputed_tile_count() == 0);
}

TEST_CASE("Prebuilt shapes: create_body(desc) vs create_shape + create_body(handle) cost", "[.][diag][backend][world]") {
    WorldConfig cfg;
    cfg.job_workers = 4;
    std::printf("%8s %10s %18s %16s %22s %16s\n", "deck_m", "triangles", "create_body(desc)", "create_shape", "create_body(handle)",
                "destroy_body");
    for (real length : {300.0, 1000.0, 3000.0}) {
        const MeshShape deck = deck_mesh(length, 0.0, 0.0, 0.0, 0.01, 0);
        constexpr int kReps = 5;
        double t_desc = 0, t_shape = 0, t_handle = 0, t_destroy = 0;
        World world(cfg);
        for (int r = 0; r < kReps; ++r) {
            auto t0 = std::chrono::steady_clock::now();
            const BodyId a = world.create_body(static_desc(deck));
            auto t1 = std::chrono::steady_clock::now();
            const ShapeHandle h = world.create_shape(deck);
            auto t2 = std::chrono::steady_clock::now();
            const BodyId b = world.create_body(static_desc(SphereShape{}), h);
            auto t3 = std::chrono::steady_clock::now();
            world.destroy_body(b);
            auto t4 = std::chrono::steady_clock::now();
            world.destroy_body(a);
            t_desc += std::chrono::duration<double, std::milli>(t1 - t0).count();
            t_shape += std::chrono::duration<double, std::milli>(t2 - t1).count();
            t_handle += std::chrono::duration<double, std::milli>(t3 - t2).count();
            t_destroy += std::chrono::duration<double, std::milli>(t4 - t3).count();
        }
        std::printf("%8.0f %10zu %15.3f ms %13.3f ms %19.4f ms %13.4f ms\n", length, deck.indices.size() / 3, t_desc / kReps, t_shape / kReps,
                    t_handle / kReps, t_destroy / kReps);
    }
}
