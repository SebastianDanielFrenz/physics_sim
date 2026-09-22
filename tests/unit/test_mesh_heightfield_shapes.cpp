// Unit tests for ps::MeshShape / ps::HeightfieldShape (PLAN.md M1 step 1:
// "Mesh and heightfield collision shapes"): a body resting on each, a shape
// cast returning the right surface id (also the test that actually proves
// the Y-up(Jolt)->Z-up(ours) axis fix-up in jolt_backend.cpp's
// kHeightfieldYupToZup is correct - see that constant's comment), and a
// steady-state zero-allocation check for update_heightfield_body.

#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>

using namespace ps;

namespace {

MeshShape flat_quad_mesh(real half_extent, SurfaceId surface) {
    MeshShape m;
    m.vertices = {
        Vec3{-half_extent, -half_extent, 0.0},
        Vec3{half_extent, -half_extent, 0.0},
        Vec3{half_extent, half_extent, 0.0},
        Vec3{-half_extent, half_extent, 0.0},
    };
    // Counter-clockwise viewed from +z (Jolt's own MeshShapeSettings convention).
    m.indices = {0, 1, 2, 0, 2, 3};
    m.triangle_surface_ids = {surface, surface};
    return m;
}

} // namespace

TEST_CASE("A body settles resting on a MeshShape", "[shapes][mesh]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.gravity = Vec3{0, 0, -9.81};
    World world(cfg);

    BodyDesc ground;
    ground.shape = flat_quad_mesh(50.0, 3);
    ground.motion = BodyMotionType::Static;
    ground.friction = 0.8;
    world.create_body(ground);

    BodyDesc ball;
    ball.shape = SphereShape{0.5};
    ball.motion = BodyMotionType::Dynamic;
    ball.mass = 1.0;
    ball.pose.position = Vec3{0, 0, 2.0};
    ball.friction = 0.8;
    ball.allow_sleep = false;
    const BodyId id = world.create_body(ball);

    for (int i = 0; i < static_cast<int>(3.0 * 240.0); ++i) world.step();

    const real z = world.get_pose(id).position.z;
    REQUIRE(std::abs(z - 0.5) < 0.02); // settles within 2 cm of the analytic resting height
}

TEST_CASE("A body settles resting on a flat HeightfieldShape", "[shapes][heightfield]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.gravity = Vec3{0, 0, -9.81};
    World world(cfg);

    HeightfieldShape hf;
    hf.sample_spacing = 5.0;
    hf.sample_count = 8; // even, PLAN.md M1 step 1 constraint
    hf.heights.assign(static_cast<std::size_t>(hf.sample_count) * hf.sample_count, 0.0);
    hf.origin = Vec3{-static_cast<real>(hf.sample_count - 1) * hf.sample_spacing / 2.0,
                      -static_cast<real>(hf.sample_count - 1) * hf.sample_spacing / 2.0, 0.0};

    BodyDesc ground;
    ground.shape = hf;
    ground.motion = BodyMotionType::Static;
    ground.friction = 0.8;
    world.create_body(ground);

    BodyDesc ball;
    ball.shape = SphereShape{0.5};
    ball.motion = BodyMotionType::Dynamic;
    ball.mass = 1.0;
    ball.pose.position = Vec3{0, 0, 2.0};
    ball.friction = 0.8;
    ball.allow_sleep = false;
    const BodyId id = world.create_body(ball);

    for (int i = 0; i < static_cast<int>(3.0 * 240.0); ++i) world.step();

    const real z = world.get_pose(id).position.z;
    REQUIRE(std::abs(z - 0.5) < 0.02);
}

// The real orientation check: a heightfield whose HEIGHT varies only with
// the x sample index and whose SURFACE varies only with the y sample index,
// probed with a shape cast at known world (x, y) - if kHeightfieldYupToZup
// (jolt_backend.cpp) or the mScale.z sign were wrong, either the returned
// height would not track x the way it should, or the returned surface would
// not track y the way it should (most likely: swapped, or the surface
// reading back constant regardless of y).
TEST_CASE("Heightfield shape cast reports the right height and surface for x and y", "[shapes][heightfield]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    constexpr std::uint32_t kN = 4; // -> 3x3 cells
    constexpr real kSpacing = 10.0;
    HeightfieldShape hf;
    hf.sample_spacing = kSpacing;
    hf.sample_count = kN;
    hf.heights.assign(static_cast<std::size_t>(kN) * kN, 0.0);
    for (std::uint32_t iy = 0; iy < kN; ++iy) {
        for (std::uint32_t ix = 0; ix < kN; ++ix) {
            hf.heights[iy * kN + ix] = static_cast<real>(ix) * 2.0; // height depends only on x
        }
    }
    hf.cell_surface_ids.assign(static_cast<std::size_t>(kN - 1) * (kN - 1), kInvalidSurfaceId);
    for (std::uint32_t iy = 0; iy + 1 < kN; ++iy) {
        for (std::uint32_t ix = 0; ix + 1 < kN; ++ix) {
            hf.cell_surface_ids[iy * (kN - 1) + ix] = static_cast<SurfaceId>(iy); // surface depends only on y
        }
    }
    hf.origin = Vec3::zero();

    BodyDesc ground;
    ground.shape = hf;
    ground.motion = BodyMotionType::Static;
    world.create_body(ground);

    IRigidBackend& backend = world.backend();
    for (std::uint32_t ix = 0; ix < kN; ++ix) {
        for (std::uint32_t iy = 0; iy + 1 < kN; ++iy) {
            // Probe at the CENTRE of cell (ix-ish, iy) in y, and exactly at
            // sample column ix in x (height is linear in x, so it is exact
            // at a grid line even though the two triangles of the quad
            // could otherwise blend differently off-grid).
            const real world_x = static_cast<real>(ix) * kSpacing;
            const real world_y = (static_cast<real>(iy) + 0.5) * kSpacing;
            const Pose start{Vec3{world_x, world_y, 100.0}, Quat::identity()};
            const ShapeCastHit hit = backend.shape_cast(SphereShape{0.05}, start, Vec3{0, 0, -1}, 200.0);
            REQUIRE(hit.hit);
            const real expected_height = static_cast<real>(ix) * 2.0;
            INFO("ix=" << ix << " iy=" << iy << " point.z=" << hit.point.z << " expected=" << expected_height);
            REQUIRE(std::abs(hit.point.z - expected_height) < 0.1);
            REQUIRE(hit.surface == static_cast<SurfaceId>(iy));
        }
    }
}

TEST_CASE("update_heightfield_body allocates nothing once warmed up", "[shapes][heightfield][alloc]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hook unavailable under this build (ASan/TSan) - skipping");
        return;
    }

    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    constexpr std::uint32_t kN = 8;
    HeightfieldShape hf;
    hf.sample_spacing = 2.0;
    hf.sample_count = kN;
    hf.heights.assign(static_cast<std::size_t>(kN) * kN, 0.0);
    hf.cell_surface_ids.assign(static_cast<std::size_t>(kN - 1) * (kN - 1), 0);

    BodyDesc ground;
    ground.shape = hf;
    ground.motion = BodyMotionType::Static;
    const BodyId id = world.create_body(ground);

    IRigidBackend& backend = world.backend();

    std::vector<real> heights(static_cast<std::size_t>(kN) * kN, 0.0);
    std::vector<SurfaceId> surfaces(static_cast<std::size_t>(kN - 1) * (kN - 1), 0);

    // debug_alloc only counts a violation while ps::jobs::debug_alloc::
    // in_job() is true (set by JobSystem::run_job around a job body, see
    // debug_alloc.cpp) - calling update_heightfield_body directly from this
    // test's own thread would never trip it regardless of what it
    // allocates, so run it as an actual job the same way a future
    // TileManager would (from inside World's terrain_tiles pipeline stage).
    ps::jobs::JobSystem js(1);
    auto call_update = [&](int seed) {
        for (std::size_t j = 0; j < heights.size(); ++j) heights[j] = static_cast<real>(seed + static_cast<int>(j));
        ps::jobs::JobSystem::WaitGroup group;
        js.submit([&] { backend.update_heightfield_body(id, heights, surfaces); }, &group);
        js.wait(group);
    };

    // Warm-up: lets the backend's scratch conversion buffers grow to their
    // steady-state size (see jolt_backend.cpp's update_heightfield_body
    // comment) before arming the allocation hook.
    for (int i = 0; i < 4; ++i) call_update(i);

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < 8; ++i) call_update(-i);
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t violations = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();

    REQUIRE(violations == 0);
}
