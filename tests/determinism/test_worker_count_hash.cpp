// Determinism: state hash identical with 1, 2, 4, 10, 24 workers, and
// under a randomised-scheduling stress test (PLAN.md P1 acceptance / D7/D8).
//
// The scenario mixes gravity (Jolt's own internal per-body parallel work,
// via the JPH::JobSystem bridge) with a batch of CounterForceElement
// instances evaluated through World's own parallel_for force-element stage
// and gathered in fixed id order (PLAN.md D8: scatter -> compute -> ordered
// gather) - so this test exercises determinism of both the Jolt bridge and
// our own job-scheduled code, not just one of them.

#include "counter_force_element.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <random>
#include <vector>

using namespace ps;
using ps_test::CounterForceElement;

namespace {

std::uint64_t run_scenario(int workers, std::size_t num_bodies, std::size_t grain, int ticks) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.gravity = Vec3{0, 0, -9.81};
    cfg.job_workers = workers;
    cfg.force_element_grain = grain;
    World world(cfg);

    std::vector<BodyId> bodies;
    std::vector<std::unique_ptr<CounterForceElement>> elements;
    bodies.reserve(num_bodies);
    elements.reserve(num_bodies);

    for (std::size_t i = 0; i < num_bodies; ++i) {
        BodyDesc desc;
        desc.shape = SphereShape{0.3};
        desc.motion = BodyMotionType::Dynamic;
        desc.mass = 1.0;
        desc.pose.position = Vec3{static_cast<double>(i) * 1.5, 0.0, 20.0 + static_cast<double>(i % 5)};
        desc.gravity_enabled = true;
        desc.allow_sleep = false;
        const BodyId id = world.create_body(desc);
        bodies.push_back(id);
        elements.push_back(std::make_unique<CounterForceElement>(Frame(id), static_cast<std::uint64_t>(i), 3.0));
        world.add_force_element(elements.back().get());
    }

    for (int t = 0; t < ticks; ++t) world.step();
    return world.state_hash();
}

} // namespace

TEST_CASE("State hash is identical across 1, 2, 4, 10 and 24 workers", "[determinism]") {
    constexpr std::size_t kBodies = 64;
    constexpr int kTicks = 40;

    const std::uint64_t reference = run_scenario(1, kBodies, 0, kTicks);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario(workers, kBodies, 0, kTicks);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

TEST_CASE("State hash survives 1000 runs of randomised worker count and scheduling grain", "[determinism][stress]") {
    constexpr std::size_t kBodies = 8;
    constexpr int kTicks = 10;

    const std::uint64_t reference = run_scenario(1, kBodies, 0, kTicks);

    std::mt19937 rng(12345); // seeds only the TEST's choice of worker count/grain, never the simulation itself
    std::uniform_int_distribution<int> worker_dist(1, 8);
    std::uniform_int_distribution<int> grain_dist(0, 5); // 0 == World's own auto grain

    for (int run = 0; run < 1000; ++run) {
        const int workers = worker_dist(rng);
        const std::size_t grain = static_cast<std::size_t>(grain_dist(rng));
        const std::uint64_t h = run_scenario(workers, kBodies, grain, kTicks);
        if (h != reference) {
            INFO("run=" << run << " workers=" << workers << " grain=" << grain << " hash=0x" << std::hex << h);
            REQUIRE(h == reference);
        }
    }
}
