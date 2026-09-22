// tests/bench/bench_contact_coupling — PLAN.md P1.5b measurement (f): cost
// per tick for each contact strategy, and how it scales with contact count.
// Plain main() (not Catch2), following bench_force_elements.cpp's pattern,
// so its measured numbers are always on stdout; registered with CTest via
// add_test() as a smoke check (it must run to completion without crashing/
// asserting) but does not itself assert a scaling bound - there is no
// PLAN.md acceptance number for this measurement yet, unlike
// bench_force_elements' "6x from 1 to 10 workers".
//
// Rig: N independent single-DOF prismatic "rail" articulations (same shape
// as tests/unit/test_articulation_contact.cpp), each with its own contact
// sphere, spaced well apart in x so they never interact with each other -
// isolates the cost of N contact-bearing articulations from any N^2
// broadphase effect a shared cluster would add. Run single-threaded
// (WorldConfig::job_workers = 1), 240 Hz tick / 960 Hz substep, on the
// release preset for a meaningful number (debug/asan numbers are not
// representative - same convention as every other bench/study in this
// directory).

#include "headless_env.h"
#include "ps/world/world.h"

#include <chrono>
#include <cstdio>
#include <vector>

using namespace ps;
using ps::articulation::ArticulationDesc;
using ps::articulation::ContactStrategy;
using ps::articulation::LinkDesc;

namespace {

BodyId make_ground(World& world, real half_x) {
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{half_x, 5, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 0.8;
    ground.pose.position = Vec3{0, 0, -0.5};
    return world.create_body(ground);
}

void add_rail(World& world, ContactStrategy strategy, real x) {
    ArticulationDesc desc;
    desc.root.floating = false;
    desc.root.pose = Pose{Vec3{x, 0, 1.0}, Quat::identity()};
    desc.contact_strategy = strategy;

    LinkDesc link;
    link.parent = -1;
    link.joint = ps::articulation::JointType::Prismatic;
    link.axis = Vec3::unit_z();
    link.mass = 1.0;
    link.com = Vec3::zero();
    link.inertia = Mat3::diagonal(0.01, 0.01, 0.01);
    link.contact.enabled = true;
    link.contact.shape = SphereShape{0.3};
    link.contact.material.stiffness = 1.0e6;
    link.contact.material.stiffness_exponent = 1.5;
    link.contact.material.damping = 1.0e6;
    link.contact.material.friction = 0.8;
    desc.links.push_back(link);
    world.create_articulation(desc);
}

// Average per-tick wall-clock cost, in ms, over num_ticks after a short
// warm-up (thread start-up, cold caches - same rationale as
// bench_force_elements.cpp).
double measure(ContactStrategy strategy, int n, int num_ticks) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -9.81};
    cfg.job_workers = 1;
    World world(cfg);

    make_ground(world, static_cast<real>(n) * 2.0 + 5.0);
    for (int i = 0; i < n; ++i) add_rail(world, strategy, static_cast<real>(i) * 2.0);

    for (int i = 0; i < 60; ++i) world.step(); // warm-up: let contacts engage, caches warm

    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < num_ticks; ++i) world.step();
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count() / static_cast<double>(num_ticks);
}

} // namespace

int main() {
    ps_tools::install_headless_crt_handlers();

    const std::vector<int> counts = {1, 2, 4, 8, 16};
    constexpr int kTicks = 500;

    std::printf("bench_contact_coupling: %d ticks per configuration, single worker, 240 Hz tick / 960 Hz substep\n", kTicks);
    for (int n : counts) {
        const double ms_compliant = measure(ContactStrategy::Compliant, n, kTicks);
        const double ms_proxy = measure(ContactStrategy::ProxyBody, n, kTicks);
        std::printf("bench_contact_coupling: contacts=%2d  Compliant=%.5f ms/tick (%.3f us/contact)  ProxyBody=%.5f ms/tick (%.3f us/contact)\n", n,
                    ms_compliant, ms_compliant * 1000.0 / n, ms_proxy, ms_proxy * 1000.0 / n);
    }

    return 0;
}
