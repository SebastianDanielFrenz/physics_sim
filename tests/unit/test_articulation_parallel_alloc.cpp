// PHYS-009: PLAN.md D8's "no allocation inside jobs" checked STRICTLY -
// exactly zero, not "within a tolerance of a noisy baseline". Three known
// violations were found and fixed (see each source file's own comment for
// the debug_alloc backtrace that pinned it down):
//
//  1. ps::jobs::JobSystem's per-worker/injector queues were std::deque -
//     calling parallel_for from INSIDE a running job (System::step_all,
//     World's force-element/vehicle-island stages) pushed onto the
//     calling worker's own deque while OTHER workers stole from its front,
//     and libstdc++'s deque allocates a new internal node every time
//     push_back crosses a block boundary a pop_front just vacated - a
//     permanent per-block churn, not a one-time warm-up cost, reproduced
//     even by a no-op nested parallel_for with no articulation/World code
///    at all (Windows was unaffected: MSVC STL's much larger per-block
//     capacity for this small element type never needed to cross a
//     boundary at the chunk counts this codebase submits). Fixed by
//     replacing every such queue with a fixed-capacity ring buffer sized
//     once at JobSystem construction (job_system.h's RingQueue) - see
//     job_system.h/.cpp.
//  2. A closure-bearing articulation (e.g. the four-bar) allocated several
//     vectors per substep inside loop_closure.cpp's constraint solve
//     (ClosureLinearAlgebra's J/minv_jrow/A and the smaller buffers
//     building them) - PHYS-009 Step 1 preallocated the open-chain aba()
//     scratch but explicitly left this path unoptimised. Fixed by
//     LoopClosureScratch (loop_closure.h/.cpp), sized once per Instance at
//     construction, same pattern as AbaScratch.
//  3. Two IRigidBackend methods World's pipeline calls every tick
//     (live_body_ids(), drain_contact_events()) returned a fresh
//     std::vector by value, and the contact listener's own drain()
//     additionally used swap() to hand its buffer away, permanently
//     stripping its capacity - both allocated inside a job every tick a
//     live body / live contact existed. Fixed with *_into() variants
//     writing into a caller-owned, already-capacity'd buffer
//     (i_rigid_backend.h, jolt_backend.cpp, world.cpp).
//
// Every case below arms ps::jobs::debug_alloc and asserts the violation
// count is EXACTLY 0 after a warm-up phase (letting every buffer above
// reach its steady-state capacity) - no baseline measurement, no
// tolerance. hook_available() is false under ASan/TSan (debug_alloc's own
// operator new/delete overrides are compiled out there, see debug_alloc.h);
// those legs WARN and skip, exactly as before.

#include "articulation/loop_closure.h"
#include "articulation/model.h"
#include "articulation/system.h"
#include "counter_force_element.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>

using namespace ps;
using ps::jobs::debug_alloc::hook_available;
using ps::jobs::JobSystem;

namespace {

// Runs `body` as exactly one outer job, submitted from the calling thread
// (never one of JobSystem's own workers) - the same shape TaskGraph gives
// a World pipeline stage - and waits for it. Anything `body` does that
// itself calls JobSystem::parallel_for is therefore the "nested
// parallel_for called from inside a running job" case violation #1 above
// was about.
template <typename Fn>
void run_nested_once(JobSystem& js, Fn&& body) {
    JobSystem::WaitGroup outer;
    js.submit([&body] { body(); }, &outer);
    js.wait(outer);
}

// Warms up (unarmed) so every ring buffer / scratch vector this workload
// touches reaches its steady-state capacity, then takes ONE armed sample
// and returns its exact violation count.
template <typename Fn>
std::uint64_t measure_nested(JobSystem& js, Fn&& body, int warmup_iters, int measured_iters) {
    for (int i = 0; i < warmup_iters; ++i) run_nested_once(js, body);

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < measured_iters; ++i) run_nested_once(js, body);
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t v = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    return v;
}

articulation::ArticulationDesc make_open_chain_desc(int num_links, bool floating, double x_offset) {
    articulation::ArticulationDesc desc;
    desc.root.floating = floating;
    desc.root.pose = Pose{Vec3{x_offset, 0, floating ? 5.0 : 0.0}, Quat::identity()};
    if (floating) {
        desc.root.mass = 2.0;
        desc.root.com = Vec3::zero();
        desc.root.inertia = Mat3::diagonal(0.1, 0.1, 0.1);
    }
    for (int i = 0; i < num_links; ++i) {
        articulation::LinkDesc link;
        link.parent = i - 1; // -1 for i==0: attaches to the (possibly floating) root
        link.joint = (i % 2 == 0) ? articulation::JointType::Revolute : articulation::JointType::Prismatic;
        link.axis = Vec3::unit_y();
        if (i > 0) link.joint_frame_in_parent.position = Vec3{0, 0, -0.4};
        link.mass = 1.0;
        link.com = Vec3{0, 0, -0.2};
        link.inertia = Mat3::diagonal(0.02, 0.02, 0.01);
        link.q0 = 0.05 * static_cast<double>(i + 1);
        desc.links.push_back(link);
    }
    return desc;
}

// A Grashof crank-rocker four-bar (same geometry as
// tests/unit/test_articulation_four_bar.cpp's make_desc - a loop-closure-
// bearing articulation, the case violation #2 above was about), offset in
// x so several instances do not overlap.
articulation::ArticulationDesc make_four_bar_desc(double x_offset) {
    using namespace articulation;
    constexpr double kA = 2.0, kB = 5.0, kC = 4.0, kD = 4.0;

    ArticulationDesc desc;
    desc.root.floating = false;
    desc.root.pose = Pose{Vec3{x_offset, 0, 0}, Quat::identity()};

    LinkDesc crank;
    crank.parent = -1;
    crank.joint = articulation::JointType::Revolute;
    crank.axis = Vec3::unit_y();
    crank.mass = 1.0;
    crank.com = Vec3{0, 0, -kA / 2.0};
    crank.inertia = Mat3::diagonal(kA * kA / 12.0, kA * kA / 12.0, 1e-4);
    desc.links.push_back(crank);

    LinkDesc coupler;
    coupler.parent = 0;
    coupler.joint = articulation::JointType::Revolute;
    coupler.joint_frame_in_parent.position = Vec3{0, 0, -kA};
    coupler.axis = Vec3::unit_y();
    coupler.mass = 1.0;
    coupler.com = Vec3{0, 0, -kB / 2.0};
    coupler.inertia = Mat3::diagonal(kB * kB / 12.0, kB * kB / 12.0, 1e-4);
    desc.links.push_back(coupler);

    LinkDesc rocker;
    rocker.parent = -1;
    rocker.joint = articulation::JointType::Revolute;
    rocker.joint_frame_in_parent.position = Vec3{kD, 0, 0};
    rocker.axis = Vec3::unit_y();
    rocker.mass = 1.0;
    rocker.com = Vec3{0, 0, -kC / 2.0};
    rocker.inertia = Mat3::diagonal(kC * kC / 12.0, kC * kC / 12.0, 1e-4);
    desc.links.push_back(rocker);

    LoopClosureDesc lc;
    lc.type = LoopClosureType::Point;
    lc.link_a = 1;
    lc.anchor_a = Vec3{0, 0, -kB};
    lc.link_b = 2;
    lc.anchor_b = Vec3{0, 0, -kC};
    desc.loop_closures.push_back(lc);

    return desc;
}

// Matches System::step_all's own auto-grain formula (system.cpp) exactly,
// so the pure-nested-parallel_for case submits precisely the same number
// of chunks System::step_all would for the same (n, workers).
std::size_t auto_grain(std::size_t n, int workers) {
    const int w = std::max(1, workers);
    return std::max<std::size_t>(1, (n + static_cast<std::size_t>(w) * 4 - 1) / (static_cast<std::size_t>(w) * 4));
}

} // namespace

TEST_CASE("A pure nested parallel_for allocates nothing at 1/2/4/10 workers", "[jobs][debug_alloc][articulation]") {
    if (!hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    for (int workers : {1, 2, 4, 10}) {
        JobSystem js(workers);
        constexpr std::size_t kN = 30;
        const std::size_t grain = auto_grain(kN, workers);

        const std::uint64_t violations = measure_nested(js, [&js, grain] {
            js.parallel_for(kN, grain, [](std::size_t begin, std::size_t end) {
                for (std::size_t i = begin; i < end; ++i) {
                    volatile std::size_t sink = i; // touch, nothing else
                    (void)sink;
                }
            });
        }, /*warmup_iters=*/500, /*measured_iters=*/200);

        INFO("workers=" << workers);
        REQUIRE(violations == 0);
    }
}

TEST_CASE("Stepping many articulations (open-chain, floating-base, four-bar) across many workers allocates nothing",
          "[jobs][articulation][debug_alloc]") {
    if (!hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    for (int workers : {1, 2, 4, 10}) {
        JobSystem js(workers);
        constexpr std::size_t kN = 30;
        const std::size_t grain = auto_grain(kN, workers);

        articulation::System sys;
        // 30 articulations: open-chain (1..6 links), two floating-base,
        // and two closure-bearing four-bar linkages - System::create is
        // allowed to allocate (never called from inside a job here); only
        // step_all() below must not allocate at all.
        for (std::size_t a = 0; a < kN; ++a) {
            if (a == 7 || a == 21) {
                sys.create(make_four_bar_desc(static_cast<double>(a) * 20.0));
                continue;
            }
            const int links = 1 + static_cast<int>(a % 6);
            sys.create(make_open_chain_desc(links, /*floating=*/a == 5 || a == 17, static_cast<double>(a) * 2.0));
        }
        articulation::InstanceConfig cfg;
        cfg.gravity = Vec3{0, 0, -9.81};
        cfg.substep_rate_hz = 960.0;
        constexpr real kTickDt = real(1.0) / real(240.0);

        // warmup/measured are far smaller than the other two cases' (500/200):
        // 30 articulations incl. two RK4 four-bar solves is real per-substep
        // cost, not measurement noise to average out - the check itself is
        // an exact equality now (not a statistical baseline comparison), so
        // it needs only enough repeats to let every scratch buffer reach its
        // steady-state capacity and to have a fair chance at catching a
        // scheduling-dependent intermittent allocation, not hundreds of
        // samples. Found to matter concretely: at the previous 500/200 this
        // one test case alone cost ~20 s per debug-leg run (PHYS-009 Step 3
        // debug-leg timing investigation).
        const std::uint64_t violations = measure_nested(js, [&sys, &js, &cfg, grain] {
            sys.step_all(js, cfg, kTickDt, grain);
            sys.clear_external_wrenches();
        }, /*warmup_iters=*/30, /*measured_iters=*/30);

        INFO("workers=" << workers);
        REQUIRE(violations == 0);
    }
}

TEST_CASE("A full World::step with articulations, force elements and Jolt bodies (incl. a resting contact) allocates nothing",
          "[world][articulation][debug_alloc][jobs]") {
    if (!hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    for (int workers : {1, 2, 4, 10}) {
        WorldConfig cfg;
        cfg.job_workers = workers;
        World world(cfg);

        // A static ground plate plus a dynamic sphere settling onto it -
        // exercises the Jolt contact-listener path (violation #3's
        // drain_contact_events_into), not just a contact-free body.
        BodyDesc ground;
        ground.shape = BoxShape{Vec3{50, 50, 0.5}};
        ground.motion = BodyMotionType::Static;
        ground.pose.position = Vec3{0, 0, -0.5};
        world.create_body(ground);

        BodyDesc falling;
        falling.shape = SphereShape{0.5};
        falling.motion = BodyMotionType::Dynamic;
        falling.mass = 1.0;
        falling.pose.position = Vec3{0, 0, 0.5};
        falling.allow_sleep = false;
        world.create_body(falling);

        // A single-link open-chain articulation, driven by a
        // CounterForceElement attached to its link Frame (exercises the
        // vehicle-island and force-element/gather stages together).
        articulation::ArticulationDesc adesc;
        adesc.root.floating = false;
        articulation::LinkDesc link;
        link.parent = -1;
        link.joint = articulation::JointType::Revolute;
        link.axis = Vec3::unit_y();
        link.mass = 1.0;
        link.com = Vec3{0, 0, -0.2};
        link.inertia = Mat3::diagonal(0.02, 0.02, 0.01);
        link.q0 = 0.1;
        adesc.links.push_back(link);
        const ArticulationId aid = world.create_articulation(adesc);

        ps_test::CounterForceElement fe(world.link_frame(aid, 0), 1);
        world.add_force_element(&fe);

        // Warm-up: lets the sphere actually settle onto the ground (real,
        // steady contacts) and every scratch/ring-buffer/queue reach its
        // stable capacity.
        for (int i = 0; i < 500; ++i) world.step();

        ps::jobs::debug_alloc::reset();
        ps::jobs::debug_alloc::set_armed(true);
        for (int i = 0; i < 100; ++i) world.step();
        ps::jobs::debug_alloc::set_armed(false);
        const std::uint64_t violations = ps::jobs::debug_alloc::violation_count();
        ps::jobs::debug_alloc::reset();

        INFO("workers=" << workers);
        REQUIRE(violations == 0);
    }
}
