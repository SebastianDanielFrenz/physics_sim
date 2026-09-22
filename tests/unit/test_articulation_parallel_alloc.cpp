// PHYS-009 Step 1 rule 2 ("No allocation inside jobs"): System::step_all
// (core/src/articulation/system.cpp) now runs one job per articulation
// (batched via parallel_for) instead of a single-threaded sequential loop.
// Every open-chain articulation's step_tick() must therefore run with zero
// heap allocation - it used to allocate through aba()'s (then thread_local)
// scratch, several per-tick/per-RK4-stage vectors in instance.cpp, and a
// fresh forward-kinematics pose vector per gravity evaluation; all of that
// scratch is now preallocated per Instance at construction time (see
// instance.h's "Step-time scratch" member block) and reused in place.
//
// This exercises exactly the debug_alloc hook (ps/jobs/debug_alloc.h) the
// same way tests/unit/test_jobs.cpp's "allocation-free workload" case does,
// but through World::step() with a real WorldConfig::job_workers pool and a
// mixed set of open-chain articulations (fixed-root chains of different
// lengths and a floating-base chain) stepped for many ticks - enough that a
// one-time, amortised-away allocation could not hide inside the "first call
// per worker" pattern the old thread_local scratch had.
//
// A closure-bearing articulation (e.g. a four-bar linkage) is deliberately
// NOT included here: loop_closure.cpp's constraint solve still allocates
// several vectors per call (ClosureLinearAlgebra's J/minv_jrow/A buffers) -
// a pre-existing cost this pass did not touch (see loop_closure.h's
// solve_constrained_dynamics_into comment) - so it would fail this
// specific check while being entirely correct dynamically (proven by
// test_articulation_four_bar_energy.cpp, unaffected by this test).

#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

using namespace ps;
using ps::jobs::debug_alloc::hook_available;

namespace {

void add_open_chain(World& world, int num_links, bool floating, double x_offset) {
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
    world.create_articulation(desc);
}

} // namespace

TEST_CASE("Stepping many articulations across many workers allocates nothing (debug_alloc hook)", "[jobs][articulation][debug_alloc]") {
    if (!hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    // Generous warm-up: work-stealing scheduling (PLAN.md D8's own "no
    // guarantee about which worker runs which chunk") means a given
    // worker's queue may not grow to its steady-state capacity until it
    // has actually been handed a chunk at least once - under random
    // scheduling that can take more than a handful of ticks to happen for
    // every worker, especially at higher worker counts where there are
    // more distinct queues to warm up. Large and cheap relative to the
    // test's total cost (these are small open chains).
    constexpr int kWarmupTicks = 500;
    constexpr int kMeasuredTicks = 50;

    // Baseline: World::step()'s own pipeline (freeze/scene_queries/force_
    // elements/gather/solver_seam/backend_step/terrain/snapshot, each
    // submitted through TaskGraph as its own job - task_graph.cpp) already
    // allocates a small, steady, PER-TICK amount with zero bodies and zero
    // articulations at all - reproduced here, unrelated to and unchanged
    // by anything in PHYS-009 Step 1 (this pass never touched World's own
    // stages, the job system, or TaskGraph). Measured per worker count
    // since it may itself depend on the pool size; subtracted below so
    // this test isolates articulation stepping's OWN allocation behaviour,
    // which is what PLAN.md D8/PHYS-009 Step 1 rule 2 is actually about.
    auto measure = [](int workers, World& world) {
        for (int t = 0; t < kWarmupTicks; ++t) world.step();
        ps::jobs::debug_alloc::reset();
        ps::jobs::debug_alloc::set_armed(true);
        for (int t = 0; t < kMeasuredTicks; ++t) world.step();
        ps::jobs::debug_alloc::set_armed(false);
        const std::uint64_t v = ps::jobs::debug_alloc::violation_count();
        ps::jobs::debug_alloc::reset();
        (void)workers;
        return v;
    };

    for (int workers : {1, 2, 4, 10}) {
        WorldConfig cfg;
        cfg.tick_rate_hz = 240.0;
        cfg.substep_rate_hz = 960.0;
        cfg.gravity = Vec3{0, 0, -9.81};
        cfg.job_workers = workers;

        World baseline_world(cfg);
        const std::uint64_t baseline = measure(workers, baseline_world);

        World world(cfg);
        // 30 open-chain articulations, deliberately uneven sizes (so the
        // parallel_for's auto-grain batches mix small and large instances
        // across jobs), including two floating-base chains - construction
        // itself (Instance::Instance) is allowed to allocate; only
        // World::step() below must not allocate any MORE than the
        // zero-articulation baseline does.
        for (int a = 0; a < 30; ++a) {
            const int links = 1 + (a % 6);
            add_open_chain(world, links, /*floating=*/a == 5 || a == 17, static_cast<double>(a) * 2.0);
        }
        const std::uint64_t with_articulations = measure(workers, world);

        INFO("workers=" << workers << " baseline=" << baseline << " with_articulations=" << with_articulations);
        REQUIRE(with_articulations == baseline);
    }
}
