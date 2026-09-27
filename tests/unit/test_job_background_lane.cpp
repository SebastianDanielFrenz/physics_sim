// Unit tests for JobSystem's background lane (terrain prefetch ring, C3):
// start_background_lane / submit_background / drain_background.
//
// Contract checked here (ps/jobs/job_system.h):
// - lane jobs run only on lane threads, never on a worker or the caller;
// - wait()/parallel_for (incl. nested waits that help drain the pool) never
//   pop or run a lane job, and drain_idle() ignores lane work;
// - drain_background() returns only after every submitted lane job ran;
// - submit_background() refuses (returns false, runs nothing) when the lane
//   is not started or its fixed queue is full;
// - destroying a JobSystem with queued lane jobs is clean (queued jobs are
//   discarded unrun, a running one finishes);
// - submit_background() is allocation-free, and debug_alloc sees an
//   allocation made inside a lane job.

#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

using namespace ps::jobs;

namespace {

// Blocks the (single-thread) lane until release is set; started tells the
// test the lane has popped it, so the lane queue is otherwise empty.
struct Latch {
    std::atomic<bool> started{false};
    std::atomic<bool> release{false};
};

void submit_latch(JobSystem& js, Latch& latch) {
    Latch* l = &latch;
    REQUIRE(js.submit_background(JobFn([l] {
        l->started.store(true, std::memory_order_release);
        while (!l->release.load(std::memory_order_acquire)) std::this_thread::yield();
    })));
    while (!latch.started.load(std::memory_order_acquire)) std::this_thread::yield();
}

std::atomic<void*> g_lane_alloc_sink{nullptr};

} // namespace

TEST_CASE("Background lane jobs run only on a lane thread", "[jobs]") {
    JobSystem js(2);
    REQUIRE_FALSE(js.background_lane_started());
    js.start_background_lane();
    REQUIRE(js.background_lane_started());
    REQUIRE(js.background_lane_thread_count() == 1);
    REQUIRE(js.worker_count() == 2); // the lane is not a worker
    REQUIRE_FALSE(js.on_background_lane());

    constexpr int kN = 64;
    struct Seen {
        std::atomic<int> on_lane{0};
        std::atomic<int> on_worker{0};
        std::thread::id id{};
    };
    std::array<Seen, kN> seen{};
    for (int i = 0; i < kN; ++i) {
        Seen* s = &seen[static_cast<std::size_t>(i)];
        const JobSystem* pjs = &js;
        REQUIRE(js.submit_background(JobFn([s, pjs] {
            s->on_lane.store(pjs->on_background_lane() ? 1 : 0, std::memory_order_relaxed);
            s->on_worker.store(pjs->on_worker_thread() ? 1 : 0, std::memory_order_relaxed);
            s->id = std::this_thread::get_id();
        })));
    }
    js.drain_background();

    const std::thread::id lane_id = seen[0].id;
    REQUIRE(lane_id != std::this_thread::get_id());
    for (const Seen& s : seen) {
        REQUIRE(s.on_lane.load() == 1);
        REQUIRE(s.on_worker.load() == 0);
        REQUIRE(s.id == lane_id); // one lane thread
    }

    // Tick jobs never run on the lane.
    std::array<std::atomic<int>, 256> tick_on_lane{};
    js.parallel_for(256, 1, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) tick_on_lane[i].store(js.on_background_lane() ? 1 : 0);
    });
    for (const auto& v : tick_on_lane) REQUIRE(v.load() == 0);
}

TEST_CASE("wait(), nested parallel_for and drain_idle never run or wait for a lane job", "[jobs]") {
    JobSystem js(4);
    js.start_background_lane();
    Latch latch;
    submit_latch(js, latch); // the lane is now busy until released

    std::atomic<int> lane_ran{0};
    std::atomic<int>* pr = &lane_ran;
    for (int i = 0; i < 16; ++i) {
        REQUIRE(js.submit_background(JobFn([pr] { pr->fetch_add(1, std::memory_order_relaxed); })));
    }

    // Plenty of tick work with nested waits: workers blocked in a nested
    // wait() help drain own/steal/injector queues - never the lane queue.
    std::atomic<long> sum{0};
    js.parallel_for(64, 1, [&](std::size_t, std::size_t) {
        js.parallel_for(32, 1, [&](std::size_t b, std::size_t e) {
            sum.fetch_add(static_cast<long>(e - b), std::memory_order_relaxed);
        });
    });
    REQUIRE(sum.load() == 64 * 32);
    REQUIRE(lane_ran.load() == 0);

    // drain_idle() counts tick jobs only: it must return while the lane is
    // still blocked (a helper thread with a deadline, so a regression fails
    // instead of hanging the test binary).
    std::atomic<bool> idle_done{false};
    std::thread drainer([&] {
        js.drain_idle();
        idle_done.store(true, std::memory_order_release);
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!idle_done.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    const bool drained_while_lane_busy = idle_done.load(std::memory_order_acquire);
    latch.release.store(true, std::memory_order_release);
    drainer.join();
    REQUIRE(drained_while_lane_busy);

    js.drain_background();
    REQUIRE(lane_ran.load() == 16);
}

TEST_CASE("drain_background waits for every lane job; submit_background refuses when not started or full", "[jobs]") {
    SECTION("not started") {
        JobSystem js(1);
        std::atomic<int> ran{0};
        std::atomic<int>* pr = &ran;
        REQUIRE_FALSE(js.submit_background(JobFn([pr] { pr->fetch_add(1); })));
        js.drain_background(); // no lane: returns at once
        REQUIRE(ran.load() == 0);
    }
    SECTION("drain after many jobs, one and two lane threads") {
        for (int threads : {1, 2}) {
            JobSystem js(2);
            js.start_background_lane(threads);
            js.start_background_lane(threads + 3); // second call is a no-op
            REQUIRE(js.background_lane_thread_count() == threads);
            std::atomic<int> ran{0};
            std::atomic<int>* pr = &ran;
            constexpr int kN = 500;
            for (int i = 0; i < kN; ++i) {
                REQUIRE(js.submit_background(JobFn([pr] {
                    volatile int spin = 0;
                    for (int k = 0; k < 200; ++k) spin = spin + k;
                    pr->fetch_add(1, std::memory_order_relaxed);
                })));
            }
            js.drain_background();
            REQUIRE(ran.load() == kN);
        }
    }
    SECTION("full queue") {
        JobSystem js(1, /*queue_capacity=*/4);
        js.start_background_lane();
        Latch latch;
        submit_latch(js, latch); // popped: the queue is empty again
        std::atomic<int> ran{0};
        std::atomic<int>* pr = &ran;
        for (int i = 0; i < 4; ++i) REQUIRE(js.submit_background(JobFn([pr] { pr->fetch_add(1); })));
        REQUIRE_FALSE(js.submit_background(JobFn([pr] { pr->fetch_add(100); })));
        latch.release.store(true, std::memory_order_release);
        js.drain_background();
        REQUIRE(ran.load() == 4);
        // Room again after the drain.
        REQUIRE(js.submit_background(JobFn([pr] { pr->fetch_add(1); })));
        js.drain_background();
        REQUIRE(ran.load() == 5);
    }
}

TEST_CASE("Destroying a JobSystem with queued lane jobs is clean", "[jobs]") {
    std::atomic<int> ran{0};
    Latch latch;
    {
        auto js = std::make_unique<JobSystem>(2);
        js->start_background_lane();
        submit_latch(*js, latch);
        std::atomic<int>* pr = &ran;
        for (int i = 0; i < 32; ++i) REQUIRE(js->submit_background(JobFn([pr] { pr->fetch_add(1); })));
        // Release the running latch job only well after the destructor has
        // begun: it must wait for the running job, then discard the queue.
        std::thread releaser([&latch] {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            latch.release.store(true, std::memory_order_release);
        });
        js.reset();
        releaser.join();
    }
    REQUIRE(latch.release.load());
    REQUIRE(ran.load() == 0); // queued jobs discarded unrun

    // A started but idle lane, and a never-started one, also tear down.
    { JobSystem idle(1); idle.start_background_lane(2); }
    { JobSystem never(1); }
}

TEST_CASE("submit_background is allocation-free and lane-job allocations are seen by debug_alloc", "[jobs][debug_alloc]") {
    if (!debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    JobSystem js(2);
    js.start_background_lane(); // setup: allocates, outside the armed window

    std::array<std::atomic<int>, 64> accepted{};
    std::atomic<int> ran{0};
    std::atomic<int>* pr = &ran;
    debug_alloc::reset();
    debug_alloc::set_armed(true);
    // Submitted from inside worker jobs, where the hook counts allocations.
    js.parallel_for(64, 1, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            accepted[i].store(js.submit_background(JobFn([pr] { pr->fetch_add(1, std::memory_order_relaxed); })) ? 1 : 0);
        }
    });
    js.drain_background();
    debug_alloc::set_armed(false);
    REQUIRE(debug_alloc::violation_count() == 0);
    for (const auto& a : accepted) REQUIRE(a.load() == 1);
    REQUIRE(ran.load() == 64);

    // Negative control: an allocation inside a LANE job is counted.
    debug_alloc::reset();
    debug_alloc::set_armed(true);
    std::atomic<int> sz{3};
    std::atomic<int>* psz = &sz;
    REQUIRE(js.submit_background(JobFn([psz] {
        int* leak = new int[static_cast<std::size_t>(psz->load()) + 1];
        leak[0] = 1;
        g_lane_alloc_sink.store(leak, std::memory_order_relaxed);
    })));
    js.drain_background();
    debug_alloc::set_armed(false);
    REQUIRE(debug_alloc::violation_count() > 0);
    debug_alloc::reset();
    delete[] static_cast<int*>(g_lane_alloc_sink.exchange(nullptr));
}
