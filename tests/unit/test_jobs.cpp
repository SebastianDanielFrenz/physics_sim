// Unit tests for ps::jobs (PLAN.md D8 scope): parallel_for correctness
// under scatter/gather, task graph barrier ordering, and the debug
// allocation hook (both the "clean" and "caught" cases).

#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/jobs/task_graph.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <numeric>
#include <vector>

using namespace ps::jobs;

TEST_CASE("parallel_for scatter/gather sums every index exactly once", "[jobs]") {
    for (int workers : {1, 2, 4}) {
        JobSystem js(workers);
        constexpr std::size_t kN = 10007; // not a multiple of any obvious grain
        std::vector<long> slots(kN, -1);

        js.parallel_for(kN, 37, [&slots](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) slots[i] = static_cast<long>(i * 2);
        });

        for (std::size_t i = 0; i < kN; ++i) {
            REQUIRE(slots[i] == static_cast<long>(i * 2));
        }
    }
}

TEST_CASE("parallel_batches runs every batch exactly once", "[jobs]") {
    JobSystem js(4);
    constexpr std::size_t kBatches = 500;
    std::vector<std::atomic<int>> hits(kBatches);
    for (auto& h : hits) h.store(0);

    js.parallel_batches(kBatches, [&hits](std::size_t i) { hits[i].fetch_add(1); });

    for (std::size_t i = 0; i < kBatches; ++i) REQUIRE(hits[i].load() == 1);
}

TEST_CASE("nested parallel_for from within a job does not deadlock", "[jobs]") {
    JobSystem js(4);
    std::atomic<int> inner_total{0};

    js.parallel_for(8, 1, [&](std::size_t, std::size_t) {
        js.parallel_for(16, 1, [&](std::size_t, std::size_t) { inner_total.fetch_add(1); });
    });

    REQUIRE(inner_total.load() == 8 * 16);
}

TEST_CASE("TaskGraph runs stages in dependency order", "[jobs]") {
    JobSystem js(4);
    TaskGraph graph;
    std::vector<int> order;
    std::mutex order_m;

    const auto a = graph.add_stage("a", [&] { std::lock_guard<std::mutex> l(order_m); order.push_back(1); });
    const auto b = graph.add_stage("b", [&] { std::lock_guard<std::mutex> l(order_m); order.push_back(2); }, {a});
    graph.add_stage("c", [&] { std::lock_guard<std::mutex> l(order_m); order.push_back(3); }, {b});

    graph.run(js);

    REQUIRE(order == std::vector<int>{1, 2, 3});
}

TEST_CASE("debug_alloc hook stays clean for an allocation-free workload", "[jobs][debug_alloc]") {
    if (!debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan (would be a duplicate-symbol link error against the sanitizer's own overrides) - nothing to check here on this build.");
        return;
    }
    JobSystem js(4);
    debug_alloc::reset();
    debug_alloc::set_armed(true);

    std::vector<double> slots(2000, 0.0);
    js.parallel_for(2000, 20, [&slots](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            // Pure arithmetic, no heap traffic - mirrors a force-element evaluate().
            double v = static_cast<double>(i);
            for (int k = 0; k < 8; ++k) v = v * 1.0000001 + 0.5;
            slots[i] = v;
        }
    });

    debug_alloc::set_armed(false);
    REQUIRE(debug_alloc::violation_count() == 0);
}

namespace {
// A sink the optimizer cannot see through, so the deliberate allocation
// below cannot be elided (the standard permits eliding a new[]/delete[]
// pair with no other observable use, which a release build actually did
// before this fix - test_jobs.cpp's "clean" case passed, but this one
// silently no-op'd instead of allocating, so the hook was never exercised).
std::atomic<void*> g_alloc_sink{nullptr};
} // namespace

TEST_CASE("debug_alloc hook catches an allocation made inside a job", "[jobs][debug_alloc]") {
    if (!debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan (would be a duplicate-symbol link error against the sanitizer's own overrides) - nothing to check here on this build.");
        return;
    }
    JobSystem js(2);
    debug_alloc::reset();
    debug_alloc::set_armed(true);

    js.parallel_for(4, 1, [](std::size_t begin, std::size_t) {
        // Deliberately violate the "no allocation inside jobs" rule to
        // prove the hook actually observes it (it must not crash the
        // process - see ps/jobs/debug_alloc.h). Size depends on a runtime
        // value and the pointer escapes to a global atomic, so the
        // compiler cannot prove the allocation is dead and elide it.
        int* leak = new int[begin + 1];
        leak[0] = static_cast<int>(begin);
        g_alloc_sink.store(leak, std::memory_order_relaxed);
    });

    debug_alloc::set_armed(false);
    REQUIRE(debug_alloc::violation_count() > 0);
    debug_alloc::reset();
    // Only the last of the 4 concurrent chunks' allocations is reachable
    // here (g_alloc_sink is last-write-wins by design, see above); the
    // other 3 are a deliberate handful-of-bytes test-only leak, not worth
    // extra machinery to avoid.
    delete[] static_cast<int*>(g_alloc_sink.exchange(nullptr));
}

TEST_CASE("JobSystem worker_count respects explicit configuration", "[jobs]") {
    JobSystem js1(1);
    REQUIRE(js1.worker_count() == 1);
    JobSystem js4(4);
    REQUIRE(js4.worker_count() == 4);
}

// P1.5b review item: a full ring queue used to abort() the process
// (fail_queue_full). It must now run the overflowing job inline on the
// submitting thread instead - checked here by forcing an unrealistically
// small queue_capacity (job_system.h's second constructor argument) so a
// modest parallel_for reliably overflows it, then asserting (a) every
// index still gets written exactly once (correctness), (b) the fallback
// counter actually saw it fire (the test is exercising what it claims to),
// and (c) the result is identical regardless of worker count, i.e. running
// some chunks inline instead of on a worker changes nothing a correct D8
// scatter->gather caller can observe.
TEST_CASE("A full job queue runs the overflowing job inline instead of aborting", "[jobs]") {
    for (int workers : {1, 2, 4}) {
        JobSystem js(workers, /*queue_capacity=*/4);
        constexpr std::size_t kN = 2000; // many more chunks than the tiny queue can hold at once
        std::vector<long> slots(kN, -1);

        js.parallel_for(kN, 1, [&slots](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) slots[i] = static_cast<long>(i * 3);
        });

        for (std::size_t i = 0; i < kN; ++i) REQUIRE(slots[i] == static_cast<long>(i * 3));

        INFO("workers=" << workers << " fallbacks=" << js.queue_full_fallback_count());
        REQUIRE(js.queue_full_fallback_count() > 0);
    }
}

TEST_CASE("The queue-full inline fallback does not break determinism of the result", "[jobs][determinism]") {
    constexpr std::size_t kN = 500;

    auto run = [](int workers, std::size_t queue_capacity) {
        JobSystem js(workers, queue_capacity);
        std::vector<std::uint64_t> slots(kN, 0);
        js.parallel_for(kN, 1, [&slots](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                // A small deterministic hash of the index - not a plain
                // copy, so a mis-ordered or duplicated write would show up.
                std::uint64_t h = 14695981039346656037ULL ^ i;
                h *= 1099511628211ULL;
                slots[i] = h;
            }
        });
        std::uint64_t hash = 0;
        for (std::uint64_t v : slots) hash = (hash ^ v) * 1099511628211ULL;
        return hash;
    };

    // A generous queue (default) never triggers the fallback; a tiny one
    // (capacity 4) forces it on nearly every submit. Both must agree with
    // each other and across worker counts.
    const std::uint64_t reference = run(1, 0);
    for (int workers : {1, 2, 4, 8}) {
        REQUIRE(run(workers, 0) == reference);
        REQUIRE(run(workers, 4) == reference);
    }
}
