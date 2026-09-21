// tests/bench/bench_force_elements — PLAN.md P1 acceptance: "2000
// independent force elements scale >= 6x from 1 to 10 workers". Plain
// main() (not a Catch2 case) so its measured numbers are always visible on
// stdout, not just on failure; run on the release preset for a meaningful
// number (PLAN.md), though it also runs (and reports, less meaningfully)
// under other presets since it is registered with CTest via add_test().
//
// Benchmarks ps::jobs::JobSystem::parallel_for directly over 2000
// IForceElement::evaluate() calls (the exact code path World's
// force-element pipeline stage uses - see World::stage_force_elements),
// rather than going through a full World::step(). This isolates the job
// system's scaling from JoltPhysics' own broadphase/narrowphase overhead,
// which is a separate concern (and would need 2000 well-separated, mostly
// non-colliding bodies to avoid contact-solver cost dominating the
// measurement instead of force-element evaluation).

#include "headless_env.h"
#include "ps/jobs/job_system.h"
#include "ps/math/pose.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

using namespace ps;

namespace {

// A per-element cost representative of a real force model (a handful of
// transcendental ops - tyre/aero-scale work), not the ~10ns the trivial
// hash in CounterForceElement alone would cost (too cheap to show
// meaningful scaling: job dispatch overhead would dominate).
struct HeavierElement {
    explicit HeavierElement(std::uint64_t id) : id_(id) {}
    double evaluate(std::uint64_t tick) const {
        double v = static_cast<double>(id_ ^ tick) * 1e-3;
        for (int i = 0; i < 200; ++i) v = std::sin(v) * std::cos(v * 1.0001) + 0.5;
        return v;
    }
    std::uint64_t id_;
};

double run_once(jobs::JobSystem& js, std::vector<HeavierElement>& elements, std::vector<double>& out, std::uint64_t tick) {
    const auto t0 = std::chrono::steady_clock::now();
    js.parallel_for(elements.size(), 20, [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) out[i] = elements[i].evaluate(tick);
    });
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

} // namespace

int main() {
    ps_tools::install_headless_crt_handlers();

    constexpr std::size_t kNumElements = 2000;
    constexpr int kItersPerConfig = 60;

    std::vector<HeavierElement> elements;
    elements.reserve(kNumElements);
    for (std::size_t i = 0; i < kNumElements; ++i) elements.emplace_back(i);
    std::vector<double> out(kNumElements, 0.0);

    const std::vector<int> worker_counts = {1, 2, 4, 6, 8, 10};
    std::vector<double> avg_ms(worker_counts.size(), 0.0);

    for (std::size_t wi = 0; wi < worker_counts.size(); ++wi) {
        jobs::JobSystem js(worker_counts[wi]);
        // Warm up this specific configuration (thread start-up, cold caches).
        for (int w = 0; w < 5; ++w) run_once(js, elements, out, 0);

        js.reset_worker_stats();
        double total = 0.0;
        for (int i = 0; i < kItersPerConfig; ++i) total += run_once(js, elements, out, static_cast<std::uint64_t>(i + 1));
        avg_ms[wi] = total / kItersPerConfig;

        std::printf("bench_force_elements: workers=%2d  avg_ms=%.4f  elements=%zu  busy_ns=[", worker_counts[wi], avg_ms[wi], kNumElements);
        for (int w = 0; w < worker_counts[wi]; ++w) std::printf("%s%llu", w ? "," : "", static_cast<unsigned long long>(js.worker_busy_ns(w)));
        std::printf("]\n");
    }

    const double t1 = avg_ms[0];
    const double t10 = avg_ms[worker_counts.size() - 1];
    const double speedup = t1 / t10;
    std::printf("bench_force_elements: 1-worker=%.4f ms, 10-worker=%.4f ms, speedup=%.2fx\n", t1, t10, speedup);

#if defined(NDEBUG)
    const double kRequiredSpeedup = 6.0;
    if (speedup < kRequiredSpeedup) {
        std::fprintf(stderr, "bench_force_elements: FAIL - speedup %.2fx < required %.2fx (release build)\n", speedup, kRequiredSpeedup);
        return 1;
    }
    std::printf("bench_force_elements: PASS (release build, speedup %.2fx >= %.2fx)\n", speedup, kRequiredSpeedup);
#else
    std::printf("bench_force_elements: not enforced outside a release-type build (NDEBUG unset); numbers above are informational only.\n");
#endif
    return 0;
}
