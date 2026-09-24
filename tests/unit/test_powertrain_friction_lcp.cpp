// tests/unit/test_powertrain_friction_lcp.cpp — brute-force cross-check for
// the box-LCP (PHYS-033 P4 I2 plan section 11.1, owner ruling Q1). Compares
// solve_box_lcp's finite block-principal-pivoting result against an
// exhaustive enumeration of all 3^k {Lo, Free, Hi} partitions, using the
// EXACT SAME regularised problem (build_regularized_problem) and partition
// evaluator (evaluate_partition) the production solver itself calls -
// see box_lcp.h's own comment on why those two are exposed.
//
// Simplification note (this commit): the plan's full spec additionally
// asks to verify "the sets are identical except at degenerate indices" -
// this file instead verifies the stronger/simpler property directly (BPP's
// lambda matches a feasible brute-force lambda within tolerance, and every
// feasible brute-force lambda agrees with every other within tolerance),
// which implies set-agreement everywhere the solution is not itself
// degenerate, without needing a separate near-a-bound/near-zero-residual
// classifier.

#include "drivetrain/box_lcp.h"
#include "drivetrain/dense.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using ps_test::SplitMix64;

namespace {

struct Problem {
    int k = 0;
    std::array<real, kMaxLcpK * kMaxLcpK> W{};
    std::array<real, kMaxLcpK> q{};
    std::array<real, kMaxLcpK> c{};
};

// Batch 1 (plan 11.1.1): W = M*M^T + 1e-3*I, M in U[-1,1]; q = s*U[-1,1];
// c = U(0.01,1]. `duplicate_row1`: batch 3's construction - M's row 1 is
// overwritten with row 0 BEFORE forming W, giving an exactly rank-deficient
// (PSD, not PD) Gram matrix before the 1e-3*I diagonal is added - "one
// row/column of W duplicated into another", exercising the epsilon
// regularisation in build_regularized_problem.
Problem make_batch1(SplitMix64& rng, int k, real s, bool duplicate_row1 = false) {
    Problem p;
    p.k = k;
    std::array<std::array<real, kMaxLcpK>, kMaxLcpK> M{};
    for (int i = 0; i < k; ++i)
        for (int j = 0; j < k; ++j) M[i][j] = rng.next_uniform(-1.0, 1.0);
    if (duplicate_row1 && k >= 2) M[1] = M[0];

    for (int i = 0; i < k; ++i) {
        for (int j = 0; j < k; ++j) {
            real sum = 0.0;
            for (int t = 0; t < k; ++t) sum += M[i][t] * M[j][t];
            p.W[static_cast<std::size_t>(i * k + j)] = sum + (i == j ? real(1e-3) : real(0.0));
        }
    }
    for (int i = 0; i < k; ++i) p.q[static_cast<std::size_t>(i)] = s * rng.next_uniform(-1.0, 1.0);
    for (int i = 0; i < k; ++i) p.c[static_cast<std::size_t>(i)] = rng.next_uniform(0.01, 1.0);
    return p;
}

// Batch 2 (plan 11.1.1): "physical" - random SPD A (n<=4), B entries from a
// small discrete set including a random ratio, W = h*B*A^-1*B^T.
Problem make_batch2(SplitMix64& rng, int k, real s) {
    Problem p;
    p.k = k;
    const int n = 1 + static_cast<int>(rng.next_u64() % 4); // 1..4

    std::array<std::array<real, 4>, 4> N{};
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) N[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = rng.next_uniform(-1.0, 1.0);
    std::array<std::array<real, 4>, 4> A{};
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            real sum = 0.0;
            for (int t = 0; t < n; ++t) sum += N[static_cast<std::size_t>(i)][static_cast<std::size_t>(t)] * N[static_cast<std::size_t>(j)][static_cast<std::size_t>(t)];
            A[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = sum + (i == j ? real(0.5) : real(0.0));
        }
    }
    // Invert A (n<=4) via Gauss-Jordan with partial pivoting.
    std::array<std::array<real, 8>, 4> aug{};
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) aug[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = A[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
        aug[static_cast<std::size_t>(i)][static_cast<std::size_t>(n + i)] = 1.0;
    }
    for (int col = 0; col < n; ++col) {
        int piv = col;
        real best = std::abs(aug[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)]);
        for (int r = col + 1; r < n; ++r) {
            const real v = std::abs(aug[static_cast<std::size_t>(r)][static_cast<std::size_t>(col)]);
            if (v > best) { best = v; piv = r; }
        }
        std::swap(aug[static_cast<std::size_t>(col)], aug[static_cast<std::size_t>(piv)]);
        const real d = aug[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)];
        for (int j = 0; j < 2 * n; ++j) aug[static_cast<std::size_t>(col)][static_cast<std::size_t>(j)] /= d;
        for (int r = 0; r < n; ++r) {
            if (r == col) continue;
            const real f = aug[static_cast<std::size_t>(r)][static_cast<std::size_t>(col)];
            if (f == 0.0) continue;
            for (int j = 0; j < 2 * n; ++j) aug[static_cast<std::size_t>(r)][static_cast<std::size_t>(j)] -= f * aug[static_cast<std::size_t>(col)][static_cast<std::size_t>(j)];
        }
    }
    std::array<std::array<real, 4>, 4> Ainv{};
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) Ainv[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = aug[static_cast<std::size_t>(i)][static_cast<std::size_t>(n + j)];

    const real ratio = rng.next_uniform(1.0, 15.0);
    const std::array<real, 7> choices = {0.0, 1.0, -1.0, 0.5, -0.5, ratio, -ratio};
    std::array<std::array<real, 4>, kMaxLcpK> B{};
    for (int i = 0; i < k; ++i)
        for (int j = 0; j < n; ++j) B[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = choices[rng.next_u64() % choices.size()];

    constexpr real h = 1.0 / 960.0;
    std::array<std::array<real, 4>, kMaxLcpK> BA{};
    for (int i = 0; i < k; ++i) {
        for (int j = 0; j < n; ++j) {
            real sum = 0.0;
            for (int t = 0; t < n; ++t) sum += B[static_cast<std::size_t>(i)][static_cast<std::size_t>(t)] * Ainv[static_cast<std::size_t>(t)][static_cast<std::size_t>(j)];
            BA[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = sum;
        }
    }
    for (int i = 0; i < k; ++i) {
        for (int j = 0; j < k; ++j) {
            real sum = 0.0;
            for (int t = 0; t < n; ++t) sum += BA[static_cast<std::size_t>(i)][static_cast<std::size_t>(t)] * B[static_cast<std::size_t>(j)][static_cast<std::size_t>(t)];
            p.W[static_cast<std::size_t>(i * k + j)] = h * sum;
        }
    }
    for (int i = 0; i < k; ++i) p.q[static_cast<std::size_t>(i)] = s * rng.next_uniform(-1.0, 1.0);
    for (int i = 0; i < k; ++i) p.c[static_cast<std::size_t>(i)] = rng.next_uniform(0.01, 1.0);
    return p;
}

// Brute force: enumerate all 3^k {Lo,Free,Hi} assignments through the
// SAME evaluate_partition() the production solver uses, collect every
// feasible lambda.
struct BruteForceResult {
    std::vector<std::array<real, kMaxLcpK>> feasible_lambdas;
};

BruteForceResult brute_force(const Problem& p) {
    LcpWorkspace ws;
    build_regularized_problem(p.W.data(), p.q.data(), p.c.data(), p.k, ws);

    BruteForceResult result;
    int pow3 = 1;
    for (int i = 0; i < p.k; ++i) pow3 *= 3;

    std::array<LcpSet, kMaxLcpK> sets{};
    std::array<int, kMaxLcpK> infeasible{};
    for (int code = 0; code < pow3; ++code) {
        int c2 = code;
        for (int i = 0; i < p.k; ++i) {
            const int digit = c2 % 3;
            c2 /= 3;
            sets[static_cast<std::size_t>(i)] = digit == 0 ? LcpSet::Lo : (digit == 1 ? LcpSet::Free : LcpSet::Hi);
        }
        const int ni = evaluate_partition(p.q.data(), p.c.data(), p.k, sets.data(), ws, infeasible.data());
        if (ni == 0) {
            std::array<real, kMaxLcpK> lam{};
            for (int i = 0; i < p.k; ++i) lam[static_cast<std::size_t>(i)] = std::clamp(ws.lam[static_cast<std::size_t>(i)], -p.c[static_cast<std::size_t>(i)], p.c[static_cast<std::size_t>(i)]);
            result.feasible_lambdas.push_back(lam);
        }
    }
    return result;
}

real max_c(const Problem& p) {
    real m = 0.0;
    for (int i = 0; i < p.k; ++i) m = std::max(m, p.c[static_cast<std::size_t>(i)]);
    return m;
}

// stage2_count / stage3_count: how many cases needed solve_box_lcp's stage
// 2 (Murty least-index fallback) / stage 3 (exhaustive active-set
// enumeration) respectively - see box_lcp.h's LcpResult::stage_used. Since
// stage 3 was added (coordinator's second ruling: a box LCP's standard-LCP
// reformulation is only P0, not P, so stage 2's Murty fallback is not
// itself guaranteed finite), cap_hit should be mathematically unreachable
// for these SPD-Wr problems, so EVERY case - no exceptions - is asserted
// to match brute force exactly again, restoring the original test contract.
void run_and_check(const Problem& p, real tol, int trial, std::array<int, 9>& max_iters_by_k, int& stage2_count, int& stage3_count,
                    int batch = -1) {
    const BruteForceResult bf = brute_force(p);
    INFO("trial=" << trial << " k=" << p.k << " batch=" << batch);
    REQUIRE(!bf.feasible_lambdas.empty());

    // Every feasible brute-force lambda should agree with the first one
    // (uniqueness for a PD/eps-regularised W).
    for (std::size_t idx = 1; idx < bf.feasible_lambdas.size(); ++idx) {
        for (int i = 0; i < p.k; ++i) {
            CHECK(std::abs(bf.feasible_lambdas[idx][static_cast<std::size_t>(i)] - bf.feasible_lambdas[0][static_cast<std::size_t>(i)]) <=
                  tol * (1.0 + max_c(p)));
        }
    }

    LcpWorkspace ws;
    std::array<LcpSet, kMaxLcpK> sets{};
    sets.fill(LcpSet::Free);
    std::array<real, kMaxLcpK> lambda_out{};
    const LcpResult r = solve_box_lcp(p.W.data(), p.q.data(), p.c.data(), p.k, sets.data(), lambda_out.data(), ws);
    max_iters_by_k[static_cast<std::size_t>(p.k)] = std::max(max_iters_by_k[static_cast<std::size_t>(p.k)], r.iterations);

    // With stage 3 (exhaustive enumeration) in place, cap_hit means EVERY
    // one of the 3^k fixed-order partitions failed evaluate_partition's
    // feasibility check for an SPD Wr - mathematically impossible (the
    // box-LCP solution is unique and must be one of them) - so this is a
    // hard assertion again, not a WARN-and-skip: if it ever fires, stop
    // and report exactly like this comment's history did for stage 1/2.
    CHECK_FALSE(r.cap_hit);
    if (r.stage_used >= 2) ++stage2_count;
    if (r.stage_used >= 3) ++stage3_count;

    for (int i = 0; i < p.k; ++i) {
        CHECK(std::abs(lambda_out[static_cast<std::size_t>(i)] - bf.feasible_lambdas[0][static_cast<std::size_t>(i)]) <= tol * (1.0 + max_c(p)));
    }
}

} // namespace

// FINDING HISTORY (PHYS-033 P4 I2, coordinator ruling on commit 2's cap-hit
// finding, in three rounds):
//   1. The fixed single-stage pivot cap (2k+2, owner ruling Q1/F1) was
//      found occasionally insufficient by itself - measured ~0.25% of
//      batch-1 cases at n=1e5, far worse for batch 2 (its own case below,
//      ~11%). Root-caused as NOT a cycling/correctness bug (a large-cap
//      re-solve of every affected case always converged and always
//      matched brute force). Coordinator's fix: add stage 2, Murty's
//      LEAST-index single-pivot method, for up to a further 2^k pivots
//      (finite for a P-matrix - Wr is SPD after epsilon regularisation).
//   2. Own diagnosis while wiring stage 2 up: the first version pivoted on
//      the LARGEST infeasible index (copy-pasted from stage 1's own
//      fallback rule) instead of the LEAST index Murty's method actually
//      requires - see box_lcp.cpp's own comment on the exact case (seed
//      0xFEEDFACE5EEDULL, trial 79244, k=3) that caught it as a genuine
//      cycle. Fixing the pivot rule drastically cut the cap-hit rate but
//      did not make it exactly zero - a small residual remained (measured
//      locally: 0/10000 fast case, 2/100000 long case, 3/4000 batch 2).
//   3. Coordinator's second ruling explained why: a box LCP's standard-LCP
//      reformulation is only P0, not P, so Murty's finite-termination
//      guarantee never actually covered stage 2 here - and a cap-hit
//      "last feasible iterate" is not an acceptable production answer
//      (can violate stick/slip complementarity - the clutch lock-up spike
//      this solver exists to prevent). Fix: add stage 3, exhaustive
//      active-set enumeration (all 3^k partitions in a fixed order,
//      return the first feasible one - box_lcp.cpp's own comment). With
//      stage 3 in place cap_hit is mathematically unreachable for an SPD
//      Wr, so the test contract is restored: EVERY case (no exceptions)
//      is asserted to match brute force exactly, and stage2_count/
//      stage3_count report how many cases needed which fallback.
TEST_CASE("box-LCP: block principal pivoting matches brute force (batches 1/3/4)", "[unit][drivetrain][lcp]") {
    SplitMix64 rng(0x5EED1234ABCDULL);
    constexpr int kCases = 10000;
    std::array<int, 9> max_iters_by_k{};
    int stage2_count = 0;
    int stage3_count = 0;

    for (int trial = 0; trial < kCases; ++trial) {
        const int k = 1 + static_cast<int>(rng.next_u64() % 6); // 1..6
        const int batch = static_cast<int>(rng.next_u64() % 3); // 0=batch1, 1=batch3(degenerate), 2=batch4(warm-start)
        const std::array<real, 3> s_choices = {1e-3, 1.0, 1e3};
        const real s = s_choices[rng.next_u64() % s_choices.size()];

        if (batch == 0) {
            run_and_check(make_batch1(rng, k, s), 1e-12, trial, max_iters_by_k, stage2_count, stage3_count, batch);
        } else if (batch == 1) {
            run_and_check(make_batch1(rng, k, s, /*duplicate_row1=*/true), 1e-6, trial, max_iters_by_k, stage2_count, stage3_count, batch);
        } else {
            // Batch 4: warm-start stress - solve once from all-Free, then
            // again from the first solution's sets with q perturbed 1%.
            Problem p = make_batch1(rng, k, s);
            LcpWorkspace ws;
            std::array<LcpSet, kMaxLcpK> sets{};
            sets.fill(LcpSet::Free);
            std::array<real, kMaxLcpK> lambda1{};
            const LcpResult r1 = solve_box_lcp(p.W.data(), p.q.data(), p.c.data(), p.k, sets.data(), lambda1.data(), ws);
            CHECK_FALSE(r1.cap_hit);
            max_iters_by_k[static_cast<std::size_t>(p.k)] = std::max(max_iters_by_k[static_cast<std::size_t>(p.k)], r1.iterations);
            if (r1.stage_used >= 2) ++stage2_count;
            if (r1.stage_used >= 3) ++stage3_count;

            Problem p2 = p;
            for (int i = 0; i < p2.k; ++i) p2.q[static_cast<std::size_t>(i)] *= 1.01;
            run_and_check(p2, 1e-12, trial, max_iters_by_k, stage2_count, stage3_count); // sets already warm from r1 (kept in `sets`)

            std::array<real, kMaxLcpK> lambda2{};
            const LcpResult r2 = solve_box_lcp(p2.W.data(), p2.q.data(), p2.c.data(), p2.k, sets.data(), lambda2.data(), ws);
            CHECK_FALSE(r2.cap_hit);
            max_iters_by_k[static_cast<std::size_t>(p2.k)] = std::max(max_iters_by_k[static_cast<std::size_t>(p2.k)], r2.iterations);
            if (r2.stage_used >= 2) ++stage2_count;
            if (r2.stage_used >= 3) ++stage3_count;
        }
    }

    WARN("cases reaching stage 2 (Murty fallback): " << stage2_count << "/" << kCases);
    WARN("cases reaching stage 3 (exhaustive enumeration): " << stage3_count << "/" << kCases);
    for (int k = 1; k <= 6; ++k) {
        WARN("max iterations for k=" << k << ": " << max_iters_by_k[static_cast<std::size_t>(k)] << " (stage-1 cap=" << (2 * k + 2)
                                      << ", stage-2 adds up to " << (1 << k) << " more)");
    }
}

// [long]: the spec's own 10^5-case size (plan section 11.1), batches 1/3
// only (batch 2 is its own, separately-tagged case below - see the finding
// history above). No longer [!mayfail]: with stage 3 in place, every case
// must agree with brute force exactly, no exceptions. 3^6=729 enumerations
// per k=6 case (stage-3-only cost; ordinarily stage 1/2 resolve it far
// sooner) makes this too slow for the debug/sanitizer legs (CLAUDE.md
// "Test selection" - [long] is excluded there and on the release-style
// fast legs, run only under -Full).
TEST_CASE("box-LCP: block principal pivoting matches brute force (10^5 cases)", "[unit][drivetrain][lcp][long]") {
    SplitMix64 rng(0xFEEDFACE5EEDULL);
    constexpr int kCases = 100000;
    std::array<int, 9> max_iters_by_k{};
    int stage2_count = 0;
    int stage3_count = 0;

    for (int trial = 0; trial < kCases; ++trial) {
        const int k = 1 + static_cast<int>(rng.next_u64() % 6);
        const int batch = static_cast<int>(rng.next_u64() % 2);
        const std::array<real, 3> s_choices = {1e-3, 1.0, 1e3};
        const real s = s_choices[rng.next_u64() % s_choices.size()];

        if (batch == 0) run_and_check(make_batch1(rng, k, s), 1e-12, trial, max_iters_by_k, stage2_count, stage3_count, 0);
        else run_and_check(make_batch1(rng, k, s, /*duplicate_row1=*/true), 1e-6, trial, max_iters_by_k, stage2_count, stage3_count, 2);
    }
    WARN("[long] cases reaching stage 2 (Murty fallback): " << stage2_count << "/" << kCases);
    WARN("[long] cases reaching stage 3 (exhaustive enumeration): " << stage3_count << "/" << kCases);
    for (int k = 1; k <= 6; ++k) {
        WARN("[long] max iterations for k=" << k << ": " << max_iters_by_k[static_cast<std::size_t>(k)] << " (stage-1 cap=" << (2 * k + 2) << ")");
    }
}

// Batch 2 ("physical": W = h*B*A^-1*B^T, plan section 11.1.1) is its OWN
// case - PHYS-033 P4 I2, coordinator ruling on commit 2's cap-hit finding
// (see the finding history above the batches-1/3/4 case). FINDING: with k
// up to 6 and A's dimension n capped at 4 (the plan's own construction), B
// is frequently rank-deficient for the LCP's k, and combined with the
// smallest q-scale (s=1e-3) this produced the worst cap-hit rates of any
// batch at every stage (~11% hit stage 1's cap; a residual ~0.075% still
// hit stage 2's cap even with the least-index fix). With stage 3
// (exhaustive enumeration) in place, every case - no exceptions - is
// asserted to match brute force exactly again.
TEST_CASE("box-LCP: block principal pivoting matches brute force (batch 2: physical)",
          "[unit][drivetrain][lcp]") {
    SplitMix64 rng(0xB2B2B2B2ULL);
    constexpr int kCases = 4000;
    std::array<int, 9> max_iters_by_k{};
    int stage2_count = 0;
    int stage3_count = 0;

    for (int trial = 0; trial < kCases; ++trial) {
        const int k = 1 + static_cast<int>(rng.next_u64() % 6);
        const std::array<real, 3> s_choices = {1e-3, 1.0, 1e3};
        const real s = s_choices[rng.next_u64() % s_choices.size()];
        Problem p = make_batch2(rng, k, s);

        run_and_check(p, 1e-12, trial, max_iters_by_k, stage2_count, stage3_count, /*batch=*/1);
    }
    WARN("batch 2 cases reaching stage 2 (Murty fallback): " << stage2_count << "/" << kCases);
    WARN("batch 2 cases reaching stage 3 (exhaustive enumeration): " << stage3_count << "/" << kCases);
    for (int k = 1; k <= 6; ++k) {
        WARN("batch 2 max iterations for k=" << k << ": " << max_iters_by_k[static_cast<std::size_t>(k)] << " (stage-1 cap=" << (2 * k + 2) << ")");
    }
}

TEST_CASE("box-LCP: k=1 closed form and c=0 caller-level filtering", "[unit][drivetrain][lcp]") {
    // k=1 closed form: lambda = clamp(-q/W, -c, c).
    SplitMix64 rng(0x1111ULL);
    for (int trial = 0; trial < 200; ++trial) {
        const real W = rng.next_uniform(0.01, 10.0);
        const real q = rng.next_uniform(-5.0, 5.0);
        const real c = rng.next_uniform(0.01, 5.0);
        LcpWorkspace ws;
        LcpSet set = LcpSet::Free;
        real lambda_out = 0.0;
        const LcpResult r = solve_box_lcp(&W, &q, &c, 1, &set, &lambda_out, ws);
        CHECK_FALSE(r.cap_hit);
        const real expected = std::clamp(-q / W, -c, c);
        CHECK(std::abs(lambda_out - expected) <= 1e-12 * (1.0 + c));
    }

    // Caller-level: a row with c=0 must be excluded by the CALLER before
    // calling solve_box_lcp (box_lcp.h's own header comment) - this case
    // just documents/checks that a k=1 system evaluated at c=0 clamps
    // lambda to exactly 0 rather than producing a NaN, so a caller that
    // forgets to filter still degrades safely.
    {
        const real W = 5.0, q = 3.0, c = 0.0;
        LcpWorkspace ws;
        LcpSet set = LcpSet::Free;
        real lambda_out = 123.0;
        solve_box_lcp(&W, &q, &c, 1, &set, &lambda_out, ws);
        CHECK(lambda_out == 0.0);
    }
}

