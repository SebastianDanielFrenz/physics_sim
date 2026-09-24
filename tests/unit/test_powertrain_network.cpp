// tests/unit/test_powertrain_network.cpp — white-box tests for
// ps::drivetrain (PHYS-033 P4 I2 plan section 11.2). core/src is on this
// target's include path (tests/unit/CMakeLists.txt), like the other
// white-box tests (test_articulation_core.cpp etc.) - ps::drivetrain is
// internal, not part of ps_core's public API.
//
// Commit 1 (this file's first landing): case (g), Cholesky against a
// reference solve. Cases (a)-(f) and (h) (nullspace/network/relaxation)
// land with commit 3/4 once those files exist.

#include "drivetrain/dense.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

using namespace ps;
using namespace ps::drivetrain;

namespace {

// Reference solve for a small SPD system via plain Gauss-Jordan with
// partial pivoting (an INDEPENDENT method from cholesky_factor/solve under
// test - not itself part of the production code path) - n <= 8, so a
// simple O(n^3) elimination is more than accurate enough as a cross-check.
template <std::size_t N>
SmallVec<N> reference_solve(const SmallMat<N, N>& A, std::size_t n, const SmallVec<N>& b) {
    // Augmented matrix [A | b] as plain arrays (small, stack only).
    std::array<std::array<real, N + 1>, N> aug{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) aug[i][j] = A(i, j);
        aug[i][n] = b[i];
    }
    for (std::size_t col = 0; col < n; ++col) {
        std::size_t pivot = col;
        real best = std::abs(aug[col][col]);
        for (std::size_t r = col + 1; r < n; ++r) {
            if (std::abs(aug[r][col]) > best) { best = std::abs(aug[r][col]); pivot = r; }
        }
        std::swap(aug[col], aug[pivot]);
        const real d = aug[col][col];
        for (std::size_t j = col; j <= n; ++j) aug[col][j] /= d;
        for (std::size_t r = 0; r < n; ++r) {
            if (r == col) continue;
            const real f = aug[r][col];
            if (f == real(0.0)) continue;
            for (std::size_t j = col; j <= n; ++j) aug[r][j] -= f * aug[col][j];
        }
    }
    SmallVec<N> x;
    x.set_size(n);
    for (std::size_t i = 0; i < n; ++i) x[i] = aug[i][n];
    return x;
}

} // namespace

TEST_CASE("cholesky_factor/cholesky_solve match a reference Gauss-Jordan solve", "[unit][drivetrain][dense]") {
    ps_test::SplitMix64 rng(0xC0FFEE1234ULL);
    constexpr std::size_t kMaxN = 8;

    for (int trial = 0; trial < 2000; ++trial) {
        const std::size_t n = 1 + static_cast<std::size_t>(rng.next_u64() % kMaxN); // 1..8

        // Random SPD: A = M*M^T + 0.5*I (plan section 11.1 batch 2's own
        // construction), M entries in [-1, 1].
        std::array<std::array<real, kMaxN>, kMaxN> M{};
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j) M[i][j] = rng.next_uniform(-1.0, 1.0);

        SmallMat<kMaxN, kMaxN> A;
        A.set_size(n, n);
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                real s = 0.0;
                for (std::size_t k = 0; k < n; ++k) s += M[i][k] * M[j][k];
                A(i, j) = s + (i == j ? real(0.5) : real(0.0));
            }
        }

        SmallVec<kMaxN> b;
        b.set_size(n);
        for (std::size_t i = 0; i < n; ++i) b[i] = rng.next_uniform(-10.0, 10.0);

        SmallMat<kMaxN, kMaxN> L;
        int nonspd_hits = 0;
        const bool ok = cholesky_factor(A, n, L, &nonspd_hits);
        REQUIRE(ok);
        REQUIRE(nonspd_hits == 0);

        SmallVec<kMaxN> x;
        cholesky_solve(L, n, b, x);

        const SmallVec<kMaxN> x_ref = reference_solve(A, n, b);
        for (std::size_t i = 0; i < n; ++i) {
            const real scale = std::max(real(1.0), std::abs(x_ref[i]));
            INFO("trial=" << trial << " n=" << n << " i=" << i);
            CHECK(std::abs(x[i] - x_ref[i]) <= real(1e-13) * scale);
        }

        // L*L^T reproduces A itself (the factorisation identity), not just
        // "solves the same system" - independent of the solve step above.
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j <= i; ++j) {
                real s = 0.0;
                for (std::size_t k = 0; k <= j; ++k) s += L(i, k) * L(j, k);
                CHECK(std::abs(s - A(i, j)) <= real(1e-12) * std::max(real(1.0), std::abs(A(i, j))));
            }
        }
    }
}

TEST_CASE("cholesky_factor regularises a merely-PSD matrix instead of producing a NaN",
          "[unit][drivetrain][dense]") {
    // A rank-deficient PSD matrix (one row duplicated into another, plan
    // section 11.1 batch 3's "degenerate" construction): [[1,1],[1,1]] is
    // PSD (eigenvalues 0 and 2) but not PD - cholesky_factor must
    // regularise rather than take sqrt of a non-positive diagonal.
    SmallMat<8, 8> A;
    A.set_size(2, 2);
    A(0, 0) = 1.0; A(0, 1) = 1.0;
    A(1, 0) = 1.0; A(1, 1) = 1.0;

    SmallMat<8, 8> L;
    int nonspd_hits = 0;
    const bool ok = cholesky_factor(A, 2, L, &nonspd_hits);
    CHECK_FALSE(ok);
    CHECK(nonspd_hits >= 1);
    CHECK(std::isfinite(L(1, 1)));
    CHECK(L(1, 1) > 0.0);
}
