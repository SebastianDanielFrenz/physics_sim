// core/src/drivetrain/dense.h — small, fixed-capacity dense linear algebra
// for the shaft-network solver (PHYS-033 P4 I2 plan section 3): SmallVec/
// SmallMat over std::array (no heap, PLAN.md D8 - "fixed-capacity
// containers in the job"), plus a Cholesky factor/solve pair. Internal
// (ps::drivetrain, not a public header). Header-only, as the plan's own
// file table requires.
//
// Every cluster's per-substep workspace (network.h's Cluster) is sized to
// the network's own fixed capacities (kMaxDofs = 8, kMaxClusterNodes = 16,
// kMaxRows = 8 - see network.h) at CONSTRUCTION time (Powertrain's
// constructor, sim thread, outside any job) and never resized inside a
// job - SmallVec<N>/SmallMat<R,C> below exist so that sizing is a compile-
// time template parameter (std::array storage) while the RUNTIME
// dimension (n <= N, rows <= R, cols <= C) can still vary per cluster/
// topology-state without a second allocation.

#pragma once

#include "ps/types.h"

#include <array>
#include <cstddef>
#include <cmath>

namespace ps::drivetrain {

// A vector of up to N reals; n (<= N) is the RUNTIME size. Never resizes
// its backing storage - only n changes.
template <std::size_t N>
struct SmallVec {
    std::array<real, N> data{};
    std::size_t n = 0;

    [[nodiscard]] std::size_t size() const { return n; }
    void set_size(std::size_t sz) { n = sz; }

    real& operator[](std::size_t i) { return data[i]; }
    const real& operator[](std::size_t i) const { return data[i]; }

    void zero() { data.fill(real(0.0)); }
};

// An R x C dense matrix, row-major, fixed stride C; rows/cols (<= R/<= C)
// are the RUNTIME dimensions. Never resizes its backing storage.
template <std::size_t R, std::size_t C>
struct SmallMat {
    std::array<real, R * C> data{};
    std::size_t rows = 0;
    std::size_t cols = 0;

    void set_size(std::size_t r, std::size_t c) {
        rows = r;
        cols = c;
    }

    real& operator()(std::size_t i, std::size_t j) { return data[i * C + j]; }
    const real& operator()(std::size_t i, std::size_t j) const { return data[i * C + j]; }

    void zero() { data.fill(real(0.0)); }
};

// Cholesky factorisation of an n x n SPD (or nearly-SPD) matrix A into a
// lower-triangular L with A = L*L^T - PHYS-033 P4 I2 plan section 5.3, no
// pivoting, fixed loop order (n <= N <= 8 in every caller, so this is a
// few dozen flops). `A` and `L` may alias different SmallMat<N,N> objects
// only (never the same object - L is written strictly after A(j,k) for
// k <= j is read, so aliasing A into L in place would corrupt later rows'
// reads of A; every caller in this codebase passes distinct objects).
//
// A diagonal that goes non-positive after subtracting the accumulated sum
// of squares (floating-point round-off on a matrix that is only PSD, e.g.
// two friction rows on the same relative velocity duplicated verbatim -
// plan section 11.1 batch 3's "degenerate" case) is regularised per the
// plan's own formula rather than left to produce a NaN; `nonspd_hits`
// (optional) counts how many diagonals needed this, mirroring
// network.h's own telemetry channel of the same name - this should be
// exactly 0 for every ordinary run (any nonzero count is a build-time
// telemetry signal, not a silent normal path - see the I2 plan's stop
// condition 5).
//
// Returns false iff at least one diagonal needed regularisation (i.e. A
// was not, to working precision, SPD) - true means the factorisation is
// the exact Cholesky factor of A itself.
template <std::size_t N>
bool cholesky_factor(const SmallMat<N, N>& A, std::size_t n, SmallMat<N, N>& L, int* nonspd_hits = nullptr) {
    L.zero();
    L.set_size(n, n);

    real trace = 0.0;
    for (std::size_t i = 0; i < n; ++i) trace += A(i, i);

    bool ok = true;
    for (std::size_t j = 0; j < n; ++j) {
        real s = A(j, j);
        for (std::size_t k = 0; k < j; ++k) s -= L(j, k) * L(j, k);

        const real ajj_original = A(j, j);
        if (s <= real(1e-14) * ajj_original) {
            s = std::max(s, real(1e-12) * trace / static_cast<real>(n) + real(1e-300));
            ok = false;
            if (nonspd_hits) ++*nonspd_hits;
        }
        L(j, j) = std::sqrt(s);

        for (std::size_t i = j + 1; i < n; ++i) {
            real v = A(i, j);
            for (std::size_t k = 0; k < j; ++k) v -= L(i, k) * L(j, k);
            L(i, j) = v / L(j, j);
        }
    }
    return ok;
}

// Solves L*L^T*x = b (forward then backward substitution) given the
// Cholesky factor L from cholesky_factor above. `x` may alias `b` (x is
// written only after the corresponding b[i]/y[i] has been consumed in
// each pass - forward pass reads b[i] once before writing y (a distinct
// local), backward pass reads y[i] once before writing x(i); if the
// caller aliases x and b, the forward pass's y must still be a separate
// buffer, which it is here).
template <std::size_t N>
void cholesky_solve(const SmallMat<N, N>& L, std::size_t n, const SmallVec<N>& b, SmallVec<N>& x) {
    SmallVec<N> y;
    y.set_size(n);
    for (std::size_t i = 0; i < n; ++i) {
        real v = b[i];
        for (std::size_t k = 0; k < i; ++k) v -= L(i, k) * y[k];
        y[i] = v / L(i, i);
    }
    x.set_size(n);
    for (std::size_t ii = 0; ii < n; ++ii) {
        const std::size_t i = n - 1 - ii;
        real v = y[i];
        for (std::size_t k = i + 1; k < n; ++k) v -= L(k, i) * x[k];
        x[i] = v / L(i, i);
    }
}

} // namespace ps::drivetrain
