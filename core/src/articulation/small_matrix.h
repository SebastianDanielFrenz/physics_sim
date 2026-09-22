// core/src/articulation/small_matrix.h — dense matrix up to 6x6, used for
// the articulated-body inertia's joint-space "D" block (Featherstone,
// "Rigid Body Dynamics Algorithms", Springer 2008, ch. 7, the U/D/u
// triple in the articulated-body algorithm) - at most 6x6 because only a
// floating base (6 DoF) needs more than a 1x1 here. Correct scalar code
// first (PLAN.md D5); this is not a hot-path type worth SIMD.

#pragma once

#include "ps/types.h"

#include <array>
#include <cstddef>

namespace ps::articulation {

constexpr int kMaxJointDof = 6;

struct SmallMatrix {
    int rows = 0;
    int cols = 0;
    std::array<std::array<real, kMaxJointDof>, kMaxJointDof> m{};

    static SmallMatrix zero(int r, int c) {
        SmallMatrix out;
        out.rows = r;
        out.cols = c;
        for (auto& row : out.m) row.fill(real(0.0));
        return out;
    }

    real& at(int r, int c) { return m[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]; }
    [[nodiscard]] real at(int r, int c) const { return m[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]; }

    [[nodiscard]] SmallMatrix transposed() const {
        SmallMatrix out = zero(cols, rows);
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c) out.at(c, r) = at(r, c);
        return out;
    }

    // Gauss-Jordan inverse with partial pivoting. Square matrices only
    // (rows == cols), sized 1..6 here. D (Featherstone ch. 7) is always
    // symmetric positive definite for a valid articulated-body inertia, so
    // this never hits a singular pivot in practice; a near-zero pivot
    // falls back to leaving that row/col at 0 rather than dividing by it,
    // which is a defensive guard, not a modelled behaviour.
    [[nodiscard]] SmallMatrix inverse() const {
        const int n = rows;
        SmallMatrix a = *this;
        SmallMatrix inv = zero(n, n);
        for (int i = 0; i < n; ++i) inv.at(i, i) = real(1.0);

        for (int col = 0; col < n; ++col) {
            int pivot_row = col;
            real pivot_val = a.at(col, col) < real(0) ? -a.at(col, col) : a.at(col, col);
            for (int r = col + 1; r < n; ++r) {
                const real v = a.at(r, col) < real(0) ? -a.at(r, col) : a.at(r, col);
                if (v > pivot_val) { pivot_val = v; pivot_row = r; }
            }
            if (pivot_val < real(1e-14)) continue; // degenerate column; leave as-is (defensive)
            if (pivot_row != col) {
                std::swap(a.m[static_cast<std::size_t>(col)], a.m[static_cast<std::size_t>(pivot_row)]);
                std::swap(inv.m[static_cast<std::size_t>(col)], inv.m[static_cast<std::size_t>(pivot_row)]);
            }
            const real pivot = a.at(col, col);
            for (int c = 0; c < n; ++c) { a.at(col, c) /= pivot; inv.at(col, c) /= pivot; }
            for (int r = 0; r < n; ++r) {
                if (r == col) continue;
                const real factor = a.at(r, col);
                if (factor == real(0.0)) continue;
                for (int c = 0; c < n; ++c) {
                    a.at(r, c) -= factor * a.at(col, c);
                    inv.at(r, c) -= factor * inv.at(col, c);
                }
            }
        }
        return inv;
    }
};

} // namespace ps::articulation
