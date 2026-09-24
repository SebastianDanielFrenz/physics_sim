// core/src/drivetrain/nullspace.h — deterministic null-space basis for a
// cluster's rigid kinematic constraints (PHYS-033 P4 I2 plan section 4.3).
// Build-time only (Powertrain's constructor, sim thread, outside any job -
// PLAN.md D8 allows ordinary allocation here, unlike the per-substep
// runtime path in network.h/box_lcp.h). Internal (ps::drivetrain).
//
// Given the rigid rows G (m x N_c, each "sum coef*omega = 0") for one
// topology state's cluster and each node's inertia J, produces a basis T_s
// (N_c x n) with omega = T_s * nu (nu the n reduced coordinates) and
// G * T_s == 0 exactly. The algorithm is a FIXED, deterministic Gauss-
// Jordan elimination (never depends on iteration order over an unordered
// container - D7) so the SAME graph always produces the SAME basis, on
// every platform and every build.

#pragma once

#include "ps/types.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ps::drivetrain {

// One rigid constraint row: sparse (node index, coefficient) pairs, in the
// order the caller declared them (network.h's Powertrain builder walks
// components in ARRAY order - PLAN.md D7).
struct ConstraintRow {
    std::vector<std::pair<int, real>> terms; // (node index, coefficient)
};

struct NullspaceBasis {
    // T_s, N_c x n, row-major (N_c = node_count passed to build_nullspace).
    std::vector<real> T;
    std::vector<int> free_nodes; // ascending node index, size n
    int node_count = 0;
    int rank = 0; // n = node_count - (kept, independent rows)
    int redundant_rows = 0; // rows dropped as linearly dependent (diagnostic)

    [[nodiscard]] real at(int node, int col) const { return T[static_cast<std::size_t>(node) * static_cast<std::size_t>(rank) + static_cast<std::size_t>(col)]; }
};

// `rows`: the cluster's rigid constraint rows, in component-array order
// (plan section 4.2). `node_inertia`: J_i for every node 0..node_count-1
// IN THIS CLUSTER's own local numbering (the caller remaps global NodeIds
// to a dense 0..node_count-1 range per cluster before calling this).
// Returns the rank n (== out.rank == out.free_nodes.size()).
inline int build_nullspace(const std::vector<ConstraintRow>& rows, const std::vector<real>& node_inertia,
                            int node_count, NullspaceBasis& out) {
    out.node_count = node_count;
    out.redundant_rows = 0;

    // col_order: nodes sorted by (J_i == 0 ? 0 : 1, node_index) - prefer
    // eliminating massless nodes first (plan section 4.3).
    std::vector<int> col_order(static_cast<std::size_t>(node_count));
    for (int i = 0; i < node_count; ++i) col_order[static_cast<std::size_t>(i)] = i;
    std::stable_sort(col_order.begin(), col_order.end(), [&](int a, int b) {
        const int ka = node_inertia[static_cast<std::size_t>(a)] == real(0.0) ? 0 : 1;
        const int kb = node_inertia[static_cast<std::size_t>(b)] == real(0.0) ? 0 : 1;
        if (ka != kb) return ka < kb;
        return a < b;
    });

    // Dense working copy R (rows.size() x node_count) - build time only,
    // std::vector is fine here (D8's "no allocation" rule is about the
    // per-substep JOB, not construction).
    const std::size_t m = rows.size();
    std::vector<std::vector<real>> R(m, std::vector<real>(static_cast<std::size_t>(node_count), real(0.0)));
    for (std::size_t r = 0; r < m; ++r) {
        for (const auto& [node, coef] : rows[r].terms) R[r][static_cast<std::size_t>(node)] += coef;
    }

    std::vector<bool> is_pivot(static_cast<std::size_t>(node_count), false);
    std::vector<int> pivot_of_row(m, -1);
    std::vector<std::size_t> kept_rows; // indices into R that survived (in original row order)

    for (std::size_t r = 0; r < m; ++r) {
        // Eliminate existing pivots from row r.
        for (std::size_t kr = 0; kr < kept_rows.size(); ++kr) {
            const std::size_t p = kept_rows[kr];
            const int c = pivot_of_row[p];
            const real coef = R[r][static_cast<std::size_t>(c)];
            if (coef != real(0.0)) {
                for (int j = 0; j < node_count; ++j) R[r][static_cast<std::size_t>(j)] -= coef * R[p][static_cast<std::size_t>(j)];
            }
        }

        real max_abs = 0.0;
        for (int j = 0; j < node_count; ++j) max_abs = std::max(max_abs, std::abs(R[r][static_cast<std::size_t>(j)]));
        const real tol = real(1e-12) * max_abs;

        int chosen_col = -1;
        for (int c : col_order) {
            if (!is_pivot[static_cast<std::size_t>(c)] && std::abs(R[r][static_cast<std::size_t>(c)]) > tol) { chosen_col = c; break; }
        }
        if (chosen_col < 0) {
            ++out.redundant_rows;
            continue; // linearly dependent given the rows kept so far - drop it
        }

        const real pivot_val = R[r][static_cast<std::size_t>(chosen_col)];
        for (int j = 0; j < node_count; ++j) R[r][static_cast<std::size_t>(j)] /= pivot_val;
        is_pivot[static_cast<std::size_t>(chosen_col)] = true;
        pivot_of_row[r] = chosen_col;

        for (std::size_t kr = 0; kr < kept_rows.size(); ++kr) {
            const std::size_t p = kept_rows[kr];
            const real coef = R[p][static_cast<std::size_t>(chosen_col)];
            if (coef != real(0.0)) {
                for (int j = 0; j < node_count; ++j) R[p][static_cast<std::size_t>(j)] -= coef * R[r][static_cast<std::size_t>(j)];
            }
        }
        kept_rows.push_back(r);
    }

    out.free_nodes.clear();
    for (int i = 0; i < node_count; ++i) {
        if (!is_pivot[static_cast<std::size_t>(i)]) out.free_nodes.push_back(i);
    }
    const int n = static_cast<int>(out.free_nodes.size());
    out.rank = n;

    out.T.assign(static_cast<std::size_t>(node_count) * static_cast<std::size_t>(std::max(n, 1)), real(0.0));
    for (int j = 0; j < n; ++j) {
        const int f = out.free_nodes[static_cast<std::size_t>(j)];
        out.T[static_cast<std::size_t>(f) * static_cast<std::size_t>(n) + static_cast<std::size_t>(j)] = real(1.0);
    }
    for (std::size_t kr = 0; kr < kept_rows.size(); ++kr) {
        const std::size_t r = kept_rows[kr];
        const int c = pivot_of_row[r];
        for (int j = 0; j < n; ++j) {
            const int f = out.free_nodes[static_cast<std::size_t>(j)];
            out.T[static_cast<std::size_t>(c) * static_cast<std::size_t>(n) + static_cast<std::size_t>(j)] = -R[r][static_cast<std::size_t>(f)];
        }
    }
    return n;
}

} // namespace ps::drivetrain
