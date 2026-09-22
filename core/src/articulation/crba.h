// core/src/articulation/crba.h — Composite Rigid Body Algorithm: the
// joint-space mass matrix M(q).
//
// Source: R. Featherstone, "Rigid Body Dynamics Algorithms", Springer
// 2008, ch. 3 (mass matrix via composite rigid-body inertias). Reference
// implementation used by tests/unit's ABA-vs-CRBA cross-check (PLAN.md
// P1.5a acceptance: ABA accelerations equal the CRBA solve
// M^-1(tau - C) to 1e-9 relative) - not used on ABA's O(n) hot path.

#pragma once

#include "model.h"
#include "small_matrix.h"

namespace ps::articulation {

// Dense n x n mass matrix, n == model.num_v, row-major flattened.
struct MassMatrix {
    int n = 0;
    std::vector<real> data; // data[r * n + c]
    [[nodiscard]] real at(int r, int c) const { return data[static_cast<std::size_t>(r * n + c)]; }
    real& at(int r, int c) { return data[static_cast<std::size_t>(r * n + c)]; }
};

[[nodiscard]] MassMatrix crba(const Model& model, const ArticulationState& state);

} // namespace ps::articulation
