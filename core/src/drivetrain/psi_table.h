// drivetrain/psi_table.h - the tabulated isentropic flow function Psi(pr) on a
// uniform grid (pr = i/n, n = table.size() - 1; SimulatedEngineMaps::psi,
// built with ps::math::pow at construction). One lookup shared by every
// orifice in the mean-value engine: the throttle (engine_simulated.cpp) and,
// since PHYS-033 P4 I7d D2, the turbo blow-off valve (turbo_pair.h) - the
// valve reuses the table, it does not duplicate the formula. A direct index,
// no search; + - * / only.
#pragma once

#include "ps/types.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace ps::drivetrain {

// pr is clamped to [0, 1]; dpsi_dpr (optional) receives the segment slope.
inline real psi_table_lookup(const std::vector<real>& table, real pr, real* dpsi_dpr = nullptr) {
    const int n = static_cast<int>(table.size()) - 1;
    const real prc = std::clamp(pr, real(0.0), real(1.0));
    const real u = prc * real(n);
    int i = static_cast<int>(u);
    if (i >= n) i = n - 1;
    const real t = u - real(i);
    const real a = table[static_cast<std::size_t>(i)];
    const real b = table[static_cast<std::size_t>(i) + 1];
    if (dpsi_dpr) *dpsi_dpr = (b - a) * real(n);
    return a + t * (b - a);
}

} // namespace ps::drivetrain
