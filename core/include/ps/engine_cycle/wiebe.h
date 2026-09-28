// engine_cycle/wiebe.h - Wiebe mass-fraction-burned function (PHYS-033 P4 I7c
// C15a, i7c plan section 3 "Combustion"; Heywood 1988 section 9.2.1, eq. 9.24).
// INTERNAL, header-only.
//
//   y      = (theta - theta_0)/dtheta
//   x_b    = 1 - exp(-a*y^(m+1))                       (y >= 0)
//   dx_b/dtheta = a*(m+1)/dtheta * y^m * exp(-a*y^(m+1))
// The burn rate is set to 0 once a*y^(m+1) > 40 (exp(-40) = 4e-18; below the
// rounding of any quantity it multiplies) so the burn has a finite end.
// ps::math only (PLAN.md D7).

#pragma once

#include "ps/math/transcendental.h"
#include "ps/types.h"

namespace ps::engine_cycle {

inline constexpr real kWiebeCutoffExponent = 40.0;

struct Wiebe {
    real a = 5.0;
    real m = 2.0;
    real dtheta_deg = 60.0;
    real theta0_deg = 0.0;   // start of combustion (spark), absolute crank deg

    [[nodiscard]] real exponent(real theta_deg) const {
        const real y = (theta_deg - theta0_deg) / dtheta_deg;
        if (y <= 0.0) return 0.0;
        return a * ps::math::pow(y, m + 1.0);
    }

    [[nodiscard]] real fraction(real theta_deg) const {
        const real e = exponent(theta_deg);
        if (e <= 0.0) return 0.0;
        if (e > kWiebeCutoffExponent) return 1.0;
        return 1.0 - ps::math::exp(-e);
    }

    // dx_b/dtheta in 1/deg.
    [[nodiscard]] real rate_per_deg(real theta_deg) const {
        const real y = (theta_deg - theta0_deg) / dtheta_deg;
        if (y <= 0.0) return 0.0;
        const real ym = ps::math::pow(y, m);
        const real e = a * ym * y;
        if (e > kWiebeCutoffExponent) return 0.0;
        return a * (m + 1.0) / dtheta_deg * ym * ps::math::exp(-e);
    }

    // Crank angle (absolute) at which x_b reaches `x` in (0,1):
    // theta0 + dtheta * (-ln(1-x)/a)^(1/(m+1)).
    [[nodiscard]] real angle_at_fraction(real x) const {
        return theta0_deg + dtheta_deg * ps::math::pow(-ps::math::log(1.0 - x) / a, 1.0 / (m + 1.0));
    }

    // The spark (theta0) that puts CA50 at `ca50_abs_deg`.
    [[nodiscard]] static real theta0_for_ca50(real a, real m, real dtheta_deg, real ca50_abs_deg) {
        return ca50_abs_deg - dtheta_deg * ps::math::pow(ps::math::log(2.0) / a, 1.0 / (m + 1.0));
    }
};

} // namespace ps::engine_cycle
