// engine_cycle/valve_flow.h - compressible orifice flow for the cycle model's
// valves and port orifices (PHYS-033 P4 I7c C15a, i7c plan section 3
// "Valve flow"). INTERNAL, header-only.
//
// Isentropic nozzle flow through an effective area Cd*A:
//   mdot = Cd*A * p_up / sqrt(R_up*T_up) * Psi(pr),  pr = p_down/p_up
//   Psi  = sqrt(2g/(g-1) * (pr^(2/g) - pr^((g+1)/g)))   (subsonic)
//        = sqrt(g) * (2/(g+1))^((g+1)/(2(g-1)))          (choked, pr <= pr_crit)
// Psi has an infinite slope at pr -> 1 (dPsi/dpr ~ 1/sqrt(1-pr)), which makes
// the ODE non-Lipschitz at pressure equality. Above pr = 0.99 Psi is
// replaced by the straight line through (0.99, Psi(0.99)) and (1, 0) - the
// plan's regularisation, continuous in value.
//
// The upstream side's gamma class selects Psi: fresh gas 1.40, burned gas
// 1.30. A mixed upstream uses the burned-mass-fraction-weighted blend of the
// two (PROVISIONAL P5, out/c15a_progress.md) so the flow stays smooth in x_b.
// Every pow goes through ps::math (PLAN.md D7).

#pragma once

#include "ps/math/transcendental.h"
#include "ps/types.h"

#include <cmath>

namespace ps::engine_cycle {

inline constexpr real kPsiRegularisePr = 0.99;

struct PsiClass {
    real gamma = 1.4;
    real pr_crit = 0.0;
    real psi_choke = 0.0;
    real coef = 0.0;       // 2g/(g-1)
    real inv_gamma = 0.0;
    real psi_reg = 0.0;    // Psi(0.99) on the exact subsonic branch

    static PsiClass make(real g) {
        PsiClass c;
        c.gamma = g;
        c.pr_crit = ps::math::pow(2.0 / (g + 1.0), g / (g - 1.0));
        c.psi_choke = std::sqrt(g * ps::math::pow(2.0 / (g + 1.0), (g + 1.0) / (g - 1.0)));
        c.coef = 2.0 * g / (g - 1.0);
        c.inv_gamma = 1.0 / g;
        c.psi_reg = c.exact_subsonic(kPsiRegularisePr);
        return c;
    }

    // Subsonic branch written as q = pr^(1/g): pr^(2/g) - pr^((g+1)/g) = q*(q - pr).
    [[nodiscard]] real exact_subsonic(real pr) const {
        const real q = ps::math::pow(pr, inv_gamma);
        real v = coef * q * (q - pr);
        if (v < 0.0) v = 0.0;
        return std::sqrt(v);
    }

    // pr in [0, 1].
    [[nodiscard]] real eval(real pr) const {
        if (pr <= pr_crit) return psi_choke;
        if (pr >= kPsiRegularisePr) return psi_reg * (1.0 - pr) / (1.0 - kPsiRegularisePr);
        return exact_subsonic(pr);
    }
};

struct PsiPair {
    PsiClass fresh;
    PsiClass burned;

    [[nodiscard]] real eval(real pr, real x_burned_up) const {
        return (1.0 - x_burned_up) * fresh.eval(pr) + x_burned_up * burned.eval(pr);
    }
};

// Signed orifice mass flow from side 1 to side 2 (negative = 2 -> 1).
// Upstream is the side with the higher pressure; its (R, T, x_b) set the flow.
struct OrificeFlow {
    real mdot = 0.0;      // kg/s, positive 1 -> 2
    bool from_1 = true;   // upstream is side 1
};

[[nodiscard]] inline OrificeFlow orifice_flow(const PsiPair& psi, real cd_area,
                                              real p1, real r1, real t1, real x1,
                                              real p2, real r2, real t2, real x2) {
    OrificeFlow f;
    if (cd_area <= 0.0) return f;
    if (p1 >= p2) {
        f.from_1 = true;
        f.mdot = cd_area * p1 / std::sqrt(r1 * t1) * psi.eval(p2 / p1, x1);
    } else {
        f.from_1 = false;
        f.mdot = -cd_area * p2 / std::sqrt(r2 * t2) * psi.eval(p1 / p2, x2);
    }
    return f;
}

} // namespace ps::engine_cycle
