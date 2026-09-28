// engine_cycle/heat_transfer.h - Woschni in-cylinder heat-transfer
// correlation (PHYS-033 P4 I7c C15a, i7c plan section 3 "Wall heat"; Woschni
// 1967 as given in Heywood 1988 section 12.4.3, eq. 12.19/12.20). INTERNAL,
// header-only.
//
//   h = 3.26 * B^-0.2 * p^0.8 * T^-0.55 * w^0.8     [W/(m^2 K)], B m, p kPa, T K
//   w = C1*Sp_mean + C2 * V_d*T_r/(p_r*V_r) * (p - p_mot)     [m/s]
//   C1 = 6.18 while any valve is open (gas exchange), 2.28 otherwise;
//   C2 = 3.24e-3 m/(s K) from spark to EVO, 0 otherwise;
//   p_mot = p_r*(V_r/V)^1.35 (the motored pressure from the IVC reference).
// p^0.8*w^0.8 is evaluated as pow(p*w, 0.8) - one pow instead of two.
// A non-positive w (can only come from a p far below p_mot) gives h = 0.
// ps::math only (PLAN.md D7).

#pragma once

#include "ps/math/transcendental.h"
#include "ps/types.h"

namespace ps::engine_cycle {

inline constexpr real kWoschniMotoredPolytropic = 1.35;

struct WoschniInput {
    real bore_m = 0.0;
    real p_pa = 0.0;
    real t_k = 0.0;
    real v_m3 = 0.0;
    real mean_piston_speed = 0.0;   // 2*S*n
    real c1 = 2.28;
    real c2 = 0.0;                   // 0 outside [spark, EVO)
    real v_d = 0.0;
    real p_ref_pa = 0.0;             // IVC reference
    real v_ref_m3 = 0.0;
    real t_ref_k = 0.0;
};

// Characteristic gas velocity w [m/s].
[[nodiscard]] inline real woschni_velocity(const WoschniInput& in) {
    real w = in.c1 * in.mean_piston_speed;
    if (in.c2 != 0.0 && in.p_ref_pa > 0.0 && in.v_ref_m3 > 0.0) {
        const real p_mot = in.p_ref_pa * ps::math::pow(in.v_ref_m3 / in.v_m3, kWoschniMotoredPolytropic);
        w += in.c2 * in.v_d * in.t_ref_k / (in.p_ref_pa * in.v_ref_m3) * (in.p_pa - p_mot);
    }
    return w;
}

// Heat-transfer coefficient [W/(m^2 K)].
[[nodiscard]] inline real woschni_h(const WoschniInput& in) {
    const real w = woschni_velocity(in);
    if (w <= 0.0 || in.p_pa <= 0.0) return 0.0;
    const real p_kpa = in.p_pa * 1.0e-3;
    return 3.26 * ps::math::pow(in.bore_m, -0.2) * ps::math::pow(p_kpa * w, 0.8) *
           ps::math::pow(in.t_k, -0.55);
}

} // namespace ps::engine_cycle
