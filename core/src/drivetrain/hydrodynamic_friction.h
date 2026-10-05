// core/src/drivetrain/hydrodynamic_friction.h - engine thermal T1b: the speed
// shape of the oil-viscosity friction increment (internal, ps::drivetrain).
//
// A cold engine's friction is the warm friction map plus a hydrodynamic
// increment
//     dF(n) = (k - 1) * F(n_ref) * shape(n),   shape(n) = sqrt(n / n_ref),
// with k = 1 + phi_h * (sqrt(mu / mu_ref) - 1) from EngineThermal. Physics: a
// hydrodynamically lubricated contact under a fixed load W (ring pack, skirt,
// heavily loaded bearing) has a film h ~ sqrt(mu U / W) and a shear force
// F ~ mu U / h ~ sqrt(mu U W): the same sqrt that gives the viscosity exponent
// 0.5 (Sandoval & Heywood's hydrodynamic terms) gives the speed exponent 0.5.
// The map's boundary/mixed part (its Coulomb value F(0) and the breakaway
// torque, the Stribeck boundary regime) does not depend on viscosity and is
// never scaled. At n_ref the whole map is scaled by exactly k (the T1
// calibration: friction "about 2x to 2.5x" at 20 C oil).
//
// Below kHydrodynamicLinearBelowRpm the sqrt is replaced by its chord (linear
// through 0, continuous at the joint) so the slope stays finite at standstill
// [AS: 50 rpm, below the cited 40..100 rpm SI minimum starting speed band, so
// it never shapes a start]. The Coulomb row holds the crank at rest.
//
// Exact zero when k == 1: callers add the increment only when k != 1, so an
// engine without thermal (or a warm one) evaluates the pre-T1 expressions.

#pragma once

#include "ps/types.h"

#include <cmath>

namespace ps::drivetrain {

constexpr real kHydrodynamicLinearBelowRpm = 50.0;

struct HydrodynamicShape {
    real value = 0.0;         // shape(n)
    real slope_per_rpm = 0.0; // d shape / d n
};

inline HydrodynamicShape hydrodynamic_speed_shape(real rpm, real ref_rpm) {
    HydrodynamicShape s;
    if (rpm >= kHydrodynamicLinearBelowRpm) {
        s.value = std::sqrt(rpm / ref_rpm);
        s.slope_per_rpm = real(0.5) * s.value / rpm;
    } else {
        const real c = std::sqrt(kHydrodynamicLinearBelowRpm / ref_rpm) / kHydrodynamicLinearBelowRpm;
        s.value = c * rpm;
        s.slope_per_rpm = c;
    }
    return s;
}

} // namespace ps::drivetrain
