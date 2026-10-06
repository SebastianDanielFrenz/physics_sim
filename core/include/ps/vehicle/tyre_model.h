// ps/vehicle/tyre_model.h — ITyreModel seam (PLAN.md 5.1: "separate tyre-
// model interface so a brush/TMeasy or soft-soil model can replace it per
// surface"). Mirrors the ps/backend/i_rigid_backend.h pattern (D2): a
// public abstract interface plus a factory function; the one P3
// implementation (Pacejka Magic Formula) is internal
// (core/src/vehicle/pacejka_tyre_model.cpp), reached only through
// create_pacejka_tyre_model below.
//
// ITyreModel::evaluate is a stateless, pure function of its inputs (safe to
// call concurrently for different wheels, PLAN.md D8) - it takes the
// already relaxation-filtered slip ratio/angle, not the raw instantaneous
// ground-contact slip. The relaxation-length transient state (per wheel,
// not per tyre spec) is owned by the caller via SlipRelaxationState below,
// not by the model.

#pragma once

#include "ps/types.h"
#include "ps/vehicle/tyre_desc.h"

#include <memory>

namespace ps::vehicle {

// Inputs at the tyre contact patch, ISO 8855 sign conventions (PLAN.md D4):
// kappa > 0 == wheel spinning faster than rolling (driving/accelerating
// slip); alpha > 0 == velocity vector points left of the wheel's rolling
// direction; gamma > 0 == top of the wheel leaning left (positive camber,
// SAE/ISO convention: camber about the forward axis).
struct TyreSlipInput {
    real kappa = 0.0;   // relaxation-filtered longitudinal slip ratio (dimensionless)
    real alpha = 0.0;   // relaxation-filtered slip angle (rad)
    real fz = 0.0;      // normal load, N (>= 0)
    real gamma = 0.0;   // camber angle, rad
    real vx = 0.0;      // contact-patch longitudinal speed, m/s (signed)
    real lambda_mu = 1.0; // surface friction scale (ps::io::SurfaceTable::lambda_mu_for), multiplies the tyre's own lambda_mux/muy
    real crr_surface = 0.0; // surface rolling-resistance coefficient (ps::io::SurfaceTable::crr_for)
};

struct TyreForces {
    real fx = 0.0;  // longitudinal force at the contact patch, N
    real fy = 0.0;  // lateral force at the contact patch, N
    real mz = 0.0;  // self-aligning moment about the contact patch's z axis, N*m
    real mx = 0.0;  // overturning moment about the contact patch's x axis, N*m
    real fx_pure = 0.0; // Fx0(kappa, Fz, gamma) before combined-slip weighting - exposed for validation/tests only
    real fy_pure = 0.0; // Fy0(alpha, Fz, gamma) before combined-slip weighting - exposed for validation/tests only
};

// PHYS-033 P4 I2 plan section 5.7: Fx together with its slope d(Fx)/d(kappa)
// at the given (combined-slip) operating point, needed by the implicit
// wheel-hub coupling (section 5.6) to linearise the tyre's torque law.
struct TyreForcesWithSlope {
    TyreForces forces;
    real dfx_dkappa = 0.0;
};

class ITyreModel {
public:
    virtual ~ITyreModel() = default;
    [[nodiscard]] virtual TyreForces evaluate(const TyreSlipInput& in) const = 0;
    [[nodiscard]] virtual const TyreDesc& desc() const = 0;

    // Non-pure, default implementation (core/src/vehicle/tyre_model.cpp):
    // central finite difference on kappa (fixed step 1e-6) around
    // evaluate(). A model can override this with an analytic derivative
    // (the Pacejka implementation does - pacejka_tyre_model.cpp); either
    // way, `.forces` must equal evaluate(in) exactly (same values, not
    // independently recomputed).
    [[nodiscard]] virtual TyreForcesWithSlope evaluate_with_slope(const TyreSlipInput& in) const;
};

// The one P3 implementation (PLAN.md 5.1: Pacejka Magic Formula, MF 5.2/6.1
// subset - see tyre_desc.h's header comment for exactly what is
// implemented).
std::unique_ptr<ITyreModel> create_pacejka_tyre_model(const TyreDesc& desc);

// The tyre's speed-dependent rolling-resistance part, qsy3*|vx/v0| +
// qsy4*(vx/v0)^4 (MF 6.1 eq. 4.E70's speed terms; vx signed, m/s). The full
// coefficient the wheel applies is qsy1 + surface crr + qsy2*|Fx|/Fz0 + this
// (core/src/vehicle/wheel_instance.cpp) - tests build their independent
// references from this same function so there is exactly one speed law.
// With qsy3 == 0 the result is bit-identical to the pre-qsy3 qsy4 term.
[[nodiscard]] real tyre_rolling_speed_term(const TyreDesc& desc, real vx);

// Closed-form derivation of qsy3/qsy4 from two anchors (a tyre file's
// optional "rolling_resistance" block). With x_r = V_r/v0 the model is
//   crr(v) = qsy1 * (1 + A*(v/V_r) + B*(v/V_r)^4),  A = qsy3*x_r/qsy1,
//   B = qsy4*x_r^4/qsy1,
// and the anchors crr(fraction*V_r) = factor*crr_base, crr(V_r) =
// factor_at_rated*crr_base give (F = factor_at_rated, f = fraction)
//   B = ((F-1)*f - (factor-1)) / (f - f^4),  A = (F-1) - B.
struct RollingResistanceAnchors {
    real rated_speed_kmh = 0.0;  // V_r, the tyre's speed-symbol rating
    real crr_base = 0.0;         // becomes qsy1
    real fraction = 0.0;         // f, in (0,1)
    real factor = 1.0;           // crr(f*V_r)/crr_base, >= 1
    real factor_at_rated = 1.0;  // crr(V_r)/crr_base, > factor
};
struct RollingResistanceCoefficients {
    real qsy1 = 0.0;
    real qsy3 = 0.0;
    real qsy4 = 0.0;
};
// Throws std::invalid_argument (message starts "rolling_resistance: ") on
// any violated precondition: rated_speed_kmh/crr_base/v0 <= 0, fraction
// outside (0,1), factor < 1, factor_at_rated <= factor, or an anchor pair
// whose solution has a negative A or B ((factor-1)/(factor_at_rated-1)
// outside [f^4, f]).
[[nodiscard]] RollingResistanceCoefficients derive_rolling_resistance(const RollingResistanceAnchors& anchors, real v0);

// PLAN.md ch.7 transient/relaxation-length model (out/handover_p3.md
// decision 2): d(kappa_eq)/dt = (slip_velocity_x - Vx*kappa_eq) /
// relaxation_length_x, integrated EXACTLY over a substep via the scalar
// exponential - unconditionally stable at any stiffness/speed, and
// degenerates to plain integration of slip velocity at Vx ~ 0 (PLAN.md's
// "damped stiffness model near standstill"), not a slip-ratio-target lag
// that would freeze at Vx = 0 (the naive approach this replaced - see
// out/handover_p3.md). Owned per wheel, not per ITyreModel.
struct SlipRelaxationState {
    real kappa_eq = 0.0;
    real alpha_eq = 0.0;
};

// slip_velocity_x = omega*R - Vx (m/s); slip_velocity_y = Vy (m/s, contact-
// patch lateral ground speed). vx_mag is the SPEED the contact patch
// advances through the footprint, used for both channels' decay rate
// (>= 0; the caller passes std::abs(Vx) or similar - this function does not
// take an absolute value itself so a test can probe the exact-zero case
// directly).
void step_slip_relaxation(SlipRelaxationState& state, real slip_velocity_x, real slip_velocity_y,
                           real vx_mag, real relaxation_length_x, real relaxation_length_y, real dt);

// PHYS-033 P4 I2 plan section 5.6: the same one-axis relaxation step as
// step_slip_relaxation above (which now delegates to this, guaranteeing
// bit-for-bit equality - see pacejka_tyre_model.cpp), but also returning
// the derivative of the new (unclamped) state with respect to
// `slip_velocity` - needed by the implicit wheel-hub coupling, which
// treats kappa's predictor as linear in the hub's own new angular
// velocity (slip_velocity = omega*R - vx, so d(kappa)/d(omega) =
// R * dx_dslipvel).
struct RelaxStep {
    real x_unclamped = 0.0;
    real dx_dslipvel = 0.0;
};
RelaxStep relaxation_step_axis(real x, real slip_velocity, real v_mag, real sigma, real dt);

} // namespace ps::vehicle
