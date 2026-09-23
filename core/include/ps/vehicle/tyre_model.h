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

class ITyreModel {
public:
    virtual ~ITyreModel() = default;
    [[nodiscard]] virtual TyreForces evaluate(const TyreSlipInput& in) const = 0;
    [[nodiscard]] virtual const TyreDesc& desc() const = 0;
};

// The one P3 implementation (PLAN.md 5.1: Pacejka Magic Formula, MF 5.2/6.1
// subset - see tyre_desc.h's header comment for exactly what is
// implemented).
std::unique_ptr<ITyreModel> create_pacejka_tyre_model(const TyreDesc& desc);

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

} // namespace ps::vehicle
