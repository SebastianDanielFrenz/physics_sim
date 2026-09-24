// ps/drivetrain/energy_ledger.h — public, additive: the mechanical energy
// ledger's per-vehicle cumulative snapshot (PHYS-033 P4 I2 plan section 8).
// Mechanical boundary at the tyre contact patch. Joules, cumulative since
// vehicle creation (never reset). Pure data.

#pragma once

#include "ps/types.h"

namespace ps::drivetrain {

struct PowertrainEnergy {
    real engine_out = 0.0;      // h*omega'_e*tau_e,lin, cumulative
    real wheel_out = 0.0;       // h*Fx_lin*vx, cumulative (mechanical work delivered to the chassis)
    real tyre_loss = 0.0;       // h*Fx_lin*(omega'*R - vx), cumulative (slip loss, >= 0 in steady sign)
    real brake_loss = 0.0;      // -h*v'*lambda, cumulative, >= 0
    real gear_loss = 0.0;       // -h*omega'_in*tau_loss, cumulative, >= 0
    real integrator_loss = 0.0; // 1/2*Delta(omega)^T*J*Delta(omega), cumulative, >= 0 (linearly-implicit Euler's own dissipation)
    real rotor_ke = 0.0;        // 1/2*sum(J_i*omega_i^2), instantaneous (NOT cumulative - "stored" in the plan's own naming)
    real balance_error = 0.0;   // per-tick closure residual (plan section 8); should stay near 0 (< 1e-9 relative)
};

} // namespace ps::drivetrain
