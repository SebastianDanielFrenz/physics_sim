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

    // PHYS-033 I3 (plan section 7.3): additive, all 0 until a caller actually
    // books into them (I2's engine/clutch/gearbox-free graphs never do, so
    // these stay exactly 0 for every I2 scenario - hash-neutral, and the
    // ledger itself is explicitly not part of any fold_state_hash, plan
    // section 7.7).
    real clutch_loss = 0.0;          // -h*v'*lambda_clutch, cumulative, >= 0
    real synchro_loss = 0.0;         // -h*v'*lambda_synchro, cumulative, >= 0
    real engine_friction_loss = 0.0; // -h*v'*lambda_engine_row (the engine's own Coulomb-to-ground row), cumulative, >= 0
    real engagement_loss = 0.0;      // 1/2*(w_old-w_proj)^T*J*(w_old-w_proj) at a topology-state switch, cumulative, >= 0
    real starter_in = 0.0;           // h*omega'*(tau_start - d_start*Delta(omega)), cumulative (mechanical work the starter puts in)
    real external_reset = 0.0;       // Delta(rotor_ke) booked by World::reset_vehicle setting network omega directly, cumulative (can be any sign)
};

} // namespace ps::drivetrain
