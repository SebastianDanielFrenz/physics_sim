// ps/drivetrain/energy_ledger.h — public, additive: the mechanical energy
// ledger's per-vehicle cumulative snapshot (PHYS-033 P4 I2 plan section 8).
// Mechanical boundary at the tyre contact patch. Joules, cumulative since
// vehicle creation (never reset). Pure data.

#pragma once

#include "ps/types.h"

#include <cstdint>

namespace ps::drivetrain {

struct PowertrainEnergy {
    real engine_out = 0.0;      // h*omega'_e*tau_e,lin, cumulative
    real wheel_out = 0.0;       // h*Fx_lin*vx, cumulative (mechanical work delivered to the chassis)
    real tyre_loss = 0.0;       // h*Fx_lin*(omega'*R - vx), cumulative (slip loss, >= 0 in steady sign)
    real brake_loss = 0.0;      // -h*v'*lambda, cumulative, >= 0
    real gear_loss = 0.0;       // -h*omega'_in*tau_loss, cumulative, >= 0
    real integrator_loss = 0.0; // 1/2*Delta(omega)^T*J*Delta(omega), cumulative, >= 0 (linearly-implicit Euler's own dissipation;
                                // with compliant shafts it also carries their spring residual and may then decrease - see shaft_pe)
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

    // PHYS-033 P4 I4 C6 (plan section 3.1): a Viscous differential's implicit
    // L-R coupling dissipation, h*c*coupling_velocity^2 per substep,
    // cumulative, >= 0. 0 for every vehicle with no viscous diff (every
    // shipped vehicle today) - hash-neutral, same as clutch_loss et al. above.
    real diff_loss = 0.0;

    // PHYS-033 P4 I4 C9 (plan section 3.3): the MFWD transfer case's own
    // `front` friction row dissipation, -h*v*lambda per substep, cumulative,
    // >= 0. 0 for every vehicle with no transfer case (car_sedan.json and
    // every other pre-C9 file) - hash-neutral, same as diff_loss et al. above.
    real awd_clutch_loss = 0.0;

    // PHYS-033 P4 I4 C11 (plan section 3.5): the PTO's own friction row
    // dissipation - both its clutch slip (-h*v*lambda) and the disengaged-
    // stub-brake viscous term (-h*omega'*tau_stub) - per substep, cumulative,
    // >= 0. 0 for every vehicle with no pto (car_sedan.json, tractor.json
    // pre-C11) - hash-neutral, same as diff_loss/awd_clutch_loss above.
    real pto_clutch_loss = 0.0;
    // PHYS-033 P4 I4 C11: mechanical work absorbed by a test/bench load_dyno
    // component (brake: -h*v*lambda; viscous/quadratic: -h*tau_applied*
    // omega'), per substep, cumulative, >= 0. No shipped vehicle carries a
    // load_dyno, so this stays exactly 0 everywhere except a test that adds
    // one in code - hash-neutral (load_dyno itself is stateless, not hashed).
    real external_absorbed = 0.0;

    // PHYS-033 P4 I4 C13: compliant shafts (PowertrainDesc::compliance ==
    // Compliant). shaft_pe = sum(1/2*k*theta^2) over every compliant shaft,
    // instantaneous (stored, like rotor_ke - written by solve(), not by a
    // reset); shaft_damping_loss = the damper's h*c*v^2 per substep
    // (v = the trapezoidal mid-step twist rate, or v' on a backward-Euler
    // substep), cumulative, >= 0. The spring's own discretisation residual
    // goes into integrator_loss (it can be negative there, see
    // Powertrain::solve). Both stay exactly 0 for a Rigid vehicle (every
    // shipped vehicle) - hash-neutral, and end_tick() only adds them to the
    // balance when the vehicle has at least one compliant shaft.
    real shaft_pe = 0.0;
    real shaft_damping_loss = 0.0;

    // PHYS-033 P4 I7a C5: the fuel energy sub-ledger, entirely separate from
    // balance_error above (a fuel map's own energy split, not a mechanical
    // term any node-balance/momentum identity constrains) and NEVER folded
    // into fold_state_hash - a vehicle whose engine has a fuel map but no
    // tank (this commit; the tank is C6) hashes bit-identically to the same
    // vehicle with no fuel map at all. `indicated_work` is booked
    // UNCONDITIONALLY (EngineTorqueMap::post_solve computes it as
    // combustion's own proportional share of `work`, the same split
    // construction as `starter_in`'s, whether or not the engine has a fuel
    // map) and is therefore generally NONZERO for every combustion-firing
    // engine, fuel map or not - it is simply meaningless (never read) without
    // one. `fuel_mass`/`fuel_energy_in`/`engine_heat`/
    // `fuel_efficiency_violations` are gated on `has_fuel_map` (there is no
    // mass/LHV to convert without a map) and stay exactly 0 for every engine
    // with no fuel map (every engine shipped through this commit) -
    // hash- and value-neutral for those.
    real fuel_mass = 0.0;             // kg, cumulative (h*mdot per substep, mdot from the fuel map); 0 without a map
    real fuel_energy_in = 0.0;        // J, cumulative (fuel_mass_delta*LHV per substep); 0 without a map
    real indicated_work = 0.0;        // J, cumulative - combustion's own share of engine_out above (see PostSolveResult); booked unconditionally
    real engine_heat = 0.0;           // J, cumulative = fuel_energy_in - indicated_work (rejected/unconverted heat); 0 without a map
    real fuel_external_added = 0.0;   // J, cumulative; stays 0 until C6's refuel hook books into it
    std::uint64_t fuel_efficiency_violations = 0; // count of substeps where engine_heat's own delta was < 0 (map claims >100% efficiency); 0 without a map
};

} // namespace ps::drivetrain
