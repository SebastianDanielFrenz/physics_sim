// core/src/drivetrain/engine_torque_map.h — the I3 torque_map engine + ECU
// (PHYS-033 P4 I3 plan section 6.1). A new class living NEXT TO I2's
// placeholder TorqueMapEngine (components.h) - not yet an IComponent, not
// yet referenced by Powertrain's constructor (that wiring, and the public
// descriptor swap, land in commit 6 "the switch"). Framework-agnostic on
// purpose (no NodeId/ShaftNetwork/Assembly member): pre_solve/post_solve
// take and return plain structs, which is also what makes it directly
// unit-testable (plan section 13.7) without building a network at all.
//
// Deliberately named EngineTorqueMap/Desc/LimiterDesc/... rather than reusing
// the plan section 4.1 names (TorqueMapEngineDesc, EngineLimiterDesc, ...):
// those names are still the PUBLIC I2 shapes in ps/drivetrain/powertrain_desc.h
// until commit 6 replaces them, and both cannot exist in ps::drivetrain at
// once. Commit 6 renames these into the public section 4.1 shapes.

#pragma once

#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <cstdint>

namespace ps::drivetrain {

class EngineTorqueMap {
public:
    enum class LimiterType : std::uint8_t { FuelCut, SoftCut };
    struct LimiterDesc {
        LimiterType type = LimiterType::FuelCut;
        real rpm = 6800.0;           // FuelCut: cut at >= rpm. SoftCut: zero fuel at rpm (governor high idle)
        real hysteresis_rpm = 150.0; // FuelCut only: restore at <= rpm - hysteresis
        real band_rpm = 200.0;       // SoftCut only: linear taper over [rpm - band, rpm] (governor droop)
    };
    struct IdleControlDesc { // PI on throttle (idle air bypass), plan section 6.1.4
        real kp_per_rpm = 0.0018;
        real ki_per_rpm_s = 0.009;
        real max_throttle = 0.3;
    };
    enum class StarterInterlock : std::uint8_t { None, ClutchOrNeutral };
    struct StarterDesc { // ideal DC starter referred to the crank
        real stall_torque_nm = 140.0; // at 0 rpm
        real free_speed_rpm = 400.0;  // torque falls linearly to 0 here
        StarterInterlock interlock = StarterInterlock::ClutchOrNeutral;
    };
    enum class InitialState : std::uint8_t { Running, Off };

    struct Desc {
        vehicle::Curve1D wot_torque_nm_vs_rpm;      // required; x ascending, x[0] >= 0
        vehicle::Curve1D motoring_torque_nm_vs_rpm; // required; y <= 0; x[0] == 0 (its y[0] is the Coulomb part)
        real inertia_kgm2 = 0.0;                    // crank + flywheel + clutch cover; > 0 (validated by the loader, commit 6)
        real breakaway_torque_nm = 0.0;              // static Coulomb capacity; >= -motoring(0) (validated at construction)
        real idle_rpm = 800.0, stall_rpm = 400.0, fire_rpm = 250.0, start_complete_rpm = 550.0;
        real dfco_enable_rpm = 1300.0, dfco_reenable_rpm = 1000.0;
        IdleControlDesc idle;
        LimiterDesc limiter;
        StarterDesc starter;
        InitialState initial_state = InitialState::Running;
    };

    enum class EngineState : std::uint8_t { Off, Stalled, Cranking, Running };

    // ECU inputs, by value (spec's "controllers get sensors only"). omega is
    // the crank node's omega_old (network state at THIS substep's start).
    struct Sensors {
        real omega = 0.0;
        real pedal_throttle = 0.0;
        real ignition = 0.0;                // > 0.5 == on
        real starter = 0.0;                 // > 0.5 == pressed
        real clutch_engaged_fraction = 1.0; // the clutch's current effective engagement e
        bool gearbox_in_neutral = true;
    };
    // From the TCU (one substep lagged - plan section 7.2's deliberate order
    // independence). Both default to "no request".
    struct TcuRequest {
        real throttle_floor = 0.0; // auto-blip
        bool ignition_cut = false; // upshift cut
        real throttle_ceiling = 1.0; // caps the pedal (not the blip floor) - the torque ramp-in after an upshift cut
    };

    struct PreSolveResult {
        real tau0 = 0.0, d = 0.0;                                      // for Assembly::torque(crank, tau0, d)
        real friction_cap_static = 0.0, friction_cap_kinetic = 0.0;    // for the engine's own Coulomb-to-ground row
        EngineState state = EngineState::Off;
        real rpm = 0.0; // |omega| in rpm, UNCLAMPED (Curve1D::evaluate clamps its own domain)
        real throttle_eff = 0.0;
        bool limiter_active = false, dfco_active = false, starter_active = false;
        real torque_estimate = 0.0; // tau_smooth + tau_comb (HUD/telemetry, plan section 6.1.8)
        // Net crank torque (Coulomb row included) the current throttle demand
        // (pedal/blip/idle governor) asks for at this rpm, IGNORING the rev
        // limiter: motoring + theta_eff * (wot - motoring). The TCU's
        // torque-following clutch close uses it as its feed-forward - the
        // limiter is a speed governor that answers the clutch's own load,
        // so feeding its (clutch-dependent) output back into the clutch
        // target held a flared engine on the limiter indefinitely.
        real torque_demand = 0.0;
    };

    struct PostSolveResult {
        real engine_out_delta = 0.0; // ledger.engine_out += this
        real starter_in_delta = 0.0; // ledger.starter_in += this
    };

    explicit EngineTorqueMap(Desc desc);

    // Evaluated first each substep (plan sections 6.1.3-6.1.7). Must be
    // followed by exactly one post_solve() call before the next pre_solve().
    [[nodiscard]] PreSolveResult pre_solve(const Sensors& sensors, const TcuRequest& tcu, real h);
    // omega_new: the crank node's omega after the network solve.
    [[nodiscard]] PostSolveResult post_solve(real omega_new, real h);

    [[nodiscard]] EngineState state() const { return state_; }
    [[nodiscard]] real idle_integrator() const { return idle_integrator_; }
    [[nodiscard]] real crank_angle() const { return crank_angle_; } // NOT hashed (sound only, plan section 6.1.9)
    [[nodiscard]] const Desc& desc() const { return desc_; }

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    // World::reset_vehicle (plan section 4.5): forces the engine directly
    // into `state` (bypassing the ordinary state-machine transitions of
    // pre_solve - see this file's own comment on why a state == Running
    // engine cannot bootstrap itself from omega == 0), clearing the idle
    // integrator and the limiter/DFCO latches. The crank's own network
    // omega is set separately by the caller (ShaftNetwork::set_omega).
    void reset(EngineState state) {
        state_ = state;
        idle_integrator_ = real(0.0);
        limiter_cut_ = false;
        dfco_active_ = false;
    }

private:
    Desc desc_;
    real theta_ff_ = 0.0; // build-time idle feedforward (plan section 6.1.4), computed once in the constructor
    real c_k_ = 0.0;      // -motoring(0): the Coulomb part of engine friction, computed once in the constructor

    // Hashed (plan section 6.1.9): state, idle integrator, limiter cut flag, dfco flag.
    EngineState state_ = EngineState::Off;
    real idle_integrator_ = 0.0;
    bool limiter_cut_ = false; // FuelCut hysteresis latch
    bool dfco_active_ = false;

    real crank_angle_ = 0.0; // NOT hashed

    // Stashed by pre_solve, read back by post_solve.
    real tau0_ = 0.0, d_ = 0.0, omega_old_ = 0.0;
    real tau_smooth_ = 0.0, tau_comb_ = 0.0, tau_start_ = 0.0;
};

} // namespace ps::drivetrain
