// core/src/drivetrain/engine.h — ps::drivetrain::IEngine (PHYS-033 P4 I7a
// C2): the engine seam that lets a second engine type (a mean-value
// simulated engine, I7b) sit next to EngineTorqueMap behind one interface.
// Pure structural refactor - EngineTorqueMap (engine_torque_map.h) is still
// the only concrete engine Powertrain ever constructs; this commit only
// carves out the interface and hoists the plain data structs the engine/ECU
// exchange (formerly nested inside EngineTorqueMap) to namespace scope so
// both this file (IEngine's own signatures) and engine_torque_map.h
// (EngineTorqueMap::pre_solve/post_solve) can name them without one header
// depending on the other's class. Layout/member order/defaults/comments of
// every hoisted struct are otherwise unchanged from EngineTorqueMap's own
// nested versions.
//
// EngineState: EngineTorqueMap's own Off/Stalled/Cranking/Running state
// machine used to be its own PRIVATE nested enum, byte-for-byte identical
// (same enumerators, same order, same std::uint8_t underlying type) to the
// already-public ps::drivetrain::EngineState (ps/drivetrain/
// powertrain_state.h, used by PowertrainSnapshot/EngineSoundState/
// ResetOptions::engine_state) - vehicle_instance.cpp's snapshot code did
// nothing but a 1:1 case-by-case translation between the two types. Hoisting
// the private enum to namespace scope here would collide with that public
// one (both would then be ps::drivetrain::EngineState); reusing the public
// type instead removes the would-be duplicate definition and needs no new
// translation, and is exactly as meaningful for a future engine type.
// EngineTorqueMap keeps `using EngineState = ps::drivetrain::EngineState;`
// so every existing EngineTorqueMap::EngineState::* call site (its own
// pre_solve/post_solve/reset, and tests/unit/test_powertrain_engine.cpp)
// keeps compiling, and byte-for-byte hashing (fold_state_hash folds
// state_'s raw bytes), unchanged.

#pragma once

#include "ps/drivetrain/powertrain_state.h" // ps::drivetrain::EngineState
#include "ps/types.h"

#include <cstdint>

namespace ps::drivetrain {

// ECU inputs, by value (spec's "controllers get sensors only"). omega is
// the crank node's omega_old (network state at THIS substep's start).
// Hoisted, unchanged, from EngineTorqueMap::Sensors (engine_torque_map.h).
struct Sensors {
    real omega = 0.0;
    real pedal_throttle = 0.0;
    real ignition = 0.0;                // > 0.5 == on
    real starter = 0.0;                 // > 0.5 == pressed
    real clutch_engaged_fraction = 1.0; // the clutch's current effective engagement e
    bool gearbox_in_neutral = true;
    // I4 C12: the governor's hand-throttle channel value (0 when there is
    // no governor - Powertrain::pre_solve_components only ever reads a
    // real channel value into this when desc().has_governor).
    real hand_throttle = 0.0;
};
// From the TCU (one substep lagged - plan section 7.2's deliberate order
// independence). Both default to "no request". Hoisted, unchanged, from
// EngineTorqueMap::TcuRequest.
struct TcuRequest {
    real throttle_floor = 0.0; // auto-blip
    bool ignition_cut = false; // upshift cut
    real throttle_ceiling = 1.0; // caps the pedal (not the blip floor) - the torque ramp-in after an upshift cut
};

// Hoisted, unchanged except the new theta_c field (PHYS-033 P4 I7a C2, see
// its own comment below), from EngineTorqueMap::PreSolveResult.
struct PreSolveResult {
    real tau0 = 0.0, d = 0.0;                                      // for Assembly::torque(crank, tau0, d)
    real friction_cap_static = 0.0, friction_cap_kinetic = 0.0;    // for the engine's own Coulomb-to-ground row
    EngineState state = EngineState::Off;
    real rpm = 0.0; // |omega| in rpm, UNCLAMPED (Curve1D::evaluate clamps its own domain)
    real throttle_eff = 0.0;
    bool limiter_active = false, dfco_active = false, starter_active = false;
    real torque_estimate = 0.0; // tau_smooth + tau_comb (HUD/telemetry, plan section 6.1.8)
    // I4 C12: the governor's own no-load setpoint n_nl this substep (0
    // without a governor), and the hand_throttle sensor value it read
    // (0 without a governor) - PowertrainSnapshot::governor_setpoint_rpm/
    // hand_throttle, never hashed.
    real governor_setpoint_rpm = 0.0;
    real hand_throttle = 0.0;
    // Net crank torque (Coulomb row included) the current throttle demand
    // (pedal/blip/idle governor) asks for at this rpm, IGNORING the rev
    // limiter: motoring + theta_eff * (wot - motoring). The TCU's
    // torque-following clutch close uses it as its feed-forward - the
    // limiter is a speed governor that answers the clutch's own load,
    // so feeding its (clutch-dependent) output back into the clutch
    // target held a flared engine on the limiter indefinitely.
    real torque_demand = 0.0;
    // PHYS-033 P4 I7a C2: the post-limiter combustion fraction theta_c
    // (engine_torque_map.cpp's own `theta_c = comb ? theta_eff*f_lim : 0`,
    // the value multiplying (w - m) in tau0 = s*(m+c_k) + theta_c*(w-m) +
    // tau_start). Write-only in C2: nothing reads it for physics yet -
    // PowertrainSnapshot::EngineSoundState::load publishes it for sound
    // (vehicle_instance.cpp), never hashed.
    real theta_c = 0.0;
};

// Hoisted, unchanged, from EngineTorqueMap::PostSolveResult.
struct PostSolveResult {
    real engine_out_delta = 0.0; // ledger.engine_out += this
    real starter_in_delta = 0.0; // ledger.starter_in += this
};

// The engine/ECU seam (PHYS-033 P4 I7a plan): exactly the operations
// Powertrain/VehicleInstance drive an engine through today.
// EngineTorqueMap (engine_torque_map.h) is the only implementation until
// I7b's mean-value simulated engine. Powertrain::engine_ always holds this
// as a std::unique_ptr<IEngine> and never downcasts it (no dynamic_cast
// anywhere on the substep hot path).
//
// Deliberately NOT part of this interface:
//  - EngineTorqueMap::Desc (and its state()/desc() accessors): genuinely
//    torque-map specific (WOT/motoring curves, limiter/starter/idle-PI
//    tuning) - a mean-value engine's own Desc is a different shape
//    entirely. The two call sites that used to read engine_->desc() are
//    resolved without it: Powertrain's constructor already has the
//    vehicle's own TorqueMapEngineDesc in hand (no need to round-trip
//    through the interface), and idle_rpm - the one field Powertrain::reset
//    still needs after construction - gets its own narrow virtual below
//    (a concept every engine type has, unlike the rest of Desc).
class IEngine {
public:
    virtual ~IEngine() = default;

    // Evaluated first each substep (plan sections 6.1.3-6.1.7). Must be
    // followed by exactly one post_solve() call before the next pre_solve().
    [[nodiscard]] virtual PreSolveResult pre_solve(const Sensors& sensors, const TcuRequest& tcu, real h) = 0;
    // omega_new: the crank node's omega after the network solve.
    [[nodiscard]] virtual PostSolveResult post_solve(real omega_new, real h) = 0;

    [[nodiscard]] virtual real idle_integrator() const = 0;
    [[nodiscard]] virtual real crank_angle() const = 0; // NOT hashed (sound only)
    // The engine's own idle setpoint (rpm) - Powertrain::reset uses this to
    // seed the crank at idle when there is no driveline speed to derive one
    // from yet (plan section 4.5's bootstrap). See the class comment above
    // for why this is a narrow virtual rather than exposing the whole Desc.
    [[nodiscard]] virtual real idle_rpm() const = 0;

    [[nodiscard]] virtual std::uint64_t fold_state_hash(std::uint64_t seed) const = 0;

    // World::reset_vehicle (plan section 4.5): forces the engine directly
    // into `state`, clearing whatever transient latches/integrators it
    // owns. The crank's own network omega is set separately by the caller
    // (ShaftNetwork::set_omega).
    virtual void reset(EngineState state) = 0;
};

} // namespace ps::drivetrain
