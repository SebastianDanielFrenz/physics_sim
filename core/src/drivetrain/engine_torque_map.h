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
//
// PHYS-033 P4 I7a C2: Sensors/TcuRequest/PreSolveResult/PostSolveResult (and
// EngineState, reusing the already-public ps::drivetrain::EngineState - see
// engine.h's header comment) moved to namespace scope in the new engine.h,
// which also declares the IEngine interface EngineTorqueMap now implements.
// The `using` aliases below keep every existing EngineTorqueMap::Sensors/
// ::TcuRequest/::PreSolveResult/::PostSolveResult/::EngineState call site
// (this file's own pre_solve/post_solve/reset, powertrain.cpp,
// tests/unit/test_powertrain_engine.cpp) compiling unchanged.

#pragma once

#include "engine.h"

#include "ps/drivetrain/fuel_desc.h" // FuelDesc (PHYS-033 P4 I7a C5)
#include "ps/drivetrain/table2d.h"   // Table2D (PHYS-033 P4 I7a C5, the optional fuel map)
#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <cstdint>

namespace ps::drivetrain {

class EngineTorqueMap final : public IEngine {
public:
    using Sensors = ps::drivetrain::Sensors;
    using ThrottleCommand = ps::drivetrain::ThrottleCommand;
    using TcuRequest = ps::drivetrain::TcuRequest;
    using PreSolveResult = ps::drivetrain::PreSolveResult;
    using PostSolveResult = ps::drivetrain::PostSolveResult;
    using EngineState = ps::drivetrain::EngineState;

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

    // PHYS-033 P4 I4 C12 (plan section 3.6): all-speed diesel governor with
    // droop - see ps/drivetrain/powertrain_desc.h's GovernorDesc (the public
    // shape this is copied field-for-field from, powertrain.cpp's
    // to_engine_desc) for the full model description.
    struct GovernorDesc {
        real high_idle_rpm = 2400.0;
        real rated_rpm = 2200.0;
        real droop = 0.0909;
    };

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
        bool has_governor = false;
        GovernorDesc governor;
        // PHYS-033 P4 I7a C5: optional fuel map (rpm x load -> g/s), copied
        // field-for-field from TorqueMapEngineDesc by powertrain.cpp's
        // to_engine_desc, same pattern as governor above. has_fuel_map false
        // (default) means pre_solve/post_solve never touch mdot_kg_s_ or any
        // PostSolveResult fuel field - byte-identical to before this commit.
        bool has_fuel_map = false;
        FuelDesc fuel;
        Table2D fuel_map;
        // PHYS-033 P4 I7a C5d: the crude no-map fuel estimate, active in
        // pre_solve/post_solve whenever has_fuel_model is true and
        // has_fuel_map is false (to_engine_desc sets `fuel` to the ACTIVE
        // fuel for whichever path applies, so this struct never needs to
        // know about TorqueMapEngineDesc::fuels/ignition itself - eta_i is
        // already resolved (override or Heywood ignition-type default) by
        // the time it gets here). has_fuel_model false (default) means
        // pre_solve/post_solve never touch mdot_kg_s_/fuel fields at all -
        // byte-identical to before this commit.
        bool has_fuel_model = false;
        real eta_i = 0.0; // only meaningful when has_fuel_model && !has_fuel_map
    };

    explicit EngineTorqueMap(Desc desc);

    // Evaluated first each substep (plan sections 6.1.3-6.1.7). Must be
    // followed by exactly one post_solve() call before the next pre_solve().
    // PHYS-033 P4 I7a C3: `throttle` is the throttle_stage.h stage's own
    // output (pedal/hand-lever already combined) - the engine never reads
    // the pedal directly.
    [[nodiscard]] PreSolveResult pre_solve(const Sensors& sensors, const ThrottleCommand& throttle, const TcuRequest& tcu, real h) override;
    // omega_new: the crank node's omega after the network solve.
    [[nodiscard]] PostSolveResult post_solve(real omega_new, real h) override;

    [[nodiscard]] EngineState state() const { return state_; }
    [[nodiscard]] real idle_integrator() const override { return idle_integrator_; }
    [[nodiscard]] real crank_angle() const override { return crank_angle_; } // NOT hashed (sound only, plan section 6.1.9)
    [[nodiscard]] const Desc& desc() const { return desc_; }
    [[nodiscard]] real idle_rpm() const override { return desc_.idle_rpm; }

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const override;

    // World::reset_vehicle (plan section 4.5): forces the engine directly
    // into `state` (bypassing the ordinary state-machine transitions of
    // pre_solve - see this file's own comment on why a state == Running
    // engine cannot bootstrap itself from omega == 0), clearing the idle
    // integrator and the limiter/DFCO latches. The crank's own network
    // omega is set separately by the caller (ShaftNetwork::set_omega).
    void reset(EngineState state) override {
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
    // PHYS-033 P4 I7a C5: this substep's fuel flow, evaluated in pre_solve at
    // the substep-start rpm/theta_c against desc_.fuel_map, consumed by
    // post_solve. PHYS-033 P4 I7a C5d review fix (N10): a prior version of
    // this comment claimed it "stays exactly 0.0 forever when
    // !desc_.has_fuel_map" - false as of C5d, which ALSO computes a nonzero
    // mdot_kg_s_ via the crude no-map estimate whenever has_fuel_model is
    // true and has_fuel_map is false (mech_power/(eta_i*LHV), see pre_solve).
    // It stays exactly 0.0 only when neither has_fuel_map nor has_fuel_model
    // is set. NOT hashed (fold_state_hash is unchanged by either commit).
    real mdot_kg_s_ = 0.0;
};

} // namespace ps::drivetrain
