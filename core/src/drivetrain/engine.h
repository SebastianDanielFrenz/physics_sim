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
#include "ps/world/ambient.h"

#include <cstdint>
#include <limits>

namespace ps::drivetrain {

class OxidizerTables; // nitrous_flow.h (immutable property tables, owned by the NitrousSystem)

// N2O plan N1: what the Powertrain-owned NitrousSystem (nitrous_system.h) hands
// the engine each substep. All zero = no nitrous (every vehicle without a kit,
// and every substep the solenoid is closed). N2: the simulated engine applies its
// wet charge model (engine_simulated.h) only while active && mdot_n_kg_s > 0 &&
// oxidizer != nullptr; the graph engine still ignores the block (N6).
struct NitrousInput {
    real mdot_n_kg_s = 0.0;  // N2O mass flow delivered to the intake
    real mdot_kf_kg_s = 0.0; // kit fuel mass flow (the engine's own fuel, wet kit)
    real h_in_j_kg = 0.0;    // liquid enthalpy at the jet (the cooling balance's inlet state)
    real retard_deg = 0.0;   // kit/controller spark retard
    std::uint8_t injection = 0;   // 0 plenum, 1 direct port
    std::uint8_t enrichment = 0;  // 0 kit fuel, 1 ECU (dry kit, later)
    bool active = false;     // solenoid open and N2O flowing
    // N2O plan N2: the oxidizer's property tables (decomposition heat, O2 fraction,
    // molar mass, gas enthalpy for the evaporative-cooling balance). Owned by the
    // NitrousSystem, immutable; nullptr = no charge model (the engine treats the
    // block as inactive).
    const OxidizerTables* oxidizer = nullptr;
};

// ECU inputs, by value (spec's "controllers get sensors only"). omega is
// the crank node's omega_old (network state at THIS substep's start).
// Hoisted, unchanged except pedal_throttle/hand_throttle's removal, from
// EngineTorqueMap::Sensors (engine_torque_map.h) - PHYS-033 P4 I7a C3: the
// pedal/hand-lever combination now happens in the throttle_stage.h stage
// BEFORE the engine ever sees a value (see ThrottleCommand below); the
// engine never reads the pedal directly.
struct Sensors {
    real omega = 0.0;
    real ignition = 0.0;                // > 0.5 == on
    real starter = 0.0;                 // > 0.5 == pressed
    real clutch_engaged_fraction = 1.0; // the clutch's current effective engagement e
    bool gearbox_in_neutral = true;
    // PHYS-033 P4 I7a C6: NOT an ECU sensor - the physical supply limit, the
    // fuel mass (kg) the feeding tank can deliver this substep (its whole
    // content; Powertrain sets it only when a fuel_tank feeds this engine).
    // The default +infinity never starves, so every engine without a tank
    // (and every direct pre_solve call site) behaves exactly as before C6.
    // See EngineTorqueMap::pre_solve's starvation step.
    real fuel_available_kg = std::numeric_limits<real>::infinity();
    // PHYS-033 P4 I7b C9: ambient atmospheric sample (ps/world/ambient.h),
    // from ComponentSubstepCtx::ambient. An environment INPUT, not an ECU
    // sensor derived from engine state - the graph engine (EngineTorqueMap)
    // ignores it entirely (owner ruling Q10); a future simulated engine
    // (I7b C11+) reads it to model altitude effects on power. Never hashed.
    AmbientSample ambient{};
    // PHYS-033 P4 I7d D5: the anti-lag control channel's value this tick
    // (Powertrain copies it from the channel the turbo configuration's
    // anti_lag block names). NaN = no channel wired (a direct pre_solve call,
    // or a hot-swapped configuration naming a channel the vehicle did not
    // declare at build time): the engine then uses the block's
    // default_enabled. Ignored by every engine without an anti_lag block.
    real anti_lag_channel = std::numeric_limits<real>::quiet_NaN();
    // N2O plan N1: the nitrous system's flows this substep (default = none).
    NitrousInput nitrous{};
    // Engine thermal plan T1 (vault PHYS-063): oil-viscosity friction factors from the Powertrain's EngineThermal
    // (both exactly 1.0 without one - and x*1.0 == x bit-exactly, so an engine without thermal is unchanged).
    // friction_scale = the PLANT's k(T_oil); ecu_friction_scale = the ECU's quantised estimate of it (T1: the ECU
    // knows the cold-friction scale, plan D7). One tick old by construction. Both engines use the plant k for the
    // physics and the ECU k for their ECU models (T1b: the graph engine's idle feed-forward and torque demand).
    // T1b: k no longer multiplies the friction map; it sets the hydrodynamic increment
    // (k - 1) * F(friction_reference_rpm) * sqrt(rpm / friction_reference_rpm) (hydrodynamic_friction.h), and the
    // Coulomb/breakaway part is never scaled. friction_reference_rpm is unused while k == 1.
    real friction_scale = 1.0;
    real ecu_friction_scale = 1.0;
    real friction_reference_rpm = 2000.0;
};
// PHYS-033 P4 I7a C3: the throttle_stage.h stage's own output - a single
// combined command plus the TCU-forwarded (electronic) or fixed (mechanical)
// floor/ceiling the engine applies AFTER its own governor (see this file's
// header comment / throttle_stage.h for why floor/ceiling stay in the
// engine rather than moving into the stage). Replaces Sensors::
// pedal_throttle/hand_throttle and TcuRequest::throttle_floor/
// throttle_ceiling below.
struct ThrottleCommand {
    real command = 0.0; // pedal (through an optional pedal_map/linkage_curve), maxed with an optional hand lever
    real floor = 0.0;   // electronic: the TCU's lagged auto-blip floor; mechanical: 0 (fixed, no auto-blip)
    real ceiling = 1.0; // electronic: the TCU's lagged post-upshift-cut ceiling; mechanical: 1 (fixed)
};
// From the TCU (one substep lagged - plan section 7.2's deliberate order
// independence). Trimmed to ignition_cut only (PHYS-033 P4 I7a C3):
// throttle_floor/throttle_ceiling moved into ThrottleCommand above - a
// mechanical linkage carries ignition_cut (the upshift cut, which acts on
// the engine's ignition directly) but not the auto-blip floor/pedal-ceiling
// ramp (electronic-only, plan section 3.4.5's rejection list). Hoisted from
// EngineTorqueMap::TcuRequest.
struct TcuRequest {
    bool ignition_cut = false; // upshift cut
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
    // without a governor) - PowertrainSnapshot::governor_setpoint_rpm,
    // never hashed. PHYS-033 P4 I7a C3: the hand-lever value itself moved
    // to the throttle_stage.h stage (Powertrain::governor_hand_throttle()
    // now reads the stage's own cached raw hand-lever value, not this
    // struct).
    real governor_setpoint_rpm = 0.0;
    // Net crank torque (Coulomb row included) the current throttle demand
    // (pedal/blip/idle governor) asks for at this rpm, IGNORING the rev
    // limiter: motoring + theta_eff * (wot - motoring). The TCU's
    // torque-following clutch close uses it as its feed-forward - the
    // limiter is a speed governor that answers the clutch's own load,
    // so feeding its (clutch-dependent) output back into the clutch
    // target held a flared engine on the limiter indefinitely.
    // Turbo MVEM also includes current-charge uncut net torque when larger
    // than the ambient steady request, so clutch capacity carries boost.
    real torque_demand = 0.0;
    // PHYS-033 P4 I7a C2: the post-limiter combustion fraction theta_c
    // (engine_torque_map.cpp's own `theta_c = comb ? theta_eff*f_lim : 0`,
    // the value multiplying (w - m) in tau0 = s*(m+c_k) + theta_c*(w-m) +
    // tau_start). Write-only in C2: nothing reads it for physics yet -
    // PowertrainSnapshot::EngineSoundState::load publishes it for sound
    // (vehicle_instance.cpp), never hashed.
    real theta_c = 0.0;
    // PHYS-033 P4 I7a C5: this substep's fuel flow rate in g/s (0 without a
    // fuel map), evaluated at this same substep-start rpm/theta_c. Write-only
    // for physics (post_solve's fuel_mass_delta is its own independent
    // computation, not derived from this): published for
    // PowertrainSnapshot::fuel_flow_g_s/EngineSoundState::fuel_flow_g_s,
    // never hashed.
    real fuel_flow_g_s = 0.0;
    // PHYS-033 P4 I7a C6: the starvation factor s_starve in [0, 1] applied
    // this substep - clamp(fuel_available_kg / (h*mdot_demand), 0, 1), 1
    // when the demand is 0 or the supply covers it. tau_comb, its share of
    // d, theta_c above and the booked fuel mass are all multiplied by it.
    // Never hashed (a function of hashed state and the tank content).
    real fuel_supply_fraction = 1.0;
    // PHYS-033 P4 I7b C12: mean-value (simulated) engine sound/HUD state,
    // published through EngineSoundState. 0 for the graph engine
    // (EngineTorqueMap), which has no manifold. Never hashed, never read back
    // for physics.
    real throttle_position = 0.0;      // plate position x_thr in [0, 1] after the actuator lag
    real turbo_rpm = 0.0, boost_bar = 0.0, p_exhaust_pa = 0.0;
    real p_charge_pa = 0.0, t_charge_k = 0.0; // compressor supply, 0 = atmospheric path
    // PHYS-033 P4 I7d D1: further turbo sound/HUD state, 0 without a turbo.
    real wastegate_opening = 0.0;          // wastegate actuator position, 1 = fully open
    real compressor_flow_kg_s = 0.0;       // total compressor mass flow (the pair)
    bool turbo_compressor_stage_used=false, turbo_turbine_stage_used=false, turbo_stage_map_fallback=false;
    real compressor_pressure_ratio = 0.0;  // compressor outlet over ambient, >= 1 with a turbo
    // PHYS-033 P4 I7d D2: blow-off valve, 0 without a valve.
    real bov_opening = 0.0;                // valve position in [0, 1]
    real bov_flow_kg_s = 0.0;              // vent flow (part of compressor_flow_kg_s)
    std::uint8_t bov_vent = 0;             // 0 atmosphere, 1 recirculate (sound only)
    std::uint32_t bov_events = 0;          // opening events since the engine was built (rising edge > 0.1, re-armed below 0.05)
    // PHYS-033 P4 I7d D3: surge flag (no dynamics, unhashed), 0 without a surge line.
    std::uint8_t surge_active = 0;
    std::uint32_t surge_events = 0;        // entries into surge since the engine was built
    // PHYS-033 P4 I7d D5: anti-lag (unhashed here; the engine hashes its own
    // state), 0 without an anti_lag block.
    std::uint8_t anti_lag_active = 0;
    std::uint32_t anti_lag_activations = 0;  // rising edges of anti_lag_active since the engine was built
    real anti_lag_extra_exhaust_w = 0.0;     // indicated work rate the retard removes this substep (W); by the exhaust energy balance it stays in the exhaust gas
    // Exhaust temperatures (exhaust_thermal, turbo.md), 0 without a turbo. Never hashed here.
    real exhaust_port_k = 0.0;               // gas temperature at the exhaust ports (this substep's energy balance)
    real turbine_inlet_k = 0.0;              // gas temperature at the turbine inlet, after the exhaust wall
    real exhaust_wall_k = 0.0;               // lumped exhaust manifold / turbine housing wall temperature
    real anti_lag_manifold_burn_w = 0.0;     // anti-lag cylinder cut: fuel energy released in the exhaust manifold (W)
    real p_intake_pa = 0.0;           // intake manifold pressure p_im
    real lambda = 0.0;                 // commanded air/fuel ratio / stoichiometric (the ECU's lambda target)
    real spark_advance_deg = 0.0;      // applied spark advance (ECU target, knock-limited)
    real knock_retard_deg = 0.0;       // engine garage P7: schedule minus the knock limit where it limits, else 0 (diagnostic)
    real exhaust_mass_flow_kg_s = 0.0; // (air + booked fuel) per revolution * rev/s (+ N2O while it flows)
    // N2O plan N2 (section 6): the wet charge model's state, 0 unless the charge model ran this substep. Never
    // hashed, never read back for physics.
    real charge_temperature_k = 0.0;   // T_mix: the cylinder charge after the N2O evaporated into the air
    real n2o_lambda_combined = 0.0;    // lambda_c = O2-equivalent air / (AFR_st * (ECU + kit fuel))
    real n2o_kit_fuel_g_s = 0.0;       // kit fuel booked this substep (g/s), part of fuel_flow_g_s
};

// Hoisted, unchanged except the three PHYS-033 P4 I7a C5 fields below, from
// EngineTorqueMap::PostSolveResult.
struct PostSolveResult {
    real engine_out_delta = 0.0; // ledger.engine_out += this
    real starter_in_delta = 0.0; // ledger.starter_in += this
    // PHYS-033 P4 I7a C5 (review-fixed): combustion's own work this substep,
    // h*omega_new*tau_comb - the same "work at the solved omega_new"
    // construction engine_out_delta's own `work` uses, just for tau_comb_
    // alone (NOT a proportional split of `work` by tau_comb_/tau0_, which is
    // singular when tau0_ -> 0 with tau_comb_ and d(omega)/dt still nonzero -
    // see EngineTorqueMap::post_solve's own comment). Computed for EVERY
    // torque_map engine, fuel map or not (owner ruling: an engine without a
    // map still models combustion mechanically, and this quantity feeds the
    // crude no-map fuel estimate - PHYS-033 P4 I7a C5d). ledger.indicated_work
    // += this, UNCONDITIONALLY, in Powertrain::post_solve_components - only
    // the mass/LHV-derived fuel_mass/fuel_energy_in/engine_heat/
    // fuel_efficiency_violations fields stay gated on has_fuel_model (C5:
    // has_fuel_map alone; C5d broadens this to has_fuel_map || !fuels.empty(),
    // see ps::drivetrain::has_fuel_model).
    real indicated_work_delta = 0.0;
    // PHYS-033 P4 I7a C5/C5d: fuel burned this substep, evaluated at
    // pre_solve's substep-start rpm/load against either a calibrated fuel map
    // or (C5d, no map) the crude constant-efficiency estimate
    // mdot = max(0, tau_comb*omega_old) / (eta_i * LHV) - both exactly 0.0
    // only for an engine with no fuel model at all (no "fuel_map" and no
    // "fuels" list). ledger.fuel_mass/fuel_energy_in += these.
    real fuel_mass_delta = 0.0;      // kg this substep (h * mdot)
    real fuel_energy_in_delta = 0.0; // J this substep (fuel_mass_delta * LHV)
    // PHYS-033 P4 I7b C12: the simulated engine's pumping and friction work
    // this substep, h*omega_old*tau_pump / h*omega_old*tau_fric (both <= 0
    // when they oppose rotation). 0 for EngineTorqueMap, whose motoring curve
    // lumps them. ledger.pumping_work/friction_work += these - diagnostic
    // only, OUTSIDE balance_error (engine_out already carries them).
    real pumping_work_delta = 0.0;
    real friction_work_delta = 0.0;
    // Engine thermal plan T1: the heat this substep puts into the thermal model. released_heat_delta = the
    // combustion energy released (J, eta_c * fuel energy; the graph engine uses eta_c = 1); friction_heat_delta =
    // the engine's own friction work turned into heat (J, >= 0 to rounding; smooth friction only - the Coulomb row's
    // loss is added by Powertrain). Both 0 until an engine fills them; read only by EngineThermal.
    real released_heat_delta = 0.0;
    real friction_heat_delta = 0.0;
    // N2O plan N2 (section 4), all 0 unless the engine received N2O this substep (and always 0 from the graph
    // engine, which ignores it until N6). n2o_mass_delta = the N2O the engine took in (h * mdot_n);
    // n2o_unburnt_delta = its pass-through share (a cut or starving substep); the burnt rest released
    // n2o_energy_in_delta (J, decomposition heat). kit_fuel_mass_delta (kg) is the kit-jet share of
    // fuel_mass_delta. indicated_work_delta keeps its definition and includes the decomposition share.
    real n2o_mass_delta = 0.0;
    real n2o_energy_in_delta = 0.0;
    real n2o_unburnt_delta = 0.0;
    real kit_fuel_mass_delta = 0.0;
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
    // PHYS-033 P4 I7a C3: `throttle` is the throttle_stage.h stage's own
    // output (pedal/hand-lever already combined, floor/ceiling forwarded
    // electronic or fixed mechanical) - the engine never reads the pedal
    // directly.
    [[nodiscard]] virtual PreSolveResult pre_solve(const Sensors& sensors, const ThrottleCommand& throttle, const TcuRequest& tcu, real h) = 0;
    // omega_new: the crank node's omega after the network solve.
    [[nodiscard]] virtual PostSolveResult post_solve(real omega_new, real h) = 0;

    [[nodiscard]] virtual real idle_integrator() const = 0;
    [[nodiscard]] virtual real crank_angle() const = 0; // NOT hashed (sound only)
    // The engine's own idle setpoint (rpm) - Powertrain::reset uses this to
    // seed the crank at idle when there is no driveline speed to derive one
    // from yet (plan section 4.5's bootstrap). See the class comment above
    // for why this is a narrow virtual rather than exposing the whole Desc.
    [[nodiscard]] virtual real idle_rpm() const = 0;
    // N2O plan N1: the pressure the intake sees right now (Pa abs) - the
    // downstream pressure of the nitrous and kit-fuel jets. A mean-value engine
    // returns its manifold pressure; the graph engine has no manifold and
    // returns the ambient pressure of its last Sensors. Never hashed.
    [[nodiscard]] virtual real intake_pressure_pa() const = 0;

    [[nodiscard]] virtual std::uint64_t fold_state_hash(std::uint64_t seed) const = 0;

    // World::reset_vehicle (plan section 4.5): forces the engine directly
    // into `state`, clearing whatever transient latches/integrators it
    // owns. The crank's own network omega is set separately by the caller
    // (ShaftNetwork::set_omega).
    virtual void reset(EngineState state) = 0;
};

} // namespace ps::drivetrain
