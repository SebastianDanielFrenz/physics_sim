#include <memory>
// ps/drivetrain/powertrain_desc.h — public, additive descriptors for the
// P4 shaft-network powertrain (PHYS-033 P4 I3 plan section 4.1). Pure
// data; no algorithm code, no Jolt/Godot type (same D2/D10 pattern as
// ps/articulation/articulation_desc.h) - the graph-building/solving
// machinery lives in core/src/drivetrain (internal, ps::drivetrain).
//
// I3 replaces I2's placeholder engine/limiter shapes wholesale (no legacy
// converter - PHYS-033 owner ruling Q7's precedent, plan section 9.1's
// "physics_sim.vehicle/2 stays /2, loud failure instead"). engine, clutch,
// gearbox, differential, gear_pair are the I3 component catalogue; I4 added
// LSD/locked diffs, transfer case, park lock, pto and (C13) compliant shafts.

#pragma once

#include "ps/drivetrain/engine_cycle_desc.h" // CycleModelDesc (PHYS-033 P4 I7c C16a, SimulatedEngineDesc::cycle)
#include "ps/drivetrain/fuel_desc.h" // FuelDesc (PHYS-033 P4 I7a C5, TorqueMapEngineDesc::fuel)
#include "ps/drivetrain/table2d.h"   // Table2D (PHYS-033 P4 I7a C5, TorqueMapEngineDesc::fuel_map)
#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h" // Curve1D (NOT vehicle_desc.h: VehicleDesc::powertrain
                                 // is a PowertrainDesc, so that include would be circular)

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace ps::drivetrain {

// "component.port" (e.g. "final_drive.in") or "wheel:<name>" (is_wheel
// true, `component` holds the wheel's WheelDesc::name).
struct PortRef {
    std::string component;
    std::string port;
    bool is_wheel = false;
};

// Joins ports into one rigid node (PHYS-033 P4 I2 plan section 2.3). Under
// PowertrainDesc::compliance == Compliant (PHYS-033 P4 I4 C13), a shaft with
// stiffness > 0 instead joins EXACTLY two ports through a torsional spring-
// damper: the two ports stay separate nodes (each must carry inertia > 0),
// twist theta = integral(omega_a - omega_b), torque tau_s = k*theta + c*
// (omega_a - omega_b) acts -tau_s on ports[0] and +tau_s on ports[1]. Under
// Rigid every shaft is rigid and stiffness/damping are ignored.
struct ShaftDesc {
    std::vector<PortRef> ports;
    real stiffness = 0.0; // N*m/rad, >= 0; > 0 makes the shaft compliant (Compliant only)
    real damping = 0.0;   // N*m*s/rad, >= 0; > 0 requires stiffness > 0
};

// PHYS-033 I3 plan section 4.1: I2's "hard_cut" placeholder is gone - the
// loader rejects it with a pointer to fuel_cut/soft_cut (section 9.1).
enum class LimiterType { FuelCut, SoftCut };
struct EngineLimiterDesc {
    LimiterType type = LimiterType::FuelCut;
    real rpm = 6800.0;            // FuelCut: cut at >= rpm. SoftCut: zero fuel at rpm (governor high idle)
    real hysteresis_rpm = 150.0;  // FuelCut only: restore at <= rpm - hysteresis
    real band_rpm = 200.0;        // SoftCut only: linear taper over [rpm - band, rpm] (governor droop)
};

struct IdleControlDesc { // PI on throttle (idle air bypass), plan section 6.1.4
    real kp_per_rpm = 0.0018;    // throttle fraction per rpm of error
    real ki_per_rpm_s = 0.009;   // throttle fraction per (rpm*s)
    real max_throttle = 0.3;
    // SimulatedEngine only (owner 2026-09-29): > 0 caps the idle controller's
    // authority at the plate/valve whose steady net torque at idle_rpm is
    // this much above zero (the no-load feed-forward) - enough to creep in
    // gear, not a hidden launch aid. 0 = max_throttle alone.
    real max_torque_nm = 0.0;
};

struct StarterDesc { // ideal DC starter referred to the crank (I9 puts it on the LV bus)
    real stall_torque_nm = 140.0; // at 0 rpm
    real free_speed_rpm = 400.0;  // torque falls linearly to 0 here
    enum class Interlock { None, ClutchOrNeutral } interlock = Interlock::ClutchOrNeutral;
};

enum class EngineInitialState { Running, Off };

// PHYS-033 P4 I4 C12 (plan section 3.6): an all-speed diesel governor with
// droop, inside the engine's own ECU - REPLACES the pedal in the throttle-
// request expression (engine_torque_map.cpp's theta_req) whenever present.
// lever = max(hand_throttle channel, pedal_throttle), each clamped to
// [0, 1]. n_nl = idle_rpm + lever*(high_idle_rpm - idle_rpm) is the no-load
// setpoint; band = droop*rated_rpm (ISO 3046-4 steady-state speed droop
// referred to rated speed); theta_gov = clamp((n_nl - n)/band, 0, 1). See
// docs/data_formats/engine.md for the full derivation (including the
// implicit dtau/domega slope contribution) and engine_torque_map.cpp for
// the implementation. Optional - no governor block means engine_torque_map.
// cpp's throttle path is byte-identical to before this commit.
struct GovernorDesc {
    real high_idle_rpm = 2400.0; // > idle_rpm; the no-load setpoint at full lever
    real rated_rpm = 2200.0;     // in (idle_rpm, high_idle_rpm]; droop reference speed (ISO 3046-4)
    real droop = 0.0909;         // in (0, 0.3]; steady-state speed droop referred to rated_rpm
    std::string channel = "hand_throttle"; // declared (default 0) via Powertrain::declare_control_channels; non-empty
};

// PHYS-033 P4 I7a C3 (plan section 3.4): how an engine's throttle command
// physically reaches it - an electronic throttle body (a pedal position
// SENSOR, an ECU can intervene: auto-blip floor, post-upshift-cut ceiling,
// ignition cut) or a mechanical linkage/cable (direct physical connection -
// only ignition cut, which acts on the engine's ignition directly rather
// than through the throttle path, still applies; see throttle_stage.h's
// header comment and docs/data_formats/vehicle.md's throttle_control
// section for the full "which intervention lives where" table). Also
// ThrottleControlDesc::kind below (the same two values, reused rather than
// a second identical enum).
enum class ThrottleInput { Electronic, Mechanical };

// PHYS-033 P4 I7a C5d: spark vs. compression ignition. Owner ruling
// 2026-09-27 (reversing an earlier C5c attempt to put this on FuelDesc
// instead): ignition is an ENGINE property, not a fuel one - any fuel
// autoignites given enough compression, real fuels differ only in ignition
// QUALITY (octane/cetane rating), not in which cycle they are "for". Used
// only to pick the crude no-map fuel model's default indicated efficiency
// (see TorqueMapEngineDesc::indicated_efficiency below) - never dispatches
// any other physics.
enum class EngineIgnition { Spark, Compression };

struct TorqueMapEngineDesc {
    vehicle::Curve1D wot_torque_nm_vs_rpm;       // required; x ascending, x[0] >= 0
    vehicle::Curve1D motoring_torque_nm_vs_rpm;  // required (I3); y <= 0; x[0] == 0 (its y[0] is the Coulomb part)
    real inertia_kgm2 = 0.0;                     // REQUIRED > 0 in I3 (crank + flywheel + clutch cover)
    real breakaway_torque_nm = 0.0;              // static Coulomb capacity; >= -motoring(0)
    real idle_rpm = 800.0, stall_rpm = 400.0, fire_rpm = 250.0, start_complete_rpm = 550.0;
    real dfco_enable_rpm = 1300.0, dfco_reenable_rpm = 1000.0;
    IdleControlDesc idle;
    EngineLimiterDesc limiter;
    StarterDesc starter;
    Vec3 crank_axis_local = Vec3::zero(); // chassis frame, direction of +omega; zero = no rotor reaction (plan section 6.7)
    EngineInitialState initial_state = EngineInitialState::Running;
    bool has_governor = false; // I4 C12: only meaningful (and validated) when true
    GovernorDesc governor;
    // PHYS-033 P4 I7a C3: how the throttle command physically reaches this
    // engine - drives the IMPLICIT throttle_control stage when no vehicle
    // (PowertrainDesc::components) explicit throttle_control component
    // names this engine. Optional for a torque_map engine (default
    // Electronic, matching every engine shipped before this commit); a
    // future simulated engine type is expected to require it explicitly.
    ThrottleInput throttle_input = ThrottleInput::Electronic;
    // PHYS-033 P4 I7a C5: an optional fuel map (rpm x load(theta_c) ->
    // fuel_flow_g_s), plus the FuelDesc (LHV) it needs to turn a booked
    // mass into energy. has_fuel_map false (default) means NO fuel
    // computation happens anywhere downstream - byte-identical to every
    // engine shipped before this commit (no data/vehicles/data/engines file
    // sets this yet; C7 ships the first). See docs/data_formats/engine.md.
    bool has_fuel_map = false;
    FuelDesc fuel;
    Table2D fuel_map; // x = rpm (ascending), y = load in [0, 1] (ascending), values in g/s
    // PHYS-033 P4 I7a C5d: spark/compression ignition (see EngineIgnition
    // above) - has_ignition is false only when neither "ignition" nor any
    // fuel field is present in the source file; the loader REQUIRES
    // "ignition" whenever a fuel model exists (has_fuel_map or a non-empty
    // fuels list below) and otherwise leaves it a harmless stored-but-
    // unread default.
    bool has_ignition = false;
    EngineIgnition ignition = EngineIgnition::Spark;
    // PHYS-033 P4 I7a C5d: fuels this engine can burn, for the crude
    // no-map estimate below - plain path strings resolved to FuelDesc at
    // load time (a hypothetical per-entry blend weight, C5e, was cancelled/
    // dropped per owner ruling 2026-09-27: no blending, just a list).
    // fuels[0] is the ACTIVE fuel whose lhv_j_kg/afr_stoich feed the crude
    // model - order matters. At the JSON level, "fuels" and the singular
    // "fuel"/"fuel_map" keys are mutually exclusive (a calibrated fuel_map
    // is keyed to exactly one fuel). Internally, though (review fix B1),
    // singular "fuel" is TRUE SHORTHAND for a one-entry fuels list: the
    // loader (vehicle_io.cpp) always sets `fuels = {fuel}` whenever "fuel"
    // is present, whether or not "fuel_map" is ALSO present alongside it -
    // so this field is never left empty just because an engine happened to
    // spell its fuel reference as "fuel" instead of "fuels": [...]. When a
    // fuel_map exists, to_engine_desc's crude-model branch is gated off
    // anyway (the map path wins), so populating this vector unconditionally
    // is harmless. Empty (default, only for an engine naming no fuel at
    // all) means this engine has no crude fuel model - has_fuel_map is then
    // the only source of fuel computation (see TorqueMapEngineDesc's own
    // has_fuel_model() free function below).
    std::vector<FuelDesc> fuels;
    // PHYS-033 P4 I7a C5d: override for the crude no-map model's assumed
    // indicated thermal efficiency eta_i (mdot = max(0, tau_comb*omega) /
    // (eta_i * LHV)). 0.0 (default, sentinel - valid values are in (0, 1)
    // exclusive) means use the Heywood (1988) ignition-type default: 0.325
    // spark (derived midpoint, p.827), 0.48 compression (p.859) - see
    // powertrain.cpp's to_engine_desc for the full citations, which cover
    // two DIFFERENT kinds of operating point (an acknowledged
    // inconsistency, not hidden). Ignored by the fuel_map path (has_fuel_map
    // uses the map's own measured flow, not an assumed efficiency) - the
    // loader rejects setting this both without any fuel model AND (review
    // fix N9) alongside a fuel_map specifically, since the map would never
    // consult it.
    real indicated_efficiency = 0.0;
};

// PHYS-033 P4 I7a C5d: true whenever this engine has ANY fuel source (a
// calibrated fuel_map or the crude no-map `fuels` list) - distinct from
// has_fuel_map, which is specifically "a measured map exists" and picks
// which of the two mutually-exclusive fuel-flow computations runs. Kept as
// a free function (not a method) to match this file's own "pure data, no
// algorithm code" convention.
inline bool has_fuel_model(const TorqueMapEngineDesc& e) {
    return e.has_fuel_map || !e.fuels.empty();
}

// --- PHYS-033 P4 I7b C10: the simulated (mean-value, MVEM) engine ----------
//
// "kind": "simulated" in an engine file (docs/data_formats/engine.md,
// "simulated" section). Pure data in SI units - the loader converts the
// file's mm / kPa / g / L into m / Pa / kg / m^3. The runtime model
// (core/src/drivetrain/engine_simulated.h, PHYS-033 P4 I7b C11) derives the
// WOT and motoring curves itself, so this desc carries none. Every map is
// an INLINE table - first-class schema, not a test-only shortcut (I7c adds
// generated maps behind this same desc; the first shipped simulated car may
// use literature/EPA-derived tables written straight into the file).

enum class EngineArchitecture { Piston4Stroke, Rotary };

// Piston 4-stroke: displacement = cylinders * pi/4 * bore^2 * stroke; one
// intake stroke per cylinder every 2 crank revolutions (c_cycle 1/2).
struct PistonGeometryDesc {
    real bore_m = 0.0;   // > 0
    real stroke_m = 0.0; // > 0
    int cylinders = 0;   // >= 1
};

// Wankel rotary (PHYS-033 section 5 / I7 plan section 3.1): displaced volume
// per face 3*sqrt(3)*R*e*b; per e-shaft revolution every rotor inducts one
// face (c_cycle 1). The rotary runtime is I7e C13 - C11 accepts the desc and
// computes the geometry but refuses to run it.
struct RotaryGeometryDesc {
    real rotor_radius_m = 0.0; // R (generating radius), > 0
    real eccentricity_m = 0.0; // e, in (0, R)
    real rotor_width_m = 0.0;  // b, > 0
    int rotors = 0;            // >= 1
};

// Throttle plate(s). Flow area at plate position x in [0, 1] is
// count * pi/4 * diameter^2 * area_fraction_vs_position(x); the effective
// area is discharge_coefficient times that.
struct ThrottleBodyDesc {
    real diameter_m = 0.0;              // > 0
    int count = 1;                      // >= 1 (individual throttle bodies share one position)
    real discharge_coefficient = 0.0;   // in (0, 1]
    vehicle::Curve1D area_fraction_vs_position; // x exactly 0..1 ascending, y in [0, 1] non-decreasing, y(0) > 0 (a
                                                // closed plate still leaks - the fixed point needs a nonzero area)
};

// The runtime maps (I7 plan section 3.1). Every Table2D here is x = rpm,
// y = its own load axis, as noted; every Curve1D its own x.
//
// eta_i is the GROSS indicated efficiency at MBT spark and lambda = 1
// (I7c planner request, adopted in C10): the indicated work per cycle is
//     W_ind = eta_i(rpm, m_air_rev) * f_lambda(lambda) * eta_spark(theta - theta_MBT) * LHV * m_fuel
// so f_lambda and eta_spark are applied ON TOP and must equal 1 at their
// reference points (the loader enforces f_lambda(1) == 1 and
// eta_spark(0) == 1). Pumping and friction are separate torque terms, never
// folded into eta_i.
struct SimulatedEngineMapsDesc {
    Table2D ve;                            // y = p_im [Pa]; volumetric efficiency, values in (0, 2)
    Table2D eta_i;                         // y = air mass per crank revolution [kg]; gross indicated efficiency in (0, 1)
    vehicle::Curve1D f_lambda;             // x = lambda (ascending, > 0); y >= 0; f_lambda(1) == 1
    vehicle::Curve1D eta_spark;            // x = theta - theta_MBT [deg]; y in [0, 1]; eta_spark(0) == 1
    Table2D theta_mbt_deg;                 // y = p_im [Pa]; MBT spark advance [deg BTDC]
    vehicle::Curve1D friction_torque_nm;   // x = rpm (x[0] == 0); y >= 0 (FMEP-equivalent torque; y(0) is the Coulomb part)
    // Reserved (I7c planner request): optional extra pumping loss over the
    // ideal (p_em - p_im) term, e.g. valve-flow losses. Absent = 0.
    bool has_pmep_excess = false;
    Table2D pmep_excess_pa;                 // y = p_im [Pa]; signed, finite, |value| <= 1e6 [Pa] (I7c C16b review N1:
                                            // the runtime uses this as a plain signed additive term - see
                                            // engine_simulated.cpp's tau_pump - so a file value follows the same
                                            // signed rule the generator already produces)
    // Reserved (I7c planner request): knock-limited spark advance. Absent =
    // no knock limit; present = spark used = min(ecu spark target, theta_kl).
    bool has_theta_kl = false;
    Table2D theta_kl_deg;                  // y = p_im [Pa]; [deg BTDC]
    std::string source;                    // required (PLAN.md section 8)
};

// Open-loop engine-management targets.
struct SimulatedEcuDesc {
    Table2D lambda_target;     // y = p_im [Pa]; values in [0.5, 2]
    Table2D spark_advance_deg; // y = p_im [Pa]; [deg BTDC], values in [-30, 70]
};

// Reduced symmetric parallel turbo pair. Each turbo sees half the flow;
// inertia/flow capacity below are totals for the pair. No measured OEM maps.
struct TurboPerformanceMaps;

// PHYS-033 P4 I7d D2: compressor bypass (blow-off / diverter) valve, an
// optional part of a turbo configuration (docs/data_formats/turbo.md). Its
// diaphragm sees charge pressure below and manifold pressure above: the
// command is clamp((p_charge - p_im - crack_dp)/(full_open_dp - crack_dp), 0, 1)
// and the opening follows with a first-order lag tau_s. Vent flow is an
// isentropic orifice of effective_area_m2 (Cd included) from the charge state
// to ambient; it adds to the COMPRESSOR flow, not to the engine's air.
// Extensive: the loader multiplies the per-turbo area by the turbo count.
struct BlowOffValveDesc {
    // Sound only (the physics is identical): the vented air leaves to the
    // atmosphere or is recirculated to the compressor inlet.
    enum class Vent : std::uint8_t { Atmosphere = 0, Recirculate = 1 };
    Vent vent = Vent::Atmosphere;
    real crack_dp_pa = 0.0;       // >= 0
    real full_open_dp_pa = 0.0;   // > crack_dp_pa
    real effective_area_m2 = 0.0; // > 0, pair total
    real tau_s = 0.0;             // >= 0 (0 = the opening follows the command at once)
    std::string source;           // required (PLAN.md section 8)
};

// PHYS-033 P4 I7d D3: compressor surge line of a turbo HARDWARE file
// (physics_sim.turbo/1 "surge_line", docs/data_formats/turbo.md): the surge
// flow as a fraction of max_mass_flow_kg_s over the compressor pressure ratio.
// It drives only a flag (TurboPair::surge_active); nothing dynamic reads it,
// so it is in neither the state hash nor the derived-map cache key.
struct SurgeLineDesc {
    ps::vehicle::Curve1D flow_fraction_vs_pr; // x strictly increasing pressure ratio (>= 1), y in [0, 1)
    std::string source;          // required (PLAN.md section 8)
};

// PHYS-033 P4 I7d D5: anti-lag (ALS), an optional part of a turbo
// configuration (docs/data_formats/turbo.md) - the ECU strategy that keeps the
// rotor spinning while the driver is off the throttle. Active while the
// channel reads >= 0.5, the engine's throttle command is below
// arm_below_command, rpm is in [min_rpm, max_rpm], the engine is Running and
// did not starve last substep, for at most max_active_s per lift (re-armed
// when the command rises to arm_below_command again). While active: DFCO is
// inhibited, the plate target is at least plate_floor (bypass air), the spark
// is retarded by retard_deg from the scheduled advance, and the indicated work
// the retard removes goes to the turbine as extra exhaust enthalpy.
struct AntiLagDesc {
    std::string channel = "anti_lag"; // control channel; declared with default default_enabled
    real default_enabled = 0.0;       // 0 or 1: the channel's declared default
    real arm_below_command = 0.0;     // in (0, 1]: throttle command below which it acts
    real min_rpm = 0.0;               // >= 0
    real max_rpm = 0.0;               // > min_rpm
    real retard_deg = 0.0;            // >= 0, spark retard from the scheduled advance
    real plate_floor = 0.0;           // in [0, 1], bypass-air throttle plate floor
    real max_active_s = 0.0;          // > 0, longest activity per lift
    std::string source;               // required (PLAN.md section 8)
};

struct TurboPairDesc {
    std::shared_ptr<const TurboPerformanceMaps> maps;
    real reference_head_pa = 324000.0;
    real target_boost_pa = 180000.0;
    real rotor_inertia_kgm2 = 0.00004;
    real reference_rpm = 120000.0;
    real max_mass_flow_kg_s = 0.75;
    real compressor_efficiency = 0.72;
    real turbine_efficiency = 0.70;
    real turbine_area_m2 = 0.0015;
    real intercooler_effectiveness = 0.85;
    real shaft_loss_w_at_reference = 1200.0;
    // Boost controller (PID on charge pressure, error per unit of
    // target_boost_pa; derivative on the measurement) and the wastegate
    // actuator's first-order lag. [A] Defaults from a gain sweep on
    // hyper_v8_one1_audio.json (2026-10-02, vault PHYS-058): every speed
    // settles on target with < 20 Pa ripple, 4500 rpm part throttle -> WOT
    // reaches 90 % boost in 0.27 s without overshoot. kd = 0 limit-cycles.
    real wastegate_kp = 1.0;
    real wastegate_ki_per_s = 1.0;
    real wastegate_kd_s = 0.05;
    real wastegate_actuator_tau_s = 0.10;
    // I7d D2: absent = no valve; the pair is then bit-identical to before.
    std::optional<BlowOffValveDesc> blow_off_valve;
    // I7d D3: absent = the surge flag is never raised. Not hashed.
    std::optional<SurgeLineDesc> surge_line;
    // I7d D4: pressure drop of the intercooler at the pair's max flow; at flow f (a fraction of
    // max_mass_flow_kg_s) the drop is k * f^2, and the throttle sees the compressor outlet pressure
    // minus it. 0 = no drop (bit-identical to before); hashed only when nonzero. Intensive: not
    // scaled by the turbo count.
    real intercooler_dp_pa_at_max_flow = 0.0;
    // I7d D5: absent = no anti-lag; the engine is then bit-identical to before.
    std::optional<AntiLagDesc> anti_lag;
};

struct SimulatedEngineDesc {
    EngineArchitecture architecture = EngineArchitecture::Piston4Stroke;
    PistonGeometryDesc piston; // meaningful iff architecture == Piston4Stroke
    RotaryGeometryDesc rotary; // meaningful iff architecture == Rotary
    real intake_manifold_volume_m3 = 0.0; // > 0 (plenum + runners)
    ThrottleBodyDesc throttle_body;
    std::optional<TurboPairDesc> turbo_pair;
    // Required for a simulated engine (plan section 3.4.3). Electronic: the
    // command is a TORQUE request, turned into a plate target through a
    // load-time inverse table; the plate follows through a first-order
    // actuator (throttle_actuator_tau_s >= 0; 0 = no lag). Mechanical: the
    // command IS the plate position; idle air comes from a bypass valve of
    // idle_bypass_area_m2 (> 0) driven by the idle PI.
    ThrottleInput throttle_input = ThrottleInput::Electronic;
    real throttle_actuator_tau_s = 0.0;   // electronic only (the loader rejects it on a mechanical engine)
    real idle_bypass_area_m2 = 0.0;       // mechanical only (idle_control.bypass_area_mm2 in the file)
    SimulatedEcuDesc ecu;
    SimulatedEngineMapsDesc maps;
    // PHYS-033 P4 I7c C16a: the engine file's "cycle" block, if any. The loader
    // runs the map generator (ps/drivetrain/engine_maps.h) on it and fills
    // `maps` with the result; keys given in the file's own "maps" block
    // override generated ones. The runtime never reads this field.
    std::optional<CycleModelDesc> cycle;

    // Shared with the graph engine, same meaning and validation.
    real inertia_kgm2 = 0.0;          // > 0
    real breakaway_torque_nm = 0.0;   // static Coulomb capacity; >= maps.friction_torque_nm(0)
    real idle_rpm = 800.0, stall_rpm = 400.0, fire_rpm = 250.0, start_complete_rpm = 550.0;
    real dfco_enable_rpm = 1300.0, dfco_reenable_rpm = 1000.0;
    IdleControlDesc idle;             // electronic: PI on the plate target; mechanical: PI on the bypass valve (max_throttle = max valve opening)
    EngineLimiterDesc limiter;
    StarterDesc starter;
    Vec3 crank_axis_local = Vec3::zero();
    EngineInitialState initial_state = EngineInitialState::Running;

    // Required and non-empty; fuels[0] is burnt unless a fuel tank selects
    // another entry (same rule as TorqueMapEngineDesc::fuels).
    std::vector<FuelDesc> fuels;
    EngineIgnition ignition = EngineIgnition::Spark; // required; only "spark" in I7b (a CI combustion model is later)
};

// A simulated engine always has a fuel model (the loader requires fuels).
inline bool has_fuel_model(const SimulatedEngineDesc& e) {
    return !e.fuels.empty();
}

struct ClutchDesc {
    real capacity_nm = 0.0;               // KINETIC capacity at engagement 1
    real static_factor = 1.15;            // static = static_factor * kinetic (>= 1)
    vehicle::Curve1D engagement_vs_pedal; // pedal 0 (released) .. 1 (pressed) -> engagement 1 .. 0, non-increasing
    real in_inertia_kgm2 = 0.0;           // port "in" (pressure plate; usually folded into the engine inertia)
    real out_inertia_kgm2 = 0.0;          // port "out" (driven disc)
    Vec3 axis_local = Vec3::zero();       // shared by both ports (one physical shaft), plan section 6.7
};

enum class GearboxType { Manual, Sequential };
struct SynchroDesc {
    std::vector<real> forward_capacity_nm; // per forward gear, input-shaft torque; 0 = no synchro (dog only)
    std::vector<real> reverse_capacity_nm; // per reverse gear
    real static_factor = 1.0;
    real max_dog_mismatch_rad_s = 5.0;     // at the input shaft
};
struct GearboxDesc {
    GearboxType type = GearboxType::Manual;
    std::vector<real> forward_ratios;    // > 0, strictly decreasing (1st first). w_in = ratio * w_out
    std::vector<real> reverse_ratios;    // < 0, R1 first (the slowest reverse)
    std::vector<real> forward_efficiency, reverse_efficiency; // (0, 1], same lengths as the ratios above
    real input_inertia_kgm2 = 0.0, output_inertia_kgm2 = 0.0;
    Vec3 input_axis_local = Vec3::zero(), output_axis_local = Vec3::zero();
    SynchroDesc synchro;
    real neutral_phase_s = 0.06;       // selector travel through neutral (minimum time dog-out -> synchro)
    real dog_release_torque_nm = 60.0; // |mesh torque at input| must be below this to pull a gear out
    real grind_after_s = 0.25;         // synchronising longer than this -> grind event
    real abort_after_s = 1.0;          // synchronising longer than this -> give up, stay in N
    int initial_gear = 0;              // 0 = N, k > 0 forward, -k reverse Rk
};

struct GearPairDesc {
    real ratio = 1.0;
    real efficiency = 1.0; // (0, 1]; < 1 needs a mesh-loss dissipative torque (section 5.4)
    real inertia_in_kgm2 = 0.0;
    real inertia_out_kgm2 = 0.0;
    Vec3 in_axis_local = Vec3::zero(), out_axis_local = Vec3::zero(); // plan section 6.7
};

// PHYS-033 P4 I4 C6 (plan section 3.1) adds Locked (spool: an unconditional,
// state-independent omega_L == omega_R row alongside the diff's own row) and
// Viscous (the open row plus an implicit L-R coupling, `viscous_nm_s_rad`
// below). PHYS-033 P4 I4 C7 (plan section 3.2) adds ClutchLsd and Torsen:
// the open row plus ONE Coulomb friction row between L and R (a = L, b = R,
// ratio_b = 1; +lambda on L, -lambda on R, so T_high - T_low = 2|lambda|),
// present in every topology state, whose capacity is set every substep from
// the PREVIOUS substep's axle torque (a one-substep lag, see the fields
// below). Active remains unimplemented (the loader rejects it).
enum class DifferentialKind { Open, Locked, Viscous, ClutchLsd, Torsen };

// PHYS-033 P4 I4 C5 (plan section 2.6/3.1): a lockable diff's L-R lock
// (omega_L == omega_R rigid row, present only in composed topology states
// whose lock bit is set). Engages when `channel` reads >= 0.5 AND
// |omega_L - omega_R| <= max_engage_mismatch_rad_s (else stays pending,
// retried every substep); releases the substep the channel reads < 0.5.
struct DiffLockDesc {
    real max_engage_mismatch_rad_s = 2.0; // >= 0
    std::string channel = "diff_lock";    // declared (default 0) via Powertrain::declare_control_channels
};

struct DifferentialDesc {
    DifferentialKind kind = DifferentialKind::Open;
    real ratio = 1.0;
    real efficiency = 1.0; // (0, 1] - I3 lifts I2's "must be exactly 1" ban (section 5.4's reconstruction node)
    real carrier_inertia_kgm2 = 0.0;
    Vec3 carrier_axis_local = Vec3::zero();
    bool lockable = false; // I4 C5: must be a leaf diff (its L/R carry no other rigid row)
    DiffLockDesc lock;     // only meaningful when lockable
    // I4 C6: only meaningful (and required > 0) when kind == Viscous - the
    // implicit L-R coupling's viscous coefficient (N*m*s/rad), applied every
    // substep via NetworkBuilder::add_coupling/Assembly::set_coupling(id, 1,
    // 1, viscous_nm_s_rad) alongside the diff's own (open) row.
    real viscous_nm_s_rad = 0.0;
    // I4 C7, kind == ClutchLsd only: the L-R friction row's kinetic capacity
    // is (preload_nm + f*T_axle)/2 (so T_high - T_low <= preload_nm +
    // f*T_axle), f = drive_factor while the carrier's input power last
    // substep was > 0, else coast_factor; static capacity = static_factor *
    // kinetic. T_axle = |ratio * lambda_row| of the PREVIOUS substep.
    real preload_nm = 0.0;    // >= 0
    real drive_factor = 0.0;  // [0, 1), required for clutch_lsd
    real coast_factor = 0.0;  // [0, 1), required for clutch_lsd
    real static_factor = 1.0; // >= 1, optional
    // I4 C7, kind == Torsen only (required > 1): capacity (T_axle/2)*(TBR-1)/
    // (TBR+1), static == kinetic, so T_high/T_low <= torque_bias_ratio.
    real torque_bias_ratio = 0.0;
};

// PHYS-033 P4 I4 C9 (plan section 3.3): mechanical front-wheel-drive (MFWD)
// transfer case - ports `in` (from the gearbox), `rear` (rigid, ratio
// `rear_ratio`: omega_in = rear_ratio*omega_rear, present in every topology
// state) and `front` (a Coulomb friction row: a = in, b = front, ratio_b =
// front_ratio, so slip v = omega_in - front_ratio*omega_front). `engagement`
// (e in [0,1]) ramps toward the channel's target (1 when the channel reads
// >= 0.5, else 0) at rate 1/engage_time_s (engage_time_s == 0: instant);
// kinetic capacity = e*capacity_nm, static = static_factor*kinetic - the same
// mechanism DirectTorqueBrakes/Clutch already use. `channel` is declared via
// Powertrain::declare_control_channels (default 0 = disengaged).
struct TransferCaseClutchDesc {
    real capacity_nm = 0.0;    // KINETIC capacity at engagement 1, > 0
    real static_factor = 1.15; // static = static_factor*kinetic, >= 1
    real engage_time_s = 0.3;  // >= 0; 0 = instant engagement
};
struct TransferCaseDesc {
    real rear_ratio = 1.0;  // != 0; omega_in = rear_ratio*omega_rear (rigid row)
    real front_ratio = 1.0; // != 0; friction row's ratio_b (slip v = omega_in - front_ratio*omega_front)
    TransferCaseClutchDesc clutch;
    std::string channel = "awd"; // declared (default 0) via Powertrain::declare_control_channels; non-empty
    real in_inertia_kgm2 = 0.0, front_inertia_kgm2 = 0.0;
    Vec3 axis_local = Vec3::zero(); // shared by `in`/`front` ports (one physical shaft), plan section 6.7
};

// PHYS-033 P4 I4 C10 (plan section 3.4, owner ruling): a park pawl on a
// single port `shaft` - one Coulomb friction row, node `shaft` to GROUND
// (no rigid row, no topology-state bit - it never changes network
// topology, only a friction row's capacity, exactly like DirectTorqueBrakes'
// per-wheel rows or TransferCaseClutchDesc's own capacity ramp). Engages
// (kinetic capacity = capacity_nm, static = static_factor*kinetic) the
// first substep `channel` reads >= 0.5 AND |omega| (of the `shaft` node,
// read BEFORE this substep's solve) <= max_engage_speed_rad_s; above that
// speed the request stays pending, retried every substep (same shape as
// DiffLockDesc's pending mechanism, but no topology switch is involved so
// no momentum-projection/engagement-loss bookkeeping is needed either).
// Releases (capacity 0) the first substep `channel` reads < 0.5 - no ramp,
// a pawl either holds or it doesn't. `channel` is declared (default 0) via
// Powertrain::declare_control_channels; at most one park_lock per vehicle.
struct ParkLockDesc {
    real capacity_nm = 0.0;              // KINETIC capacity while engaged, > 0
    real static_factor = 1.0;            // static = static_factor*kinetic, >= 1
    real max_engage_speed_rad_s = 5.0;   // >= 0; pawl can only engage at or below this speed
    std::string channel = "park_lock";   // declared (default 0) via Powertrain::declare_control_channels; non-empty
};

// PHYS-033 P4 I4 C11 (plan section 3.5): power take-off - ports `in`
// (shafted, e.g. to engine.crank/clutch.in - "live", ahead of the main
// clutch) and `out` (external; EXEMPT from Powertrain's "port not connected
// by any shaft" check, so it may stand alone carrying only
// out_inertia_kgm2, or be shafted to an external load such as a
// load_dyno). One Coulomb friction row (a = in, b = out, ratio_b =
// ratios[mode] - 3.61 for "540", 1.95 for "1000") re-set every substep via
// Assembly::set_friction_row_ratio (TransferCase's pattern). `engagement`
// (e in [0,1]) ramps toward the channel's target (>= 0.5 -> 1) at rate
// 1/engage_time_s; kinetic capacity = e*capacity_nm, static =
// static_factor*kinetic. `speed_channel` (>= 770 selects "1000", else
// "540") only takes effect while e == 0 AND |omega_out| < 1 rad/s, else the
// current mode is held (recomputed every substep, not hashed). Optional
// `efficiency` in (0,1] (default 1): a lagged loss torque at `in`,
// -(1-efficiency)*|lambda_row_prev|*sign(omega_in_old), booked into
// gear_loss. While e == 0 EXACTLY, an implicit viscous drag at `out`
// (c = J_out_node/disengaged_brake_time_s, J_out_node fixed at build time)
// so a disengaged/unconnected `out` decays instead of spinning forever -
// booked into pto_clutch_loss (as is the row's own clutch-slip loss). At
// most one pto per vehicle.
struct PtoDesc {
    bool has_540 = false;
    real ratio_540 = 3.61;  // > 0, required when has_540
    bool has_1000 = false;
    real ratio_1000 = 1.95; // > 0, required when has_1000
    real capacity_nm = 0.0;                  // KINETIC capacity at engagement 1, > 0
    real static_factor = 1.2;                // static = static_factor*kinetic, >= 1
    real engage_time_s = 1.5;                // >= 0; 0 = instant engagement
    real efficiency = 1.0;                   // (0, 1]
    real out_inertia_kgm2 = 0.0;             // >= 0
    real disengaged_brake_time_s = 1.0;      // > 0
    std::string channel = "pto";             // declared (default 0) via Powertrain::declare_control_channels; non-empty
    std::string speed_channel = "pto_speed"; // declared (default 540) via same; non-empty
    Vec3 axis_local = Vec3::zero();          // shared by `in`/`out` ports (one physical shaft), plan section 6.7
};

// PHYS-033 P4 I4 C11: a test/bench load on a single port `shaft` - no
// shipped vehicle uses this (test rigs add one in code), but the loader
// supports it. `mode` selects the load law: Brake (an exact-Coulomb
// friction row to ground, kinetic capacity = torque_nm*scale, static =
// kinetic, holds at rest), Viscous (implicit tau0 = -c*omega_old, d = c,
// c = viscous_nm_s_rad*scale) or Quadratic (implicit tau0 =
// -k*omega_old*|omega_old|, d = 2*k*|omega_old|, linearised about
// omega_old, k = quadratic_nm_s2_rad2*scale). `scale_channel`, if
// non-empty, is declared (default 1.0) via Powertrain::declare_control_
// channels and multiplies the mode's own coefficient every substep; empty
// means a fixed scale of 1. Stateless (no fold_state_hash override).
enum class LoadDynoMode { Brake, Viscous, Quadratic };
struct LoadDynoDesc {
    LoadDynoMode mode = LoadDynoMode::Brake;
    real inertia_kgm2 = 0.0;            // >= 0
    real torque_nm = 0.0;               // required > 0 for Brake
    real viscous_nm_s_rad = 0.0;        // required > 0 for Viscous
    real quadratic_nm_s2_rad2 = 0.0;    // required > 0 for Quadratic
    std::string scale_channel;          // empty = fixed scale 1.0
    Vec3 axis_local = Vec3::zero();
};

// PHYS-033 P4 I7a C3 (plan section 3.4): the throttle-command stage between
// the pedal and an engine - a pure control-signal seam (no shaft-network
// port, no node, no rows; Powertrain constructs it as a TYPED member,
// throttle_stage.h's ThrottleStage, not a generic IComponent). At most one
// per vehicle; `engine` must name the vehicle's one engine component. When
// no vehicle declares one, Powertrain builds an IMPLICIT default stage
// instead: kind = the named engine's own throttle_input, no pedal_map/
// linkage_curve, and a hand lever exactly when that engine has a governor
// (then using the governor's own channel) - see throttle_stage.h and
// docs/data_formats/vehicle.md's throttle_control section for the full
// arithmetic and the loader's rejection list (which TCU interventions a
// mechanical linkage cannot carry, and why).
struct ThrottleControlDesc {
    std::string engine;     // required; must name a TorqueMapEngineDesc component in the same PowertrainDesc
    ThrottleInput kind = ThrottleInput::Electronic;
    bool has_curve = false; // true iff pedal_map (electronic) / linkage_curve (mechanical) was given
    vehicle::Curve1D curve; // pedal/lever position -> effective position; only meaningful when has_curve
    std::string hand_lever_channel; // optional; empty = no hand lever for this stage
};

// PHYS-033 P4 I7a C6 (plan section 2.4): a fuel tank - the first lumped
// ENERGY STORE component (PHYS-033 owner ruling "Tanks and stores": a fill
// state only, no sloshing/temperature; a pressure is not needed for a liquid
// fuel feeding a pumped injection system and is not modelled). No shaft
// port, no node, no rows: it feeds an engine's fuel input through a
// PowertrainDesc::fluid link ("<tank>.out" -> "<engine>.fuel"). `fuel` is
// the tank's content (loaded from a physics_sim.fuel/1 file resolved
// relative to the VEHICLE file's directory); it must be one of the fed
// engine's `fuels` (the loader compares file paths with
// std::filesystem::equivalent) and becomes the fuel that engine burns (see
// docs/data_formats/vehicle.md). initial_kg in [0, capacity_kg],
// capacity_kg > 0, both finite. In I7a the fuel mass does not change the
// chassis mass; a later increment adds it as a point mass at the tank
// position (PLAN.md "Fuel mass moves the chassis").
struct FuelTankDesc {
    FuelDesc fuel;
    real capacity_kg = 0.0;
    real initial_kg = 0.0;
};

// Closed, visited exhaustively (like ps::ShapeDesc) - adding a component
// type is a compile-checked change plus a data-format minor bump.
// SimulatedEngineDesc (PHYS-033 P4 I7b C10) is appended LAST so no existing
// alternative's index moves.
using ComponentParams = std::variant<TorqueMapEngineDesc, ClutchDesc, GearboxDesc, GearPairDesc, DifferentialDesc, TransferCaseDesc,
                                      ParkLockDesc, PtoDesc, LoadDynoDesc, ThrottleControlDesc, FuelTankDesc,
                                      SimulatedEngineDesc>;

struct ComponentDesc {
    std::string id; // unique, non-empty
    ComponentParams params;
};

// PHYS-033 P4 I4 C13: Rigid - every shaft merges its ports into one node;
// Compliant - shafts with stiffness > 0 become two-node spring-dampers (see
// ShaftDesc), every other shaft stays rigid.
enum class Compliance { Rigid, Compliant };

// PHYS-033 I3 (plan section 4.1): the gear_request channel's "no request"
// sentinel. Public because the adapter/scenario layer needs it too - out of
// forward AND reverse gear range for any gearbox this project ships (car:
// -1..6, tractor: -4..12).
inline constexpr real kGearRequestNone = real(-100.0);

// PHYS-033 P4 I7a C6: one fluid (fuel) connection, "<tank>.out" ->
// "<engine>.fuel" - a supply path, not a shaft: it carries mass, never
// torque. A separate namespace from ShaftDesc's ports.
struct FluidLinkDesc {
    PortRef from; // a fuel_tank component's "out"
    PortRef to;   // an engine component's "fuel"
};

struct PowertrainDesc {
    std::vector<ComponentDesc> components; // ARRAY order == evaluation order (PLAN.md D7)
    std::vector<ShaftDesc> shafts;
    std::vector<FluidLinkDesc> fluid; // PHYS-033 P4 I7a C6: optional, empty for every vehicle without a tank
    Compliance compliance = Compliance::Rigid;
    bool rotor_reaction = true; // plan section 6.7 (Q11); false only for a temporary data migration step
};

} // namespace ps::drivetrain
