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

#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h" // Curve1D (NOT vehicle_desc.h: VehicleDesc::powertrain
                                 // is a PowertrainDesc, so that include would be circular)

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
};

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

// Closed, visited exhaustively (like ps::ShapeDesc) - adding a component
// type is a compile-checked change plus a data-format minor bump.
using ComponentParams = std::variant<TorqueMapEngineDesc, ClutchDesc, GearboxDesc, GearPairDesc, DifferentialDesc, TransferCaseDesc,
                                      ParkLockDesc, PtoDesc, LoadDynoDesc, ThrottleControlDesc>;

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

struct PowertrainDesc {
    std::vector<ComponentDesc> components; // ARRAY order == evaluation order (PLAN.md D7)
    std::vector<ShaftDesc> shafts;
    Compliance compliance = Compliance::Rigid;
    bool rotor_reaction = true; // plan section 6.7 (Q11); false only for a temporary data migration step
};

} // namespace ps::drivetrain
