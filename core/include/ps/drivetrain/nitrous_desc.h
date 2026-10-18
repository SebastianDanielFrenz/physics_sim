// ps/drivetrain/nitrous_desc.h - public, additive: the nitrous-oxide injection
// hardware descriptors (N2O plan N1). Pure data, no algorithm code. Three
// slot-in part files (docs/data_formats/nitrous.md) resolve into these:
//   physics_sim.n2o_bottle/1          -> N2oBottleDesc
//   physics_sim.nitrous_kit/1         -> NitrousKitDesc
//   physics_sim.nitrous_controller/1  -> NitrousControllerDesc (embedded in the kit)
// plus the oxidizer fluid file (oxidizer_desc.h). They appear in a vehicle as
// two ComponentParams alternatives (appended last) linked by fluid links
// "<bottle>.out" -> "<kit>.n2o" and "<kit>.out" -> "<engine>.nitrous"
// (powertrain_desc.h). The runtime (NitrousSystem) is internal:
// core/src/drivetrain/nitrous_system.h.
//
// Overrides are bottom-up: a part file's default, then the vehicle component's
// own optional fields (bottle initial_kg/pressure, kit controller), then (a
// later increment) the runtime user.
#pragma once

#include "ps/drivetrain/oxidizer_desc.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <string>

namespace ps::drivetrain {

// How the bottle's saturated state is chosen (no thermodynamic ODE - PLAN.md
// "Tanks": a fill state plus a data-given pressure).
enum class N2oPressureMode {
    Constant,    // gauge_bar
    VsFill,      // gauge bar as a Curve1D over the fill fraction 0..1
    Temperature, // bottle temperature in K, the pressure is the saturation pressure at it
};

struct N2oPressureDesc {
    N2oPressureMode mode = N2oPressureMode::Constant;
    real gauge_bar = 0.0;           // Constant: gauge bar (ISA sea-level reference, 1.01325 bar)
    vehicle::Curve1D gauge_vs_fill; // VsFill: x = fill fraction (ascending, spanning 0..1), y = gauge bar
    real temperature_k = 0.0;       // Temperature
};

// Optional bottle heater: while its channel is on (>= 0.5) the pressure is
// replaced by the set point, reached instantly (owner ruling 2026-10-04, no ODE).
struct N2oHeaterDesc {
    std::string channel;
    real default_on = 0.0;      // channel default value (0 = off)
    bool set_point_is_temperature = false;
    real set_point_gauge_bar = 0.0; // when !set_point_is_temperature
    real set_point_k = 0.0;         // when set_point_is_temperature
};

struct N2oBottleDesc {
    OxidizerDesc oxidizer;      // the fluid (loaded from the file's "oxidizer" path)
    real capacity_kg = 0.0;     // > 0
    real initial_kg = 0.0;      // [0, capacity]; the file's own value is the capacity, a vehicle component may lower it
    N2oPressureDesc pressure;   // the file's pressure, or the vehicle component's override
    bool has_heater = false;
    N2oHeaterDesc heater;
    std::string source;         // provenance (part file)
};

enum class NitrousKitType { Wet }; // "dry" is rejected until the dry increment
enum class NitrousInjection { Plenum, DirectPort }; // recorded for sound/docs; physically identical at mean-value level
enum class NitrousFlowModel { Hem, Dyer };
enum class NitrousFuelReference { Atmosphere, Manifold };

// Kit rating: metadata for the graph-engine add-on and the validation report.
// Never feeds the simulated engine.
struct NitrousRatingDesc {
    real power_kw = 0.0;
    real bottle_gauge_bar = 0.0;
    std::string basis;   // "crank" or "wheel"
    std::string source;  // required
};

struct NitrousProgressiveDesc {
    real start_fraction = 1.0; // flow fraction while t_active < delay_s
    real delay_s = 0.0;
    real ramp_s = 0.0;         // 0: jump to 1 after the delay
    // A closure shorter than this keeps the ramp where it was (the time counter is held, not reset), so a one-tick
    // window/limiter/traction interruption does not restart the ramp at start_fraction (owner report, n2o-oscillation:
    // a single-tick close at the rpm window edge restarted the ramp, and the torque dip made a ~1 Hz rev cycle). 0 = the
    // old behaviour (every closure restarts the ramp). A shift cut is longer than the default, so it still restarts it.
    real resume_within_s = 0.0;
};

struct NitrousSafetyDesc {
    real min_bottle_gauge_bar = 0.0; // 0 = no bottle-pressure switch
    bool cut_on_fuel_starve = false;
    real max_active_s = 0.0;         // 0 = unlimited
};

struct NitrousPurgeDesc {
    std::string channel;
    real jet_diameter_mm = 0.0;
    real cd = 0.0;
};

// The "ECU strategy" part: when does the solenoid open, how fast does the flow
// ramp, how much does the kit retard the spark.
struct NitrousControllerDesc {
    std::string arm_channel;
    real arm_default = 0.0;
    real min_throttle_command = 0.95; // on the throttle stage's command
    real min_rpm = 0.0;
    real max_rpm = 1.0e9;
    // Hysteresis on the throttle gate: once the solenoid is open it stays open until the command falls below
    // min_throttle_command - throttle_hysteresis (an engine command hovering at the switch, e.g. a hybrid supervisor's
    // power share, otherwise chatters the solenoid). 0 = the old single threshold.
    real throttle_hysteresis = 0.0;
    // The flow fades linearly to zero over the last max_rpm_fade_rpm below max_rpm instead of closing at the edge: a
    // hard close there is a bang-bang limiter (the flow steps 1 -> 0 -> restarted ramp), a taper has negative rpm
    // feedback and settles. 0 = the old hard edge.
    real max_rpm_fade_rpm = 0.0;
    int min_gear = 1;                 // >= 1 (the loader rejects less): neutral (0) and reverse never spray; the system also enforces it
    int max_gear = 99;
    real min_speed_mps = 0.0;
    NitrousProgressiveDesc progressive;
    // Spark retard (deg): a constant, or a curve over the progressive flow fraction.
    bool retard_is_curve = false;
    real spark_retard_deg = 0.0;
    vehicle::Curve1D retard_vs_fraction; // x = flow fraction ascending, y = deg
    NitrousSafetyDesc safety;
    bool has_purge = false;
    NitrousPurgeDesc purge;
    std::string source;
};

struct NitrousKitDesc {
    NitrousKitType type = NitrousKitType::Wet;
    NitrousInjection injection = NitrousInjection::Plenum;
    int nozzle_count = 1; // direct_port: one jet pair per nozzle; flows multiply
    real nitrous_jet_diameter_mm = 0.0;
    real nitrous_jet_cd = 0.0; // (0, 1]
    NitrousFlowModel flow_model = NitrousFlowModel::Hem;
    real fuel_jet_diameter_mm = 0.0;
    real fuel_jet_cd = 0.0;
    real fuel_pressure_gauge_bar = 0.0; // >= 0
    NitrousFuelReference fuel_reference = NitrousFuelReference::Atmosphere;
    NitrousRatingDesc rating;
    NitrousControllerDesc controller; // the kit's default_controller, or the vehicle component's override
    std::string source;
};

} // namespace ps::drivetrain
