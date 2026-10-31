// ps/drivetrain/limit_findings.h - engine garage P3b-1: structured "limit findings" the dyno sweep measures, for the
// garage's limit advisor (racing_game rg::LimitAdvisor, out/engine_garage_design.md section 18).
//
// A finding says WHAT limits (kind), HOW BAD (severity), WHERE (rpm; NaN = static), with the measured value and the limit
// it is held against, and which PART to change (PartRef). Where the simulation can compute it, a finding carries a
// quantified Remedy ("increase X by n x"). The advice TEXT is game data keyed by LimitKind; every number comes from here.
//
// Units are SI (Pa, K, N*m, m/s, kg/s, m^2) with ONE exception that follows the project's own convention everywhere
// else: rotational speed is rpm (LimitUnit::RevPerMinute). Pressures are gauge where a finding says so (boost).
//
// Shipping increments: P3b-1 = WastegateSaturated, BoostBelowTarget, BoostAboveTarget, TurboRotorCap, CompressorSurge,
// CompressorChoke, ClutchCapacity, RevLimiter, FuelCut, MeanPistonSpeed, MapGridEdge; P3b-2 = KitFuelLean, N2OBottleDuration,
// N2OBottlePressureDrop (a sweep with DynoSweepSpec::nitrous) and the Simulated remedies (ps/drivetrain/limit_remedies.h);
// T0 adds GearboxInputTorque (check_driveline_limits against GearboxDesc::rated_input_torque_nm); G1 adds BeyondEngineData;
// not produced (value reserved, append-only): ChargeTemperature (no limit exists in any data file), InjectorDuty (no injector model exists: fuel is a mass flow), ValveFlowChoke
// (no valve Mach diagnostic exists), Knock (C17b).
// docs/data_formats/engine.md "Dyno sweep" -> "Limit findings" has the thresholds and sources.
#pragma once

#include "ps/types.h"

#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace ps::drivetrain {

// Stable, append-only values (they may be stored by a game or a save file): never renumber, never reuse.
enum class LimitKind : std::uint16_t {
    WastegateSaturated = 0,    // P3b-1: the wastegate has no authority left (fully open)
    BoostBelowTarget = 1,      // P3b-1
    BoostAboveTarget = 2,      // P3b-1
    TurboRotorCap = 3,         // P3b-1: rotor at the end of the compressor head table's speed axis
    CompressorSurge = 4,       // P3b-1: only with a surge line in the turbo hardware file
    CompressorChoke = 5,       // P3b-1: flow near the compressor's flow capacity (max_mass_flow_kg_s)
    ChargeTemperature = 6,     // not produced: no charge-temperature limit exists in any data file
    ClutchCapacity = 7,        // P3b-1: check_driveline_limits
    GearboxInputTorque = 8,    // transmission plan T0: check_driveline_limits, a gearbox rated_input_torque_nm below the dyno peak
    KitFuelLean = 9,           // P3b-2: the combined lambda with the kit is too lean (Closed remedy: fuel jet)
    InjectorDuty = 10,         // not produced: no injector model exists
    N2OBottleDuration = 11,    // P3b-2: the bottle empties within a drag-strip pass (Closed remedy: bottle content)
    N2OBottlePressureDrop = 12, // P3b-2: a bottle whose pressure falls with the fill loses N2O flow during the pass
    RevLimiter = 13,           // P3b-1: a SoftCut limiter taper is reached
    FuelCut = 14,              // P3b-1: a FuelCut limiter is reached
    ValveFlowChoke = 15,       // not produced: no valve Mach diagnostic exists
    MeanPistonSpeed = 16,      // P3b-1
    MapGridEdge = 17,          // P3b-1: a map lookup is clamped at the edge of its grid
    Knock = 18,                // engine garage P7: the knock limit holds the applied spark below the ECU schedule
    BeyondEngineData = 19,     // G1: the point lies above the engine's own data, where an armed roll-off cuts the fired torque
                               // (ps/drivetrain/limiter_range.h): Limiting inside the band, Exceeded at/after its end
    TowedMass = 20,            // coupling A1: more behind a towing point than it is rated for (ps/machine/hitch_limits.h)
    TongueLoad = 21,           // coupling A1: more down on a towing point than it is rated for
    LiftCapacity = 22,         // linkage A3: the lift at or past its force (ps/machine/hitch_limits.h)
    PtoShaftTorque = 23,       // tools M1: a tool's PTO torque against the tractor's PTO clutch (ps/machine/tool.h)
    ToolPower = 24,            // tools M1: a tool's power demand against the tractor's engine (ps/machine/tool.h)
};

// Near: within a margin of the limit (information); Limiting: this part caps the output here; Exceeded: the target is
// missed or a guard rail is hit (the result beyond it is not trustworthy). Ordered: a larger value is worse.
enum class LimitSeverity : std::uint8_t { Near = 0, Limiting = 1, Exceeded = 2 };

// A short enum rather than a string: no allocation, a game switches on it (display text and locale are game data), and a
// typo cannot compile. to_string gives the stable symbol.
enum class LimitUnit : std::uint8_t {
    Dimensionless = 0, // ratios and counts
    Fraction = 1,      // 0..1 (a gate opening, a share of a window)
    Pascal = 2,
    Kelvin = 3,
    NewtonMetre = 4,
    MetrePerSecond = 5,
    RevPerMinute = 6,  // the one non-SI unit, like every rpm in this repo
    SquareMetre = 7,
    Millimetre = 8,    // P3b-2: a jet diameter in the part file's own unit
    Kilogram = 9,      // P3b-2: a bottle content
    Second = 10,       // P3b-2: a duration
    CrankDegree = 11,  // P7: a spark advance / retard, deg crank
    Newton = 12,       // coupling A1: a hitch load
    Kilowatt = 13,     // machine M1: what a tool needs of an engine
};

// The part to change. slot: the part's role in physics_sim's vocabulary ("engine", "turbo", "forced_induction",
// "clutch", "gearbox"); the game maps its own slot names. file: the part file the descriptor came from. A descriptor does
// not remember its file, so run_dyno_sweep fills it from DynoSweepSpec::part_files (empty when the caller gave none) and
// check_driveline_limits takes it from its `clutch_file` argument. field: an RFC 6901 JSON pointer into that file
// (e.g. "/wastegate_area_m2"); empty = the part as a whole.
struct PartRef {
    std::string slot;
    std::string file;
    std::string field;
};

enum class RemedyCost : std::uint8_t {
    Closed = 0,    // a closed-form number computed with every sweep
    Simulated = 1, // needs a search over dyno runs (P3b-2: solve_remedy / solve_power_target, limit_remedies.h)
};

// "Change `part` from `current` to at least `required_min` (`required_margin` leaves the table's safety margin)".
// All three are in `unit`, in the part field's own terms. Ratios are derived, not stored.
struct Remedy {
    PartRef part;
    real current = std::numeric_limits<real>::quiet_NaN();
    real required_min = std::numeric_limits<real>::quiet_NaN();
    real required_margin = std::numeric_limits<real>::quiet_NaN();
    LimitUnit unit = LimitUnit::Dimensionless;
    RemedyCost cost = RemedyCost::Closed;
    // Simulated remedies: false on the placeholder a sweep attaches (current filled, the required values NaN) and on a solve
    // that found no value; true after a successful solve_remedy (ps/drivetrain/limit_remedies.h). A Closed remedy is always solved.
    bool solved = true;

    [[nodiscard]] real ratio_min() const { return required_min / current; }
    [[nodiscard]] real ratio_margin() const { return required_margin / current; }
};

struct LimitFinding {
    LimitKind kind = LimitKind::WastegateSaturated;
    LimitSeverity severity = LimitSeverity::Near;
    real rpm = std::numeric_limits<real>::quiet_NaN(); // NaN = static (not tied to a speed)
    real value = 0.0;                                   // what was measured
    real limit = 0.0;                                   // what it is held against
    LimitUnit unit = LimitUnit::Dimensionless;          // of value and limit
    PartRef part;
    std::optional<Remedy> remedy;
};

// Stable symbols for logs and tests ("WastegateSaturated", "Near", "N*m", ...). An unknown value returns "Unknown".
[[nodiscard]] const char* to_string(LimitKind k);
[[nodiscard]] const char* to_string(LimitSeverity s);
[[nodiscard]] const char* to_string(LimitUnit u);

// The index range [first, first + count) of one dyno point's findings in DynoResult::limits.
struct LimitRange {
    std::uint32_t first = 0;
    std::uint32_t count = 0;
};

} // namespace ps::drivetrain
