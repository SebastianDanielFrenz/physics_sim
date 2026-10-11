// core/src/drivetrain/limit_thresholds.h - engine garage P3b-1: the ONE table of every threshold the limit findings use.
// INTERNAL (the docs list the values in docs/data_formats/engine.md "Limit findings"; a game does not tune them).
//
// Every entry carries a source tag: [C] cited, [D] derived from data/code in this repo, [AS] assumed (no source was
// available locally; the reasoning is in the entry), [M] measured on a shipped engine. Nothing here is a fitted constant of
// the physics - they only decide when a measurement is worth telling the player about.
//
// kAllLimitThresholds lists every entry so a test can assert that none is untagged and none is unused.
#pragma once

#include "ps/types.h"

#include <cstddef>

namespace ps::drivetrain::limits {

struct Threshold {
    const char* name;
    real value;
    const char* source; // starts with "[C]", "[D]", "[AS]" or "[M]"
};

// --- Wastegate / boost (turbo points; window = the settled window of one point) ------------------------------------------
// Gate opening at which the actuator has no authority left. The N7 procedure used "gate opening <= 0.92" as the design
// margin of a wastegate sizing (engine_garage_design.md section 18), so a mean opening above it is "near".
inline constexpr Threshold kGateNear{"gate_near_opening", 0.92, "[AS] N7 wastegate-sizing margin (design section 18): opening <= 0.92"};
// Fully open for practical purposes: the PID output clamps at 1 and the actuator lags it with tau 0.1 s, so 0.98 is reached
// only when the command sits at 1 for several tau.
inline constexpr Threshold kGateFull{"gate_full_opening", 0.98, "[AS] 2 % below the actuator's asymptote"};
// Share of the window the gate must sit at kGateFull to call it saturated (not a transient peak).
inline constexpr Threshold kGateSaturatedFraction{"gate_saturated_window_fraction", 0.5, "[AS] half of the settled window"};
// Boost band, as a fraction of the configuration's target_boost_pa: mean charge pressure above target by more than
// kBoostBandExceeded is a target miss; above by kBoostBandNear is information. The wastegate PID holds < 20 Pa ripple on the
// hyper (CLAUDE.md "wastegate PID", vault PHYS-058), so 2 % of 1.8 bar (3.6 kPa) is far outside the controller's own noise.
inline constexpr Threshold kBoostBandNear{"boost_band_near", 0.02, "[AS] >> the 20 Pa ripple measured on the hyper (PHYS-058)"};
inline constexpr Threshold kBoostBandExceeded{"boost_band_exceeded", 0.05, "[AS] 5 % of the target boost"};

// --- Turbo rotor ----------------------------------------------------------------------------------------------------------
// Fractions of the speed at the end of the compressor head table (TurboPair: head_pa.x.back() * reference_rpm, 1.5 *
// reference_rpm without maps). Beyond it the head is extended by affinity scaling, not looked up.
inline constexpr Threshold kRotorNear{"rotor_near_fraction", 0.90, "[AS] 10 % below the table edge"};
inline constexpr Threshold kRotorLimiting{"rotor_limiting_fraction", 0.97, "[AS] 3 % below the table edge: within the turbine's reach in one more step of load"};

// --- Compressor -----------------------------------------------------------------------------------------------------------
// Surge: flow fraction over the surge line's flow at the same pressure ratio; the engine's own flag uses 1.0 (entry) and 1.05
// (exit hysteresis), so 1.10 is a margin of twice the hysteresis.
inline constexpr Threshold kSurgeNearMargin{"surge_near_flow_ratio", 1.10, "[AS] twice TurboPair's 1.05 exit hysteresis"};
// A window needs this share of surge substeps before it is reported (one substep of noise is not a surge).
inline constexpr Threshold kSurgeMinFraction{"surge_min_window_fraction", 0.02, "[AS] 2 % of the window (~10 substeps at 960 Hz x 0.5 s)"};
// Choke/flow capacity: pair flow over max_mass_flow_kg_s (the corrected-flow normalisation of the head table; head -> 0
// at 1.0 on the authored law). Without per-speed choke data this is a capacity margin, not a map-edge choke.
inline constexpr Threshold kChokeNear{"choke_near_flow_fraction", 0.90, "[AS] 10 % below the flow capacity"};
inline constexpr Threshold kChokeLimiting{"choke_limiting_flow_fraction", 1.0, "[D] the head table's normalised flow = 1 (turbo_pair.h)"};

// --- Rev limiter / fuel cut -----------------------------------------------------------------------------------------------
// FuelCut limiter: points within this fraction of the limiter rpm are "near" (the latch cuts at the limiter itself).
inline constexpr Threshold kLimiterNear{"limiter_near_fraction", 0.97, "[AS] 3 % below the cut rpm"};
// SoftCut limiter: near starts this many taper bands below the zero-fuel rpm.
inline constexpr Threshold kSoftCutNearBands{"soft_cut_near_bands", 2.0, "[AS] two bands below the zero-fuel rpm"};

// --- Mean piston speed [m/s] (stroke * rpm / 30) --------------------------------------------------------------------------
// No local source states a limit (Heywood 1988 discusses mean piston speed but no scan with a table is available here),
// so these are [AS]: production road engines run roughly 15-23 m/s at the redline (the shipped Toyota A25A-FKS: 22.4 m/s at
// its 6500 rpm limiter [D] from its 103.4 mm stroke), Formula 1's 2.4 L V8 era peaked near 25 m/s with an engine life of a few
// hundred km (recalled, not checked against a local source). Near 23 (just above the shipped road engine), Limiting 25
// (the race-engine ceiling above), Exceeded 28 (12 % beyond it - ring and rod loads rise with the square of the speed).
inline constexpr Threshold kPistonSpeedNear{"piston_speed_near_mps", 23.0, "[AS] just above the A25A's 22.4 m/s [D]"};
inline constexpr Threshold kPistonSpeedLimiting{"piston_speed_limiting_mps", 25.0, "[AS] F1 2.4 L V8 era peak ~25 m/s, recalled"};
inline constexpr Threshold kPistonSpeedExceeded{"piston_speed_exceeded_mps", 28.0, "[AS] 12 % above the race-engine ceiling"};

// --- Map grid edge --------------------------------------------------------------------------------------------------------
// Share of the window a lookup must be clamped before it is reported (a grid edge is crossed briefly by a transient).
inline constexpr Threshold kEdgeMinFraction{"grid_edge_min_window_fraction", 0.05, "[AS] 5 % of the window"};
// "Near" an UPPER edge: within this fraction of the axis span (lower edges are the normal idle/closed-throttle region).
inline constexpr Threshold kEdgeNearFraction{"grid_edge_near_span_fraction", 0.02, "[AS] 2 % of the axis span"};

// --- Driveline ------------------------------------------------------------------------------------------------------------
// The clutch's recommended capacity is this factor over the engine's peak torque (the Closed remedy's required_margin).
// Near starts when peak torque exceeds 1/factor of the static capacity.
inline constexpr Threshold kDrivelineMargin{"driveline_torque_margin", 1.3, "[AS] design section 18 (owner-visible 'x1.3')"};

inline constexpr Threshold kAllLimitThresholds[] = {
    kGateNear,        kGateFull,        kGateSaturatedFraction, kBoostBandNear,      kBoostBandExceeded, kRotorNear,
    kRotorLimiting,   kSurgeNearMargin, kSurgeMinFraction,      kChokeNear,          kChokeLimiting,     kLimiterNear,
    kSoftCutNearBands, kPistonSpeedNear, kPistonSpeedLimiting,   kPistonSpeedExceeded, kEdgeMinFraction,   kEdgeNearFraction,
    kDrivelineMargin,
};
inline constexpr std::size_t kAllLimitThresholdCount = sizeof(kAllLimitThresholds) / sizeof(kAllLimitThresholds[0]);

} // namespace ps::drivetrain::limits
