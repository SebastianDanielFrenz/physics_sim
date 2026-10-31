// core/src/drivetrain/limit_policy_table.h - increment L0: the ONE table that ties a LimitPolicy field (ps/drivetrain/
// limit_policy.h) to its compiled default (limit_thresholds.h / engine_rules_table.h) and its domain. INTERNAL.
//
// Used by: LimitPolicy's constructor (defaults), validate_limit_policy (domains, orderings), the policy-file loader
// (group / name / source text) and the tests (default.json == compiled defaults entry by entry; every entry reachable).
// Adding a policy entry = one Threshold in a table header, one field in LimitPolicy, one row here (a static_assert in
// limit_policy.cpp fails when the field count and the row count disagree).
//
// Domain columns: the value must be > 0, >= `lo` and <= `hi` (kNoMax = no upper bound).
#pragma once

#include "engine_rules_table.h"
#include "limit_thresholds.h"
#include "ps/drivetrain/limit_policy.h"

#include <cstddef>
#include <limits>

namespace ps::drivetrain::policy_table {

namespace limits = ps::drivetrain::limits;
namespace rules = ps::drivetrain::rules;

inline constexpr real kNoMax = std::numeric_limits<real>::infinity();

struct Row {
    const char* group;  // top-level object of the policy file
    const char* name;   // key inside the group == the LimitPolicy field's name
    real LimitPolicy::*member;
    real def;           // compiled default
    const char* source; // compiled default's source text ("[C]"/"[D]"/"[AS]"/"[M]" ...)
    real lo;            // inclusive lower bound (0: only > 0 is required)
    real hi;            // inclusive upper bound
};

template <class T>
constexpr Row row(const char* group, const T& t, real LimitPolicy::*m, real lo, real hi) {
    return Row{group, t.name, m, t.value, t.source, lo, hi};
}

inline constexpr Row kRows[] = {
    row("turbo", limits::kGateNear, &LimitPolicy::gate_near_opening, 0.0, 1.0),
    row("turbo", limits::kGateFull, &LimitPolicy::gate_full_opening, 0.0, 1.0),
    row("turbo", limits::kGateSaturatedFraction, &LimitPolicy::gate_saturated_window_fraction, 0.0, 1.0),
    row("turbo", limits::kBoostBandNear, &LimitPolicy::boost_band_near, 0.0, 1.0),
    row("turbo", limits::kBoostBandExceeded, &LimitPolicy::boost_band_exceeded, 0.0, 1.0),
    row("turbo", limits::kRotorNear, &LimitPolicy::rotor_near_fraction, 0.0, 1.0),
    row("turbo", limits::kRotorLimiting, &LimitPolicy::rotor_limiting_fraction, 0.0, 1.0),
    row("turbo", limits::kSurgeNearMargin, &LimitPolicy::surge_near_flow_ratio, 1.0, kNoMax),
    row("turbo", limits::kSurgeMinFraction, &LimitPolicy::surge_min_window_fraction, 0.0, 1.0),
    row("turbo", limits::kChokeNear, &LimitPolicy::choke_near_flow_fraction, 0.0, 1.0),
    row("turbo", limits::kChokeLimiting, &LimitPolicy::choke_limiting_flow_fraction, 0.0, kNoMax),
    row("turbo", limits::kWastegateRemedyMargin, &LimitPolicy::wastegate_remedy_margin, 1.0, kNoMax),
    row("limiter", limits::kLimiterNear, &LimitPolicy::limiter_near_fraction, 0.0, 1.0),
    row("limiter", limits::kSoftCutNearBands, &LimitPolicy::soft_cut_near_bands, 1.0, kNoMax),
    row("piston_speed", limits::kPistonSpeedNear, &LimitPolicy::piston_speed_near_mps, 0.0, kNoMax),
    row("piston_speed", limits::kPistonSpeedLimiting, &LimitPolicy::piston_speed_limiting_mps, 0.0, kNoMax),
    row("piston_speed", limits::kPistonSpeedExceeded, &LimitPolicy::piston_speed_exceeded_mps, 0.0, kNoMax),
    row("map_grid", limits::kEdgeMinFraction, &LimitPolicy::grid_edge_min_window_fraction, 0.0, 1.0),
    row("map_grid", limits::kEdgeNearFraction, &LimitPolicy::grid_edge_near_span_fraction, 0.0, 1.0),
    row("driveline", limits::kDrivelineMargin, &LimitPolicy::driveline_torque_margin, 1.0, kNoMax),
    row("nitrous", limits::kKitLambdaNear, &LimitPolicy::kit_lambda_near, 0.0, kNoMax),
    row("nitrous", limits::kKitLambdaLimiting, &LimitPolicy::kit_lambda_limiting, 0.0, kNoMax),
    row("nitrous", limits::kKitLambdaExceeded, &LimitPolicy::kit_lambda_exceeded, 0.0, kNoMax),
    row("nitrous", limits::kKitLambdaTarget, &LimitPolicy::kit_lambda_target, 0.0, kNoMax),
    row("nitrous", limits::kBottleDurationTarget, &LimitPolicy::bottle_duration_target_s, 0.0, kNoMax),
    row("nitrous", limits::kBottleDurationNearFactor, &LimitPolicy::bottle_duration_near_factor, 1.0, kNoMax),
    row("nitrous", limits::kBottleReserve, &LimitPolicy::bottle_reserve_factor, 1.0, kNoMax),
    row("nitrous", limits::kBottleFlowRetentionNear, &LimitPolicy::bottle_flow_retention_near, 0.0, 1.0),
    row("nitrous", limits::kBottleFlowRetentionLimiting, &LimitPolicy::bottle_flow_retention_limiting, 0.0, 1.0),
    row("knock", limits::kKnockNear, &LimitPolicy::knock_near_retard_deg, 0.0, kNoMax),
    row("knock", limits::kKnockLimiting, &LimitPolicy::knock_limiting_retard_deg, 0.0, kNoMax),
    row("valves", rules::kIntake1Min, &LimitPolicy::intake_d_over_bore_1v_min, 0.0, 1.0),
    row("valves", rules::kIntake1Max, &LimitPolicy::intake_d_over_bore_1v_max, 0.0, 1.0),
    row("valves", rules::kIntake2Min, &LimitPolicy::intake_d_over_bore_2v_min, 0.0, 1.0),
    row("valves", rules::kIntake2Max, &LimitPolicy::intake_d_over_bore_2v_max, 0.0, 1.0),
    row("valves", rules::kIntake3Min, &LimitPolicy::intake_d_over_bore_3v_min, 0.0, 1.0),
    row("valves", rules::kIntake3Max, &LimitPolicy::intake_d_over_bore_3v_max, 0.0, 1.0),
    row("valves", rules::kExhaustRatioMin, &LimitPolicy::exhaust_over_intake_min, 0.0, kNoMax),
    row("valves", rules::kExhaustRatioMax, &LimitPolicy::exhaust_over_intake_max, 0.0, kNoMax),
    row("valves", rules::kPackShare, &LimitPolicy::pack_share_of_ideal, 0.0, 1.0),
    row("rod", rules::kRodRatioMin, &LimitPolicy::rod_over_crank_radius_min, 0.0, kNoMax),
    row("rod", rules::kRodRatioMax, &LimitPolicy::rod_over_crank_radius_max, 0.0, kNoMax),
    row("compression", rules::kCrNaMin, &LimitPolicy::cr_na_min, 1.0, kNoMax),
    row("compression", rules::kCrNaMax, &LimitPolicy::cr_na_max, 1.0, kNoMax),
    row("compression", rules::kCrFiMin, &LimitPolicy::cr_forced_min, 1.0, kNoMax),
    row("compression", rules::kCrFiMax, &LimitPolicy::cr_forced_max, 1.0, kNoMax),
    row("grid_coverage", rules::kGridOvershootLimiting, &LimitPolicy::grid_overshoot_limiting, 0.0, kNoMax),
    row("grid_coverage", rules::kGridOvershootExceeded, &LimitPolicy::grid_overshoot_exceeded, 0.0, kNoMax),
    row("cam", rules::kCamSwitchFractionMin, &LimitPolicy::cam_switch_over_limiter_min, 0.0, 1.0),
    row("cam", rules::kCamSwitchFractionMax, &LimitPolicy::cam_switch_over_limiter_max, 0.0, 1.0),
    row("cam", rules::kCamHysteresisMin, &LimitPolicy::cam_switch_hysteresis_min_rpm, 0.0, kNoMax),
    row("cam", rules::kCamHysteresisMax, &LimitPolicy::cam_switch_hysteresis_max_rpm, 0.0, kNoMax),
    row("cam", rules::kCamLiftOverValveMax, &LimitPolicy::cam_peak_lift_over_valve_d_max, 0.0, kNoMax),
    row("lift", rules::kLiftMinPeakMinMm, &LimitPolicy::lift_min_node_peak_min_mm, 0.0, kNoMax),
    row("lift", rules::kLiftMinPeakMaxFraction, &LimitPolicy::lift_min_node_peak_max_fraction_of_full, 0.0, 1.0),
    row("gearbox", rules::kGearboxOverLimiting, &LimitPolicy::gearbox_rating_over_limiting, 0.0, kNoMax),
    row("gearbox", rules::kGearboxOverExceeded, &LimitPolicy::gearbox_rating_over_exceeded, 0.0, kNoMax),
    row("gearbox", rules::kGearboxRemedyMargin, &LimitPolicy::gearbox_rating_remedy_margin, 1.0, kNoMax),
    row("hitch", limits::kHitchNear, &LimitPolicy::hitch_near_share, 0.0, kNoMax),
    row("hitch", limits::kHitchLimiting, &LimitPolicy::hitch_limiting_share, 0.0, kNoMax),
    row("hitch", limits::kHitchExceeded, &LimitPolicy::hitch_exceeded_share, 0.0, kNoMax),
};
inline constexpr std::size_t kRowCount = sizeof(kRows) / sizeof(kRows[0]);

// Orderings every valid policy keeps: `lo` below `hi` (strictly, or <= when `strict` is false).
struct Order {
    real LimitPolicy::*lo;
    real LimitPolicy::*hi;
    bool strict;
};
inline constexpr Order kOrders[] = {
    {&LimitPolicy::gate_near_opening, &LimitPolicy::gate_full_opening, true},
    {&LimitPolicy::boost_band_near, &LimitPolicy::boost_band_exceeded, true},
    {&LimitPolicy::rotor_near_fraction, &LimitPolicy::rotor_limiting_fraction, true},
    {&LimitPolicy::choke_near_flow_fraction, &LimitPolicy::choke_limiting_flow_fraction, true},
    {&LimitPolicy::piston_speed_near_mps, &LimitPolicy::piston_speed_limiting_mps, true},
    {&LimitPolicy::piston_speed_limiting_mps, &LimitPolicy::piston_speed_exceeded_mps, true},
    {&LimitPolicy::kit_lambda_near, &LimitPolicy::kit_lambda_limiting, true},
    {&LimitPolicy::kit_lambda_limiting, &LimitPolicy::kit_lambda_exceeded, true},
    {&LimitPolicy::kit_lambda_target, &LimitPolicy::kit_lambda_near, false},
    {&LimitPolicy::bottle_flow_retention_limiting, &LimitPolicy::bottle_flow_retention_near, true},
    {&LimitPolicy::knock_near_retard_deg, &LimitPolicy::knock_limiting_retard_deg, true},
    {&LimitPolicy::intake_d_over_bore_1v_min, &LimitPolicy::intake_d_over_bore_1v_max, true},
    {&LimitPolicy::intake_d_over_bore_2v_min, &LimitPolicy::intake_d_over_bore_2v_max, true},
    {&LimitPolicy::intake_d_over_bore_3v_min, &LimitPolicy::intake_d_over_bore_3v_max, true},
    {&LimitPolicy::exhaust_over_intake_min, &LimitPolicy::exhaust_over_intake_max, true},
    {&LimitPolicy::rod_over_crank_radius_min, &LimitPolicy::rod_over_crank_radius_max, true},
    {&LimitPolicy::cr_na_min, &LimitPolicy::cr_na_max, true},
    {&LimitPolicy::cr_forced_min, &LimitPolicy::cr_forced_max, true},
    {&LimitPolicy::grid_overshoot_limiting, &LimitPolicy::grid_overshoot_exceeded, true},
    {&LimitPolicy::cam_switch_over_limiter_min, &LimitPolicy::cam_switch_over_limiter_max, true},
    {&LimitPolicy::cam_switch_hysteresis_min_rpm, &LimitPolicy::cam_switch_hysteresis_max_rpm, true},
    {&LimitPolicy::gearbox_rating_over_limiting, &LimitPolicy::gearbox_rating_over_exceeded, true},
    {&LimitPolicy::hitch_near_share, &LimitPolicy::hitch_limiting_share, true},
    {&LimitPolicy::hitch_limiting_share, &LimitPolicy::hitch_exceeded_share, true},
};
inline constexpr std::size_t kOrderCount = sizeof(kOrders) / sizeof(kOrders[0]);

} // namespace ps::drivetrain::policy_table
