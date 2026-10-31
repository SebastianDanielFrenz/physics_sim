// ps/drivetrain/limit_policy.h - increment L0 (owner 2026-10-13): refusal limits and advice thresholds are CONFIG, not code.
//
// A LimitPolicy holds every number that decides only what the player is TOLD or REFUSED: the Near/Limiting/Exceeded bands of
// the limit findings (limit_findings.h), the window fractions they are measured over, the remedy margins (x1.3 clutch, x1.10
// wastegate, ...), the buildability bands of check_buildable (valve size, rod ratio, compression ratio, piston speed, grid
// overshoot, cam/lift bands), the gearbox rating grades - plus the rule that turns a severity into a refusal (`refuse_at`)
// and into a warning (`warn_at`).
//
// What it does NOT hold: anything a map generator or a loader uses to BUILD something (packing densities, the row chord
// fraction, the valve-layout list, the boost/NA grid helpers' constants, comparison slack) - those stay constexpr in
// core/src/drivetrain/engine_rules_table.h. A policy value never enters engine_map_cache_key, the turbo table key, a content
// hash or a state hash (tests/unit/test_limit_policy.cpp guards this). The full classification table is in
// core/src/drivetrain/limit_thresholds.h / engine_rules_table.h (header comments) and docs/data_formats/limit_policy.md.
//
// Usage: every consumer takes an optional `const LimitPolicy*` (nullptr = default_limit_policy(), the compiled defaults, which
// reproduce every pre-L0 result bit for bit): DynoSweepSpec::policy (run_dyno_sweep, and through RemedyRequest::spec
// solve_remedy / solve_power_target), the trailing argument of check_driveline_limits, check_buildable, check_gearbox_rating
// and has_blocking_issue. The pointee must outlive the call. A policy is a plain value (copy it, change a field, pass its
// address); files load through ps::io::load_limit_policy_json (ps/io/limit_policy_io.h, physics_sim.limit_policy/1,
// docs/data_formats/limit_policy.md). A garage that wants to FILTER what it shows uses warns()/refuses(); the engine never
// drops a finding by itself, so the findings stay complete for any reader.
//
// Field names are the entries' stable `name` strings (the keys of the policy file). Doubles throughout, no strings: nothing
// here allocates on a hot path.
#pragma once

#include "ps/drivetrain/limit_findings.h"
#include "ps/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps::drivetrain {

struct BuildIssue; // engine_rules.h

// Severity from which the garage REFUSES (blocks save/drive/build). The default is the owner's rule: only Exceeded.
enum class RefuseAt : std::uint8_t { Exceeded = 0, Limiting = 1, Never = 2 };
// Severity from which a finding/issue is worth TELLING the player. The default shows everything.
enum class WarnAt : std::uint8_t { Near = 0, Limiting = 1, Exceeded = 2, Never = 3 };

[[nodiscard]] const char* to_string(RefuseAt r); // "exceeded", "limiting", "never" (the file's spelling)
[[nodiscard]] const char* to_string(WarnAt w);   // "near", "limiting", "exceeded", "never"

struct LimitPolicy {
    // The defaults: every field = the compiled default (limit_thresholds.h / engine_rules_table.h), refuse_at Exceeded,
    // warn_at Near. A default-constructed LimitPolicy equals default_limit_policy().
    LimitPolicy();

    RefuseAt refuse_at;
    WarnAt warn_at;

    // Severity >= warn_at / >= refuse_at.
    [[nodiscard]] bool warns(LimitSeverity s) const;
    [[nodiscard]] bool refuses(LimitSeverity s) const;

    // The numbers, grouped as the file groups them (each group is a top-level object of the policy file).
    // clang-format off
    // --- turbo ---
    real gate_near_opening;
    real gate_full_opening;
    real gate_saturated_window_fraction;
    real boost_band_near;
    real boost_band_exceeded;
    real rotor_near_fraction;
    real rotor_limiting_fraction;
    real surge_near_flow_ratio;
    real surge_min_window_fraction;
    real choke_near_flow_fraction;
    real choke_limiting_flow_fraction;
    real wastegate_remedy_margin;
    // --- limiter ---
    real limiter_near_fraction;
    real soft_cut_near_bands;
    // --- piston_speed ---
    real piston_speed_near_mps;
    real piston_speed_limiting_mps;
    real piston_speed_exceeded_mps;
    // --- map_grid ---
    real grid_edge_min_window_fraction;
    real grid_edge_near_span_fraction;
    // --- driveline ---
    real driveline_torque_margin;
    // --- nitrous ---
    real kit_lambda_near;
    real kit_lambda_limiting;
    real kit_lambda_exceeded;
    real kit_lambda_target;
    real bottle_duration_target_s;
    real bottle_duration_near_factor;
    real bottle_reserve_factor;
    real bottle_flow_retention_near;
    real bottle_flow_retention_limiting;
    // --- knock ---
    real knock_near_retard_deg;
    real knock_limiting_retard_deg;
    // --- valves ---
    real intake_d_over_bore_1v_min;
    real intake_d_over_bore_1v_max;
    real intake_d_over_bore_2v_min;
    real intake_d_over_bore_2v_max;
    real intake_d_over_bore_3v_min;
    real intake_d_over_bore_3v_max;
    real exhaust_over_intake_min;
    real exhaust_over_intake_max;
    real pack_share_of_ideal;
    // --- rod ---
    real rod_over_crank_radius_min;
    real rod_over_crank_radius_max;
    // --- compression ---
    real cr_na_min;
    real cr_na_max;
    real cr_forced_min;
    real cr_forced_max;
    // --- grid_coverage ---
    real grid_overshoot_limiting;
    real grid_overshoot_exceeded;
    // --- cam ---
    real cam_switch_over_limiter_min;
    real cam_switch_over_limiter_max;
    real cam_switch_hysteresis_min_rpm;
    real cam_switch_hysteresis_max_rpm;
    real cam_peak_lift_over_valve_d_max;
    // --- lift ---
    real lift_min_node_peak_min_mm;
    real lift_min_node_peak_max_fraction_of_full;
    // --- gearbox ---
    real gearbox_rating_over_limiting;
    real gearbox_rating_over_exceeded;
    real gearbox_rating_remedy_margin;
    // --- hitch ---
    real hitch_near_share;
    real hitch_limiting_share;
    real hitch_exceeded_share;
    // clang-format on
};

// The compiled defaults (static storage; thread-safe to read).
[[nodiscard]] const LimitPolicy& default_limit_policy();

// nullptr -> the defaults.
[[nodiscard]] inline const LimitPolicy& resolve_limit_policy(const LimitPolicy* p) { return p != nullptr ? *p : default_limit_policy(); }

// "" when the policy is usable, else "<key>: <what is wrong>" for the first problem: every number finite and inside its domain
// (> 0; fractions <= 1; margins >= 1) and each Near < Limiting < Exceeded style ordering holds. The loader calls it; consumers
// do not (they accept any numbers, so a test may probe with extremes).
[[nodiscard]] std::string validate_limit_policy(const LimitPolicy& policy);

// True iff any finding's severity is refused by the policy (nullptr = defaults: any Exceeded).
[[nodiscard]] bool has_refusing_finding(const std::vector<LimitFinding>& findings, const LimitPolicy* policy = nullptr);

} // namespace ps::drivetrain
