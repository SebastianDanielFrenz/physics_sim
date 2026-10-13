// core/src/drivetrain/engine_rules.cpp - engine garage P4: check_buildable (A10) and the grid helpers (A12).
// The classification (A11) is in engine_change.cpp. Numbers: engine_rules_table.h (and limit_thresholds.h for the piston speed).
#include "ps/drivetrain/engine_rules.h"

#include "engine_rules_table.h"
#include "gearbox_rating.h"
#include "limit_thresholds.h"

#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/limit_policy.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>

namespace ps::drivetrain {

namespace {

namespace tab = rules;

[[nodiscard]] std::string num(real v, int digits = 4) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*g", digits, static_cast<double>(v));
    return buf;
}
[[nodiscard]] std::string mm(real metres) { return num(metres * real(1000.0), 4) + " mm"; }

// True iff v is outside [lo, hi] by more than the comparison slack.
[[nodiscard]] bool below(real v, real lo) { return v < lo * (real(1.0) - tab::kRelTolerance.value); }
[[nodiscard]] bool above(real v, real hi) { return v > hi * (real(1.0) + tab::kRelTolerance.value); }

// A rule number that is NOT policy (a mathematical constant or a build-time rule): the table's own source text.
template <class T>
[[nodiscard]] const char* label_of(const T& t) {
    return t.source;
}

// The source label of a POLICY number (L0): the table's text while the policy holds the compiled default, else a label that
// says the number was overridden (the default's source would claim a citation the override does not have).
template <class T>
[[nodiscard]] std::string plab(const T& t, real current) {
    if (current == t.value) return t.source;
    return std::string("[policy] ") + t.name + " = " + num(current, 6) + " from a limit policy (default " + num(t.value, 6) +
           "; default source: " + t.source + ")";
}

// Relative overshoot of `need` over `edge` -> severity: any is Near, beyond kGridOvershootLimiting Limiting, beyond
// kGridOvershootExceeded Exceeded. Returns false when there is no overshoot.
[[nodiscard]] bool overshoot_severity(const LimitPolicy& pol, real need, real edge, LimitSeverity& out) {
    if (!(need > edge)) return false;
    const real rel = (need - edge) / edge;
    out = rel > pol.grid_overshoot_exceeded   ? LimitSeverity::Exceeded
          : rel > pol.grid_overshoot_limiting ? LimitSeverity::Limiting
                                                     : LimitSeverity::Near;
    return true;
}

struct Sink {
    std::vector<BuildIssue>& out;
    const LimitPolicy& pol;
    void add(BuildRule rule, LimitSeverity sev, const char* slot, std::string pointer, std::string message, std::string source,
             real value, real limit, LimitUnit unit) {
        BuildIssue b;
        b.rule = rule;
        b.severity = sev;
        b.slot = slot;
        b.field_pointer = std::move(pointer);
        b.message = std::move(message);
        b.source_label = std::move(source);
        b.value = value;
        b.limit = limit;
        b.unit = unit;
        out.push_back(std::move(b));
    }
};

[[nodiscard]] bool layout_allowed(int ni, int ne) {
    return (ni == 1 && ne == 1) || (ni == 2 && ne == 1) || (ni == 2 && ne == 2) || (ni == 3 && ne == 2);
}

[[nodiscard]] real ideal_density(int n_valves) {
    switch (n_valves) {
    case 2: return tab::kPackDensity2.value;
    case 3: return tab::kPackDensity3.value;
    case 4: return tab::kPackDensity4.value;
    case 5: return tab::kPackDensity5.value;
    default: return real(0.0);
    }
}

// The intake valve diameter band of a layout and the source labels of its two ends.
void intake_band(const LimitPolicy& pol, int ni, real& lo, real& hi, std::string& lo_label, std::string& hi_label) {
    switch (ni) {
    case 1:
        lo = pol.intake_d_over_bore_1v_min;
        hi = pol.intake_d_over_bore_1v_max;
        lo_label = plab(tab::kIntake1Min, lo);
        hi_label = plab(tab::kIntake1Max, hi);
        return;
    case 2:
        lo = pol.intake_d_over_bore_2v_min;
        hi = pol.intake_d_over_bore_2v_max;
        lo_label = plab(tab::kIntake2Min, lo);
        hi_label = plab(tab::kIntake2Max, hi);
        return;
    default:
        lo = pol.intake_d_over_bore_3v_min;
        hi = pol.intake_d_over_bore_3v_max;
        lo_label = plab(tab::kIntake3Min, lo);
        hi_label = plab(tab::kIntake3Max, hi);
        return;
    }
}

void check_mean_piston_speed(Sink& s, real stroke_m, real limiter_rpm) {
    if (!(stroke_m > real(0.0)) || !(limiter_rpm > real(0.0))) return;
    const real v = stroke_m * limiter_rpm / real(30.0);
    LimitSeverity sev{};
    std::string label;
    real limit = 0.0;
    // P3b's thresholds, one step lower: the dyno reports Limiting from 25 m/s because the part caps output there; as a static
    // rule 25 m/s is only 'unusual for a road engine' (the shipped hyper runs 26.2 m/s at its stock limiter), refusal starts
    // where P3b says Exceeded.
    if (v >= s.pol.piston_speed_exceeded_mps) {
        sev = LimitSeverity::Exceeded;
        limit = s.pol.piston_speed_exceeded_mps;
        label = plab(limits::kPistonSpeedExceeded, s.pol.piston_speed_exceeded_mps);
    } else if (v >= s.pol.piston_speed_limiting_mps) {
        sev = LimitSeverity::Near;
        limit = s.pol.piston_speed_limiting_mps;
        label = plab(limits::kPistonSpeedLimiting, s.pol.piston_speed_limiting_mps);
    } else {
        return;
    }
    const real max_rpm = limit * real(30.0) / stroke_m;
    s.add(BuildRule::MeanPistonSpeed, sev, "engine", "/limiter/rpm",
          "mean piston speed at the " + num(limiter_rpm, 5) + " rpm limiter is " + num(v, 3) + " m/s for a " + mm(stroke_m) +
              " stroke" + (sev == LimitSeverity::Exceeded ? ": above " : ": beyond ") + num(limit, 3) + " m/s (" + num(max_rpm, 5) +
              " rpm)",
          label, v, limit, LimitUnit::MetrePerSecond);
}

void check_cycle(Sink& s, const SimulatedEngineDesc& e, const CycleModelDesc& c) {
    const real bore = c.bore_m;
    const int ni = c.intake_valves, ne = c.exhaust_valves;
    const real di = c.intake_valve_d_m, de = c.exhaust_valve_d_m;

    // --- layout, diameter band, packaging, ratio ---
    const bool layout_ok = layout_allowed(ni, ne);
    if (!layout_ok) {
        s.add(BuildRule::ValveLayout, LimitSeverity::Exceeded, "engine", "/cycle/intake_valves",
              std::to_string(ni) + " intake and " + std::to_string(ne) +
                  " exhaust valves per cylinder is not a production layout (2V 1+1, 3V 2+1, 4V 2+2, 5V 3+2)",
              label_of(tab::kLayoutSource), real(ni + ne), real(0.0), LimitUnit::Dimensionless);
    }
    if (bore > real(0.0) && di > real(0.0) && de > real(0.0)) {
        if (layout_ok) {
            real lo = 0.0, hi = 0.0;
            std::string lo_label, hi_label;
            intake_band(s.pol, ni, lo, hi, lo_label, hi_label);
            const real frac = di / bore;
            if (below(frac, lo) || above(frac, hi)) {
                const bool low = below(frac, lo);
                s.add(BuildRule::ValveDiameterBand, LimitSeverity::Exceeded, "engine", "/cycle/intake_valve_d_mm",
                      "intake valve diameter " + mm(di) + " is " + num(frac, 3) + " of the " + mm(bore) + " bore; " +
                          std::to_string(ni + ne) + "-valve heads allow " + num(lo, 3) + "-" + num(hi, 3) + " (" +
                          mm(lo * bore) + " to " + mm(hi * bore) + ")",
                      low ? lo_label : hi_label, frac,
                      low ? lo : hi, LimitUnit::Dimensionless);
            }
            // Row rule per side (one or two valves).
            const auto row = [&](int n, real d, const char* name, const char* pointer) {
                if (n > 2) return;
                const real width = real(n) * d;
                const real cap = tab::kRowChordFraction.value * bore;
                if (above(width, cap)) {
                    s.add(BuildRule::ValvePackingRow, LimitSeverity::Exceeded, "engine", pointer,
                          std::string(name) + " valves " + mm(d) + " do not fit a " + mm(bore) + " bore with " +
                              std::to_string(ni + ne) + " valves: max " + mm(cap / real(n)),
                          label_of(tab::kRowChordFraction), d * real(1000.0), cap / real(n) * real(1000.0), LimitUnit::Millimetre);
                }
            };
            row(ni, di, "intake", "/cycle/intake_valve_d_mm");
            row(ne, de, "exhaust", "/cycle/exhaust_valve_d_mm");
            // Area rule over all valves.
            const real total = real(ni) * di * di + real(ne) * de * de;
            const real cap = s.pol.pack_share_of_ideal * ideal_density(ni + ne) * bore * bore;
            if (above(total, cap)) {
                const real scale = std::sqrt(cap / total);
                s.add(BuildRule::ValvePackingArea, LimitSeverity::Exceeded, "engine", "/cycle/intake_valve_d_mm",
                      "the " + std::to_string(ni + ne) + " valve heads cover " + num(total / (bore * bore), 3) +
                          " of the bore diameter squared; at most " + num(cap / (bore * bore), 3) +
                          " fits (scale every valve by " + num(scale, 3) + ")",
                      plab(tab::kPackShare, s.pol.pack_share_of_ideal), total / (bore * bore), cap / (bore * bore), LimitUnit::Dimensionless);
            }
        }
        const real ratio = de / di;
        if (below(ratio, s.pol.exhaust_over_intake_min) || above(ratio, s.pol.exhaust_over_intake_max)) {
            const bool low = below(ratio, s.pol.exhaust_over_intake_min);
            s.add(BuildRule::ExhaustIntakeRatio, LimitSeverity::Exceeded, "engine", "/cycle/exhaust_valve_d_mm",
                  "exhaust valve " + mm(de) + " over intake valve " + mm(di) + " is " + num(ratio, 3) + "; production heads run " +
                      num(s.pol.exhaust_over_intake_min, 2) + "-" + num(s.pol.exhaust_over_intake_max, 2) + " (" +
                      mm(s.pol.exhaust_over_intake_min * di) + " to " + mm(s.pol.exhaust_over_intake_max * di) + ")",
                  low ? plab(tab::kExhaustRatioMin, s.pol.exhaust_over_intake_min) : plab(tab::kExhaustRatioMax, s.pol.exhaust_over_intake_max), ratio,
                  low ? s.pol.exhaust_over_intake_min : s.pol.exhaust_over_intake_max, LimitUnit::Dimensionless);
        }
    }

    // --- rod ratio: rod length over crank radius (stroke / 2) ---
    if (c.rod_m > real(0.0) && c.stroke_m > real(0.0)) {
        const real r = c.rod_m / (c.stroke_m * real(0.5));
        if (below(r, s.pol.rod_over_crank_radius_min) || above(r, s.pol.rod_over_crank_radius_max)) {
            const bool low = below(r, s.pol.rod_over_crank_radius_min);
            s.add(BuildRule::RodRatio, LimitSeverity::Exceeded, "engine", "/cycle/rod_mm",
                  "rod " + mm(c.rod_m) + " over the crank radius " + mm(c.stroke_m * real(0.5)) + " is " + num(r, 3) +
                      "; allowed " + num(s.pol.rod_over_crank_radius_min, 2) + "-" + num(s.pol.rod_over_crank_radius_max, 2) + " (rod " +
                      mm(s.pol.rod_over_crank_radius_min * c.stroke_m * real(0.5)) + " to " +
                      mm(s.pol.rod_over_crank_radius_max * c.stroke_m * real(0.5)) + ")",
                  low ? plab(tab::kRodRatioMin, s.pol.rod_over_crank_radius_min) : plab(tab::kRodRatioMax, s.pol.rod_over_crank_radius_max), r,
                  low ? s.pol.rod_over_crank_radius_min : s.pol.rod_over_crank_radius_max, LimitUnit::Dimensionless);
        }
    }

    // --- compression ratio by aspiration ---
    if (c.compression_ratio > real(0.0)) {
        const bool forced = e.turbo_pair.has_value();
        const real lo = forced ? s.pol.cr_forced_min : s.pol.cr_na_min;
        const real hi = forced ? s.pol.cr_forced_max : s.pol.cr_na_max;
        if (below(c.compression_ratio, lo) || above(c.compression_ratio, hi)) {
            const bool low = below(c.compression_ratio, lo);
            s.add(BuildRule::CompressionRatio, LimitSeverity::Exceeded, "engine", "/cycle/compression_ratio",
                  "compression ratio " + num(c.compression_ratio, 4) + " is outside " + num(lo, 3) + "-" + num(hi, 3) +
                      (forced ? " for a forced-induction engine" : " for a naturally aspirated engine"),
                  forced ? (low ? plab(tab::kCrFiMin, lo) : plab(tab::kCrFiMax, hi)) : (low ? plab(tab::kCrNaMin, lo) : plab(tab::kCrNaMax, hi)),
                  c.compression_ratio, low ? lo : hi, LimitUnit::Dimensionless);
        }
    }

    // --- the generated maps must cover the limiter and the boost ---
    if (!c.grid_rpm.empty()) {
        const real top = c.grid_rpm.back();
        LimitSeverity sev{};
        if (overshoot_severity(s.pol, e.limiter.rpm, top, sev)) {
            s.add(BuildRule::LimiterAboveRpmGrid, sev, "engine", "/cycle/grid_rpm",
                  "the " + num(e.limiter.rpm, 5) + " rpm limiter is above the last rpm node " + num(top, 5) +
                      " of the generated maps (lookups hold the last row); extend the grid to the limiter",
                  plab(tab::kGridOvershootLimiting, s.pol.grid_overshoot_limiting), e.limiter.rpm, top, LimitUnit::RevPerMinute);
        }
    }
    if (e.turbo_pair.has_value() && !c.grid_p_im.empty()) {
        real top = c.grid_p_im.back();
        if (c.pressure_sampling.enabled && c.pressure_sampling.max_pa > real(0.0)) top = std::max(top, c.pressure_sampling.max_pa);
        const real need = kIsaAirPressurePa + e.turbo_pair->target_boost_pa;
        LimitSeverity sev{};
        if (overshoot_severity(s.pol, need, top, sev)) {
            s.add(BuildRule::BoostAbovePressureGrid, sev, "engine", "/cycle/grid_p_im_kpa",
                  "ambient plus the " + num(e.turbo_pair->target_boost_pa / real(1e5), 3) + " bar boost target is " +
                      num(need / real(1e3), 5) + " kPa, above the last manifold-pressure node " + num(top / real(1e3), 5) +
                      " kPa of the generated maps; add rows up to " + num(need / real(1e3), 5) + " kPa",
                  plab(tab::kGridOvershootLimiting, s.pol.grid_overshoot_limiting), need, top, LimitUnit::Pascal);
        }
    }
}

// Engine garage P9: peak lift (m) and the lift area (m*deg, trapezoids over the profile's own knots) of one lift curve.
void lift_figures(const ValveLiftProfile& p, real& peak, real& area) {
    peak = real(0.0);
    area = real(0.0);
    const auto& x = p.lift_m_vs_deg.x;
    const auto& y = p.lift_m_vs_deg.y;
    for (std::size_t i = 0; i < y.size(); ++i) {
        peak = std::max(peak, y[i]);
        if (i > 0) area += real(0.5) * (y[i] + y[i - 1]) * (x[i] - x[i - 1]);
    }
}

void check_cam_profiles(Sink& s, const SimulatedEngineDesc& e, const CycleModelDesc& c) {
    if (c.cam_profiles.size() < 2) return;
    const CamSwitchingDesc& sw = e.cam_switching;
    // --- the switch window ---
    if (!c.grid_rpm.empty() && (sw.rpm_off < c.grid_rpm.front() || sw.rpm_on > c.grid_rpm.back())) {
        const bool low = sw.rpm_off < c.grid_rpm.front();
        s.add(BuildRule::CamSwitchWindow, LimitSeverity::Limiting, "ecu", low ? "/switching/rpm_off" : "/switching/rpm_on",
              "the cam switch (" + num(sw.rpm_off, 5) + "-" + num(sw.rpm_on, 5) + " rpm) lies outside the generated rpm grid " +
                  num(c.grid_rpm.front(), 5) + "-" + num(c.grid_rpm.back(), 5) + " rpm (lookups hold the edge row there)",
              plab(tab::kGridOvershootLimiting, s.pol.grid_overshoot_limiting), low ? sw.rpm_off : sw.rpm_on, low ? c.grid_rpm.front() : c.grid_rpm.back(),
              LimitUnit::RevPerMinute);
    }
    if (e.limiter.rpm > real(0.0)) {
        const real frac = sw.rpm_on / e.limiter.rpm;
        if (below(frac, s.pol.cam_switch_over_limiter_min) || above(frac, s.pol.cam_switch_over_limiter_max)) {
            const bool low = below(frac, s.pol.cam_switch_over_limiter_min);
            s.add(BuildRule::CamSwitchWindow, LimitSeverity::Near, "ecu", "/switching/rpm_on",
                  "the cam switches at " + num(sw.rpm_on, 5) + " rpm, " + num(frac, 3) + " of the " + num(e.limiter.rpm, 5) +
                      " rpm limiter (production VTEC: 0.60-0.72)",
                  low ? plab(tab::kCamSwitchFractionMin, s.pol.cam_switch_over_limiter_min) : plab(tab::kCamSwitchFractionMax, s.pol.cam_switch_over_limiter_max), frac,
                  low ? s.pol.cam_switch_over_limiter_min : s.pol.cam_switch_over_limiter_max, LimitUnit::Fraction);
        }
    }
    const real hyst = sw.rpm_on - sw.rpm_off;
    if (below(hyst, s.pol.cam_switch_hysteresis_min_rpm) || above(hyst, s.pol.cam_switch_hysteresis_max_rpm)) {
        const bool low = below(hyst, s.pol.cam_switch_hysteresis_min_rpm);
        s.add(BuildRule::CamSwitchWindow, LimitSeverity::Near, "ecu", "/switching/rpm_off",
              "the cam switch hysteresis is " + num(hyst, 4) + " rpm (band " + num(s.pol.cam_switch_hysteresis_min_rpm, 3) + "-" +
                  num(s.pol.cam_switch_hysteresis_max_rpm, 4) + " rpm)",
              low ? plab(tab::kCamHysteresisMin, s.pol.cam_switch_hysteresis_min_rpm) : plab(tab::kCamHysteresisMax, s.pol.cam_switch_hysteresis_max_rpm), hyst,
              low ? s.pol.cam_switch_hysteresis_min_rpm : s.pol.cam_switch_hysteresis_max_rpm, LimitUnit::RevPerMinute);
    }
    // --- profile sanity ---
    for (int valve = 0; valve < 2; ++valve) {
        const char* name = valve == 0 ? "intake" : "exhaust";
        real pk0 = 0.0, ar0 = 0.0, pk1 = 0.0, ar1 = 0.0;
        lift_figures(valve == 0 ? c.cam_profiles[0].intake_lift : c.cam_profiles[0].exhaust_lift, pk0, ar0);
        lift_figures(valve == 0 ? c.cam_profiles[1].intake_lift : c.cam_profiles[1].exhaust_lift, pk1, ar1);
        if (below(pk1, pk0) || below(ar1, ar0)) {
            s.add(BuildRule::CamProfileOrder, LimitSeverity::Near, "cam",
                  std::string("/profiles/1/") + name + "_lift",
                  std::string("the second cam profile ('") + c.cam_profiles[1].id + "') has less " + name +
                      " lift than the first ('" + c.cam_profiles[0].id + "'): peak " + mm(pk1) + " vs " + mm(pk0) +
                      "; the ECU switches to it at high rpm",
                  "[D] the switching rule selects profile 2 above rpm_on, so it is the high-speed cam", pk1, pk0,
                  LimitUnit::Millimetre);
        }
        const real d = valve == 0 ? c.intake_valve_d_m : c.exhaust_valve_d_m;
        for (std::size_t k = 0; k < c.cam_profiles.size() && d > real(0.0); ++k) {
            real pk = 0.0, ar = 0.0;
            lift_figures(valve == 0 ? c.cam_profiles[k].intake_lift : c.cam_profiles[k].exhaust_lift, pk, ar);
            if (above(pk / d, s.pol.cam_peak_lift_over_valve_d_max)) {
                s.add(BuildRule::CamLiftOverValve, LimitSeverity::Limiting, "cam",
                      "/profiles/" + std::to_string(k) + "/" + name + "_lift",
                      std::string("cam profile '") + c.cam_profiles[k].id + "' lifts the " + name + " valves " + mm(pk) +
                          ", " + num(pk / d, 3) + " of their " + mm(d) + " head (band <= " +
                          num(s.pol.cam_peak_lift_over_valve_d_max, 3) + ")",
                      plab(tab::kCamLiftOverValveMax, s.pol.cam_peak_lift_over_valve_d_max), pk / d, s.pol.cam_peak_lift_over_valve_d_max, LimitUnit::Dimensionless);
            }
        }
    }
}

// Engine garage P10: a continuous intake lift family - monotone in u (the ECU's lift strategy inverts torque over u), the
// minimum node inside its band, and every node's lift under the valve band (the exhaust lift is the engine's single one).
void check_lift_family(Sink& s, const SimulatedEngineDesc& e, const CycleModelDesc& c) {
    const auto& f = c.lift_family;
    if (f.size() < 2) return;
    const auto node_ptr = [](std::size_t k) { return "/intake_lift_family/" + std::to_string(k) + "/intake_lift"; };
    for (std::size_t k = 1; k < f.size(); ++k) {
        real pk0 = 0.0, ar0 = 0.0, pk1 = 0.0, ar1 = 0.0;
        lift_figures(f[k - 1].intake_lift, pk0, ar0);
        lift_figures(f[k].intake_lift, pk1, ar1);
        if (!(pk1 > pk0) || !(ar1 > ar0)) {
            s.add(BuildRule::LiftFamilyOrder, LimitSeverity::Limiting, "cam", node_ptr(k),
                  "intake lift node u = " + num(f[k].u, 3) + " has no more lift than node u = " + num(f[k - 1].u, 3) + ": peak " +
                      mm(pk1) + " vs " + mm(pk0) + ", area " + num(ar1 * real(1e3), 4) + " vs " + num(ar0 * real(1e3), 4) +
                      " mm*deg (the ECU's lift strategy needs the load to rise with u)",
                  "[D] the ECU interpolates its torque targets over u, which assumes lift (and air) rising with u", pk1, pk0,
                  LimitUnit::Millimetre);
        }
    }
    // The minimum node: the ECU's min_u if it is a node, else the first node.
    std::size_t kmin = 0;
    for (std::size_t k = 0; k < f.size(); ++k) {
        if (f[k].u == e.lift_control.min_u) kmin = k;
    }
    real pk_min = 0.0, ar_min = 0.0, pk_full = 0.0, ar_full = 0.0;
    lift_figures(f[kmin].intake_lift, pk_min, ar_min);
    lift_figures(f.back().intake_lift, pk_full, ar_full);
    const real min_mm = pk_min * real(1000.0);
    if (below(min_mm, s.pol.lift_min_node_peak_min_mm)) {
        s.add(BuildRule::LiftMinimumBand, LimitSeverity::Near, "cam", node_ptr(kmin),
              "the minimum intake lift is " + mm(pk_min) + " (band >= " + num(s.pol.lift_min_node_peak_min_mm, 3) +
                  " mm; BMW's second-generation Valvetronic runs 0.18 mm)",
              plab(tab::kLiftMinPeakMinMm, s.pol.lift_min_node_peak_min_mm), min_mm, s.pol.lift_min_node_peak_min_mm, LimitUnit::Millimetre);
    }
    if (pk_full > real(0.0) && above(pk_min / pk_full, s.pol.lift_min_node_peak_max_fraction_of_full)) {
        s.add(BuildRule::LiftMinimumBand, LimitSeverity::Near, "cam", node_ptr(kmin),
              "the minimum intake lift " + mm(pk_min) + " is " + num(pk_min / pk_full, 3) + " of the " + mm(pk_full) +
                  " full lift (band <= " + num(s.pol.lift_min_node_peak_max_fraction_of_full, 3) +
                  "): the throttle, not the lift, sets the light loads",
              plab(tab::kLiftMinPeakMaxFraction, s.pol.lift_min_node_peak_max_fraction_of_full), pk_min / pk_full, s.pol.lift_min_node_peak_max_fraction_of_full, LimitUnit::Fraction);
    }
    // Lift over valve head: every intake node and the exhaust lift.
    for (std::size_t k = 0; k < f.size() && c.intake_valve_d_m > real(0.0); ++k) {
        real pk = 0.0, ar = 0.0;
        lift_figures(f[k].intake_lift, pk, ar);
        const real d = c.intake_valve_d_m;
        if (above(pk / d, s.pol.cam_peak_lift_over_valve_d_max)) {
            s.add(BuildRule::CamLiftOverValve, LimitSeverity::Limiting, "cam", node_ptr(k),
                  "intake lift node u = " + num(f[k].u, 3) + " lifts the intake valves " + mm(pk) + ", " + num(pk / d, 3) +
                      " of their " + mm(d) + " head (band <= " + num(s.pol.cam_peak_lift_over_valve_d_max, 3) + ")",
                  plab(tab::kCamLiftOverValveMax, s.pol.cam_peak_lift_over_valve_d_max), pk / d, s.pol.cam_peak_lift_over_valve_d_max, LimitUnit::Dimensionless);
        }
    }
    if (c.exhaust_valve_d_m > real(0.0)) {
        real pk = 0.0, ar = 0.0;
        lift_figures(c.exhaust_lift, pk, ar);
        const real d = c.exhaust_valve_d_m;
        if (above(pk / d, s.pol.cam_peak_lift_over_valve_d_max)) {
            s.add(BuildRule::CamLiftOverValve, LimitSeverity::Limiting, "cam", "/exhaust_lift",
                  "the exhaust lift " + mm(pk) + " is " + num(pk / d, 3) + " of the " + mm(d) + " exhaust valve head (band <= " +
                      num(s.pol.cam_peak_lift_over_valve_d_max, 3) + ")",
                  plab(tab::kCamLiftOverValveMax, s.pol.cam_peak_lift_over_valve_d_max), pk / d, s.pol.cam_peak_lift_over_valve_d_max, LimitUnit::Dimensionless);
        }
    }
}

void sort_issues(std::vector<BuildIssue>& v) {
    std::stable_sort(v.begin(), v.end(), [](const BuildIssue& a, const BuildIssue& b) {
        if (a.rule != b.rule) return static_cast<int>(a.rule) < static_cast<int>(b.rule);
        return a.field_pointer < b.field_pointer;
    });
}

} // namespace

const char* to_string(BuildRule r) {
    switch (r) {
    case BuildRule::ValveLayout: return "ValveLayout";
    case BuildRule::ValveDiameterBand: return "ValveDiameterBand";
    case BuildRule::ValvePackingRow: return "ValvePackingRow";
    case BuildRule::ValvePackingArea: return "ValvePackingArea";
    case BuildRule::ExhaustIntakeRatio: return "ExhaustIntakeRatio";
    case BuildRule::RodRatio: return "RodRatio";
    case BuildRule::MeanPistonSpeed: return "MeanPistonSpeed";
    case BuildRule::CompressionRatio: return "CompressionRatio";
    case BuildRule::LimiterAboveRpmGrid: return "LimiterAboveRpmGrid";
    case BuildRule::BoostAbovePressureGrid: return "BoostAbovePressureGrid";
    case BuildRule::LimiterAboveCurve: return "LimiterAboveCurve";
    case BuildRule::CamSwitchWindow: return "CamSwitchWindow";
    case BuildRule::CamProfileOrder: return "CamProfileOrder";
    case BuildRule::CamLiftOverValve: return "CamLiftOverValve";
    case BuildRule::LiftFamilyOrder: return "LiftFamilyOrder";
    case BuildRule::LiftMinimumBand: return "LiftMinimumBand";
    case BuildRule::GearboxRating: return "GearboxRating";
    }
    return "Unknown";
}

std::vector<BuildIssue> check_buildable(const SimulatedEngineDesc& e, const LimitPolicy* policy) {
    std::vector<BuildIssue> out;
    Sink s{out, resolve_limit_policy(policy)};
    if (e.architecture == EngineArchitecture::Piston4Stroke) {
        check_mean_piston_speed(s, e.piston.stroke_m, e.limiter.rpm);
        if (e.cycle.has_value()) check_cycle(s, e, *e.cycle);
        if (e.cycle.has_value()) check_cam_profiles(s, e, *e.cycle); // engine garage P9
        if (e.cycle.has_value()) check_lift_family(s, e, *e.cycle);  // engine garage P10
    }
    sort_issues(out);
    return out;
}

std::vector<BuildIssue> check_buildable(const TorqueMapEngineDesc& e, const LimitPolicy* policy) {
    std::vector<BuildIssue> out;
    Sink s{out, resolve_limit_policy(policy)};
    // A governed engine (the tractor's all-speed governor) never runs above its high idle: the fuel is cut there, and the
    // overspeed limiter above it is a backstop, not an operating speed.
    const real top_rpm = e.has_governor ? std::min(e.limiter.rpm, e.governor.high_idle_rpm) : e.limiter.rpm;
    const auto curve = [&](const vehicle::Curve1D& c, const char* pointer, const char* name) {
        if (c.x.empty()) return;
        LimitSeverity sev{};
        if (overshoot_severity(s.pol, top_rpm, c.x.back(), sev)) {
            s.add(BuildRule::LimiterAboveCurve, sev, "engine", pointer,
                  "the " + num(top_rpm, 5) + " rpm " + (e.has_governor && top_rpm < e.limiter.rpm ? "governor high idle" : "limiter") +
                      " is above the last rpm " + num(c.x.back(), 5) + " of the " + name +
                      " curve (the curve is held flat beyond it); extend the curve to the limiter",
                  plab(tab::kGridOvershootLimiting, s.pol.grid_overshoot_limiting), top_rpm, c.x.back(), LimitUnit::RevPerMinute);
        }
    };
    curve(e.wot_torque_nm_vs_rpm, "/wot_torque_nm_vs_rpm", "full-load torque");
    curve(e.motoring_torque_nm_vs_rpm, "/motoring_torque_nm_vs_rpm", "motoring torque");
    sort_issues(out);
    return out;
}

std::vector<BuildIssue> check_buildable(const ComponentParams& engine, const LimitPolicy* policy) {
    if (const auto* sim = std::get_if<SimulatedEngineDesc>(&engine)) return check_buildable(*sim, policy);
    if (const auto* tm = std::get_if<TorqueMapEngineDesc>(&engine)) return check_buildable(*tm, policy);
    return {};
}

real peak_net_torque_nm(const TorqueMapEngineDesc& e) {
    real peak = real(0.0);
    const vehicle::Curve1D& c = e.wot_torque_nm_vs_rpm;
    for (std::size_t i = 0; i < c.x.size() && i < c.y.size(); ++i) {
        if (c.x[i] > e.limiter.rpm && i > 0) break; // the curve is held flat beyond its last node; the limiter ends the sweep
        peak = std::max(peak, c.y[i]);
    }
    return peak;
}

std::vector<BuildIssue> check_gearbox_rating(const GearboxDesc& g, real peak_torque_nm, const LimitPolicy* policy) {
    const LimitPolicy& pol = resolve_limit_policy(policy);
    std::vector<BuildIssue> out;
    LimitSeverity sev{};
    if (!gearbox_rating_severity(peak_torque_nm, g.rated_input_torque_nm, pol, sev)) return out;
    const real over_pct = (peak_torque_nm / g.rated_input_torque_nm - real(1.0)) * real(100.0);
    BuildIssue b;
    b.rule = BuildRule::GearboxRating;
    b.severity = sev;
    b.slot = "gearbox";
    b.field_pointer = "/rated_input_torque_nm";
    b.message = "the engine's peak " + num(peak_torque_nm, 4) + " N*m is " + num(over_pct, 3) + " % over the gearbox's rated input torque " +
                num(g.rated_input_torque_nm, 4) + " N*m" +
                (sev == LimitSeverity::Exceeded
                     ? " (more than " + num(pol.gearbox_rating_over_exceeded * real(100.0), 3) + " % over" + (pol.refuses(sev) ? ": refused" : "") +
                           "); fit a stronger gearbox"
                     : "; a stronger gearbox would carry it");
    b.source_label = sev == LimitSeverity::Exceeded ? plab(tab::kGearboxOverExceeded, pol.gearbox_rating_over_exceeded)
                                                    : plab(tab::kGearboxOverLimiting, pol.gearbox_rating_over_limiting);
    b.value = peak_torque_nm;
    b.limit = g.rated_input_torque_nm;
    b.unit = LimitUnit::NewtonMetre;
    out.push_back(std::move(b));
    return out;
}

std::vector<BuildIssue> check_gearbox_rating(const PowertrainDesc& pt, real peak_torque_nm, const LimitPolicy* policy) {
    std::vector<BuildIssue> out;
    for (const ComponentDesc& c : pt.components) {
        if (const auto* g = std::get_if<GearboxDesc>(&c.params)) {
            std::vector<BuildIssue> one = check_gearbox_rating(*g, peak_torque_nm, policy);
            for (BuildIssue& b : one) out.push_back(std::move(b));
        }
    }
    return out;
}

bool has_blocking_issue(const std::vector<BuildIssue>& issues, const LimitPolicy* policy) {
    const LimitPolicy& pol = resolve_limit_policy(policy);
    return std::any_of(issues.begin(), issues.end(), [&pol](const BuildIssue& i) { return pol.refuses(i.severity); });
}

// ---------------------------------------------------------------------------------------------------------------------------
// A12 grid helpers
// ---------------------------------------------------------------------------------------------------------------------------

namespace {
[[nodiscard]] std::vector<real> nodes_at_or_below_ambient(const CycleModelDesc& desc) {
    std::vector<real> out;
    for (real p : desc.grid_p_im) {
        if (p <= kIsaAirPressurePa) out.push_back(p);
    }
    if (out.size() < 2) {
        throw std::invalid_argument("grid helper: the manifold-pressure axis needs at least two nodes at or below ambient (" +
                                    num(kIsaAirPressurePa / real(1e3), 6) + " kPa), found " + std::to_string(out.size()));
    }
    return out;
}
} // namespace

std::vector<real> ambient_grid_p_im(const CycleModelDesc& desc) {
    // Nodes up to the NA cap; if the old axis had nodes between the cap and ambient (a boosted engine's WOT row at 100 kPa, or a
    // node above ambient), they collapse into one node at the cap - the generator throws for a node the engine cannot reach with
    // the throttle fully open, which for a big-valve V8 is already below 98 kPa at the top of its rpm range.
    const real cap = tab::kNaGridTopFraction.value * kIsaAirPressurePa;
    std::vector<real> out;
    bool capped = false;
    for (real p : desc.grid_p_im) {
        if (p <= cap) {
            out.push_back(p);
        } else {
            capped = true;
        }
    }
    if (capped && (out.empty() || out.back() < cap - real(1e-6))) out.push_back(cap);
    if (out.size() < 2) {
        throw std::invalid_argument("ambient_grid_p_im: the manifold-pressure axis needs at least two nodes at or below " +
                                    num(cap / real(1e3), 6) + " kPa, found " + std::to_string(out.size()));
    }
    return out;
}

std::vector<real> boost_grid_p_im(const CycleModelDesc& desc, real target_boost_bar) {
    if (!(target_boost_bar > real(0.0)) || !std::isfinite(target_boost_bar)) {
        throw std::invalid_argument("boost_grid_p_im: the boost target must be a finite number > 0 bar");
    }
    std::vector<real> out = nodes_at_or_below_ambient(desc);
    // Engine garage P5: a node AT ambient when the stock axis stops well short of it (see kBoostGridAmbientNodeGapPa).
    if (kIsaAirPressurePa - out.back() > tab::kBoostGridAmbientNodeGapPa.value) out.push_back(kIsaAirPressurePa);
    const real top = std::round((tab::kBoostGridAmbientBar.value + target_boost_bar + tab::kBoostGridMargin.value) * real(1e5) *
                                real(1e3)) /
                     real(1e3);
    const real p_last = out.back();
    const real step = tab::kBoostGridStepPa.value;
    const real min_last = tab::kBoostGridMinLastStepPa.value;
    for (int k = 1;; ++k) {
        const real node = p_last + real(k) * step;
        if (top - node < min_last - real(1e-6)) break;
        out.push_back(node);
    }
    out.push_back(top);
    return out;
}

std::vector<real> extend_grid_rpm(const CycleModelDesc& desc, real limiter_rpm) {
    const std::vector<real>& g = desc.grid_rpm;
    if (g.size() < 2) throw std::invalid_argument("extend_grid_rpm: the rpm axis needs at least two nodes");
    std::vector<real> out = g;
    if (!(limiter_rpm > g.back())) return out;
    const real step = g.back() - g[g.size() - 2];
    real node = g.back() + step;
    while (limiter_rpm - node > real(0.5) * step) {
        out.push_back(node);
        node += step;
    }
    out.push_back(limiter_rpm);
    return out;
}

} // namespace ps::drivetrain
