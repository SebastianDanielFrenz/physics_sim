// core/src/drivetrain/engine_rules.cpp - engine garage P4: check_buildable (A10) and the grid helpers (A12).
// The classification (A11) is in engine_change.cpp. Numbers: engine_rules_table.h (and limit_thresholds.h for the piston speed).
#include "ps/drivetrain/engine_rules.h"

#include "engine_rules_table.h"
#include "limit_thresholds.h"

#include "ps/drivetrain/isa_air.h"

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

template <class T>
[[nodiscard]] const char* label_of(const T& t) {
    return t.source;
}

// Relative overshoot of `need` over `edge` -> severity: any is Near, beyond kGridOvershootLimiting Limiting, beyond
// kGridOvershootExceeded Exceeded. Returns false when there is no overshoot.
[[nodiscard]] bool overshoot_severity(real need, real edge, LimitSeverity& out) {
    if (!(need > edge)) return false;
    const real rel = (need - edge) / edge;
    out = rel > tab::kGridOvershootExceeded.value   ? LimitSeverity::Exceeded
          : rel > tab::kGridOvershootLimiting.value ? LimitSeverity::Limiting
                                                     : LimitSeverity::Near;
    return true;
}

struct Sink {
    std::vector<BuildIssue>& out;
    void add(BuildRule rule, LimitSeverity sev, const char* slot, std::string pointer, std::string message, const char* source,
             real value, real limit, LimitUnit unit) {
        BuildIssue b;
        b.rule = rule;
        b.severity = sev;
        b.slot = slot;
        b.field_pointer = std::move(pointer);
        b.message = std::move(message);
        b.source_label = source;
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

void intake_band(int ni, real& lo, real& hi, const tab::Threshold*& lo_t) {
    switch (ni) {
    case 1: lo = tab::kIntake1Min.value; hi = tab::kIntake1Max.value; lo_t = &tab::kIntake1Max; return;
    case 2: lo = tab::kIntake2Min.value; hi = tab::kIntake2Max.value; lo_t = &tab::kIntake2Max; return;
    default: lo = tab::kIntake3Min.value; hi = tab::kIntake3Max.value; lo_t = &tab::kIntake3Max; return;
    }
}

void check_mean_piston_speed(Sink& s, real stroke_m, real limiter_rpm) {
    if (!(stroke_m > real(0.0)) || !(limiter_rpm > real(0.0))) return;
    const real v = stroke_m * limiter_rpm / real(30.0);
    LimitSeverity sev{};
    const char* label = nullptr;
    real limit = 0.0;
    // P3b's thresholds, one step lower: the dyno reports Limiting from 25 m/s because the part caps output there; as a static
    // rule 25 m/s is only 'unusual for a road engine' (the shipped hyper runs 26.2 m/s at its stock limiter), refusal starts
    // where P3b says Exceeded.
    if (v >= limits::kPistonSpeedExceeded.value) {
        sev = LimitSeverity::Exceeded;
        limit = limits::kPistonSpeedExceeded.value;
        label = label_of(limits::kPistonSpeedExceeded);
    } else if (v >= limits::kPistonSpeedLimiting.value) {
        sev = LimitSeverity::Near;
        limit = limits::kPistonSpeedLimiting.value;
        label = label_of(limits::kPistonSpeedLimiting);
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
            const tab::Threshold* t = nullptr;
            intake_band(ni, lo, hi, t);
            const real frac = di / bore;
            if (below(frac, lo) || above(frac, hi)) {
                const bool low = below(frac, lo);
                s.add(BuildRule::ValveDiameterBand, LimitSeverity::Exceeded, "engine", "/cycle/intake_valve_d_mm",
                      "intake valve diameter " + mm(di) + " is " + num(frac, 3) + " of the " + mm(bore) + " bore; " +
                          std::to_string(ni + ne) + "-valve heads allow " + num(lo, 3) + "-" + num(hi, 3) + " (" +
                          mm(lo * bore) + " to " + mm(hi * bore) + ")",
                      label_of(low ? (ni == 1 ? tab::kIntake1Min : ni == 2 ? tab::kIntake2Min : tab::kIntake3Min) : *t), frac,
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
            const real cap = tab::kPackShare.value * ideal_density(ni + ne) * bore * bore;
            if (above(total, cap)) {
                const real scale = std::sqrt(cap / total);
                s.add(BuildRule::ValvePackingArea, LimitSeverity::Exceeded, "engine", "/cycle/intake_valve_d_mm",
                      "the " + std::to_string(ni + ne) + " valve heads cover " + num(total / (bore * bore), 3) +
                          " of the bore diameter squared; at most " + num(cap / (bore * bore), 3) +
                          " fits (scale every valve by " + num(scale, 3) + ")",
                      label_of(tab::kPackShare), total / (bore * bore), cap / (bore * bore), LimitUnit::Dimensionless);
            }
        }
        const real ratio = de / di;
        if (below(ratio, tab::kExhaustRatioMin.value) || above(ratio, tab::kExhaustRatioMax.value)) {
            const bool low = below(ratio, tab::kExhaustRatioMin.value);
            s.add(BuildRule::ExhaustIntakeRatio, LimitSeverity::Exceeded, "engine", "/cycle/exhaust_valve_d_mm",
                  "exhaust valve " + mm(de) + " over intake valve " + mm(di) + " is " + num(ratio, 3) + "; production heads run " +
                      num(tab::kExhaustRatioMin.value, 2) + "-" + num(tab::kExhaustRatioMax.value, 2) + " (" +
                      mm(tab::kExhaustRatioMin.value * di) + " to " + mm(tab::kExhaustRatioMax.value * di) + ")",
                  label_of(low ? tab::kExhaustRatioMin : tab::kExhaustRatioMax), ratio,
                  low ? tab::kExhaustRatioMin.value : tab::kExhaustRatioMax.value, LimitUnit::Dimensionless);
        }
    }

    // --- rod ratio: rod length over crank radius (stroke / 2) ---
    if (c.rod_m > real(0.0) && c.stroke_m > real(0.0)) {
        const real r = c.rod_m / (c.stroke_m * real(0.5));
        if (below(r, tab::kRodRatioMin.value) || above(r, tab::kRodRatioMax.value)) {
            const bool low = below(r, tab::kRodRatioMin.value);
            s.add(BuildRule::RodRatio, LimitSeverity::Exceeded, "engine", "/cycle/rod_mm",
                  "rod " + mm(c.rod_m) + " over the crank radius " + mm(c.stroke_m * real(0.5)) + " is " + num(r, 3) +
                      "; allowed " + num(tab::kRodRatioMin.value, 2) + "-" + num(tab::kRodRatioMax.value, 2) + " (rod " +
                      mm(tab::kRodRatioMin.value * c.stroke_m * real(0.5)) + " to " +
                      mm(tab::kRodRatioMax.value * c.stroke_m * real(0.5)) + ")",
                  label_of(low ? tab::kRodRatioMin : tab::kRodRatioMax), r,
                  low ? tab::kRodRatioMin.value : tab::kRodRatioMax.value, LimitUnit::Dimensionless);
        }
    }

    // --- compression ratio by aspiration ---
    if (c.compression_ratio > real(0.0)) {
        const bool forced = e.turbo_pair.has_value();
        const real lo = forced ? tab::kCrFiMin.value : tab::kCrNaMin.value;
        const real hi = forced ? tab::kCrFiMax.value : tab::kCrNaMax.value;
        if (below(c.compression_ratio, lo) || above(c.compression_ratio, hi)) {
            const bool low = below(c.compression_ratio, lo);
            s.add(BuildRule::CompressionRatio, LimitSeverity::Exceeded, "engine", "/cycle/compression_ratio",
                  "compression ratio " + num(c.compression_ratio, 4) + " is outside " + num(lo, 3) + "-" + num(hi, 3) +
                      (forced ? " for a forced-induction engine" : " for a naturally aspirated engine"),
                  label_of(forced ? (low ? tab::kCrFiMin : tab::kCrFiMax) : (low ? tab::kCrNaMin : tab::kCrNaMax)),
                  c.compression_ratio, low ? lo : hi, LimitUnit::Dimensionless);
        }
    }

    // --- the generated maps must cover the limiter and the boost ---
    if (!c.grid_rpm.empty()) {
        const real top = c.grid_rpm.back();
        LimitSeverity sev{};
        if (overshoot_severity(e.limiter.rpm, top, sev)) {
            s.add(BuildRule::LimiterAboveRpmGrid, sev, "engine", "/cycle/grid_rpm",
                  "the " + num(e.limiter.rpm, 5) + " rpm limiter is above the last rpm node " + num(top, 5) +
                      " of the generated maps (lookups hold the last row); extend the grid to the limiter",
                  label_of(tab::kGridOvershootLimiting), e.limiter.rpm, top, LimitUnit::RevPerMinute);
        }
    }
    if (e.turbo_pair.has_value() && !c.grid_p_im.empty()) {
        real top = c.grid_p_im.back();
        if (c.pressure_sampling.enabled && c.pressure_sampling.max_pa > real(0.0)) top = std::max(top, c.pressure_sampling.max_pa);
        const real need = kIsaAirPressurePa + e.turbo_pair->target_boost_pa;
        LimitSeverity sev{};
        if (overshoot_severity(need, top, sev)) {
            s.add(BuildRule::BoostAbovePressureGrid, sev, "engine", "/cycle/grid_p_im_kpa",
                  "ambient plus the " + num(e.turbo_pair->target_boost_pa / real(1e5), 3) + " bar boost target is " +
                      num(need / real(1e3), 5) + " kPa, above the last manifold-pressure node " + num(top / real(1e3), 5) +
                      " kPa of the generated maps; add rows up to " + num(need / real(1e3), 5) + " kPa",
                  label_of(tab::kGridOvershootLimiting), need, top, LimitUnit::Pascal);
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
              label_of(tab::kGridOvershootLimiting), low ? sw.rpm_off : sw.rpm_on, low ? c.grid_rpm.front() : c.grid_rpm.back(),
              LimitUnit::RevPerMinute);
    }
    if (e.limiter.rpm > real(0.0)) {
        const real frac = sw.rpm_on / e.limiter.rpm;
        if (below(frac, tab::kCamSwitchFractionMin.value) || above(frac, tab::kCamSwitchFractionMax.value)) {
            const bool low = below(frac, tab::kCamSwitchFractionMin.value);
            s.add(BuildRule::CamSwitchWindow, LimitSeverity::Near, "ecu", "/switching/rpm_on",
                  "the cam switches at " + num(sw.rpm_on, 5) + " rpm, " + num(frac, 3) + " of the " + num(e.limiter.rpm, 5) +
                      " rpm limiter (production VTEC: 0.60-0.72)",
                  label_of(low ? tab::kCamSwitchFractionMin : tab::kCamSwitchFractionMax), frac,
                  low ? tab::kCamSwitchFractionMin.value : tab::kCamSwitchFractionMax.value, LimitUnit::Fraction);
        }
    }
    const real hyst = sw.rpm_on - sw.rpm_off;
    if (below(hyst, tab::kCamHysteresisMin.value) || above(hyst, tab::kCamHysteresisMax.value)) {
        const bool low = below(hyst, tab::kCamHysteresisMin.value);
        s.add(BuildRule::CamSwitchWindow, LimitSeverity::Near, "ecu", "/switching/rpm_off",
              "the cam switch hysteresis is " + num(hyst, 4) + " rpm (band " + num(tab::kCamHysteresisMin.value, 3) + "-" +
                  num(tab::kCamHysteresisMax.value, 4) + " rpm)",
              label_of(low ? tab::kCamHysteresisMin : tab::kCamHysteresisMax), hyst,
              low ? tab::kCamHysteresisMin.value : tab::kCamHysteresisMax.value, LimitUnit::RevPerMinute);
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
            if (above(pk / d, tab::kCamLiftOverValveMax.value)) {
                s.add(BuildRule::CamLiftOverValve, LimitSeverity::Limiting, "cam",
                      "/profiles/" + std::to_string(k) + "/" + name + "_lift",
                      std::string("cam profile '") + c.cam_profiles[k].id + "' lifts the " + name + " valves " + mm(pk) +
                          ", " + num(pk / d, 3) + " of their " + mm(d) + " head (band <= " +
                          num(tab::kCamLiftOverValveMax.value, 3) + ")",
                      label_of(tab::kCamLiftOverValveMax), pk / d, tab::kCamLiftOverValveMax.value, LimitUnit::Dimensionless);
            }
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
    }
    return "Unknown";
}

std::vector<BuildIssue> check_buildable(const SimulatedEngineDesc& e) {
    std::vector<BuildIssue> out;
    Sink s{out};
    if (e.architecture == EngineArchitecture::Piston4Stroke) {
        check_mean_piston_speed(s, e.piston.stroke_m, e.limiter.rpm);
        if (e.cycle.has_value()) check_cycle(s, e, *e.cycle);
        if (e.cycle.has_value()) check_cam_profiles(s, e, *e.cycle); // engine garage P9
    }
    sort_issues(out);
    return out;
}

std::vector<BuildIssue> check_buildable(const TorqueMapEngineDesc& e) {
    std::vector<BuildIssue> out;
    Sink s{out};
    // A governed engine (the tractor's all-speed governor) never runs above its high idle: the fuel is cut there, and the
    // overspeed limiter above it is a backstop, not an operating speed.
    const real top_rpm = e.has_governor ? std::min(e.limiter.rpm, e.governor.high_idle_rpm) : e.limiter.rpm;
    const auto curve = [&](const vehicle::Curve1D& c, const char* pointer, const char* name) {
        if (c.x.empty()) return;
        LimitSeverity sev{};
        if (overshoot_severity(top_rpm, c.x.back(), sev)) {
            s.add(BuildRule::LimiterAboveCurve, sev, "engine", pointer,
                  "the " + num(top_rpm, 5) + " rpm " + (e.has_governor && top_rpm < e.limiter.rpm ? "governor high idle" : "limiter") +
                      " is above the last rpm " + num(c.x.back(), 5) + " of the " + name +
                      " curve (the curve is held flat beyond it); extend the curve to the limiter",
                  label_of(tab::kGridOvershootLimiting), top_rpm, c.x.back(), LimitUnit::RevPerMinute);
        }
    };
    curve(e.wot_torque_nm_vs_rpm, "/wot_torque_nm_vs_rpm", "full-load torque");
    curve(e.motoring_torque_nm_vs_rpm, "/motoring_torque_nm_vs_rpm", "motoring torque");
    sort_issues(out);
    return out;
}

std::vector<BuildIssue> check_buildable(const ComponentParams& engine) {
    if (const auto* sim = std::get_if<SimulatedEngineDesc>(&engine)) return check_buildable(*sim);
    if (const auto* tm = std::get_if<TorqueMapEngineDesc>(&engine)) return check_buildable(*tm);
    return {};
}

bool has_blocking_issue(const std::vector<BuildIssue>& issues) {
    return std::any_of(issues.begin(), issues.end(), [](const BuildIssue& i) { return i.severity == LimitSeverity::Exceeded; });
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
