// core/src/drivetrain/engine_dyno.cpp - engine garage P3: run_dyno_sweep (ps/drivetrain/engine_dyno.h).
//
// Three methods, one per engine kind; see the public header for what each delivers. Every number a point reports
// comes from the runtime engine's own code: EngineTorqueMap's resolved curves/fuel (powertrain.cpp's
// to_engine_desc), SimulatedEngine::steady_point (the definition of the engine's steady tables), or the stepped
// SimulatedEngine itself (pre_solve/post_solve with TurboPair, wastegate PID, charge lag). No physics lives here.
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/map_pressure_index.h"

#include "engine_params_util.h"
#include "engine_simulated.h"
#include "engine_torque_map.h"
#include "gearbox_rating.h"
#include "nitrous_system.h"
#include "powertrain.h" // to_engine_desc

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace ps::drivetrain {
namespace {

constexpr real kNaN = std::numeric_limits<real>::quiet_NaN();
constexpr real kTwoPiRad = 6.283185307179586; // bench_hyper_calibration's literal, so a point at the same rpm sees the same omega
constexpr real kStepHz = 960.0;               // [C] the car's substep rate (WorldConfig::substep_rate_hz default)
constexpr real kDefaultRpmStep = 250.0;       // [AS] design 10.1

void validate_spec(const DynoSweepSpec& s) {
    const auto bad = [](const char* what) { throw std::invalid_argument(std::string("run_dyno_sweep: ") + what); };
    for (real r : s.rpm) {
        if (!(std::isfinite(r) && r > real(0.0))) bad("every rpm must be finite and > 0");
    }
    if (!(std::isfinite(s.throttle) && s.throttle >= real(0.0) && s.throttle <= real(1.0))) bad("throttle must be in [0, 1]");
    if (!(std::isfinite(s.ambient.pressure_pa) && s.ambient.pressure_pa > real(0.0) && std::isfinite(s.ambient.temperature_k) &&
          s.ambient.temperature_k > real(0.0))) {
        bad("ambient pressure and temperature must be finite and > 0");
    }
    if (!(std::isfinite(s.max_settle_s) && s.max_settle_s > real(0.0))) bad("max_settle_s must be > 0");
    if (!(std::isfinite(s.settle_window_s) && s.settle_window_s > real(0.0))) bad("settle_window_s must be > 0");
    if (!(std::isfinite(s.settle_boost_bar_per_s) && s.settle_boost_bar_per_s >= real(0.0))) bad("settle_boost_bar_per_s must be >= 0");
    if (!(std::isfinite(s.settle_torque_rel_per_s) && s.settle_torque_rel_per_s >= real(0.0))) bad("settle_torque_rel_per_s must be >= 0");
}

// idle .. limiter in kDefaultRpmStep steps, the limiter point last and never duplicated.
std::vector<real> default_rpm_grid(real idle_rpm, real limiter_rpm) {
    if (!(idle_rpm > real(0.0)) || !(limiter_rpm > idle_rpm)) {
        throw std::invalid_argument("run_dyno_sweep: the engine needs 0 < idle_rpm < limiter rpm for the default rpm grid");
    }
    std::vector<real> out;
    for (int k = 0;; ++k) {
        const real r = idle_rpm + real(k) * kDefaultRpmStep;
        if (!(r < limiter_rpm - real(1e-9))) break;
        out.push_back(r);
    }
    out.push_back(limiter_rpm);
    return out;
}

real bsfc_g_kwh(real fuel_g_s, real power_kw) {
    return (power_kw > real(0.0) && std::isfinite(fuel_g_s)) ? fuel_g_s * real(3600.0) / power_kw : kNaN;
}

bool cancelled(const std::atomic<bool>* cancel) { return cancel != nullptr && cancel->load(std::memory_order_relaxed); }

// ---------------------------------------------------------------------------------------------------------------
// Limit findings (engine garage P3b-1). Thresholds: limit_thresholds.h (the one sourced table).
using Findings = std::vector<LimitFinding>;
constexpr real kInf = std::numeric_limits<real>::infinity();

LimitFinding make_finding(LimitKind kind, LimitSeverity severity, real rpm, real value, real limit, LimitUnit unit, PartRef part) {
    LimitFinding f;
    f.kind = kind;
    f.severity = severity;
    f.rpm = rpm;
    f.value = value;
    f.limit = limit;
    f.unit = unit;
    f.part = std::move(part);
    return f;
}

// The engine file's grid edges a simulated engine's (rpm, p_im)-indexed maps (ve, the ECU tables, MBT, knock limit, ...)
// are looked up on. Table2D clamps silently outside them. All those tables share one grid (the loader/generator builds them
// together); the volumetric efficiency table stands for them.
struct GridProbe {
    bool valid = false;
    real rpm_lo = 0.0, rpm_hi = 0.0, p_lo = 0.0, p_hi = 0.0;
    const char* rpm_field = "";
    const char* p_field = "";
    // Engine garage P7d: generated WOT rows - the manifold pressure is compared in the tables' own (warped) coordinate
    // (map_pressure_index at the sweep's ambient), and reaching the top (WOT) row is the design, not "near" an edge.
    const SimulatedEngineMapsDesc* warp = nullptr;
    real ambient_pa = 0.0;
    bool p_near_upper = true;
};

GridProbe make_grid_probe(const SimulatedEngineDesc& d, const SimulatedEngineMaps& m, real ambient_pa) {
    GridProbe g;
    const Table2D& ve = m.maps.ve;
    if (ve.x.empty() || ve.y.empty()) return g;
    if (has_wot_rows(m.maps)) {
        g.warp = &m.maps;
        g.ambient_pa = ambient_pa;
        g.p_near_upper = false;
    }
    g.valid = true;
    g.rpm_lo = ve.x.front();
    g.rpm_hi = ve.x.back();
    g.p_lo = ve.y.front();
    g.p_hi = ve.y.back();
    // A generated engine owns its grid in the "cycle" block, a measured one in the maps block.
    g.rpm_field = d.cycle.has_value() ? "/cycle/grid_rpm" : "/maps/ve/rpm";
    g.p_field = d.cycle.has_value() ? "/cycle/grid_p_im_kpa" : "/maps/ve/p_im_kpa";
    return g;
}

// Samples of one settled window (or the one instant evaluation of a steady point) against the grid.
struct EdgeAcc {
    long n = 0, p_above = 0, p_below = 0, rpm_above = 0, rpm_below = 0;
    real p_max = -kInf, p_min = kInf, rpm_max = -kInf, rpm_min = kInf;
    void add(const GridProbe& g, real rpm, real p_in_pa) {
        const real p_pa = g.warp != nullptr ? map_pressure_index(*g.warp, rpm, p_in_pa, g.ambient_pa).y : p_in_pa;
        ++n;
        if (p_pa > g.p_hi) ++p_above;
        if (p_pa < g.p_lo) ++p_below;
        if (rpm > g.rpm_hi) ++rpm_above;
        if (rpm < g.rpm_lo) ++rpm_below;
        p_max = std::max(p_max, p_pa);
        p_min = std::min(p_min, p_pa);
        rpm_max = std::max(rpm_max, rpm);
        rpm_min = std::min(rpm_min, rpm);
    }
};

// `near_upper`: report "near" the upper edge. The rpm axis does not: a sweep ends at the limiter, which a grid built for the
// engine ends at too, so touching the top rpm is the design, only going past it is a finding.
void emit_edge_axis(Findings& out, const LimitPolicy& pol, real rpm_tag, long n, long above, long below, real vmax, real vmin, real lo, real hi, LimitUnit unit,
                    const char* field, const std::string& engine_file, bool near_upper) {
    const real min_frac = pol.grid_edge_min_window_fraction;
    const real nf = real(n);
    if (real(above) / nf >= min_frac) {
        out.push_back(make_finding(LimitKind::MapGridEdge, LimitSeverity::Exceeded, rpm_tag, vmax, hi, unit, PartRef{"engine", engine_file, field}));
    } else if (near_upper && vmax >= hi - pol.grid_edge_near_span_fraction * (hi - lo)) {
        out.push_back(make_finding(LimitKind::MapGridEdge, LimitSeverity::Near, rpm_tag, vmax, hi, unit, PartRef{"engine", engine_file, field}));
    }
    // A lower edge is the normal idle / closed-throttle region: only a clamp is reported, never "near".
    if (real(below) / nf >= min_frac) {
        out.push_back(make_finding(LimitKind::MapGridEdge, LimitSeverity::Exceeded, rpm_tag, vmin, lo, unit, PartRef{"engine", engine_file, field}));
    }
}

void edge_findings(Findings& out, const LimitPolicy& pol, real rpm_tag, const GridProbe& g, const EdgeAcc& e, const std::string& engine_file) {
    if (!g.valid || e.n == 0) return;
    emit_edge_axis(out, pol, rpm_tag, e.n, e.p_above, e.p_below, e.p_max, e.p_min, g.p_lo, g.p_hi, LimitUnit::Pascal, g.p_field, engine_file, g.p_near_upper);
    emit_edge_axis(out, pol, rpm_tag, e.n, e.rpm_above, e.rpm_below, e.rpm_max, e.rpm_min, g.rpm_lo, g.rpm_hi, LimitUnit::RevPerMinute,
                   g.rpm_field, engine_file, false);
}

// RevLimiter (SoftCut taper) / FuelCut (latch): the swept rpm reaches the limiter. The dyno never applies the limiter, so a
// point at or above it is one the engine would not hold.
void limiter_findings(Findings& out, const LimitPolicy& pol, const EngineLimiterDesc& lim, real rpm, const std::string& engine_file) {
    constexpr real kEps = 1e-9;
    if (lim.type == LimiterType::FuelCut) {
        LimitSeverity s = LimitSeverity::Near;
        if (rpm > lim.rpm + kEps) s = LimitSeverity::Exceeded;
        else if (rpm >= lim.rpm - kEps) s = LimitSeverity::Limiting;
        else if (rpm >= pol.limiter_near_fraction * lim.rpm) s = LimitSeverity::Near;
        else return;
        out.push_back(make_finding(LimitKind::FuelCut, s, rpm, rpm, lim.rpm, LimitUnit::RevPerMinute, PartRef{"engine", engine_file, "/limiter/rpm"}));
    } else {
        LimitSeverity s = LimitSeverity::Near;
        if (rpm >= lim.rpm - kEps) s = LimitSeverity::Exceeded;
        else if (rpm > lim.rpm - lim.band_rpm) s = LimitSeverity::Limiting;
        else if (rpm > lim.rpm - pol.soft_cut_near_bands * lim.band_rpm) s = LimitSeverity::Near;
        else return;
        out.push_back(make_finding(LimitKind::RevLimiter, s, rpm, rpm, lim.rpm, LimitUnit::RevPerMinute, PartRef{"engine", engine_file, "/limiter/rpm"}));
    }
}

// MeanPistonSpeed = stroke * rpm / 30 against the table. The lever the player has is the limiter, hence its field.
void piston_speed_finding(Findings& out, const LimitPolicy& pol, const SimulatedEngineDesc& d, real rpm, const std::string& engine_file) {
    if (d.architecture != EngineArchitecture::Piston4Stroke) return;
    const real v = d.piston.stroke_m * rpm / real(30.0);
    LimitSeverity s = LimitSeverity::Near;
    if (v >= pol.piston_speed_exceeded_mps) s = LimitSeverity::Exceeded;
    else if (v >= pol.piston_speed_limiting_mps) s = LimitSeverity::Limiting;
    else if (v >= pol.piston_speed_near_mps) s = LimitSeverity::Near;
    else return;
    out.push_back(make_finding(LimitKind::MeanPistonSpeed, s, rpm, v, pol.piston_speed_limiting_mps, LimitUnit::MetrePerSecond,
                               PartRef{"engine", engine_file, "/limiter/rpm"}));
}

// G1: BeyondEngineData - the point lies above the engine's own data and the roll-off (armed: the limiter lies above the
// data) cuts the fired torque there. value = rpm, limit = the data's top rpm; the lever is the limiter. Nothing while
// disarmed, so every engine whose limiter lies inside its data reports exactly what it did before G1.
void beyond_data_finding(Findings& out, const RpmRolloff& r, real rpm, const std::string& engine_file) {
    if (!r.active(rpm)) return;
    const LimitSeverity s = rpm >= r.end_rpm ? LimitSeverity::Exceeded : LimitSeverity::Limiting;
    out.push_back(make_finding(LimitKind::BeyondEngineData, s, rpm, rpm, r.start_rpm, LimitUnit::RevPerMinute,
                               PartRef{"engine", engine_file, "/limiter/rpm"}));
}

// Engine garage P7: Knock - the knock limit holds the applied spark `retard` deg below the ECU schedule at this point.
// Part: the burnt fuel's octane (the ECU's knock_margin_deg is the other lever, named in engine.md). Remedy: an unsolved
// Simulated placeholder - the octane that clears it needs the cycle model's knock replay at this node (not inverted here).
void knock_finding(Findings& out, const LimitPolicy& pol, const SimulatedEngineDesc& sim, real rpm, real retard, const std::string& fuel_file) {
    if (!(retard > pol.knock_near_retard_deg)) return;
    const LimitSeverity s = retard >= pol.knock_limiting_retard_deg ? LimitSeverity::Limiting : LimitSeverity::Near;
    PartRef part{"fuel", fuel_file, "/knock/ron"};
    LimitFinding f = make_finding(LimitKind::Knock, s, rpm, retard, pol.knock_limiting_retard_deg, LimitUnit::CrankDegree, part);
    Remedy r;
    r.part = std::move(part);
    r.current = (!sim.fuels.empty() && sim.fuels.front().has_knock) ? sim.fuels.front().knock.ron : std::numeric_limits<real>::quiet_NaN();
    r.unit = LimitUnit::Dimensionless;
    r.cost = RemedyCost::Simulated;
    r.solved = false;
    f.remedy = std::move(r);
    out.push_back(std::move(f));
}

// One point's findings are emitted in producer order; the sweep sorts them by kind (stable), so within a kind the emission
// order is the documented one.
void sort_point_findings(Findings& f) {
    std::stable_sort(f.begin(), f.end(), [](const LimitFinding& a, const LimitFinding& b) { return a.kind < b.kind; });
}

// Concatenates the per-point findings in ascending rpm order (points of equal rpm share their first member's block) and
// fills DynoResult::point_limits. `per_point[i]` belongs to result.points[i] and is already sorted by kind.
void assemble_limits(DynoResult& result, std::vector<Findings>& per_point) {
    const std::size_t n = result.points.size();
    std::vector<std::size_t> order(n);
    for (std::size_t i = 0; i < n; ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return result.points[a].rpm < result.points[b].rpm; });
    result.limits.clear();
    result.point_limits.assign(n, LimitRange{});
    std::size_t k = 0;
    while (k < n) {
        const std::size_t head = order[k];
        const std::uint32_t first = static_cast<std::uint32_t>(result.limits.size());
        for (LimitFinding& f : per_point[head]) result.limits.push_back(std::move(f));
        const LimitRange range{first, static_cast<std::uint32_t>(result.limits.size()) - first};
        std::size_t j = k;
        while (j < n && result.points[order[j]].rpm == result.points[head].rpm) {
            result.point_limits[order[j]] = range;
            ++j;
        }
        k = j;
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Torque map: the WOT curve, instant.
DynoPoint torque_map_point(const EngineTorqueMap::Desc& d, real rpm, real throttle) {
    const real w_curve = d.wot_torque_nm_vs_rpm.evaluate(rpm);
    const real m = d.motoring_torque_nm_vs_rpm.evaluate(rpm);
    // G1: past the curves' data an armed roll-off scales the combustion span (EngineTorqueMap::pre_solve's own rule).
    const bool roll = d.rolloff.active(rpm);
    const real roll_f = roll ? d.rolloff.factor(rpm) : real(1.0);
    const real w = roll ? m + (w_curve - m) * roll_f : w_curve;
    // tau0 - c_k = (m + c_k) + theta*(w - m) - c_k, i.e. motoring plus theta of the combustion span. Written as
    // w - (1 - theta)*(w - m) so that theta == 1 returns the curve value w EXACTLY (m + 1*(w - m) can differ by an ulp).
    const real net = w - (real(1.0) - throttle) * (w - m);
    const real omega = rpm * kTwoPiRad / real(60.0);
    DynoPoint p;
    p.rpm = rpm;
    p.torque_nm = net;
    p.power_kw = net * omega / real(1000.0);
    p.lambda = p.spark_deg = p.p_intake_kpa = p.exhaust_k = kNaN;
    p.knock_retard_deg = kNaN;
    // Fuel at the same (rpm, theta_c) the runtime evaluates it: the fuel map at theta, or the crude constant-efficiency
    // model on the combustion torque theta*(w - m).
    if (d.has_fuel_map) {
        p.fuel_g_s = roll ? d.fuel_map.value(rpm, throttle) * roll_f : d.fuel_map.value(rpm, throttle);
    } else if (d.has_fuel_model) {
        p.fuel_g_s = std::max(real(0.0), throttle * (w - m) * omega) / (d.eta_i * d.fuel.lhv_j_kg) * real(1000.0);
    } else {
        p.fuel_g_s = kNaN;
    }
    p.bsfc_g_kwh = bsfc_g_kwh(p.fuel_g_s, p.power_kw);
    return p;
}

// ---------------------------------------------------------------------------------------------------------------
// Simulated engine without a turbo: SimulatedEngine::steady_point, instant.
DynoPoint steady_sim_point(const SimulatedEngineMaps& maps, real rpm, const DynoSweepSpec& spec, real& p_im_pa_out) {
    const SimulatedEngine::SteadyPoint sp = SimulatedEngine::steady_point(maps, rpm, spec.throttle, spec.ambient);
    const real omega = rpm * kTwoPiRad / real(60.0);
    DynoPoint p;
    p.rpm = rpm;
    p.torque_nm = sp.net_torque_nm;
    p.power_kw = sp.net_torque_nm * omega / real(1000.0);
    p.fuel_g_s = sp.fuel_g_s;
    p.bsfc_g_kwh = bsfc_g_kwh(p.fuel_g_s, p.power_kw);
    p.lambda = sp.lambda;
    p.spark_deg = sp.spark_deg;
    p.knock_retard_deg = sp.knock_retard_deg;
    p.p_intake_kpa = sp.p_im_pa / real(1000.0);
    p.exhaust_k = kNaN; // no exhaust thermal model without a turbo
    p_im_pa_out = sp.p_im_pa;
    return p;
}

// ---------------------------------------------------------------------------------------------------------------
// Simulated engine with a turbo: the real engine stepped with the crank held.

// Sums of the per-substep samples of ONE settle window (the last PreSolveResult kept whole).
struct WindowStats {
    long n = 0;
    real net_torque = 0.0, fuel_g_s = 0.0, boost_bar = 0.0, charge_boost_bar = 0.0, turbo_rpm = 0.0;
    real lambda = 0.0, spark_deg = 0.0, p_intake_pa = 0.0, exhaust_k = 0.0;
    real knock_retard_deg = 0.0; // P7
    // Engine garage P3b-1: window measurements for the limit findings (never fed back, no state of the engine).
    long gate_full_n = 0;                        // substeps with the wastegate at/above kGateFull
    real gate_sum = 0.0;                         // sum of the opening
    long target_n = 0;                           // substeps with a boost target (plate >= 10 %)
    real target_gauge_sum = 0.0, charge_gauge_sum = 0.0; // over those substeps, Pa gauge
    real rotor_max_rpm = 0.0;
    long surge_n = 0;                            // substeps with the engine's surge flag set
    real surge_min_ratio = kInf;                 // min over substeps of flow fraction / surge-line flow (pr > 1.05 only)
    real flow_frac_max = 0.0;                    // max pair flow / max_mass_flow_kg_s
    EdgeAcc edge;
    // Engine garage P3b-2: the nitrous window (all zero without a kit).
    real n2o_kg_s_sum = 0.0;                     // sum of the delivered N2O flow, kg/s
    real kit_fuel_g_s_sum = 0.0;                 // sum of the engine's booked kit fuel, g/s
    real lam_c_sum = 0.0;                        // sum of n2o_lambda_combined over the substeps the charge model ran
    long lam_c_n = 0;
    PreSolveResult last;

    // Constants of the point that the window samples are measured against.
    struct Probe {
        real ambient_pa = 0.0;
        real target_boost_pa = 0.0;
        real max_mass_flow_kg_s = 1.0;
        const SurgeLineDesc* surge = nullptr;
        real gate_full = 0.98; // LimitPolicy::gate_full_opening of the sweep (set by stepped_point)
        GridProbe grid;
    };

    void clear() { *this = WindowStats{}; }
    void add(const PreSolveResult& r, real c_k, real p_amb, const Probe& probe, real n2o_kg_s = 0.0) {
        ++n;
        n2o_kg_s_sum += n2o_kg_s;
        kit_fuel_g_s_sum += r.n2o_kit_fuel_g_s;
        if (r.n2o_lambda_combined > real(0.0)) {
            lam_c_sum += r.n2o_lambda_combined;
            ++lam_c_n;
        }
        gate_sum += r.wastegate_opening;
        if (r.wastegate_opening >= probe.gate_full) ++gate_full_n;
        if (r.throttle_position >= real(0.10)) {
            ++target_n;
            target_gauge_sum += probe.target_boost_pa * std::min(r.throttle_position, real(1.0));
            charge_gauge_sum += r.p_charge_pa - probe.ambient_pa;
        }
        rotor_max_rpm = std::max(rotor_max_rpm, r.turbo_rpm);
        const real flow_frac = r.compressor_flow_kg_s / probe.max_mass_flow_kg_s;
        flow_frac_max = std::max(flow_frac_max, flow_frac);
        if (r.surge_active != 0) ++surge_n;
        if (probe.surge != nullptr && r.compressor_pressure_ratio > real(1.05)) {
            const real line = probe.surge->flow_fraction_vs_pr.evaluate(r.compressor_pressure_ratio);
            if (line > real(0.0)) surge_min_ratio = std::min(surge_min_ratio, flow_frac / line);
        }
        edge.add(probe.grid, r.rpm, r.p_intake_pa);
        // tau0 excludes the engine's Coulomb row (Powertrain carries it as a friction row to ground with kinetic
        // capacity c_k, which opposes the turning crank), so the torque the crank delivers is tau0 - c_k - the
        // quantity bench_hyper_calibration reports as net_Nm.
        net_torque += r.tau0 - c_k;
        fuel_g_s += r.fuel_flow_g_s;
        boost_bar += r.boost_bar;
        charge_boost_bar += (r.p_charge_pa - p_amb) / real(100000.0);
        turbo_rpm += r.turbo_rpm;
        lambda += r.lambda;
        spark_deg += r.spark_advance_deg;
        knock_retard_deg += r.knock_retard_deg;
        p_intake_pa += r.p_intake_pa;
        exhaust_k += r.exhaust_port_k;
        last = r;
    }
    [[nodiscard]] real mean(real sum) const { return sum / real(n); }
};

// The turbo findings of one settled window (engine garage P3b-1). Reads the window maxima/fractions in `w` and the turbo's
// descriptor; appends in producer order (the sweep sorts them by kind afterwards).
// A Simulated remedy as the sweep attaches it: the part and its current value, the required values still NaN, unsolved
// (limit_remedies.h solve_remedy fills it).
Remedy simulated_placeholder(PartRef part, real current, LimitUnit unit) {
    Remedy r;
    r.part = std::move(part);
    r.current = current;
    r.unit = unit;
    r.cost = RemedyCost::Simulated;
    r.solved = false;
    return r;
}

void turbo_window_findings(Findings& out, const SimulatedEngine& engine, const WindowStats& w, real rpm, const DynoSweepSpec& spec) {
    const LimitPolicy& pol = resolve_limit_policy(spec.policy);
    const DynoSweepSpec::PartFiles& files = spec.part_files;
    const TurboPair* tp = engine.turbo();
    if (tp == nullptr || w.n == 0) return;
    const TurboPairDesc& td = tp->desc;
    const real nf = real(w.n);
    const real mean_open = w.gate_sum / nf;
    const real sat_fraction = real(w.gate_full_n) / nf;
    // Wastegate area in the hardware FILE's terms (per turbo): the descriptor holds the installation total.
    const real wg_file_area = td.wastegate_area() / real(std::max(1, spec.turbo_count));
    const auto wg_placeholder = [&] {
        return simulated_placeholder(PartRef{"turbo", files.turbo, "/wastegate_area_m2"}, wg_file_area, LimitUnit::SquareMetre);
    };

    // Boost against the PID target (target x plate); both are means over the substeps that had a target.
    bool have_target = w.target_n > 0;
    real charge_gauge = 0.0, target_gauge = 0.0, over = 0.0;
    if (have_target) {
        charge_gauge = w.charge_gauge_sum / real(w.target_n);
        target_gauge = w.target_gauge_sum / real(w.target_n);
        over = (charge_gauge - target_gauge) / td.target_boost_pa;
    }

    // WastegateSaturated.
    {
        const PartRef part{"turbo", files.turbo, "/wastegate_area_m2"};
        if (sat_fraction >= pol.gate_saturated_window_fraction) {
            const LimitSeverity s = (have_target && over > pol.boost_band_near) ? LimitSeverity::Exceeded : LimitSeverity::Limiting;
            out.push_back(make_finding(LimitKind::WastegateSaturated, s, rpm, mean_open, pol.gate_full_opening, LimitUnit::Fraction, part));
            out.back().remedy = wg_placeholder();
        } else if (mean_open >= pol.gate_near_opening) {
            out.push_back(make_finding(LimitKind::WastegateSaturated, LimitSeverity::Near, rpm, mean_open, pol.gate_full_opening, LimitUnit::Fraction, part));
            out.back().remedy = wg_placeholder();
        }
    }
    // BoostAboveTarget / BoostBelowTarget.
    if (have_target) {
        if (over > pol.boost_band_near) {
            const LimitSeverity s = over > pol.boost_band_exceeded ? LimitSeverity::Exceeded : LimitSeverity::Near;
            out.push_back(make_finding(LimitKind::BoostAboveTarget, s, rpm, charge_gauge, target_gauge, LimitUnit::Pascal,
                                       PartRef{"turbo", files.turbo, "/wastegate_area_m2"}));
            out.back().remedy = wg_placeholder();
        } else if (-over > pol.boost_band_near) {
            const LimitSeverity s = -over > pol.boost_band_exceeded ? LimitSeverity::Exceeded : LimitSeverity::Near;
            out.push_back(make_finding(LimitKind::BoostBelowTarget, s, rpm, charge_gauge, target_gauge, LimitUnit::Pascal,
                                       PartRef{"turbo", files.turbo, ""}));
        }
    }
    // TurboRotorCap.
    {
        const real edge_rpm = tp->speed_edge_rpm();
        const real f = w.rotor_max_rpm / edge_rpm;
        LimitSeverity s = LimitSeverity::Near;
        bool hit = true;
        if (f > real(1.0)) s = LimitSeverity::Exceeded;
        else if (f >= pol.rotor_limiting_fraction) s = LimitSeverity::Limiting;
        else if (f >= pol.rotor_near_fraction) s = LimitSeverity::Near;
        else hit = false;
        if (hit) {
            out.push_back(make_finding(LimitKind::TurboRotorCap, s, rpm, w.rotor_max_rpm, edge_rpm, LimitUnit::RevPerMinute,
                                       PartRef{"turbo", files.turbo, ""}));
            out.back().remedy = wg_placeholder(); // solve_remedy tries the wastegate first
        }
    }
    // CompressorSurge: only with a surge line in the hardware file (no line, no flag, no margin).
    if (td.surge_line.has_value()) {
        const PartRef part{"turbo", files.turbo, "/surge_line"};
        if (real(w.surge_n) / nf >= pol.surge_min_window_fraction) {
            out.push_back(make_finding(LimitKind::CompressorSurge, LimitSeverity::Exceeded, rpm, w.surge_min_ratio, real(1.0),
                                       LimitUnit::Dimensionless, part));
        } else if (w.surge_min_ratio < pol.surge_near_flow_ratio) {
            out.push_back(make_finding(LimitKind::CompressorSurge, LimitSeverity::Near, rpm, w.surge_min_ratio, real(1.0),
                                       LimitUnit::Dimensionless, part));
        }
    }
    // CompressorChoke: the pair flow against the compressor's flow capacity (max_mass_flow_kg_s). The shipped maps carry no
    // per-speed choke line, so this is a capacity margin, not a map-edge choke (limit_thresholds.h).
    {
        const real f = w.flow_frac_max;
        const PartRef part{"turbo", files.turbo, "/max_mass_flow_kg_s"};
        if (f >= pol.choke_limiting_flow_fraction) {
            out.push_back(make_finding(LimitKind::CompressorChoke, LimitSeverity::Limiting, rpm, f, pol.choke_limiting_flow_fraction, LimitUnit::Fraction, part));
        } else if (f >= pol.choke_near_flow_fraction) {
            out.push_back(make_finding(LimitKind::CompressorChoke, LimitSeverity::Near, rpm, f, pol.choke_limiting_flow_fraction, LimitUnit::Fraction, part));
        }
    }
}

// KitFuelLean (P3b-2) from one settled window: the mean combined lambda over the substeps the charge model ran (needs the kit to
// have sprayed for at least half of the window). The Closed remedy sizes the kit FUEL jet from the window's own flows: the
// O2-equivalent air demand is lambda_c * (ECU + kit fuel), so the kit fuel that gives a lambda L is
// kit + total * (lambda_c / L - 1), with the ECU's own fuel taken as unchanged; the jet flow is Cd * A * sqrt(2 rho dp) at the
// operating point's dp, so the diameter scales with sqrt(flow). required_min reaches kKitLambdaNear (the finding clears),
// required_margin the target lambda. The first-order assumption (ECU fuel and air independent of the kit jet) is verified by
// re-sweeping in the tests.
void kit_lean_finding(Findings& out, const WindowStats& w, real rpm, const DynoSweepSpec& spec) {
    const LimitPolicy& pol = resolve_limit_policy(spec.policy);
    if (!spec.nitrous.has_value() || w.n == 0 || w.lam_c_n * 2 < w.n) return;
    const real lam = w.lam_c_sum / real(w.lam_c_n);
    LimitSeverity s = LimitSeverity::Near;
    if (lam >= pol.kit_lambda_exceeded) s = LimitSeverity::Exceeded;
    else if (lam >= pol.kit_lambda_limiting) s = LimitSeverity::Limiting;
    else if (lam >= pol.kit_lambda_near) s = LimitSeverity::Near;
    else return;
    const NitrousKitDesc& kit = spec.nitrous->kit;
    LimitFinding f = make_finding(LimitKind::KitFuelLean, s, rpm, lam, pol.kit_lambda_near, LimitUnit::Dimensionless,
                                  PartRef{"nitrous_kit", spec.part_files.nitrous_kit, "/fuel_jet_diameter_mm"});
    const real total = w.fuel_g_s / real(w.n);
    const real kit_fuel = w.kit_fuel_g_s_sum / real(w.n);
    if (kit_fuel > real(0.0) && total > kit_fuel) {
        const auto jet_for = [&](real lambda_goal) {
            const real kit_needed = kit_fuel + total * (lam / lambda_goal - real(1.0));
            return kit.fuel_jet_diameter_mm * std::sqrt(std::max(kit_needed, kit_fuel) / kit_fuel);
        };
        Remedy r;
        r.part = f.part;
        r.current = kit.fuel_jet_diameter_mm;
        r.required_min = jet_for(pol.kit_lambda_near);
        r.required_margin = jet_for(pol.kit_lambda_target);
        r.unit = LimitUnit::Millimetre;
        r.cost = RemedyCost::Closed;
        r.solved = true;
        f.remedy = std::move(r);
    }
    out.push_back(std::move(f));
}

// THE per-point settled-window evaluation: everything a point reports is derived from one window of samples (means in
// `w`, the final substep's PreSolveResult in `w.last`) and the engine instance that produced them. The limit findings
// (P3b-1) are produced here from the same window (turbo_window_findings) next to the DynoPoint this returns.
DynoPoint settled_window_point(const SimulatedEngine& engine, const WindowStats& w, real rpm, real omega, bool converged,
                               real settle_s, const DynoSweepSpec& spec, const GridProbe& grid, Findings& findings) {
    turbo_window_findings(findings, engine, w, rpm, spec);
    kit_lean_finding(findings, w, rpm, spec);
    edge_findings(findings, resolve_limit_policy(spec.policy), rpm, grid, w.edge, spec.part_files.engine);
    DynoPoint p;
    p.rpm = rpm;
    p.torque_nm = w.mean(w.net_torque);
    p.power_kw = p.torque_nm * omega / real(1000.0);
    p.boost_bar = w.mean(w.boost_bar);
    p.charge_boost_bar = w.mean(w.charge_boost_bar);
    p.turbo_rpm = w.mean(w.turbo_rpm);
    p.fuel_g_s = w.mean(w.fuel_g_s);
    p.bsfc_g_kwh = bsfc_g_kwh(p.fuel_g_s, p.power_kw);
    p.lambda = w.mean(w.lambda);
    p.spark_deg = w.mean(w.spark_deg);
    p.knock_retard_deg = w.mean(w.knock_retard_deg);
    p.p_intake_kpa = w.mean(w.p_intake_pa) / real(1000.0);
    p.exhaust_k = w.mean(w.exhaust_k);
    if (spec.nitrous.has_value()) {
        p.n2o_g_s = w.n2o_kg_s_sum / real(w.n) * real(1000.0);
        p.kit_fuel_g_s = w.kit_fuel_g_s_sum / real(w.n);
        if (w.lam_c_n > 0) p.lambda_combined = w.lam_c_sum / real(w.lam_c_n);
    }
    const bool finite = std::isfinite(p.torque_nm) && std::isfinite(p.boost_bar) && std::isfinite(p.turbo_rpm);
    p.converged = converged && finite;
    p.settle_s = settle_s;
    return p;
}

// The kit as the dyno runs it (engine garage P3b-2): the controller's progressive ramp is skipped (full command from the first
// substep - "sprayed at full controller command") and its duty-time safety is off (a point may run up to max_settle_s; the
// bottle duration is a finding of its own). The bottle-pressure switch, the rpm/throttle windows and the safety latch of the
// controller stay as authored. Gear and road-speed conditions do not exist on a dyno (see engine.md).
NitrousKitDesc dyno_kit(const NitrousKitDesc& k) {
    NitrousKitDesc d = k;
    d.controller.progressive = NitrousProgressiveDesc{1.0, 0.0, 0.0};
    d.controller.safety.max_active_s = real(0.0);
    return d;
}

// The stepped kind: the real runtime engine with the crank held. With a turbo (TurboPair, wastegate PID, charge lag) or
// without one when a nitrous kit is present (then the engine is the same SimulatedEngine, the turbo findings are skipped).
// Pedal plan P1: envelope = the full-load envelope a part-throttle point runs against (nullptr at WOT and while the
// envelope itself is built); end_state (optional) receives the turbo's charge/exhaust state at the end of the run;
// pre_spin_rpm > 0 first runs kPreSpinS of WOT with the crank at that speed (a spun rotor and a full charge), ramps the
// crank down to rpm at kPreSpinRampRpmPerS, then holds rpm as usual - the spooled state a slow creep would take minutes
// to reach from rest.
struct EndState {
    real charge_pa = 0.0, charge_k = 0.0, exhaust_pa = 0.0;
};
constexpr real kPreSpinS = 5.0;                // [AS] WOT at the pre-spin speed (hyper 3rd-gear spool to 90 % in 1.37 s, B0)
constexpr real kPreSpinAboveRpm = 1500.0;      // [AS] pre-spin speed = min(top of the grid, rpm + this)
constexpr real kPreSpinRampRpmPerS = 250.0;    // [AS] crank ramp down to the held rpm (hyper 1950 rpm: 1000 rpm/s lands
                                               // below the closed gate's holding point, 250 rpm/s holds 1.8 bar, measured)
DynoPoint stepped_point(const SimulatedEngineDesc& desc, const std::shared_ptr<const SimulatedEngineMaps>& maps, real rpm,
                        const DynoSweepSpec& spec, Findings& findings,
                        const std::shared_ptr<const FullLoadEnvelope>& envelope = nullptr, EndState* end_state = nullptr,
                        real pre_spin_rpm = 0.0) {
    SimulatedEngine engine(desc, maps, envelope);
    if (maps->cam_high && spec.cam_profile >= 0) engine.set_cam_blend_override(spec.cam_profile > 0 ? real(1.0) : real(0.0)); // P9
    // The nitrous system is the car's own (nitrous_system.h): the same flow arithmetic feeds Sensors::nitrous. The bottle is
    // never committed, so it stays at its descriptor fill and its pressure is held for the whole point.
    std::unique_ptr<NitrousSystem> nsys;
    if (spec.nitrous.has_value()) {
        nsys = std::make_unique<NitrousSystem>(spec.nitrous->bottle, dyno_kit(spec.nitrous->kit), desc.fuels.front().density_kg_m3, -1, -1, -1);
    }
    const real heater_channel = (spec.nitrous.has_value() && spec.nitrous->bottle.has_heater) ? spec.nitrous->bottle.heater.default_on : real(0.0);
    engine.reset(EngineState::Running); // seeded Running, ignition on, no starter (a held crank never starts the engine)

    const real h = real(1.0) / kStepHz;
    const real omega = rpm * kTwoPiRad / real(60.0);
    const long window_steps = std::max(1L, static_cast<long>(std::floor(spec.settle_window_s * kStepHz + real(0.5))));
    const long max_steps = std::max(window_steps, static_cast<long>(std::floor(spec.max_settle_s * kStepHz + real(0.5))));
    const long windows = max_steps / window_steps; // whole windows; the run ends at a window boundary
    const real window_s = real(window_steps) * h;

    Sensors s;
    s.omega = omega;
    s.ignition = real(1.0);
    s.ambient = spec.ambient;
    s.anti_lag_channel = spec.anti_lag ? real(1.0) : real(0.0);
    const ThrottleCommand throttle{spec.throttle, real(0.0), real(1.0)};
    const real c_k = engine.maps().c_k;

    WindowStats::Probe probe;
    probe.ambient_pa = spec.ambient.pressure_pa;
    probe.gate_full = resolve_limit_policy(spec.policy).gate_full_opening;
    if (engine.turbo() != nullptr) {
        const TurboPairDesc& td = engine.turbo()->desc;
        probe.target_boost_pa = td.target_boost_pa;
        probe.max_mass_flow_kg_s = td.max_mass_flow_kg_s;
        probe.surge = td.surge_line.has_value() ? &*td.surge_line : nullptr;
    }
    const GridProbe grid = make_grid_probe(desc, engine.maps(), spec.ambient.pressure_pa);
    probe.grid = grid;

    if (pre_spin_rpm > real(0.0)) {
        Sensors sp = s;
        sp.omega = pre_spin_rpm * kTwoPiRad / real(60.0);
        const long n_pre = static_cast<long>(std::floor(kPreSpinS * kStepHz + real(0.5)));
        for (long i = 0; i < n_pre; ++i) {
            (void)engine.pre_solve(sp, ThrottleCommand{real(1.0), real(0.0), real(1.0)}, TcuRequest{}, h);
            (void)engine.post_solve(sp.omega, h);
        }
        // Then down to rpm at kPreSpinRampRpmPerS, still WOT, so the wastegate PID follows and the charge stays at its
        // target wherever the closed gate can hold it (an instant drop leaves the high-rpm gate opening while the exhaust
        // energy collapses, and the rotor falls back onto the slow creep a cold start shows - hyper 2000 rpm, measured).
        const long n_ramp = static_cast<long>(std::floor((pre_spin_rpm - rpm) / kPreSpinRampRpmPerS * kStepHz + real(0.5)));
        for (long i = 1; i <= n_ramp; ++i) {
            const real r_i = pre_spin_rpm + (rpm - pre_spin_rpm) * real(i) / real(n_ramp);
            sp.omega = r_i * kTwoPiRad / real(60.0);
            (void)engine.pre_solve(sp, ThrottleCommand{real(1.0), real(0.0), real(1.0)}, TcuRequest{}, h);
            (void)engine.post_solve(sp.omega, h);
        }
    }

    WindowStats w;
    // Convergence (measured on the hyper, see engine.md "Dyno sweep"): a window pair passes when the boost mean moved
    // less than settle_boost_bar_per_s AND the net-torque mean less than settle_torque_rel_per_s (relative); the point
    // settles after kPassingPairs consecutive passing pairs. One pair alone exits at the peak of a wastegate overshoot
    // (boost rate crosses zero there) and a boost rate alone cannot see a slowly spooling low-boost rotor.
    constexpr int kPassingPairs = 2;
    real prev_boost = kNaN, prev_torque = kNaN;
    int passing = 0;
    bool converged = false;
    long k = 0;
    for (; k < windows; ++k) {
        w.clear();
        for (long i = 0; i < window_steps; ++i) {
            if (nsys) {
                // What Powertrain::pre_solve_components hands NitrousSystem::step, for a wheel-less rig: armed, the throttle
                // command as the pedal, no gearbox (no gear condition) and no road speed (the speed condition is met).
                NitrousInputs ni;
                ni.h = h;
                ni.arm = real(1.0);
                ni.heater = heater_channel;
                ni.purge = real(0.0);
                ni.throttle_command = throttle.command;
                ni.omega = omega;
                ni.has_gearbox = false;
                ni.gear = 0;
                ni.speed_mps = std::numeric_limits<real>::max();
                ni.engine_running = true;
                ni.ignition_cut = false;
                ni.p_intake_pa = engine.intake_pressure_pa();
                ni.p_ambient_pa = spec.ambient.pressure_pa;
                nsys->step(ni);
                s.nitrous = nsys->engine_input();
            }
            const PreSolveResult r = engine.pre_solve(s, throttle, TcuRequest{}, h);
            if (nsys) nsys->observe(r.limiter_active || r.dfco_active, r.fuel_supply_fraction < real(1.0));
            (void)engine.post_solve(omega, h);
            w.add(r, c_k, spec.ambient.pressure_pa, probe, s.nitrous.mdot_n_kg_s);
        }
        const real boost = w.mean(w.boost_bar);
        const real torque = w.mean(w.net_torque);
        if (k > 0) {
            const bool boost_ok = std::abs(boost - prev_boost) / window_s < spec.settle_boost_bar_per_s;
            const bool torque_ok = std::abs(torque - prev_torque) / window_s <
                                   spec.settle_torque_rel_per_s * std::max(std::abs(torque), real(1.0));
            passing = (boost_ok && torque_ok) ? passing + 1 : 0;
            if (passing >= kPassingPairs) {
                converged = true;
                ++k;
                break;
            }
        }
        prev_boost = boost;
        prev_torque = torque;
    }
    if (end_state != nullptr && engine.turbo() != nullptr) {
        end_state->charge_pa = engine.turbo()->charge_pa;
        end_state->charge_k = engine.turbo()->charge_k;
        end_state->exhaust_pa = engine.turbo()->exhaust_pa;
    }
    DynoPoint p = settled_window_point(engine, w, rpm, omega, converged, real(k * window_steps) * h, spec, grid, findings);
    p.cam_profile = engine.cam_high_requested() ? 1 : 0; // engine garage P9
    p.lift_u = engine.lift_u();                           // engine garage P10 (1 without a family)
    return p;
}

// ---------------------------------------------------------------------------------------------------------------
// Bottle findings (engine garage P3b-2): static per sweep, tagged with the rpm of the sweep's largest N2O flow (the shortest
// bottle life). Both read only the points' window means and the descriptors.
void bottle_findings(std::vector<Findings>& per_point, const DynoResult& result, const DynoSweepSpec& spec, const SimulatedEngineDesc& sim) {
    const LimitPolicy& pol = resolve_limit_policy(spec.policy);
    if (!spec.nitrous.has_value()) return;
    std::size_t worst = result.points.size();
    for (std::size_t i = 0; i < result.points.size(); ++i) {
        if (result.points[i].n2o_g_s > real(0.0) && (worst == result.points.size() || result.points[i].n2o_g_s > result.points[worst].n2o_g_s)) worst = i;
    }
    if (worst == result.points.size()) return; // the controller never sprayed in this sweep
    const DynoPoint& pt = result.points[worst];
    const N2oBottleDesc& bottle = spec.nitrous->bottle;
    const NitrousKitDesc& kit = spec.nitrous->kit;
    const real flow_kg_s = pt.n2o_g_s * real(0.001);
    Findings& out = per_point[worst];

    // N2OBottleDuration: the bottle content over the flow at its worst point.
    if (bottle.initial_kg > real(0.0)) {
        const real duration = bottle.initial_kg / flow_kg_s;
        const real target = pol.bottle_duration_target_s;
        const LimitSeverity s = duration < target ? LimitSeverity::Limiting : LimitSeverity::Near;
        if (duration < target * pol.bottle_duration_near_factor) {
            LimitFinding f = make_finding(LimitKind::N2OBottleDuration, s, pt.rpm, duration, target, LimitUnit::Second,
                                          PartRef{"n2o_bottle", spec.part_files.n2o_bottle, "/capacity_kg"});
            Remedy r;
            r.part = f.part;
            r.current = bottle.initial_kg;
            r.required_min = target * flow_kg_s;
            r.required_margin = target * flow_kg_s * pol.bottle_reserve_factor;
            r.unit = LimitUnit::Kilogram;
            r.cost = RemedyCost::Closed;
            r.solved = true;
            f.remedy = std::move(r);
            out.push_back(std::move(f));
        }
    }

    // N2OBottlePressureDrop: only a bottle whose pressure falls with the fill (vs_fill; a constant or temperature-set pressure
    // does not drop, and a heater holds its set point). The flow at the END of a target-length pass over the flow at the
    // start, at the same jet and manifold pressure (the flow model's flux ratio); the fill at the end is the start fill less
    // target x the start flow.
    const bool heater_on = bottle.has_heater && bottle.heater.default_on >= real(0.5);
    if (bottle.pressure.mode == N2oPressureMode::VsFill && !heater_on && bottle.capacity_kg > real(0.0)) {
        const NitrousSystem sys(bottle, kit, sim.fuels.front().density_kg_m3, -1, -1, -1);
        const real target = pol.bottle_duration_target_s;
        const real fill_end = std::max(real(0.0), bottle.initial_kg - target * flow_kg_s);
        const real p0 = sys.bottle_pressure_pa(bottle.initial_kg, false);
        const real p1 = sys.bottle_pressure_pa(fill_end, false);
        const real p_im = std::isfinite(pt.p_intake_kpa) ? pt.p_intake_kpa * real(1000.0) : spec.ambient.pressure_pa;
        const auto flux = [&](real pb) { return kit.flow_model == NitrousFlowModel::Hem ? sys.tables().g_hem(pb) : sys.tables().g_dyer(pb, p_im); };
        const real g0 = flux(p0);
        if (g0 > real(0.0)) {
            const real retention = flux(p1) / g0;
            const real gauge_end_bar = (p1 - kIsaAirPressurePa) * real(1.0e-5);
            LimitSeverity s = LimitSeverity::Near;
            bool hit = true;
            if (kit.controller.safety.min_bottle_gauge_bar > real(0.0) && gauge_end_bar < kit.controller.safety.min_bottle_gauge_bar) s = LimitSeverity::Exceeded;
            else if (retention < pol.bottle_flow_retention_limiting) s = LimitSeverity::Limiting;
            else if (retention < pol.bottle_flow_retention_near) s = LimitSeverity::Near;
            else hit = false;
            if (hit) {
                out.push_back(make_finding(LimitKind::N2OBottlePressureDrop, s, pt.rpm, retention, pol.bottle_flow_retention_near,
                                           LimitUnit::Fraction, PartRef{"n2o_bottle", spec.part_files.n2o_bottle, "/pressure"}));
            }
        }
    }
    sort_point_findings(out);
}

struct Shared {
    const std::atomic<bool>* cancel = nullptr;
    std::atomic<bool> skipped{false}; // some point observed the cancel flag and did not run
};

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// Pedal plan P1 (engine_simulated.h FullLoadEnvelope): the dyno-settled WOT curve of a turbo engine, memoised.
namespace {

struct EnvelopeHash {
    std::uint64_t h = 14695981039346656037ULL;
    template <class T> void pod(const T& v) {
        const auto* b = reinterpret_cast<const unsigned char*>(&v);
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    }
    void opt(const std::optional<real>& v) {
        pod(v.has_value());
        if (v) pod(*v);
    }
};

// Every input the WOT run reads besides the maps (whose content hash covers the model, limiter and ECU tables).
std::uint64_t envelope_key(const SimulatedEngineDesc& d, const SimulatedEngineMaps& m) {
    EnvelopeHash f;
    f.pod(m.content_hash);
    const TurboPairDesc& t = *d.turbo_pair;
    f.pod(t.maps ? t.maps->content_hash : std::uint64_t{0});
    f.pod(t.reference_head_pa); f.pod(t.target_boost_pa); f.pod(t.rotor_inertia_kgm2); f.pod(t.reference_rpm);
    f.pod(t.max_mass_flow_kg_s); f.pod(t.compressor_efficiency); f.pod(t.turbine_efficiency); f.pod(t.turbine_area_m2);
    f.pod(t.compressible_exhaust); f.opt(t.wastegate_area_m2); f.pod(t.intercooler_effectiveness);
    f.pod(t.shaft_loss_w_at_reference); f.pod(t.wastegate_kp); f.pod(t.wastegate_ki_per_s); f.pod(t.wastegate_kd_s);
    f.pod(t.wastegate_actuator_tau_s); f.pod(t.intercooler_dp_pa_at_max_flow);
    f.pod(t.blow_off_valve.has_value());
    if (t.blow_off_valve) {
        const BlowOffValveDesc& b = *t.blow_off_valve;
        f.pod(b.vent); f.pod(b.crack_dp_pa); f.pod(b.full_open_dp_pa); f.pod(b.effective_area_m2); f.pod(b.tau_s);
    }
    f.pod(t.in_cylinder_wall_heat_fraction); f.pod(t.exhaust_wall_heat_capacity_j_k);
    f.pod(t.exhaust_wall_gas_conductance_w_k); f.pod(t.exhaust_wall_ambient_conductance_w_k); f.pod(t.exhaust_wall_initial_k);
    f.pod(d.throttle_actuator_tau_s); f.pod(d.idle_rpm); f.pod(d.limiter.rpm); f.pod(d.limiter.type); f.pod(d.limiter.band_rpm);
    f.pod(d.limiter.hysteresis_rpm);
    return f.h;
}

std::mutex g_envelope_mutex;
std::vector<std::pair<std::uint64_t, std::shared_ptr<const FullLoadEnvelope>>> g_envelopes; // FIFO, kEnvelopeMemo entries
constexpr std::size_t kEnvelopeMemo = 32; // [AS] distinct engines/turbo configurations per process

} // namespace

std::shared_ptr<const FullLoadEnvelope> full_load_envelope(const SimulatedEngineDesc& desc,
                                                           const std::shared_ptr<const SimulatedEngineMaps>& maps) {
    if (!maps || !desc.turbo_pair || desc.throttle_input != ThrottleInput::Electronic || desc.maps_pending ||
        desc.architecture != EngineArchitecture::Piston4Stroke || !desc.turbo_pair->maps) {
        return nullptr;
    }
    const std::uint64_t key = envelope_key(desc, *maps);
    {
        std::lock_guard<std::mutex> lock(g_envelope_mutex);
        for (const auto& e : g_envelopes) {
            if (e.first == key) return e.second;
        }
    }
    // WOT, ISA, crank held, run twice per rpm: rotor from rest (SimulatedEngine::reset) and pre-spun (kPreSpinS of WOT at
    // min(top, rpm + kPreSpinAboveRpm), then a crank ramp down); the higher settled torque and its charge/exhaust state are kept (coordinator decision
    // 2026-10-13: F is the reachable state, so full pedal makes F - no step at c = 1 where a held point has two branches;
    // hyper 1950 rpm: cold ~465 N*m creeping for minutes vs 1169 N*m at 1.8 bar spun). idle .. just below the limiter (the fuel-cut latch or the soft cut's band),
    // the dyno's 250 rpm steps; end-of-window values where a point does not settle (15 s).
    const real band = desc.limiter.type == LimiterType::SoftCut ? desc.limiter.band_rpm : real(1.0);
    const real top = std::max(desc.idle_rpm + real(1.0), desc.limiter.rpm - band);
    const std::vector<real> rpms = default_rpm_grid(desc.idle_rpm, top);
    DynoSweepSpec spec;
    spec.throttle = real(1.0);
    auto env = std::make_shared<FullLoadEnvelope>();
    env->key = key;
    for (real rpm : rpms) {
        Findings ignored;
        EndState st_cold, st_spun;
        const DynoPoint p_cold = stepped_point(desc, maps, rpm, spec, ignored, nullptr, &st_cold);
        const DynoPoint p_spun = stepped_point(desc, maps, rpm, spec, ignored, nullptr, &st_spun, std::min(top, rpm + kPreSpinAboveRpm));
        const bool spun = p_spun.torque_nm > p_cold.torque_nm;
        const DynoPoint& p = spun ? p_spun : p_cold;
        const EndState& st = spun ? st_spun : st_cold;
        const real amb = spec.ambient.pressure_pa;
        const real m_ref = SimulatedEngine::boosted_model_torque(*maps, rpm, st.charge_pa, st.charge_k, st.exhaust_pa, real(1.0),
                                                                 real(2000.0));
        env->torque_nm.x.push_back(rpm);
        env->torque_nm.y.push_back(p.torque_nm);
        env->pressure_ratio.x.push_back(rpm);
        env->pressure_ratio.y.push_back(st.charge_pa / amb);
        env->charge_k.x.push_back(rpm);
        env->charge_k.y.push_back(st.charge_k);
        env->exhaust_ratio.x.push_back(rpm);
        env->exhaust_ratio.y.push_back(st.exhaust_pa / amb);
        env->model_offset_nm.x.push_back(rpm);
        env->model_offset_nm.y.push_back(p.torque_nm - m_ref);
    }
    std::shared_ptr<const FullLoadEnvelope> out = std::move(env);
    std::lock_guard<std::mutex> lock(g_envelope_mutex);
    for (const auto& e : g_envelopes) {
        if (e.first == key) return e.second; // another thread built it meanwhile: one object per key
    }
    if (g_envelopes.size() >= kEnvelopeMemo) g_envelopes.erase(g_envelopes.begin());
    g_envelopes.emplace_back(key, out);
    return out;
}

void clear_full_load_envelope_memo() {
    std::lock_guard<std::mutex> lock(g_envelope_mutex);
    g_envelopes.clear();
}

DynoResult run_dyno_sweep(const ComponentParams& engine, EngineMapsPtr maps, const DynoSweepSpec& spec, jobs::JobSystem* pool,
                          const std::atomic<bool>* cancel) {
    validate_spec(spec);
    const LimitPolicy& pol = resolve_limit_policy(spec.policy);
    // Cancelled before it started: nothing built, no point run (MapGenerationControl::cancel semantics).
    if (cancelled(cancel)) throw GenerationCancelled();

    const ComponentParams resolved = with_engine_maps(engine, maps, "run_dyno_sweep");
    DynoResult result;

    if (const TorqueMapEngineDesc* tm = std::get_if<TorqueMapEngineDesc>(&resolved)) {
        if (spec.nitrous.has_value()) {
            throw std::invalid_argument("run_dyno_sweep: a nitrous kit needs a simulated engine (the torque-map engine ignores the kit)");
        }
        const EngineTorqueMap::Desc d = to_engine_desc(*tm);
        const std::vector<real> rpms = spec.rpm.empty() ? default_rpm_grid(d.idle_rpm, d.limiter.rpm) : spec.rpm;
        result.points.reserve(rpms.size());
        std::vector<Findings> per_point;
        per_point.reserve(rpms.size());
        for (real rpm : rpms) {
            if (cancelled(cancel)) throw GenerationCancelled();
            result.points.push_back(torque_map_point(d, rpm, spec.throttle));
            Findings f;
            limiter_findings(f, pol, tm->limiter, rpm, spec.part_files.engine);
            beyond_data_finding(f, d.rolloff, rpm, spec.part_files.engine);
            sort_point_findings(f);
            per_point.push_back(std::move(f));
        }
        assemble_limits(result, per_point);
        return result;
    }

    const SimulatedEngineDesc* sim = std::get_if<SimulatedEngineDesc>(&resolved);
    if (sim == nullptr) throw std::invalid_argument("run_dyno_sweep: the component is not an engine (torque map or simulated)");

    const std::vector<real> rpms = spec.rpm.empty() ? default_rpm_grid(sim->idle_rpm, sim->limiter.rpm) : spec.rpm;

    if (spec.nitrous.has_value() && sim->fuels.empty()) {
        throw std::invalid_argument("run_dyno_sweep: a nitrous kit needs an engine with a fuel (its kit-jet density)");
    }
    if (!sim->turbo_pair.has_value() && !spec.nitrous.has_value()) {
        // build_model_maps throws SimulatedEngine's "maps are pending" / rotary / geometry messages like the engine would.
        const std::shared_ptr<const SimulatedEngineMaps> m = SimulatedEngine::build_model_maps(*sim);
        result.points.reserve(rpms.size());
        std::vector<Findings> per_point;
        per_point.reserve(rpms.size());
        const GridProbe grid = make_grid_probe(*sim, *m, spec.ambient.pressure_pa);
        for (real rpm : rpms) {
            if (cancelled(cancel)) throw GenerationCancelled();
            real p_im_pa = 0.0;
            // Engine garage P9: the profile this point runs on (see DynoSweepSpec::cam_profile).
            int profile = 0;
            if (m->cam_high) {
                if (spec.cam_profile >= 0) {
                    profile = spec.cam_profile > 0 ? 1 : 0;
                } else if (rpm >= m->cam_switching.rpm_on) {
                    const SimulatedEngine::SteadyPoint lo = SimulatedEngine::steady_point(*m, rpm, spec.throttle, spec.ambient);
                    profile = lo.p_im_pa >= m->cam_switching.min_p_im_pa ? 1 : 0;
                }
            }
            // Engine garage P10: a held lift node (see DynoSweepSpec::lift_node); the default is the u = 1 node (*m itself).
            const SimulatedEngineMaps* plant = profile == 1 ? m->cam_high.get() : m.get();
            real lift_u = 1.0;
            if (m->has_lift_family() && spec.lift_node >= 0) {
                const std::size_t k = static_cast<std::size_t>(spec.lift_node);
                if (k >= m->lift_u.size()) {
                    throw std::invalid_argument("run_dyno_sweep: lift_node " + std::to_string(k) + " is out of range (the engine has " +
                                                std::to_string(m->lift_u.size()) + " lift nodes)");
                }
                if (k + 1 < m->lift_u.size()) plant = m->lift_nodes[k].get();
                lift_u = m->lift_u[k];
            }
            result.points.push_back(steady_sim_point(*plant, rpm, spec, p_im_pa));
            result.points.back().cam_profile = profile;
            result.points.back().lift_u = lift_u;
            Findings f;
            limiter_findings(f, pol, sim->limiter, rpm, spec.part_files.engine);
            piston_speed_finding(f, pol, *sim, rpm, spec.part_files.engine);
            knock_finding(f, pol, *sim, rpm, result.points.back().knock_retard_deg, spec.part_files.fuel);
            EdgeAcc e;
            e.add(grid, rpm, p_im_pa);
            edge_findings(f, pol, rpm, grid, e, spec.part_files.engine);
            beyond_data_finding(f, m->rolloff, rpm, spec.part_files.engine);
            sort_point_findings(f);
            per_point.push_back(std::move(f));
        }
        assemble_limits(result, per_point);
        return result;
    }

    // Turbo (or a kit): the limiter is a time-domain fuel-cut latch, not what the engine can deliver; moved above the swept range
    // so the limiter point itself is a normal WOT point (the instant kinds ignore it the same way). Only when needed:
    // a descriptor whose limiter already lies above every point is used as it is (bit-identical to the bench).
    SimulatedEngineDesc d = *sim;
    {
        const real top = *std::max_element(rpms.begin(), rpms.end());
        const real band = d.limiter.type == LimiterType::SoftCut ? d.limiter.band_rpm : real(0.0);
        if (!(d.limiter.rpm - band > top)) {
            // G1: the moved limiter must not arm a roll-off the real engine does not have (an engine whose limiter is the
            // top of its data); an armed one keeps its start and end (they do not depend on the limiter).
            if (!rpm_rolloff(d).armed()) d.rpm_rolloff_band_fraction = real(0.0);
            d.limiter.rpm = top + band + real(1.0);
        }
    }
    const std::shared_ptr<const SimulatedEngineMaps> m = SimulatedEngine::build_maps(d); // throws if maps_pending
    // Pedal plan P1: a part-throttle point runs the manual law against this sweep's own WOT envelope (built once, before the
    // jobs; WOT points need none).
    const std::shared_ptr<const FullLoadEnvelope> envelope = spec.throttle < real(1.0) ? full_load_envelope(d, m) : nullptr;

    result.points.assign(rpms.size(), DynoPoint{});
    std::vector<Findings> per_point(rpms.size());
    std::vector<std::exception_ptr> errors(rpms.size());
    Shared shared;
    shared.cancel = cancel;
    const auto run_one = [&](std::size_t i) {
        if (cancelled(shared.cancel)) {
            shared.skipped.store(true, std::memory_order_relaxed);
            return;
        }
        try {
            result.points[i] = stepped_point(d, m, rpms[i], spec, per_point[i], envelope);
            // The limiter findings use the ORIGINAL limiter: the sweep moved it out of the way only to hold WOT.
            limiter_findings(per_point[i], pol, sim->limiter, rpms[i], spec.part_files.engine);
            piston_speed_finding(per_point[i], pol, *sim, rpms[i], spec.part_files.engine);
            knock_finding(per_point[i], pol, *sim, rpms[i], result.points[i].knock_retard_deg, spec.part_files.fuel);
            beyond_data_finding(per_point[i], m->rolloff, rpms[i], spec.part_files.engine);
            sort_point_findings(per_point[i]);
        } catch (...) {
            errors[i] = std::current_exception();
        }
    };
    if (pool == nullptr) {
        for (std::size_t i = 0; i < rpms.size(); ++i) {
            run_one(i);
            if (shared.skipped.load(std::memory_order_relaxed)) break;
        }
    } else {
        pool->parallel_batches(rpms.size(), run_one);
    }
    if (shared.skipped.load(std::memory_order_relaxed)) throw GenerationCancelled();
    for (const std::exception_ptr& e : errors) {
        if (e) std::rethrow_exception(e);
    }
    bottle_findings(per_point, result, spec, *sim);
    assemble_limits(result, per_point);
    return result;
}

std::optional<DynoNitrous> nitrous_from_powertrain(const PowertrainDesc& pt) {
    const N2oBottleDesc* bottle = nullptr;
    const NitrousKitDesc* kit = nullptr;
    for (const ComponentDesc& c : pt.components) {
        if (bottle == nullptr) bottle = std::get_if<N2oBottleDesc>(&c.params);
        if (kit == nullptr) kit = std::get_if<NitrousKitDesc>(&c.params);
    }
    if (bottle == nullptr || kit == nullptr) return std::nullopt;
    return DynoNitrous{*bottle, *kit};
}

std::vector<LimitFinding> check_driveline_limits(const DynoResult& dyno, const PowertrainDesc& powertrain, const std::string& clutch_file,
                                                 const std::string& gearbox_file, const LimitPolicy* policy) {
    const LimitPolicy& pol = resolve_limit_policy(policy);
    std::vector<LimitFinding> out;
    // The peak net torque and where it occurs (the first such point on a tie).
    const DynoPoint* peak = nullptr;
    for (const DynoPoint& p : dyno.points) {
        if (std::isfinite(p.torque_nm) && (peak == nullptr || p.torque_nm > peak->torque_nm)) peak = &p;
    }
    if (peak == nullptr || !(peak->torque_nm > real(0.0))) return out;
    const real margin = pol.driveline_torque_margin;
    for (const ComponentDesc& c : powertrain.components) {
        const ClutchDesc* cl = std::get_if<ClutchDesc>(&c.params);
        if (cl == nullptr || !(cl->capacity_nm > real(0.0))) continue;
        const real static_capacity = cl->capacity_nm * cl->static_factor;
        LimitSeverity s{};
        if (!clutch_capacity_severity(peak->torque_nm, static_capacity, pol, s)) continue;
        LimitFinding f = make_finding(LimitKind::ClutchCapacity, s, peak->rpm, peak->torque_nm, static_capacity, LimitUnit::NewtonMetre,
                                      PartRef{"clutch", clutch_file, "/capacity_nm"});
        Remedy r;
        r.part = f.part;
        r.current = cl->capacity_nm;
        r.required_min = peak->torque_nm / cl->static_factor;
        r.required_margin = margin * peak->torque_nm / cl->static_factor;
        r.unit = LimitUnit::NewtonMetre;
        r.cost = RemedyCost::Closed;
        r.solved = true;
        f.remedy = std::move(r);
        out.push_back(std::move(f));
    }
    // Transmission plan T0: a gearbox's rated input torque against the same peak. Graded over the rating, not under it
    // (gearbox_rating.h); the Closed remedy names the rating field: required_min = the peak, required_margin = peak x 1.1.
    for (const ComponentDesc& c : powertrain.components) {
        const GearboxDesc* gb = std::get_if<GearboxDesc>(&c.params);
        if (gb == nullptr) continue;
        LimitSeverity s{};
        if (!gearbox_rating_severity(peak->torque_nm, gb->rated_input_torque_nm, pol, s)) continue;
        LimitFinding f = make_finding(LimitKind::GearboxInputTorque, s, peak->rpm, peak->torque_nm, gb->rated_input_torque_nm,
                                      LimitUnit::NewtonMetre, PartRef{"gearbox", gearbox_file, "/rated_input_torque_nm"});
        Remedy r;
        r.part = f.part;
        r.current = gb->rated_input_torque_nm;
        r.required_min = peak->torque_nm;
        r.required_margin = pol.gearbox_rating_remedy_margin * peak->torque_nm;
        r.unit = LimitUnit::NewtonMetre;
        r.cost = RemedyCost::Closed;
        r.solved = true;
        f.remedy = std::move(r);
        out.push_back(std::move(f));
    }
    std::stable_sort(out.begin(), out.end(), [](const LimitFinding& a, const LimitFinding& b) { return a.kind < b.kind; });
    return out;
}

} // namespace ps::drivetrain
