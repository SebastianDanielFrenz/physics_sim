// turbo_meanline.cpp - C++ port of tools/turbo_sim/meanline.py (see ps/drivetrain/turbo_meanline.h).
// Every function below names the Python function it ports; the arithmetic follows the Python line by line, in the
// same evaluation order, so the two differ only by the transcendental implementations (ps::math vs the platform libm).
#include "ps/drivetrain/turbo_meanline.h"

#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/turbo_stage_maps.h"
#include "ps/jobs/job_system.h"
#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <stdexcept>
#include <tuple>

namespace ps::drivetrain::meanline {
namespace {

constexpr real kPi = 3.141592653589793;  // Python math.pi
constexpr real kTau = 6.283185307179586; // Python math.tau

real radians(real deg) { return deg * (kPi / real(180)); } // CPython math.radians: x * (pi / 180)
real sq(real x) { return x * x; }


void validate_gas(const Gas& g) {
    if (!std::isfinite(g.r) || g.r <= 0 || !(real(1) < g.gamma && g.gamma <= real(1.67)))
        throw std::invalid_argument("invalid ideal-gas properties");
}

// meanline.py subsonic_velocity: (velocity, mach) on the subsonic branch, or false beyond sonic capacity.
bool subsonic_velocity(real mdot, real area, real pt, real tt, const Gas& gas, real& velocity, real& mach_out) {
    const real critical = ps::math::pow(real(2) / (gas.gamma + real(1)), gas.gamma / (gas.gamma - real(1)));
    const real capacity = area * nozzle_flux(pt, pt * critical, tt, gas);
    if (mdot > capacity * (real(1) + real(1e-10))) return false;
    real lo = 0, hi = 1;
    for (int i = 0; i < 55; ++i) {
        const real mach = (lo + hi) / 2;
        const real ts = tt / (real(1) + (gas.gamma - real(1)) / real(2) * mach * mach);
        const real ps_ = pt * ps::math::pow(ts / tt, gas.gamma / (gas.gamma - real(1)));
        const real v = mach * std::sqrt(gas.gamma * gas.r * ts);
        if (area * ps_ / (gas.r * ts) * v < mdot) lo = mach; else hi = mach;
    }
    const real mach = (lo + hi) / 2;
    const real ts = tt / (real(1) + (gas.gamma - real(1)) / real(2) * mach * mach);
    velocity = mach * std::sqrt(gas.gamma * gas.r * ts);
    mach_out = mach;
    return true;
}

void check_axes(const Sampling& s) {
    for (const auto* axis : {&s.corrected_rpm, &s.flow_kg_s, &s.expansion_ratio}) {
        if (axis->size() < 2) throw std::invalid_argument("finite strictly increasing axes with >=2 points required");
        for (std::size_t i = 0; i < axis->size(); ++i) {
            if (!std::isfinite((*axis)[i]) || (i > 0 && !((*axis)[i] > (*axis)[i - 1])))
                throw std::invalid_argument("finite strictly increasing axes with >=2 points required");
        }
    }
    if (s.corrected_rpm.front() < 0 || s.flow_kg_s.front() < 0 || s.expansion_ratio.front() < 1)
        throw std::invalid_argument("invalid map axis domain");
}

// Runs fn(i) for i in [0, n) on the pool (or inline). Each i writes only its own slot: order-independent.
template <typename Fn> void for_each_index(std::size_t n, ps::jobs::JobSystem* pool, Fn&& fn) {
    if (pool == nullptr) {
        for (std::size_t i = 0; i < n; ++i) fn(i);
        return;
    }
    pool->parallel_for(n, 16, [&fn](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) fn(i);
    });
}

// meanline.py Generator.interpolation_audit. Returns the report per stage and the cells (error, rpm, coordinate).
struct Cell { real error, n, q; };
struct AuditOut {
    StageAudit compressor, turbine;
    std::vector<Cell> compressor_cells, turbine_cells;
};

real turbine_exponent(const StageGeometry& g) { return (g.exhaust.gamma - real(1)) / g.exhaust.gamma; }

AuditOut interpolation_audit(const StageGeometry& g, const StageMapResult& r, ps::jobs::JobSystem* pool) {
    AuditOut out;
    const auto& speeds = r.axes.corrected_rpm;
    for (int stage = 0; stage < 2; ++stage) {
        const bool comp = stage == 0;
        const auto& axis = comp ? r.axes.flow_kg_s : r.axes.expansion_ratio;
        const std::size_t ns = speeds.size() - 1, na = axis.size() - 1;
        // Independent points at every cell centre, solved in parallel into their own slots.
        std::vector<real> nc(ns * na), qc(ns * na);
        for (std::size_t ix = 0; ix < ns; ++ix) {
            for (std::size_t iy = 0; iy < na; ++iy) {
                nc[ix * na + iy] = (speeds[ix] + speeds[ix + 1]) / 2;
                real q = (axis[iy] + axis[iy + 1]) / 2;
                if (!comp) {
                    const real e = turbine_exponent(g);
                    const real chi = (std::sqrt(real(1) - ps::math::pow(axis[iy], -e)) +
                                      std::sqrt(real(1) - ps::math::pow(axis[iy + 1], -e))) / 2;
                    q = ps::math::pow(real(1) - chi * chi, -real(1) / e);
                }
                qc[ix * na + iy] = q;
            }
        }
        std::vector<CompressorPoint> ca(comp ? ns * na : 0);
        std::vector<TurbinePoint> ta(comp ? 0 : ns * na);
        for_each_index(ns * na, pool, [&](std::size_t i) {
            if (comp) ca[i] = compressor_point(g, nc[i], qc[i]);
            else ta[i] = turbine_point(g, nc[i], qc[i]);
        });
        StageAudit& rep = comp ? out.compressor : out.turbine;
        auto& cells = comp ? out.compressor_cells : out.turbine_cells;
        for (std::size_t ix = 0; ix < ns; ++ix) {
            for (std::size_t iy = 0; iy < na; ++iy) {
                const std::size_t c = ix * na + iy;
                const std::size_t k[4] = {ix * axis.size() + iy, ix * axis.size() + iy + 1, (ix + 1) * axis.size() + iy,
                                          (ix + 1) * axis.size() + iy + 1};
                real error = 0;
                if (comp) {
                    bool valid = true;
                    for (std::size_t j : k) valid = valid && r.compressor[j].status == CompressorStatus::Ok;
                    const bool actual_ok = ca[c].status == CompressorStatus::Ok;
                    if (!valid || !actual_ok) {
                        rep.coverage_boundary_cells += int(valid != actual_ok);
                        continue;
                    }
                    const real sum_pr = real(0) + r.compressor[k[0]].pressure_ratio + r.compressor[k[1]].pressure_ratio +
                                        r.compressor[k[2]].pressure_ratio + r.compressor[k[3]].pressure_ratio;
                    const real sum_w = real(0) + r.compressor[k[0]].specific_work_j_kg + r.compressor[k[1]].specific_work_j_kg +
                                       r.compressor[k[2]].specific_work_j_kg + r.compressor[k[3]].specific_work_j_kg;
                    error = std::max(std::abs(ca[c].pressure_ratio - sum_pr / 4) / std::max(std::abs(ca[c].pressure_ratio), real(1)),
                                     std::abs(ca[c].specific_work_j_kg - sum_w / 4) /
                                         std::max(std::abs(ca[c].specific_work_j_kg), real(1000)));
                } else { // turbine points are always "ok"
                    const real sum_m = real(0) + r.turbine[k[0]].mass_flow_kg_s + r.turbine[k[1]].mass_flow_kg_s +
                                       r.turbine[k[2]].mass_flow_kg_s + r.turbine[k[3]].mass_flow_kg_s;
                    const real sum_t = real(0) + r.turbine[k[0]].specific_torque_nm_per_kg_s +
                                       r.turbine[k[1]].specific_torque_nm_per_kg_s + r.turbine[k[2]].specific_torque_nm_per_kg_s +
                                       r.turbine[k[3]].specific_torque_nm_per_kg_s;
                    error = std::max(std::abs(ta[c].mass_flow_kg_s - sum_m / 4) / std::max(std::abs(ta[c].mass_flow_kg_s), real(0.001)),
                                     std::abs(ta[c].specific_torque_nm_per_kg_s - sum_t / 4) /
                                         std::max(std::abs(ta[c].specific_torque_nm_per_kg_s), real(1)));
                }
                cells.push_back({error, nc[c], qc[c]});
                rep.cell_centers_checked += 1;
                rep.maximum_relative_error = std::max(rep.maximum_relative_error, error);
            }
        }
    }
    return out;
}

std::uint64_t fnv(std::uint64_t h, const void* data, std::size_t n) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ULL; }
    return h;
}
std::uint64_t fnv_real(std::uint64_t h, real v) {
    unsigned char b[sizeof(real)];
    std::memcpy(b, &v, sizeof(v));
    return fnv(h, b, sizeof(b)); // little-endian on every supported platform (x86-64, aarch64)
}
std::uint64_t fnv_str(std::uint64_t h, const std::string& s) {
    const std::uint64_t n = s.size();
    h = fnv(h, &n, sizeof(n));
    return fnv(h, s.data(), s.size());
}

} // namespace

const char* to_string(CompressorStatus s) {
    switch (s) {
    case CompressorStatus::Ok: return "ok";
    case CompressorStatus::InletChoke: return "inlet_choke";
    case CompressorStatus::ExitChokeOrNoPositiveHead: return "exit_choke_or_no_positive_head";
    case CompressorStatus::RelativeChoke: return "relative_choke";
    case CompressorStatus::TransonicUnmodelled: return "transonic_unmodelled";
    case CompressorStatus::NoPositivePressureRise: return "no_positive_pressure_rise";
    }
    return "unknown";
}

void validate(const StageGeometry& g) {
    const auto& c = g.compressor;
    const auto& t = g.turbine;
    if (!(0 < c.hub_mm && c.hub_mm < c.inducer_mm && c.inducer_mm < c.exducer_mm))
        throw std::invalid_argument("compressor requires hub < inducer < exducer");
    if (c.exit_width_mm <= 0 || c.blades < 3) throw std::invalid_argument("invalid compressor blade geometry");
    if (!(0 < c.blockage && c.blockage <= 1)) throw std::invalid_argument("blockage must be in (0,1]");
    if (!(0 < c.exit_angle_deg && c.exit_angle_deg <= 90) || !(0 < c.inlet_angle_deg && c.inlet_angle_deg < 90))
        throw std::invalid_argument("invalid compressor blade angle");
    if (std::min({c.passage_loss, c.incidence_loss, c.diffuser_loss}) < 0)
        throw std::invalid_argument("loss coefficients must be nonnegative");
    if (t.diameter_mm <= 0 || t.effective_throat_mm2 <= 0) throw std::invalid_argument("positive turbine dimensions required");
    if (!(0 < t.nozzle_efficiency && t.nozzle_efficiency <= 1) || !(0 <= t.rotor_velocity_retention && t.rotor_velocity_retention <= 1))
        throw std::invalid_argument("invalid turbine losses");
    if (!(0 < t.nozzle_angle_deg && t.nozzle_angle_deg < 90)) throw std::invalid_argument("invalid turbine nozzle angle");
    validate_gas(g.air);
    validate_gas(g.exhaust);
    for (real v : {c.inducer_mm, c.exducer_mm, c.hub_mm, c.exit_width_mm, c.exit_angle_deg, c.inlet_angle_deg, c.blockage,
                   c.passage_loss, c.incidence_loss, c.diffuser_loss, t.diameter_mm, t.effective_throat_mm2, t.nozzle_angle_deg,
                   t.nozzle_efficiency, t.rotor_velocity_retention})
        if (!std::isfinite(v)) throw std::invalid_argument("nonfinite geometry");
    if (g.source.empty()) throw std::invalid_argument("source and provenance required");
}

real nozzle_flux(real upstream_pa, real downstream_pa, real total_k, const Gas& gas) {
    if (upstream_pa <= 0 || downstream_pa <= 0 || total_k <= 0)
        throw std::invalid_argument("positive nozzle boundary conditions required");
    if (downstream_pa >= upstream_pa) return 0;
    const real g = gas.gamma;
    const real q = std::max(downstream_pa / upstream_pa, ps::math::pow(real(2) / (g + real(1)), g / (g - real(1))));
    const real psi = std::sqrt(real(2) * g / (g - real(1)) *
                               (ps::math::pow(q, real(2) / g) - ps::math::pow(q, (g + real(1)) / g)));
    return upstream_pa / std::sqrt(gas.r * total_k) * psi;
}

real normal_shock_pressure_retention(real mach, const Gas& gas) {
    if (!std::isfinite(mach) || mach < 0) throw std::invalid_argument("finite nonnegative shock Mach required");
    if (mach <= 1) return 1;
    const real g = gas.gamma, m2 = mach * mach;
    const real log_ratio = (g * ps::math::log((g + real(1)) * m2 / ((g - real(1)) * m2 + real(2))) +
                            ps::math::log((g + real(1)) / (real(2) * g * m2 - (g - real(1))))) / (g - real(1));
    return ps::math::exp(log_ratio);
}

CompressorPoint compressor_point(const StageGeometry& geo, real rpm, real mdot, real pt, real tt) {
    const CompressorGeometry& c = geo.compressor;
    const Gas& gas = geo.air;
    if (rpm < 0 || mdot < 0 || pt <= 0 || tt <= 0) throw std::invalid_argument("invalid compressor operating point");
    const real cp = gas.cp();
    const real a1 = kPi / real(4) * (c.inducer_mm * c.inducer_mm - c.hub_mm * c.hub_mm) * real(1e-6) * c.blockage;
    const real a2 = kPi * c.exducer_mm * c.exit_width_mm * real(1e-6) * c.blockage;
    CompressorPoint out;
    real cm1 = 0, mach1 = 0;
    if (!subsonic_velocity(mdot, a1, pt, tt, gas, cm1, mach1)) {
        out.status = CompressorStatus::InletChoke;
        return out;
    }
    const real omega = rpm * kTau / real(60);
    const real u2 = omega * c.exducer_mm * real(0.0005);
    const real u1 = omega * std::sqrt((c.inducer_mm * c.inducer_mm + c.hub_mm * c.hub_mm) / real(2)) * real(0.0005);
    const real inlet_static_k = tt / (real(1) + (gas.gamma - real(1)) / real(2) * mach1 * mach1);
    const real inlet_relative_mach = std::sqrt(cm1 * cm1 + u1 * u1) / std::sqrt(gas.gamma * gas.r * inlet_static_k); // Python: hypot
    const bool shock = c.shock_model == ShockModel::InletNormalEntropy;
    const real shock_retention = shock ? normal_shock_pressure_retention(inlet_relative_mach, gas) : real(1);
    const real shock_entropy = -gas.r * ps::math::log(shock_retention);
    const real incidence = cm1 - u1 * ps::math::tan(radians(c.inlet_angle_deg));
    const real cot_beta = real(1) / ps::math::tan(radians(c.exit_angle_deg));
    const real slip_velocity = u2 * kPi * ps::math::sin(radians(c.exit_angle_deg)) / real(c.blades);
    const real kappa_inv = gas.gamma / (gas.gamma - real(1));

    struct State { real predicted, work, loss, tout, pout, ts, cu2; };
    const auto state = [&](real cm2, State& s) -> bool {
        const real cu2 = u2 - cm2 * cot_beta - slip_velocity;
        const real work = u2 * cu2; // zero prewhirl
        if (work <= 0) return false;
        const real relative2 = sq(cm2) + sq(cu2 - u2);
        const real loss = (c.incidence_loss * sq(incidence) + c.passage_loss * relative2 +
                           c.diffuser_loss * (sq(cm2) + sq(cu2))) / real(2);
        const real isentropic = work - loss;
        if (isentropic <= 0) return false;
        const real tout = tt + work / cp;
        const real pout = pt * ps::math::pow(real(1) + isentropic / (cp * tt), kappa_inv) * shock_retention;
        const real ts = tout - (sq(cm2) + sq(cu2)) / (real(2) * cp);
        if (ts <= 0) return false;
        const real ps_ = pout * ps::math::pow(ts / tout, kappa_inv);
        s = {ps_ / (gas.r * ts) * a2 * cm2, work, loss, tout, pout, ts, cu2};
        return true;
    };

    // The first (subsonic, positive-work) continuity root.
    real previous = 0, lo = 0, hi = 0;
    bool bracket = false;
    const real upper = std::sqrt(gas.gamma * gas.r * tt) * real(3) + u2;
    State s{};
    for (int i = 1; i <= 160; ++i) {
        const real trial = upper * real(i) / real(160);
        if (!state(trial, s)) continue;
        if (s.predicted >= mdot) { lo = previous; hi = trial; bracket = true; break; }
        previous = trial;
    }
    if (!bracket) {
        out.status = CompressorStatus::ExitChokeOrNoPositiveHead;
        out.inlet_mach = mach1;
        return out;
    }
    for (int i = 0; i < 55; ++i) {
        const real mid = (lo + hi) / 2;
        if (!state(mid, s) || s.predicted < mdot) lo = mid; else hi = mid;
    }
    const real cm2 = (lo + hi) / 2;
    state(cm2, s); // valid: the Python unpacks it unconditionally
    const real relative_mach = std::sqrt(sq(cm2) + sq(s.cu2 - u2)) / std::sqrt(gas.gamma * gas.r * s.ts);
    out.status = relative_mach >= 1 ? CompressorStatus::RelativeChoke
                 : (inlet_relative_mach >= 1 && !shock) ? CompressorStatus::TransonicUnmodelled
                                                         : CompressorStatus::Ok;
    if (s.pout <= pt) out.status = CompressorStatus::NoPositivePressureRise;
    const real effective_isentropic = cp * tt * (ps::math::pow(s.pout / pt, (gas.gamma - real(1)) / gas.gamma) - real(1));
    out.pressure_ratio = s.pout / pt;
    out.outlet_total_k = s.tout;
    out.specific_work_j_kg = s.work;
    out.shaft_power_w = mdot * s.work;
    out.efficiency = effective_isentropic / s.work;
    out.shock_pressure_retention = shock_retention;
    out.shock_entropy_j_kg_k = shock_entropy;
    out.inlet_mach = mach1;
    out.inlet_relative_mach = inlet_relative_mach;
    out.exit_relative_mach = relative_mach;
    out.incidence_risk = std::abs(incidence) / std::max(u1, real(1e-12));
    out.continuity_residual_kg_s = s.predicted - mdot;
    out.energy_residual_w = mdot * (cp * (s.tout - tt) - s.work);
    return out;
}

TurbinePoint turbine_point(const StageGeometry& geo, real rpm, real expansion_ratio, real pt, real tt) {
    const TurbineGeometry& t = geo.turbine;
    const Gas& gas = geo.exhaust;
    if (rpm < 0 || expansion_ratio < 1 || pt <= 0 || tt <= 0) throw std::invalid_argument("invalid turbine operating point");
    const real cp = gas.cp();
    TurbinePoint out;
    const real mdot = t.effective_throat_mm2 * real(1e-6) * nozzle_flux(pt, pt / expansion_ratio, tt, gas);
    const real his = cp * tt * (real(1) - ps::math::pow(expansion_ratio, -(gas.gamma - real(1)) / gas.gamma));
    const real jet = std::sqrt(real(2) * his * t.nozzle_efficiency);
    const real cu1 = jet * ps::math::cos(radians(t.nozzle_angle_deg));
    const real u = rpm * kTau / real(60) * t.diameter_mm * real(0.0005);
    const real cu2 = u - t.rotor_velocity_retention * (cu1 - u);
    const real work = u * (cu1 - cu2);
    out.mass_flow_kg_s = mdot;
    out.specific_work_j_kg = work;
    out.shaft_power_w = mdot * work;
    out.specific_torque_nm_per_kg_s = t.diameter_mm * real(0.0005) * (real(1) + t.rotor_velocity_retention) * (cu1 - u);
    out.outlet_total_k = tt - work / cp;
    out.efficiency = his > 0 ? work / his : real(0);
    out.energy_residual_w = mdot * (cp * (tt - out.outlet_total_k) - work);
    out.choked = expansion_ratio >= ps::math::pow((gas.gamma + real(1)) / real(2), gas.gamma / (gas.gamma - real(1)));
    return out;
}

Sampling uniform_sampling(const StageGeometry& g, real rpm_max, int speed_lines, real flow_max, int flow_points,
                          real expansion_max, int expansion_points) {
    if (std::min({speed_lines, flow_points, expansion_points}) < 2) throw std::invalid_argument("axis counts must be >=2");
    if (!(expansion_max > 1)) throw std::invalid_argument("expansion max must exceed one");
    Sampling s;
    for (int i = 0; i < speed_lines; ++i) s.corrected_rpm.push_back(rpm_max * real(i) / real(speed_lines - 1));
    for (int i = 0; i < flow_points; ++i) s.flow_kg_s.push_back(flow_max * real(i) / real(flow_points - 1));
    const real e = turbine_exponent(g);
    const real chi_max = std::sqrt(real(1) - ps::math::pow(expansion_max, -e));
    for (int i = 0; i < expansion_points; ++i) {
        const real x = chi_max * real(i) / real(expansion_points - 1);
        s.expansion_ratio.push_back(ps::math::pow(real(1) - x * x, -real(1) / e));
    }
    return s;
}

StageMapResult generate(const StageGeometry& g, const Sampling& s, ps::jobs::JobSystem* pool) {
    validate(g);
    check_axes(s);
    StageMapResult r;
    r.axes = s;
    r.geometry = g;
    const std::size_t nf = s.flow_kg_s.size(), ne = s.expansion_ratio.size();
    r.compressor.resize(s.corrected_rpm.size() * nf);
    r.turbine.resize(s.corrected_rpm.size() * ne);
    for_each_index(r.compressor.size() + r.turbine.size(), pool, [&](std::size_t i) {
        if (i < r.compressor.size()) r.compressor[i] = compressor_point(g, s.corrected_rpm[i / nf], s.flow_kg_s[i % nf]);
        else {
            const std::size_t j = i - r.compressor.size();
            r.turbine[j] = turbine_point(g, s.corrected_rpm[j / ne], s.expansion_ratio[j % ne]);
        }
    });
    return r;
}

StageMapResult adaptive_generate(const StageGeometry& g, const Sampling& initial, const AdaptiveOptions& o,
                                 ps::jobs::JobSystem* pool) {
    if (!(0 < o.tolerance && o.tolerance < 1) || o.max_passes < 0) throw std::invalid_argument("invalid adaptive sampling settings");
    if (initial.corrected_rpm.size() > std::size_t(o.max_speed_lines) || initial.flow_kg_s.size() > std::size_t(o.max_flow_points) ||
        initial.expansion_ratio.size() > std::size_t(o.max_expansion_points))
        throw std::invalid_argument("initial grid exceeds sampling budget");
    Sampling axes = initial;
    StageMapResult result;
    AuditOut audit;
    bool exhausted = false;
    int iteration = 0;
    for (iteration = 0; iteration <= o.max_passes; ++iteration) {
        result = generate(g, axes, pool);
        audit = interpolation_audit(g, result, pool);
        // (error, stage, n, q) with stage "compressor" < "turbine", as the Python's tuples compare.
        std::vector<std::tuple<real, int, real, real>> failing;
        for (const Cell& c : audit.compressor_cells) if (c.error > o.tolerance) failing.emplace_back(c.error, 0, c.n, c.q);
        for (const Cell& c : audit.turbine_cells) if (c.error > o.tolerance) failing.emplace_back(c.error, 1, c.n, c.q);
        if (failing.empty()) break;
        if (iteration == o.max_passes) { exhausted = true; break; }
        std::set<real> speed(axes.corrected_rpm.begin(), axes.corrected_rpm.end());
        std::set<real> flow(axes.flow_kg_s.begin(), axes.flow_kg_s.end());
        std::set<real> er(axes.expansion_ratio.begin(), axes.expansion_ratio.end());
        std::sort(failing.begin(), failing.end(), [](const auto& a, const auto& b) { return a > b; });
        for (const auto& [error, stage, n, q] : failing) {
            (void)error;
            if (speed.size() < std::size_t(o.max_speed_lines)) speed.insert(n);
            auto& target = stage == 0 ? flow : er;
            const std::size_t budget = std::size_t(stage == 0 ? o.max_flow_points : o.max_expansion_points);
            if (target.size() < budget) target.insert(q);
        }
        if (speed.size() == axes.corrected_rpm.size() && flow.size() == axes.flow_kg_s.size() &&
            er.size() == axes.expansion_ratio.size()) {
            exhausted = true;
            break;
        }
        axes.corrected_rpm.assign(speed.begin(), speed.end());
        axes.flow_kg_s.assign(flow.begin(), flow.end());
        axes.expansion_ratio.assign(er.begin(), er.end());
    }
    result.adaptive = true;
    result.sampling.target_relative_error = o.tolerance;
    result.sampling.passes = iteration;
    result.sampling.budget_limited = exhausted;
    result.sampling.compressor = audit.compressor;
    result.sampling.turbine = audit.turbine;
    return result;
}

std::uint64_t cache_key(const StageGeometry& g, const Sampling& initial, const AdaptiveOptions* adaptive) {
    std::uint64_t h = 1469598103934665603ULL;
    h = fnv_str(h, kSolverVersion);
    h = fnv_str(h, g.profile_json);
    for (const auto* axis : {&initial.corrected_rpm, &initial.flow_kg_s, &initial.expansion_ratio}) {
        const std::uint64_t n = axis->size();
        h = fnv(h, &n, sizeof(n));
        for (real v : *axis) h = fnv_real(h, v);
    }
    const std::uint8_t is_adaptive = adaptive != nullptr ? 1 : 0;
    h = fnv(h, &is_adaptive, 1);
    if (adaptive != nullptr) {
        h = fnv_real(h, adaptive->tolerance);
        for (int v : {adaptive->max_passes, adaptive->max_speed_lines, adaptive->max_flow_points, adaptive->max_expansion_points}) {
            const std::int64_t w = v;
            h = fnv(h, &w, sizeof(w));
        }
    }
    return h;
}

std::vector<DemandPoint> demand_points(const TurboStageDemand& d, real air_kappa) {
    std::vector<DemandPoint> out;
    if (!d.applies) return out;
    const real rise = ps::math::pow(d.pressure_ratio, air_kappa) - real(1);
    for (const TurboStageDemand::Step& st : d.steps) {
        real n = st.rotor_rpm;
        if (!st.reached) {
            const real top_rise = ps::math::pow(std::max(st.top_line_pressure_ratio, real(1)), air_kappa) - real(1);
            n = top_rise > 0 ? d.top_speed_line_rpm * std::sqrt(rise / top_rise) : d.top_speed_line_rpm * real(2);
        }
        out.push_back({st.flow_kg_s, n});
    }
    return out;
}

Sampling extend_sampling_for_demand(const Sampling& current, const std::vector<DemandPoint>& demand, const ExtensionPolicy& p) {
    check_axes(current);
    if (!(p.flow_margin >= 0) || !(p.speed_margin >= 0) || !(p.speed_step_rpm > 0) || !(p.flow_step_kg_s > 0) ||
        p.max_speed_lines < 2 || p.max_flow_points < 2)
        throw std::invalid_argument("invalid stage-map extension policy");
    if (demand.empty()) return current;
    real n_min = demand.front().rotor_rpm, n_max = n_min, w_min = demand.front().flow_kg_s, w_max = w_min;
    for (const DemandPoint& d : demand) {
        if (!std::isfinite(d.flow_kg_s) || !std::isfinite(d.rotor_rpm) || d.flow_kg_s < 0 || d.rotor_rpm < 0)
            throw std::invalid_argument("invalid stage-map demand point");
        n_min = std::min(n_min, d.rotor_rpm); n_max = std::max(n_max, d.rotor_rpm);
        w_min = std::min(w_min, d.flow_kg_s); w_max = std::max(w_max, d.flow_kg_s);
    }
    const real n_hi = n_max * (real(1) + p.speed_margin), w_hi = w_max * (real(1) + p.flow_margin);
    // Multiples of step covering [lo, hi] (one step either side), added to a copy of axis; empty when over budget.
    const auto add = [](const std::vector<real>& axis, real lo, real hi, real step, std::size_t budget) {
        std::set<real> knots(axis.begin(), axis.end());
        const real k0 = std::max(real(0), std::floor(lo / step) - real(1)), k1 = std::ceil(hi / step) + real(1);
        for (real k = k0; k <= k1; k += 1) {
            knots.insert(k * step);
            if (knots.size() > budget) return std::vector<real>{};
        }
        return std::vector<real>(knots.begin(), knots.end());
    };
    Sampling out = current;
    for (real step = p.speed_step_rpm;; step *= 2) {
        out.corrected_rpm = add(current.corrected_rpm, n_min, n_hi, step, std::size_t(p.max_speed_lines));
        if (!out.corrected_rpm.empty()) break;
        if (step > n_hi) throw std::invalid_argument("stage-map extension does not fit the speed-line budget");
    }
    for (real step = p.flow_step_kg_s;; step *= 2) {
        out.flow_kg_s = add(current.flow_kg_s, w_min, w_hi, step, std::size_t(p.max_flow_points));
        if (!out.flow_kg_s.empty()) break;
        if (step > w_hi) throw std::invalid_argument("stage-map extension does not fit the flow budget");
    }
    return out;
}

std::shared_ptr<TurboStageMaps> to_stage_maps(const StageMapResult& r, int units) {
    auto sm = std::make_shared<TurboStageMaps>();
    sm->source = r.geometry.source;
    sm->solver = kSolverVersion;
    sm->reference_pa = kReferencePa;
    sm->compressor_reference_k = kReferenceK;
    sm->turbine_reference_k = kTurbineReferenceK;
    sm->exhaust_r = r.geometry.exhaust.r;
    sm->exhaust_gamma = r.geometry.exhaust.gamma;
    sm->compressor_cp = r.geometry.air.cp();
    sm->air_kappa = (r.geometry.air.gamma - real(1)) / r.geometry.air.gamma;
    sm->units = units;
    const auto& a = r.axes;
    sm->compressor_pr.x = sm->compressor_work_j_kg.x = sm->turbine_flow_kg_s.x = sm->turbine_work_j_kg.x =
        sm->turbine_torque_nm_per_kg_s.x = a.corrected_rpm;
    sm->compressor_pr.y = sm->compressor_work_j_kg.y = a.flow_kg_s;
    for (real er : a.expansion_ratio) {
        const real chi = sm->turbine_coordinate(er);
        sm->turbine_flow_kg_s.y.push_back(chi);
        sm->turbine_work_j_kg.y.push_back(chi);
        sm->turbine_torque_nm_per_kg_s.y.push_back(chi);
    }
    for (const CompressorPoint& p : r.compressor) {
        const bool valid = p.status == CompressorStatus::Ok;
        sm->compressor_valid.push_back(valid);
        sm->compressor_pr.values.push_back(valid ? p.pressure_ratio : real(1));
        sm->compressor_work_j_kg.values.push_back(valid ? p.specific_work_j_kg : real(0));
    }
    for (const TurbinePoint& p : r.turbine) {
        sm->turbine_flow_kg_s.values.push_back(p.mass_flow_kg_s);
        sm->turbine_work_j_kg.values.push_back(p.specific_work_j_kg);
        sm->turbine_torque_nm_per_kg_s.values.push_back(p.specific_torque_nm_per_kg_s);
    }
    return sm;
}

} // namespace ps::drivetrain::meanline
