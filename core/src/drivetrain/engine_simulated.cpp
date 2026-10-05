// core/src/drivetrain/engine_simulated.cpp — ps::drivetrain::SimulatedEngine
// (PHYS-033 P4 I7b C11). See engine_simulated.h for the model.
//
// Engine management (state machine, DFCO, idle PI, limiter, starter,
// starvation) is DUPLICATED from engine_torque_map.cpp on purpose (I7 plan
// section 3.2): sharing it would reorder the graph engine's floating-point
// operations and move every hash gate. Keep the two in step by hand.
//
// Runtime (pre_solve/post_solve): + - * /, table lookups only (plan section
// 3.5). ps::math::pow and the bisection solves run in build_maps only;
// std::sqrt only in set_ambient/construction.

#include "engine_simulated.h"
#include "psi_table.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace ps::drivetrain {
namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kTwoPi = real(2.0) * kPi;
constexpr real kFourPi = real(4.0) * kPi;
// I7d compressor-flow fix, both PROVISIONAL (owner/coordinator to confirm; no cited value exists):
// - kTurboFlowLagS [AS]: first-order relaxation of the throttle-path flow handed to the compressor. 5 ms is
//   the charge-pipe/compressor flow inertia and, empirically, the loop-stability margin: unrelaxed (0) a held
//   7500-8200 rpm WOT ran away; 0.5 ms and up are stable over 7000-8200 rpm (steady values equal to 1e-4 bar).
//   25 ms (the charge-pressure lag) is stable too but keeps the compressor loaded so long after a lift that
//   the rotor stops again within 100 ms.
// (The former kExhaustHeatHoldS peak-hold of the fuel watts, 0.5 s [AS], is replaced by the exhaust energy
// balance below and the lumped exhaust wall in turbo_pair.h - exhaust_thermal in turbo.md.)
constexpr real kTurboFlowLagS = real(0.005);
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);

real sign(real x) { return x > real(0.0) ? real(1.0) : (x < real(0.0) ? real(-1.0) : real(0.0)); }

// Copied from engine_torque_map.cpp (0 at/beyond the ends; an interior knot
// takes the segment ending there - Table2D's convention too).
real curve_slope_at(const vehicle::Curve1D& c, real xv) {
    if (c.x.size() < 2) return real(0.0);
    if (xv <= c.x.front() || xv >= c.x.back()) return real(0.0);
    for (std::size_t i = 1; i < c.x.size(); ++i) {
        if (xv <= c.x[i]) {
            const real dx = c.x[i] - c.x[i - 1];
            return dx > real(0.0) ? (c.y[i] - c.y[i - 1]) / dx : real(0.0);
        }
    }
    return real(0.0);
}

// --- FNV-1a folding (same byte fold as EngineTorqueMap::fold_state_hash) ---
struct Fnv {
    std::uint64_t h;
    template <typename T>
    void pod(const T& v) {
        unsigned char bytes[sizeof(T)];
        std::memcpy(bytes, &v, sizeof(T));
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    }
    void vec(const std::vector<real>& v) {
        pod(static_cast<std::uint64_t>(v.size()));
        for (real x : v) pod(x);
    }
    void curve(const vehicle::Curve1D& c) {
        vec(c.x);
        vec(c.y);
    }
    void table(const Table2D& t) {
        vec(t.x);
        vec(t.y);
        vec(t.values);
    }
};

// Ambient the free functions below evaluate against.
struct Ambient {
    real p_amb;
    real r_t;
    real inv_sqrt_rt;
};

Ambient make_ambient(real p_pa, real t_k) {
    const real r_t = SimulatedEngine::kGasConstant * t_k;
    return Ambient{p_pa, r_t, real(1.0) / std::sqrt(r_t)};
}

// Tabulated Psi(pr) on a uniform grid of kPsiIntervals intervals.
real psi_lookup(const std::vector<real>& table, real pr, real* dpsi_dpr) {
    return psi_table_lookup(table, pr, dpsi_dpr);
}

// True isentropic flow function (build time only, ps::math::pow).
real psi_true(real pr) {
    constexpr real g = SimulatedEngine::kGamma;
    const real crit = ps::math::pow(real(2.0) / (g + real(1.0)), g / (g - real(1.0)));
    if (pr <= crit) {
        return std::sqrt(g * ps::math::pow(real(2.0) / (g + real(1.0)), (g + real(1.0)) / (g - real(1.0))));
    }
    const real a = ps::math::pow(pr, real(2.0) / g);
    const real b = ps::math::pow(pr, (g + real(1.0)) / g);
    return std::sqrt(real(2.0) * g / (g - real(1.0)) * std::max(real(0.0), a - b));
}

SimulatedEngine::ManifoldRhs manifold_rhs_impl(const SimulatedEngineMaps& m, const Ambient& amb, real p, real omega, real x_thr,
                                               real x_iac) {
    SimulatedEngine::ManifoldRhs r;
    const real frac = m.area_fraction.evaluate(x_thr);
    const real a_eff = m.cd * m.a_throttle_full_m2 * frac + x_iac * m.a_bypass_m2;
    const real k = a_eff * amb.inv_sqrt_rt;
    real dm_thr_dp = real(0.0);
    if (p <= amb.p_amb) {
        real dpsi = real(0.0);
        const real ps = psi_lookup(m.psi, p / amb.p_amb, &dpsi);
        r.m_thr = k * amb.p_amb * ps;
        dm_thr_dp = k * dpsi;
    } else {
        const real q = amb.p_amb / p;
        real dpsi = real(0.0);
        const real ps = psi_lookup(m.psi, q, &dpsi);
        r.m_thr = -k * p * ps;
        dm_thr_dp = -k * (ps - q * dpsi);
    }
    const real rpm = std::abs(omega) * kRadPerSecToRpm;
    const real ve = m.maps.ve.value(rpm, p);
    const real dve_dp = m.maps.ve.dvalue_dy(rpm, p);
    const real vol_rate = m.v_disp_m3 * m.c_cycle * std::max(omega, real(0.0)) / kTwoPi; // m^3/s swept (VE 1)
    r.m_cyl = ve * p / amb.r_t * vol_rate;
    const real dm_cyl_dp = (dve_dp * p + ve) / amb.r_t * vol_rate;
    const real c = amb.r_t / m.v_im_m3;
    r.f = c * (r.m_thr - r.m_cyl);
    r.dfdp = c * (dm_thr_dp - dm_cyl_dp);
    return r;
}

SimulatedEngine::TorqueEval evaluate_torque_impl(const SimulatedEngineMaps& m, const Ambient& amb, const StarterDesc& starter,
                                                 real omega, real p, bool fired, real s_starve, bool starter_active,
                                                 bool ignore_limiter = false, const AntiLagDesc* als = nullptr, real k_fric = real(1.0)) {
    SimulatedEngine::TorqueEval e;
    const real rpm = std::abs(omega) * kRadPerSecToRpm;
    const real s_w = sign(omega);
    const real vc_2pi = m.v_disp_m3 * m.c_cycle / kTwoPi;

    // air per revolution (and its rpm slope at frozen p)
    const real ve = m.maps.ve.value(rpm, p);
    const real dve_drpm = m.maps.ve.dvalue_dx(rpm, p);
    const real air_per_ve = p / amb.r_t * m.v_disp_m3 * m.c_cycle;
    e.m_air_rev = ve * air_per_ve;
    const real dma_drpm = dve_drpm * air_per_ve;

    // combustion
    real fire = real(0.0), dfire_drpm = real(0.0);
    if (fired) {
        if (m.limiter.type == LimiterType::SoftCut && !ignore_limiter) {
            fire = std::clamp((m.limiter.rpm - rpm) / m.limiter.band_rpm, real(0.0), real(1.0));
            const bool in_band = rpm < m.limiter.rpm && rpm > m.limiter.rpm - m.limiter.band_rpm;
            dfire_drpm = in_band ? real(-1.0) / m.limiter.band_rpm : real(0.0);
        } else {
            fire = real(1.0);
        }
    }
    e.fire = fire;
    e.lambda = m.ecu.lambda_target.value(rpm, p);
    real dlambda_drpm = m.ecu.lambda_target.dvalue_dx(rpm, p);
    if (als && als->lambda > real(0.0)) { e.lambda = als->lambda; dlambda_drpm = real(0.0); } // anti-lag enrichment
    const real spark_target = m.ecu.spark_advance_deg.value(rpm, p);
    real spark = spark_target;
    real dspark_drpm = m.ecu.spark_advance_deg.dvalue_dx(rpm, p);
    if (m.maps.has_theta_kl) {
        const real kl = m.maps.theta_kl_deg.value(rpm, p);
        if (kl < spark_target) {
            spark = kl;
            dspark_drpm = m.maps.theta_kl_deg.dvalue_dx(rpm, p);
        }
    }
    // I7d D5: the anti-lag retard moves the applied spark from the scheduled
    // (knock-limited) advance; a constant offset, so dspark_drpm is unchanged.
    // An absolute anti-lag spark setting is constant in rpm.
    real spark_applied = spark;
    if (als && als->spark_advance_deg) {
        spark_applied = *als->spark_advance_deg;
        dspark_drpm = real(0.0);
    } else if (als && als->retard_deg > real(0.0)) {
        spark_applied = spark - als->retard_deg;
    }
    e.spark_deg = spark_applied;
    e.theta_mbt_deg = m.maps.theta_mbt_deg.value(rpm, p);
    const real dmbt_drpm = m.maps.theta_mbt_deg.dvalue_dx(rpm, p);
    e.eta_i = m.maps.eta_i.value(rpm, e.m_air_rev);
    const real deta_drpm = m.maps.eta_i.dvalue_dx(rpm, e.m_air_rev) + m.maps.eta_i.dvalue_dy(rpm, e.m_air_rev) * dma_drpm;
    e.f_lambda = m.maps.f_lambda.evaluate(e.lambda);
    const real dfl_dlambda = curve_slope_at(m.maps.f_lambda, e.lambda);
    const real dspark_rel = spark_applied - e.theta_mbt_deg;
    e.eta_spark = m.maps.eta_spark.evaluate(dspark_rel);
    real des_d = curve_slope_at(m.maps.eta_spark, dspark_rel);
    // Anti-lag only: below the table's most retarded node (the generator's cycle points end at 40 deg
    // after MBT) eta_spark is extrapolated along the first segment's slope, floored at 0 (PROVISIONAL -
    // a real ALS fires 35-45 deg ATDC, past the table). Without anti-lag the table clamps as before.
    if (als && m.maps.eta_spark.x.size() >= 2 && dspark_rel < m.maps.eta_spark.x.front()) {
        const auto& cx = m.maps.eta_spark.x;
        const auto& cy = m.maps.eta_spark.y;
        const real slope = (cy[1] - cy[0]) / (cx[1] - cx[0]);
        const real v = cy[0] + slope * (dspark_rel - cx[0]);
        e.eta_spark = std::max(real(0.0), v);
        des_d = v > real(0.0) ? slope : real(0.0);
    }

    const real g = e.eta_i * e.f_lambda * e.eta_spark;
    const real dg_drpm = deta_drpm * e.f_lambda * e.eta_spark + e.eta_i * dfl_dlambda * dlambda_drpm * e.eta_spark +
                         e.eta_i * e.f_lambda * des_d * (dspark_drpm - dmbt_drpm);
    const real afr = e.lambda * m.afr_stoich;
    const real fuel_per_fire = e.m_air_rev / afr; // kg/rev at fire 1, s 1
    e.m_fuel_rev = fuel_per_fire * fire * s_starve;
    const real dfuel_per_fire_drpm = dma_drpm / afr - e.m_air_rev * dlambda_drpm / (e.lambda * afr);
    const real dmfuel_drpm = s_starve * (fire * dfuel_per_fire_drpm + fuel_per_fire * dfire_drpm);
    const real lhv_2pi = m.lhv_j_kg / kTwoPi;
    e.tau_comb = g * lhv_2pi * e.m_fuel_rev;
    e.tau_comb_unretarded = e.tau_comb;
    if (spark_applied != spark) {
        const real g0 = e.eta_i * e.f_lambda * m.maps.eta_spark.evaluate(spark - e.theta_mbt_deg);
        e.tau_comb_unretarded = g0 * lhv_2pi * e.m_fuel_rev;
    }
    real dcomb_drpm = lhv_2pi * (dg_drpm * e.m_fuel_rev + g * dmfuel_drpm);
    // Anti-lag cylinder cut: the fuel is still injected (m_fuel_rev, booked), only the fired share burns in
    // the cylinder; the cut charge reaches the exhaust manifold (the exhaust energy balance).
    if (als && als->cylinder_cut_fraction > real(0.0)) {
        e.cut_fraction = als->cylinder_cut_fraction;
        const real fired_share = real(1.0) - e.cut_fraction;
        e.tau_comb *= fired_share;
        dcomb_drpm *= fired_share;
    }

    // pumping (ideal gas exchange + optional excess), friction (Coulomb part excluded)
    real pmep = real(0.0), dpmep_drpm = real(0.0);
    if (m.maps.has_pmep_excess) {
        pmep = m.maps.pmep_excess_pa.value(rpm, p);
        dpmep_drpm = m.maps.pmep_excess_pa.dvalue_dx(rpm, p);
    }
    e.tau_pump = -s_w * (amb.p_amb - p + pmep) * vc_2pi;
    const real f_rpm = m.maps.friction_torque_nm.evaluate(rpm);
    // Engine thermal plan T1: k_fric = the oil-viscosity friction scale of the CALLER's model (the plant's k for the
    // physics, the ECU's estimate for the steady tables); the Coulomb part c_k is excluded here and scaled by the
    // Coulomb row. k_fric = 1.0: x*1.0 == x, bit-identical.
    e.tau_fric = -s_w * (k_fric * (f_rpm - m.c_k));

    // starter (EngineTorqueMap's own law)
    const real omega_s0 = starter.free_speed_rpm / kRadPerSecToRpm;
    e.tau_start = starter_active ? starter.stall_torque_nm * std::max(real(0.0), real(1.0) - std::max(omega, real(0.0)) / omega_s0)
                                 : real(0.0);

    e.tau = e.tau_comb + e.tau_pump + e.tau_fric + e.tau_start;

    // d/domega = K*sign(omega)*d/drpm for rpm-table terms; combustion only
    // ever runs at omega > 0 (sign 1). Pumping/friction: sign(omega)^2 = 1.
    real dtau = kRadPerSecToRpm * s_w * dcomb_drpm;
    dtau += -kRadPerSecToRpm * dpmep_drpm * vc_2pi * (s_w != real(0.0) ? real(1.0) : real(0.0));
    dtau += -kRadPerSecToRpm * k_fric * curve_slope_at(m.maps.friction_torque_nm, rpm) * (s_w != real(0.0) ? real(1.0) : real(0.0));
    if (starter_active && omega >= real(0.0) && omega < omega_s0) dtau -= starter.stall_torque_nm / omega_s0;
    e.dtau_domega = dtau;
    return e;
}

// Steady manifold pressure f(p) = 0 by bisection (build time only).
real steady_p(const SimulatedEngineMaps& m, const Ambient& amb, real omega, real x_thr, real x_iac) {
    real lo = real(1e-3) * amb.p_amb;
    real hi = amb.p_amb;
    if (manifold_rhs_impl(m, amb, lo, omega, x_thr, x_iac).f <= real(0.0)) return lo;
    if (manifold_rhs_impl(m, amb, hi, omega, x_thr, x_iac).f >= real(0.0)) return hi;
    for (int it = 0; it < 200; ++it) {
        const real mid = real(0.5) * (lo + hi);
        if (mid <= lo || mid >= hi) break;
        if (manifold_rhs_impl(m, amb, mid, omega, x_thr, x_iac).f > real(0.0)) lo = mid;
        else hi = mid;
    }
    return real(0.5) * (lo + hi);
}

// Net steady crank torque (Coulomb row included, omega > 0), fired torque
// ignoring every limiter (the steady tables are the ECU's demand model - see
// header). C12: the limiter is bypassed by evaluate_torque_impl's
// ignore_limiter flag (fire = 1 when fired, exactly what C11's limiter-free
// maps copy gave) instead of copying the whole maps struct, which allocated.
real steady_net_torque(const SimulatedEngineMaps& m, const Ambient& amb, const StarterDesc& st, real rpm, real x_thr, real x_iac,
                       bool fired, real k_ecu = real(1.0)) {
    const real omega = rpm / kRadPerSecToRpm;
    const real p = steady_p(m, amb, omega, x_thr, x_iac);
    // k_ecu: the ECU's friction scale (engine thermal plan T1); the Coulomb row's kinetic value scales with it too.
    return evaluate_torque_impl(m, amb, st, omega, p, fired, real(1.0), false, true, nullptr, k_ecu).tau - k_ecu * m.c_k;
}

// Setup builds use 60 iterations; live control solves only the current idle operating point.
void fill_idle_control(const SimulatedEngineMaps& m,const Ambient& amb,real idle_rpm,real idle_max_x,
                       real idle_max_torque_nm,SimulatedSteadyTables& t,int iterations,real k_ecu = real(1.0)) {
    const StarterDesc no_starter{};
    // Idle feed-forward: the plate (electronic) / bypass valve (mechanical,
    // plate at 0) position whose steady net torque at idle_rpm is zero.
    {
        const bool mech = m.throttle_input == ThrottleInput::Mechanical;
        auto net = [&](real x) {
            return mech ? steady_net_torque(m, amb, no_starter, idle_rpm, real(0.0), x, true, k_ecu)
                        : steady_net_torque(m, amb, no_starter, idle_rpm, x, real(0.0), true, k_ecu);
        };
        real ff = real(0.0);
        if (net(real(0.0)) >= real(0.0)) {
            ff = real(0.0);
        } else if (net(idle_max_x) <= real(0.0)) {
            ff = idle_max_x;
        } else {
            real lo = real(0.0), hi = idle_max_x;
            for (int it = 0; it < iterations; ++it) {
                const real mid = real(0.5) * (lo + hi);
                if (net(mid) < real(0.0)) lo = mid;
                else hi = mid;
            }
            ff = real(0.5) * (lo + hi);
        }
        t.idle_ff = ff;
        const real omega_idle = idle_rpm / kRadPerSecToRpm;
        t.idle_p_im = mech ? steady_p(m, amb, omega_idle, real(0.0), ff) : steady_p(m, amb, omega_idle, ff, real(0.0));

        // Idle authority cap (owner 2026-09-29: "not orders of magnitude
        // more" than creeping in gear needs): the position whose steady net
        // torque at idle_rpm is max_torque_nm, bounded by idle_max_x.
        t.idle_cap_x = real(0.0);
        if (idle_max_torque_nm > real(0.0)) {
            real x_cap = idle_max_x;
            if (net(idle_max_x) > idle_max_torque_nm) {
                real lo = ff, hi = idle_max_x;
                for (int it = 0; it < iterations; ++it) {
                    const real mid = real(0.5) * (lo + hi);
                    if (net(mid) < idle_max_torque_nm) lo = mid;
                    else hi = mid;
                }
                x_cap = real(0.5) * (lo + hi);
            }
            t.idle_cap_x = x_cap;
        }
    }
}

// Fill the ambient-dependent derived tables IN PLACE. t's axes and value
// arrays must already be sized (build_maps presets them; a SimulatedEngine's
// own copy keeps them) - only indexed writes below, no allocation, so a
// runtime refill (set_ambient) is allocation-free. check_monotone throws on
// a steady torque that decreases with plate position (build time only; a
// runtime refill never throws).
void fill_steady_tables(const SimulatedEngineMaps& m, const Ambient& amb, real idle_rpm, real idle_max_x, real idle_max_torque_nm,
                        SimulatedSteadyTables& t,
                        bool check_monotone) {
    const StarterDesc no_starter{};
    const std::vector<real>& rpm_axis = t.steady_torque_nm.x;
    const std::vector<real>& plate_axis = t.steady_torque_nm.y;
    for (std::size_t ir = 0; ir < rpm_axis.size(); ++ir) {
        const real rpm = rpm_axis[ir];
        for (std::size_t ix = 0; ix < plate_axis.size(); ++ix) {
            const real v = steady_net_torque(m, amb, no_starter, rpm, plate_axis[ix], real(0.0), true);
            if (check_monotone && ix > 0 && v < t.steady_torque_nm.values[ir * plate_axis.size() + ix - 1] - real(1e-9)) {
                throw std::runtime_error("SimulatedEngine: steady torque is not non-decreasing in plate position (rpm " +
                                         std::to_string(rpm) + ", plate " + std::to_string(plate_axis[ix - 1]) + " -> " +
                                         std::to_string(plate_axis[ix]) + ": " +
                                         std::to_string(t.steady_torque_nm.values[ir * plate_axis.size() + ix - 1]) +
                                         " -> " + std::to_string(v) + " N*m)");
            }
            t.steady_torque_nm.values[ir * plate_axis.size() + ix] = v;
        }
        t.steady_wot_nm.y[ir] = t.steady_torque_nm.values[ir * plate_axis.size() + plate_axis.size() - 1];
        t.steady_motoring_nm.y[ir] = steady_net_torque(m, amb, no_starter, rpm, real(0.0), real(0.0), false);
    }

    // Electronic inverse: request r -> plate x with T(x) = mot + r*(wot - mot).
    if (!t.inverse_plate.values.empty()) {
        const std::size_t nq = t.inverse_plate.y.size();
        for (std::size_t ir = 0; ir < rpm_axis.size(); ++ir) {
            const real rpm = rpm_axis[ir];
            const real wot = t.steady_wot_nm.y[ir];
            const real mot = t.steady_motoring_nm.y[ir];
            const real t0 = t.steady_torque_nm.values[ir * plate_axis.size()];
            for (std::size_t iq = 0; iq < nq; ++iq) {
                const real r = t.inverse_plate.y[iq];
                real x = real(0.0);
                if (iq + 1 == nq) {
                    x = real(1.0);
                } else {
                    const real target = mot + r * (wot - mot);
                    if (t0 >= target) {
                        x = real(0.0);
                    } else {
                        real lo = real(0.0), hi = real(1.0);
                        for (int it = 0; it < 60; ++it) {
                            const real mid = real(0.5) * (lo + hi);
                            if (steady_net_torque(m, amb, no_starter, rpm, mid, real(0.0), true) < target) lo = mid;
                            else hi = mid;
                        }
                        x = real(0.5) * (lo + hi);
                    }
                }
                t.inverse_plate.values[ir * nq + iq] = x;
            }
        }
    }

    fill_idle_control(m,amb,idle_rpm,idle_max_x,idle_max_torque_nm,t,60);
    t.p_pa = amb.p_amb;
}

} // namespace

// ---------------------------------------------------------------------------

std::shared_ptr<const SimulatedEngineMaps> SimulatedEngine::build_maps(const SimulatedEngineDesc& desc) {
    if (desc.architecture != EngineArchitecture::Piston4Stroke) {
        throw std::runtime_error("SimulatedEngine: rotary architecture is not implemented yet (I7e C13)");
    }
    if (desc.fuels.empty()) throw std::runtime_error("SimulatedEngine: at least one fuel is required");
    if (!(desc.intake_manifold_volume_m3 > real(0.0))) throw std::runtime_error("SimulatedEngine: intake_manifold_volume_m3 must be > 0");
    if (desc.maps.friction_torque_nm.x.empty() || desc.maps.friction_torque_nm.x.front() != real(0.0)) {
        throw std::runtime_error("SimulatedEngine: maps.friction_torque_nm must start at rpm 0");
    }

    auto m = std::make_shared<SimulatedEngineMaps>();
    m->throttle_input = desc.throttle_input;
    m->v_disp_m3 = real(desc.piston.cylinders) * kPi / real(4.0) * desc.piston.bore_m * desc.piston.bore_m * desc.piston.stroke_m;
    m->c_cycle = real(0.5);
    m->v_im_m3 = desc.intake_manifold_volume_m3;
    m->a_throttle_full_m2 = real(desc.throttle_body.count) * kPi / real(4.0) * desc.throttle_body.diameter_m * desc.throttle_body.diameter_m;
    m->cd = desc.throttle_body.discharge_coefficient;
    m->a_bypass_m2 = desc.throttle_input == ThrottleInput::Mechanical ? desc.idle_bypass_area_m2 : real(0.0);
    m->area_fraction = desc.throttle_body.area_fraction_vs_position;
    m->ecu = desc.ecu;
    m->maps = desc.maps;
    m->maps.source.clear();
    m->lhv_j_kg = desc.fuels.front().lhv_j_kg;
    m->afr_stoich = desc.fuels.front().afr_stoich;
    m->c_k = desc.maps.friction_torque_nm.evaluate(real(0.0));
    m->limiter = desc.limiter;
    if (desc.breakaway_torque_nm < m->c_k) {
        throw std::runtime_error("SimulatedEngine: breakaway_torque_nm must be >= maps.friction_torque_nm(0)");
    }

    // Psi table: exact on [0, 0.99], linear to 0 over [0.99, 1] (the true
    // Psi has an infinite slope at pr -> 1, which would make df/dp and the
    // air step's denominator unbounded near ambient).
    constexpr int n = kPsiIntervals;
    const int i_reg = static_cast<int>(kPsiRegularisationStart * real(n) + real(0.5));
    m->psi.resize(static_cast<std::size_t>(n) + 1);
    const real psi_reg = psi_true(real(i_reg) / real(n));
    for (int i = 0; i <= n; ++i) {
        m->psi[static_cast<std::size_t>(i)] =
            i < i_reg ? psi_true(real(i) / real(n)) : psi_reg * real(n - i) / real(n - i_reg);
    }

    // Steady tables at ISA (C12: the shared fill_steady_tables, the same
    // routine a SimulatedEngine's runtime refill uses).
    m->ref_p_pa = kIsaPressurePa;
    m->ref_t_k = kIsaTemperatureK;
    {
        const Ambient amb = make_ambient(kIsaPressurePa, kIsaTemperatureK);
        std::vector<real> rpm_axis;
        for (real r = real(500.0); r <= desc.limiter.rpm + real(250.0) + real(1e-9); r += real(250.0)) rpm_axis.push_back(r);
        std::vector<real> plate_axis;
        for (int i = 0; i <= 40; ++i) plate_axis.push_back(real(i) / real(40.0));
        SimulatedSteadyTables t;
        t.steady_wot_nm.x = rpm_axis;
        t.steady_wot_nm.y.assign(rpm_axis.size(), real(0.0));
        t.steady_motoring_nm.x = rpm_axis;
        t.steady_motoring_nm.y.assign(rpm_axis.size(), real(0.0));
        t.steady_torque_nm.x = rpm_axis;
        t.steady_torque_nm.y = plate_axis;
        t.steady_torque_nm.values.assign(rpm_axis.size() * plate_axis.size(), real(0.0));
        if (desc.throttle_input == ThrottleInput::Electronic) {
            constexpr int kReq = 20;
            t.inverse_plate.x = rpm_axis;
            for (int i = 0; i <= kReq; ++i) t.inverse_plate.y.push_back(real(i) / real(kReq));
            t.inverse_plate.values.assign(rpm_axis.size() * t.inverse_plate.y.size(), real(0.0));
        }
        fill_steady_tables(*m, amb, desc.idle_rpm, desc.idle.max_throttle, desc.idle.max_torque_nm, t, true);
        m->steady_wot_nm = std::move(t.steady_wot_nm);
        m->steady_motoring_nm = std::move(t.steady_motoring_nm);
        m->steady_torque_nm = std::move(t.steady_torque_nm);
        m->inverse_plate = std::move(t.inverse_plate);
        m->idle_ff = t.idle_ff;
        m->idle_p_im = t.idle_p_im;
        m->idle_cap_x = t.idle_cap_x;
    }

    // Content hash over every number the model reads (I7c interface request 2).
    Fnv f{14695981039346656037ULL};
    f.pod(static_cast<int>(m->throttle_input));
    f.pod(m->v_disp_m3);
    f.pod(m->c_cycle);
    f.pod(m->v_im_m3);
    f.pod(m->a_throttle_full_m2);
    f.pod(m->cd);
    f.pod(m->a_bypass_m2);
    f.curve(m->area_fraction);
    f.table(m->ecu.lambda_target);
    f.table(m->ecu.spark_advance_deg);
    f.table(m->maps.ve);
    f.table(m->maps.eta_i);
    f.curve(m->maps.f_lambda);
    f.curve(m->maps.eta_spark);
    f.table(m->maps.theta_mbt_deg);
    f.curve(m->maps.friction_torque_nm);
    f.pod(m->maps.has_pmep_excess);
    if (m->maps.has_pmep_excess) f.table(m->maps.pmep_excess_pa);
    f.pod(m->maps.has_theta_kl);
    if (m->maps.has_theta_kl) f.table(m->maps.theta_kl_deg);
    f.pod(m->lhv_j_kg);
    f.pod(m->afr_stoich);
    f.pod(m->c_k);
    f.pod(static_cast<int>(m->limiter.type));
    f.pod(m->limiter.rpm);
    f.pod(m->limiter.hysteresis_rpm);
    f.pod(m->limiter.band_rpm);
    f.vec(m->psi);
    f.pod(m->ref_p_pa);
    f.pod(m->ref_t_k);
    f.curve(m->steady_wot_nm);
    f.curve(m->steady_motoring_nm);
    f.table(m->steady_torque_nm);
    f.table(m->inverse_plate);
    f.pod(m->idle_ff);
    f.pod(m->idle_p_im);
    // Owner 2026-09-29: folded only when set, so engines without it hash as before.
    if (desc.idle.max_torque_nm > real(0.0)) f.pod(desc.idle.max_torque_nm);
    m->content_hash = f.h;
    return m;
}

SimulatedEngine::SimulatedEngine(const SimulatedEngineDesc& desc) : SimulatedEngine(desc, build_maps(desc)) {}

SimulatedEngine::SimulatedEngine(const SimulatedEngineDesc& desc, std::shared_ptr<const SimulatedEngineMaps> maps)
    : maps_(std::move(maps)) {
    if (!maps_) throw std::runtime_error("SimulatedEngine: null maps");
    if (desc.architecture != EngineArchitecture::Piston4Stroke) {
        throw std::runtime_error("SimulatedEngine: rotary architecture is not implemented yet (I7e C13)");
    }
    if (desc.turbo_pair) turbo_.emplace(*desc.turbo_pair);
    desc_idle_rpm_ = desc.idle_rpm;
    stall_rpm_ = desc.stall_rpm;
    fire_rpm_ = desc.fire_rpm;
    start_complete_rpm_ = desc.start_complete_rpm;
    dfco_enable_rpm_ = desc.dfco_enable_rpm;
    dfco_reenable_rpm_ = desc.dfco_reenable_rpm;
    idle_ = desc.idle;
    starter_ = desc.starter;
    breakaway_torque_nm_ = desc.breakaway_torque_nm;
    actuator_tau_s_ = desc.throttle_input == ThrottleInput::Electronic ? desc.throttle_actuator_tau_s : real(0.0);
    // C12: the per-engine derived tables start as a copy of the shared
    // maps' own (built at ref_p_pa/ref_t_k = ISA), so set_ambient(ISA)
    // below refills nothing.
    steady_.p_pa = maps_->ref_p_pa;
    steady_.t_k = maps_->ref_t_k;
    steady_.steady_wot_nm = maps_->steady_wot_nm;
    steady_.steady_motoring_nm = maps_->steady_motoring_nm;
    steady_.steady_torque_nm = maps_->steady_torque_nm;
    steady_.inverse_plate = maps_->inverse_plate;
    steady_.idle_ff = maps_->idle_ff;
    steady_.idle_p_im = maps_->idle_p_im;
    steady_.idle_cap_x = maps_->idle_cap_x;
    set_ambient(kIsaPressurePa, kIsaTemperatureK);
    state_ = desc.initial_state == EngineInitialState::Running ? EngineState::Running : EngineState::Off;
    init_state_for(state_);
}

void SimulatedEngine::set_ambient(real p_pa, real t_k) {
    if (!(std::isfinite(p_pa) && p_pa > real(0.0) && std::isfinite(t_k) && t_k > real(0.0))) return;
    p_amb_ = p_pa;
    t_amb_ = t_k;
    r_t_ = kGasConstant * t_k;
    inv_sqrt_rt_ = real(1.0) / std::sqrt(r_t_);
    if (p_pa != steady_.p_pa || t_k != steady_.t_k) {
        fill_steady_tables(*maps_, make_ambient(p_pa, t_k), desc_idle_rpm_, idle_.max_throttle, idle_.max_torque_nm, steady_, false);
        steady_.t_k = t_k;
        ++steady_rebuilds_;
    }
}

void SimulatedEngine::set_live_ambient(real p_pa,real t_k) {
    if(!(std::isfinite(p_pa)&&p_pa>0&&std::isfinite(t_k)&&t_k>0)) return;
    p_amb_=p_pa; t_amb_=t_k; r_t_=kGasConstant*t_k; inv_sqrt_rt_=real(1)/std::sqrt(r_t_);
    if(live_steady()) {
        // Stack-only scalar output; no table vectors are populated or allocated.
        SimulatedSteadyTables idle;
        fill_idle_control(*maps_,make_ambient(p_pa,t_k),desc_idle_rpm_,idle_.max_throttle,idle_.max_torque_nm,idle,24,ecu_k_);
        live_idle_ff_=idle.idle_ff; live_idle_p_im_=idle.idle_p_im; live_idle_cap_x_=idle.idle_cap_x;
    }
}

real SimulatedEngine::live_steady_torque(real rpm,real plate,bool fired) const {
    const real clamped_rpm=std::clamp(rpm,steady_.steady_torque_nm.x.front(),steady_.steady_torque_nm.x.back());
    return steady_net_torque(*maps_,make_ambient(p_amb_,t_amb_),StarterDesc{},clamped_rpm,plate,real(0),fired,ecu_k_);
}

real SimulatedEngine::live_inverse_plate(real rpm,real request) const {
    if(request<=0) return 0;
    if(request>=1) return 1;
    const real mot=live_steady_torque(rpm,0,false), wot=live_steady_torque(rpm,1,true);
    const real target=mot+request*(wot-mot);
    if(live_steady_torque(rpm,0,true)>=target) return 0;
    real lo=0, hi=1;
    // Plate uncertainty <= 2^-24: no whole RPM/request grid and no altitude hysteresis.
    for(int i=0;i<24;++i) {
        const real mid=(lo+hi)*real(.5);
        if(live_steady_torque(rpm,mid,true)<target) lo=mid; else hi=mid;
    }
    return (lo+hi)*real(.5);
}

void SimulatedEngine::replace_turbo_configuration(const TurboPairDesc& desc) {
    if(!desc.maps) throw std::runtime_error("replace_turbo_configuration: missing prepared maps");
    if(turbo_) {
        // Preserve shaft angular speed, manifold and charge states; replacing
        // hardware does not reset the crank or teleport the car.
        const real omega=turbo_->omega();
        turbo_->desc=desc;
        if(!desc.blow_off_valve) { turbo_->bov_opening=real(0.0); turbo_->bov_flow_kg_s=real(0.0); turbo_->bov_latched=false; }
        if(!desc.anti_lag) clear_anti_lag_state(); // I7d D5: a configuration without anti-lag starts from rest
        turbo_->energy_j=real(0.5)*desc.rotor_inertia_kgm2*omega*omega;
    } else {
        turbo_.emplace(desc);
        turbo_->reset(p_amb_,t_amb_);
    }
}

void SimulatedEngine::request_replace_maps(std::shared_ptr<const SimulatedEngineMaps> new_maps) {
    if (!new_maps) throw std::runtime_error("SimulatedEngine::request_replace_maps: null maps");
    // Mirror a fresh engine on new_maps (constructor + set_ambient): start
    // from the maps' own ISA tables, refill iff the current ambient differs.
    // Caller thread, between steps (allocates: copy-assignment may grow the
    // vectors the first time) - see the header comment.
    SimulatedSteadyTables& t = pending_steady_;
    t.p_pa = new_maps->ref_p_pa;
    t.t_k = new_maps->ref_t_k;
    t.steady_wot_nm = new_maps->steady_wot_nm;
    t.steady_motoring_nm = new_maps->steady_motoring_nm;
    t.steady_torque_nm = new_maps->steady_torque_nm;
    t.inverse_plate = new_maps->inverse_plate;
    t.idle_ff = new_maps->idle_ff;
    t.idle_p_im = new_maps->idle_p_im;
    t.idle_cap_x = new_maps->idle_cap_x;
    if (p_amb_ != t.p_pa || t_amb_ != t.t_k) {
        fill_steady_tables(*new_maps, make_ambient(p_amb_, t_amb_), desc_idle_rpm_, idle_.max_throttle, idle_.max_torque_nm, t, false);
        t.t_k = t_amb_;
    }
    pending_maps_ = std::move(new_maps);
}

std::shared_ptr<const SimulatedEngineMaps> SimulatedEngine::apply_pending_maps_swap() {
    if (!pending_maps_) return nullptr;
    const bool identical=maps_->content_hash==pending_maps_->content_hash;
    std::shared_ptr<const SimulatedEngineMaps> old = std::move(maps_);
    maps_ = std::move(pending_maps_);
    pending_maps_.reset();
    // Buffer exchange only: steady_ takes the tables prefilled from the new
    // maps by request_replace_maps; pending_steady_ keeps the old buffers
    // (same sizes) for the next request to overwrite. No refill here.
    if(!identical) {
        std::swap(steady_, pending_steady_);
        if(live_steady()) set_live_ambient(p_amb_,t_amb_); // request may have preceded a newer atmospheric sample
    } // Identical maps must preserve the control cache/algorithm and remain invisible at any altitude.
    return old;
}

void SimulatedEngine::init_state_for(EngineState state) {
    const SimulatedEngineMaps& m = *maps_;
    if (state == EngineState::Running) {
        p_im_ = live_steady()?live_idle_p_im_:steady_.idle_p_im;
        if (m.throttle_input == ThrottleInput::Mechanical) {
            x_thr_ = real(0.0);
            x_iac_ = live_steady()?live_idle_ff_:steady_.idle_ff;
        } else {
            x_thr_ = live_steady()?live_idle_ff_:steady_.idle_ff;
            x_iac_ = real(0.0);
        }
    } else {
        p_im_ = p_amb_;
        x_thr_ = real(0.0);
        x_iac_ = real(0.0);
    }
}

void SimulatedEngine::reset(EngineState state) {
    state_ = state;
    if (turbo_) turbo_->reset(p_amb_,t_amb_);
    turbo_air_kg_s_=turbo_exhaust_kg_s_=turbo_port_k_=real(0.0);
    exhaust_split_ = ExhaustSplit{};
    clear_anti_lag_state();
    idle_integrator_ = real(0.0);
    limiter_cut_ = false;
    dfco_active_ = false;
    init_state_for(state);
}

real SimulatedEngine::psi(real pr, real* dpsi_dpr) const { return psi_lookup(maps_->psi, pr, dpsi_dpr); }

SimulatedEngine::ManifoldRhs SimulatedEngine::manifold_rhs(real p_im, real omega, real x_thr, real x_iac) const {
    if (turbo_) {
        const real rt = kGasConstant*turbo_->charge_k;
        return manifold_rhs_impl(*maps_, Ambient{turbo_->charge_pa,rt,real(1.0)/std::sqrt(rt)},p_im,omega,x_thr,x_iac);
    }
    return manifold_rhs_impl(*maps_, Ambient{p_amb_, r_t_, inv_sqrt_rt_}, p_im, omega, x_thr, x_iac);
}

SimulatedEngine::TorqueEval SimulatedEngine::evaluate_torque(real omega, real p_im, bool fired, real s_starve, bool starter_active,
                                                             const AntiLagDesc* als) const {
    if (turbo_) {
        const real rt = kGasConstant*turbo_->charge_k;
        auto result = evaluate_torque_impl(*maps_,Ambient{turbo_->exhaust_pa,rt,real(1.0)/std::sqrt(rt)},starter_,omega,p_im,fired,s_starve,starter_active,
                                           false,als,plant_k_);
        return result;
    }
    return evaluate_torque_impl(*maps_, Ambient{p_amb_, r_t_, inv_sqrt_rt_}, starter_, omega, p_im, fired, s_starve, starter_active,
                                false, als, plant_k_);
}

void SimulatedEngine::clear_anti_lag_state() {
    als_active_ = false;
    als_armed_ = true;
    als_timer_s_ = real(0.0);
    als_extra_w_ = real(0.0);
}

PreSolveResult SimulatedEngine::pre_solve(const Sensors& sensors, const ThrottleCommand& throttle, const TcuRequest& tcu, real h) {
    const SimulatedEngineMaps& m = *maps_;
    const bool mech = m.throttle_input == ThrottleInput::Mechanical;
    // Physical atmosphere follows every finite change, including microscopic altitude motion.
    // Control uses the current operating point; never refill the whole steady grid in a job.
    // Engine thermal plan T1: the plant's friction scale acts at once; a changed ECU estimate (once per 1 K of oil
    // temperature, quantised by EngineThermal) recomputes the live idle control in place - never the steady grid.
    plant_k_ = sensors.friction_scale;
    const bool ecu_k_changed = sensors.ecu_friction_scale != ecu_k_;
    ecu_k_ = sensors.ecu_friction_scale;
    if (sensors.ambient.pressure_pa != p_amb_ || sensors.ambient.temperature_k != t_amb_) {
        set_live_ambient(sensors.ambient.pressure_pa, sensors.ambient.temperature_k);
    } else if (ecu_k_changed) {
        set_live_ambient(p_amb_, t_amb_);
    }
    const real omega_old = sensors.omega;
    omega_old_ = omega_old;
    const real rpm = std::abs(omega_old) * kRadPerSecToRpm;

    // (1) state machine - EngineTorqueMap's, verbatim in semantics.
    const EngineState prev_state = state_;
    EngineState next = state_;
    if (sensors.ignition <= real(0.5)) {
        next = EngineState::Off;
    } else {
        switch (state_) {
            case EngineState::Off:
                next = EngineState::Stalled;
                break;
            case EngineState::Stalled: {
                const bool interlock_ok = starter_.interlock == StarterDesc::Interlock::None ||
                                           (sensors.gearbox_in_neutral || sensors.clutch_engaged_fraction < real(0.1));
                if (sensors.starter > real(0.5) && interlock_ok) {
                    next = EngineState::Cranking;
                } else if (omega_old > real(0.0) && rpm >= start_complete_rpm_) {
                    next = EngineState::Running; // bump start
                }
                break;
            }
            case EngineState::Cranking:
                if (rpm >= start_complete_rpm_) next = EngineState::Running;
                else if (sensors.starter <= real(0.5)) next = EngineState::Stalled;
                break;
            case EngineState::Running:
                if (rpm < stall_rpm_ || omega_old < real(0.0)) next = EngineState::Stalled;
                break;
        }
    }
    if (next != prev_state && (next == EngineState::Off || next == EngineState::Stalled || next == EngineState::Cranking)) {
        idle_integrator_ = real(0.0);
    }
    state_ = next;
    const bool starter_active = (state_ == EngineState::Cranking);

    // (2) engine management. theta_req is the torque request (electronic)
    // or, on a mechanical linkage, only the DFCO trigger - the plate itself
    // follows the command below and is not closed by the ignition cut.
    const real theta_cmd = std::max(std::min(throttle.command, throttle.ceiling), throttle.floor);
    const real theta_req = tcu.ignition_cut ? real(0.0) : theta_cmd;

    // I7d D5 anti-lag: decided on the ECU's command theta_cmd (an upshift ignition cut does not disarm it -
    // the cut itself still suppresses combustion below). Arms while the command is at or above
    // arm_below_command; below it, acts while every condition holds, for at most max_active_s per lift
    // (round(max_active_s/h) substeps: the half-step margin absorbs the rounding of the summed timer).
    // Starvation disarms it for the rest of the lift (below), so a dry tank cannot toggle it every substep.
    const AntiLagDesc* als = (turbo_ && turbo_->desc.anti_lag) ? &*turbo_->desc.anti_lag : nullptr;
    if (als) {
        if (theta_cmd >= als->arm_below_command) {
            als_armed_ = true;
            als_timer_s_ = real(0.0);
            als_active_ = false;
        } else {
            const real ch = sensors.anti_lag_channel;
            const bool requested = std::isnan(ch) ? als->default_enabled >= real(0.5) : ch >= real(0.5);
            const bool ok = requested && state_ == EngineState::Running && rpm >= als->min_rpm && rpm <= als->max_rpm;
            bool active = false;
            if (ok && als_armed_) {
                if (als_timer_s_ + real(0.5) * h <= als->max_active_s) {
                    active = true;
                    als_timer_s_ += h;
                } else {
                    als_armed_ = false;
                }
            }
            if (active && !als_active_) ++als_activations_;
            als_active_ = active;
        }
    }

    if (state_ == EngineState::Running && theta_req < real(0.01) && rpm > dfco_enable_rpm_ && !als_active_) {
        dfco_active_ = true;
    } else if (theta_req >= real(0.01) || rpm < dfco_reenable_rpm_ || als_active_) {
        dfco_active_ = false;
    }

    // Idle PI in ACTUATOR units: plate position (electronic) or bypass
    // valve opening (mechanical); feed-forward = maps.idle_ff.
    const real ff = live_steady()?live_idle_ff_:steady_.idle_ff;
    // Authority cap (IdleControlDesc::max_torque_nm, owner 2026-09-29): enough
    // to creep in gear, not a hidden launch aid.
    const real idle_cap=live_steady()?live_idle_cap_x_:steady_.idle_cap_x;
    const real cap = idle_cap > real(0.0) ? std::min(idle_cap, idle_.max_throttle) : idle_.max_throttle;
    real theta_idle = real(0.0);
    if ((state_ == EngineState::Running || state_ == EngineState::Cranking) && !dfco_active_) {
        // One-sided around the model's own no-load feed-forward (owner
        // 2026-09-29: never less air than idle needs): P and I only ADD air
        // for a load; above idle the feed-forward alone brings the engine
        // down (its net torque falls with rpm at a fixed position). A
        // two-sided P closed the plate on the way down and dipped below idle.
        // With the pedal pressed the pre-2026-09-29 two-sided law stays: the
        // idle air backs off above idle so part-throttle torque is the pedal's.
        const real e = desc_idle_rpm_ - rpm;
        if (theta_req < real(0.01)) {
            theta_idle = std::clamp(ff + idle_.kp_per_rpm * std::max(e, real(0.0)) + idle_integrator_, real(0.0), std::max(cap, ff));
            idle_integrator_ = std::clamp(idle_integrator_ + idle_.ki_per_rpm_s * e * h, real(0.0), std::max(cap - ff, real(0.0)));
        } else {
            theta_idle = std::clamp(ff + idle_.kp_per_rpm * e + idle_integrator_, real(0.0), std::max(cap, ff));
            idle_integrator_ = std::clamp(idle_integrator_ + idle_.ki_per_rpm_s * e * h, -ff, std::max(cap - ff, real(0.0)));
        }
    } else if (state_ == EngineState::Running && dfco_active_) {
        // Dashpot (owner 2026-09-29, "as soon as the engine reaches idle, the
        // ECU should inject just enough fuel and air to keep the idle rpm"):
        // fuel cut, idle air kept, so the manifold is at its idle-plate
        // pressure when fuelling resumes. A closed plate here emptied the
        // plenum and the engine fell ~260 rpm below idle after every lift.
        idle_integrator_ = real(0.0);
        theta_idle = std::min(ff, idle_.max_throttle);
    } else {
        idle_integrator_ = real(0.0);
        theta_idle = real(0.0);
    }
    // Manifold-fill compensation (the ECU's air-charge dynamics): with the
    // pedal released the idle position theta_idle sets the manifold's
    // TARGET; the actuator is driven to x with f(p, x) = G*f(p, theta_idle),
    // so the plenum reaches theta_idle's steady pressure G times faster and x
    // returns to theta_idle there (f = 0 <=> f = 0). Without it the 3 L
    // plenum (~0.3 s at idle) lagged the falling rpm and the engine dipped
    // below idle after fuelling resumed.
    real x_idle = theta_idle;
    if (theta_idle > real(0.0) && theta_req < real(0.01)) {
        const real xp = mech ? theta_cmd : real(0.0);
        auto f_at = [&](real xi) {
            return mech ? manifold_rhs(p_im_, omega_old, xp, xi).f : manifold_rhs(p_im_, omega_old, xi, real(0.0)).f;
        };
        const real f0 = f_at(theta_idle);
        const real dfdx = (f_at(theta_idle + kIdleFillDx) - f0) / kIdleFillDx;
        if (dfdx > real(0.0)) {
            x_idle = std::clamp(theta_idle + (kIdleFillGain - real(1.0)) * f0 / dfdx, real(0.0), idle_.max_throttle);
        }
    }

    bool limiter_active = false;
    if (m.limiter.type == LimiterType::FuelCut) {
        if (rpm >= m.limiter.rpm) limiter_cut_ = true;
        else if (rpm <= m.limiter.rpm - m.limiter.hysteresis_rpm) limiter_cut_ = false;
        limiter_active = limiter_cut_;
    } else {
        limiter_active = rpm > m.limiter.rpm - m.limiter.band_rpm;
    }

    // (3) actuator
    if (mech) {
        x_thr_ = theta_cmd;
        if (als_active_) x_thr_ = std::max(x_thr_, als->plate_floor); // I7d D5: anti-lag throttle kicker
        x_iac_ = x_idle;
    } else {
        real x_target = std::max(live_steady()?live_inverse_plate(rpm,theta_req):steady_.inverse_plate.value(rpm, theta_req), x_idle);
        if (als_active_) x_target = std::max(x_target, als->plate_floor); // I7d D5: anti-lag bypass air
        if (!(actuator_tau_s_ > real(0.0))) {
            x_thr_ = x_target;
        } else {
            if (h != act_h_) {
                act_h_ = h;
                act_a_ = h / (actuator_tau_s_ + h);
            }
            x_thr_ = x_thr_ + act_a_ * (x_target - x_thr_);
        }
        x_iac_ = real(0.0);
    }

    if (turbo_) {
        // I7d D5: an anti-lag boost target holds the wastegate PID toward boost_target_fraction of the target
        // boost while active (a plate below 10 % would otherwise command it open).
        const real boost_plate = (als_active_ && als->boost_target_fraction > x_thr_) ? als->boost_target_fraction : x_thr_;
        turbo_->step(h,p_amb_,t_amb_,turbo_air_kg_s_,turbo_exhaust_kg_s_,turbo_port_k_,boost_plate,p_im_,&maps_->psi);
    }

    // (4) air step, omega frozen at omega_old
    {
        const real p0 = p_im_;
        const ManifoldRhs rhs = manifold_rhs(p0, omega_old, x_thr_, x_iac_);
        const real den = std::max(real(1.0), real(1.0) - h * rhs.dfdp);
        real p1 = std::max(p0 + h * rhs.f / den, real(1e-3) * p_amb_);
        // Equilibrium guard (provisional decision P-C11-AIR): with omega
        // frozen the exact solution is monotone and never passes through the
        // point where f = 0. The tangent step can (f is concave in p - Psi
        // steepens toward pr = 1 - so a large rising step at a small plenum
        // overshoots, even above p_amb). If f changed sign across the step,
        // take the chord root of f between p0 and p1 instead (for a concave
        // f it lies on p0's side of the root). One extra rhs evaluation, no
        // iteration; a no-op in the linear regime (den >= 1 never crosses).
        const real f1 = manifold_rhs(p1, omega_old, x_thr_, x_iac_).f;
        if ((rhs.f > real(0.0) && f1 < real(0.0)) || (rhs.f < real(0.0) && f1 > real(0.0))) {
            p1 = p0 + (p1 - p0) * (rhs.f / (rhs.f - f1));
        }
        p_im_ = p1;
        // I7d compressor-flow fix: the compressor delivers into the charge pipe, which feeds the THROTTLE; the
        // cylinders draw from the manifold behind it. The turbo therefore sees the throttle-path flow of this
        // air step (m_thr = throttle plate + idle-air bypass, the whole path from the charge pipe into the
        // manifold), not the cylinder flow: on a lift-off the throttle flow collapses at once while the
        // cylinders keep pumping the manifold down for ~100 ms. Positive part only: reverse flow out of the
        // manifold does not pass the compressor forward. Read by the NEXT substep's turbo step (the same
        // one-substep hand-off the cylinder flow had; the turbo step above runs before this air step).
        // Relaxed with kTurboFlowLagS before it is handed over (PROVISIONAL, see the constant): fed raw, the
        // loop charge pressure -> throttle flow -> compressor flow -> head curve -> charge pressure has a
        // one-substep delay and a gain above 1 near the end of the compressor map, and a held 7500 rpm WOT
        // ran away through a period-2 oscillation (rotor to its 1.5x cap, 3.2 bar).
        if (turbo_) {
            const real a = h / (kTurboFlowLagS + h);
            turbo_air_kg_s_ += a * (std::max(real(0.0), rhs.m_thr) - turbo_air_kg_s_);
        }
    }

    // (5) torque at (omega_old, p_im_new), then starvation
    const bool comb = (state_ == EngineState::Running || state_ == EngineState::Cranking) && rpm >= fire_rpm_ && omega_old > real(0.0);
    const bool fired = comb && !dfco_active_ && !tcu.ignition_cut && !(m.limiter.type == LimiterType::FuelCut && limiter_cut_);
    const AntiLagDesc* als_eval = als_active_ ? als : nullptr; // I7d D5
    TorqueEval ev = evaluate_torque(omega_old, p_im_, fired, real(1.0), starter_active, als_eval);
    const real rev_rate = std::max(omega_old, real(0.0)) / kTwoPi;
    real s_starve = real(1.0);
    booked_fuel_mass_kg_ = h * ev.m_fuel_rev * rev_rate;
    if (booked_fuel_mass_kg_ > sensors.fuel_available_kg) {
        const real available = std::max(real(0.0), sensors.fuel_available_kg);
        s_starve = available / booked_fuel_mass_kg_;
        booked_fuel_mass_kg_ = available;
        ev = evaluate_torque(omega_old, p_im_, fired, s_starve, starter_active, als_eval);
    }
    if (als) {
        // I7d D5 energy routing: the indicated work the retard removes this substep, as a rate at the speed
        // the fuel is metered and the indicated work is booked at (post_solve books h*omega_old*tau_comb).
        // It stays inside fuel_energy_in - indicated_work (engine_heat); the exhaust energy balance below
        // puts it into the exhaust sensible enthalpy (a diagnostic here, not an extra term).
        als_extra_w_ = std::max(omega_old, real(0.0)) * (ev.tau_comb_unretarded - ev.tau_comb);
        if (als_active_ && s_starve < real(1.0)) als_armed_ = false; // starving: no more anti-lag this lift
    }
    // Engine thermal plan T1 (4.1): the share of the metered fuel energy released in the cylinder
    // (oxygen-limited, [AS] as the exhaust balance below), less an anti-lag cut share (burnt in the manifold).
    eta_c_ = (ev.lambda > real(0.0) ? std::min(real(1.0), ev.lambda) : real(0.0)) * (real(1.0) - ev.cut_fraction);
    tau_comb_ = ev.tau_comb;
    tau_pump_ = ev.tau_pump;
    tau_fric_ = ev.tau_fric;
    tau_start_ = ev.tau_start;
    tau0_ = ev.tau;
    d_ = std::max(-ev.dtau_domega, real(0.0));

    PreSolveResult r;
    r.tau0 = tau0_;
    r.d = d_;
    r.friction_cap_static = breakaway_torque_nm_ * plant_k_; // engine thermal plan T1: the Coulomb part scales with k
    r.friction_cap_kinetic = m.c_k * plant_k_;
    r.state = state_;
    r.rpm = rpm;
    r.throttle_eff = x_thr_;
    r.limiter_active = limiter_active;
    r.dfco_active = dfco_active_;
    r.starter_active = starter_active;
    r.torque_estimate = tau0_ - tau_start_;
    r.governor_setpoint_rpm = real(0.0);
    {
        const real s = sign(omega_old);
        const real mot = live_steady()?live_steady_torque(rpm,0,false):steady_.steady_motoring_nm.evaluate(rpm);
        if (mech) {
            r.torque_demand = comb ? (live_steady()?live_steady_torque(rpm,x_thr_,true):steady_.steady_torque_nm.value(rpm, x_thr_)) : s * mot;
        } else {
            const real wot = live_steady()?live_steady_torque(rpm,1,true):steady_.steady_wot_nm.evaluate(rpm);
            real demand = s * mot + (comb ? theta_req : real(0.0)) * (wot - mot);
            if (comb && !dfco_active_ && theta_idle > real(0.0)) demand = std::max(demand, live_steady()?live_steady_torque(rpm,theta_idle,true):steady_.steady_torque_nm.value(rpm, theta_idle));
            r.torque_demand = demand;
        }
    }
    // The ambient steady tables do not include a spinning turbo's charge.
    // Carry present boosted torque without feeding rev-limiter pulses back
    // into clutch capacity (that would trap a flare on the limiter). Explicit
    // shift cuts and DFCO still reduce torque for the bounded rolling close.
    // Retain the steady request during spool-up.
    if (turbo_ && comb && !dfco_active_ && !tcu.ignition_cut) {
        const real rt = kGasConstant * turbo_->charge_k;
        const auto uncut = evaluate_torque_impl(m,
            Ambient{turbo_->exhaust_pa, rt, real(1.0) / std::sqrt(rt)},
            starter_, omega_old, p_im_, true, real(1.0), false, true, als_eval, ecu_k_);
        r.torque_demand = std::max(r.torque_demand, uncut.tau - sign(omega_old) * (ecu_k_ * m.c_k));
    }
    r.theta_c = std::clamp(ev.fire * s_starve * p_im_ / p_amb_, real(0.0), real(1.0));
    r.fuel_flow_g_s = booked_fuel_mass_kg_ / h * real(1000.0);
    r.fuel_supply_fraction = s_starve;
    r.throttle_position = x_thr_;
    r.p_intake_pa = p_im_;
    r.lambda = ev.lambda;
    r.spark_advance_deg = ev.spark_deg;
    r.exhaust_mass_flow_kg_s = ev.m_air_rev * std::abs(omega_old) / kTwoPi + booked_fuel_mass_kg_ / h;
    if (turbo_) {
        turbo_exhaust_kg_s_ = r.exhaust_mass_flow_kg_s;
        // Exhaust energy balance (exhaust_thermal, turbo.md): of the fuel energy that entered the cylinders
        // this substep, the share oxygen allows to burn (eta_c = min(1, lambda), [AS] ideal oxygen-limited
        // combustion) is released; it leaves as indicated work (omega_old*tau_comb, exactly the rate post_solve
        // books), in-cylinder wall heat (in_cylinder_wall_heat_fraction of the released energy) and exhaust
        // sensible enthalpy (the rest); the unreleased share leaves as chemical enthalpy (CO/H2, not heat).
        // A spark retard (anti-lag) lowers tau_comb and so raises the sensible part by exactly the work it
        // removes. The port gas temperature is that enthalpy over the actual exhaust flow (air + fuel):
        // bounded by construction - a kg of gas carries at most its own fuel's energy - so no cap is needed.
        // Pumping work done on the gas is not included (PROVISIONAL). Handed to the NEXT substep's turbo step
        // (the same one-substep hand-off the flows have), which passes it through the exhaust wall.
        {
            const TurboPairDesc& td = turbo_->desc;
            ExhaustSplit x;
            x.fuel_w = booked_fuel_mass_kg_ / h * maps_->lhv_j_kg;
            const real eta_c = ev.lambda > real(0.0) ? std::min(real(1.0), ev.lambda) : real(0.0);
            x.chemical_w = (real(1.0) - eta_c) * x.fuel_w;
            real released = x.fuel_w - x.chemical_w;
            if (ev.cut_fraction > real(0.0)) {
                // Anti-lag cylinder cut: the cut share of the charge passes the cylinder unburnt; in the hot
                // manifold manifold_burn_fraction of its oxygen-limited energy is released (the mixture is the
                // same lambda as the fired charge), the rest stays chemical. Wall heat: fired cylinders only.
                const real cut_released = ev.cut_fraction * released;
                x.manifold_w = als_eval->manifold_burn_fraction * cut_released;
                x.chemical_w += cut_released - x.manifold_w;
                released -= cut_released;
            }
            x.indicated_w = std::max(omega_old, real(0.0)) * ev.tau_comb;
            x.wall_w = td.in_cylinder_wall_heat_fraction * released;
            const real sensible = released - x.indicated_w - x.wall_w + x.manifold_w;
            x.sensible_w = std::max(sensible, real(0.0));
            x.clamp_w = x.sensible_w - sensible; // > 0 only if the maps claim more work than the released energy allows
            const real t_in = turbo_->charge_k;
            x.port_k = t_in;
            if (r.exhaust_mass_flow_kg_s > real(0.0))
                x.port_k = t_in + x.sensible_w / (TurboPair::kExhaustCp * r.exhaust_mass_flow_kg_s);
            exhaust_split_ = x;
            turbo_port_k_ = x.port_k;
        }
        r.exhaust_port_k = turbo_port_k_;
        r.turbine_inlet_k = turbo_->exhaust_k;
        r.exhaust_wall_k = turbo_->wall_k;
        if (als) {
            r.anti_lag_active = als_active_ ? std::uint8_t(1) : std::uint8_t(0);
            r.anti_lag_activations = als_activations_;
            r.anti_lag_extra_exhaust_w = als_extra_w_;
            r.anti_lag_manifold_burn_w = exhaust_split_.manifold_w;
        }
        r.turbo_rpm=turbo_->rpm(); r.boost_bar=(p_im_-p_amb_)/real(100000.0);
        r.p_exhaust_pa=turbo_->exhaust_pa;
        r.p_charge_pa=turbo_->charge_pa; r.t_charge_k=turbo_->charge_k;
        r.wastegate_opening=turbo_->wastegate; r.compressor_flow_kg_s=turbo_->compressor_flow_kg_s;
        r.compressor_pressure_ratio=turbo_->compressor_pr;
        r.bov_opening=turbo_->bov_opening; r.bov_flow_kg_s=turbo_->bov_flow_kg_s;
        r.bov_vent=turbo_->desc.blow_off_valve ? static_cast<std::uint8_t>(turbo_->desc.blow_off_valve->vent) : std::uint8_t(0);
        r.bov_events=turbo_->bov_events;
        r.surge_active=turbo_->surge_active ? std::uint8_t(1) : std::uint8_t(0); r.surge_events=turbo_->surge_events;
    }
    return r;
}

PostSolveResult SimulatedEngine::post_solve(real omega_new, real h) {
    const real tau_lin = tau0_ - d_ * (omega_new - omega_old_);
    const real work = h * omega_new * tau_lin;
    const real frac_starter = tau0_ != real(0.0) ? tau_start_ / tau0_ : real(0.0);

    PostSolveResult r;
    r.starter_in_delta = work * frac_starter;
    r.engine_out_delta = work - r.starter_in_delta;
    // At omega_old, the speed the booked fuel was metered at, so
    // indicated_work == eta_i*f_lambda*eta_spark*LHV*fuel_mass holds exactly
    // (T5). EngineTorqueMap uses omega_new (its fuel is not per revolution).
    r.indicated_work_delta = h * omega_old_ * tau_comb_;
    r.fuel_mass_delta = booked_fuel_mass_kg_;
    r.fuel_energy_in_delta = booked_fuel_mass_kg_ * maps_->lhv_j_kg;
    work_split_.indicated = r.indicated_work_delta;
    work_split_.pumping = h * omega_old_ * tau_pump_;
    work_split_.friction = h * omega_old_ * tau_fric_;
    r.pumping_work_delta = work_split_.pumping;
    r.friction_work_delta = work_split_.friction;
    // Engine thermal plan T1 (4.1/4.4): released combustion energy and the smooth friction work turned into heat
    // (tau_fric opposes motion, so -work >= 0 to rounding; booked as is so the thermal closure stays exact).
    r.released_heat_delta = eta_c_ * r.fuel_energy_in_delta;
    r.friction_heat_delta = -work_split_.friction;

    // std::fmod is exact (IEEE), like EngineTorqueMap's; folded into
    // [0, 4*pi). A subtraction loop here hung on a non-finite omega (found
    // by the dp/dt-sign sabotage). Sound only, never hashed.
    real a = std::fmod(crank_angle_ + omega_new * h, kFourPi);
    if (a < real(0.0)) a += kFourPi;
    crank_angle_ = (a >= real(0.0) && a < kFourPi) ? a : real(0.0);
    return r;
}

std::uint64_t SimulatedEngine::fold_state_hash(std::uint64_t seed) const {
    Fnv f{seed};
    f.pod(maps_->content_hash);
    // Non-reference setup/swap caches select tabulated control when their ambient matches.
    // This anchor is now persistent control state; include it so future divergence is visible.
    // Reference-ambient engines retain their existing hash byte layout.
    if(steady_.p_pa!=maps_->ref_p_pa || steady_.t_k!=maps_->ref_t_k) {
        f.pod(steady_.p_pa); f.pod(steady_.t_k);
    }
    f.pod(state_);
    f.pod(idle_integrator_);
    f.pod(limiter_cut_);
    f.pod(dfco_active_);
    f.pod(p_im_);
    f.pod(x_thr_);
    if (maps_->throttle_input == ThrottleInput::Mechanical) f.pod(x_iac_);
    if (turbo_) {
        f.pod(turbo_->energy_j); f.pod(turbo_->charge_pa); f.pod(turbo_->charge_k);
        f.pod(turbo_->exhaust_pa); f.pod(turbo_->wastegate); f.pod(turbo_->boost_integral);
        f.pod(turbo_air_kg_s_); f.pod(turbo_exhaust_kg_s_); f.pod(turbo_port_k_);
        f.pod(turbo_->wall_k); // exhaust thermal mass state
        const auto& t=turbo_->desc;
        f.pod(t.reference_head_pa);
        if(t.maps) f.pod(t.maps->content_hash);
        f.pod(t.target_boost_pa); f.pod(t.rotor_inertia_kgm2); f.pod(t.reference_rpm);
        f.pod(t.max_mass_flow_kg_s); f.pod(t.compressor_efficiency); f.pod(t.turbine_efficiency);
        f.pod(t.turbine_area_m2); f.pod(t.intercooler_effectiveness); f.pod(t.shaft_loss_w_at_reference);
        f.pod(t.wastegate_kp); f.pod(t.wastegate_ki_per_s); f.pod(t.wastegate_kd_s); f.pod(t.wastegate_actuator_tau_s);
        f.pod(t.in_cylinder_wall_heat_fraction); f.pod(t.exhaust_wall_heat_capacity_j_k);
        f.pod(t.exhaust_wall_gas_conductance_w_k); f.pod(t.exhaust_wall_ambient_conductance_w_k);
        f.pod(t.wastegate_area()); // parallel wastegate orifice (effective area, default ratio resolved)
        // I7d D4: appended and only when nonzero, so an engine without a cooler drop keeps its byte layout.
        if (t.intercooler_dp_pa_at_max_flow != real(0.0)) f.pod(t.intercooler_dp_pa_at_max_flow);
        // I7d D2: appended, so an engine without a valve keeps its byte layout.
        if (t.blow_off_valve) {
            f.pod(turbo_->bov_opening); f.pod(t.blow_off_valve->crack_dp_pa); f.pod(t.blow_off_valve->full_open_dp_pa);
            f.pod(t.blow_off_valve->effective_area_m2); f.pod(t.blow_off_valve->tau_s);
        }
        // I7d D5: appended, so an engine without anti-lag keeps its byte layout.
        if (t.anti_lag) {
            f.pod(als_active_); f.pod(als_armed_); f.pod(als_timer_s_);
            const auto& a = *t.anti_lag;
            f.pod(a.arm_below_command); f.pod(a.min_rpm); f.pod(a.max_rpm); f.pod(a.retard_deg);
            f.pod(a.plate_floor); f.pod(a.max_active_s);
            // ALS extension (cut, enrichment, absolute spark, boost target): appended only when used, so a
            // D5 block keeps its byte layout.
            if (a.has_extension()) {
                const real spark = a.spark_advance_deg ? *a.spark_advance_deg : real(0.0);
                f.pod(a.spark_advance_deg.has_value()); f.pod(spark); f.pod(a.cylinder_cut_fraction);
                f.pod(a.manifold_burn_fraction); f.pod(a.lambda); f.pod(a.boost_target_fraction);
            }
        }
    }
    return f.h;
}

} // namespace ps::drivetrain
