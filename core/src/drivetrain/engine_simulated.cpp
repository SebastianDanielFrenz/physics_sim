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
#include "hydrodynamic_friction.h"
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

// --- N2O plan N2: the wet charge model (out/n2o_plan.md section 1.2) --------------------------
// Constants (labels: [C] cited, [D] derived):
// - kAirMolarMassKgMol 28.96442 g/mol [C: ISO 2533:1975 Table 1, the standard atmosphere's dry air - the
//   same standard isa_air.h's R = 287.05 J/(kg K) comes from (8314.32/28.96442)].
// - kAirO2MassFraction 0.2314 [D: ISO 2533:1975 Table 1 dry-air composition, O2 volume fraction 0.20946,
//   M_O2 31.9988 g/mol: 0.20946 * 31.9988 / 28.96442 = 0.23140].
// - cp of air = R * gamma / (gamma - 1) = 1004.675 J/(kg K) [D from the engine's own R and gamma 1.4].
// The N2O side (molar mass, O2 fraction 0.3635, decomposition heat 1.8643 MJ/kg, the 1 bar gas enthalpy)
// comes from the oxidizer file through OxidizerTables.
constexpr real kAirMolarMassKgMol = 0.02896442;
constexpr real kAirO2MassFraction = 0.2314;
constexpr real kAirCp = SimulatedEngine::kGasConstant * SimulatedEngine::kGamma / (SimulatedEngine::kGamma - real(1.0));

// The air a (kg per crank revolution) the cylinder traps next to n kg of N2O, and the charge temperature.
// Two balances at the port (section 1.2 steps 1-2):
//   moles:  a + r*n = X / T_mix                       r = M_air/M_N2O, X = VE*p*V_c/R (kg K)
//   energy: a*cp_a*(T_in - T_mix) = n*(h_g(T_mix) - h_in)     (adiabatic constant-pressure mixing; the
//           evaporated N2O as an ideal gas at its low partial pressure [AS], h_g on the 1 bar isobar)
// With h_g(T) = h0 + cp_n*(T - t0) on one isobar segment, eliminating T_mix gives
//   A2*a^2 + A1*a + A0 = 0,  A2 = cp_a*T_in,  A1 = r*n*cp_a*T_in - n*B - X*cp_a,  A0 = -n*(r*n*B + X*cp_n),
//   B = h0 - cp_n*t0 - h_in,
// whose positive root is taken in closed form (one sqrt). The segment is the one holding T_mix: start at
// T_in's, move one segment toward the solution while it lies outside, stop on a reversal (h_g is
// continuous, so the root is unique). A0 >= 0 means the N2O alone fills the cylinder (step 9): a = 0.
// Derivatives by implicit differentiation of F(a, X, n) = 0 on the final segment.
struct N2oAir {
    real a = 0.0, t_mix = 0.0;
    real da_dx = 0.0, da_dn = 0.0;
    bool overflow = false;
};

N2oAir solve_n2o_air(real x, real n, real t_in, real h_in, const OxidizerTables& ox) {
    const real r = kAirMolarMassKgMol / ox.molar_mass_kg_mol();
    const std::size_t last = ox.gas_segment_count() - 1;
    std::size_t k = ox.gas_segment_index(t_in);
    int dir = 0;
    N2oAir s;
    for (std::size_t it = 0; it <= last + 1; ++it) {
        const OxidizerTables::GasSegment g = ox.gas_segment(k);
        const real b = g.h0 - g.cp * g.t0 - h_in;
        const real a2 = kAirCp * t_in;
        const real a1 = r * n * kAirCp * t_in - n * b - x * kAirCp;
        const real a0 = -n * (r * n * b + x * g.cp);
        if (!(a0 < real(0.0))) {
            s = N2oAir{};
            s.overflow = true;
            s.t_mix = g.cp > real(0.0) ? -b / g.cp : t_in;
            return s;
        }
        const real sq = std::sqrt(a1 * a1 - real(4.0) * a2 * a0);
        s.a = a1 <= real(0.0) ? (sq - a1) / (real(2.0) * a2) : (real(-2.0) * a0) / (a1 + sq);
        s.t_mix = x / (s.a + r * n);
        const real fa = real(2.0) * a2 * s.a + a1;
        const real fx = -kAirCp * s.a - n * g.cp;
        const real fn = (r * kAirCp * t_in - b) * s.a - (real(2.0) * r * n * b + x * g.cp);
        s.da_dx = -fx / fa;
        s.da_dn = -fn / fa;
        if (s.t_mix < g.t0 && k > 0 && dir <= 0) {
            --k;
            dir = -1;
            continue;
        }
        if (s.t_mix > g.t1 && k < last && dir >= 0) {
            ++k;
            dir = 1;
            continue;
        }
        break;
    }
    return s;
}

SimulatedEngine::ManifoldRhs manifold_rhs_impl(const SimulatedEngineMaps& m, const Ambient& amb, real p, real omega, real x_thr,
                                               real x_iac, const NitrousCharge* nc = nullptr) {
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
    real dm_cyl_dp = real(0.0);
    if (nc && omega > real(0.0) && nc->mdot_n_kg_s > real(0.0) && nc->oxidizer) {
        // N2O plan N2: the cylinders draw the charge model's air a per revolution (section 1.2 step 3), so
        // the plenum mass balance stays consistent with the torque evaluation; da/dp = da/dX * dX/dp.
        const real rev_rate = omega / kTwoPi;
        const real vc_r = m.v_disp_m3 * m.c_cycle / SimulatedEngine::kGasConstant;
        const N2oAir s = solve_n2o_air(ve * p * vc_r, nc->mdot_n_kg_s / rev_rate, amb.r_t / SimulatedEngine::kGasConstant,
                                       nc->h_in_j_kg, *nc->oxidizer);
        r.m_cyl = s.a * rev_rate;
        dm_cyl_dp = s.da_dx * (dve_dp * p + ve) * vc_r * rev_rate;
    } else {
        r.m_cyl = ve * p / amb.r_t * vol_rate;
        dm_cyl_dp = (dve_dp * p + ve) / amb.r_t * vol_rate;
    }
    const real c = amb.r_t / m.v_im_m3;
    r.f = c * (r.m_thr - r.m_cyl);
    r.dfdp = c * (dm_thr_dp - dm_cyl_dp);
    return r;
}

SimulatedEngine::TorqueEval evaluate_torque_impl(const SimulatedEngineMaps& m, const Ambient& amb, const StarterDesc& starter,
                                                 real omega, real p, bool fired, real s_starve, bool starter_active,
                                                 bool ignore_limiter = false, const AntiLagDesc* als = nullptr, real k_fric = real(1.0),
                                                 real hyd_ref_rpm = real(2000.0), const NitrousCharge* nc = nullptr,
                                                 real limiter_rpm_override = real(0.0)) {
    SimulatedEngine::TorqueEval e;
    const real rpm = std::abs(omega) * kRadPerSecToRpm;
    const real s_w = sign(omega);
    const real vc_2pi = m.v_disp_m3 * m.c_cycle / kTwoPi;
    // N2O plan N2: the wet charge model runs only with a charge input and a turning crank (n = mdot_n*2pi/omega);
    // otherwise every expression below is the pre-N2 one, bit for bit.
    const bool n2o = nc != nullptr && omega > real(0.0) && nc->mdot_n_kg_s > real(0.0) && nc->oxidizer != nullptr;

    // air per revolution (and its rpm slope at frozen p)
    const real ve = m.maps.ve.value(rpm, p);
    const real dve_drpm = m.maps.ve.dvalue_dx(rpm, p);
    const real air_per_ve = p / amb.r_t * m.v_disp_m3 * m.c_cycle;
    e.m_air_rev = ve * air_per_ve;
    const real dma_drpm = dve_drpm * air_per_ve;
    e.charge.t_in_k = amb.r_t / SimulatedEngine::kGasConstant;
    e.charge.t_mix_k = e.charge.t_in_k;
    e.charge.air_kg_rev = e.m_air_rev;

    // combustion
    real fire = real(0.0), dfire_drpm = real(0.0);
    if (fired) {
        if (m.limiter.type == LimiterType::SoftCut && !ignore_limiter) {
            // Engine garage P8 (A16): launch control's second limit replaces the limiter speed while it is below it.
            const real lim_rpm = (limiter_rpm_override > real(0.0) && limiter_rpm_override < m.limiter.rpm) ? limiter_rpm_override : m.limiter.rpm;
            fire = std::clamp((lim_rpm - rpm) / m.limiter.band_rpm, real(0.0), real(1.0));
            const bool in_band = rpm < lim_rpm && rpm > lim_rpm - m.limiter.band_rpm;
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
    // N2O plan D7: the kit's retard comes off the schedule BEFORE the knock limit, spark = min(schedule - retard,
    // theta_kl); a constant offset, so dspark_drpm is unchanged. Without N2O spark_sched IS spark_target.
    const real spark_sched = (n2o && nc->retard_deg != real(0.0)) ? spark_target - nc->retard_deg : spark_target;
    real spark = spark_sched;
    real dspark_drpm = m.ecu.spark_advance_deg.dvalue_dx(rpm, p);
    if (m.maps.has_theta_kl) {
        const real kl = m.maps.theta_kl_deg.value(rpm, p);
        if (kl < spark_sched) {
            spark = kl;
            dspark_drpm = m.maps.theta_kl_deg.dvalue_dx(rpm, p);
            e.knock_retard_deg = spark_sched - kl;
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
    // N2O plan N3 (retard heat): the applied spark WITHOUT the kit's retard - the knock-limited schedule (and the
    // anti-lag retard/setting, which the kit does not touch). Where the knock limit already holds the spark below
    // schedule - retard it equals spark_applied (the retard costs nothing). Used only to report the work the kit
    // retard removes (tau_comb_no_n2o_retard); the dynamics never read it.
    real spark_applied_nr = spark_applied;
    if (n2o && nc->retard_deg != real(0.0)) {
        real spark_nr = spark_target;
        if (m.maps.has_theta_kl) spark_nr = std::min(spark_nr, m.maps.theta_kl_deg.value(rpm, p));
        if (als && als->spark_advance_deg) spark_applied_nr = spark_applied;
        else if (als && als->retard_deg > real(0.0)) spark_applied_nr = spark_nr - als->retard_deg;
        else spark_applied_nr = spark_nr;
    }
    e.theta_mbt_deg = m.maps.theta_mbt_deg.value(rpm, p);
    const real dmbt_drpm = m.maps.theta_mbt_deg.dvalue_dx(rpm, p);

    // N2O plan N2 (section 1.2 steps 1-4): the charge. Per rev n = mdot_n*2pi/omega (dn/drpm = -n/rpm, the
    // constant-mass-flow source), kit fuel kf likewise; the air a and T_mix from the two port balances
    // (solve_n2o_air), a_eq = a + k_O2*n the O2-equivalent air, the ECU fuels the trapped air at its lambda
    // target [AS, owner Q8], lambda_c = a_eq / (AFR_st * (f_ecu + kf)). eta_i is read at a_eq (step 5 [AS]),
    // f_lambda at lambda_c. Without N2O the reads below use e.m_air_rev/dma_drpm/e.lambda as before.
    real eta_y = e.m_air_rev, deta_y_drpm = dma_drpm, lambda_fl = e.lambda;
    real n_rev = real(0.0), dn_drpm = real(0.0), kf_rev = real(0.0), dkf_drpm = real(0.0);
    real f_ecu = real(0.0), df_ecu_drpm = real(0.0), dlc_drpm = real(0.0);
    if (n2o) {
        const OxidizerTables& ox = *nc->oxidizer;
        n_rev = nc->mdot_n_kg_s * kTwoPi / omega;
        kf_rev = nc->mdot_kf_kg_s * kTwoPi / omega;
        dn_drpm = -n_rev / rpm;
        dkf_drpm = -kf_rev / rpm;
        const real vc_r = m.v_disp_m3 * m.c_cycle / SimulatedEngine::kGasConstant;
        const N2oAir s = solve_n2o_air(ve * p * vc_r, n_rev, e.charge.t_in_k, nc->h_in_j_kg, ox);
        const real da_drpm = s.da_dx * dve_drpm * p * vc_r + s.da_dn * dn_drpm;
        e.m_air_rev = s.a;
        const real k_o2 = ox.oxygen_mass_fraction() / kAirO2MassFraction;
        const real a_eq = s.a + k_o2 * n_rev;
        const real daeq_drpm = da_drpm + k_o2 * dn_drpm;
        const real afr_t = e.lambda * m.afr_stoich;
        f_ecu = s.a / afr_t;
        df_ecu_drpm = da_drpm / afr_t - s.a * dlambda_drpm / (e.lambda * afr_t);
        const real f_tot = f_ecu + kf_rev;
        const real df_tot = df_ecu_drpm + dkf_drpm;
        real lambda_c = real(0.0);
        if (f_tot > real(0.0)) {
            lambda_c = a_eq / (m.afr_stoich * f_tot);
            dlc_drpm = (daeq_drpm * f_tot - a_eq * df_tot) / (m.afr_stoich * f_tot * f_tot);
        }
        eta_y = a_eq;
        deta_y_drpm = daeq_drpm;
        lambda_fl = lambda_c;
        e.charge.t_mix_k = s.t_mix;
        e.charge.air_kg_rev = s.a;
        e.charge.n2o_kg_rev = n_rev;
        e.charge.ecu_fuel_kg_rev = f_ecu;
        e.charge.kit_fuel_kg_rev = kf_rev;
        e.charge.o2_equiv_air_kg_rev = a_eq;
        e.charge.lambda_combined = lambda_c;
        e.charge.overflow = s.overflow;
        e.n2o_rev = n_rev;
    }
    e.eta_i = m.maps.eta_i.value(rpm, eta_y);
    const real deta_drpm = m.maps.eta_i.dvalue_dx(rpm, eta_y) + m.maps.eta_i.dvalue_dy(rpm, eta_y) * deta_y_drpm;
    e.f_lambda = m.maps.f_lambda.evaluate(lambda_fl);
    const real dfl_dlambda = curve_slope_at(m.maps.f_lambda, lambda_fl);
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

    const real lhv_2pi = m.lhv_j_kg / kTwoPi;
    real dcomb_drpm = real(0.0);
    real tau_no_n2o_retard = real(0.0); // N3: tau_comb at the kit-unretarded spark (n2o branch only), before the anti-lag cut
    if (!n2o) {
        const real g = e.eta_i * e.f_lambda * e.eta_spark;
        const real dg_drpm = deta_drpm * e.f_lambda * e.eta_spark + e.eta_i * dfl_dlambda * dlambda_drpm * e.eta_spark +
                             e.eta_i * e.f_lambda * des_d * (dspark_drpm - dmbt_drpm);
        const real afr = e.lambda * m.afr_stoich;
        const real fuel_per_fire = e.m_air_rev / afr; // kg/rev at fire 1, s 1
        e.m_fuel_rev = fuel_per_fire * fire * s_starve;
        const real dfuel_per_fire_drpm = dma_drpm / afr - e.m_air_rev * dlambda_drpm / (e.lambda * afr);
        const real dmfuel_drpm = s_starve * (fire * dfuel_per_fire_drpm + fuel_per_fire * dfire_drpm);
        e.tau_comb = g * lhv_2pi * e.m_fuel_rev;
        e.tau_comb_unretarded = e.tau_comb;
        if (spark_applied != spark) {
            const real g0 = e.eta_i * e.f_lambda * m.maps.eta_spark.evaluate(spark - e.theta_mbt_deg);
            e.tau_comb_unretarded = g0 * lhv_2pi * e.m_fuel_rev;
        }
        dcomb_drpm = lhv_2pi * (dg_drpm * e.m_fuel_rev + g * dmfuel_drpm);
    } else {
        // N2O plan N2 (section 1.2 step 6): work per rev = eta_i*eta_spark*(f_lambda(lambda_c)*LHV*f*fire*s +
        // dH_dec*n*fire*s). The fired charge burns f = f_ecu + kf; the N2O in it decomposes completely [AS].
        // Booked fuel = (f_ecu*fire + kf)*s: the ECU skips a cut cylinder, the kit jet does not (its fuel passes
        // unburnt [AS], the same one-substep-late cut as section 1.3). The decomposition share uses fire*s_starve
        // (PROVISIONAL: the plan writes fire_n = fire; a starving substep burns only the fuelled share, so the
        // N2O in the unfuelled share passes through as well - identical when not starving).
        const real dh_dec = nc->oxidizer->decomposition_j_per_kg();
        const real f_tot = f_ecu + kf_rev;
        const real df_tot = df_ecu_drpm + dkf_drpm;
        const real fs = fire * s_starve;
        const real fb = f_tot * fs;
        const real dfb = s_starve * (df_tot * fire + f_tot * dfire_drpm);
        const real nb = n_rev * fs;
        const real dnb = s_starve * (dn_drpm * fire + n_rev * dfire_drpm);
        const real heat = e.f_lambda * m.lhv_j_kg * fb + dh_dec * nb; // J per rev released in the fired charges
        const real dheat = dfl_dlambda * dlc_drpm * m.lhv_j_kg * fb + e.f_lambda * m.lhv_j_kg * dfb + dh_dec * dnb;
        const real gs = e.eta_i * e.eta_spark;
        const real dgs = deta_drpm * e.eta_spark + e.eta_i * des_d * (dspark_drpm - dmbt_drpm);
        e.m_fuel_rev = (f_ecu * fire + kf_rev) * s_starve;
        e.kit_fuel_rev = kf_rev * s_starve;
        e.burnt_fuel_rev = fb;
        e.n2o_burnt_rev = nb;
        e.tau_comb = gs * heat / kTwoPi;
        e.tau_comb_unretarded = e.tau_comb;
        if (spark_applied != spark) {
            const real g0 = e.eta_i * m.maps.eta_spark.evaluate(spark - e.theta_mbt_deg);
            e.tau_comb_unretarded = g0 * heat / kTwoPi;
        }
        tau_no_n2o_retard = e.tau_comb;
        if (spark_applied_nr != spark_applied) {
            // N3 / D5 P2 rule: eta_spark(schedule) against eta_spark(schedule - retard) at the same charge, times the
            // released energy and eta_i; the plain table read (no anti-lag extrapolation) on the unretarded side.
            tau_no_n2o_retard = e.eta_i * m.maps.eta_spark.evaluate(spark_applied_nr - e.theta_mbt_deg) * heat / kTwoPi;
        }
        dcomb_drpm = (dgs * heat + gs * dheat) / kTwoPi;
    }
    // Anti-lag cylinder cut: the fuel is still injected (m_fuel_rev, booked), only the fired share burns in
    // the cylinder; the cut charge reaches the exhaust manifold (the exhaust energy balance).
    if (als && als->cylinder_cut_fraction > real(0.0)) {
        e.cut_fraction = als->cylinder_cut_fraction;
        const real fired_share = real(1.0) - e.cut_fraction;
        e.tau_comb *= fired_share;
        dcomb_drpm *= fired_share;
        tau_no_n2o_retard *= fired_share;
    }
    e.tau_comb_no_n2o_retard = n2o ? tau_no_n2o_retard : e.tau_comb;

    // pumping (ideal gas exchange + optional excess), friction (Coulomb part excluded)
    real pmep = real(0.0), dpmep_drpm = real(0.0);
    if (m.maps.has_pmep_excess) {
        pmep = m.maps.pmep_excess_pa.value(rpm, p);
        dpmep_drpm = m.maps.pmep_excess_pa.dvalue_dx(rpm, p);
    }
    e.tau_pump = -s_w * (amb.p_amb - p + pmep) * vc_2pi;
    const real f_rpm = m.maps.friction_torque_nm.evaluate(rpm);
    // Engine thermal T1b: k_fric = the oil-viscosity friction scale of the CALLER's model (the plant's k for the
    // physics, the ECU's estimate for the steady tables). The map's smooth part (f - c_k; c_k is the Coulomb row's)
    // gets the hydrodynamic increment (k - 1) * F(n_ref) * sqrt(n / n_ref) (hydrodynamic_friction.h); the Coulomb
    // part and the breakaway are boundary friction and never scaled. k_fric == 1.0: the pre-T1 expression exactly.
    real f_smooth = f_rpm - m.c_k;
    real f_smooth_slope = curve_slope_at(m.maps.friction_torque_nm, rpm);
    if (k_fric != real(1.0)) {
        const HydrodynamicShape hs = hydrodynamic_speed_shape(rpm, hyd_ref_rpm);
        const real dk_fref = (k_fric - real(1.0)) * m.maps.friction_torque_nm.evaluate(hyd_ref_rpm);
        f_smooth += dk_fref * hs.value;
        f_smooth_slope += dk_fref * hs.slope_per_rpm;
    }
    e.tau_fric = -s_w * f_smooth;

    // starter (EngineTorqueMap's own law)
    const real omega_s0 = starter.free_speed_rpm / kRadPerSecToRpm;
    e.tau_start = starter_active ? starter.stall_torque_nm * std::max(real(0.0), real(1.0) - std::max(omega, real(0.0)) / omega_s0)
                                 : real(0.0);

    e.tau = e.tau_comb + e.tau_pump + e.tau_fric + e.tau_start;

    // d/domega = K*sign(omega)*d/drpm for rpm-table terms; combustion only
    // ever runs at omega > 0 (sign 1). Pumping/friction: sign(omega)^2 = 1.
    real dtau = kRadPerSecToRpm * s_w * dcomb_drpm;
    dtau += -kRadPerSecToRpm * dpmep_drpm * vc_2pi * (s_w != real(0.0) ? real(1.0) : real(0.0));
    dtau += -kRadPerSecToRpm * f_smooth_slope * (s_w != real(0.0) ? real(1.0) : real(0.0));
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
// exh: the pressure the exhaust works against when it differs from the supply amb (a turbo
// engine's turbine inlet, see SimulatedEngine::evaluate_torque); nullptr = amb for both.
real steady_net_torque(const SimulatedEngineMaps& m, const Ambient& amb, const StarterDesc& st, real rpm, real x_thr, real x_iac,
                       bool fired, real k_ecu = real(1.0), real hyd_ref_rpm = real(2000.0), const Ambient* exh = nullptr) {
    const real omega = rpm / kRadPerSecToRpm;
    const real p = steady_p(m, amb, omega, x_thr, x_iac);
    // k_ecu: the ECU's friction scale (engine thermal plan T1); T1b: the Coulomb row's kinetic value is not scaled.
    return evaluate_torque_impl(m, exh ? *exh : amb, st, omega, p, fired, real(1.0), false, true, nullptr, k_ecu, hyd_ref_rpm).tau - m.c_k;
}

// The three scalars fill_idle_control produces. A plain POD on purpose: the live path
// (set_live_ambient, once per engine pre_solve) used to default-construct a whole
// SimulatedSteadyTables for them, whose ten empty std::vector members each allocate a
// _Container_proxy on the MSVC debug STL (_ITERATOR_DEBUG_LEVEL=2) - ten debug_alloc
// violations per tick, invisible in release.
struct IdleControlOut {
    real idle_ff = real(0.0);
    real idle_p_im = real(0.0);
    real idle_cap_x = real(0.0);
};

// Setup builds use 60 iterations; live control solves only the current idle operating point.
void fill_idle_control(const SimulatedEngineMaps& m,const Ambient& amb,real idle_rpm,real idle_max_x,
                       real idle_max_torque_nm,IdleControlOut& t,int iterations,real k_ecu = real(1.0),
                       real hyd_ref_rpm = real(2000.0), const Ambient* exh = nullptr) {
    const StarterDesc no_starter{};
    // Idle feed-forward: the plate (electronic) / bypass valve (mechanical,
    // plate at 0) position whose steady net torque at idle_rpm is zero.
    {
        const bool mech = m.throttle_input == ThrottleInput::Mechanical;
        auto net = [&](real x) {
            return mech ? steady_net_torque(m, amb, no_starter, idle_rpm, real(0.0), x, true, k_ecu, hyd_ref_rpm, exh)
                        : steady_net_torque(m, amb, no_starter, idle_rpm, x, real(0.0), true, k_ecu, hyd_ref_rpm, exh);
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

    IdleControlOut idle;
    fill_idle_control(m,amb,idle_rpm,idle_max_x,idle_max_torque_nm,idle,60);
    t.idle_ff = idle.idle_ff;
    t.idle_p_im = idle.idle_p_im;
    t.idle_cap_x = idle.idle_cap_x;
    t.p_pa = amb.p_amb;
}

} // namespace

// ---------------------------------------------------------------------------

std::shared_ptr<const SimulatedEngineMaps> SimulatedEngine::build_maps(const SimulatedEngineDesc& desc) {
    return build_maps_impl(desc, true);
}

std::shared_ptr<const SimulatedEngineMaps> SimulatedEngine::build_model_maps(const SimulatedEngineDesc& desc) {
    return build_maps_impl(desc, false);
}

std::shared_ptr<const SimulatedEngineMaps> SimulatedEngine::build_maps_impl(const SimulatedEngineDesc& desc, bool steady_tables) {
    if (desc.maps_pending) {
        throw std::runtime_error("SimulatedEngine: the engine's maps are pending (it was loaded with EngineMapOptions::generate = "
                                 "false); load it again with generate = true to run it");
    }
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
    if (steady_tables) { // engine garage P3: build_model_maps skips the ~70 ms derived grid (the dyno's NA path never reads it)
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
    if (desc.maps_pending) {
        throw std::runtime_error("SimulatedEngine: the engine's maps are pending (it was loaded with EngineMapOptions::generate = "
                                 "false); load it again with generate = true to run it");
    }
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

SimulatedEngine::SteadyPoint SimulatedEngine::steady_point(const SimulatedEngineMaps& m, real rpm, real command,
                                                           const AmbientSample& ambient) {
    const Ambient amb = make_ambient(ambient.pressure_pa, ambient.temperature_k);
    const StarterDesc no_starter{};
    const real omega = rpm / kRadPerSecToRpm;
    const bool mech = m.throttle_input == ThrottleInput::Mechanical;
    real plate = std::clamp(command, real(0.0), real(1.0));
    if (!mech && plate > real(0.0) && plate < real(1.0)) {
        // The electronic command is a torque REQUEST: the plate whose steady torque is mot + r*(wot - mot) - the same
        // definition and 60-step bisection fill_steady_tables uses for the runtime's inverse_plate, evaluated exactly
        // at this rpm instead of interpolated on the table. The idle controller's air (x_idle) is NOT added here:
        // part-throttle points below the idle plate are off the garage's use (WOT sweeps).
        const real wot = steady_net_torque(m, amb, no_starter, rpm, real(1.0), real(0.0), true);
        const real mot = steady_net_torque(m, amb, no_starter, rpm, real(0.0), real(0.0), false);
        const real target = mot + plate * (wot - mot);
        if (steady_net_torque(m, amb, no_starter, rpm, real(0.0), real(0.0), true) >= target) {
            plate = real(0.0);
        } else {
            real lo = real(0.0), hi = real(1.0);
            for (int it = 0; it < 60; ++it) {
                const real mid = real(0.5) * (lo + hi);
                if (steady_net_torque(m, amb, no_starter, rpm, mid, real(0.0), true) < target) lo = mid;
                else hi = mid;
            }
            plate = real(0.5) * (lo + hi);
        }
    }
    // Both kinds: plate at `plate`, bypass closed - fill_steady_tables' steady_torque_nm definition.
    const real p = steady_p(m, amb, omega, plate, real(0.0));
    // Exactly steady_net_torque's evaluation (same call, same arguments), keeping the whole TorqueEval.
    const TorqueEval ev = evaluate_torque_impl(m, amb, no_starter, omega, p, true, real(1.0), false, true, nullptr, real(1.0), real(2000.0));
    SteadyPoint sp;
    sp.net_torque_nm = ev.tau - m.c_k;
    sp.p_im_pa = p;
    sp.plate = plate;
    sp.fuel_g_s = ev.m_fuel_rev * (omega / kTwoPi) * real(1000.0);
    sp.lambda = ev.lambda;
    sp.spark_deg = ev.spark_deg;
    sp.knock_retard_deg = ev.knock_retard_deg;
    return sp;
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
        IdleControlOut idle;
        fill_idle_control(*maps_,make_ambient(p_pa,t_k),desc_idle_rpm_,idle_.max_throttle,idle_.max_torque_nm,idle,24,ecu_k_,hyd_ref_rpm_);
        live_idle_ff_=idle.idle_ff; live_idle_p_im_=idle.idle_p_im; live_idle_cap_x_=idle.idle_cap_x;
    }
}

real SimulatedEngine::live_steady_torque(real rpm,real plate,bool fired) const {
    const real clamped_rpm=std::clamp(rpm,steady_.steady_torque_nm.x.front(),steady_.steady_torque_nm.x.back());
    return steady_net_torque(*maps_,make_ambient(p_amb_,t_amb_),StarterDesc{},clamped_rpm,plate,real(0),fired,ecu_k_,hyd_ref_rpm_);
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
    n2o_on_ = false; // N2O plan N2: re-derived from the next substep's sensors
    n2o_in_kg_ = n2o_burnt_kg_ = kit_fuel_kg_ = n2o_dh_j_kg_ = burnt_fuel_kg_ = real(0.0);
    init_state_for(state);
}

real SimulatedEngine::psi(real pr, real* dpsi_dpr) const { return psi_lookup(maps_->psi, pr, dpsi_dpr); }

SimulatedEngine::ManifoldRhs SimulatedEngine::manifold_rhs(real p_im, real omega, real x_thr, real x_iac, const NitrousCharge* nc) const {
    if (turbo_) {
        const real rt = kGasConstant*turbo_->charge_k;
        return manifold_rhs_impl(*maps_, Ambient{turbo_->charge_pa,rt,real(1.0)/std::sqrt(rt)},p_im,omega,x_thr,x_iac,nc);
    }
    return manifold_rhs_impl(*maps_, Ambient{p_amb_, r_t_, inv_sqrt_rt_}, p_im, omega, x_thr, x_iac, nc);
}

SimulatedEngine::TorqueEval SimulatedEngine::evaluate_torque(real omega, real p_im, bool fired, real s_starve, bool starter_active,
                                                             const AntiLagDesc* als, const NitrousCharge* nc) const {
    if (turbo_) {
        const real rt = kGasConstant*turbo_->charge_k;
        auto result = evaluate_torque_impl(*maps_,Ambient{turbo_->exhaust_pa,rt,real(1.0)/std::sqrt(rt)},starter_,omega,p_im,fired,s_starve,starter_active,
                                           false,als,plant_k_,hyd_ref_rpm_,nc,limiter_rpm_override_);
        return result;
    }
    return evaluate_torque_impl(*maps_, Ambient{p_amb_, r_t_, inv_sqrt_rt_}, starter_, omega, p_im, fired, s_starve, starter_active,
                                false, als, plant_k_, hyd_ref_rpm_, nc, limiter_rpm_override_);
}

void SimulatedEngine::clear_anti_lag_state() {
    als_active_ = false;
    als_armed_ = true;
    als_timer_s_ = real(0.0);
    als_extra_w_ = real(0.0);
}

PreSolveResult SimulatedEngine::pre_solve(const Sensors& sensors, const ThrottleCommand& throttle, const TcuRequest& tcu, real h) {
    const SimulatedEngineMaps& m = *maps_;
    limiter_rpm_override_ = tcu.limiter_rpm; // engine garage P8: not hashed, rewritten every substep
    const bool mech = m.throttle_input == ThrottleInput::Mechanical;
    // Physical atmosphere follows every finite change, including microscopic altitude motion.
    // Control uses the current operating point; never refill the whole steady grid in a job.
    // Engine thermal plan T1: the plant's friction scale acts at once; a changed ECU estimate (once per 1 K of oil
    // temperature, quantised by EngineThermal) recomputes the live idle control in place - never the steady grid.
    plant_k_ = sensors.friction_scale;
    const bool ecu_k_changed = sensors.ecu_friction_scale != ecu_k_ || sensors.friction_reference_rpm != hyd_ref_rpm_;
    ecu_k_ = sensors.ecu_friction_scale;
    hyd_ref_rpm_ = sensors.friction_reference_rpm;
    if (sensors.ambient.pressure_pa != p_amb_ || sensors.ambient.temperature_k != t_amb_) {
        set_live_ambient(sensors.ambient.pressure_pa, sensors.ambient.temperature_k);
    } else if (ecu_k_changed) {
        set_live_ambient(p_amb_, t_amb_);
    }
    const real omega_old = sensors.omega;
    omega_old_ = omega_old;
    const real rpm = std::abs(omega_old) * kRadPerSecToRpm;
    // N2O plan N2: this substep's wet charge input (the NitrousSystem decided it before this pre_solve). nc is
    // nullptr - the pre-N2 code path, bit for bit - unless N2O actually flows with property tables.
    n2o_on_ = sensors.nitrous.active && sensors.nitrous.mdot_n_kg_s > real(0.0) && sensors.nitrous.oxidizer != nullptr;
    if (n2o_on_) {
        n2o_nc_ = NitrousCharge{sensors.nitrous.mdot_n_kg_s, sensors.nitrous.mdot_kf_kg_s, sensors.nitrous.h_in_j_kg,
                                sensors.nitrous.retard_deg, sensors.nitrous.oxidizer};
    }
    const NitrousCharge* nc = n2o_on_ ? &n2o_nc_ : nullptr;

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
    real ff = live_steady()?live_idle_ff_:steady_.idle_ff;
    // Boost-aware idle feed-forward (2026-10-06, car_hyper idle hunt after a free rev): after a lift the
    // turbo rotors spin down over tens of seconds and keep the supply ahead of the plate far above ambient
    // (3.1 bar abs at the lift, still 1.2 bar 20 s later). The feed-forward above is solved against the
    // AMBIENT supply, so it admitted too much air and the one-sided idle PI could not take it back - the
    // engine cycled between the DFCO enable and re-enable rpm. With the pedal released, a turbo engine
    // whose charge pressure is above ambient solves its feed-forward against the charge pressure and
    // temperature (supply) and the turbine-inlet pressure (exhaust), exactly as evaluate_torque and
    // manifold_rhs see them. Quantised (1 kPa, 1 K) and cached: a pure function of the quantised state,
    // recomputed only when it changes. At or below ambient + 1 kPa the ambient feed-forward stays, so a
    // turbo engine at rest and every atmospheric engine are unchanged.
    if (turbo_ && theta_req < real(0.01) && turbo_->charge_pa > p_amb_ + kBoostIdleThresholdPa &&
        (state_ == EngineState::Running || state_ == EngineState::Cranking)) {
        const real qp = std::round(turbo_->charge_pa / real(1000.0)) * real(1000.0);
        const real qt = std::round(turbo_->charge_k);
        const real qe = std::round(turbo_->exhaust_pa / real(1000.0)) * real(1000.0);
        if (qp != boost_idle_p_ || qt != boost_idle_t_ || qe != boost_idle_exh_ || ff != boost_idle_base_ff_) {
            IdleControlOut idle;
            const Ambient exh = make_ambient(qe, qt);
            // Feed-forward only (no authority-cap solve): the cap below stays the ambient one, max(cap, ff) as before.
            fill_idle_control(m, make_ambient(qp, qt), desc_idle_rpm_, idle_.max_throttle, real(0.0), idle, 24, ecu_k_, hyd_ref_rpm_, &exh);
            boost_idle_p_ = qp;
            boost_idle_t_ = qt;
            boost_idle_exh_ = qe;
            boost_idle_base_ff_ = ff;
            boost_idle_ff_ = std::min(idle.idle_ff, ff); // boost never asks for MORE idle air than ambient supply
        }
        ff = boost_idle_ff_;
    }
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
            return mech ? manifold_rhs(p_im_, omega_old, xp, xi, nc).f : manifold_rhs(p_im_, omega_old, xi, real(0.0), nc).f;
        };
        const real f0 = f_at(theta_idle);
        const real dfdx = (f_at(theta_idle + kIdleFillDx) - f0) / kIdleFillDx;
        if (dfdx > real(0.0)) {
            x_idle = std::clamp(theta_idle + (kIdleFillGain - real(1.0)) * f0 / dfdx, real(0.0), idle_.max_throttle);
        }
    }

    bool limiter_active = false;
    // Engine garage P8 (A16): launch control's second limit (set at the top of pre_solve) replaces the limiter speed.
    const real lim_rpm = (limiter_rpm_override_ > real(0.0) && limiter_rpm_override_ < m.limiter.rpm) ? limiter_rpm_override_ : m.limiter.rpm;
    if (m.limiter.type == LimiterType::FuelCut) {
        if (rpm >= lim_rpm) limiter_cut_ = true;
        else if (rpm <= lim_rpm - m.limiter.hysteresis_rpm) limiter_cut_ = false;
        limiter_active = limiter_cut_;
    } else {
        limiter_active = rpm > lim_rpm - m.limiter.band_rpm;
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
        const ManifoldRhs rhs = manifold_rhs(p0, omega_old, x_thr_, x_iac_, nc);
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
        const real f1 = manifold_rhs(p1, omega_old, x_thr_, x_iac_, nc).f;
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
    TorqueEval ev = evaluate_torque(omega_old, p_im_, fired, real(1.0), starter_active, als_eval, nc);
    const real rev_rate = std::max(omega_old, real(0.0)) / kTwoPi;
    real s_starve = real(1.0);
    booked_fuel_mass_kg_ = h * ev.m_fuel_rev * rev_rate;
    if (booked_fuel_mass_kg_ > sensors.fuel_available_kg) {
        const real available = std::max(real(0.0), sensors.fuel_available_kg);
        s_starve = available / booked_fuel_mass_kg_;
        booked_fuel_mass_kg_ = available;
        ev = evaluate_torque(omega_old, p_im_, fired, s_starve, starter_active, als_eval, nc);
    }
    // N2O plan N2 bookings for post_solve (section 1.3/4): every delivered kg enters the engine; the fired,
    // fuelled share decomposes (fire * s_starve, the same factor as the charge model's n2o_burnt_rev); the rest
    // passes through. Without a turning crank there is no charge model: all of it passes through [AS].
    n2o_in_kg_ = n2o_burnt_kg_ = kit_fuel_kg_ = n2o_dh_j_kg_ = burnt_fuel_kg_ = real(0.0);
    if (n2o_on_) {
        n2o_in_kg_ = h * n2o_nc_.mdot_n_kg_s;
        if (omega_old > real(0.0)) {
            n2o_burnt_kg_ = n2o_in_kg_ * (ev.fire * s_starve);
            kit_fuel_kg_ = h * ev.kit_fuel_rev * rev_rate;
            burnt_fuel_kg_ = h * ev.burnt_fuel_rev * rev_rate;
            n2o_dh_j_kg_ = n2o_nc_.oxidizer->decomposition_j_per_kg();
            if (ev.charge.overflow) ++n2o_overflow_substeps_;
        }
    }
    last_charge_ = ev.charge;
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
    // N2O plan N2: with the charge model the fuel burns at the combined lambda (O2 from air and N2O).
    const real lambda_burn = (n2o_on_ && omega_old > real(0.0)) ? ev.charge.lambda_combined : ev.lambda;
    eta_c_ = (lambda_burn > real(0.0) ? std::min(real(1.0), lambda_burn) : real(0.0)) * (real(1.0) - ev.cut_fraction);
    tau_comb_ = ev.tau_comb;
    tau_pump_ = ev.tau_pump;
    tau_fric_ = ev.tau_fric;
    tau_start_ = ev.tau_start;
    tau0_ = ev.tau;
    d_ = std::max(-ev.dtau_domega, real(0.0));

    PreSolveResult r;
    r.tau0 = tau0_;
    r.d = d_;
    r.friction_cap_static = breakaway_torque_nm_; // engine thermal T1b: boundary friction, never viscosity-scaled
    r.friction_cap_kinetic = m.c_k;
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
    // N2O plan N2: the same rule carries the torque a nitrous shot adds (the steady tables have no N2O either).
    if (turbo_ && comb && !dfco_active_ && !tcu.ignition_cut) {
        const real rt = kGasConstant * turbo_->charge_k;
        const auto uncut = evaluate_torque_impl(m,
            Ambient{turbo_->exhaust_pa, rt, real(1.0) / std::sqrt(rt)},
            starter_, omega_old, p_im_, true, real(1.0), false, true, als_eval, ecu_k_, hyd_ref_rpm_, nc);
        r.torque_demand = std::max(r.torque_demand, uncut.tau - sign(omega_old) * m.c_k);
    } else if (n2o_on_ && comb && !dfco_active_ && !tcu.ignition_cut) {
        const auto uncut = evaluate_torque_impl(m, Ambient{p_amb_, r_t_, inv_sqrt_rt_}, starter_, omega_old, p_im_, true, real(1.0), false,
                                                true, als_eval, ecu_k_, hyd_ref_rpm_, nc);
        r.torque_demand = std::max(r.torque_demand, uncut.tau - sign(omega_old) * m.c_k);
    }
    r.theta_c = std::clamp(ev.fire * s_starve * p_im_ / p_amb_, real(0.0), real(1.0));
    r.fuel_flow_g_s = booked_fuel_mass_kg_ / h * real(1000.0);
    r.fuel_supply_fraction = s_starve;
    r.throttle_position = x_thr_;
    r.p_intake_pa = p_im_;
    r.lambda = ev.lambda;
    r.spark_advance_deg = ev.spark_deg;
    r.knock_retard_deg = ev.knock_retard_deg;
    r.exhaust_mass_flow_kg_s = ev.m_air_rev * std::abs(omega_old) / kTwoPi + booked_fuel_mass_kg_ / h;
    if (n2o_on_) {
        // N2O plan N2 (section 1.2 step 7): the N2O (decomposed or not) leaves through the exhaust too; the kit
        // fuel is already in the booked fuel. Sound/HUD fields (section 6).
        r.exhaust_mass_flow_kg_s += n2o_nc_.mdot_n_kg_s;
        if (omega_old > real(0.0)) {
            r.charge_temperature_k = ev.charge.t_mix_k;
            r.n2o_lambda_combined = ev.charge.lambda_combined;
            r.n2o_kit_fuel_g_s = kit_fuel_kg_ / h * real(1000.0);
        }
    }
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
            real released = real(0.0);
            if (n2o_on_ && omega_old > real(0.0)) {
                // N2O plan N3 (turbine heat): with the wet charge model the exhaust balance follows the ENGINE's own
                // release bookkeeping (D-N2-9), not the ECU lambda over the booked fuel. (1) Only the fuel of the
                // fired charges burns: the kit jet's fuel through a cut cylinder was booked (drawn from the tank) but
                // leaves as unburnt chemical enthalpy. (2) It burns at the COMBINED lambda (oxygen from the air and
                // from the decomposed N2O): eta_c = min(1, lambda_c), the same eta_c_ the thermal model uses.
                // (3) The decomposition heat of the N2O that burnt (n2o_burnt * dH, the ledger's n2o_energy_in) is
                // released in full and joins the chemical input the exhaust balance splits:
                //   fuel_w + n2o_w = indicated + wall + sensible + chemical - clamp.
                // The liquid-to-gas enthalpy deficit of the evaporating N2O (~4 % of the decomposition heat at a
                // 100 hp shot) is not subtracted (PROVISIONAL, section 1.2 keeps T_in as the reference).
                const real lambda_c = ev.charge.lambda_combined;
                const real eta_c = lambda_c > real(0.0) ? std::min(real(1.0), lambda_c) : real(0.0);
                const real burnt_fuel_w = burnt_fuel_kg_ / h * maps_->lhv_j_kg;
                x.n2o_w = n2o_burnt_kg_ * n2o_dh_j_kg_ / h;
                x.chemical_w = (x.fuel_w - burnt_fuel_w) + (real(1.0) - eta_c) * burnt_fuel_w;
                released = eta_c * burnt_fuel_w + x.n2o_w;
                // The work the kit's retard removes (D5 P2 rule) needs no extra term: indicated_w below is the
                // retarded work, so the lost part stays in the released energy and reaches the exhaust as sensible
                // enthalpy, and from there the lumped exhaust wall (the held heat). Reported for tests and the HUD.
                x.n2o_retard_w = omega_old * (ev.tau_comb_no_n2o_retard - ev.tau_comb);
            } else {
                const real eta_c = ev.lambda > real(0.0) ? std::min(real(1.0), ev.lambda) : real(0.0);
                x.chemical_w = (real(1.0) - eta_c) * x.fuel_w;
                released = x.fuel_w - x.chemical_w;
            }
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
        r.turbo_compressor_stage_used=turbo_->compressor_stage_used;
        r.turbo_turbine_stage_used=turbo_->turbine_stage_used;
        r.turbo_stage_map_fallback=turbo_->stage_map_fallback;
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
    if (n2o_on_) {
        // N2O plan N2 (section 4): the engine's split of this substep's N2O; the decomposition heat joins the
        // fuel energy (engine_heat = fuel_energy_in + n2o_energy_in - indicated_work, Powertrain).
        r.n2o_mass_delta = n2o_in_kg_;
        r.n2o_unburnt_delta = n2o_in_kg_ - n2o_burnt_kg_;
        r.n2o_energy_in_delta = n2o_burnt_kg_ * n2o_dh_j_kg_;
        r.kit_fuel_mass_delta = kit_fuel_kg_;
        // Only the fuel of the fired charges is released: in a cut the kit fuel is booked (drawn from the tank)
        // but passes the cylinder unburnt, like the anti-lag cut share.
        r.released_heat_delta = eta_c_ * burnt_fuel_kg_ * maps_->lhv_j_kg + r.n2o_energy_in_delta;
    }

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
        if(t.compressible_exhaust) f.pod(t.compressible_exhaust);
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
