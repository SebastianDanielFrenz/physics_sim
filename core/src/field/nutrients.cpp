// core/src/field/nutrients.cpp - the nutrient half of Field (farming F1b); see
// ps/field/field.h for the overview and docs/data_formats/{soil,crop,field_material}.md
// for the parameters. Every rate, factor and threshold is read from the soil, crop,
// material and method descriptors; what is written below are the equations.
//
// Equation sources: Justes et al. 1994 (critical N dilution curve, Ann. Bot. 74:397);
// first-order temperature response with a Q10 (van't Hoff); competing first-order
// volatilisation and infiltration of the surface ammoniacal N (the form of the
// UBA/EMEP incorporation-time reductions, UBA Texte 39/2013 Tab. 6.3); a mixing cell
// for nitrate transport with the water the water balance moves; Mitscherlich for the
// soil-test yield response; alkalinity bookkeeping in kg CaO for pH (VDLUFA lime
// requirement tables, DLG-Merkblatt 456).

#include "ps/field/field.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ps::field {

namespace {

inline real clamp01(real x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

// Critical N concentration (% of dry matter) at aboveground dry matter w (t/ha).
real critical_n_pct(const CropNutrientDesc& cn, real w) {
    if (w < cn.nc_w_min_t_ha) return cn.nc_const_pct;
    return cn.nc_a * ps::math::pow(w, -cn.nc_b);
}

// pH availability factor of mineralisation and of P/K.
real avail_factor(const SoilNutrientDesc& sn, real ph) {
    return sn.avail_floor_factor +
           (1.0 - sn.avail_floor_factor) * clamp01((ph - sn.ph_avail_floor) / (sn.ph_avail_full - sn.ph_avail_floor));
}

// Biological temperature factor: Q10 around 20 degC, ramping to zero at 0 degC.
real temperature_factor(const SoilNutrientDesc& sn, real t) {
    if (t <= 0.0) return 0.0;
    return ps::math::pow(sn.min_q10, (t - 20.0) / 10.0) * clamp01(t / sn.soil_t_ramp_c);
}

// Mitscherlich relative yield of a soil-test value, 1 at `target`.
real mitscherlich(real test, real b, real target) {
    if (!(b > 0.0)) return 1.0;
    const real r = (1.0 - ps::math::exp(-b * test)) / (1.0 - ps::math::exp(-b * target));
    return r > 1.0 ? 1.0 : (r < 0.0 ? 0.0 : r);
}

} // namespace

// ---------------------------------------------------------------------------
// construction, small queries
// ---------------------------------------------------------------------------
void Field::init_nutrients() {
    const std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    for (auto* v : {&s_.no3_1, &s_.no3_2, &s_.nh4, &s_.org_slow, &s_.org_fresh, &s_.tan_surf, &s_.kv_w, &s_.ki_w, &s_.kir_w,
                    &s_.soil_t, &s_.n_crop, &s_.n_root, &s_.p_crop, &s_.k_crop, &s_.p_av, &s_.p_res, &s_.k_av, &s_.k_res,
                    &s_.alk, &s_.lime_surf, &s_.lime_inc, &s_.lodge, &s_.l_n_init, &s_.l_n_fmin, &s_.l_n_forg, &s_.l_n_dep,
                    &s_.l_n_fix, &s_.l_n_rem, &s_.l_n_leach, &s_.l_n_nh3, &s_.l_n_den, &s_.l_n_min, &s_.l_p_init,
                    &s_.l_p_app, &s_.l_p_rem, &s_.l_k_init, &s_.l_k_app, &s_.l_k_rem, &s_.l_ca_app, &s_.l_ca_loss})
        v->assign(n, 0.0);
    for (auto* v : {&s_.f_n, &s_.f_pk, &s_.f_ph}) v->assign(n, 1.0);
    s_.inc_h.assign(n, -1.0);
    for (std::size_t c = 0; c < n; ++c) {
        const SoilDesc& so = soil_of(c);
        const SoilNutrientDesc& sn = so.nutrients;
        const real f1 = so.fallow_root_depth_m / so.depth_m;
        s_.no3_1[c] = sn.no3_init_kg_ha * f1;
        s_.no3_2[c] = sn.no3_init_kg_ha * (1.0 - f1);
        s_.nh4[c] = sn.nh4_init_kg_ha;
        s_.org_slow[c] = sn.org_n_kg_ha;
        s_.soil_t[c] = sn.soil_t_init_c;
        s_.p_av[c] = sn.p_test_init_mg100g * sn.soil_test_kg_per_mg100g;
        s_.p_res[c] = sn.p_reserve_kg_ha;
        s_.k_av[c] = sn.k_test_init_mg100g * sn.soil_test_kg_per_mg100g;
        s_.k_res[c] = sn.k_reserve_kg_ha;
        s_.l_n_init[c] = n_storage(c);
        s_.l_p_init[c] = s_.p_av[c] + s_.p_res[c];
        s_.l_k_init[c] = s_.k_av[c] + s_.k_res[c];
    }
}

real Field::ph_of(std::size_t c) const {
    const SoilNutrientDesc& sn = soil_of(c).nutrients;
    return std::min(8.0, std::max(3.0, sn.ph_init + s_.alk[c] / sn.buffer_cao_kg_ha_per_ph));
}

real Field::n_storage(std::size_t c) const {
    return s_.no3_1[c] + s_.no3_2[c] + s_.nh4[c] + s_.org_slow[c] + s_.org_fresh[c] + s_.tan_surf[c] + s_.n_crop[c] + s_.n_root[c];
}

void Field::drop_crop_nutrients(std::size_t c) {
    s_.org_fresh[c] += s_.n_crop[c] + s_.n_root[c];
    s_.p_av[c] += s_.p_crop[c];
    s_.k_av[c] += s_.k_crop[c];
    s_.n_crop[c] = s_.n_root[c] = s_.p_crop[c] = s_.k_crop[c] = 0.0;
    s_.lodge[c] = 0.0;
}

// ---------------------------------------------------------------------------
// growth factors at the start of the day (the stress hook)
// ---------------------------------------------------------------------------
void Field::nutrient_growth_factors(std::size_t c, int st, bool emerged, real& f_n, real& f_pk, real& f_ph) {
    f_n = f_pk = f_ph = 1.0;
    if (s_.crop[c] == 0 || !emerged) return;
    const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(s_.crop[c] - 1)];
    const CropNutrientDesc& cn = cr.nutrients;
    const SoilNutrientDesc& sn = soil_of(c).nutrients;
    const real w = s_.biomass[c] * 0.01;
    if (st < cr.stage_senescence && w > cn.n_stress_start_t_ha) {
        const real nc = critical_n_pct(cn, w);
        const real pct = s_.n_crop[c] / (10.0 * w);
        f_n = clamp01((pct - cn.n_min_conc_pct) / (nc - cn.n_min_conc_pct));
    }
    const real ph = ph_of(c);
    const real av = avail_factor(sn, ph);
    const real fp = mitscherlich(s_.p_av[c] / sn.soil_test_kg_per_mg100g * av, sn.p_rel_b, sn.p_target_mg100g);
    const real fk = mitscherlich(s_.k_av[c] / sn.soil_test_kg_per_mg100g * av, sn.k_rel_b, sn.k_target_mg100g);
    f_pk = std::min(fp, fk);
    if (ph >= cn.ph_opt_low && ph <= cn.ph_opt_high) f_ph = 1.0;
    else if (ph < cn.ph_opt_low) f_ph = cn.ph_floor_factor + (1.0 - cn.ph_floor_factor) * clamp01((ph - cn.ph_floor) / (cn.ph_opt_low - cn.ph_floor));
    else f_ph = 1.0;
}

// ---------------------------------------------------------------------------
// the daily nutrient step of one cell (after the water balance and the growth)
// ---------------------------------------------------------------------------
void Field::step_nutrients(std::size_t c, const DailyWeather& w, const DayFlows& fl) {
    FieldState& s = s_;
    const SoilDesc& so = soil_of(c);
    const SoilNutrientDesc& sn = so.nutrients;
    const CropDesc* cr = s.crop[c] != 0 ? &cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)] : nullptr;

    // --- soil temperature, biological factors --------------------------------------------------
    s.soil_t[c] += (w.tmean_c - s.soil_t[c]) / sn.soil_t_lag_days;
    const real ft = temperature_factor(sn, s.soil_t[c]);
    const real rel_wet = fl.taw1 > 0.0 ? clamp01(1.0 - s.d_root[c] / fl.taw1) : 1.0;
    const real fm = sn.min_moisture_floor + (1.0 - sn.min_moisture_floor) * rel_wet;
    const real f_av = avail_factor(sn, ph_of(c));

    // --- ammoniacal N on the surface: volatilisation and infiltration compete (first order) ---------
    if (s.tan_surf[c] > 0.0) {
        const real tan = s.tan_surf[c];
        const real tfac = ps::math::pow(sn.nh3_temp_q10, (w.tmean_c - 15.0) / 10.0);
        const real kv = s.kv_w[c] / tan * tfac;
        const real ki = s.ki_w[c] / tan + s.kir_w[c] / tan * w.rain_mm;
        const real k = kv + ki;
        const real exposed_h = s.inc_h[c] < 0.0 ? 24.0 : std::min(24.0, s.inc_h[c]);
        real lost = 0.0, inf = 0.0;
        if (k > 0.0) {
            const real f_end = 1.0 - ps::math::exp(-k * exposed_h / 24.0);
            lost = tan * kv / k * f_end;
            inf = tan * ki / k * f_end;
        }
        real rem = tan - lost - inf;
        s.l_n_nh3[c] += lost;
        s.nh4[c] += inf;
        const bool incorporate = s.inc_h[c] >= 0.0 && s.inc_h[c] <= 24.0;
        if (incorporate || rem < 1.0e-6) {
            s.nh4[c] += rem;
            rem = 0.0;
            s.inc_h[c] = -1.0;
        } else if (s.inc_h[c] > 24.0) {
            s.inc_h[c] -= 24.0;
        }
        const real keep = rem / tan;
        s.kv_w[c] *= keep;
        s.ki_w[c] *= keep;
        s.kir_w[c] *= keep;
        s.tan_surf[c] = rem;
        if (rem == 0.0) s.kv_w[c] = s.ki_w[c] = s.kir_w[c] = 0.0;
    }

    // --- mineralisation, deposition, nitrification, denitrification -------------------------------------
    {
        const real m_slow = s.org_slow[c] * (1.0 - ps::math::exp(-sn.min_rate_per_day_20c * ft * fm * f_av));
        const real m_fresh = s.org_fresh[c] * (1.0 - ps::math::exp(-sn.fresh_rate_per_day_20c * ft * fm));
        s.org_slow[c] -= m_slow;
        s.org_fresh[c] -= m_fresh;
        s.nh4[c] += m_slow + m_fresh;
        s.l_n_min[c] += m_slow + m_fresh;
        s.nh4[c] += sn.n_deposition_kg_ha_day;
        s.l_n_dep[c] += sn.n_deposition_kg_ha_day;
        const real nit = s.nh4[c] * (1.0 - ps::math::exp(-sn.nitrif_rate_per_day_20c * ft * fm));
        s.nh4[c] -= nit;
        s.no3_1[c] += nit;
        if (rel_wet > sn.denit_wetness) {
            const real wf = (rel_wet - sn.denit_wetness) / (1.0 - sn.denit_wetness);
            const real den = s.no3_1[c] * (1.0 - ps::math::exp(-sn.denit_rate_per_day_20c * ft * wf));
            s.no3_1[c] -= den;
            s.l_n_den[c] += den;
        }
    }

    // --- crop uptake: N along the critical dilution curve, P and K with the growth ------------------------------
    if (cr && fl.emerged) {
        const CropNutrientDesc& cn = cr->nutrients;
        if (fl.stage < cr->stage_senescence) {
            const real wt = s.biomass[c] * 0.01;
            const real n_crit = 10.0 * critical_n_pct(cn, wt) * wt;
            const real dem_above = std::max(0.0, n_crit - s.n_crop[c]);
            const real dem_tot = dem_above * (1.0 + cn.n_root_fraction);
            const real fix = cn.n_fixation_fraction * dem_tot;
            const real soil_dem = dem_tot - fix;
            const real mineral = s.no3_1[c] + s.nh4[c];
            const real avail = std::max(0.0, mineral - cn.n_residual_kg_ha);
            real take = std::min(soil_dem, std::min(cn.n_uptake_max_kg_ha_day, cn.n_uptake_frac_per_day * avail));
            if (!(mineral > 0.0) || take < 0.0) take = 0.0;
            if (take > 0.0) {
                const real d_no3 = take * s.no3_1[c] / mineral;
                s.no3_1[c] -= d_no3;
                s.nh4[c] -= take - d_no3;
            }
            const real total = take + fix;
            const real above = total / (1.0 + cn.n_root_fraction);
            s.n_crop[c] += above;
            s.n_root[c] += total - above;
            s.l_n_fix[c] += fix;
        }
        if (fl.stage < cr->stage_maturity) {
            const real d_bio = (s.biomass[c] - fl.bio_before) * 10.0;   // g/m2 -> kg/ha
            if (d_bio > 0.0) {
                const real dp = std::min(s.p_av[c], cn.p2o5_conc_pct_dm / 100.0 * d_bio);
                const real dk = std::min(s.k_av[c], cn.k2o_conc_pct_dm / 100.0 * d_bio);
                s.p_av[c] -= dp;
                s.p_crop[c] += dp;
                s.k_av[c] -= dk;
                s.k_crop[c] += dk;
            }
        }
        // lodging risk: surplus soil mineral N in the sensitive window
        if (fl.bbch >= cn.lodging_bbch_from && fl.bbch <= cn.lodging_bbch_to) {
            const real nmin = s.no3_1[c] + s.nh4[c];
            const real loss = cn.lodging_max_loss * clamp01((nmin - cn.lodging_nmin_start_kg_ha) /
                                                           (cn.lodging_nmin_full_kg_ha - cn.lodging_nmin_start_kg_ha));
            s.lodge[c] = std::max(s.lodge[c], loss);
        }
    }

    // --- nitrate transport: a mixing cell per layer with the water the balance moved ----------------------------------
    {
        const real v1 = so.theta_fc * s.zr[c] * 1000.0;
        const real v2 = so.theta_fc * (so.depth_m - s.zr[c]) * 1000.0;
        if (fl.dp > 0.0 && v1 > 0.0) {
            const real mv = s.no3_1[c] * (1.0 - ps::math::exp(-fl.dp / v1));
            s.no3_1[c] -= mv;
            if (v2 > 1.0e-9) s.no3_2[c] += mv;
            else s.l_n_leach[c] += mv;
        }
        if (fl.drain > 0.0 && v2 > 1.0e-9) {
            const real out = s.no3_2[c] * (1.0 - ps::math::exp(-fl.drain / v2));
            s.no3_2[c] -= out;
            s.l_n_leach[c] += out;
        }
    }

    // --- lime reaction, unavoidable base loss ----------------------------------------------------------------------
    {
        const real ri = s.lime_inc[c] * (1.0 - ps::math::exp(-sn.lime_react_per_day));
        const real rs = s.lime_surf[c] * (1.0 - ps::math::exp(-sn.lime_react_per_day * sn.lime_surface_factor));
        s.lime_inc[c] -= ri;
        s.lime_surf[c] -= rs;
        s.alk[c] += ri + rs;
        const real loss = sn.cao_loss_kg_ha_per_mm_drain * fl.dp;
        s.alk[c] -= loss;
        s.l_ca_loss[c] += loss;
    }

    // --- P and K: slow exchange between the reserve and the available pool ----------------------------------------------
    {
        const real eq_p = sn.p_eq_mg100g * sn.soil_test_kg_per_mg100g;
        if (s.p_av[c] < eq_p) {
            const real fx = std::min(s.p_res[c], sn.p_release_per_day * (eq_p - s.p_av[c]));
            s.p_av[c] += fx;
            s.p_res[c] -= fx;
        } else {
            const real fx = sn.p_fix_per_day * (s.p_av[c] - eq_p);
            s.p_av[c] -= fx;
            s.p_res[c] += fx;
        }
        const real eq_k = sn.k_eq_mg100g * sn.soil_test_kg_per_mg100g;
        if (s.k_av[c] < eq_k) {
            const real fx = std::min(s.k_res[c], sn.k_release_per_day * (eq_k - s.k_av[c]));
            s.k_av[c] += fx;
            s.k_res[c] -= fx;
        } else {
            const real fx = sn.k_fix_per_day * (s.k_av[c] - eq_k);
            s.k_av[c] -= fx;
            s.k_res[c] += fx;
        }
    }
}

// ---------------------------------------------------------------------------
// operations: the deposit effect
// ---------------------------------------------------------------------------
void Field::apply(int cell, const OperationEffect& e) {
    if (!cfg_.nutrients_enabled) throw std::logic_error("Field::apply: the field runs without nutrients");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::apply: bad cell");
    if (!(e.coverage >= 0.0)) throw std::invalid_argument("Field::apply: coverage must be >= 0");
    const Deposit& d = e.deposit;
    if (d.material == 0) return;
    if (d.material < 1 || static_cast<std::size_t>(d.material) > cfg_.materials.size())
        throw std::invalid_argument("Field::apply: unknown material");
    if (d.method < 1 || static_cast<std::size_t>(d.method) > cfg_.methods.size())
        throw std::invalid_argument("Field::apply: unknown method");
    if (!(d.rate_t_ha >= 0.0)) throw std::invalid_argument("Field::apply: rate must be >= 0");
    const std::size_t c = static_cast<std::size_t>(cell);
    const MaterialDesc& m = cfg_.materials[static_cast<std::size_t>(d.material - 1)];
    const MethodDesc& me = cfg_.methods[static_cast<std::size_t>(d.method - 1)];
    const real amt = d.rate_t_ha * e.coverage;   // t/ha of fresh matter, summed over the passes

    const real ammoniacal = (m.nh4_n_kg_t + m.urea_n_kg_t) * amt;
    const real placed = me.soil_placed_fraction * ammoniacal;
    const real surface = ammoniacal - placed;
    const real organic = m.org_n_kg_t() * amt;
    s_.nh4[c] += placed;
    s_.tan_surf[c] += surface;
    s_.kv_w[c] += surface * m.k_vol_per_day * me.k_vol_factor;
    s_.ki_w[c] += surface * m.k_inf_per_day;
    s_.kir_w[c] += surface * m.k_inf_per_mm_rain;
    s_.no3_1[c] += m.no3_n_kg_t * amt;
    s_.org_slow[c] += organic * m.org_slow_fraction;
    s_.org_fresh[c] += organic * (1.0 - m.org_slow_fraction);
    s_.l_n_fmin[c] += ammoniacal + m.no3_n_kg_t * amt;
    s_.l_n_forg[c] += organic;
    if (surface > 0.0) {
        const real delay = d.incorporation_delay_h < -1.5 ? me.incorporation_delay_h : d.incorporation_delay_h;
        if (delay >= 0.0) s_.inc_h[c] = s_.inc_h[c] < 0.0 ? delay : std::min(s_.inc_h[c], delay);
    }
    s_.p_av[c] += m.p2o5_kg_t * amt;
    s_.k_av[c] += m.k2o_kg_t * amt;
    s_.l_p_app[c] += m.p2o5_kg_t * amt;
    s_.l_k_app[c] += m.k2o_kg_t * amt;
    const real cao = m.cao_kg_t * amt;
    if (cao >= 0.0) (me.incorporated ? s_.lime_inc[c] : s_.lime_surf[c]) += cao;
    else s_.alk[c] += cao;
    s_.l_ca_app[c] += cao;
}

void Field::apply_all(const OperationEffect& e) {
    for (int c = 0; c < cell_count(); ++c) apply(c, e);
}

// ---------------------------------------------------------------------------
// queries
// ---------------------------------------------------------------------------
NutrientView Field::nutrient_view(int cell) const {
    if (!cfg_.nutrients_enabled) throw std::logic_error("Field::nutrient_view: the field runs without nutrients");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::nutrient_view: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell);
    const SoilNutrientDesc& sn = soil_of(c).nutrients;
    NutrientView v;
    v.no3_root = s_.no3_1[c];
    v.no3_sub = s_.no3_2[c];
    v.nh4 = s_.nh4[c];
    v.org_slow = s_.org_slow[c];
    v.org_fresh = s_.org_fresh[c];
    v.tan_surface = s_.tan_surf[c];
    v.nmin = v.no3_root + v.no3_sub + v.nh4;
    v.n_crop = s_.n_crop[c];
    v.n_root = s_.n_root[c];
    v.p2o5_crop = s_.p_crop[c];
    v.k2o_crop = s_.k_crop[c];
    v.p2o5_avail = s_.p_av[c];
    v.p2o5_reserve = s_.p_res[c];
    v.k2o_avail = s_.k_av[c];
    v.k2o_reserve = s_.k_res[c];
    v.p_test_mg100g = s_.p_av[c] / sn.soil_test_kg_per_mg100g;
    v.k_test_mg100g = s_.k_av[c] / sn.soil_test_kg_per_mg100g;
    v.ph = ph_of(c);
    v.alk_cao = s_.alk[c];
    v.lime_surface = s_.lime_surf[c];
    v.lime_incorporated = s_.lime_inc[c];
    v.soil_temp_c = s_.soil_t[c];
    v.f_n = s_.f_n[c];
    v.f_pk = s_.f_pk[c];
    v.f_ph = s_.f_ph[c];
    v.lodging_loss = s_.lodge[c];
    return v;
}

NutrientLedger Field::nutrient_ledger(int cell) const {
    if (!cfg_.nutrients_enabled) throw std::logic_error("Field::nutrient_ledger: the field runs without nutrients");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::nutrient_ledger: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell);
    NutrientLedger l;
    l.n_initial = s_.l_n_init[c];
    l.n_fert_mineral = s_.l_n_fmin[c];
    l.n_fert_organic = s_.l_n_forg[c];
    l.n_deposition = s_.l_n_dep[c];
    l.n_fixation = s_.l_n_fix[c];
    l.n_removed = s_.l_n_rem[c];
    l.n_leached = s_.l_n_leach[c];
    l.n_volatilised = s_.l_n_nh3[c];
    l.n_denitrified = s_.l_n_den[c];
    l.p_initial = s_.l_p_init[c];
    l.p_applied = s_.l_p_app[c];
    l.p_removed = s_.l_p_rem[c];
    l.k_initial = s_.l_k_init[c];
    l.k_applied = s_.l_k_app[c];
    l.k_removed = s_.l_k_rem[c];
    l.cao_applied = s_.l_ca_app[c];
    l.cao_lost = s_.l_ca_loss[c];
    l.n_mineralised = s_.l_n_min[c];
    return l;
}

real Field::n_balance_residual(int cell) const {
    const NutrientLedger l = nutrient_ledger(cell);
    return l.n_initial + l.n_fert_mineral + l.n_fert_organic + l.n_deposition + l.n_fixation - l.n_removed - l.n_leached -
           l.n_volatilised - l.n_denitrified - n_storage(static_cast<std::size_t>(cell));
}

real Field::p_balance_residual(int cell) const {
    const NutrientLedger l = nutrient_ledger(cell);
    const std::size_t c = static_cast<std::size_t>(cell);
    return l.p_initial + l.p_applied - l.p_removed - (s_.p_av[c] + s_.p_res[c] + s_.p_crop[c]);
}

real Field::k_balance_residual(int cell) const {
    const NutrientLedger l = nutrient_ledger(cell);
    const std::size_t c = static_cast<std::size_t>(cell);
    return l.k_initial + l.k_applied - l.k_removed - (s_.k_av[c] + s_.k_res[c] + s_.k_crop[c]);
}

real Field::cao_balance_residual(int cell) const {
    const NutrientLedger l = nutrient_ledger(cell);
    const std::size_t c = static_cast<std::size_t>(cell);
    return l.cao_applied - l.cao_lost - (s_.alk[c] + s_.lime_surf[c] + s_.lime_inc[c]);
}

} // namespace ps::field
