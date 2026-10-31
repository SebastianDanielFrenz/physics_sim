// core/src/field/field.cpp - see ps/field/field.h for the model overview.
//
// Equation sources: FAO-56 (Allen et al. 1998) eq 69-77 (dual crop coefficient,
// surface evaporation), eq 84-88 (root-zone depletion, Ks), Table 22 note (p
// adjustment); AquaCrop (Raes et al. 2012 reference manual; Steduto et al.
// 2009) for the canopy, biomass and harvest-index laws; Wang & Engel 1998 for
// the vernalisation response; USDA NRCS NEH-4 for the curve number method.
// Every tunable number is read from the soil/crop descriptors; what remains
// below are the constants of the published equations.

#include "ps/field/field.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ps::field {

namespace {

inline real clamp01(real x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
inline real clampr(real x, real lo, real hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Daily growing degree days with both temperatures clamped to [t_base, t_upper]
// (AquaCrop GDD method 2).
real thermal_time(real tmin, real tmax, real tb, real tu) {
    return 0.5 * (clampr(tmax, tb, tu) + clampr(tmin, tb, tu)) - tb;
}

// Wang & Engel (1998) vernalisation response, days of vernalisation per day.
real vernalisation_rate(real t, real tmin, real topt, real tmax) {
    if (t <= tmin || t >= tmax) return 0.0;
    const real alpha = ps::math::log(2.0) / ps::math::log((tmax - tmin) / (topt - tmin));
    const real a = t - tmin, b = topt - tmin;
    const real aa = ps::math::pow(a, alpha), ba = ps::math::pow(b, alpha);
    return (2.0 * aa * ba - aa * aa) / (ba * ba);
}

// AquaCrop stress coefficient curve: 1 at relative depletion <= p_upper, 0 at
// >= p_lower, convex for shape > 0, linear for shape = 0.
real ks_curve(real dep_frac, real p_upper, real p_lower, real shape) {
    const real drel = clamp01((dep_frac - p_upper) / (p_lower - p_upper));
    if (drel <= 0.0) return 1.0;
    if (shape == 0.0) return 1.0 - drel;
    return 1.0 - (ps::math::exp(shape * drel) - 1.0) / (ps::math::exp(shape) - 1.0);
}

// AquaCrop flowering distribution (Eq. 3.12i): f(k) = 0.00558 k^0.63 - 0.000969 k - 0.00383, k = % of the
// flowering duration. flowering_cdf is its integral from 0 to k, normalised to 1 at k = 100 (the raw
// integral is 1.001).
real flowering_cdf(real k) {
    if (k <= 0.0) return 0.0;
    const auto raw = [](real x) { return 0.00558 * ps::math::pow(x, 1.63) / 1.63 - 0.000969 * x * x / 2.0 - 0.00383 * x; };
    return raw(k) / raw(100.0);
}

// Logistic stress curve on x in [0, 1], rescaled to hit exactly 0 at 0 and 1 at 1.
real logistic_s(real x, real steep) {
    const auto l = [steep](real t) { return 1.0 / (1.0 + ps::math::exp(-steep * (t - 0.5))); };
    return (l(x) - l(0.0)) / (l(1.0) - l(0.0));
}

// SCS runoff for one day (mm), curve number cn.
real scs_runoff(real liquid, real cn, real ia_ratio) {
    if (liquid <= 0.0) return 0.0;
    const real s = 25.4 * (1000.0 / cn - 10.0);
    const real ia = ia_ratio * s;
    if (liquid <= ia) return 0.0;
    const real q = liquid - ia;
    return q * q / (q + s);
}

inline std::uint64_t fnv_bytes(std::uint64_t h, const void* p, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ull;
    }
    return h;
}
template <class T>
std::uint64_t fold_vec(std::uint64_t h, const std::vector<T>& v) {
    const std::uint64_t n = v.size();
    h = fnv_bytes(h, &n, sizeof n);
    return v.empty() ? h : fnv_bytes(h, v.data(), v.size() * sizeof(T));
}

int stage_of(const CropDesc& cr, real gdd) {
    int st = 0;
    for (int i = 1; i < cr.stage_count(); ++i)
        if (gdd >= cr.stages[static_cast<std::size_t>(i)].gdd) st = i;
    return st;
}

real bbch_of(const CropDesc& cr, real gdd) {
    const int n = cr.stage_count();
    const int st = stage_of(cr, gdd);
    if (st >= n - 1) return cr.stages[static_cast<std::size_t>(n - 1)].bbch;
    const CropStageDesc& a = cr.stages[static_cast<std::size_t>(st)];
    const CropStageDesc& b = cr.stages[static_cast<std::size_t>(st + 1)];
    return a.bbch + (b.bbch - a.bbch) * (gdd - a.gdd) / (b.gdd - a.gdd);
}

} // namespace

Field::Field(const FieldConfig& config) : cfg_(config), nx_(config.nx), ny_(config.ny) {
    if (nx_ < 1 || ny_ < 1) throw std::invalid_argument("Field: nx and ny must be >= 1");
    if (!(cfg_.cell_size_m > 0.0)) throw std::invalid_argument("Field: cell_size_m must be > 0");
    if (cfg_.soils.empty()) throw std::invalid_argument("Field: at least one soil is required");
    if (cfg_.crops.size() > 254) throw std::invalid_argument("Field: at most 254 crops");
    if (cfg_.initial_depletion_fraction < 0.0 || cfg_.initial_depletion_fraction > 1.0)
        throw std::invalid_argument("Field: initial_depletion_fraction must be in [0, 1]");
    for (const SoilDesc& so : cfg_.soils) {
        const std::string e = soil_validation_error(so);
        if (!e.empty()) throw std::invalid_argument("Field: " + e);
    }
    for (const CropDesc& cr : cfg_.crops) {
        const std::string e = crop_validation_error(cr);
        if (!e.empty()) throw std::invalid_argument("Field: " + e);
    }
    if (!(cfg_.co2_ppm > 0.0)) throw std::invalid_argument("Field: co2_ppm must be > 0");
    for (const CropDesc& cr : cfg_.crops) co2_f_.push_back(cr.co2_factor(cfg_.co2_ppm));
    if (cfg_.management_enabled && !cfg_.nutrients_enabled) throw std::invalid_argument("Field: management_enabled needs nutrients_enabled");
    if (cfg_.pathogens_enabled && !cfg_.management_enabled) throw std::invalid_argument("Field: pathogens_enabled needs management_enabled");
    if (cfg_.nutrients_enabled) {
        for (const SoilDesc& so : cfg_.soils)
            if (!so.has_nutrients) throw std::invalid_argument("Field: soil " + so.name + " has no nutrient block");
        for (const CropDesc& cr : cfg_.crops)
            if (!cr.has_nutrients) throw std::invalid_argument("Field: crop " + cr.name + " has no nutrient block");
        for (const MaterialDesc& m : cfg_.materials) {
            const std::string e = material_validation_error(m);
            if (!e.empty()) throw std::invalid_argument("Field: " + e);
        }
        for (const MethodDesc& m : cfg_.methods) {
            const std::string e = method_validation_error(m);
            if (!e.empty()) throw std::invalid_argument("Field: " + e);
        }
    }
    const std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    if (!cfg_.cell_soil.empty() && cfg_.cell_soil.size() != n)
        throw std::invalid_argument("Field: cell_soil must be empty or have nx * ny entries");
    soil_idx_.assign(n, 0);
    for (std::size_t i = 0; i < cfg_.cell_soil.size(); ++i) {
        if (cfg_.cell_soil[i] >= cfg_.soils.size()) throw std::invalid_argument("Field: cell_soil index out of range");
        soil_idx_[i] = cfg_.cell_soil[i];
    }
    cfg_.cell_soil.clear();

    s_.last_day = cfg_.initial_day;
    auto fill = [n](std::vector<real>& v, real x) { v.assign(n, x); };
    for (auto* v : {&s_.swe, &s_.d_root, &s_.d_sub, &s_.d_evap, &s_.zr, &s_.gdd_sow, &s_.gdd_em, &s_.vern_days,
                    &s_.cc, &s_.cc_peak, &s_.t_sen, &s_.biomass, &s_.grain, &s_.hi_gdd, &s_.pol_sum, &s_.pol_n,
                    &s_.sto_sum, &s_.sto_n, &s_.days_ripe, &s_.last_tr, &s_.last_e, &s_.last_runoff, &s_.last_drain,
                    &s_.cum_precip, &s_.cum_runoff, &s_.cum_evap, &s_.cum_tr, &s_.cum_drain, &s_.cum_irrig,
                    &s_.initial_storage})
        fill(*v, 0.0);
    for (auto* v : {&s_.stand, &s_.stress_n, &s_.stress_pk, &s_.stress_ph, &s_.stress_weed, &s_.stress_rotation,
                    &s_.residue_evap_factor, &s_.last_ks_sto, &s_.last_ks_exp})
        fill(*v, 1.0);
    s_.crop.assign(n, 0);
    s_.stage.assign(n, 0);
    s_.weed_f.assign(n, 1.0);
    for (std::size_t c = 0; c < n; ++c) {
        const SoilDesc& so = soil_of(c);
        s_.zr[c] = so.fallow_root_depth_m;
        const real f = cfg_.initial_depletion_fraction;
        s_.d_root[c] = f * so.taw_mm_per_m() * so.fallow_root_depth_m;
        s_.d_sub[c] = f * so.taw_mm_per_m() * (so.depth_m - so.fallow_root_depth_m);
        s_.d_evap[c] = f * so.tew_mm();
        s_.initial_storage[c] = storage_mm(static_cast<int>(c));
    }
    if (cfg_.management_enabled) init_management();   // before the nutrients: their initial N includes the weeds'
    if (cfg_.nutrients_enabled) init_nutrients();
    if (cfg_.pathogens_enabled) init_pathogens();
}

int Field::cell_at_world(real x, real y) const {
    const real fx = std::floor((x - cfg_.origin_x_m) / cfg_.cell_size_m);
    const real fy = std::floor((y - cfg_.origin_y_m) / cfg_.cell_size_m);
    if (!(fx >= 0.0 && fy >= 0.0 && fx < nx_ && fy < ny_)) return -1;
    return cell_index(static_cast<int>(fx), static_cast<int>(fy));
}

// ---------------------------------------------------------------------------
// root depth change with conservation of water
// ---------------------------------------------------------------------------
void Field::set_root_depth(std::size_t c, real z_new) {
    const SoilDesc& so = soil_of(c);
    z_new = std::min(z_new, so.depth_m);
    const real z_old = s_.zr[c];
    if (z_new > z_old) {
        const real below = so.depth_m - z_old;
        if (below > 0.0) {
            const real moved = s_.d_sub[c] * ((z_new - z_old) / below);
            s_.d_root[c] += moved;
            s_.d_sub[c] -= moved;
            if (cfg_.nutrients_enabled) {   // the nitrate of the strip the roots take over moves with it
                const real mn = s_.no3_2[c] * ((z_new - z_old) / below);
                s_.no3_1[c] += mn;
                s_.no3_2[c] -= mn;
            }
        }
    } else if (z_new < z_old) {
        const real moved = s_.d_root[c] * ((z_old - z_new) / z_old);
        s_.d_root[c] -= moved;
        s_.d_sub[c] += moved;
        if (cfg_.nutrients_enabled) {
            const real mn = s_.no3_1[c] * ((z_old - z_new) / z_old);
            s_.no3_1[c] -= mn;
            s_.no3_2[c] += mn;
        }
    }
    s_.zr[c] = z_new;
}

// ---------------------------------------------------------------------------
// the daily step of one cell
// ---------------------------------------------------------------------------
void Field::step_cell(std::size_t c, const DailyWeather& w) {
    FieldState& s = s_;
    const SoilDesc& so = soil_of(c);

    if (irrigate_) {
        const real add = s.d_root[c] + s.d_sub[c];
        s.cum_irrig[c] += add;
        s.d_root[c] = 0.0;
        s.d_sub[c] = 0.0;
        s.d_evap[c] = 0.0;
    }

    const bool mgmt = cfg_.management_enabled;
    if (mgmt) manage_prestep(c, w);
    const bool has_crop = s.crop[c] != 0;
    const CropDesc* cr = has_crop ? &cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)] : nullptr;
    const real et0 = w.et0_mm;

    // --- development, frost, roots -------------------------------------------------
    bool emerged = false;
    real gdd = 0.0;
    int st = 0;
    if (cr) {
        st = s.stage[c];
        gdd = thermal_time(w.tmin_c, w.tmax_c, cr->t_base_c, cr->t_upper_c);
        // Frost kill of an emerged stand; snow cover shields it.
        if (st >= cr->stage_emergence) {
            real tmin_eff = w.tmin_c;
            if (s.swe[c] >= cr->snow_insulation_swe_mm) tmin_eff = std::max(tmin_eff, cr->frost_snow_floor_c);
            const real kill = clamp01((cr->frost_kill_onset_c - tmin_eff) / (cr->frost_kill_onset_c - cr->frost_kill_full_c));
            if (kill > 0.0) {
                s.cc[c] *= (1.0 - kill);
                s.stand[c] *= (1.0 - kill);
                s.cc_peak[c] = std::min(s.cc_peak[c], cr->ccx * s.stand[c]);
            }
        }
        real dev = gdd;
        if (cr->vernalises) {
            if (st < cr->stage_vernalisation_end) {   // vernalising seeds and plants; only the emerged plant is slowed
                s.vern_days[c] = std::min(cr->vern_days_saturation,
                                          s.vern_days[c] + vernalisation_rate(w.tmean_c, cr->vern_t_min_c,
                                                                              cr->vern_t_opt_c, cr->vern_t_max_c));
                const real vf = cr->vern_min_rate_factor +
                                (1.0 - cr->vern_min_rate_factor) * clamp01(s.vern_days[c] / cr->vern_days_saturation);
                if (st >= cr->stage_emergence) dev = gdd * vf;
            }
        }
        if (mgmt && s.sow_seeds[c] > 0.0 && st < cr->stage_emergence) {   // a sown seed needs a moist seed layer
            const CropManagementDesc& cm = cr->management;
            const real tew_s = so.tew_mm();
            const real wet = tew_s > 0.0 ? clamp01(1.0 - s.d_evap[c] / tew_s) : 1.0;
            dev *= clamp01((wet - cm.germ_wet_min) / (cm.germ_wet_full - cm.germ_wet_min));
            if (wet < cm.germ_wet_min) s.sow_viab[c] *= 1.0 - cm.dry_hazard_per_day;
        }
        const int prev = st;
        s.gdd_sow[c] += dev;
        st = stage_of(*cr, s.gdd_sow[c]);
        s.stage[c] = static_cast<std::uint8_t>(st);
        emerged = st >= cr->stage_emergence;
        if (emerged) {
            if (prev < cr->stage_emergence) {
                if (mgmt) establish(c, *cr);   // plants from the seeds sown (sow() stored the full stand otherwise)
                s.cc_peak[c] = s.cc[c];
                if (cfg_.pathogens_enabled) pathogen_emergence(c);
            }
            else s.gdd_em[c] += gdd;
            const real t_em = cr->stages[static_cast<std::size_t>(cr->stage_emergence)].gdd;
            const real frac = clamp01((s.gdd_sow[c] - t_em) / (cr->root_gdd_full - t_em));
            real zt = cr->zr_min_m + (cr->zr_max_m - cr->zr_min_m) * frac;
            if (mgmt) zt *= 1.0 - so.management.damage_root_penalty * s.damage[c];
            if (zt > s.zr[c]) set_root_depth(c, zt);
            if (cfg_.pathogens_enabled) s.stress_rotation[c] = 1.0 - s.path_loss0[c] * (1.0 - fungicide_protection(c));
        }
    }

    const real taw1 = so.taw_mm_per_m() * s.zr[c];
    const real dep_frac = taw1 > 0.0 ? clamp01(s.d_root[c] / taw1) : 1.0;

    // --- canopy --------------------------------------------------------------------
    real ks_exp = 1.0, ks_sen = 1.0;
    if (emerged) {
        ks_exp = ks_curve(dep_frac, cr->exp_p_upper, cr->exp_p_lower, cr->exp_shape);
        ks_sen = ks_curve(dep_frac, cr->sen_p_upper, 1.0, cr->sen_shape);
        if (st < cr->stage_senescence) {
            const real ccx_eff = cr->ccx * s.stand[c];
            const real g = cr->cgc_per_gdd * ks_exp * gdd;
            if (s.cc[c] < 0.5 * ccx_eff) s.cc[c] *= ps::math::exp(g);
            else s.cc[c] = ccx_eff - (ccx_eff - s.cc[c]) * ps::math::exp(-g);
            if (s.cc[c] > ccx_eff) s.cc[c] = ccx_eff;
            s.cc_peak[c] = std::max(s.cc_peak[c], s.cc[c]);
        } else {
            s.t_sen[c] += gdd * (1.0 + cr->senescence_acceleration * (1.0 - ks_sen));
            const real peak = s.cc_peak[c];
            if (peak > 0.0) {
                const real v = peak * (1.0 - 0.05 * (ps::math::exp(cr->cdc_per_gdd / peak * s.t_sen[c]) - 1.0));
                s.cc[c] = std::max(0.0, std::min(s.cc[c], v));
            } else {
                s.cc[c] = 0.0;
            }
        }
    }
    const real cc = emerged ? s.cc[c] : 0.0;

    // --- snow, runoff ----------------------------------------------------------------
    s.cum_precip[c] += w.rain_mm + w.snow_mm;
    s.swe[c] += w.snow_mm;
    const real melt = std::min(s.swe[c], so.snow_melt_mm_per_k_day * std::max(0.0, w.tmean_c - so.snow_melt_base_c));
    s.swe[c] -= melt;
    const real liquid = w.rain_mm + melt;
    real runoff = 0.0;
    if (liquid > 0.0) {
        real cn2 = so.curve_number_ii;
        if (mgmt) cn2 = std::max(30.0, std::min(98.0, cn2 + so.management.damage_cn_increase * s.damage[c] - so.management.loose_cn_decrease * s.loose[c]));
        const real cn1 = cn2 / (2.281 - 0.01281 * cn2);
        const real cn3 = cn2 / (0.427 + 0.00573 * cn2);
        const real wet = taw1 > 0.0 ? clamp01(1.0 - s.d_root[c] / taw1) : 1.0;
        const real wn = so.cn_normal_wetness;
        const real cn = wet < wn ? cn1 + (cn2 - cn1) * wet / wn : cn2 + (cn3 - cn2) * (wet - wn) / (1.0 - wn);
        runoff = scs_runoff(liquid, cn, so.ia_ratio);
    }
    const real infil = liquid - runoff;

    // --- evaporation and transpiration demand (dual crop coefficient) -----------------
    const real tew = so.tew_mm();
    const real cc_star = std::min(1.0, 1.72 * cc - cc * cc + 0.3 * cc * cc * cc);   // AquaCrop micro-advection adjustment
    const real kcb = emerged ? cr->kc_tr_x * cc_star : 0.0;
    const real height = emerged ? cr->height_max_m * clamp01(cc / cr->ccx) : 0.0;
    const real kc_max = std::max(
        1.2 + (0.04 * (w.wind2_m_s - 2.0) - 0.004 * (w.rh_min * 100.0 - 45.0)) * ps::math::pow(height / 3.0, 0.3),
        kcb + 0.05);
    const real few = 1.0 - cc_star;
    real de_a = s.d_evap[c] - infil;
    if (de_a < 0.0) de_a = 0.0;
    real kr = de_a <= so.rew_mm ? 1.0 : std::max(0.0, (tew - de_a) / (tew - so.rew_mm));
    if (mgmt && kr < 1.0) kr *= s.tilth_e2[c];   // loose, dry surface soil is a mulch against stage-2 evaporation
    real ke = std::min(kr * std::max(0.0, kc_max - kcb), few * kc_max);
    if (s.swe[c] > 0.0) ke = 0.0;
    real evap = ke * et0 * s.residue_evap_factor[c];
    // Wet soil evaporates at most at the energy-limited rate; in the cold season the aerodynamic share of the
    // reference evapotranspiration is not available to the soil (F1c fix A). Stage 2 scales like stage 1.
    if (w.e_wet_mm >= 0.0) evap = std::min(evap, kr * few * w.e_wet_mm * s.residue_evap_factor[c]);
    if (few > 1.0e-9) evap = std::min(evap, few * (tew - de_a));
    else evap = 0.0;
    evap = std::max(0.0, evap);

    real ks_sto = 1.0;
    if (emerged) {
        real p = cr->p_depletion;
        if (cr->p_adjust_for_etc) p = clampr(p + 0.04 * (5.0 - (kcb * et0 + evap)), 0.1, 0.8);
        const real raw = p * taw1;
        ks_sto = s.d_root[c] <= raw ? 1.0 : std::max(0.0, (taw1 - s.d_root[c]) / ((1.0 - p) * taw1));
    }
    real tr = ks_sto * kcb * et0;

    // --- root-zone water balance (FAO-56 eq 85, 88) -------------------------------------
    real eta = tr + evap;
    const real cap = std::max(0.0, taw1 - s.d_root[c] + infil);
    if (eta > cap) {
        const real k = cap / eta;
        tr *= k;
        evap *= k;
        eta = cap;
    }
    real d1 = s.d_root[c] - infil + eta;
    real dp = 0.0;
    if (d1 < 0.0) {
        dp = -d1;
        d1 = 0.0;
    }
    s.d_root[c] = d1;
    real drain = 0.0;
    if (dp > s.d_sub[c]) {
        drain = dp - s.d_sub[c];
        s.d_sub[c] = 0.0;
    } else {
        s.d_sub[c] -= dp;
    }
    s.d_evap[c] = few > 1.0e-9 ? std::min(tew, de_a + evap / few) : de_a;
    s.cum_runoff[c] += runoff;
    s.cum_evap[c] += evap;
    s.cum_tr[c] += tr;
    s.cum_drain[c] += drain;
    s.last_tr[c] = tr;
    s.last_e[c] = evap;
    s.last_runoff[c] = runoff;
    s.last_drain[c] = drain;
    s.last_ks_sto[c] = ks_sto;
    s.last_ks_exp[c] = ks_exp;

    // --- biomass, harvest index, ripeness ------------------------------------------------
    const real bio_before = s.biomass[c];
    real shed_g = 0.0;
    real nf_n = 1.0, nf_pk = 1.0, nf_ph = 1.0;
    if (cfg_.nutrients_enabled) {
        nutrient_growth_factors(c, st, emerged, nf_n, nf_pk, nf_ph);
        s.f_n[c] = nf_n;
        s.f_pk[c] = nf_pk;
        s.f_ph[c] = nf_ph;
    }
    if (emerged) {
        if (st < cr->stage_maturity) {
            if (et0 > 1.0e-9 && cc > 0.0) {
                const real ft = cr->biomass_gdd_full > 0.0 ? clamp01(gdd / cr->biomass_gdd_full) : 1.0;
                const real f_stress = std::min(std::min(s.stress_n[c] * nf_n, s.stress_pk[c] * nf_pk),
                                               std::min(s.stress_ph[c] * nf_ph, s.stress_weed[c])) *
                                      s.stress_rotation[c];
                const real f_rad = cr->biomass_rs_ref_mj_m2 > 0.0 ? clamp01(w.rs_mj_m2 / cr->biomass_rs_ref_mj_m2) : 1.0;
                const real f_co2 = co2_f_[static_cast<std::size_t>(s.crop[c] - 1)];
                real gain = cr->wp_star_g_m2 * f_co2 * (tr / et0) * ft * f_rad;
                if (cr->biomass_rue_g_mj > 0.0) {
                    const real ft_rue = cr->biomass_rue_t_full_c > 0.0
                                            ? clamp01((w.tmean_c - cr->biomass_rue_t_low_c) / (cr->biomass_rue_t_full_c - cr->biomass_rue_t_low_c))
                                            : 1.0;
                    const real light = cr->biomass_rue_g_mj * f_co2 * ft_rue * w.rs_mj_m2 * cc;
                    if (cr->biomass_light_primary) {
                        const real tr_pot = kcb * et0;
                        gain = light * (tr_pot > 1.0e-9 ? tr / tr_pot : 1.0);
                    } else {
                        gain = std::min(gain, light);
                    }
                }
                s.biomass[c] += gain * f_stress * s.weed_f[c];
            }
            if (st >= cr->stage_flowering) {
                // Harvest index (AquaCrop ch. 3.12): pollination during the flowering distribution, then
                // the stomatal-closure effect over the yield-formation days.
                const real k0 = clampr(100.0 * s.hi_gdd[c] / cr->flowering_length_gdd, 0.0, 100.0);
                s.hi_gdd[c] += gdd;
                const real k1 = clampr(100.0 * s.hi_gdd[c] / cr->flowering_length_gdd, 0.0, 100.0);
                if (cr->pollination && k1 > k0) {
                    const real fj = flowering_cdf(k1) - flowering_cdf(k0);
                    const real ks_w = dep_frac <= cr->ppol ? 1.0 : std::max(0.0, (1.0 - dep_frac) / (1.0 - cr->ppol));
                    const real ks_c = 1.0 - logistic_s(clamp01((cr->pol_cold_t_c - w.tmin_c) / cr->pol_cold_width_k),
                                                       cr->pol_logistic_steepness);
                    const real ks_h = 1.0 - logistic_s(clamp01((w.tmax_c - cr->pol_heat_t_c) / cr->pol_heat_width_k),
                                                       cr->pol_logistic_steepness);
                    s.pol_sum[c] += fj * std::min(ks_w, std::min(ks_c, ks_h));
                    s.pol_n[c] += fj;
                }
                if (s.hi_gdd[c] <= cr->hi_build_gdd) {
                    const real r10 = ks_sto > 0.0 ? ps::math::pow(ks_sto, 0.1) : 0.0;
                    s.sto_sum[c] += 1.0 - (1.0 - r10) / cr->hi_sto_b;
                    s.sto_n[c] += 1.0;
                }
                const real f_pol = std::min(1.0, (1.0 + cr->excess_fruits_pct / 100.0) * (s.pol_sum[c] + (1.0 - s.pol_n[c])));
                const real f_sto = s.sto_n[c] > 0.0 ? s.sto_sum[c] / s.sto_n[c] : 1.0;
                const real hi = cr->hi0 * f_pol * f_sto * clamp01(s.hi_gdd[c] / cr->hi_build_gdd);
                s.grain[c] = s.biomass[c] * hi * (1.0 - (cfg_.nutrients_enabled ? s.lodge[c] : 0.0));
            }
        } else {
            s.days_ripe[c] += 1.0;
            if (s.days_ripe[c] > cr->harvest_window_days) {
                shed_g = s.grain[c] * cr->shed_rate_per_day;
                s.grain[c] *= (1.0 - cr->shed_rate_per_day);
            }
        }
    }

    if (cfg_.nutrients_enabled) {
        DayFlows fl;
        fl.taw1 = taw1;
        fl.dp = dp;
        fl.drain = drain;
        fl.bio_before = bio_before;
        fl.bbch = cr ? bbch_of(*cr, s.gdd_sow[c]) : 0.0;
        fl.stage = st;
        fl.emerged = emerged;
        fl.shed_g_m2 = shed_g;
        fl.gdd = gdd;
        step_nutrients(c, w, fl);
        if (mgmt) manage_poststep(c, w, fl);
        if (cfg_.pathogens_enabled) step_pathogens(c, fl);
    }
}

real Field::crop_bbch(std::size_t c) const {
    if (s_.crop[c] == 0) return 0.0;
    return bbch_of(cfg_.crops[static_cast<std::size_t>(s_.crop[c] - 1)], s_.gdd_sow[c]);
}

void Field::step_cells(int begin, int end, const DailyWeather& w) {
    if (begin < 0 || end > cell_count() || begin > end) throw std::out_of_range("Field::step_cells: bad cell range");
    for (int c = begin; c < end; ++c) step_cell(static_cast<std::size_t>(c), w);
}

void Field::finish_day(std::int64_t day) {
    if (day != s_.last_day + 1) throw std::invalid_argument("Field::finish_day: days must be consecutive");
    s_.last_day = day;
}

void Field::step_day(const DailyWeather& w) {
    if (w.day != s_.last_day + 1)
        throw std::invalid_argument("Field::step_day: expected day " + std::to_string(s_.last_day + 1) + ", got " +
                                    std::to_string(w.day));
    step_cells(0, cell_count(), w);
    s_.last_day = w.day;
}

void Field::advance_to(std::int64_t day, DailyWeatherSource& source) {
    if (day < s_.last_day) throw std::invalid_argument("Field::advance_to: cannot go back in time");
    while (s_.last_day < day) step_day(source.record(s_.last_day + 1));
}

// ---------------------------------------------------------------------------
// operations
// ---------------------------------------------------------------------------
void Field::sow(int cell, int crop_id, real density_scale) {
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::sow: bad cell");
    if (crop_id < 1 || static_cast<std::size_t>(crop_id) > cfg_.crops.size())
        throw std::invalid_argument("Field::sow: unknown crop id");
    if (!(density_scale > 0.0)) throw std::invalid_argument("Field::sow: density_scale must be > 0");
    const std::size_t c = static_cast<std::size_t>(cell);
    const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(crop_id - 1)];
    if (cfg_.management_enabled) {
        if (s_.crop[c] != 0) crop_to_residue(c, 1.0, false);   // sown over: the old stand lies as residue
    } else if (cfg_.nutrients_enabled && s_.crop[c] != 0) {
        drop_crop_nutrients(c);   // sown over: the old stand is ploughed in
    }
    s_.crop[c] = static_cast<std::uint8_t>(crop_id);
    s_.stage[c] = 0;
    s_.gdd_sow[c] = 0.0;
    s_.gdd_em[c] = 0.0;
    s_.vern_days[c] = 0.0;
    s_.cc[c] = cr.cc0(density_scale);   // the seedling canopy waits here until emergence
    s_.cc_peak[c] = 0.0;
    s_.t_sen[c] = 0.0;
    s_.biomass[c] = 0.0;
    s_.grain[c] = 0.0;
    s_.stand[c] = 1.0;
    s_.hi_gdd[c] = 0.0;
    s_.pol_sum[c] = 0.0;
    s_.pol_n[c] = 0.0;
    s_.sto_sum[c] = 0.0;
    s_.sto_n[c] = 0.0;
    s_.days_ripe[c] = 0.0;
    set_root_depth(c, cr.zr_min_m);
    if (cfg_.management_enabled) {   // a full stand by construction, no establishment from seed
        s_.sow_seeds[c] = -1.0;
        s_.sow_fe[c] = 1.0;
        s_.sow_viab[c] = 1.0;
        s_.plants[c] = cr.plant_density_per_ha * 1.0e-4 * density_scale;
    }
}

void Field::sow_all(int crop_id, real density_scale) {
    for (int c = 0; c < cell_count(); ++c) sow(c, crop_id, density_scale);
}

HarvestResult Field::harvest(int cell, real straw_removed) {
    if (!(straw_removed >= 0.0 && straw_removed <= 1.0)) throw std::invalid_argument("Field::harvest: straw_removed must be in [0, 1]");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::harvest: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell);
    HarvestResult r;
    if (s_.crop[c] == 0) return r;
    if (cfg_.management_enabled) {   // the same path as a harvest operation: straw lies chopped or leaves, the rest is residue
        OperationEffect e;
        e.harvest.harvest = true;
        e.harvest.straw_carried = straw_removed;
        e.harvest.straw_chopped = 0.5;
        apply(cell, e, &r);
        return r;
    }
    const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(s_.crop[c] - 1)];
    r.crop_id = s_.crop[c];
    r.grain_dm_t_ha = s_.grain[c] * 0.01;
    const real ng = std::max(0.0, s_.biomass[c] - s_.grain[c]);
    const real share_h = cr.straw_harvest_fraction;   // harvestable share of the non-grain biomass; the rest is stubble
    r.straw_dm_t_ha = ng * share_h * 0.01;
    r.stubble_dm_t_ha = ng * (1.0 - share_h) * 0.01;
    r.bbch = bbch_of(cr, s_.gdd_sow[c]);
    if (cfg_.nutrients_enabled) {
        const CropNutrientDesc& cn = cr.nutrients;
        r.n_grain_kg_ha = cn.n_grain_share * s_.n_crop[c];
        r.n_straw_kg_ha = s_.n_crop[c] - r.n_grain_kg_ha;
        r.n_root_kg_ha = s_.n_root[c];
        r.p2o5_grain_kg_ha = cn.p2o5_grain_share * s_.p_crop[c];
        r.p2o5_straw_kg_ha = s_.p_crop[c] - r.p2o5_grain_kg_ha;
        r.k2o_grain_kg_ha = cn.k2o_grain_share * s_.k_crop[c];
        r.k2o_straw_kg_ha = s_.k_crop[c] - r.k2o_grain_kg_ha;
        r.lodging_loss = s_.lodge[c];
        const real taken = straw_removed * share_h;   // share of the straw nutrients that leave
        s_.l_n_rem[c] += r.n_grain_kg_ha + taken * r.n_straw_kg_ha;
        s_.l_p_rem[c] += r.p2o5_grain_kg_ha + taken * r.p2o5_straw_kg_ha;
        s_.l_k_rem[c] += r.k2o_grain_kg_ha + taken * r.k2o_straw_kg_ha;
        s_.org_fresh[c] += (1.0 - taken) * r.n_straw_kg_ha + s_.n_root[c];
        s_.p_av[c] += (1.0 - taken) * r.p2o5_straw_kg_ha;
        s_.k_av[c] += (1.0 - taken) * r.k2o_straw_kg_ha;
        s_.n_crop[c] = s_.n_root[c] = s_.p_crop[c] = s_.k_crop[c] = s_.lodge[c] = 0.0;
    }
    s_.crop[c] = 0;
    s_.stage[c] = 0;
    s_.gdd_sow[c] = s_.gdd_em[c] = s_.vern_days[c] = s_.cc[c] = s_.cc_peak[c] = s_.t_sen[c] = 0.0;
    s_.biomass[c] = s_.grain[c] = s_.hi_gdd[c] = s_.pol_sum[c] = s_.pol_n[c] = s_.sto_sum[c] = s_.sto_n[c] = s_.days_ripe[c] = 0.0;
    s_.stand[c] = 1.0;
    set_root_depth(c, soil_of(c).fallow_root_depth_m);
    return r;
}

HarvestResult Field::harvest_all(real straw_removed) {
    HarvestResult m;
    int n = 0;
    for (int c = 0; c < cell_count(); ++c) {
        const HarvestResult r = harvest(c, straw_removed);
        if (r.crop_id == 0) continue;
        m.crop_id = r.crop_id;
        m.grain_dm_t_ha += r.grain_dm_t_ha;
        m.straw_dm_t_ha += r.straw_dm_t_ha;
        m.stubble_dm_t_ha += r.stubble_dm_t_ha;
        m.bbch += r.bbch;
        m.n_grain_kg_ha += r.n_grain_kg_ha;
        m.n_straw_kg_ha += r.n_straw_kg_ha;
        m.n_root_kg_ha += r.n_root_kg_ha;
        m.p2o5_grain_kg_ha += r.p2o5_grain_kg_ha;
        m.p2o5_straw_kg_ha += r.p2o5_straw_kg_ha;
        m.k2o_grain_kg_ha += r.k2o_grain_kg_ha;
        m.k2o_straw_kg_ha += r.k2o_straw_kg_ha;
        m.lodging_loss += r.lodging_loss;
        ++n;
    }
    if (n > 0) {
        for (real* x : {&m.grain_dm_t_ha, &m.straw_dm_t_ha, &m.stubble_dm_t_ha, &m.bbch, &m.n_grain_kg_ha, &m.n_straw_kg_ha, &m.n_root_kg_ha,
                        &m.p2o5_grain_kg_ha, &m.p2o5_straw_kg_ha, &m.k2o_grain_kg_ha, &m.k2o_straw_kg_ha, &m.lodging_loss})
            *x /= n;
    }
    return m;
}

void Field::set_stress(int cell, const StressFactors& f) {
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::set_stress: bad cell");
    for (real v : {f.n, f.pk, f.ph, f.weed, f.rotation})
        if (!(v >= 0.0 && v <= 1.0)) throw std::invalid_argument("Field::set_stress: factors must be in [0, 1]");
    if (cfg_.pathogens_enabled && f.rotation != 1.0)
        throw std::logic_error("Field::set_stress: the pathogen model owns the rotation factor (pathogens_enabled)");
    const std::size_t c = static_cast<std::size_t>(cell);
    s_.stress_n[c] = f.n;
    s_.stress_pk[c] = f.pk;
    s_.stress_ph[c] = f.ph;
    s_.stress_weed[c] = f.weed;
    s_.stress_rotation[c] = f.rotation;
}

// ---------------------------------------------------------------------------
// queries and persistence
// ---------------------------------------------------------------------------
CellView Field::view(int cell) const {
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::view: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell);
    const SoilDesc& so = soil_of(c);
    CellView v;
    v.soil = soil_idx_[c];
    v.crop_id = s_.crop[c];
    v.stage = s_.stage[c];
    if (v.crop_id != 0) {
        const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(v.crop_id - 1)];
        v.bbch = bbch_of(cr, s_.gdd_sow[c]);
        v.cc = s_.stage[c] >= cr.stage_emergence ? s_.cc[c] : 0.0;
    }
    v.gdd_sow = s_.gdd_sow[c];
    v.vern_days = s_.vern_days[c];
    v.stand = s_.stand[c];
    v.biomass_g_m2 = s_.biomass[c];
    v.grain_g_m2 = s_.grain[c];
    v.days_ripe = s_.days_ripe[c];
    v.zr_m = s_.zr[c];
    v.d_root_mm = s_.d_root[c];
    v.d_sub_mm = s_.d_sub[c];
    v.d_evap_mm = s_.d_evap[c];
    v.swe_mm = s_.swe[c];
    v.taw_root_mm = so.taw_mm_per_m() * s_.zr[c];
    v.tew_mm = so.tew_mm();
    v.last_tr_mm = s_.last_tr[c];
    v.last_e_mm = s_.last_e[c];
    v.last_runoff_mm = s_.last_runoff[c];
    v.last_drain_mm = s_.last_drain[c];
    v.last_ks_sto = s_.last_ks_sto[c];
    v.last_ks_exp = s_.last_ks_exp[c];
    return v;
}

real Field::storage_mm(int cell) const {
    const std::size_t c = static_cast<std::size_t>(cell);
    return s_.swe[c] + soil_of(c).total_taw_mm() - s_.d_root[c] - s_.d_sub[c];
}

real Field::taw_root_mm(int cell) const {
    const std::size_t c = static_cast<std::size_t>(cell);
    return soil_of(c).taw_mm_per_m() * s_.zr[c];
}

WaterLedger Field::ledger(int cell) const {
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::ledger: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell);
    WaterLedger l;
    l.precipitation = s_.cum_precip[c];
    l.runoff = s_.cum_runoff[c];
    l.evaporation = s_.cum_evap[c];
    l.transpiration = s_.cum_tr[c];
    l.drainage = s_.cum_drain[c];
    l.irrigation = s_.cum_irrig[c];
    l.initial_storage = s_.initial_storage[c];
    return l;
}

real Field::water_balance_residual_mm(int cell) const {
    const WaterLedger l = ledger(cell);
    return l.initial_storage + l.precipitation + l.irrigation - l.runoff - l.evaporation - l.transpiration -
           l.drainage - storage_mm(cell);
}

FieldCheckpoint Field::checkpoint() const {
    FieldCheckpoint cp;
    cp.nx = nx_;
    cp.ny = ny_;
    cp.state = s_;
    return cp;
}

void Field::restore(const FieldCheckpoint& cp) {
    if (cp.nx != nx_ || cp.ny != ny_ || cp.state.d_root.size() != s_.d_root.size() ||
        cp.state.crop.size() != s_.crop.size() || cp.state.no3_1.size() != s_.no3_1.size() ||
        cp.state.res_c[0].size() != s_.res_c[0].size() || cp.state.path_p.size() != s_.path_p.size() ||
        cp.state.res_crop.size() != s_.res_crop.size())
        throw std::invalid_argument("Field::restore: checkpoint does not fit this grid");
    s_ = cp.state;
}

std::uint64_t Field::fold_hash(std::uint64_t h) const {
    h = fnv_bytes(h, &s_.last_day, sizeof s_.last_day);
    for (const auto* v : {&s_.swe, &s_.d_root, &s_.d_sub, &s_.d_evap, &s_.zr, &s_.gdd_sow, &s_.gdd_em, &s_.vern_days,
                          &s_.cc, &s_.cc_peak, &s_.t_sen, &s_.biomass, &s_.grain, &s_.stand, &s_.hi_gdd, &s_.pol_sum,
                          &s_.pol_n, &s_.sto_sum, &s_.sto_n, &s_.days_ripe, &s_.stress_n, &s_.stress_pk, &s_.stress_ph,
                          &s_.stress_weed, &s_.stress_rotation, &s_.residue_evap_factor, &s_.cum_precip,
                          &s_.cum_runoff, &s_.cum_evap, &s_.cum_tr, &s_.cum_drain, &s_.cum_irrig, &s_.initial_storage})
        h = fold_vec(h, *v);
    h = fold_vec(h, s_.crop);
    h = fold_vec(h, s_.stage);
    if (cfg_.nutrients_enabled) {
        for (const auto* v : {&s_.no3_1, &s_.no3_2, &s_.nh4, &s_.org_slow, &s_.org_fresh, &s_.tan_surf, &s_.kv_w, &s_.ki_w,
                              &s_.kir_w, &s_.inc_h, &s_.soil_t, &s_.n_crop, &s_.n_root, &s_.p_crop, &s_.k_crop, &s_.p_av,
                              &s_.p_res, &s_.k_av, &s_.k_res, &s_.alk, &s_.lime_surf, &s_.lime_inc, &s_.lodge, &s_.l_n_init,
                              &s_.l_n_fmin, &s_.l_n_forg, &s_.l_n_dep, &s_.l_n_fix, &s_.l_n_rem, &s_.l_n_leach, &s_.l_n_nh3,
                              &s_.l_n_den, &s_.l_n_min, &s_.l_p_init, &s_.l_p_app, &s_.l_p_rem, &s_.l_k_init, &s_.l_k_app,
                              &s_.l_k_rem, &s_.l_ca_app, &s_.l_ca_loss})
            h = fold_vec(h, *v);
    }
    if (cfg_.management_enabled) {
        for (const auto& a : {&s_.res_c, &s_.res_n})
            for (const auto& v : *a) h = fold_vec(h, v);
        for (const auto& v : s_.res_p) h = fold_vec(h, v);
        for (const auto& v : s_.res_k) h = fold_vec(h, v);
        for (const auto* v : {&s_.res_chop, &s_.loose, &s_.damage, &s_.seedbed, &s_.soc, &s_.tilth_e2, &s_.sow_seeds, &s_.sow_fe,
                              &s_.sow_viab, &s_.plants, &s_.w_seed_s, &s_.w_seed_a, &s_.w_seed_b, &s_.w_dens, &s_.w_bio,
                              &s_.w_tt, &s_.w_n, &s_.w_seed_pl, &s_.w_res_pl, &s_.w_res_sd, &s_.h_act, &s_.l_c_crop,
                              &s_.l_c_wg, &s_.l_c_rem, &s_.l_c_co2, &s_.weed_f})
            h = fold_vec(h, *v);
    }
    if (cfg_.pathogens_enabled) {
        h = fold_vec(h, s_.path_p);
        h = fold_vec(h, s_.path_s);
        h = fold_vec(h, s_.path_loss0);
        h = fold_vec(h, s_.res_crop);
    }
    return h;
}

} // namespace ps::field
