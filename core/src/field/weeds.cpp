// core/src/field/weeds.cpp - weeds and herbicides of Field (farming F1c). A weed group is a population per cell:
// a seedbank in three layers (surface, upper, lower), emerged plants (number and biomass), seed on the plants, and
// the share of plants and seed resistant to each mode of action. Everything is per cell and per day.
//
// Equation sources: triangular temperature response of germination and logistic growth in thermal time (common
// forms of weed emergence/growth models, e.g. Grundy 2003; Gonzalez-Andujar); the Cousens (1985) rectangular
// hyperbola for yield loss, loss% = I d / (1 + I d / A); a log-logistic dose-response (Streibig 1980; Seefeldt et
// al. 1995) for herbicide efficacy; selection by Hardy-Weinberg-free population fractions: the resistant share
// after a kill is r (1 - k_r) / ((1 - r)(1 - k_s) + r (1 - k_r)). All rates are in the group and herbicide
// descriptors; what is written below are the equations. Simplifications (documented in docs/data_formats/
// weed_group.md): weeds take no P or K; the carbon of the seed itself is not tracked. Water (F1d part 6): a group
// transpires from the root-zone bucket of the crop at kc_tr x cover x (1 - shade x crop cover) x ET0, reduced by its own
// FAO-56 depletion stress; with the crop's transpiration and the soil evaporation it is cut proportionally when the
// bucket cannot give it, and what the weeds take is missing for the crop through its depletion stress.

#include "ps/field/field.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ps::field {

namespace {

inline real clamp01(real x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

real tri_response(real t, real tmin, real topt, real tmax) {
    if (t <= tmin || t >= tmax) return 0.0;
    return t <= topt ? (t - tmin) / (topt - tmin) : (tmax - t) / (tmax - topt);
}

// Mineral N taken for new plant tissue (kg/ha); returns what could be taken.
real take_mineral_n(FieldState& s, std::size_t c, real want) {
    const real mineral = s.no3_1[c] + s.nh4[c];
    if (!(want > 0.0) || !(mineral > 0.0)) return 0.0;
    const real take = std::min(want, mineral);
    const real d_no3 = take * s.no3_1[c] / mineral;
    s.no3_1[c] -= d_no3;
    s.nh4[c] -= take - d_no3;
    return take;
}

} // namespace

// ---------------------------------------------------------------------------
// competition: the growth factor the crop sees (Cousens hyperbola per group, groups multiply)
// ---------------------------------------------------------------------------
real Field::weed_competition(std::size_t c) const {
    const std::size_t G = cfg_.weeds.size();
    if (G == 0 || s_.crop[c] == 0) return 1.0;
    const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(s_.crop[c] - 1)];
    if (s_.stage[c] < cr.stage_emergence) return 1.0;
    real f = 1.0;
    for (std::size_t g = 0; g < G; ++g) {
        const real d = s_.w_dens[c * G + g];
        if (!(d > 0.0)) continue;
        const WeedGroupDesc& w = cfg_.weeds[g];
        if (!(w.cousens_i_pct > 0.0)) continue;
        const real lag = std::max(0.0, s_.gdd_em[c] - s_.w_tt[c * G + g]);
        const real ie = w.cousens_i_pct * ps::math::exp(-w.rel_emerge_decay * lag / 100.0);
        const real loss = ie * d / (1.0 + ie * d / w.cousens_a_pct);
        f *= 1.0 - clamp01(loss / 100.0);
    }
    return f;
}

real Field::weed_transpiration(std::size_t c, real taw1, real cc_star, real et0, real& cover_out) const {
    const std::size_t G = cfg_.weeds.size();
    cover_out = 0.0;
    real open = 1.0;   // share of the soil no weed group covers
    real tot = 0.0;
    for (std::size_t g = 0; g < G; ++g) {
        const real bio = s_.w_bio[c * G + g];
        if (!(bio > 0.0)) continue;
        const WeedGroupDesc& w = cfg_.weeds[g];
        const real cover = 1.0 - ps::math::exp(-w.cover_per_g_m2 * bio);
        open *= 1.0 - cover;   // the soil is shaded whether or not the group transpires
        if (!(w.kc_tr > 0.0) || !(et0 > 1.0e-9)) continue;
        const real light = 1.0 - w.shade * clamp01(cc_star);
        const real raw = w.p_depletion * taw1;
        const real ks = taw1 <= 0.0 ? 0.0 : (s_.d_root[c] <= raw ? 1.0 : std::max(0.0, (taw1 - s_.d_root[c]) / ((1.0 - w.p_depletion) * taw1)));
        tot += ks * w.kc_tr * cover * light * et0;
    }
    cover_out = 1.0 - open;
    return tot;
}

// ---------------------------------------------------------------------------
// a share of a group's plants dies and lies on the soil surface as residue
// ---------------------------------------------------------------------------
void Field::weed_to_residue(std::size_t c, std::size_t gi, real fraction) {
    const std::size_t G = cfg_.weeds.size();
    const std::size_t i = c * G + gi;
    if (!(fraction > 0.0) || !(s_.w_dens[i] > 0.0 || s_.w_bio[i] > 0.0)) return;
    const real f = std::min(1.0, fraction);
    const WeedGroupDesc& w = cfg_.weeds[gi];
    s_.res_c[0][c] += s_.w_bio[i] * f * 10.0 * w.c_frac;
    s_.res_n[0][c] += s_.w_n[i] * f;
    const real keep = 1.0 - f;
    if (f >= 1.0) {
        s_.w_bio[i] = s_.w_n[i] = s_.w_dens[i] = s_.w_seed_pl[i] = s_.w_tt[i] = 0.0;
        return;
    }
    s_.w_bio[i] *= keep;
    s_.w_n[i] *= keep;
    s_.w_dens[i] *= keep;
    s_.w_seed_pl[i] *= keep;
}

// ---------------------------------------------------------------------------
// spraying
// ---------------------------------------------------------------------------
void Field::do_spray(std::size_t c, const Spray& sp, real cov) {
    FieldState& s = s_;
    if (sp.product < 1 || static_cast<std::size_t>(sp.product) > cfg_.herbicides.size())
        throw std::invalid_argument("Field::apply: unknown herbicide");
    if (!(sp.dose >= 0.0)) throw std::invalid_argument("Field::apply: spray dose must be >= 0");
    const std::size_t H = cfg_.herbicides.size(), G = cfg_.weeds.size(), M = static_cast<std::size_t>(moa_count_);
    const std::size_t hi = static_cast<std::size_t>(sp.product - 1);
    const HerbicideDesc& hd = cfg_.herbicides[hi];
    const real phi = std::min(1.0, cov);
    const real dose = sp.dose * std::max(1.0, cov);   // overlapping passes add up
    if (!(phi > 0.0) || !(dose > 0.0)) return;
    const std::size_t m = static_cast<std::size_t>(hd.moa_index);

    for (std::size_t g = 0; g < G; ++g) {
        const std::size_t i = c * G + g;
        if (!(s.w_dens[i] > 0.0)) continue;
        const HerbicideKindResponse& kr = hd.kind[cfg_.weeds[g].kind];
        if (!(kr.ed50_rel > 0.0)) continue;
        real stage = 1.0;
        if (s.w_tt[i] > kr.stage_gdd_full)
            stage = kr.stage_gdd_zero > kr.stage_gdd_full ? clamp01((kr.stage_gdd_zero - s.w_tt[i]) / (kr.stage_gdd_zero - kr.stage_gdd_full)) : 0.0;
        const real ks = stage / (1.0 + ps::math::pow(kr.ed50_rel / dose, kr.slope));
        const real kres = stage / (1.0 + ps::math::pow(kr.ed50_rel * hd.resistance_ed50_factor / dose, kr.slope));
        const real r = s.w_res_pl[i * M + m];
        const real survive_s = 1.0 - phi * ks, survive_r = 1.0 - phi * kres;
        const real survive = (1.0 - r) * survive_s + r * survive_r;
        if (survive > 0.0) s.w_res_pl[i * M + m] = r * survive_r / survive;
        weed_to_residue(c, g, 1.0 - survive);
    }

    // the crop: damage outside the safe window, or with an overdose
    if (s.crop[c] != 0 && s.stage[c] >= cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)].stage_emergence) {
        const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)];
        real max_bbch = hd.default_max_bbch, d_in = hd.default_damage_in, d_out = hd.default_damage_out;
        for (const HerbicideCropResponse& cd : hd.crops)
            if (cd.crop == cr.name) {
                max_bbch = cd.max_bbch;
                d_in = cd.damage_in;
                d_out = cd.damage_out;
            }
        const real base = crop_bbch(c) <= max_bbch ? d_in : d_out;
        const real dmg = clamp01(phi * base * ps::math::pow(dose, hd.damage_dose_exponent));
        if (dmg > 0.0) crop_to_residue(c, dmg, false);
    }
    // residual activity in the soil (decays in manage_poststep)
    real& a = s.h_act[c * H + hi];
    if (hd.fungicide_efficacy_max > 0.0) {   // a fungicide: h_act is the protection of the growing crop (F1d)
        if (s.crop[c] == 0) return;
        const real p = hd.fungicide_efficacy_max * phi * dose / (dose + hd.fungicide_dose_half);
        a = std::min(hd.fungicide_efficacy_max, 1.0 - (1.0 - a) * (1.0 - p));
        return;
    }
    a = std::max(a, std::min(2.0, dose) * phi);
}

// ---------------------------------------------------------------------------
// the daily weed step of a cell
// ---------------------------------------------------------------------------
void Field::step_weeds(std::size_t c, const DailyWeather& wx) {
    FieldState& s = s_;
    const std::size_t G = cfg_.weeds.size();
    if (G == 0) return;
    const std::size_t M = static_cast<std::size_t>(moa_count_), H = cfg_.herbicides.size();
    const real tew = tew_of(c);
    const real wsl = tew > 0.0 ? clamp01(1.0 - s.d_evap[c] / tew) : 1.0;
    const real rel = rel_wet(c);
    real ccv = 0.0;
    if (s.crop[c] != 0 && s.stage[c] >= cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)].stage_emergence) ccv = s.cc[c];
    // residual herbicide in the soil kills germinating seedlings
    real germ_pass = 1.0;
    for (std::size_t h = 0; h < H; ++h) germ_pass *= 1.0 - clamp01(s.h_act[c * H + h] * cfg_.herbicides[h].residual_germ_kill);

    for (std::size_t g = 0; g < G; ++g) {
        const WeedGroupDesc& w = cfg_.weeds[g];
        const std::size_t i = c * G + g;

        // seed decay
        s.w_seed_s[i] *= ps::math::exp(-w.decay_surface_per_day);
        s.w_seed_a[i] *= ps::math::exp(-w.decay_upper_per_day);
        s.w_seed_b[i] *= ps::math::exp(-w.decay_lower_per_day);

        // germination and emergence
        {
            const real tr = tri_response(s.soil_t[c], w.germ_t_min_c, w.germ_t_opt_c, w.germ_t_max_c);
            const real wf = clamp01((wsl - w.germ_wet_min) / (w.germ_wet_full - w.germ_wet_min));
            const real rate = w.germ_rate_per_day * tr * wf;
            if (rate > 0.0) {
                const real gs = s.w_seed_s[i] * rate, ga = s.w_seed_a[i] * rate, gb = s.w_seed_b[i] * rate;
                s.w_seed_s[i] -= gs;
                s.w_seed_a[i] -= ga;
                s.w_seed_b[i] -= gb;
                real em = (gs * w.emerge_surface + ga * w.emerge_upper + gb * w.emerge_lower) * germ_pass;
                if (em > 0.0) {
                    const real want = em * w.seedling_g * 10.0 * w.n_conc_pct / 100.0;
                    const real got = take_mineral_n(s, c, want);
                    if (want > 0.0) em *= got / want;
                    const real d_old = s.w_dens[i];
                    for (std::size_t m = 0; m < M; ++m)
                        s.w_res_pl[i * M + m] = (s.w_res_pl[i * M + m] * d_old + s.w_res_sd[i * M + m] * em) / (d_old + em);
                    s.w_tt[i] = s.w_tt[i] * d_old / (d_old + em);
                    s.w_dens[i] = d_old + em;
                    s.w_bio[i] += em * w.seedling_g;
                    s.w_n[i] += got;
                    s.l_c_wg[c] += em * w.seedling_g * 10.0 * w.c_frac;
                }
            }
        }

        if (s.w_dens[i] > 0.0) {
            const bool active = !w.perennial || wx.tmean_c >= w.dormancy_t_c;
            // perennials: shoots die back in the cold, regrow from the clumps
            if (w.perennial && !active) {
                const real f = w.senesce_per_day;
                s.res_c[0][c] += s.w_bio[i] * f * 10.0 * w.c_frac;
                s.res_n[0][c] += s.w_n[i] * f;
                s.w_bio[i] *= 1.0 - f;
                s.w_n[i] *= 1.0 - f;
                s.w_seed_s[i] += s.w_seed_pl[i];
                s.w_seed_pl[i] = 0.0;
                s.w_tt[i] = 0.0;
            }
            // growth: logistic per plant in thermal time, crop shade, a soft water factor
            const real gdd = active ? std::max(0.0, wx.tmean_c - w.t_base_c) : 0.0;
            if (gdd > 0.0) {
                real wp = s.w_bio[i] / s.w_dens[i];
                if (w.perennial && wp < w.regrow_g) {
                    const real add = (w.regrow_g - wp) * s.w_dens[i];
                    const real want = add * 10.0 * w.n_conc_pct / 100.0;
                    const real got = take_mineral_n(s, c, want);
                    const real sc = want > 0.0 ? got / want : 0.0;
                    s.w_bio[i] += add * sc;
                    s.w_n[i] += got;
                    s.l_c_wg[c] += add * sc * 10.0 * w.c_frac;
                    wp = s.w_bio[i] / s.w_dens[i];
                }
                const real shade = 1.0 - w.shade * clamp01(ccv);
                const real mf = 0.3 + 0.7 * clamp01(rel / 0.4);
                const real cap = std::max(0.0, 1.0 - s.w_bio[i] / w.area_max_g_m2);
                real db = w.growth_per_gdd * gdd * wp * std::max(0.0, 1.0 - wp / w.w_max_g) * shade * mf * cap * s.w_dens[i];
                if (db > 0.0) {
                    const real want = db * 10.0 * w.n_conc_pct / 100.0;
                    const real got = take_mineral_n(s, c, want);
                    if (want > 0.0) db *= got / want;
                    s.w_bio[i] += db;
                    s.w_n[i] += got;
                    s.l_c_wg[c] += db * 10.0 * w.c_frac;
                }
                // development and seed set
                const real tt0 = s.w_tt[i];
                s.w_tt[i] += gdd;
                if (s.w_tt[i] > w.gdd_flower && tt0 < w.gdd_ripe && w.gdd_ripe > w.gdd_flower) {
                    const real lo = std::max(tt0, w.gdd_flower), hi = std::min(s.w_tt[i], w.gdd_ripe);
                    if (hi > lo) s.w_seed_pl[i] += s.w_bio[i] * w.seeds_per_g * (hi - lo) / (w.gdd_ripe - w.gdd_flower);
                }
            }
            // self-thinning of a crowded stand: plants die (their small biomass stays and decomposes with the rest)
            if (s.w_bio[i] > 0.8 * w.area_max_g_m2) {
                const real m = w.self_thin_per_day * std::min(1.0, (s.w_bio[i] / w.area_max_g_m2 - 0.8) / 0.2);
                s.w_dens[i] *= 1.0 - m;
                s.w_seed_pl[i] *= 1.0 - m;
            }
            // ripe seed drops to the ground
            if (s.w_tt[i] >= w.gdd_ripe && s.w_seed_pl[i] > 0.0 && w.gdd_ripe > 0.0) {
                const real shed = s.w_seed_pl[i] * w.shed_per_day;
                const real tot = s.w_seed_s[i] + s.w_seed_a[i] + s.w_seed_b[i];
                for (std::size_t m = 0; m < M; ++m)
                    s.w_res_sd[i * M + m] = (s.w_res_sd[i * M + m] * tot + s.w_res_pl[i * M + m] * shed) / (tot + shed);
                s.w_seed_s[i] += shed;
                s.w_seed_pl[i] -= shed;
            }
            // death of annuals (age, frost)
            if (!w.perennial) {
                real kill = 0.0;
                if (s.w_tt[i] > w.gdd_death) kill = 1.0;
                else if (w.cold_kill_onset_c > w.cold_kill_full_c)
                    kill = clamp01((w.cold_kill_onset_c - wx.tmin_c) / (w.cold_kill_onset_c - w.cold_kill_full_c));
                if (kill > 0.0) {
                    if (kill >= 1.0) {
                        const real tot = s.w_seed_s[i] + s.w_seed_a[i] + s.w_seed_b[i];
                        for (std::size_t m = 0; m < M; ++m)
                            s.w_res_sd[i * M + m] = (s.w_res_sd[i * M + m] * tot + s.w_res_pl[i * M + m] * s.w_seed_pl[i]) / (tot + s.w_seed_pl[i]);
                        s.w_seed_s[i] += s.w_seed_pl[i];
                        s.w_seed_pl[i] = 0.0;
                    }
                    weed_to_residue(c, g, kill);
                }
            }
            if (s.w_dens[i] > 0.0 && s.w_dens[i] < 1.0e-9) weed_to_residue(c, g, 1.0);
        }
    }
}

} // namespace ps::field
