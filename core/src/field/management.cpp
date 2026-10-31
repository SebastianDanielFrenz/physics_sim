// core/src/field/management.cpp - the management half of Field (farming F1c): the operation effect record
// (tillage, harvest, straw pickup, sowing), tilth and structure, residue pools and decomposition, establishment of
// a sown crop. Weeds and herbicides are in weeds.cpp. See ps/field/field.h and ps/field/management_desc.h.
//
// Equation sources: soil cover from residue mass 1 - exp(-A m) (Gregory 1982 form; A is a soil parameter);
// evaporation reduced by the mulch cover (FAO-56 ch. 11, AquaCrop mulch factor); first-order residue
// decomposition with a Q10 temperature response, a moisture factor and an assimilation efficiency that builds
// microbial biomass of fixed C:N, so a wide C:N residue immobilises mineral N (Parton et al. 1987 CENTURY form);
// the yield-density law of establishment 1 - exp(-d/d0) (Willey and Heath 1969 form). All rates and thresholds are
// read from the descriptors; what is written below are the equations.

#include "ps/field/field.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ps::field {

namespace {

inline real clamp01(real x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
inline real clampr(real x, real lo, real hi) { return x < lo ? lo : (x > hi ? hi : x); }

real temp_factor(const SoilNutrientDesc& sn, real t) {
    if (t <= 0.0) return 0.0;
    return ps::math::pow(sn.min_q10, (t - 20.0) / 10.0) * clamp01(t / sn.soil_t_ramp_c);
}

// Effective share of an area-weighted fraction: a pass covering `cov` of the cell does `f` on that part; a second
// pass over the same ground compounds (1 - (1 - f)^cov).
real eff(real f, real cov) {
    if (!(f > 0.0) || !(cov > 0.0)) return 0.0;
    if (f >= 1.0) return cov >= 1.0 ? 1.0 : cov;
    return cov <= 1.0 ? f * cov : 1.0 - ps::math::pow(1.0 - f, cov);
}

struct Till {
    real fS = 0, fA = 0, fB = 0, shA = 1, shB = 0, inv = 0, waf = 0, wbf = 0;
    bool deep = false;
};

// One pool with contents in the surface (S), upper (A) and lower (B) layer, moved by a tillage pass. Linear in the
// contents and conservative: everything that leaves a layer arrives in another.
void till3(const Till& t, real& S, real& A, real& B) {
    const real oS = S * t.fS, oA = A * t.fA, oB = B * t.fB;
    const real mixed = oS + oA + oB;
    S -= oS;
    A += mixed * t.shA - oA;
    B += mixed * t.shB - oB;
    if (t.inv > 0.0) {
        const real s0 = S, a0 = A, b0 = B;
        const real dS = s0 * t.inv;
        if (t.deep) {
            const real dA = a0 * t.inv * t.waf, uB = b0 * t.inv * t.wbf;
            S = s0 - dS;
            A = a0 - dA + uB;
            B = b0 + dS + dA - uB;
        } else {
            S = s0 - dS;
            A = a0 + dS;
        }
    }
}

} // namespace

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------
void Field::init_management() {
    if (!cfg_.nutrients_enabled) throw std::invalid_argument("Field: management_enabled needs nutrients_enabled");
    for (const SoilDesc& so : cfg_.soils)
        if (!so.has_management) throw std::invalid_argument("Field: soil " + so.name + " has no management block");
    for (const CropDesc& cr : cfg_.crops)
        if (!cr.has_management) throw std::invalid_argument("Field: crop " + cr.name + " has no management block");
    if (cfg_.weeds.size() > static_cast<std::size_t>(kMaxWeedGroups)) throw std::invalid_argument("Field: too many weed groups");
    if (cfg_.herbicides.size() > static_cast<std::size_t>(kMaxHerbicides)) throw std::invalid_argument("Field: too many herbicides");
    for (const WeedGroupDesc& w : cfg_.weeds) {
        const std::string e = weed_group_validation_error(w);
        if (!e.empty()) throw std::invalid_argument("Field: " + e);
    }
    std::vector<std::string> moas;
    for (HerbicideDesc& h : cfg_.herbicides) {
        const std::string e = herbicide_validation_error(h);
        if (!e.empty()) throw std::invalid_argument("Field: " + e);
        int idx = -1;
        for (std::size_t i = 0; i < moas.size(); ++i)
            if (moas[i] == h.moa) idx = static_cast<int>(i);
        if (idx < 0) {
            if (moas.size() >= static_cast<std::size_t>(kMaxMoa)) throw std::invalid_argument("Field: too many modes of action");
            moas.push_back(h.moa);
            idx = static_cast<int>(moas.size()) - 1;
        }
        h.moa_index = idx;
    }
    moa_count_ = static_cast<int>(moas.size());
    weed_vol_crop_.assign(cfg_.weeds.size(), 0);
    for (std::size_t g = 0; g < cfg_.weeds.size(); ++g) {
        if (cfg_.weeds[g].volunteer_crop.empty()) continue;
        for (std::size_t i = 0; i < cfg_.crops.size(); ++i)
            if (cfg_.crops[i].name == cfg_.weeds[g].volunteer_crop) weed_vol_crop_[g] = static_cast<int>(i) + 1;
        if (weed_vol_crop_[g] == 0) throw std::invalid_argument("Field: weed group " + cfg_.weeds[g].name + " names an unknown volunteer crop");
    }

    const std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    const std::size_t G = cfg_.weeds.size(), H = cfg_.herbicides.size(), M = static_cast<std::size_t>(moa_count_);
    for (auto& v : s_.res_c) v.assign(n, 0.0);
    for (auto& v : s_.res_n) v.assign(n, 0.0);
    for (auto& v : s_.res_p) v.assign(n, 0.0);
    for (auto& v : s_.res_k) v.assign(n, 0.0);
    for (auto* v : {&s_.res_chop, &s_.loose, &s_.damage, &s_.soc, &s_.sow_seeds, &s_.sow_fe, &s_.plants, &s_.l_c_crop, &s_.l_c_wg,
                    &s_.l_c_rem, &s_.l_c_co2})
        v->assign(n, 0.0);
    s_.sow_viab.assign(n, 1.0);
    s_.tilth_e2.assign(n, 1.0);
    s_.seedbed.assign(n, 0.0);
    for (std::size_t c = 0; c < n; ++c) s_.seedbed[c] = soil_of(c).management.seedbed_rest;
    for (auto* v : {&s_.w_seed_s, &s_.w_seed_a, &s_.w_seed_b, &s_.w_dens, &s_.w_bio, &s_.w_tt, &s_.w_n, &s_.w_seed_pl})
        v->assign(n * G, 0.0);
    s_.w_res_pl.assign(n * G * M, 0.0);
    s_.w_res_sd.assign(n * G * M, 0.0);
    s_.h_act.assign(n * H, 0.0);
    for (std::size_t c = 0; c < n; ++c) {
        for (std::size_t g = 0; g < G; ++g) {
            const WeedGroupDesc& w = cfg_.weeds[g];
            const std::size_t i = c * G + g;
            s_.w_seed_s[i] = w.seeds_init_m2 * w.init_surface;
            s_.w_seed_a[i] = w.seeds_init_m2 * w.init_upper;
            s_.w_seed_b[i] = w.seeds_init_m2 * w.init_lower;
            if (w.perennial) {
                s_.w_dens[i] = w.clumps_init_m2;
                s_.w_bio[i] = w.clumps_init_m2 * w.regrow_g;
                s_.w_n[i] = s_.w_bio[i] * 10.0 * w.n_conc_pct / 100.0;
                s_.l_c_wg[c] += s_.w_bio[i] * 10.0 * w.c_frac;
            }
            for (std::size_t m = 0; m < M; ++m) {
                s_.w_res_pl[i * M + m] = w.resistance_init;
                s_.w_res_sd[i * M + m] = w.resistance_init;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// small state queries
// ---------------------------------------------------------------------------
real Field::rel_wet(std::size_t c) const {
    const real taw1 = taw_per_m_of(c) * s_.zr[c];
    return taw1 > 0.0 ? clamp01(1.0 - s_.d_root[c] / taw1) : 1.0;
}

real Field::workability(std::size_t c) const {
    const SoilManagementDesc& m = soil_of(c).management;
    const real w = rel_wet(c);
    const real wet = w <= m.workable_wet_full ? 1.0 : clamp01((m.workable_wet_zero - w) / (m.workable_wet_zero - m.workable_wet_full));
    const real dry = clamp01((w - m.workable_dry_zero) / (m.workable_dry_full - m.workable_dry_zero));
    return wet * dry;
}

real Field::residue_dm_surface(std::size_t c) const {
    return (s_.res_c[0][c] + s_.res_c[1][c]) / soil_of(c).management.res_c_fraction * 1.0e-3;   // t/ha
}

real Field::residue_cover(std::size_t c) const {
    return 1.0 - ps::math::exp(-soil_of(c).management.cover_per_t_ha * residue_dm_surface(c));
}

void Field::release_residue_pk(std::size_t c, int pool, real fraction) {
    if (pool > 1 || !(fraction > 0.0)) return;
    const real p = s_.res_p[static_cast<std::size_t>(pool)][c] * fraction;
    const real k = s_.res_k[static_cast<std::size_t>(pool)][c] * fraction;
    s_.res_p[static_cast<std::size_t>(pool)][c] -= p;
    s_.res_k[static_cast<std::size_t>(pool)][c] -= k;
    s_.p_av[c] += p;
    s_.k_av[c] += k;
}

// ---------------------------------------------------------------------------
// the crop leaves: into the residue pools (kill, damage, sowing over) or out of the field (harvest)
// ---------------------------------------------------------------------------
void Field::reset_crop_state(std::size_t c) {
    FieldState& s = s_;
    s.crop[c] = 0;
    s.stage[c] = 0;
    s.gdd_sow[c] = s.gdd_em[c] = s.vern_days[c] = s.cc[c] = s.cc_peak[c] = s.t_sen[c] = 0.0;
    s.biomass[c] = s.grain[c] = s.hi_gdd[c] = s.pol_sum[c] = s.pol_n[c] = s.sto_sum[c] = s.sto_n[c] = s.days_ripe[c] = 0.0;
    s.stand[c] = 1.0;
    s.n_crop[c] = s.n_root[c] = s.p_crop[c] = s.k_crop[c] = s.lodge[c] = 0.0;
    s.sow_seeds[c] = 0.0;
    s.sow_fe[c] = 0.0;
    s.sow_viab[c] = 1.0;
    s.plants[c] = 0.0;
    set_root_depth(c, soil_of(c).fallow_root_depth_m);
}

void Field::clear_crop(std::size_t c) { reset_crop_state(c); }

// Destroys `fraction` of the standing crop; its aboveground matter lies on the surface (`to_pool` 0 loose, 1 standing),
// its roots go into the buried pools. Everything the crop held stays in the field.
void Field::crop_to_residue(std::size_t c, real fraction, bool standing) {
    FieldState& s = s_;
    if (s.crop[c] == 0 || !(fraction > 0.0)) return;
    if (fraction > 1.0) fraction = 1.0;
    const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)];
    const SoilManagementDesc& m = soil_of(c).management;
    if (cfg_.pathogens_enabled) s.res_crop[c] = s.crop[c];   // its residue now carries the family's inoculum
    const std::size_t pool = standing ? 1 : 0;
    const real dm_above = fraction * s.biomass[c];                  // g/m2
    const real c_above = dm_above * 10.0 * m.res_c_fraction;        // kg C/ha
    const real c_root = fraction * s.biomass[c] * cr.management.root_dm_fraction * 10.0 * m.res_c_fraction;
    const real n_above = fraction * s.n_crop[c], n_root = fraction * s.n_root[c];
    s.res_c[pool][c] += c_above;
    s.res_n[pool][c] += n_above;
    s.res_p[pool][c] += fraction * s.p_crop[c];
    s.res_k[pool][c] += fraction * s.k_crop[c];
    const real us = cr.management.root_upper_share;
    s.res_c[2][c] += c_root * us;
    s.res_c[3][c] += c_root * (1.0 - us);
    s.res_n[2][c] += n_root * us;
    s.res_n[3][c] += n_root * (1.0 - us);
    s.l_c_crop[c] += c_above + c_root;
    if (fraction >= 1.0) {
        s.n_crop[c] = s.n_root[c] = s.p_crop[c] = s.k_crop[c] = 0.0;
        clear_crop(c);
        return;
    }
    const real keep = 1.0 - fraction;
    for (auto* v : {&s.cc, &s.stand, &s.biomass, &s.grain, &s.n_crop, &s.n_root, &s.p_crop, &s.k_crop}) (*v)[c] *= keep;
    s.cc_peak[c] *= keep;
    s.plants[c] *= keep;
}

// ---------------------------------------------------------------------------
// harvest
// ---------------------------------------------------------------------------
void Field::do_harvest_mgmt(std::size_t c, const HarvestEffect& h, real cov, HarvestResult* out) {
    FieldState& s = s_;
    if (s.crop[c] == 0) return;
    const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)];
    const CropNutrientDesc& cn = cr.nutrients;
    const SoilManagementDesc& m = soil_of(c).management;
    const real phi = std::min(1.0, cov);
    if (!(phi > 0.0)) return;
    if (cfg_.pathogens_enabled) s.res_crop[c] = s.crop[c];
    const real g_all = phi * s.grain[c];                             // g/m2 on the plants
    const real lost = h.grain_loss * g_all;                          // dropped on the ground by the machine
    const real g = g_all - lost;                                     // taken away
    const real b = phi * s.biomass[c];
    const real ng = std::max(0.0, b - g_all);
    const real fh = cr.straw_harvest_fraction;
    const real straw_h = fh * ng, stub = ng - straw_h;
    const real n_tot = phi * s.n_crop[c], n_grain_all = cn.n_grain_share * n_tot, n_straw = n_tot - n_grain_all;
    const real p_tot = phi * s.p_crop[c], p_grain_all = cn.p2o5_grain_share * p_tot, p_straw = p_tot - p_grain_all;
    const real k_tot = phi * s.k_crop[c], k_grain_all = cn.k2o_grain_share * k_tot, k_straw = k_tot - k_grain_all;
    const real n_grain = (1.0 - h.grain_loss) * n_grain_all, p_grain = (1.0 - h.grain_loss) * p_grain_all, k_grain = (1.0 - h.grain_loss) * k_grain_all;
    const real share_h = ng > 0.0 ? straw_h / ng : 0.0;              // harvestable share of the straw
    const real carried = h.straw_carried;
    if (out) {
        out->crop_id = s.crop[c];
        out->grain_dm_t_ha = g * 0.01;
        out->straw_dm_t_ha = straw_h * 0.01;
        out->stubble_dm_t_ha = stub * 0.01;
        out->bbch = crop_bbch(c);
        out->n_grain_kg_ha = n_grain;
        out->n_straw_kg_ha = n_straw;
        out->n_root_kg_ha = phi * s.n_root[c];
        out->p2o5_grain_kg_ha = p_grain;
        out->p2o5_straw_kg_ha = p_straw;
        out->k2o_grain_kg_ha = k_grain;
        out->k2o_straw_kg_ha = k_straw;
        out->lodging_loss = s.lodge[c];
    }
    // removal: the grain and the straw carried off with it
    s.l_n_rem[c] += n_grain + carried * share_h * n_straw;
    s.l_p_rem[c] += p_grain + carried * share_h * p_straw;
    s.l_k_rem[c] += k_grain + carried * share_h * k_straw;
    // what stays: harvestable straw not carried off (loose, chopped or in a swath), stubble and chaff (standing), roots
    const real left_h = (1.0 - carried) * share_h;
    const real c_loose = (1.0 - carried) * straw_h * 10.0 * m.res_c_fraction;
    const real c_stub = stub * 10.0 * m.res_c_fraction;
    const real c_root = phi * s.biomass[c] * cr.management.root_dm_fraction * 10.0 * m.res_c_fraction;
    const real n_root = phi * s.n_root[c];
    const real chop_old_w = s.res_c[0][c];
    const real c_lost = lost * 10.0 * m.res_c_fraction;
    s.res_c[0][c] += c_loose + c_lost;
    s.res_n[0][c] += left_h * n_straw + h.grain_loss * n_grain_all;
    s.res_p[0][c] += left_h * p_straw + h.grain_loss * p_grain_all;
    s.res_k[0][c] += left_h * k_straw + h.grain_loss * k_grain_all;
    if (s.res_c[0][c] > 0.0) s.res_chop[c] = (s.res_chop[c] * chop_old_w + h.straw_chopped * c_loose) / s.res_c[0][c];
    if (lost > 0.0) {   // the dropped grain seeds volunteers
        const std::size_t G = cfg_.weeds.size();
        for (std::size_t gi = 0; gi < G; ++gi)
            if (weed_vol_crop_[gi] == s.crop[c]) s.w_seed_s[c * G + gi] += lost / cr.management.seed_mass_g;
    }
    s.res_c[1][c] += c_stub;
    s.res_n[1][c] += (1.0 - share_h) * n_straw;
    s.res_p[1][c] += (1.0 - share_h) * p_straw;
    s.res_k[1][c] += (1.0 - share_h) * k_straw;
    const real us = cr.management.root_upper_share;
    s.res_c[2][c] += c_root * us;
    s.res_c[3][c] += c_root * (1.0 - us);
    s.res_n[2][c] += n_root * us;
    s.res_n[3][c] += n_root * (1.0 - us);
    s.l_c_crop[c] += c_loose + c_lost + c_stub + c_root;
    if (phi >= 1.0) {
        s.n_crop[c] = s.n_root[c] = s.p_crop[c] = s.k_crop[c] = s.lodge[c] = 0.0;
        clear_crop(c);
    } else {
        const real keep = 1.0 - phi;
        for (auto* v : {&s.cc, &s.stand, &s.biomass, &s.grain, &s.n_crop, &s.n_root, &s.p_crop, &s.k_crop}) (*v)[c] *= keep;
        s.cc_peak[c] *= keep;
        s.plants[c] *= keep;
    }
}

void Field::do_straw_pickup(std::size_t c, real f) {
    if (!(f > 0.0)) return;
    FieldState& s = s_;
    const real cc = s.res_c[0][c] * f;
    s.l_c_rem[c] += cc;
    s.l_n_rem[c] += s.res_n[0][c] * f;
    s.l_p_rem[c] += s.res_p[0][c] * f;
    s.l_k_rem[c] += s.res_k[0][c] * f;
    s.res_c[0][c] -= cc;
    s.res_n[0][c] -= s.res_n[0][c] * f;
    s.res_p[0][c] -= s.res_p[0][c] * f;
    s.res_k[0][c] -= s.res_k[0][c] * f;
    if (s.res_c[0][c] < 1.0e-12) s.res_chop[c] = 0.0;
}

// ---------------------------------------------------------------------------
// tillage
// ---------------------------------------------------------------------------
void Field::do_tillage(std::size_t c, const TillageEffect& t, real cov) {
    FieldState& s = s_;
    const SoilDesc& so = soil_of(c);
    const SoilManagementDesc& m = so.management;
    const std::size_t G = cfg_.weeds.size();
    // 1. destruction of the standing crop and the emerged weeds (before the soil is moved, so the pass buries them)
    if (t.crop_kill > 0.0) crop_to_residue(c, eff(t.crop_kill, cov), false);
    for (std::size_t g = 0; g < G; ++g) {
        const real k = eff(t.weed_kill[cfg_.weeds[g].kind], cov);
        if (k > 0.0) weed_to_residue(c, g, k);
    }
    if (!(t.depth_m > 0.0)) return;

    const real tA = m.layer_a_m, tB = m.layer_b_m;
    const real wa = std::min(t.depth_m, tA), wb = clampr(t.depth_m - tA, 0.0, tB);
    const real wk = workability(c);
    Till tl;
    tl.fS = eff(t.mix_surface, cov);
    tl.fA = eff(t.mix_upper, cov) * wa / tA;
    tl.fB = eff(t.mix_lower, cov) * wb / tB;
    tl.shA = wa / (wa + wb);
    tl.shB = wb / (wa + wb);
    tl.inv = eff(t.inversion, cov);
    tl.waf = wa / tA;
    tl.wbf = wb / tB;
    tl.deep = wb > 0.0;
    // share of the surface content that leaves the surface
    real sM = 1.0, aM = 0.0, bM = 0.0;
    till3(tl, sM, aM, bM);
    const real mS = 1.0 - sM;

    // 2. residue chop on the loose straw, then the movement of every pool
    const real chop = eff(t.residue_chop, cov);
    if (chop > 0.0) s.res_chop[c] += (1.0 - s.res_chop[c]) * chop;
    for (auto* arr : {&s.res_c, &s.res_n}) {
        real inA = 0.0, inB = 0.0;
        for (int p = 0; p < 2; ++p) {
            real S = (*arr)[static_cast<std::size_t>(p)][c], A = 0.0, B = 0.0;
            till3(tl, S, A, B);
            (*arr)[static_cast<std::size_t>(p)][c] = S;
            inA += A;
            inB += B;
        }
        real S = 0.0, A = (*arr)[2][c], B = (*arr)[3][c];
        till3(tl, S, A, B);
        (*arr)[2][c] = A + inA;
        (*arr)[3][c] = B + inB;
    }
    release_residue_pk(c, 0, mS);
    release_residue_pk(c, 1, mS);
    for (std::size_t g = 0; g < G; ++g) {
        const std::size_t i = c * G + g;
        till3(tl, s.w_seed_s[i], s.w_seed_a[i], s.w_seed_b[i]);
    }
    // lime and the ammoniacal N lying on the surface go into the soil
    const real lm = s.lime_surf[c] * mS;
    s.lime_surf[c] -= lm;
    s.lime_inc[c] += lm;
    if (s.tan_surf[c] > 0.0 && mS > 0.0) {
        const real mv = s.tan_surf[c] * mS;
        s.nh4[c] += mv;
        const real keep = 1.0 - mS;
        s.tan_surf[c] -= mv;
        s.kv_w[c] *= keep;
        s.ki_w[c] *= keep;
        s.kir_w[c] *= keep;
        if (s.tan_surf[c] < 1.0e-9) {
            s.nh4[c] += s.tan_surf[c];
            s.tan_surf[c] = 0.0;
            s.kv_w[c] = s.ki_w[c] = s.kir_w[c] = 0.0;
            s.inc_h[c] = -1.0;
        }
    }

    // 3. structure: loosening, damage, seedbed, the water lost when the soil is opened
    const real dfrac = std::min(1.0, t.depth_m / (tA + tB));
    const real lf = eff(t.loosening, cov) * dfrac;
    s.loose[c] += (1.0 - s.loose[c]) * lf;
    s.damage[c] *= (1.0 - lf);
    s.damage[c] = std::min(1.0, s.damage[c] + m.damage_gain * (1.0 - wk) * dfrac * std::min(1.0, cov));
    if (t.seedbed_fineness > 0.0) {
        const real fresh = t.seedbed_fineness * wk + std::min(t.seedbed_fineness, m.seedbed_wet_floor) * (1.0 - wk);
        s.seedbed[c] += std::min(1.0, cov) * (fresh - s.seedbed[c]);
    }
    const real taw1 = taw_per_m_of(c) * s.zr[c];
    const real pulse = std::min(m.tillage_dry_out_mm_per_m * t.depth_m * wk * std::min(1.0, cov), std::max(0.0, taw1 - s.d_root[c]));
    if (pulse > 0.0) {
        s.d_root[c] += pulse;
        s.d_evap[c] = std::min(tew_of(c), s.d_evap[c] + pulse);
        s.cum_evap[c] += pulse;
    }
}

// ---------------------------------------------------------------------------
// sowing
// ---------------------------------------------------------------------------
void Field::do_seeding(std::size_t c, const SeedPlacement& sp, real cov) {
    FieldState& s = s_;
    if (sp.crop == 0 || !(sp.seeds_per_m2 > 0.0) || !(cov > 0.0)) return;
    if (static_cast<std::size_t>(sp.crop) > cfg_.crops.size()) throw std::invalid_argument("Field::apply: unknown crop");
    const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(sp.crop - 1)];
    const CropManagementDesc& cm = cr.management;
    if (s.crop[c] != 0 && s.stage[c] >= cr.stage_emergence && s.crop[c] == sp.crop) return;   // seed into a standing crop is lost
    if (s.crop[c] != 0 && s.stage[c] >= cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)].stage_emergence) return;
    const real rate = sp.seeds_per_m2 * cov;
    real fd;
    if (sp.depth_m <= 0.0) fd = cm.depth_surface_factor;
    else if (sp.depth_m < cm.depth_opt_min_m) fd = cm.depth_surface_factor + (1.0 - cm.depth_surface_factor) * sp.depth_m / cm.depth_opt_min_m;
    else if (sp.depth_m <= cm.depth_opt_max_m) fd = 1.0;
    else fd = clamp01((cm.depth_zero_m - sp.depth_m) / (cm.depth_zero_m - cm.depth_opt_max_m));
    const real fsb = cm.seedbed_floor + (1.0 - cm.seedbed_floor) * s.seedbed[c];
    const real fres = 1.0 - cm.residue_emergence_penalty * residue_cover(c) * (1.0 - sp.residue_tolerance);
    const real fe = cm.seed_viability * fd * fsb * fres * sp.quality;
    if (s.crop[c] == sp.crop && s.sow_seeds[c] > 0.0) {   // a second pass adds seed
        const real tot = s.sow_seeds[c] + rate;
        s.sow_fe[c] = (s.sow_seeds[c] * s.sow_fe[c] + rate * fe) / tot;
        s.sow_seeds[c] = tot;
        return;
    }
    if (s.crop[c] != 0) crop_to_residue(c, 1.0, false);   // sown over: the old stand is ploughed in
    s.crop[c] = static_cast<std::uint8_t>(sp.crop);
    s.stage[c] = 0;
    s.sow_seeds[c] = rate;
    s.sow_fe[c] = fe;
    s.sow_viab[c] = 1.0;
    s.plants[c] = 0.0;
    s.cc[c] = 0.0;
    s.stand[c] = 1.0;
    set_root_depth(c, cr.zr_min_m);
}

// At the day the sown crop emerges: plants per m2 from seeds, emergence potential and the viability left.
void Field::establish(std::size_t c, const CropDesc& cr) {
    FieldState& s = s_;
    if (s.sow_seeds[c] < 0.0) return;   // legacy sow(): the stand is set by sow()
    const CropManagementDesc& cm = cr.management;
    const real plants = s.sow_seeds[c] * s.sow_fe[c] * s.sow_viab[c];
    const real ref = cr.plant_density_per_ha * 1.0e-4;
    const real ds = plants / ref;
    s.plants[c] = plants;
    s.cc[c] = cr.cc0(ds);
    const real d0 = cm.compensation_density_m2;
    s.stand[c] = std::min(1.0, (1.0 - ps::math::exp(-plants / d0)) / (1.0 - ps::math::exp(-ref / d0)));
    s.sow_seeds[c] = 0.0;
}

// ---------------------------------------------------------------------------
// the record
// ---------------------------------------------------------------------------
void Field::apply_management(std::size_t c, const OperationEffect& e, int phase, HarvestResult* harvest_out) {
    const real cov = e.coverage;
    if (phase == 1) {
        if (e.harvest.harvest) {
            HarvestResult r;
            do_harvest_mgmt(c, e.harvest, cov, &r);
            if (harvest_out && r.crop_id != 0) *harvest_out = r;
        }
        if (e.straw_pickup > 0.0) do_straw_pickup(c, eff(e.straw_pickup, cov));
        if (e.tillage.active()) do_tillage(c, e.tillage, cov);
    } else {
        if (e.spray.product != 0) do_spray(c, e.spray, cov);
        if (e.seed.crop != 0) do_seeding(c, e.seed, cov);
    }
}

// ---------------------------------------------------------------------------
// daily: factors before the water balance
// ---------------------------------------------------------------------------
void Field::manage_prestep(std::size_t c, const DailyWeather&) {
    const SoilManagementDesc& m = soil_of(c).management;
    s_.residue_evap_factor[c] = 1.0 - m.mulch_evap_max_reduction * residue_cover(c);
    s_.tilth_e2[c] = 1.0 - m.loose_stage2_evap_reduction * s_.loose[c];
    s_.weed_f[c] = weed_competition(c);
}

// ---------------------------------------------------------------------------
// daily: after the water balance and the nutrients
// ---------------------------------------------------------------------------
void Field::manage_poststep(std::size_t c, const DailyWeather& w, const DayFlows& fl) {
    FieldState& s = s_;
    const SoilDesc& so = soil_of(c);
    const SoilManagementDesc& m = so.management;
    const SoilNutrientDesc& sn = so.nutrients;

    // tilth and structure settle with rain and time
    const real settle = ps::math::exp(-(m.settle_per_mm_rain * w.rain_mm + m.settle_per_day));
    s.loose[c] *= settle;
    s.damage[c] *= ps::math::exp(-m.damage_recovery_per_day);
    s.seedbed[c] += (m.seedbed_rest - s.seedbed[c]) * (1.0 - ps::math::exp(-(m.seedbed_settle_per_mm * w.rain_mm + m.seedbed_settle_per_day)));

    // residue decomposition (temperature, moisture), immobilising or releasing mineral N
    const real ft = temp_factor(sn, s.soil_t[c]);
    const real tew = tew_of(c);
    const real ws = tew > 0.0 ? clamp01(1.0 - s.d_evap[c] / tew) : 1.0;
    const real fm_surf = m.res_moisture_floor + (1.0 - m.res_moisture_floor) * ws;
    const real fm_bur = m.res_moisture_floor + (1.0 - m.res_moisture_floor) * rel_wet(c);
    for (int p = 0; p < kResPools; ++p) {
        const std::size_t pi = static_cast<std::size_t>(p);
        const real C = s.res_c[pi][c];
        if (C <= 1.0e-12) continue;
        const real layer = p == 0 ? m.res_surface_factor * (1.0 + m.res_chop_gain * s.res_chop[c]) : (p == 1 ? m.res_stubble_factor : 1.0);
        const real k = m.res_k20_per_day * ft * (p < 2 ? fm_surf : fm_bur) * layer;
        real dC = C * (1.0 - ps::math::exp(-k));
        real dN = dC * s.res_n[pi][c] / C;
        real n_need = m.res_assim_eff * dC / m.res_cn_bio;
        if (dN < n_need) {   // wide C:N: microbes take mineral N, as far as there is any
            const real mineral = s.no3_1[c] + s.nh4[c];
            const real avail = std::max(0.0, mineral - m.res_n_floor_kg_ha);
            const real want = n_need - dN;
            if (want > avail) {
                const real sc = n_need > 0.0 ? (dN + avail) / n_need : 0.0;
                dC *= sc;
                dN *= sc;
                n_need *= sc;
            }
            const real take = n_need - dN;
            if (take > 0.0 && mineral > 0.0) {
                const real d_no3 = take * s.no3_1[c] / mineral;
                s.no3_1[c] -= d_no3;
                s.nh4[c] -= take - d_no3;
            }
        } else {
            s.nh4[c] += dN - n_need;
        }
        s.res_c[pi][c] -= dC;
        s.res_n[pi][c] -= dN;
        if (cfg_.soc_enabled) {   // only the humification coefficient of the decomposed carbon reaches the humus; the rest of what was assimilated is respired as the biomass turns over
            const SoilOrganicDesc& o = so.organic;
            const real dh = o.humification * dC;
            const real nh = dh / o.humus_cn;
            s.l_c_co2[c] += dC - dh;
            s.soc[c] += dh;
            s.hum_c[c] += dh;
            s.org_slow[c] += nh;
            s.org_fresh[c] += n_need - nh;
        } else {
            s.l_c_co2[c] += dC * (1.0 - m.res_assim_eff);
            s.soc[c] += dC * m.res_assim_eff;
            s.org_fresh[c] += n_need;
        }
        if (p < 2) release_residue_pk(c, p, dC / C);
        if (p == 0 && s.res_c[0][c] < 1.0e-12) s.res_chop[c] = 0.0;
    }

    // herbicide residues fade
    const std::size_t H = cfg_.herbicides.size();
    for (std::size_t h = 0; h < H; ++h) {
        real& a = s.h_act[c * H + h];
        const HerbicideDesc& hd = cfg_.herbicides[h];
        const real hl = hd.fungicide_efficacy_max > 0.0 ? hd.fungicide_half_life_days : hd.residual_half_life_days;
        a = (hl > 0.0 && a > 1.0e-9) ? a * ps::math::exp(-0.6931471805599453 / hl) : 0.0;
    }

    // shed grain of the crop seeds the volunteers
    if (fl.shed_g_m2 > 0.0) {
        for (std::size_t g = 0; g < cfg_.weeds.size(); ++g) {
            if (weed_vol_crop_[g] != s.crop[c] || s.crop[c] == 0) continue;
            const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(s.crop[c] - 1)];
            s.w_seed_s[c * cfg_.weeds.size() + g] += fl.shed_g_m2 / cr.management.seed_mass_g;
        }
    }

    step_weeds(c, w);
}

// ---------------------------------------------------------------------------
// queries
// ---------------------------------------------------------------------------
ManagementView Field::management_view(int cell) const {
    if (!cfg_.management_enabled) throw std::logic_error("Field::management_view: the field runs without management");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::management_view: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell);
    const SoilManagementDesc& m = soil_of(c).management;
    ManagementView v;
    const real k = 1.0e-3 / m.res_c_fraction;
    v.straw_t_ha = s_.res_c[0][c] * k;
    v.stubble_t_ha = s_.res_c[1][c] * k;
    v.buried_upper_t_ha = s_.res_c[2][c] * k;
    v.buried_lower_t_ha = s_.res_c[3][c] * k;
    for (int p = 0; p < kResPools; ++p) {
        v.residue_c_kg_ha += s_.res_c[static_cast<std::size_t>(p)][c];
        v.residue_n_kg_ha += s_.res_n[static_cast<std::size_t>(p)][c];
    }
    v.cover = residue_cover(c);
    v.loosening = s_.loose[c];
    v.damage = s_.damage[c];
    v.seedbed = s_.seedbed[c];
    v.soc_gain_c_kg_ha = s_.soc[c];
    v.plants_m2 = s_.plants[c];
    v.sown_seeds_m2 = s_.sow_seeds[c] > 0.0 ? s_.sow_seeds[c] : 0.0;
    v.weed_factor = s_.weed_f[c];
    const std::size_t G = cfg_.weeds.size(), M = static_cast<std::size_t>(moa_count_);
    for (std::size_t g = 0; g < G; ++g) {
        const std::size_t i = c * G + g;
        WeedGroupView w;
        w.seeds_surface = s_.w_seed_s[i];
        w.seeds_upper = s_.w_seed_a[i];
        w.seeds_lower = s_.w_seed_b[i];
        w.seeds_total = w.seeds_surface + w.seeds_upper + w.seeds_lower;
        w.plants_m2 = s_.w_dens[i];
        w.biomass_g_m2 = s_.w_bio[i];
        w.seed_on_plants_m2 = s_.w_seed_pl[i];
        w.tt_gdd = s_.w_tt[i];
        for (std::size_t mi = 0; mi < M; ++mi) {
            w.resistant_plants[mi] = s_.w_res_pl[i * M + mi];
            w.resistant_seeds[mi] = s_.w_res_sd[i * M + mi];
        }
        v.weeds.push_back(w);
    }
    for (std::size_t h = 0; h < cfg_.herbicides.size(); ++h) v.herbicide_residual.push_back(s_.h_act[c * cfg_.herbicides.size() + h]);
    return v;
}

CarbonLedger Field::carbon_ledger(int cell) const {
    if (!cfg_.management_enabled) throw std::logic_error("Field::carbon_ledger: the field runs without management");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::carbon_ledger: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell);
    CarbonLedger l;
    l.c_from_crop = s_.l_c_crop[c];
    l.c_weed_growth = s_.l_c_wg[c];
    l.c_removed = s_.l_c_rem[c];
    l.c_co2 = s_.l_c_co2[c];
    l.c_humus = s_.soc[c];
    if (cfg_.soc_enabled) {
        l.c_humus_decay = s_.l_c_hdec[c];
        l.c_amendment = s_.l_c_amend[c];
        l.c_humus_initial = s_.hum_c0[c];
    }
    return l;
}

real Field::c_balance_residual(int cell) const {
    const CarbonLedger l = carbon_ledger(cell);
    const std::size_t c = static_cast<std::size_t>(cell);
    real stored = 0.0;
    for (int p = 0; p < kResPools; ++p) stored += s_.res_c[static_cast<std::size_t>(p)][c];
    const std::size_t G = cfg_.weeds.size();
    for (std::size_t g = 0; g < G; ++g) stored += s_.w_bio[c * G + g] * 10.0 * cfg_.weeds[g].c_frac;
    if (cfg_.soc_enabled)   // the humus pool is a stock: check its own state, not the cumulative gain
        return l.c_from_crop + l.c_weed_growth + l.c_amendment - l.c_removed - l.c_co2 - l.c_humus_decay - (s_.hum_c[c] - l.c_humus_initial) - stored;
    return l.c_from_crop + l.c_weed_growth - l.c_removed - l.c_co2 - l.c_humus - stored;
}

} // namespace ps::field
