// core/src/field/soc.cpp - soil organic carbon of the plough layer (farming F1d part 5).
//
// Model (parameters in the soil's "organic_matter" block, equations here):
//   Humus is the slow organic N pool of the nutrient model; its carbon is  C_h = N_slow x humus_cn  at construction.
//   Daily the humus decays like the mineralisation of the nutrient model:
//        f = 1 - exp(-rate_per_day_20c x ft x fm x f_av)        (temperature, moisture, pH factors of the nutrient model)
//        N_slow -= f N_slow  (to ammonium),     C_h -= f C_h  (to CO2)
//   so the C:N of the pool does not drift and the nitrogen it releases falls with its carbon.
//   Residue carbon dC decomposed in a day builds humus  dH = humification x dC  (with dH / humus_cn of nitrogen taken from
//   the N the microbes immobilise); what the microbes assimilated beyond that is respired as the biomass turns over.
//   Organic material applied to the field joins the humus with the carbon its slow organic N carries at the humus C:N.
//   This is the structure of the two-pool ICBM model (Andren and Kaetterer 1997, Ecol. Appl. 7:1226: young pool decays
//   fast, a fraction h of the decomposed carbon passes to the old pool, which decays at k2) with the young pool being the
//   residue pools of the management layer.
//   Water. The plant-available water of the soil follows the carbon content. With  pct = 100 C_h / (bulk density x depth)
//   the volumetric water content at field capacity changes by  awc_per_soc_pct x (pct - pct_initial)  in the layer of
//   depth layer_depth_m (Minasny and McBratney 2018, Eur. J. Soil Sci. 69:39: plant-available water rises by 1.16 mm per
//   100 mm of soil per 1 % SOC); the field applies the profile mean of it (x layer_depth_m / depth_m) to theta_fc, so the
//   soil file's water values are the ones at the initial carbon content. When the capacity changes the depletions change
//   by the same amount, so the stored water stays and only what the smaller capacity can no longer hold leaves as drainage.

#include "ps/field/field.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ps::field {

namespace {
inline real soil_mass_kg_ha(const SoilOrganicDesc& o) { return o.bulk_density_t_m3 * 1000.0 * o.layer_depth_m * 10000.0; }
}

void Field::init_soc() {
    const std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    for (const SoilDesc& so : cfg_.soils) {
        if (!so.has_organic) throw std::invalid_argument("Field: soc_enabled needs an organic_matter block in soil " + so.name);
        if (!so.has_management) throw std::invalid_argument("Field: soc_enabled needs a management block in soil " + so.name);
        const SoilOrganicDesc& o = so.organic;
        const SoilManagementDesc& m = so.management;
        if (o.humification > m.res_assim_eff)
            throw std::invalid_argument("Field: soil " + so.name + ": humification exceeds res_assim_eff (humus cannot exceed what the microbes assimilate)");
        if (o.humification / o.humus_cn > m.res_assim_eff / m.res_cn_bio)
            throw std::invalid_argument("Field: soil " + so.name + ": the humus nitrogen of the humification exceeds the nitrogen the microbes immobilise");
        if (o.layer_depth_m > so.depth_m) throw std::invalid_argument("Field: soil " + so.name + ": organic_matter layer_depth_m exceeds depth_m");
    }
    s_.hum_c.assign(n, 0.0);
    s_.hum_c0.assign(n, 0.0);
    s_.soc_dfc.assign(n, 0.0);
    s_.l_c_hdec.assign(n, 0.0);
    s_.l_c_amend.assign(n, 0.0);
    for (std::size_t c = 0; c < n; ++c) {
        s_.hum_c[c] = s_.org_slow[c] * soil_of(c).organic.humus_cn;
        s_.hum_c0[c] = s_.hum_c[c];
    }
}

void Field::step_soc(std::size_t c) {
    FieldState& s = s_;
    const SoilDesc& so = soil_of(c);
    const SoilOrganicDesc& o = so.organic;
    const real mass = soil_mass_kg_ha(o);
    const real pct = 100.0 * s.hum_c[c] / mass, pct0 = 100.0 * s.hum_c0[c] / mass;
    real target = o.awc_per_soc_pct * (pct - pct0) * (o.layer_depth_m / so.depth_m);
    const real floor_dfc = -0.5 * (so.theta_fc - so.theta_wp);   // the capacity never shrinks to less than half of the file's
    target = std::max(target, floor_dfc);
    const real delta = target - s.soc_dfc[c];
    if (delta == 0.0) return;
    // the stored water stays: capacity and depletion move together
    real nr = s.d_root[c] + 1000.0 * delta * s.zr[c];
    real ns = s.d_sub[c] + 1000.0 * delta * (so.depth_m - s.zr[c]);
    if (nr < 0.0) {   // the root zone holds more than its smaller capacity: the rest sinks
        ns += nr;
        nr = 0.0;
    }
    if (ns < 0.0) {   // and what the whole profile cannot hold leaves as drainage
        s.cum_drain[c] += -ns;
        ns = 0.0;
    }
    s.d_root[c] = nr;
    s.d_sub[c] = ns;
    s.soc_dfc[c] = target;
    s.d_evap[c] = std::min(tew_of(c), std::max(0.0, s.d_evap[c] + 1000.0 * delta * so.evaporation_depth_m));
}

SocView Field::soc_view(int cell) const {
    if (!cfg_.soc_enabled) throw std::logic_error("Field::soc_view: the field runs without soc_enabled");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::soc_view: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell);
    const SoilOrganicDesc& o = soil_of(c).organic;
    const real mass = soil_mass_kg_ha(o);
    SocView v;
    v.hum_c_kg_ha = s_.hum_c[c];
    v.soc_pct = 100.0 * s_.hum_c[c] / mass;
    v.soc_pct_initial = 100.0 * s_.hum_c0[c] / mass;
    v.cn = s_.org_slow[c] > 0.0 ? s_.hum_c[c] / s_.org_slow[c] : 0.0;
    v.dfc = s_.soc_dfc[c];
    v.taw_total_mm = total_taw_of(c);
    return v;
}

} // namespace ps::field
