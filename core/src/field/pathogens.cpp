// core/src/field/pathogens.cpp - soil-borne pathogen pressure per crop family and cell (farming F1d).
//
// Model (all rates are in the crop descriptors' "pathogen" block, what is written here are the equations):
//   P  inoculum of a family in a cell, 0..1 (relative to the level that gives the full loss_max).
//   S  suppression, 0..1 (the antagonist population that builds under a high inoculum: take-all decline).
// Daily, per family f of the cell:
//   host growing (crop of family f, emerged, before maturity), x = gdd / season_gdd:
//        P += (1 - P) (1 - exp(-build_per_season x (1 - protection)))
//        S += (1 - S) (1 - exp(-suppression_gain P x))
//   no host growing:  P *= exp(-ln2 / decay_half_life_days)         (break years, fallow)
//   always:           S *= exp(-ln2 / suppression_half_life_days)
//   surface residue of a crop of family f (loose straw + stubble, carbon c_s):
//        P += (1 - P) residue_source_per_day min(1, c_s / residue_ref_c)
// When a crop emerges its untreated loss is fixed from the pressure it meets:
//        loss0 = loss_max P (1 - suppression_max S)
// and every day it grows its growth factor (stress_rotation, inside the biomass law) is
//        1 - loss0 (1 - protection)
// where protection is what the fungicide sprays since have left (herbicide-like products with a "fungicide" block;
// their h_act is the protection and decays with the product's half-life; protection never exceeds efficacy_max < 1).
//
// The form is a two-state (inoculum, antagonist) version of the take-all epidemiology: inoculum builds on the roots of
// each susceptible crop, falls when the host is absent, and a suppressive microflora follows a high inoculum with a lag
// and outlasts short breaks (Hornby 1998 "Take-all disease of cereals"; Cook 2003 Annu Rev Phytopathol 41:
// "Take-all decline"). The numbers are in the data files with their source tags.

#include "ps/field/field.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace ps::field {

namespace {
inline real clamp01(real x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
constexpr real kLn2 = 0.6931471805599453;
} // namespace

void Field::init_pathogens() {
    std::set<std::string> fam;
    for (const CropDesc& cr : cfg_.crops) {
        if (cr.family.empty()) throw std::invalid_argument("Field: pathogens_enabled needs a family in crop " + cr.name);
        if (!cr.has_pathogen) throw std::invalid_argument("Field: pathogens_enabled needs a pathogen block in crop " + cr.name);
        fam.insert(cr.family);
    }
    family_names_.assign(fam.begin(), fam.end());
    const std::size_t F = family_names_.size();
    crop_family_.assign(cfg_.crops.size(), 0);
    std::vector<int> first(F, -1);
    for (std::size_t i = 0; i < cfg_.crops.size(); ++i) {
        const std::size_t f = static_cast<std::size_t>(std::find(family_names_.begin(), family_names_.end(), cfg_.crops[i].family) - family_names_.begin());
        crop_family_[i] = static_cast<int>(f);
        if (first[f] < 0) {
            first[f] = static_cast<int>(i);
            continue;
        }
        const PathogenDesc& a = cfg_.crops[static_cast<std::size_t>(first[f])].pathogen;
        const PathogenDesc& b = cfg_.crops[i].pathogen;
        if (a.decay_half_life_days != b.decay_half_life_days || a.suppression_half_life_days != b.suppression_half_life_days)
            throw std::invalid_argument("Field: crops " + cfg_.crops[static_cast<std::size_t>(first[f])].name + " and " + cfg_.crops[i].name +
                                        " are one family (" + cfg_.crops[i].family + ") and must have equal pathogen half-lives");
    }
    fam_decay_.assign(F, 1.0);
    fam_sdecay_.assign(F, 1.0);
    for (std::size_t f = 0; f < F; ++f) {
        const PathogenDesc& p = cfg_.crops[static_cast<std::size_t>(first[f])].pathogen;
        fam_decay_[f] = ps::math::exp(-kLn2 / p.decay_half_life_days);
        fam_sdecay_[f] = ps::math::exp(-kLn2 / p.suppression_half_life_days);
    }
    const std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    s_.path_p.assign(n * F, 0.0);
    s_.path_s.assign(n * F, 0.0);
    s_.path_loss0.assign(n, 0.0);
    s_.res_crop.assign(n, 0);
}

// What the fungicide sprays since have left for the growing crop (1 - product of what each protection leaves).
real Field::fungicide_protection(std::size_t c) const {
    if (s_.crop[c] == 0) return 0.0;
    const std::string& fam = cfg_.crops[static_cast<std::size_t>(s_.crop[c] - 1)].family;
    const std::size_t H = cfg_.herbicides.size();
    real left = 1.0;
    for (std::size_t h = 0; h < H; ++h) {
        const HerbicideDesc& hd = cfg_.herbicides[h];
        if (!(hd.fungicide_efficacy_max > 0.0)) continue;
        if (!hd.fungicide_family.empty() && hd.fungicide_family != fam) continue;
        left *= 1.0 - s_.h_act[c * H + h];
    }
    return 1.0 - left;
}

// The crop of cell c has just emerged: it meets the pressure of its family as it is now.
void Field::pathogen_emergence(std::size_t c) {
    const std::size_t F = family_names_.size();
    const std::size_t ci = static_cast<std::size_t>(s_.crop[c] - 1);
    const std::size_t f = static_cast<std::size_t>(crop_family_[ci]);
    const PathogenDesc& p = cfg_.crops[ci].pathogen;
    s_.path_loss0[c] = p.loss_max * s_.path_p[c * F + f] * (1.0 - p.suppression_max * s_.path_s[c * F + f]);
}

void Field::step_pathogens(std::size_t c, const DayFlows& fl) {
    FieldState& s = s_;
    const std::size_t F = family_names_.size();
    const int cid = s.crop[c];
    int host = -1;
    const PathogenDesc* hp = nullptr;
    if (cid != 0) {
        const CropDesc& cr = cfg_.crops[static_cast<std::size_t>(cid - 1)];
        if (fl.emerged && fl.stage < cr.stage_maturity) {
            host = crop_family_[static_cast<std::size_t>(cid - 1)];
            hp = &cr.pathogen;
        }
    }
    const real prot = host >= 0 ? fungicide_protection(c) : 0.0;
    const int rcid = s.res_crop[c];
    for (std::size_t f = 0; f < F; ++f) {
        real P = s.path_p[c * F + f], S = s.path_s[c * F + f];
        if (static_cast<int>(f) == host) {
            const real x = fl.gdd / hp->season_gdd;
            P += (1.0 - P) * (1.0 - ps::math::exp(-hp->build_per_season * x * (1.0 - prot)));
            S += (1.0 - S) * (1.0 - ps::math::exp(-hp->suppression_gain * P * x));
        } else {
            P *= fam_decay_[f];
        }
        S *= fam_sdecay_[f];
        if (rcid != 0 && crop_family_[static_cast<std::size_t>(rcid - 1)] == static_cast<int>(f)) {
            const PathogenDesc& rp = cfg_.crops[static_cast<std::size_t>(rcid - 1)].pathogen;
            const real surf = s.res_c[0][c] + s.res_c[1][c];
            P += (1.0 - P) * rp.residue_source_per_day * clamp01(surf / rp.residue_ref_c_kg_ha);
        }
        s.path_p[c * F + f] = P;
        s.path_s[c * F + f] = S;
    }
}

PathogenView Field::pathogen_view(int cell) const {
    if (!cfg_.pathogens_enabled) throw std::logic_error("Field::pathogen_view: the field runs without pathogens");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::pathogen_view: bad cell");
    const std::size_t c = static_cast<std::size_t>(cell), F = family_names_.size();
    PathogenView v;
    for (std::size_t f = 0; f < F; ++f) {
        v.inoculum.push_back(s_.path_p[c * F + f]);
        v.suppression.push_back(s_.path_s[c * F + f]);
    }
    v.loss_untreated = s_.path_loss0[c];
    v.fungicide_protection = fungicide_protection(c);
    v.growth_factor = s_.stress_rotation[c];
    return v;
}

void Field::set_pathogen(int cell, const std::string& family, real inoculum, real suppression) {
    if (!cfg_.pathogens_enabled) throw std::logic_error("Field::set_pathogen: the field runs without pathogens");
    if (cell < 0 || cell >= cell_count()) throw std::out_of_range("Field::set_pathogen: bad cell");
    if (!(inoculum >= 0.0 && inoculum < 1.0 && suppression >= 0.0 && suppression < 1.0))
        throw std::invalid_argument("Field::set_pathogen: inoculum and suppression must be in [0, 1)");
    const auto it = std::find(family_names_.begin(), family_names_.end(), family);
    if (it == family_names_.end()) throw std::invalid_argument("Field::set_pathogen: unknown family " + family);
    const std::size_t F = family_names_.size(), f = static_cast<std::size_t>(it - family_names_.begin());
    s_.path_p[static_cast<std::size_t>(cell) * F + f] = inoculum;
    s_.path_s[static_cast<std::size_t>(cell) * F + f] = suppression;
}

} // namespace ps::field
