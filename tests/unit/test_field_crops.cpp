// F1d part 2: the four added crops (winter barley, winter rapeseed, grain maize, sugar beet) against the Destatis national
// yields, plus their data (family tags, C4 CO2 response, root-crop flag).
//
// Recorded sabotages (core/src/field/field.cpp or the data file, run on this file, reverted; measured 2026-10-31):
//  - CR-a: the pollination switch ignored in field.cpp (always on) -> CR3 fails (the beet harvest index collapses at Tmax 42); CR2 does not
//    notice it, the beet never meets the heat limit on the home climate;
//  - CR-b: sugar_beet.json yield.pollination removed -> loader keeps the default 1, CR1 and CR3 fail (not re-run after the CR3 rewrite, by construction);
//  - CR-c: grain_maize.json biomass.rue_g_mj doubled -> CR2 maize fails its upper band (15.44 t/ha, ratio 1.62).
#include "field_test_support.h"
#include "ps/field/field_scenario.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdio>
#include <string>

using namespace ps;
using namespace ps::field;
using Catch::Matchers::WithinAbs;

namespace {

CropDesc load_crop(const char* name) { return ps::io::load_crop_json(test_field::data_dir() / "crops" / (std::string(name) + ".json")); }

struct NationalRef {
    const char* crop;
    double fresh_per_dm;      // fresh matter per dry matter of the harvested product
    double national_t_ha;     // Destatis, fresh matter at the standard moisture
    double lo, hi;            // accepted ratio model / national
};

}  // namespace

TEST_CASE("CR1 the added crops load with their family, photosynthetic type and root-crop flag", "[field][crop]") {
    const CropDesc wheat = load_crop("winter_wheat"), barley = load_crop("winter_barley"), rape = load_crop("winter_rapeseed"),
                   maize = load_crop("grain_maize"), beet = load_crop("sugar_beet");
    CHECK(wheat.family == "cereal");
    CHECK(barley.family == "cereal");        // one family with wheat (take-all, eyespot hosts)
    CHECK(rape.family == "brassica");
    CHECK(maize.family == "maize");
    CHECK(beet.family == "beet");
    CHECK(wheat.pollination);
    CHECK(barley.pollination);
    CHECK(rape.pollination);
    CHECK(maize.pollination);
    CHECK_FALSE(beet.pollination);           // a root crop
    // AquaCrop Annex I thermal limits as printed
    CHECK_THAT(barley.t_upper_c, WithinAbs(15.0, 1e-12));
    CHECK_THAT(maize.t_base_c, WithinAbs(8.0, 1e-12));
    CHECK_THAT(beet.t_base_c, WithinAbs(5.0, 1e-12));
    CHECK_THAT(maize.wp_star_g_m2, WithinAbs(33.7, 1e-12));
    // C4 maize: f_type = (40 - 33.7)/20 = 0.315 of the C3 CO2 response (1.073 at 400 ppm)
    CHECK_THAT(maize.co2_factor(400.0), WithinAbs(1.0 + 0.315 * 0.073, 1e-12));
    CHECK_THAT(wheat.co2_factor(400.0), WithinAbs(1.073, 1e-12));
    CHECK_THAT(beet.co2_factor(400.0), WithinAbs(1.073, 1e-12));      // WP* 17 -> f_type 1
    for (const CropDesc* c : {&barley, &rape, &maize, &beet}) CHECK(crop_validation_error(*c).empty());
}

TEST_CASE("CR2 thirty years of each added crop on the home loess against the Destatis national yields", "[field][crop]") {
    // Destatis (Fachserie 3 Reihe 3.2.1, Wachstum und Ernte - Feldfruechte 2022, table 'Deutschland 2016/21'): winter rapeseed
    // 33.6 dt/ha, grain maize (incl. corn-cob mix, 14 % moisture) 95.3, sugar beet 752.2; winter barley: BMEL Erntebericht 2020
    // (Destatis) six-year mean 2014/19 71.9 dt/ha. The scenario sites are the deep loess of the home climate (a favourable site:
    // winter wheat on it is +5 % over the national mean), good-practice fertilising, straw/stover left.
    const NationalRef refs[] = {
        {"winter_barley", 1.0 / 0.86, 7.19, 0.85, 1.12},
        {"winter_rapeseed", 1.0 / 0.91, 3.36, 0.95, 1.20},
        {"grain_maize", 1.0 / 0.86, 9.53, 0.90, 1.25},
        {"sugar_beet", 1.0 / 0.23, 75.22, 0.80, 1.10},
    };
    for (const NationalRef& r : refs) {
        const auto path = test_field::data_dir() / "field_scenarios" / (std::string("crop_") + r.crop + "_30y.json");
        const io::FieldScenario sc = io::load_field_scenario(path);
        const io::FieldRunSummary sum = io::run_field_scenario(sc);
        REQUIRE(sum.harvests.size() >= 29);
        double mean = 0.0, lo = 1e9, hi = 0.0;
        for (const auto& h : sum.harvests) {
            const double fresh = h.result.grain_dm_t_ha * r.fresh_per_dm;
            mean += fresh;
            lo = std::min(lo, fresh);
            hi = std::max(hi, fresh);
        }
        mean /= static_cast<double>(sum.harvests.size());
        std::printf("CR2 %-16s model mean %.2f t/ha fresh (min %.2f max %.2f), national %.2f, ratio %.3f\n", r.crop, mean, lo, hi,
                    r.national_t_ha, mean / r.national_t_ha);
        INFO(r.crop << " model " << mean << " national " << r.national_t_ha);
        CHECK(mean > r.lo * r.national_t_ha);
        CHECK(mean < r.hi * r.national_t_ha);
        // the ledgers close and the single-year spread is that of a rain-fed crop
        CHECK(sum.max_balance_residual_mm < 1e-9);
        CHECK(sum.max_n_residual < 1e-9);
        CHECK(sum.max_p_residual < 1e-9);
        CHECK(sum.max_k_residual < 1e-9);
        CHECK(sum.max_cao_residual < 1e-9);
        CHECK(lo > 0.35 * r.national_t_ha);
        CHECK(hi < 1.7 * r.national_t_ha);
    }
}

TEST_CASE("CR3 a root crop has no pollination: heat at 'flowering' does not touch the sugar beet harvest index", "[field][crop]") {
    CropDesc beet = load_crop("sugar_beet");
    REQUIRE_FALSE(beet.pollination);
    CropDesc flowering_beet = beet;
    flowering_beet.pollination = true;       // the same crop treated as a flowering crop: Tmax 42 is 7 K over the 35 C limit
    const auto final_hi = [](const CropDesc& c) {
        Field f(test_field::one_cell_config(test_field::loess(), c));
        f.set_irrigate_to_capacity(true);
        f.sow_all(1);
        int guard = 0;
        while (f.view(0).stage < c.stage_maturity && guard++ < 700) f.step_day(test_field::synth(f.last_day() + 1, 16.0, 42.0, 0.0, 0.0, 4.0));
        REQUIRE(f.view(0).stage >= c.stage_maturity);
        const CellView v = f.view(0);
        return v.grain_g_m2 / v.biomass_g_m2;
    };
    CHECK_THAT(final_hi(beet), Catch::Matchers::WithinRel(beet.hi0, 0.01));   // irrigated: Ks_sto = 1; the build-up is 99.5 % complete at the maturity day
    CHECK(final_hi(flowering_beet) < 1e-6);                     // pollination fails completely
}
