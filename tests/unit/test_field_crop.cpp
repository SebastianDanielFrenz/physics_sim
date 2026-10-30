// tests/unit/test_field_crop.cpp - the AquaCrop-style winter wheat of
// ps::field::Field: phenology, vernalisation, canopy, biomass, harvest index,
// frost, ripeness, persistence of an unharvested crop, and the two outside
// references (FAO-33 yield response, German statistics).
//
// Recorded sabotages (F1a, 2026-10-31), each made in core/src/field/field.cpp,
// the test run, then reverted (failing cases in brackets):
//  - C-a: GDD from the unclamped Tmax/Tmin [C1, C7];
//  - C-b: vernalisation response always 1 [C2];
//  - C-c: vernalisation factor dropped from the development rate [C2, C10, C12];
//  - C-d: canopy phase two replaced by exponential growth [C3];
//  - C-e: senescence exponent without the 1/peak [C3];
//  - C-f: biomass with Tr instead of Tr/ET0 [C4, C11];
//  - C-g: stress factors multiplied instead of min() [C5];
//  - C-h: frost kill ignoring the snow shield [C7];
//  - C-i: shedding from the first day after maturity [C8, C9, C4];
//  - C-j: biomass multiplied by (1 + 0.001 doy), i.e. the model reads the date
//         [C6; C4 and C11 fail as well];
//  - C-k: Ks_sto forced to 1 (no water stress on transpiration or pollination) [C10];
//  - C-l: WP* doubled [C4, C11].
// C10 checks the response against FAO-33 and the crop file's hi_water_stress_k
// is FITTED to it (see that file); C10 therefore guards the calibration.
#include "field_test_support.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>

using namespace ps;
using namespace ps::field;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

CropDesc no_vern() {
    CropDesc c = test_field::wheat();
    c.vernalises = false;
    return c;
}

// A field of one cell, irrigated, sown, with constant weather.
Field irrigated_field(const CropDesc& crop, int nx = 1, int ny = 1) {
    Field f(test_field::one_cell_config(test_field::loess(), crop, nx, ny));
    f.set_irrigate_to_capacity(true);
    f.sow_all(1);
    return f;
}

double cc_star(double cc) { return std::min(1.0, 1.72 * cc - cc * cc + 0.3 * cc * cc * cc); }

bool same_view(const CellView& a, const CellView& b) {
    return a.soil == b.soil && a.crop_id == b.crop_id && a.stage == b.stage && a.bbch == b.bbch && a.gdd_sow == b.gdd_sow &&
           a.vern_days == b.vern_days && a.cc == b.cc && a.stand == b.stand && a.biomass_g_m2 == b.biomass_g_m2 &&
           a.grain_g_m2 == b.grain_g_m2 && a.days_ripe == b.days_ripe && a.zr_m == b.zr_m && a.d_root_mm == b.d_root_mm &&
           a.d_sub_mm == b.d_sub_mm && a.d_evap_mm == b.d_evap_mm && a.swe_mm == b.swe_mm && a.last_tr_mm == b.last_tr_mm &&
           a.last_e_mm == b.last_e_mm && a.last_runoff_mm == b.last_runoff_mm && a.last_drain_mm == b.last_drain_mm;
}

// A continuous 30-year winter-wheat run of one cell on the given soil, natural rain: sown doy 288,
// harvested doy 212 of the next year. Yields in t/ha dry matter, per harvest.
struct YearlyRun {
    std::vector<double> grain_dm;
    std::vector<double> doy_bbch30, doy_bbch65, doy_bbch89;
};

YearlyRun yearly_run(const SoilDesc& soil) {
    DailyWeatherSource& src = test_field::weather30();
    FieldConfig cfg = test_field::one_cell_config(soil, test_field::wheat(), 1, 1, 0.2);
    Field f(cfg);
    YearlyRun r;
    double d30 = -1, d65 = -1, d89 = -1;
    for (std::int64_t d = 0; d < 30 * 365; ++d) {
        const DailyWeather& w = src.record(d);
        if (w.doy == 288) {
            f.sow_all(1);
            d30 = d65 = d89 = -1;
        }
        if (w.doy == 212 && d > 365) {
            const HarvestResult h = f.harvest(0);
            if (h.crop_id != 0) {
                r.grain_dm.push_back(h.grain_dm_t_ha);
                r.doy_bbch30.push_back(d30);
                r.doy_bbch65.push_back(d65);
                r.doy_bbch89.push_back(d89);
            }
        }
        f.step_day(w);
        const CellView v = f.view(0);
        if (v.crop_id != 0) {
            if (d30 < 0 && v.bbch >= 30.0) d30 = w.doy;
            if (d65 < 0 && v.bbch >= 65.0) d65 = w.doy;
            if (d89 < 0 && v.bbch >= 89.0) d89 = w.doy;
        }
    }
    return r;
}

const YearlyRun& loess_run() {
    static const YearlyRun r = yearly_run(test_field::loess());
    return r;
}

double mean_of(const std::vector<double>& v) { return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size()); }
double sd_of(const std::vector<double>& v) {
    const double m = mean_of(v);
    double s = 0.0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size() - 1));
}

} // namespace

TEST_CASE("C1 phenology at a constant temperature, development is thermal time", "[field][crop]") {
    const CropDesc crop = no_vern();
    Field f = irrigated_field(crop);
    // Tmin 5, Tmax 15 -> 10 GDD per day.
    for (int k = 1; k <= 260; ++k) {
        f.step_day(test_field::synth(k - 1, 5.0, 15.0, 0.0, 0.0, 3.0));
        const CellView v = f.view(0);
        int expected = 0;
        for (int i = 0; i < crop.stage_count(); ++i)
            if (10.0 * k >= crop.stages[static_cast<std::size_t>(i)].gdd) expected = i;
        INFO("day " << k);
        REQUIRE(v.stage == expected);
        REQUIRE_THAT(v.gdd_sow, WithinAbs(10.0 * k, 1e-9));
    }
    // Emergence at 140 GDD = 14 days exactly; BBCH interpolates linearly between the stage codes.
    Field g = irrigated_field(crop);
    for (int k = 1; k <= 14; ++k) {
        g.step_day(test_field::synth(k - 1, 5.0, 15.0, 0.0, 0.0, 3.0));
        if (k == 7) CHECK_THAT(g.view(0).bbch, WithinAbs(5.0, 1e-9));
        if (k == 13) CHECK(g.view(0).stage == 0);
    }
    CHECK(g.view(0).stage == 1);
    CHECK_THAT(g.view(0).bbch, WithinAbs(10.0, 1e-9));
    // Both temperatures are clamped to [Tbase, Tupper]: a frost day adds nothing, a 40 C day at most 26 - 0.
    Field h = irrigated_field(crop);
    h.step_day(test_field::synth(0, -10.0, -2.0, 0.0, 0.0, 3.0));
    CHECK(h.view(0).gdd_sow == 0.0);
    h.step_day(test_field::synth(1, 30.0, 40.0, 0.0, 0.0, 3.0));
    CHECK_THAT(h.view(0).gdd_sow, WithinAbs(26.0, 1e-12));
}

TEST_CASE("C2 vernalisation rate: Wang-Engel response and the slowing of development", "[field][crop]") {
    const CropDesc& crop = test_field::wheat();
    // Independent evaluation of the response curve (libm, the field uses ps::math).
    auto rate = [&](double t) {
        if (t <= crop.vern_t_min_c || t >= crop.vern_t_max_c) return 0.0;
        const double alpha = std::log(2.0) / std::log((crop.vern_t_max_c - crop.vern_t_min_c) / (crop.vern_t_opt_c - crop.vern_t_min_c));
        const double a = std::pow(t - crop.vern_t_min_c, alpha), b = std::pow(crop.vern_t_opt_c - crop.vern_t_min_c, alpha);
        return (2.0 * a * b - a * a) / (b * b);
    };
    auto vern_after_one_day = [&](double tmean) {
        Field f = irrigated_field(crop);
        f.step_day(test_field::synth(0, tmean - 2.0, tmean + 2.0, 0.0, 0.0, 0.0));
        return f.view(0).vern_days;
    };
    CHECK_THAT(vern_after_one_day(crop.vern_t_opt_c), WithinAbs(1.0, 1e-12));   // 1 effective day per day at the optimum
    CHECK_THAT(vern_after_one_day(crop.vern_t_min_c), WithinAbs(0.0, 1e-9));   // (the mean of the two inputs may round to a hair above t_min)
    CHECK_THAT(vern_after_one_day(crop.vern_t_max_c), WithinAbs(0.0, 1e-9));
    CHECK(vern_after_one_day(crop.vern_t_min_c - 0.5) == 0.0);
    CHECK(vern_after_one_day(crop.vern_t_max_c + 0.5) == 0.0);
    CHECK_THAT(vern_after_one_day(10.0), WithinAbs(rate(10.0), 1e-9));
    CHECK_THAT(vern_after_one_day(10.0), WithinAbs(0.7393, 5e-4));                // hand value: 2 r - r^2, r = (11.3/6.2)^0.68717
    CHECK_THAT(vern_after_one_day(0.0), WithinAbs(rate(0.0), 1e-9));
    CHECK(vern_after_one_day(0.0) > 0.0);
    CHECK(vern_after_one_day(2.0) > vern_after_one_day(10.0));                    // nearer the optimum

    // At the optimum: vernalisation days +1 per day, development rate 0.3 + 0.7 days/50.
    Field f = irrigated_field(crop);
    double expected_gdd = 0.0;
    for (int n = 1; n <= 10; ++n) {
        f.step_day(test_field::synth(n - 1, 2.9, 6.9, 0.0, 0.0, 0.0));            // mean 4.9, 4.9 GDD per day
        expected_gdd += 4.9 * (0.3 + 0.7 * n / 50.0);
        REQUIRE_THAT(f.view(0).vern_days, WithinAbs(static_cast<double>(n), 1e-12));
        REQUIRE_THAT(f.view(0).gdd_sow, WithinAbs(expected_gdd, 1e-9));
    }
    // Saturation: vernalisation days stop at 50, the factor reaches 1.
    for (int n = 11; n <= 80; ++n) f.step_day(test_field::synth(n - 1, 2.9, 6.9, 0.0, 0.0, 0.0));
    CHECK_THAT(f.view(0).vern_days, WithinAbs(50.0, 1e-12));
    const double g0 = f.view(0).gdd_sow;
    f.step_day(test_field::synth(80, 2.9, 6.9, 0.0, 0.0, 0.0));
    CHECK_THAT(f.view(0).gdd_sow - g0, WithinAbs(4.9, 1e-9));

    // Slower to the end of the vernalisation period than the same crop without vernalisation.
    auto days_to_stem_elongation = [&](const CropDesc& c) {
        Field g = irrigated_field(c);
        for (int d = 0; d < 1000; ++d) {
            g.step_day(test_field::synth(d, 2.9, 6.9, 0.0, 0.0, 0.0));
            if (g.view(0).stage >= c.stage_vernalisation_end) return d + 1;
        }
        return -1;
    };
    const int with = days_to_stem_elongation(crop);
    const int without = days_to_stem_elongation(no_vern());
    CHECK(without == 164);            // 800 / 4.9, rounded up
    // 50 days at 4.9 (0.3 + 0.014 n) GDD = 160.965 GDD, then 4.9 GDD per day: 50 + ceil((800 - 160.965) / 4.9) days.
    CHECK(with == 181);
    // After the vernalisation period development is not slowed any more.
    Field g = irrigated_field(crop);
    for (int d = 0; d < with + 5; ++d) g.step_day(test_field::synth(d, 2.9, 6.9, 0.0, 0.0, 0.0));
    const double a = g.view(0).gdd_sow;
    g.step_day(test_field::synth(with + 5, 2.9, 6.9, 0.0, 0.0, 0.0));
    CHECK_THAT(g.view(0).gdd_sow - a, WithinAbs(4.9, 1e-9));
}

TEST_CASE("C3 canopy cover follows the AquaCrop growth and senescence laws", "[field][crop]") {
    const CropDesc crop = no_vern();
    Field f = irrigated_field(crop);
    const double cc0 = crop.cc0(1.0);
    CHECK_THAT(cc0, WithinAbs(0.0375, 1e-12));
    const double g = crop.cgc_per_gdd * 10.0;
    const double ccx = crop.ccx;

    // Phase boundary of the discrete exponential growth.
    int m1 = 0;
    while (cc0 * std::exp(g * m1) < 0.5 * ccx) ++m1;
    const double cc_half = cc0 * std::exp(g * m1);
    CHECK(m1 > 40);
    CHECK(m1 < 55);

    double peak = 0.0;
    for (int k = 1; k <= 300; ++k) {
        f.step_day(test_field::synth(k - 1, 5.0, 15.0, 0.0, 0.0, 3.0));
        const double cc = f.view(0).cc;
        INFO("day " << k);
        if (k < 14) {
            REQUIRE(cc == 0.0);                       // before emergence the canopy is not shown
        } else if (k < 190) {
            const int m = k - 13;                     // growth steps since emergence (the emergence day counts)
            const double expected = m <= m1 ? cc0 * std::exp(g * m) : ccx - (ccx - cc_half) * std::exp(-g * (m - m1));
            REQUIRE_THAT(cc, WithinRel(expected, 1e-10));
            REQUIRE(cc <= ccx + 1e-15);
            if (k == 189) peak = cc;
        } else if (k <= 250) {
            // senescence from 1900 GDD (day 190): CC = peak (1 - 0.05 (exp(CDC t / peak) - 1)), t in GDD since then
            const double t = 10.0 * (k - 189);
            const double expected = std::max(0.0, peak * (1.0 - 0.05 * (std::exp(crop.cdc_per_gdd / peak * t) - 1.0)));
            REQUIRE_THAT(cc, WithinAbs(expected, 1e-10));
        }
    }
    CHECK(peak > 0.99 * ccx);                         // the canopy reaches CCx
    CHECK(f.view(0).cc >= 0.0);
}

TEST_CASE("C4 biomass = WP* Tr/ET0 (thermally limited), harvest index builds after flowering", "[field][crop]") {
    const CropDesc crop = no_vern();
    // Same crop at ET0 = 4 and 6 mm/day: Tr/ET0 (the crop coefficient) is the same, so is the biomass.
    Field a = irrigated_field(crop), b = irrigated_field(crop);
    double prev_biomass = 0.0;
    for (int k = 1; k <= 234; ++k) {
        a.step_day(test_field::synth(k - 1, 5.0, 15.0, 0.0, 0.0, 4.0));
        b.step_day(test_field::synth(k - 1, 5.0, 15.0, 0.0, 0.0, 6.0));
        const CellView va = a.view(0), vb = b.view(0);
        INFO("day " << k);
        if (va.cc > 0.0) {
            // Ks = 1: Tr = Kcb ET0 with Kcb = 1.10 CC* (CC* = 1.72 CC - CC^2 + 0.3 CC^3)
            REQUIRE_THAT(va.last_tr_mm, WithinRel(crop.kc_tr_x * cc_star(va.cc) * 4.0, 1e-12));
            // dB = WP* Tr/ET0 x (GDD/14), 10 GDD per day
            REQUIRE_THAT(va.biomass_g_m2 - prev_biomass,
                         WithinAbs(crop.wp_star_g_m2 * crop.kc_tr_x * cc_star(va.cc) * (10.0 / crop.biomass_gdd_full), 1e-9));
        }
        REQUIRE_THAT(va.biomass_g_m2, WithinRel(vb.biomass_g_m2, 1e-12));
        prev_biomass = va.biomass_g_m2;
        // Harvest index: grain = biomass HI0 min(1, hi_gdd / 700), hi_gdd = 10 (k - 159) from flowering (160 GDD-days).
        if (k >= 160) {
            const double hi = crop.hi0 * std::min(1.0, 10.0 * (k - 159) / crop.hi_build_gdd);
            REQUIRE_THAT(va.grain_g_m2, WithinRel(va.biomass_g_m2 * hi, 1e-12));
        } else {
            REQUIRE(va.grain_g_m2 == 0.0);
        }
    }
    CHECK(prev_biomass > 1200.0);        // 12 t/ha biomass by maturity: the season of a healthy crop
    CHECK(prev_biomass < 2200.0);

    // Heat at flowering: Tmax 36 C between 32 and 38 fails a third of ... two thirds of the pollination.
    const CropDesc c2 = no_vern();
    Field h = irrigated_field(c2);
    for (int k = 1; k <= 20; ++k) h.step_day(test_field::synth(k - 1, 16.0, 36.0, 0.0, 0.0, 4.0));     // 21 GDD/day
    while (h.view(0).stage < c2.stage_maturity) h.step_day(test_field::synth(h.last_day() + 1, 16.0, 36.0, 0.0, 0.0, 4.0));
    // freeze one day before maturity is hard to catch; use the final grain/biomass ratio: HI0 (1 - 4/6)
    const CellView v = h.view(0);
    CHECK_THAT(v.grain_g_m2 / v.biomass_g_m2, WithinRel(crop.hi0 * (1.0 - (36.0 - 32.0) / (38.0 - 32.0)), 1e-9));
}

TEST_CASE("C5 stress factors combine as a minimum, rotation multiplies", "[field][crop]") {
    const CropDesc crop = no_vern();
    Field f = irrigated_field(crop, 3, 1);
    f.set_stress(1, StressFactors{0.5, 0.8, 0.9, 1.0, 1.0});    // min = 0.5 (a product would give 0.36)
    f.set_stress(2, StressFactors{0.5, 1.0, 1.0, 1.0, 0.8});    // min 0.5 x rotation 0.8 = 0.4
    for (int d = 0; d < 120; ++d) f.step_day(test_field::synth(d, 5.0, 15.0, 0.0, 0.0, 4.0));
    const double b0 = f.view(0).biomass_g_m2;
    CHECK(b0 > 200.0);
    CHECK_THAT(f.view(1).biomass_g_m2 / b0, WithinAbs(0.5, 1e-12));
    CHECK_THAT(f.view(2).biomass_g_m2 / b0, WithinAbs(0.4, 1e-12));
    // The stress factors do not touch the canopy or the water: the transpiration is the same.
    CHECK(f.view(1).cc == f.view(0).cc);
    CHECK(f.view(1).last_tr_mm == f.view(0).last_tr_mm);
    CHECK_THROWS_AS(f.set_stress(0, StressFactors{1.2, 1, 1, 1, 1}), std::invalid_argument);
}

TEST_CASE("C6 nothing is a function of the calendar: the same weather on other dates gives the same state",
          "[field][crop]") {
    DailyWeatherSource& src = test_field::weather30();
    FieldConfig cfg;
    cfg.nx = 2;
    cfg.ny = 2;
    cfg.soils = {test_field::loess(), test_field::sandy()};
    cfg.crops = {test_field::wheat()};
    cfg.cell_soil = {0, 1, 1, 0};
    cfg.initial_depletion_fraction = 0.25;
    FieldConfig cfg_b = cfg;
    cfg_b.initial_day = 4999;
    Field a(cfg), b(cfg_b);
    const std::int64_t start = 100;
    const int n_days = 820;
    for (int i = 0; i < n_days; ++i) {
        DailyWeather wa = src.record(start + i);
        DailyWeather wb = wa;
        wa.day = i;
        wb.day = 5000 + i;
        wb.doy = (wa.doy + 97) % 365 + 1;          // a different date, same weather
        if (i == 188) { a.sow_all(1); b.sow_all(1); }
        if (i == 520) { a.harvest_all(); b.harvest_all(); }
        if (i == 560) { a.sow_all(1, 0.8); b.sow_all(1, 0.8); }
        a.step_day(wa);
        b.step_day(wb);
        if (i % 41 == 0 || i == n_days - 1)
            for (int c = 0; c < 4; ++c) REQUIRE(same_view(a.view(c), b.view(c)));
    }
    for (int c = 0; c < 4; ++c) {
        const WaterLedger la = a.ledger(c), lb = b.ledger(c);
        CHECK(la.transpiration == lb.transpiration);
        CHECK(la.drainage == lb.drainage);
    }
    CHECK(a.view(0).crop_id == 1);
    CHECK(a.view(0).biomass_g_m2 > 0.0);
}

TEST_CASE("C7 frost kills an emerged stand, snow cover shields it", "[field][crop]") {
    const CropDesc crop = no_vern();
    auto make = [&] {
        Field f = irrigated_field(crop);
        for (int d = 0; d < 40; ++d) f.step_day(test_field::synth(d, 5.0, 15.0, 0.0, 0.0, 2.0));
        return f;
    };
    {
        Field f = make();
        const CellView before = f.view(0);
        REQUIRE(before.stage >= 1);
        f.step_day(test_field::synth(40, -20.0, -10.0, 0.0, 0.0, 1.0));    // kill = (-12 + 20) / (-12 + 22) = 0.8
        CHECK_THAT(f.view(0).stand, WithinAbs(0.2, 1e-12));
        CHECK_THAT(f.view(0).cc, WithinRel(0.2 * before.cc, 1e-10));
    }
    {
        Field f = make();
        f.step_day(test_field::synth(40, -17.0, -10.0, 0.0, 0.0, 1.0));    // 0.5
        CHECK_THAT(f.view(0).stand, WithinAbs(0.5, 1e-12));
    }
    {
        Field f = make();
        f.step_day(test_field::synth(40, -12.0, -5.0, 0.0, 0.0, 1.0));     // exactly the onset: no kill
        CHECK(f.view(0).stand == 1.0);
    }
    {   // snow cover of 15 mm (>= 10) holds the stand at -3 C at the coldest
        Field f = make();
        f.step_day(test_field::synth(40, -8.0, -2.0, 0.0, 15.0, 1.0));
        REQUIRE_THAT(f.view(0).swe_mm, WithinAbs(15.0, 1e-12));
        f.step_day(test_field::synth(41, -20.0, -10.0, 0.0, 0.0, 1.0));
        CHECK(f.view(0).stand == 1.0);
    }
    {   // 5 mm are not enough
        Field f = make();
        f.step_day(test_field::synth(40, -8.0, -2.0, 0.0, 5.0, 1.0));
        f.step_day(test_field::synth(41, -20.0, -10.0, 0.0, 0.0, 1.0));
        CHECK_THAT(f.view(0).stand, WithinAbs(0.2, 1e-12));
    }
    {   // a seedling that has not emerged yet is not killed
        Field f = irrigated_field(crop);
        for (int d = 0; d < 5; ++d) f.step_day(test_field::synth(d, 5.0, 15.0, 0.0, 0.0, 2.0));
        f.step_day(test_field::synth(5, -20.0, -10.0, 0.0, 0.0, 1.0));
        CHECK(f.view(0).stand == 1.0);
        CHECK(f.view(0).stage == 0);
    }
    {   // the killed canopy regrows only to the surviving stand: a 20 % stand cannot exceed 20 % of CCx
        Field f = make();
        f.step_day(test_field::synth(40, -20.0, -10.0, 0.0, 0.0, 1.0));
        for (int d = 41; d < 150; ++d) f.step_day(test_field::synth(d, 5.0, 15.0, 0.0, 0.0, 2.0));
        CHECK(f.view(0).cc <= 0.2 * crop.ccx + 1e-12);
        CHECK(f.view(0).cc > 0.15 * crop.ccx);
    }
}

TEST_CASE("C8 ripe grain waits for the harvest window, then sheds", "[field][crop]") {
    const CropDesc crop = no_vern();
    Field f = irrigated_field(crop);
    for (int k = 1; k <= 234; ++k) f.step_day(test_field::synth(k - 1, 5.0, 15.0, 0.0, 0.0, 3.0));
    REQUIRE(f.view(0).stage == crop.stage_maturity - 1);
    f.step_day(test_field::synth(234, 5.0, 15.0, 0.0, 0.0, 3.0));          // gdd 2350: maturity
    REQUIRE(f.view(0).stage == crop.stage_maturity);
    const double grain = f.view(0).grain_g_m2;
    const double biomass = f.view(0).biomass_g_m2;
    CHECK(grain > 0.0);
    double prev = grain;
    for (int j = 1; j <= 24; ++j) {
        f.step_day(test_field::synth(234 + j, 5.0, 15.0, 0.0, 0.0, 3.0));
        const double now = f.view(0).grain_g_m2;
        INFO("day " << j << " after maturity");
        // days_ripe = 1 on the maturity day, so it exceeds the 14-day window on day j = 14.
        if (j <= 13) REQUIRE(now == prev);
        else REQUIRE_THAT(now / prev, WithinAbs(1.0 - crop.shed_rate_per_day, 1e-12));
        prev = now;
    }
    CHECK(f.view(0).biomass_g_m2 == biomass);          // the standing biomass stays
    CHECK(f.view(0).days_ripe == 25.0);
}

TEST_CASE("C9 an unharvested crop stays in the cell for years", "[field][crop]") {
    DailyWeatherSource& src = test_field::weather30();
    const std::int64_t sow_day = test_field::find_doy(src, 3 * 365, 288);
    FieldConfig cfg = test_field::one_cell_config(test_field::loess(), test_field::wheat(), 1, 1, 0.2);
    cfg.initial_day = sow_day - 1;
    Field f(cfg);
    f.sow_all(1);
    double grain_at_maturity = -1.0, prev_grain = 1e300, prev_ripe = -1.0, biomass_at_maturity = -1.0;
    for (std::int64_t d = sow_day; d < sow_day + 3 * 365 + 200; ++d) {
        f.step_day(src.record(d));
        const CellView v = f.view(0);
        REQUIRE(v.crop_id == 1);                        // nothing but an operation removes it
        REQUIRE(std::isfinite(v.d_root_mm));
        if (v.stage >= test_field::wheat().stage_maturity) {
            if (grain_at_maturity < 0.0) { grain_at_maturity = v.grain_g_m2; biomass_at_maturity = v.biomass_g_m2; }
            REQUIRE(v.biomass_g_m2 == biomass_at_maturity);
            REQUIRE(v.grain_g_m2 <= prev_grain);
            REQUIRE(v.days_ripe == prev_ripe + 1.0);
            prev_grain = v.grain_g_m2;
            prev_ripe = v.days_ripe;
        } else {
            prev_ripe = 0.0;                            // days_ripe is 1 on the day the crop reaches maturity
        }
    }
    const CellView v = f.view(0);
    CHECK(grain_at_maturity > 300.0);
    CHECK(v.days_ripe > 3.0 * 365 - 250);
    // 0.4 % of the remaining grain a day after the 14-day window: 0.996^(days_ripe - 14) is left.
    CHECK_THAT(v.grain_g_m2 / grain_at_maturity, WithinRel(std::pow(0.996, v.days_ripe - 14.0), 1e-9));
    CHECK(v.grain_g_m2 < 0.03 * grain_at_maturity);
    CHECK(v.grain_g_m2 > 0.0);
    CHECK(v.cc == 0.0);                                 // the canopy is gone
    CHECK_THAT(v.bbch, WithinAbs(92.0, 1e-12));
    CHECK(std::fabs(f.water_balance_residual_mm(0)) < 1e-9);
    // An operation removes it, and the cell is free again.
    const HarvestResult h = f.harvest(0);
    CHECK(h.crop_id == 1);
    CHECK(f.view(0).crop_id == 0);
    CHECK(f.harvest(0).crop_id == 0);
}

TEST_CASE("C10 FAO-33 yield response: relative yield drops with the ET deficit with Ky = 1.0 (winter wheat)",
          "[field][crop]") {
    // FAO Irrigation and Drainage Paper 33 (Doorenbos & Kassam 1979), winter wheat, seasonal Ky = 1.00
    // (table at fao.org/4/y3655e/y3655e04.htm): 1 - Ya/Ym = Ky (1 - ETa/ETm).
    // Reference run: irrigated to field capacity every morning (ETm, Ym); drought runs: the same season with the
    // rain and snow scaled. Several seasons x several rainfall levels.
    const SoilDesc& so = test_field::loess();
    const CropDesc& crop = test_field::wheat();
    double worst = 0.0;
    int stressed_runs = 0;
    double ky_sum = 0.0;
    for (int year = 1; year <= 9; ++year) {
        const test_field::SeasonResult ref = test_field::run_season(so, crop, year, 1.0, true);
        REQUIRE(ref.grain_dm_t_ha > 5.0);
        for (double scale : {0.0, 0.2, 0.35, 0.5, 0.7}) {
            const test_field::SeasonResult r = test_field::run_season(so, crop, year, scale, false);
            const double et_rel = r.et_mm / ref.et_mm;
            const double y_rel = r.grain_dm_t_ha / ref.grain_dm_t_ha;
            const double predicted = 1.0 - 1.0 * (1.0 - et_rel);
            INFO("year " << year << " rain x" << scale << " ETa/ETm " << et_rel << " Ya/Ym " << y_rel << " predicted " << predicted);
            worst = std::max(worst, std::fabs(predicted - y_rel));
            REQUIRE(std::fabs(r.residual_mm) < 1e-9);
            REQUIRE(y_rel <= 1.05);
            if (et_rel < 0.9) {
                ++stressed_runs;
                ky_sum += (1.0 - y_rel) / (1.0 - et_rel);
            }
        }
    }
    INFO("worst |predicted - simulated| " << worst << ", mean implied Ky " << (stressed_runs ? ky_sum / stressed_runs : 0.0));
    CHECK(stressed_runs >= 15);                           // the drought runs really are droughts
    CHECK(worst < 0.12);                                  // measured 0.097 (hi_water_stress_k 0.4 is fitted to this, see the crop file)
    // The implied Ky of the stressed runs is of the order of the cited 1.0.
    CHECK(ky_sum / stressed_runs > 0.7);
    CHECK(ky_sum / stressed_runs < 1.3);
}

TEST_CASE("C11 German winter wheat: yield of 29 harvests against the national statistics", "[field][crop]") {
    // Destatis winter wheat yields of Germany 2011-2020 (as tabulated by proplanta.de, quoting DESTATIS), dt/ha:
    const double national_dt_ha[] = {70.6, 74.0, 80.3, 86.3, 81.5, 76.9, 76.9, 67.7, 74.5, 78.9};
    std::vector<double> nat(std::begin(national_dt_ha), std::end(national_dt_ha));
    for (double& x : nat) x *= 0.1;                      // t/ha fresh matter
    const double nat_mean = mean_of(nat);                // 7.676
    const double nat_sd = sd_of(nat);
    CHECK_THAT(nat_mean, WithinAbs(7.676, 1e-9));

    const YearlyRun& run = loess_run();
    REQUIRE(run.grain_dm.size() == 29);
    std::vector<double> fresh;
    for (double dm : run.grain_dm) fresh.push_back(dm / 0.86);       // standard 14 % grain moisture
    const double m = mean_of(fresh), sd = sd_of(fresh);
    INFO("national mean " << nat_mean << " sd " << nat_sd << " t/ha; model mean " << m << " sd " << sd << " min "
                          << *std::min_element(fresh.begin(), fresh.end()) << " max "
                          << *std::max_element(fresh.begin(), fresh.end()));
    // Good deep loess with the home climate: at or somewhat above the national mean (national yields average
    // lighter soils and poorer years; the best German regions reach 9-10 t/ha), well inside the national
    // range of single years (6.8-8.6).
    CHECK(m > 0.92 * nat_mean);
    CHECK(m < 1.20 * nat_mean);
    // The year-to-year spread is that of a rain-fed crop: a local site varies more than the national average.
    CHECK(sd / m > 0.05);
    CHECK(sd / m < 0.20);
    for (double y : fresh) {
        CHECK(y > 0.55 * nat_mean);
        CHECK(y < 1.40 * nat_mean);
    }
}

TEST_CASE("C12 winter wheat phenology dates in a German climate", "[field][crop]") {
    // DWD phenological network, Winterweizen in the Rhein-Main lowland (recalled, not re-read): shooting (BBCH 30)
    // in April, flowering (BBCH 65) in the first half of June, full ripeness (BBCH 89) in July.
    const YearlyRun& run = loess_run();
    REQUIRE(run.doy_bbch30.size() == 29);
    for (const auto* v : {&run.doy_bbch30, &run.doy_bbch65, &run.doy_bbch89})
        for (double d : *v) REQUIRE(d > 0.0);
    INFO("BBCH30 " << mean_of(run.doy_bbch30) << " (sd " << sd_of(run.doy_bbch30) << "), BBCH65 " << mean_of(run.doy_bbch65)
                   << " (sd " << sd_of(run.doy_bbch65) << "), BBCH89 " << mean_of(run.doy_bbch89) << " (sd "
                   << sd_of(run.doy_bbch89) << ")");
    CHECK(mean_of(run.doy_bbch30) > 75.0);       // 16 March
    CHECK(mean_of(run.doy_bbch30) < 120.0);      // 30 April
    CHECK(mean_of(run.doy_bbch65) > 145.0);      // 25 May
    CHECK(mean_of(run.doy_bbch65) < 170.0);      // 19 June
    CHECK(mean_of(run.doy_bbch89) > 188.0);      // 7 July
    CHECK(mean_of(run.doy_bbch89) < 215.0);      // 3 August
    // Year to year the dates move by a day or two only: the climate model has weather noise but no year-level
    // temperature anomalies (observed standard deviations are 5-8 days). Bounds guard against a collapse either way.
    CHECK(sd_of(run.doy_bbch65) < 12.0);
    CHECK(sd_of(run.doy_bbch89) < 12.0);
    CHECK(sd_of(run.doy_bbch65) > 0.2);
    CHECK(sd_of(run.doy_bbch89) > 0.2);
}

// Hidden measurement report: the numbers behind the bands of C10-C12 and the water totals.
//   ps_field_tests "F1a measurement report" -s    (prints with std::printf)
TEST_CASE("F1a measurement report", "[.][diag][field]") {
    const SoilDesc& so = test_field::loess();
    const CropDesc& crop = test_field::wheat();
    std::printf("\n-- FAO-33 (winter wheat, Ky 1.00): ETa/ETm, Ya/Ym, predicted, deviation, implied Ky --\n");
    double worst = 0.0;
    for (int year = 1; year <= 9; ++year) {
        const test_field::SeasonResult ref = test_field::run_season(so, crop, year, 1.0, true);
        const test_field::SeasonResult nat = test_field::run_season(so, crop, year, 1.0, false);
        std::printf("year %d irrigated: grain %.2f t/ha dm, ETm %.0f mm, biomass %.0f g/m2; natural rain %.0f mm: grain %.2f ETa/ETm %.3f\n",
                    year, ref.grain_dm_t_ha, ref.et_mm, ref.biomass_g_m2, nat.precip_mm, nat.grain_dm_t_ha, nat.et_mm / ref.et_mm);
        for (double scale : {0.0, 0.2, 0.35, 0.5, 0.7}) {
            const test_field::SeasonResult r = test_field::run_season(so, crop, year, scale, false);
            const double et_rel = r.et_mm / ref.et_mm, y_rel = r.grain_dm_t_ha / ref.grain_dm_t_ha;
            const double pred = et_rel;
            worst = std::max(worst, std::fabs(pred - y_rel));
            std::printf("   rain x%.2f (%.0f mm): ETa/ETm %.3f Ya/Ym %.3f predicted %.3f dev %+.3f Ky %.2f\n", scale, r.precip_mm,
                        et_rel, y_rel, pred, y_rel - pred, et_rel < 0.999 ? (1.0 - y_rel) / (1.0 - et_rel) : 0.0);
        }
    }
    std::printf("worst deviation %.3f\n", worst);
    for (const SoilDesc* s : {&test_field::loess(), &test_field::sandy()}) {
        const YearlyRun r = yearly_run(*s);
        std::vector<double> fresh;
        for (double dm : r.grain_dm) fresh.push_back(dm / 0.86);
        std::printf("\n-- %s: %zu harvests, fresh yield mean %.2f sd %.2f min %.2f max %.2f t/ha; BBCH30 doy %.1f (sd %.1f), BBCH65 %.1f (%.1f), BBCH89 %.1f (%.1f)\n",
                    s->name.c_str(), fresh.size(), mean_of(fresh), sd_of(fresh), *std::min_element(fresh.begin(), fresh.end()),
                    *std::max_element(fresh.begin(), fresh.end()), mean_of(r.doy_bbch30), sd_of(r.doy_bbch30), mean_of(r.doy_bbch65),
                    sd_of(r.doy_bbch65), mean_of(r.doy_bbch89), sd_of(r.doy_bbch89));
    }
}
