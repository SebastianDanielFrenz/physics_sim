// tests/unit/test_field_nutrient_processes.cpp - single processes of the nutrient model, each against a cited
// figure or an independent hand computation: ammonia loss by method, liming, the annual pH loss, the stress hook
// (N, P/K, pH, lodging), nitrate transport, the bookkeeping of a deposit. The plan-level tests (balances, removal,
// decline, N response) are in test_field_nutrients.cpp; the sources are listed there.
// The recorded sabotages S3-S9 that these cases catch are listed in test_field_nutrients.cpp.

#include "field_test_support.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

using namespace ps;
using namespace ps::field;
using Catch::Approx;
namespace tf = ps::test_field;

namespace {

// Volatilised share of the ammoniacal N of one application on bare soil at constant weather.
double ammonia_share(const std::string& material, const std::string& method, double temp_c, double rain_day0 = 0.0,
                     double delay_h = -2.0, int days = 12) {
    Field f(tf::nutrient_config(tf::loess(), tf::wheat()));
    OperationEffect e;
    e.deposit = tf::deposit(material, method, 20.0, delay_h);
    f.apply(0, e);
    const MaterialDesc& m = tf::materials()[static_cast<std::size_t>(tf::mat_id(material) - 1)];
    const double tan = (m.nh4_n_kg_t + m.urea_n_kg_t) * 20.0;
    for (int d = 0; d < days; ++d) f.step_day(tf::synth(d, temp_c, temp_c, d == 0 ? rain_day0 : 0.0, 0.0, 1.0));
    return f.nutrient_ledger(0).n_volatilised / tan;
}

} // namespace

TEST_CASE("N5 ammonia loss by method, incorporation time, temperature and rain against the cited factors", "[field][nutrients]") {
    // 15 degC, no rain, the 20 t/ha of material left to itself for 12 days. Share of the ammoniacal N lost.
    const double cattle = ammonia_share("cattle_slurry_75", "broadcast", 15.0);
    const double hose = ammonia_share("cattle_slurry_75", "trailing_hose", 15.0);
    const double shoe = ammonia_share("cattle_slurry_75", "trailing_shoe", 15.0);
    const double slot_open = ammonia_share("cattle_slurry_75", "injected_open", 15.0);
    const double slot_closed = ammonia_share("cattle_slurry_75", "injected_closed", 15.0);
    const double pig = ammonia_share("pig_slurry_5", "broadcast", 15.0);
    const double digest = ammonia_share("digestate_liquid_75", "broadcast", 15.0);
    const double solid = ammonia_share("cattle_solid_manure", "broadcast", 15.0);
    std::printf("N5 EF broadcast cattle %.3f hose %.3f shoe %.3f open slot %.3f closed slot %.3f | pig %.3f digestate %.3f solid %.3f\n",
                cattle, hose, shoe, slot_open, slot_closed, pig, digest, solid);
    // UBA Texte 39/2013 Tab. 6.3 / 6.5 (EF of the ammoniacal N): cattle slurry 0.50, hose -10 %, shoe -30 %, open slot -60 %, closed slot > -80 %
    CHECK(cattle == Approx(0.50).margin(0.02));
    CHECK(hose == Approx(0.50 * 0.90).margin(0.03));
    CHECK(shoe == Approx(0.50 * 0.70).margin(0.03));
    CHECK(slot_open == Approx(0.50 * 0.40).margin(0.03));
    CHECK(slot_closed < 0.50 * 0.20 + 0.02);
    CHECK(cattle == Approx(0.55).margin(0.07));        // EMEP/EEA 3.B Table 3-9: cattle slurry 0.55
    CHECK(pig == Approx(0.40).margin(0.02));            // EMEP/EEA Table 3-9: finishing-pig slurry 0.40
    CHECK(solid == Approx(0.90).margin(0.03));          // UBA Tab. 6.3: cattle solid manure 0.90
    CHECK(pig < cattle);                                // LfL 2017: pig slurry < cattle slurry < digestate
    CHECK(cattle < digest);

    // Incorporation after 1 / 4 / 6 / 12 / 24 h removes 80 / 48 / 30 / 13 / 8 % of the broadcast loss (UBA Tab. 6.3).
    const double hours[] = {1.0, 4.0, 6.0, 12.0, 24.0};
    const double cited[] = {0.80, 0.48, 0.30, 0.13, 0.08};
    for (int i = 0; i < 5; ++i) {
        const double ef = ammonia_share("cattle_slurry_75", "broadcast", 15.0, 0.0, hours[i]);
        const double reduction = 1.0 - ef / cattle;
        std::printf("N5 incorporation after %2.0f h: reduction %.3f (cited %.2f)\n", hours[i], reduction, cited[i]);
        CHECK(reduction == Approx(cited[i]).margin(0.08));
    }
    // the same for solid manure after 1 / 4 / 24 h: -90 / -50 / 0 %
    const double solid_hours[] = {1.0, 4.0, 24.0};
    const double solid_cited[] = {0.90, 0.50, 0.0};
    for (int i = 0; i < 3; ++i) {
        const double reduction = 1.0 - ammonia_share("cattle_solid_manure", "broadcast", 15.0, 0.0, solid_hours[i]) / solid;
        CHECK(reduction == Approx(solid_cited[i]).margin(0.10));
    }

    // Temperature (LfL Versuchsbericht 2017): cold March under 35 % of the NH4, warm dry June about 60 %.
    const double cold = ammonia_share("cattle_slurry_75", "broadcast", 2.0);
    const double warm = ammonia_share("cattle_slurry_75", "broadcast", 22.0);
    std::printf("N5 cattle slurry broadcast: 2 degC %.3f, 15 degC %.3f, 22 degC %.3f\n", cold, cattle, warm);
    CHECK(cold < 0.35);
    CHECK(warm > 0.55);
    CHECK(warm < 0.70);
    // Rain soon after spreading washes the ammonium in and cuts the loss (EMEP: rain has a significant effect).
    const double wet = ammonia_share("cattle_slurry_75", "broadcast", 15.0, 10.0);
    CHECK(wet < 0.75 * cattle);

    // Mineral fertilisers at 15 degC, no rain: EMEP/EEA 3.D Table 3-2 / A1.1, g NH3 per kg N applied = 2.0 % (CAN),
    // 7.2 % (UAN) and 6.9 % (DAP) of the total N.
    const double can = ammonia_share("can_27", "broadcast", 15.0);
    const double uan = ammonia_share("uan_28", "broadcast", 15.0);
    const double dap = ammonia_share("dap_18_46", "broadcast", 15.0);
    const double can_n = can * 0.5, uan_n = uan * 0.75, dap_n = dap * 1.0;   // ammoniacal share of the total N
    std::printf("N5 mineral, share of the total N: CAN %.4f UAN %.4f DAP %.4f\n", can_n, uan_n, dap_n);
    CHECK(can_n == Approx(0.020).margin(0.004));
    CHECK(uan_n == Approx(0.072).margin(0.010));
    CHECK(dap_n == Approx(0.069).margin(0.010));
}

TEST_CASE("N6 liming: pH gain per tonne of CaO, reaction time and the acidity of a fertiliser", "[field][nutrients]") {
    // Soils without the unavoidable base loss, bare, constant 10 degC, no rain: only the lime acts.
    const auto gain = [](SoilDesc soil, const char* method, double cao_t, int days, double* reacted_fraction = nullptr) {
        soil.nutrients.cao_loss_kg_ha_per_mm_drain = 0.0;
        Field f(tf::nutrient_config(soil, tf::wheat()));
        const double ph0 = f.nutrient_view(0).ph;
        f.apply(0, {1.0, tf::deposit("lime_carbonate_90", method, cao_t * 1000.0 / 504.0)});
        for (int d = 0; d < days; ++d) f.step_day(tf::synth(d, 10.0, 10.0, 0.0, 0.0, 0.0));
        const NutrientView v = f.nutrient_view(0);
        if (reacted_fraction) *reacted_fraction = v.alk_cao / (cao_t * 1000.0);
        CHECK(std::fabs(f.cao_balance_residual(0)) < 1.0e-9);
        return (v.ph - ph0) / cao_t;
    };
    // After two years everything has reacted. VDLUFA / LfL Bas22 Tab. 8a, BAG 3-5 (silty loam): 117/75/50/25 dt CaO/ha
    // from pH 4.0/5.0/5.5/6.0, slopes 42-50 dt per pH unit = 0.20-0.24 pH per t CaO; BAG 2 (loamy sand) 39-38 dt (0.256-0.263)
    // between pH 4 and 5.5, 18 dt (0.55) above.
    const double loess = gain(tf::loess(), "incorporated", 2.0, 730);
    const double sandy = gain(tf::sandy(), "incorporated", 2.0, 730);
    std::printf("N6 pH gain per t CaO after two years: silt loam %.3f (table 0.20-0.24), sandy loam %.3f (table 0.256-0.55)\n", loess, sandy);
    CHECK(loess > 0.20);
    CHECK(loess < 0.24);
    CHECK(sandy > 0.256);
    CHECK(sandy < 0.55);
    // A pure carbonate lime counts by its CaO: 4 t of 50.4 % CaO equal 2 t CaO of any other material.
    const double ph_a = gain(tf::loess(), "incorporated", 1.0, 730);
    CHECK(ph_a == Approx(loess).epsilon(1.0e-9));

    // Reaction over months, faster when worked in: fractions reacted after 30 / 90 days / one year.
    double inc30, inc90, inc365, surf30, surf90, surf365;
    gain(tf::loess(), "incorporated", 2.0, 30, &inc30);
    gain(tf::loess(), "incorporated", 2.0, 90, &inc90);
    gain(tf::loess(), "incorporated", 2.0, 365, &inc365);
    gain(tf::loess(), "broadcast", 2.0, 30, &surf30);
    gain(tf::loess(), "broadcast", 2.0, 90, &surf90);
    gain(tf::loess(), "broadcast", 2.0, 365, &surf365);
    std::printf("N6 reacted share of the lime, incorporated 30/90/365 d: %.2f %.2f %.2f; on the surface: %.2f %.2f %.2f\n", inc30,
                inc90, inc365, surf30, surf90, surf365);
    CHECK(inc30 < 0.30);
    CHECK(inc90 > 0.40);
    CHECK(inc90 < 0.60);
    CHECK(inc365 > 0.90);
    CHECK(surf30 < inc30);
    CHECK(surf90 < inc90);
    CHECK(surf365 < inc365);
    CHECK(surf365 > 0.5);

    // An acidifying fertiliser costs its neutralising value at once: 100 kg N as CAN = 0.370 t x 150 kg CaO/t = 55.6 kg CaO.
    Field f(tf::nutrient_config(tf::loess(), tf::wheat()));
    const double ph0 = f.nutrient_view(0).ph;
    f.apply(0, tf::n_dose("can_27", "broadcast", 100.0));
    CHECK(f.nutrient_view(0).alk_cao == Approx(-100.0 / 270.0 * 150.0).epsilon(1.0e-12));
    CHECK(f.nutrient_view(0).ph == Approx(ph0 - 100.0 / 270.0 * 150.0 / 4600.0).epsilon(1.0e-9));
    // quicklime 90 acts like 900 kg CaO per tonne
    Field g(tf::nutrient_config(tf::loess(), tf::wheat()));
    g.apply(0, {1.0, tf::deposit("quicklime_90", "incorporated", 2.0)});
    CHECK(g.nutrient_view(0).lime_incorporated == Approx(1800.0).epsilon(1.0e-12));
}

TEST_CASE("N7 the annual CaO loss and pH fall of a fertilised wheat rotation against the DLG range", "[field][nutrients]") {
    // 190 kg N as CAN, DAP, MOP, no lime for ten years on the farm weather. DLG-Merkblatt 456 Tab. 1: the unavoidable
    // annual loss (leaching + neutralisation, fertiliser acidity included) is 200-570 kg CaO/ha, 300-600 on medium soils.
    tf::Plan plan;
    plan.n_kg = 190.0;
    plan.p2o5_kg = 70.0;
    plan.k2o_kg = 120.0;
    Field f(tf::nutrient_config(tf::loess(), tf::wheat()));
    const double ph0 = f.nutrient_view(0).ph;
    (void)tf::run_plan(f, plan, 10);
    const NutrientLedger l = f.nutrient_ledger(0);
    const double acid = -l.cao_applied;                       // no lime: everything applied is acidity
    const double per_year = (l.cao_lost + acid) / 10.0;
    const double ph_fall = (ph0 - f.nutrient_view(0).ph) / 10.0;
    std::printf("N7 CaO loss per year: natural %.0f + fertiliser %.0f = %.0f kg CaO/ha (cited 200-570); pH fall %.3f per year\n",
                l.cao_lost / 10.0, acid / 10.0, per_year, ph_fall);
    CHECK(per_year > 200.0);
    CHECK(per_year < 570.0);
    CHECK(acid > 0.0);
    CHECK(l.cao_lost > 0.0);
    CHECK(ph_fall == Approx(per_year / 4600.0).epsilon(1.0e-9));
    CHECK(std::fabs(f.cao_balance_residual(0)) < 1.0e-9);
}

TEST_CASE("N8 the stress hook: N, P/K, pH and lodging", "[field][nutrients]") {
    const CropDesc& wheat = tf::wheat();
    const int emerged_stage = wheat.stage_emergence;

    // --- N: a fertilised crop is not N-limited in spring, an unfertilised one is ---------------------------------------
    {
        double f_n_fert = 0.0, f_n_nil = 0.0;
        for (int variant = 0; variant < 2; ++variant) {
            tf::Plan plan;
            plan.n_kg = variant == 0 ? 220.0 : 0.0;
            plan.p2o5_kg = 80.0;
            plan.k2o_kg = 150.0;
            Field f(tf::nutrient_config(tf::loess(), wheat));
            DailyWeatherSource& src = tf::weather30();
            double min_fn = 1.0;
            for (std::int64_t d = 0; d < 365 + 150; ++d) {
                const DailyWeather& w = src.record(d);
                tf::apply_plan_day(f, w, plan, nullptr);
                f.step_day(w);
                if (d >= 365 + 100 && d <= 365 + 140) min_fn = std::min(min_fn, f.nutrient_view(0).f_n);
            }
            (variant == 0 ? f_n_fert : f_n_nil) = min_fn;
        }
        std::printf("N8 lowest N factor, doy 100-140 of the first spring: fertilised %.3f, unfertilised %.3f\n", f_n_fert, f_n_nil);
        CHECK(f_n_fert > 0.90);
        CHECK(f_n_nil < 0.65);   // F1d part 1: was 0.60; the lower early biomass (light-limited growth) cut the spring N demand, measured 0.603
    }

    // --- P and K: the factor is the Mitscherlich response of the soil test ----------------------------------------------------
    {
        SoilDesc so = tf::loess();
        so.nutrients.p_test_init_mg100g = 3.0;
        so.nutrients.k_test_init_mg100g = 5.0;
        Field f(tf::nutrient_config(so, wheat));
        f.sow(0, 1);
        int guard = 0;
        while (f.view(0).stage < emerged_stage && guard++ < 200) f.step_day(tf::synth(f.last_day() + 1, 6.0, 16.0));
        REQUIRE(f.view(0).stage >= emerged_stage);
        const NutrientView v = f.nutrient_view(0);
        const SoilNutrientDesc& n = so.nutrients;
        const double av = n.avail_floor_factor + (1.0 - n.avail_floor_factor) *
                                                     std::min(1.0, std::max(0.0, (v.ph - n.ph_avail_floor) / (n.ph_avail_full - n.ph_avail_floor)));
        const double fp = (1.0 - std::exp(-n.p_rel_b * v.p_test_mg100g * av)) / (1.0 - std::exp(-n.p_rel_b * n.p_target_mg100g));
        const double fk = (1.0 - std::exp(-n.k_rel_b * v.k_test_mg100g * av)) / (1.0 - std::exp(-n.k_rel_b * n.k_target_mg100g));
        std::printf("N8 f_pk %.4f, expected min(%.4f, %.4f)\n", v.f_pk, fp, fk);
        CHECK(v.f_pk == Approx(std::min(fp, fk)).epsilon(0.01));
        CHECK(v.f_pk < 0.9);
        CHECK(v.f_pk > 0.3);
    }

    // --- pH: below the optimum the growth factor is the linear response; it multiplies the growth --------------------------------
    {
        SoilDesc acid = tf::loess();
        acid.nutrients.ph_init = 4.2;
        acid.nutrients.avail_floor_factor = 1.0;   // isolate the crop response from the availability of N, P and K
        acid.nutrients.no3_init_kg_ha = 300.0;      // N is no limit, so the pH factor alone sets the growth (law of the minimum)
        SoilDesc good = acid;
        good.nutrients.ph_init = 6.0;
        double biomass[2], f_ph[2];
        for (int i = 0; i < 2; ++i) {
            Field f(tf::nutrient_config(i == 0 ? acid : good, wheat));
            f.sow(0, 1);
            for (int d = 0; d < 120; ++d) f.step_day(tf::synth(f.last_day() + 1, 7.0, 17.0, d % 4 == 0 ? 6.0 : 0.0));
            biomass[i] = f.view(0).biomass_g_m2;
            f_ph[i] = f.nutrient_view(0).f_ph;
        }
        const CropNutrientDesc& cn = wheat.nutrients;
        const double expect = cn.ph_floor_factor + (1.0 - cn.ph_floor_factor) * (4.2 - cn.ph_floor) / (cn.ph_opt_low - cn.ph_floor);
        std::printf("N8 pH 4.2: f_ph %.4f (expected %.4f), biomass ratio %.4f\n", f_ph[0], expect, biomass[0] / biomass[1]);
        CHECK(f_ph[0] == Approx(expect).epsilon(0.01));
        CHECK(f_ph[1] == 1.0);
        REQUIRE(biomass[1] > 50.0);
        CHECK(biomass[0] / biomass[1] == Approx(expect).epsilon(0.02));
    }

    // --- lodging: surplus mineral N in the sensitive window costs a share of the grain, exactly --------------------------------------
    {
        CropDesc no_lodging = wheat;
        no_lodging.nutrients.lodging_max_loss = 0.0;
        double loss = 0.0, ratio = 0.0, grain_lodged = 0.0;
        double results[2];
        for (int i = 0; i < 2; ++i) {
            Field f(tf::nutrient_config(tf::loess(), i == 0 ? wheat : no_lodging));
            DailyWeatherSource& src = tf::weather30();
            tf::Plan plan;
            plan.p2o5_kg = 80.0;
            plan.k2o_kg = 150.0;
            for (std::int64_t d = 0; d < 365 + 213; ++d) {
                const DailyWeather& w = src.record(d);
                tf::apply_plan_day(f, w, plan, nullptr);
                if (d / 365 == 1 && (w.doy == 90 || w.doy == 110 || w.doy == 125)) f.apply_all(tf::n_dose("can_27", "broadcast", 220.0));
                f.step_day(w);
                if (d == 365 + 211) {
                    results[i] = f.view(0).grain_g_m2;
                    if (i == 0) loss = f.nutrient_view(0).lodging_loss;
                }
            }
        }
        grain_lodged = results[0];
        ratio = results[0] / results[1];
        std::printf("N8 lodging: loss %.4f, grain ratio %.4f\n", loss, ratio);
        CHECK(loss > 0.05);
        CHECK(loss <= wheat.nutrients.lodging_max_loss);
        CHECK(ratio == Approx(1.0 - loss).epsilon(1.0e-9));
        CHECK(grain_lodged > 100.0);
    }
}

TEST_CASE("N9 nitrate moves with the water through a mixing cell", "[field][nutrients]") {
    // Bare sandy soil at field capacity, all biology off, 100 kg N as CAN (50 kg nitrate N). One wet day: the nitrate of
    // the root layer is carried below by the water that percolates; the layer below passes its share out of the profile.
    SoilDesc so = tf::sandy();
    SoilNutrientDesc& n = so.nutrients;
    n.min_rate_per_day_20c = 0.0;
    n.fresh_rate_per_day_20c = 0.0;
    n.nitrif_rate_per_day_20c = 0.0;
    n.denit_rate_per_day_20c = 0.0;
    n.n_deposition_kg_ha_day = 0.0;
    n.org_n_kg_ha = 0.0;
    FieldConfig cfg = tf::nutrient_config(so, tf::wheat());
    cfg.initial_depletion_fraction = 0.0;
    Field f(cfg);
    f.apply(0, tf::n_dose("can_27", "broadcast", 100.0));
    const NutrientView b = f.nutrient_view(0);
    CHECK(b.no3_root == Approx(50.0 + 40.0 * 0.3 / 1.5).epsilon(1.0e-12));
    CHECK(b.no3_sub == Approx(40.0 * 1.2 / 1.5).epsilon(1.0e-12));

    f.step_day(tf::synth(0, 8.0, 12.0, 45.0, 0.0, 1.0));
    const CellView cv = f.view(0);
    const double infil = 45.0 - cv.last_runoff_mm;
    const double dp = infil - cv.last_e_mm;                  // the root zone started at field capacity
    REQUIRE(dp > 5.0);
    REQUIRE(cv.last_drain_mm == Approx(dp).epsilon(1.0e-9));  // ... and so did the layer below: it all leaves
    const double v1 = so.theta_fc * 0.3 * 1000.0, v2 = so.theta_fc * 1.2 * 1000.0;
    const double mv = b.no3_root * (1.0 - std::exp(-dp / v1));
    const double out = (b.no3_sub + mv) * (1.0 - std::exp(-dp / v2));
    const NutrientView a = f.nutrient_view(0);
    std::printf("N9 dp %.2f mm: moved %.3f kg, left the profile %.3f kg\n", dp, mv, out);
    CHECK(a.no3_root == Approx(b.no3_root - mv).epsilon(1.0e-9));
    CHECK(a.no3_sub == Approx(b.no3_sub + mv - out).epsilon(1.0e-9));
    CHECK(f.nutrient_ledger(0).n_leached == Approx(out).epsilon(1.0e-9));
    CHECK(std::fabs(f.n_balance_residual(0)) < 1.0e-9);

    // A wet spell carries all of it out; more rain carries more, and the balance still closes.
    double leached_prev = f.nutrient_ledger(0).n_leached;
    for (int d = 1; d < 40; ++d) {
        f.step_day(tf::synth(d, 8.0, 12.0, 30.0, 0.0, 1.0));
        const double l = f.nutrient_ledger(0).n_leached;
        CHECK(l >= leached_prev);
        leached_prev = l;
    }
    CHECK(f.nutrient_view(0).no3_root + f.nutrient_view(0).no3_sub < 0.05 * (b.no3_root + b.no3_sub));
    CHECK(std::fabs(f.n_balance_residual(0)) < 1.0e-9);
}

TEST_CASE("N10 a deposit is booked into the right pools", "[field][nutrients]") {
    const auto& mats = tf::materials();
    const auto mat = [&](const char* n) -> const MaterialDesc& { return mats[static_cast<std::size_t>(tf::mat_id(n) - 1)]; };
    // cattle slurry, 30 t/ha, trailing hose: ammoniacal N on the surface, organic N split slow / fresh, P2O5, K2O
    {
        Field f(tf::nutrient_config(tf::loess(), tf::wheat()));
        const NutrientView b = f.nutrient_view(0);
        f.apply(0, {1.0, tf::deposit("cattle_slurry_75", "trailing_hose", 30.0)});
        const NutrientView a = f.nutrient_view(0);
        const MaterialDesc& m = mat("cattle_slurry_75");
        CHECK(a.tan_surface - b.tan_surface == Approx(m.nh4_n_kg_t * 30.0).epsilon(1.0e-12));
        CHECK(a.org_slow - b.org_slow == Approx((m.n_total_kg_t - m.nh4_n_kg_t) * 30.0 * m.org_slow_fraction).epsilon(1.0e-12));
        CHECK(a.org_fresh - b.org_fresh == Approx((m.n_total_kg_t - m.nh4_n_kg_t) * 30.0 * (1.0 - m.org_slow_fraction)).epsilon(1.0e-12));
        CHECK(a.p2o5_avail - b.p2o5_avail == Approx(m.p2o5_kg_t * 30.0).epsilon(1.0e-12));
        CHECK(a.k2o_avail - b.k2o_avail == Approx(m.k2o_kg_t * 30.0).epsilon(1.0e-12));
        CHECK(a.alk_cao == b.alk_cao);
        const NutrientLedger l = f.nutrient_ledger(0);
        CHECK(l.n_fert_mineral + l.n_fert_organic == Approx(m.n_total_kg_t * 30.0).epsilon(1.0e-12));
    }
    // closed-slot injection puts 85 % of the ammoniacal N in the soil at once
    {
        Field f(tf::nutrient_config(tf::loess(), tf::wheat()));
        const NutrientView b = f.nutrient_view(0);
        f.apply(0, {1.0, tf::deposit("pig_slurry_5", "injected_closed", 10.0)});
        const NutrientView a = f.nutrient_view(0);
        const double tan = mat("pig_slurry_5").nh4_n_kg_t * 10.0;
        CHECK(a.nh4 - b.nh4 == Approx(0.85 * tan).epsilon(1.0e-12));
        CHECK(a.tan_surface - b.tan_surface == Approx(0.15 * tan).epsilon(1.0e-12));
    }
    // UAN: the urea and ammonium N lie on the surface, the nitrate N goes into the soil; a double pass applies twice
    {
        Field f(tf::nutrient_config(tf::loess(), tf::wheat()));
        const NutrientView b = f.nutrient_view(0);
        f.apply(0, tf::n_dose("uan_28", "broadcast", 60.0, 2.0));
        const NutrientView a = f.nutrient_view(0);
        CHECK(a.tan_surface - b.tan_surface == Approx(2.0 * 60.0 * 0.75).epsilon(1.0e-12));
        CHECK(a.no3_root - b.no3_root == Approx(2.0 * 60.0 * 0.25).epsilon(1.0e-12));
        CHECK(a.alk_cao == Approx(-2.0 * 60.0 / 280.0 * 280.0).epsilon(1.0e-12));
    }
    // DAP and MOP carry P2O5 and K2O to the available pools and the ledger
    {
        Field f(tf::nutrient_config(tf::loess(), tf::wheat()));
        f.apply(0, {1.0, tf::deposit("dap_18_46", "broadcast", 0.2)});
        f.apply(0, {1.0, tf::deposit("mop_60", "broadcast", 0.2)});
        const NutrientLedger l = f.nutrient_ledger(0);
        CHECK(l.p_applied == Approx(92.0).epsilon(1.0e-12));
        CHECK(l.k_applied == Approx(120.0).epsilon(1.0e-12));
    }
    // refusals
    {
        Field f(tf::nutrient_config(tf::loess(), tf::wheat()));
        CHECK_THROWS_AS(f.apply(0, {1.0, tf::deposit("can_27", "broadcast", -1.0)}), std::invalid_argument);
        CHECK_THROWS_AS(f.apply(1, {1.0, tf::deposit("can_27", "broadcast", 1.0)}), std::out_of_range);
        Deposit d;
        d.material = 99;
        d.method = 1;
        CHECK_THROWS_AS(f.apply(0, {1.0, d}), std::invalid_argument);
        Field plain(tf::one_cell_config(tf::loess(), tf::wheat()));
        CHECK_THROWS_AS(plain.apply(0, {1.0, tf::deposit("can_27", "broadcast", 1.0)}), std::logic_error);
        CHECK_THROWS_AS(plain.nutrient_view(0), std::logic_error);
    }
}
