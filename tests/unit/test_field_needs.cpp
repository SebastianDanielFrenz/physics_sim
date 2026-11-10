// Needs of the crop a game may switch off (FieldConfig::lime_need_enabled, pathogen_loss_enabled, weed_loss_enabled): on (default) the model is as before,
// off the yield effect of that cause is exactly none and every state keeps stepping.
//
//  ND  defaults: the three flags are true, and a field with them spelled out is the field without (state hash);
//  NL  lime: pH below the crop's optimum costs growth by default; with the need off the pH growth factor is exactly 1 every day, the pH and
//      the lime pools step as before, and the crop grows more;
//  NP  pathogens: the loss stays computed (loss_untreated, inoculum, suppression identical to the field with the need on) but the growth
//      factor is exactly 1, so the crop grows like its twin without the pathogen model;
//  NW  weeds: the competition factor is exactly 1, the weeds draw no water from the crop and the crop yields more; the weeds do not come at
//      all (no germination, standing plants go to the residue, no mineral N drawn): the crop ends close to its twin without a weed palette.
//
// Recorded sabotages (made in core/src/field, this file run, reverted; 2026-11-10):
//  - NS-w2 (2026-11-10): weed_to_residue + continue removed in step_weeds: NW fails (weeds_off above 0);
//  - NS-l: the lime flag ignored (nf_ph kept): NL fails (f_ph below 1 with the need off);
//  - NS-p: the pathogen flag ignored (loss kept in stress_rotation): NP fails (growth_factor below 1);
//  - NS-w: the weed flag ignored in manage_prestep only: NW fails (weed_factor below 1); ignored in the transpiration only: NW fails (weed transpiration above 0).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "field_test_support.h"
#include "ps/field/field_scenario.h"

#include <algorithm>
#include <cmath>

using namespace ps;
using namespace ps::field;
using Catch::Matchers::WithinAbs;
namespace tf = ps::test_field;

TEST_CASE("ND the needs are on by default and spelling them out changes nothing", "[field][needs]") {
    const FieldConfig cfg = tf::pathogen_config();
    CHECK(cfg.lime_need_enabled);
    CHECK(cfg.pathogen_loss_enabled);
    CHECK(cfg.weed_loss_enabled);
    FieldConfig spelled = cfg;
    spelled.lime_need_enabled = spelled.pathogen_loss_enabled = spelled.weed_loss_enabled = true;
    Field a(cfg), b(spelled);
    a.sow(0, 1);
    b.sow(0, 1);
    for (int i = 0; i < 60; ++i) {
        const DailyWeather w = tf::synth(a.last_day() + 1, 12.0, 12.0, 2.0, 0.0, 3.0);
        a.step_day(w);
        b.step_day(w);
    }
    CHECK(a.fold_hash(1469598103934665603ull) == b.fold_hash(1469598103934665603ull));
}

TEST_CASE("NS set_needs switches a running field and a switched field equals one made with the flags", "[field][needs]") {
    SoilDesc soil = tf::loess();
    soil.nutrients.ph_init = 4.5;
    const FieldConfig on = tf::nutrient_config(soil, tf::wheat());
    FieldConfig off = on;
    off.lime_need_enabled = false;
    Field a(on), b(off);
    a.sow(0, 1);
    b.sow(0, 1);
    for (int i = 0; i < 40; ++i) {
        const DailyWeather w = tf::synth(a.last_day() + 1, 14.0, 14.0, 2.0, 0.0, 3.0);
        a.step_day(w);
        b.step_day(w);
    }
    CHECK(a.fold_hash(1469598103934665603ull) != b.fold_hash(1469598103934665603ull));
    a.set_needs(false, true, true);   // the player switches the need off on day 40: from now on a follows the same rule as b
    CHECK(a.nutrient_view(0).f_ph < 1.0);   // the factor of the last step, set before the switch
    const DailyWeather w = tf::synth(a.last_day() + 1, 14.0, 14.0, 2.0, 0.0, 3.0);
    a.step_day(w);
    CHECK(a.nutrient_view(0).f_ph == 1.0);
    a.set_needs(true, true, true);
    a.step_day(tf::synth(a.last_day() + 1, 14.0, 14.0, 2.0, 0.0, 3.0));
    CHECK(a.nutrient_view(0).f_ph < 1.0);   // and back on
}

TEST_CASE("NL lime: with the need off the pH costs no growth", "[field][needs][lime]") {
    SoilDesc soil = tf::loess();
    soil.nutrients.ph_init = 4.5;   // below the wheat's optimum (5.4) and nearly at its floor (4.0)
    FieldConfig on = tf::nutrient_config(soil, tf::wheat());
    FieldConfig off = on;
    off.lime_need_enabled = false;
    Field a(on), b(off);
    a.sow(0, 1);
    b.sow(0, 1);
    double min_on = 1.0;
    for (int i = 0; i < 120; ++i) {
        const DailyWeather w = tf::synth(a.last_day() + 1, 14.0, 14.0, 2.0, 0.0, 3.0);
        a.step_day(w);
        b.step_day(w);
        min_on = std::min(min_on, a.nutrient_view(0).f_ph);
        CHECK(b.nutrient_view(0).f_ph == 1.0);
        CHECK(a.nutrient_view(0).ph == b.nutrient_view(0).ph);   // the pH keeps stepping alike
        CHECK(a.nutrient_view(0).alk_cao == b.nutrient_view(0).alk_cao);
    }
    REQUIRE(min_on < 0.9);   // the cost is real with the need on
    CHECK(b.view(0).biomass_g_m2 > a.view(0).biomass_g_m2);
}

TEST_CASE("NP pesticide: with the need off the pathogen costs no growth", "[field][needs][pathogen]") {
    const FieldConfig on = tf::pathogen_config();
    FieldConfig off = on;
    off.pathogen_loss_enabled = false;
    FieldConfig none = on;
    none.pathogens_enabled = false;
    Field a(on), b(off), c(none);
    for (Field* f : {&a, &b}) f->set_pathogen(0, "cereal", 0.5, 0.4);
    for (Field* f : {&a, &b, &c}) f->sow(0, 1);
    for (int i = 0; i < 110; ++i) {
        const DailyWeather w = tf::synth(a.last_day() + 1, 15.0, 15.0, 2.0, 0.0, 3.0);
        a.step_day(w);
        b.step_day(w);
        c.step_day(w);
        CHECK(b.pathogen_view(0).growth_factor == 1.0);
        CHECK(b.pathogen_view(0).loss_untreated == a.pathogen_view(0).loss_untreated);   // the pathogen state keeps stepping alike
        CHECK(b.pathogen_view(0).inoculum == a.pathogen_view(0).inoculum);
    }
    REQUIRE(a.pathogen_view(0).loss_untreated > 0.05);
    CHECK(a.view(0).biomass_g_m2 < b.view(0).biomass_g_m2);
    CHECK_THAT(b.view(0).biomass_g_m2, WithinAbs(c.view(0).biomass_g_m2, 1e-9));   // as if the model were off
}

TEST_CASE("NW weeds: with the need off the weeds cost no yield, draw no nitrogen and do not come", "[field][needs][weeds]") {
    const ps::io::FieldScenario sc = ps::io::load_field_scenario(tf::data_dir() / "field_scenarios/wheat_managed_11y.json");
    FieldConfig on = sc.field;
    on.nx = on.ny = 1;
    on.cell_soil.assign(1, 0);
    FieldConfig off = on;
    off.weed_loss_enabled = false;
    FieldConfig none = on;   // the twin without a weed palette at all
    none.weeds.clear();
    DailyWeatherSource src(sc.climate, 21, sc.weather);
    Field a(on), b(off), c(none);
    double min_on = 1.0, weeds_off = 0.0, weeds_on = 0.0;
    for (std::int64_t d = 0; d < 3 * 365; ++d) {
        const DailyWeather& w = src.record(d);
        for (const ps::io::FieldOperationRule& rule : sc.rules) {
            if (w.doy != rule.doy || d / 365 < rule.first_year) continue;
            if (rule.every_years > 1 && (d / 365 - rule.first_year) % rule.every_years != 0) continue;
            if (rule.op != "operation" && rule.op != "apply") continue;
            if (rule.effect.spray.product != 0) continue;   // no spraying: the weeds stand
            a.apply_all(rule.effect);
            b.apply_all(rule.effect);
            c.apply_all(rule.effect);
        }
        a.step_day(w);
        b.step_day(w);
        c.step_day(w);
        min_on = std::min(min_on, a.management_view(0).weed_factor);
        CHECK(b.management_view(0).weed_factor == 1.0);
        for (const WeedGroupView& g : b.management_view(0).weeds) weeds_off += g.biomass_g_m2;
        for (const WeedGroupView& g : a.management_view(0).weeds) weeds_on += g.biomass_g_m2;
    }
    REQUIRE(min_on < 0.95);   // the competition is real with the need on
    CHECK(b.management_view(0).weed_transpiration_mm == 0.0);
    CHECK(a.management_view(0).weed_transpiration_mm > 0.0);
    CHECK(weeds_on > 0.0);
    CHECK(weeds_off == 0.0);   // the weeds do not come
    // and the crop is where its twin without a weed palette is (the weeds' residue and the seed bank are the only differences)
    CHECK_THAT(b.view(0).biomass_g_m2, WithinAbs(c.view(0).biomass_g_m2, 0.02 * std::max(1.0, c.view(0).biomass_g_m2)));
}
