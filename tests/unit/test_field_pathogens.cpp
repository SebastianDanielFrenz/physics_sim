// F1d part 4: soil-borne pathogen pressure per crop family and cell, its effect on growth through stress_rotation,
// and the fungicide product of the herbicide-like spray path.
//
// What the tests pin down (the model is in core/src/field/pathogens.cpp, the numbers in data/crops/*.json "pathogen"):
//  P1  data and loaders: the five crops carry a pathogen block with source tags, wheat and barley are one family with
//      equal family half-lives, every rejection message, pathogens_enabled needs management and complete crops;
//  P2  decay: without a host the inoculum falls with its half-life, the suppression with its own, a crop of another
//      family is no host;
//  P3  build: a host crop closes the inoculum gap 1 - P by exp(-b gdd/season_gdd) per day, suppression follows;
//  P4  the loss: fixed at emergence from the pressure the crop meets, applied through stress_rotation, the biomass
//      of the pathogen field is (1 - loss) of its twin without pathogens; set_stress cannot touch the factor;
//  P5  surface residue raises the inoculum; ploughing it under removes most of that; zero source, zero effect;
//  P6  fungicide: protection from a spray, decay, cap below the efficacy limit, family filter, bare soil, part of the
//      loss bought back and never all of it;
//  P7  determinism: partitions, checkpoint/restore and lazy jumps give the same bits with pathogens on;
//  P8  published margins (tag [long]): second wheat 10-15 % below first wheat after a break, third wheat 10-20 %,
//      the peak in year 3-5 followed by decline, rapeseed with a 4-year break 8-15 % above rapeseed every third year.
//
// Recorded sabotages (made in core/src/field/pathogens.cpp or weeds.cpp, this file run, reverted; 2026-10-31):
//  - PA-a: decay applied also while the host grows: P3 fails (and the P8 margins move out of their bands); P2 does not notice;
//  - PA-b: the build ignoring (1 - protection): P6 fails ('sprayed inoculum below unsprayed');
//  - PA-c: the loss fixed without the suppression factor: P4 fails;
//  - PA-d: the residue source counting the buried pools too: P5 fails (the plough check);
//  - PA-e: fungicide protection capped at 1 instead of efficacy_max: P6 fails (the cap check);
//  - PA-f: step_pathogens reading the next cell's inoculum: P7 and P10 fail (P7 only because cells 7 and 8 carry one family
//    on purpose; with mixed neighbours it did not notice).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include "field_test_support.h"
#include "ps/field/field_scenario.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::field;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
namespace tf = ps::test_field;
using nlohmann::json;

namespace {

const double kLn2 = 0.6931471805599453;

json load_json_file(const std::filesystem::path& p) {
    std::ifstream in(p);
    return json::parse(in);
}

OperationEffect op(const FieldConfig& cfg, const std::string& name) {
    ps::io::OperationPalette pal;
    for (const auto& c : cfg.crops) pal.crops.push_back(c.name);
    for (const auto& h : cfg.herbicides) pal.herbicides.push_back(h.name);
    for (const auto& m : cfg.materials) pal.materials.push_back(m.name);
    for (const auto& m : cfg.methods) pal.methods.push_back(m.name);
    return ps::io::load_operation_json(tf::data_dir() / "operations" / (name + ".json"), pal);
}

void run_synth(Field& f, int n, double tmin = 15.0, double tmax = 15.0, double rain = 2.0, double et0 = 3.0) {
    for (int i = 0; i < n; ++i) f.step_day(tf::synth(f.last_day() + 1, tmin, tmax, rain, 0.0, et0));
}

void run_to(Field& f, DailyWeatherSource& src, std::int64_t day) {
    for (std::int64_t d = f.last_day() + 1; d <= day; ++d) f.step_day(src.record(d));
}

}  // namespace

TEST_CASE("P1 the pathogen data and its loaders", "[field][pathogen]") {
    const CropDesc wheat = tf::crop_named("winter_wheat"), barley = tf::crop_named("winter_barley"), rape = tf::crop_named("winter_rapeseed"),
                   maize = tf::crop_named("grain_maize"), beet = tf::crop_named("sugar_beet");
    for (const CropDesc* c : {&wheat, &barley, &rape, &maize, &beet}) {
        INFO(c->name);
        CHECK(c->has_pathogen);
        CHECK(crop_validation_error(*c).empty());
        CHECK(c->pathogen.loss_max > 0.0);
        CHECK(c->pathogen.loss_max < 1.0);
    }
    CHECK(wheat.family == barley.family);   // one family: take-all and the cereal root diseases
    CHECK(wheat.pathogen.decay_half_life_days == barley.pathogen.decay_half_life_days);
    CHECK(wheat.pathogen.suppression_half_life_days == barley.pathogen.suppression_half_life_days);
    CHECK(rape.pathogen.suppression_max == 0.0);   // take-all decline is a cereal phenomenon here
    CHECK(rape.pathogen.decay_half_life_days > 3.0 * wheat.pathogen.decay_half_life_days);   // resting spores outlast take-all

    // fungicide product
    const HerbicideDesc fung = ps::io::load_herbicide_json(tf::data_dir() / "herbicides/cereal_fungicide.json");
    CHECK(fung.fungicide_efficacy_max > 0.0);
    CHECK(fung.fungicide_efficacy_max < 1.0);
    CHECK(fung.fungicide_family == "cereal");
    CHECK(herbicide_validation_error(fung).empty());

    // rejections: a missing source tag, an out-of-range loss, an unknown key, an empty agent
    json j = load_json_file(tf::data_dir() / "crops/winter_wheat.json");
    {
        json b = j;
        b["pathogen"]["source"].erase("loss_max");
        CHECK_THROWS_WITH(ps::io::parse_crop_json(b, "t"), ContainsSubstring("loss_max") && ContainsSubstring("source tag"));
    }
    {
        json b = j;
        b["pathogen"]["loss_max"] = 1.5;
        CHECK_THROWS_WITH(ps::io::parse_crop_json(b, "t"), ContainsSubstring("pathogen.loss_max"));
    }
    {
        json b = j;
        b["pathogen"]["suppression_max"] = 1.0;
        CHECK_THROWS_WITH(ps::io::parse_crop_json(b, "t"), ContainsSubstring("suppression_max"));
    }
    {
        json b = j;
        b["pathogen"]["extra"] = 1.0;
        CHECK_THROWS_WITH(ps::io::parse_crop_json(b, "t"), ContainsSubstring("extra"));
    }
    {
        json b = j;
        b["pathogen"]["agent"] = "";
        CHECK_THROWS_WITH(ps::io::parse_crop_json(b, "t"), ContainsSubstring("agent"));
    }
    json fj = load_json_file(tf::data_dir() / "herbicides/cereal_fungicide.json");
    {
        json b = fj;
        b["fungicide"]["efficacy_max"] = 1.0;   // a fungicide never buys back the whole loss
        CHECK_THROWS_WITH(ps::io::parse_herbicide_json(b, "t"), ContainsSubstring("fungicide_efficacy_max"));
    }
    {
        json b = fj;
        b["fungicide"]["source"].erase("dose_half");
        CHECK_THROWS_WITH(ps::io::parse_herbicide_json(b, "t"), ContainsSubstring("dose_half"));
    }

    // the field refuses incomplete configurations
    {
        FieldConfig cfg = tf::pathogen_config();
        cfg.management_enabled = false;
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("pathogens_enabled needs management_enabled"));
    }
    {
        FieldConfig cfg = tf::pathogen_config();
        cfg.crops[3].has_pathogen = false;
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("pathogen block in crop grain_maize"));
    }
    {
        FieldConfig cfg = tf::pathogen_config();
        cfg.crops[1].pathogen.decay_half_life_days *= 2.0;   // barley disagrees with wheat about one family's inoculum
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("equal pathogen half-lives"));
    }
    {
        FieldConfig cfg = tf::pathogen_config();
        cfg.crops[2].family.clear();
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("family"));
    }
    // the model owns the rotation factor
    Field f(tf::pathogen_config());
    CHECK_THROWS_AS(f.set_stress(0, StressFactors{1, 1, 1, 1, 0.9}), std::logic_error);
    CHECK_NOTHROW(f.set_stress(0, StressFactors{0.9, 1, 1, 1, 1}));
    CHECK_THROWS_AS(f.set_pathogen(0, "unknown", 0.1, 0.1), std::invalid_argument);
    CHECK_THROWS_AS(f.set_pathogen(0, "cereal", 1.0, 0.1), std::invalid_argument);
    CHECK(f.family_names() == std::vector<std::string>{"beet", "brassica", "cereal", "maize"});
    // and without the switch there is no pathogen state
    FieldConfig off = tf::pathogen_config();
    off.pathogens_enabled = false;
    Field g(off);
    CHECK_THROWS_AS(g.pathogen_view(0), std::logic_error);
    CHECK_THROWS_AS(g.set_pathogen(0, "cereal", 0.1, 0.1), std::logic_error);
}

TEST_CASE("P2 without a host the inoculum and the suppression decay with their half-lives", "[field][pathogen]") {
    const FieldConfig cfg = tf::pathogen_config();
    const PathogenDesc& pc = cfg.crops[0].pathogen;
    Field f(cfg);
    f.set_pathogen(0, "cereal", 0.6, 0.5);
    f.set_pathogen(0, "brassica", 0.3, 0.2);
    run_synth(f, 100);
    const PathogenView v = f.pathogen_view(0);   // family order: beet, brassica, cereal, maize
    CHECK_THAT(v.inoculum[2], WithinAbs(0.6 * std::exp(-100.0 * kLn2 / pc.decay_half_life_days), 1e-12));
    CHECK_THAT(v.suppression[2], WithinAbs(0.5 * std::exp(-100.0 * kLn2 / pc.suppression_half_life_days), 1e-12));
    const PathogenDesc& pr = cfg.crops[2].pathogen;
    CHECK_THAT(v.inoculum[1], WithinAbs(0.3 * std::exp(-100.0 * kLn2 / pr.decay_half_life_days), 1e-12));
    CHECK(v.inoculum[0] == 0.0);
    CHECK(v.inoculum[3] == 0.0);
    // the families differ: take-all decays faster than the resting spores of the brassica diseases
    CHECK(v.inoculum[2] / 0.6 < v.inoculum[1] / 0.3);

    // a crop of another family is no host: the cereal inoculum keeps decaying while maize grows (and maize builds its own)
    Field g(cfg);
    g.set_pathogen(0, "cereal", 0.6, 0.5);
    g.sow(0, 4);
    run_synth(g, 60, 20.0, 20.0);
    const PathogenView w = g.pathogen_view(0);
    CHECK_THAT(w.inoculum[2], WithinAbs(0.6 * std::exp(-60.0 * kLn2 / pc.decay_half_life_days), 1e-12));
    CHECK(w.inoculum[3] > 0.0);   // maize emerged and builds the maize inoculum
    CHECK(w.inoculum[2] < 0.6);
}

TEST_CASE("P3 a host crop closes the inoculum gap with its thermal time and builds the suppression", "[field][pathogen]") {
    const FieldConfig cfg = tf::pathogen_config();
    const CropDesc& cr = cfg.crops[0];
    const PathogenDesc& p = cr.pathogen;
    Field f(cfg);
    const double P0 = 0.2, S0 = 0.1;
    f.set_pathogen(0, "cereal", P0, S0);
    f.sow(0, 1);
    double P = P0, S = S0;
    const double dk = std::exp(-kLn2 / p.decay_half_life_days), sk = std::exp(-kLn2 / p.suppression_half_life_days);
    int host_days = 0;
    for (int i = 0; i < 90; ++i) {
        f.step_day(tf::synth(f.last_day() + 1, 15.0, 15.0, 2.0, 0.0, 3.0));
        const CellView c = f.view(0);
        const bool host = c.stage >= cr.stage_emergence && c.stage < cr.stage_maturity;
        if (host) {
            ++host_days;
            const double x = 15.0 / p.season_gdd;   // Tmin = Tmax = 15, base 0: 15 gdd a day
            P += (1.0 - P) * (1.0 - std::exp(-p.build_per_season * x));
            S += (1.0 - S) * (1.0 - std::exp(-p.suppression_gain * P * x));
        } else {
            P *= dk;
        }
        S *= sk;
        const PathogenView v = f.pathogen_view(0);
        REQUIRE_THAT(v.inoculum[2], WithinAbs(P, 1e-12));
        REQUIRE_THAT(v.suppression[2], WithinAbs(S, 1e-12));
    }
    CHECK(host_days > 60);
    CHECK(P > P0 + 0.02);   // it did build
    CHECK(S > S0 * std::pow(sk, 90) + 1e-4);
}

TEST_CASE("P4 the loss is fixed at emergence and acts through stress_rotation", "[field][pathogen]") {
    const FieldConfig cfg = tf::pathogen_config();
    const CropDesc& cr = cfg.crops[0];
    const PathogenDesc& p = cr.pathogen;
    FieldConfig off_cfg = cfg;
    off_cfg.pathogens_enabled = false;
    Field on(cfg), off(off_cfg);
    on.set_pathogen(0, "cereal", 0.5, 0.4);
    on.sow(0, 1);
    off.sow(0, 1);
    double P_before = 0.5, S_before = 0.4, expected_loss = -1.0, early_ratio = 0.0;
    int days_after = 0;
    for (int i = 0; i < 110; ++i) {
        const PathogenView pv = on.pathogen_view(0);
        P_before = pv.inoculum[2];
        S_before = pv.suppression[2];
        const DailyWeather w = tf::synth(on.last_day() + 1, 15.0, 15.0, 2.0, 0.0, 3.0);
        on.step_day(w);
        off.step_day(w);
        if (expected_loss < 0.0 && on.view(0).stage >= cr.stage_emergence) expected_loss = p.loss_max * P_before * (1.0 - p.suppression_max * S_before);
        if (expected_loss >= 0.0) {
            if (++days_after == 4) early_ratio = on.view(0).biomass_g_m2 / off.view(0).biomass_g_m2;
            const PathogenView v = on.pathogen_view(0);
            REQUIRE_THAT(v.loss_untreated, WithinAbs(expected_loss, 1e-12));   // fixed, although P keeps changing
            REQUIRE_THAT(v.growth_factor, WithinAbs(1.0 - expected_loss, 1e-12));
        } else {
            CHECK(on.pathogen_view(0).growth_factor == 1.0);   // nothing before emergence
        }
    }
    REQUIRE(expected_loss > 0.05);
    // the pathogen field grows (1 - loss) of its twin: stress_rotation multiplies the daily gain and nothing else differs (read
    // in the first days after emergence, before the lower biomass can change the nitrogen status through the dilution curve)
    REQUIRE(early_ratio > 0.0);
    CHECK_THAT(early_ratio, WithinAbs(1.0 - expected_loss, 1e-9));
    // later the smaller crop needs less nitrogen, so its ratio drifts up, but never above 1
    const double b_on = on.view(0).biomass_g_m2, b_off = off.view(0).biomass_g_m2;
    REQUIRE(b_off > 50.0);
    CHECK(b_on / b_off >= 1.0 - expected_loss);
    CHECK(b_on / b_off < 1.0);
    // with no inoculum the two fields are identical
    Field clean(cfg);
    Field twin(off_cfg);
    clean.sow(0, 1);
    twin.sow(0, 1);
    for (int i = 0; i < 110; ++i) {
        const DailyWeather w = tf::synth(clean.last_day() + 1, 15.0, 15.0, 2.0, 0.0, 3.0);
        clean.step_day(w);
        twin.step_day(w);
    }
    CHECK(clean.pathogen_view(0).loss_untreated == 0.0);
    CHECK(clean.view(0).biomass_g_m2 == twin.view(0).biomass_g_m2);
    // suppression lowers the loss at the same inoculum (take-all decline)
    Field s0(cfg), s1(cfg);
    s0.set_pathogen(0, "cereal", 0.5, 0.0);
    s1.set_pathogen(0, "cereal", 0.5, 0.8);
    s0.sow(0, 1);
    s1.sow(0, 1);
    run_synth(s0, 40);
    run_synth(s1, 40);
    CHECK(s1.pathogen_view(0).loss_untreated < 0.5 * s0.pathogen_view(0).loss_untreated);
}

TEST_CASE("P5 surface residue raises the inoculum, ploughing it under removes most of that", "[field][pathogen]") {
    DailyWeatherSource& src = tf::weather30();
    auto season = [&](const FieldConfig& base, const char* tillage) {
        FieldConfig cfg = base;
        cfg.initial_day = tf::find_doy(src, 365, 288) - 1;
        Field f(cfg);
        f.sow(0, 1);
        run_to(f, src, tf::find_doy(src, f.last_day(), 212) - 1);
        f.apply(0, op(cfg, "combine_straw_chopped"));   // harvest: straw and stubble on the surface, res_crop = wheat
        f.set_pathogen(0, "cereal", 0.0, 0.0);         // only the residue can build from here
        if (tillage) f.apply(0, op(cfg, tillage));
        run_to(f, src, f.last_day() + 90);
        return f.pathogen_view(0).inoculum[2];
    };
    const FieldConfig cfg = tf::pathogen_config();
    const double none = season(cfg, nullptr);
    const double plough = season(cfg, "plough");
    FieldConfig zero = cfg;
    for (CropDesc& c : zero.crops) c.pathogen.residue_source_per_day = 0.0;
    const double no_source = season(zero, nullptr);
    std::printf("P5 inoculum 90 d after harvest from the residue alone: stubble left %.4f, ploughed %.4f, source 0 %.4f\n", none, plough, no_source);
    CHECK(none > 0.01);
    CHECK(plough < 0.4 * none);
    CHECK(plough >= 0.0);
    CHECK(no_source == 0.0);
}

TEST_CASE("P6 a fungicide buys back part of the loss and never all of it", "[field][pathogen]") {
    const FieldConfig cfg = tf::pathogen_config();
    const HerbicideDesc& fd = cfg.herbicides[0];
    const double p1 = fd.fungicide_efficacy_max * 1.0 / (1.0 + fd.fungicide_dose_half);
    // protection of one label spray, its decay, the cap
    Field f(cfg);
    f.sow(0, 1);
    f.apply(0, tf::fungicide_spray(1.0));
    CHECK_THAT(f.pathogen_view(0).fungicide_protection, WithinAbs(p1, 1e-12));
    run_synth(f, static_cast<int>(fd.fungicide_half_life_days));
    CHECK_THAT(f.pathogen_view(0).fungicide_protection, WithinAbs(p1 * 0.5, 1e-9));
    Field g(cfg);
    g.sow(0, 1);
    g.apply(0, tf::fungicide_spray(1.0));
    g.apply(0, tf::fungicide_spray(1.0));
    CHECK_THAT(g.pathogen_view(0).fungicide_protection, WithinAbs(std::min(fd.fungicide_efficacy_max, 1.0 - (1.0 - p1) * (1.0 - p1)), 1e-12));
    g.apply(0, tf::fungicide_spray(1000.0));
    CHECK(g.pathogen_view(0).fungicide_protection <= fd.fungicide_efficacy_max + 1e-15);
    CHECK(g.pathogen_view(0).fungicide_protection < 1.0);
    // bare soil: nothing to protect, and nothing carried to the next crop
    Field h(cfg);
    h.apply(0, tf::fungicide_spray(1.0));
    h.sow(0, 1);
    CHECK(h.pathogen_view(0).fungicide_protection == 0.0);
    // another family: maize is not protected by the cereal product
    Field m(cfg);
    m.sow(0, 4);
    m.apply(0, tf::fungicide_spray(1.0));
    CHECK(m.pathogen_view(0).fungicide_protection == 0.0);
    // barley is of the cereal family: protected
    Field b(cfg);
    b.sow(0, 2);
    b.apply(0, tf::fungicide_spray(1.0));
    CHECK_THAT(b.pathogen_view(0).fungicide_protection, WithinAbs(p1, 1e-12));

    // the loss: sprayed field vs unsprayed vs clean; the share bought back is partial
    FieldConfig off_cfg = cfg;
    off_cfg.pathogens_enabled = false;
    Field sprayed(cfg), plain(cfg), clean(off_cfg);
    for (Field* x : {&sprayed, &plain}) {
        x->set_pathogen(0, "cereal", 0.6, 0.0);
        x->sow(0, 1);
    }
    clean.sow(0, 1);
    sprayed.apply(0, tf::fungicide_spray(1.0));
    for (int i = 0; i < 150; ++i) {
        const DailyWeather w = tf::synth(sprayed.last_day() + 1, 15.0, 15.0, 2.0, 0.0, 3.0);
        sprayed.step_day(w);
        plain.step_day(w);
        clean.step_day(w);
    }
    const double bs = sprayed.view(0).biomass_g_m2, bp = plain.view(0).biomass_g_m2, bc = clean.view(0).biomass_g_m2;
    REQUIRE(bc > bp);
    const double share = (bs - bp) / (bc - bp);
    std::printf("P6 biomass clean %.1f, unsprayed %.1f, sprayed %.1f g/m2: %.0f %% of the loss bought back\n", bc, bp, bs, 100.0 * share);
    CHECK(share > 0.15);
    CHECK(share < fd.fungicide_efficacy_max);   // the whole loss never comes back
    CHECK(bs < bc);
    // the protection also slows the build of the next season's inoculum
    CHECK(sprayed.pathogen_view(0).inoculum[2] < plain.pathogen_view(0).inoculum[2]);
}

TEST_CASE("P7 determinism: partitions, checkpoint and lazy jumps give the same bits with pathogens on", "[field][pathogen][determinism]") {
    DailyWeatherSource& src = tf::weather30();
    FieldConfig cfg = tf::pathogen_config(4, 4, 0.2);
    cfg.initial_day = tf::find_doy(src, 365, 270) - 1;
    const OperationEffect harvest = op(cfg, "combine_straw_chopped"), fung = tf::fungicide_spray(1.0);
    auto events = [&](Field& f, int i) {
        if (i == 0) {
            for (int c = 0; c < 16; ++c) {
                f.sow(c, c < 8 ? 1 : (c < 13 ? 2 : c - 10));   // wheat | barley (cells 7 and 8 neighbours of one family) | rapeseed, maize, beet
                f.set_pathogen(c, "cereal", 0.05 + 0.05 * (c % 7), 0.03 * (c % 5));
                f.set_pathogen(c, "brassica", 0.04 * (c % 4), 0.0);
            }
        }
        if (i == 40) f.apply_all(fung);
        if (i == 330) f.apply_all(harvest);
    };
    const int total = 400;
    auto drive = [&](Field& f, int mode) {
        for (int i = 0; i < total; ++i) {
            events(f, i);
            const DailyWeather& w = src.record(f.last_day() + 1);
            if (mode == 0) {
                f.step_day(w);
            } else {
                f.step_cells(8, 16, w);
                f.step_cells(0, 8, w);
                f.finish_day(w.day);
            }
        }
    };
    Field a(cfg), b(cfg);
    drive(a, 0);
    drive(b, 1);
    const std::uint64_t h0 = 1469598103934665603ull;
    CHECK(a.fold_hash(h0) == b.fold_hash(h0));
    // the pathogen state is part of the hash: a changed inoculum changes it
    Field c(cfg), d(cfg), e2(cfg);
    drive(c, 0);
    drive(d, 0);
    drive(e2, 0);
    const std::uint64_t hc = c.fold_hash(h0);
    CHECK(hc == d.fold_hash(h0));
    d.set_pathogen(5, "cereal", 0.123, d.pathogen_view(5).suppression[2]);   // the inoculum alone
    CHECK(hc != d.fold_hash(h0));
    e2.set_pathogen(5, "cereal", e2.pathogen_view(5).inoculum[2], 0.5);      // the suppression alone
    CHECK(hc != e2.fold_hash(h0));
    // checkpoint in the middle
    {
        Field e(cfg);
        FieldCheckpoint cp;
        for (int i = 0; i < total; ++i) {
            if (i == 200) cp = e.checkpoint();
            events(e, i);
            e.step_day(src.record(e.last_day() + 1));
        }
        Field r(cfg);
        r.restore(cp);
        for (int i = 200; i < total; ++i) {
            events(r, i);
            r.step_day(src.record(r.last_day() + 1));
        }
        CHECK(e.fold_hash(h0) == r.fold_hash(h0));
        CHECK(a.fold_hash(h0) == e.fold_hash(h0));
    }
    // lazy jumps between the event days
    {
        Field j(cfg);
        for (int i = 0; i < total; ++i)
            if (i == 0 || i == 40 || i == 330) {
                j.advance_to(cfg.initial_day + i, src);
                events(j, i);
            }
        j.advance_to(cfg.initial_day + total, src);
        CHECK(a.fold_hash(h0) == j.fold_hash(h0));
    }
}

// ---- P8: the published margins -------------------------------------------------------------------------

namespace {

struct Plan {
    double dap, mop;
    int sow, harvest;
    bool winter;
    std::vector<std::pair<int, int>> n;
};
const std::map<std::string, Plan>& plans() {
    static const std::map<std::string, Plan> p = {
        {"winter_wheat", {0.15, 0.17, 288, 212, true, {{95, 60}, {130, 50}, {160, 40}}}},
        {"winter_barley", {0.15, 0.17, 275, 196, true, {{60, 50}, {95, 60}, {125, 40}}}},
        {"winter_rapeseed", {0.16, 0.17, 235, 208, true, {{50, 80}, {80, 60}, {110, 40}}}},
    };
    return p;
}

// Rotation scenario on one cell of the deep loess: crop `seq[k]` is harvested in year k + 1, k = 0.. cyclic.
json rotation_json(const std::vector<std::string>& seq, int years, bool pathogens) {
    std::vector<std::string> names;
    for (const std::string& s : seq)
        if (std::find(names.begin(), names.end(), s) == names.end()) names.push_back(s);
    json ops = json::array();
    const auto d = tf::data_dir();
    ops.push_back({{"op", "apply"}, {"material", "lime_carbonate_90"}, {"method", "incorporated"}, {"rate_t_ha", 4.0}, {"doy", 250}, {"every_years", 4}});
    const int L = static_cast<int>(seq.size());
    for (int i = 0; i < L; ++i) {
        const Plan& p = plans().at(seq[static_cast<std::size_t>(i)]);
        const int hy = i + 1, sy = hy - 1;
        ops.push_back({{"op", "apply"}, {"material", "dap_18_46"}, {"method", "broadcast"}, {"rate_t_ha", p.dap}, {"doy", 262}, {"first_year", sy}, {"every_years", L}});
        ops.push_back({{"op", "apply"}, {"material", "mop_60"}, {"method", "broadcast"}, {"rate_t_ha", p.mop}, {"doy", 262}, {"first_year", sy}, {"every_years", L}});
        ops.push_back({{"op", "sow"}, {"crop", seq[static_cast<std::size_t>(i)]}, {"doy", p.sow}, {"first_year", sy}, {"every_years", L}});
        for (const auto& dn : p.n)
            ops.push_back({{"op", "apply"}, {"material", "can_27"}, {"method", "broadcast"}, {"rate_n_kg_ha", dn.second}, {"doy", dn.first}, {"first_year", hy}, {"every_years", L}});
        ops.push_back({{"op", "harvest"}, {"doy", p.harvest}, {"first_year", hy}, {"every_years", L}, {"straw_removed", 0.0}});
    }
    json crops = json::array();
    for (const std::string& n : names) crops.push_back((d / "crops" / (n + ".json")).generic_string());
    return {{"format", "physics_sim.field_scenario/1"},
            {"climate", (d / "climates/farm_de_home.json").generic_string()},
            {"seed", 21},
            {"years", years + 1},
            {"nutrients", true},
            {"management", true},
            {"pathogens", pathogens},
            {"soils", json::array({(d / "soils/silt_loam_loess.json").generic_string()})},
            {"crops", crops},
            {"materials", json::array({(d / "field_materials/can_27.json").generic_string(), (d / "field_materials/dap_18_46.json").generic_string(),
                                       (d / "field_materials/mop_60.json").generic_string(), (d / "field_materials/lime_carbonate_90.json").generic_string()})},
            {"methods", (d / "field_materials/methods.json").generic_string()},
            {"grid", {{"nx", 1}, {"ny", 1}, {"cell_size_m", 4.0}}},
            {"initial_depletion_fraction", 0.2},
            {"co2_ppm", 400.0},
            {"operations", ops}};
}

// grain dry matter by harvest year
std::map<int, double> run_rotation(const std::vector<std::string>& seq, int years, bool pathogens) {
    static int counter = 0;
    const auto path = std::filesystem::temp_directory_path() / ("ps_field_p8_" + std::to_string(++counter) + "_" + std::to_string(std::hash<std::string>{}(seq.front() + std::to_string(years) + (pathogens ? "p" : "n") + std::to_string(seq.size()))) + ".json");
    {
        std::ofstream out(path);
        out << rotation_json(seq, years, pathogens).dump();
    }
    const ps::io::FieldScenario sc = ps::io::load_field_scenario(path);
    std::filesystem::remove(path);
    const ps::io::FieldRunSummary sum = ps::io::run_field_scenario(sc);
    std::map<int, double> y;
    for (const auto& h : sum.harvests) y[h.year] = h.result.grain_dm_t_ha;
    return y;
}

}  // namespace

TEST_CASE("P8a take-all margins: second wheat 10-15 % below first wheat after a break, third wheat 10-20 %, peak year 3-5, then decline", "[field][pathogen][long]") {
    // Sources: AHDB take-all guidance and Farmers Weekly (second wheat 10-15 % below first wheat; 10-20 % common in second and
    // third wheat); Rothamsted take-all decline as summarised on Wikipedia 'Take-all' (peak in the 3rd to 5th cropping year,
    // then decline to 80-90 % of the early yields; not checked against the Rothamsted primary source).
    const int N = 16;
    const auto cont_on = run_rotation({"winter_wheat"}, N, true), cont_off = run_rotation({"winter_wheat"}, N, false);
    // four wheats, a rapeseed year, wheat again (cycle of 6 harvest years: W W W W R W): the wheat right after the break
    const std::vector<std::string> cyc = {"winter_wheat", "winter_wheat", "winter_wheat", "winter_wheat", "winter_rapeseed", "winter_wheat"};
    const auto rot_on = run_rotation(cyc, N, true), rot_off = run_rotation(cyc, N, false);
    auto ratio = [](const std::map<int, double>& on, const std::map<int, double>& off, int y) { return on.at(y) / off.at(y); };
    // first wheat after a break: harvest years 6 and 12 of the cycle (the sixth slot); both after a rapeseed year
    const double r1 = 0.5 * (ratio(rot_on, rot_off, 6) + ratio(rot_on, rot_off, 12));
    const double r2 = ratio(cont_on, cont_off, 2), r3 = ratio(cont_on, cont_off, 3), r4 = ratio(cont_on, cont_off, 4);
    double late = 0.0;
    for (int y = 11; y <= N; ++y) late += ratio(cont_on, cont_off, y) / (N - 10);
    std::printf("P8a on/off yield ratios: first wheat after a break %.3f; continuous: 2nd %.3f, 3rd %.3f, 4th %.3f, years 11-16 mean %.3f\n", r1, r2, r3, r4, late);
    CHECK(ratio(cont_on, cont_off, 1) == 1.0);          // a clean field has no inoculum in its first year
    CHECK(r1 > 0.93);                                    // wheat after a break meets little
    CHECK(r2 / r1 >= 0.85);                              // second wheat 10-15 % below first wheat
    CHECK(r2 / r1 <= 0.90);
    CHECK(r3 / r1 >= 0.80);                              // third wheat 10-20 %
    CHECK(r3 / r1 <= 0.90);
    // the worst years are 3-5 (cropping year 1 clean, so the loss of cropping year k is read at harvest year k)
    int worst = 2;
    for (int y = 2; y <= N; ++y)
        if (ratio(cont_on, cont_off, y) < ratio(cont_on, cont_off, worst)) worst = y;
    CHECK(worst >= 3);
    CHECK(worst <= 5);
    // decline: later years are better than the worst, still not back to the level of first wheat
    CHECK(late > ratio(cont_on, cont_off, worst) + 0.03);
    CHECK(late / r1 >= 0.80);
    CHECK(late / r1 <= 0.95);
}

TEST_CASE("P8b rapeseed: a 4-year break yields 8-15 % more than rapeseed every third year", "[field][pathogen][long]") {
    // topagrar 'Rapsanbau in Deutschland: weite Fruchtfolge': 8-15 % more with a 4-year break (rapeseed every fifth year) than with
    // rapeseed every third year (another passage of the same article gives 4-7 %).
    const int N = 40;
    const std::vector<std::string> r3 = {"winter_rapeseed", "winter_wheat", "winter_barley"};
    const std::vector<std::string> r5 = {"winter_rapeseed", "winter_wheat", "winter_barley", "winter_wheat", "winter_barley"};
    auto rape_ratio = [&](const std::vector<std::string>& s, int L) {
        const auto on = run_rotation(s, N, true), off = run_rotation(s, N, false);
        double sum = 0.0;
        int n = 0;
        for (int y = 1 + 2 * L; y <= N; y += L) {   // rapeseed is harvested in years 1, 1 + L, ...; skip the first two cycles
            sum += on.at(y) / off.at(y);
            ++n;
        }
        return sum / n;
    };
    const double a3 = rape_ratio(r3, 3), a5 = rape_ratio(r5, 5);
    std::printf("P8b rapeseed on/off yield ratio: every 3rd year %.3f, every 5th year %.3f, 5th/3rd %.3f\n", a3, a5, a5 / a3);
    CHECK(a5 / a3 >= 1.08);
    CHECK(a5 / a3 <= 1.15);
    // rapeseed after rapeseed (every year) is much worse still
    const auto on1 = run_rotation({"winter_rapeseed"}, 12, true), off1 = run_rotation({"winter_rapeseed"}, 12, false);
    CHECK(on1.at(12) / off1.at(12) < a3 - 0.15);
}
