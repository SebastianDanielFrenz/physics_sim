// F1d part 5: soil organic carbon - humus that decays and mineralises N, is built from residue, and sets the water capacity.
//
// What the tests pin down (the model is in core/src/field/soc.cpp and the soc branches of nutrients.cpp / management.cpp,
// the numbers in data/soils/*.json "organic_matter"):
//  S1  data and loaders: both soils carry the block with source tags, every rejection message, soc_enabled needs
//      management and the block, the humification must fit inside what the microbes assimilate;
//  S2  decay: the carbon of the humus follows its nitrogen exactly (C:N constant, N mineralised = N lost), and the
//      logarithmic decay is proportional to rate_per_day_20c;
//  S3  humification: of the residue carbon decomposed, humification x dC reaches the humus and the rest is respired
//      (ledger identity to rounding); both ledgers close, the C ledger with the humus as a stock;
//  S4  water: the capacity follows the carbon content by awc_per_soc_pct, the stored water does not change when it
//      does, a full profile that loses capacity drains the difference, the water ledger closes;
//  S5  residue raises the humus (straw left vs straw removed, same weather and fertiliser);
//  S6  organic material applied joins the humus with the carbon of its slow organic N at the humus C:N;
//  S7  determinism: partitions, checkpoint/restore, lazy jumps; a checkpoint of a soc-less field is refused;
//  S8  (tag [long]) 30 years of continuous wheat with the straw removed against the straw left: the removed-straw field
//      loses humus, the difference to straw incorporation is inside the published range.
//
// Recorded sabotages (made in the source, this file run, reverted; 2026-10-31; cases that failed in brackets):
//  - SA-a: the carbon of the humus not decaying with the N (hum_c untouched in the mineralisation): [S2 S3 S4 S5 S6 S8];
//  - SA-b: humification 0.4 x dC (the whole assimilated share) instead of o.humification: [S3 S5 S8];
//  - SA-c: the depletions not following the capacity (step_soc without the d_root/d_sub shift): [S3 S4 S8] (storage moves);
//  - SA-d: the drainage of the lost capacity not booked: [S3 S4 S8] (the water residual);
//  - SA-e: the amendment carbon not added in apply_deposit: [S6];
//  - SA-f: soc_dfc left out of the hash: [S7] (the changed-state check);
//  - SA-g: the slow pool decaying with the nutrient block's rate, not the organic block's: [S2 S8].
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "field_test_support.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::field;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
namespace tf = ps::test_field;

namespace {

FieldConfig soc_config(int nx = 1, int ny = 1, double depletion = 0.2) {
    FieldConfig cfg = tf::nutrient_config(tf::loess(), tf::wheat(), nx, ny);
    cfg.initial_depletion_fraction = depletion;
    cfg.management_enabled = true;
    cfg.soc_enabled = true;
    return cfg;
}

}  // namespace

TEST_CASE("S1 the organic_matter data and its loaders", "[field][soc]") {
    for (const SoilDesc* s : {&tf::loess(), &tf::sandy()}) {
        INFO(s->name);
        CHECK(s->has_organic);
        CHECK(soil_validation_error(*s).empty());
        CHECK(s->organic.humification == 0.125);
        CHECK(s->organic.awc_per_soc_pct == 0.0116);
        CHECK(s->organic.humus_cn == 10.0);
        CHECK(s->organic.rate_per_day_20c > 0.0);
        // fits inside what the microbes assimilate
        CHECK(s->organic.humification <= s->management.res_assim_eff);
        CHECK(s->organic.humification / s->organic.humus_cn <= s->management.res_assim_eff / s->management.res_cn_bio);
    }
    // the shipped files carry a source tag per parameter (the loader refuses a number without one)
    {
        SoilDesc bad = tf::loess();
        bad.organic.humification = 1.5;
        CHECK_THAT(soil_validation_error(bad), ContainsSubstring("humification must be in [0, 1]"));
        bad = tf::loess();
        bad.organic.rate_per_day_20c = 0.0;
        CHECK_THAT(soil_validation_error(bad), ContainsSubstring("rate_per_day_20c must be a finite number > 0"));
        bad = tf::loess();
        bad.organic.bulk_density_t_m3 = 3.5;
        CHECK_THAT(soil_validation_error(bad), ContainsSubstring("bulk_density_t_m3 must be below 3"));
        bad = tf::loess();
        bad.organic.awc_per_soc_pct = -0.1;
        CHECK_THAT(soil_validation_error(bad), ContainsSubstring("awc_per_soc_pct"));
    }
    // configuration rejections
    {
        FieldConfig cfg = soc_config();
        cfg.management_enabled = false;
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("soc_enabled needs management_enabled"));
        cfg = soc_config();
        cfg.soils[0].has_organic = false;
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("needs an organic_matter block"));
        cfg = soc_config();
        cfg.soils[0].organic.humification = 0.5;   // above res_assim_eff 0.4
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("humification exceeds res_assim_eff"));
        cfg = soc_config();
        cfg.soils[0].organic.humus_cn = 2.5;   // humus nitrogen above what the microbes immobilise
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("humus nitrogen"));
        cfg = soc_config();
        cfg.soils[0].organic.layer_depth_m = 5.0;
        CHECK_THROWS_WITH(Field(cfg), ContainsSubstring("layer_depth_m"));
        Field off(tf::nutrient_config(tf::loess(), tf::wheat()));
        CHECK_THROWS_AS(off.soc_view(0), std::logic_error);
    }
    // initial humus: slow organic N x C:N, in % of the layer mass
    Field f(soc_config());
    const SocView v = f.soc_view(0);
    CHECK_THAT(v.hum_c_kg_ha, WithinRel(5850.0 * 10.0, 1e-12));
    CHECK_THAT(v.soc_pct, WithinAbs(100.0 * 58500.0 / 3.9e6, 1e-9));
    CHECK(v.dfc == 0.0);
    CHECK_THAT(v.cn, WithinAbs(10.0, 1e-12));
}

TEST_CASE("S2 humus decays with its nitrogen: C:N constant, N lost = N mineralised, rate proportional", "[field][soc]") {
    auto run = [](double rate, double awc) {
        FieldConfig cfg = soc_config();
        cfg.soils[0].organic.rate_per_day_20c = rate;
        cfg.soils[0].organic.awc_per_soc_pct = awc;
        Field f(cfg);
        const double n0 = f.nutrient_view(0).org_slow, c0 = f.soc_view(0).hum_c_kg_ha;
        tf::run_synth(f, 300, 12.0, 18.0, 2.0, 3.0);
        struct R { double n0, c0, n1, c1, cn, nmin; } r;
        r.n0 = n0;
        r.c0 = c0;
        r.n1 = f.nutrient_view(0).org_slow;
        r.c1 = f.soc_view(0).hum_c_kg_ha;
        r.cn = f.soc_view(0).cn;
        r.nmin = f.nutrient_ledger(0).n_mineralised;
        CHECK(std::fabs(f.n_balance_residual(0)) < 1e-9);
        CHECK(std::fabs(f.c_balance_residual(0)) < 1e-9);
        return r;
    };
    const auto a = run(4.2e-5, 0.0);
    CHECK_THAT(a.cn, WithinAbs(10.0, 1e-9));
    CHECK(a.c1 < a.c0);
    CHECK_THAT(a.nmin, WithinAbs(a.n0 - a.n1, 1e-9));              // all the N that left the humus was mineralised (no fresh pool, no residue)
    CHECK_THAT((a.c0 - a.c1) / 10.0, WithinAbs(a.n0 - a.n1, 1e-9));  // ... and the carbon that left is that N times the C:N
    const auto b = run(8.4e-5, 0.0);
    const double la = -std::log(a.c1 / a.c0), lb = -std::log(b.c1 / b.c0);
    CHECK_THAT(lb / la, WithinAbs(2.0, 1e-9));   // log-decay is linear in the rate
    // the rate of the organic block, not the nutrient block's (1.15e-4), drives the decay
    const auto c = run(1.15e-4, 0.0);
    CHECK_THAT(-std::log(c.c1 / c.c0) / la, WithinAbs(1.15e-4 / 4.2e-5, 1e-9));
}

TEST_CASE("S3 humification: dH = humification x decomposed residue carbon, the rest is respired, ledgers close", "[field][soc]") {
    FieldConfig cfg = soc_config();
    Field f(cfg);
    tf::Plan p;
    p.n_kg = 160.0;
    p.p2o5_kg = 60.0;
    p.k2o_kg = 80.0;
    p.straw_removed = 0.0;   // straw chopped on the field
    p.lime_every = 4;
    p.lime_t = 2.0;
    DailyWeatherSource& src = tf::weather30();
    for (std::int64_t d = 0; d < 4 * 365; ++d) {
        const DailyWeather& w = src.record(d);
        tf::apply_plan_day(f, w, p, nullptr);
        f.step_day(w);
    }
    const CarbonLedger l = f.carbon_ledger(0);
    const ManagementView m = f.management_view(0);
    const double decomposed = l.c_from_crop - m.residue_c_kg_ha - l.c_removed;   // no weeds in this field
    CHECK(decomposed > 3000.0);   // straw, stubble and roots of three harvests
    const double h = tf::loess().organic.humification;
    CHECK_THAT(l.c_humus, WithinAbs(h * decomposed, 1e-8));
    CHECK_THAT(l.c_co2, WithinAbs((1.0 - h) * decomposed, 1e-8));
    CHECK(l.c_humus_decay > 0.0);
    CHECK(std::fabs(f.c_balance_residual(0)) < 1e-9);
    CHECK(std::fabs(f.n_balance_residual(0)) < 1e-9);
    CHECK(std::fabs(f.water_balance_residual_mm(0)) < 1e-9);
    // the humus nitrogen came with the carbon at the humus C:N
    CHECK_THAT(f.soc_view(0).cn, WithinAbs(10.0, 1e-6));
}

TEST_CASE("S4 the water capacity follows the humus carbon; the stored water stays; the lost capacity drains", "[field][soc]") {
    // no rain, no evaporation: the only thing that can move the stored water is the capacity change
    for (const double depletion : {0.3, 0.0}) {
        INFO("initial depletion " << depletion);
        FieldConfig cfg = soc_config(1, 1, depletion);
        cfg.soils[0].organic.rate_per_day_20c = 4.0e-4;   // fast enough for a visible change in 250 days (test only)
        Field f(cfg);
        const double s0 = f.storage_mm(0), taw0 = f.soc_view(0).taw_total_mm;
        tf::run_synth(f, 250, 12.0, 18.0, 0.0, 0.0);
        const SocView v = f.soc_view(0);
        const SoilDesc& so = cfg.soils[0];
        const double expect = so.organic.awc_per_soc_pct * (v.soc_pct - v.soc_pct_initial) * (so.organic.layer_depth_m / so.depth_m);
        CHECK(v.soc_pct < v.soc_pct_initial);
        CHECK_THAT(v.dfc, WithinAbs(expect, 1e-12));
        CHECK_THAT(v.taw_total_mm - taw0, WithinAbs(1000.0 * v.dfc * so.depth_m, 1e-9));
        CHECK(v.taw_total_mm < taw0);
        const double lost_capacity = taw0 - v.taw_total_mm;
        if (depletion > 0.0) {   // a part-empty profile: the water it holds does not change
            CHECK_THAT(f.storage_mm(0), WithinAbs(s0, 1e-9));
            CHECK(f.ledger(0).drainage == 0.0);
        } else {                 // a full profile cannot hold the lost capacity: it drains
            CHECK_THAT(f.ledger(0).drainage, WithinAbs(lost_capacity, 1e-9));
            CHECK_THAT(f.storage_mm(0), WithinAbs(s0 - lost_capacity, 1e-9));
        }
        CHECK(std::fabs(f.water_balance_residual_mm(0)) < 1e-9);
        CHECK(lost_capacity > 0.03);   // a real effect, not rounding
        // the root-zone capacity the crop sees is the one of the changed soil
        CHECK_THAT(f.taw_root_mm(0), WithinAbs((so.taw_mm_per_m() + 1000.0 * v.dfc) * f.view(0).zr_m, 1e-9));
    }
    // the effect needs the coefficient: awc 0 leaves the soil file's water
    {
        FieldConfig cfg = soc_config(1, 1, 0.3);
        cfg.soils[0].organic.rate_per_day_20c = 4.0e-4;
        cfg.soils[0].organic.awc_per_soc_pct = 0.0;
        Field f(cfg);
        tf::run_synth(f, 250, 12.0, 18.0, 0.0, 0.0);
        CHECK(f.soc_view(0).dfc == 0.0);
        CHECK(f.soc_view(0).taw_total_mm == cfg.soils[0].total_taw_mm());
    }
}

TEST_CASE("S5 residue raises the humus: straw left against straw removed", "[field][soc]") {
    auto run = [](double straw_removed) {
        Field f(soc_config());
        tf::Plan p;
        p.n_kg = 160.0;
        p.p2o5_kg = 60.0;
        p.k2o_kg = 80.0;
        p.straw_removed = straw_removed;
        p.lime_every = 4;
        p.lime_t = 2.0;
        const auto yrs = tf::run_plan(f, p, 8);
        struct R { double hum, nslow; std::size_t n; } r{f.soc_view(0).hum_c_kg_ha, f.nutrient_view(0).org_slow, yrs.size()};
        return r;
    };
    const auto left = run(0.0), removed = run(1.0);
    INFO("humus after 8 years: straw left " << left.hum << ", removed " << removed.hum << " kg C/ha");
    CHECK(left.hum > removed.hum);
    const double gain = left.hum - removed.hum;
    CHECK(gain > 500.0);    // ~ 0.125 x 2 t C/yr of straw x 7 harvests, less decay
    CHECK(gain < 3000.0);
    CHECK(left.nslow > removed.nslow);   // the humus nitrogen follows the carbon
    CHECK_THAT((left.hum - removed.hum) / (left.nslow - removed.nslow), WithinAbs(10.0, 1e-6));
}

TEST_CASE("S6 organic material joins the humus with the carbon of its slow nitrogen", "[field][soc]") {
    Field f(soc_config());
    const double c0 = f.soc_view(0).hum_c_kg_ha, n0 = f.nutrient_view(0).org_slow;
    const int mat = tf::mat_id("cattle_solid_manure");
    const MaterialDesc& m = tf::materials()[static_cast<std::size_t>(mat - 1)];
    const double rate = 30.0;
    f.apply(0, OperationEffect(1.0, tf::deposit("cattle_solid_manure", "broadcast", rate)));
    const double org_n = m.org_n_kg_t() * rate;
    const double dn = f.nutrient_view(0).org_slow - n0, dc = f.soc_view(0).hum_c_kg_ha - c0;
    CHECK_THAT(dn, WithinAbs(org_n * m.org_slow_fraction, 1e-9));
    CHECK_THAT(dc, WithinAbs(dn * 10.0, 1e-9));
    CHECK_THAT(f.carbon_ledger(0).c_amendment, WithinAbs(dc, 1e-9));
    CHECK(dc > 100.0);
    CHECK(std::fabs(f.c_balance_residual(0)) < 1e-9);
    CHECK(std::fabs(f.n_balance_residual(0)) < 1e-9);
    tf::run_synth(f, 200, 12.0, 18.0, 2.0, 3.0);
    CHECK(std::fabs(f.c_balance_residual(0)) < 1e-9);
    CHECK(std::fabs(f.n_balance_residual(0)) < 1e-9);
}

TEST_CASE("S7 determinism: partitions, checkpoint and lazy jumps give the same bits with soc on", "[field][soc][determinism]") {
    DailyWeatherSource& src = tf::weather30();
    FieldConfig cfg = soc_config(4, 4, 0.2);
    cfg.initial_day = tf::find_doy(src, 365, 270) - 1;
    auto events = [&](Field& f, int i) {
        if (i == 0)
            for (int c = 0; c < 16; ++c) f.sow(c, 1);
        if (i == 30) f.apply_all(OperationEffect(1.0, tf::deposit("cattle_solid_manure", "broadcast", 20.0)));
        if (i == 330) f.apply_all([] { OperationEffect e; e.harvest.harvest = true; e.harvest.straw_chopped = 1.0; return e; }());
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
    CHECK(a.soc_view(0).dfc != 0.0);
    // the capacity shift is part of the hash
    {
        Field c(cfg), d(cfg);
        drive(c, 0);
        drive(d, 0);
        CHECK(c.fold_hash(h0) == d.fold_hash(h0));
        FieldCheckpoint cp = d.checkpoint();
        cp.state.soc_dfc[3] += 1e-9;
        d.restore(cp);
        CHECK(c.fold_hash(h0) != d.fold_hash(h0));
    }
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
        // a checkpoint of a field without soc does not fit this one
        FieldConfig plain = cfg;
        plain.soc_enabled = false;
        Field p(plain);
        CHECK_THROWS_AS(r.restore(p.checkpoint()), std::invalid_argument);
    }
    {
        Field j(cfg);
        for (int i = 0; i < total; ++i)
            if (i == 0 || i == 30 || i == 330) {
                j.advance_to(cfg.initial_day + i, src);
                events(j, i);
            }
        j.advance_to(cfg.initial_day + total, src);
        CHECK(a.fold_hash(h0) == j.fold_hash(h0));
    }
    // soc off is the F1d part 4 field: no soc state at all
    {
        FieldConfig plain = cfg;
        plain.soc_enabled = false;
        Field p(plain);
        CHECK(p.checkpoint().state.hum_c.empty());
        CHECK(p.checkpoint().state.soc_dfc.empty());
    }
}

TEST_CASE("S8 30 years of continuous wheat: straw removed loses humus, straw left gains, difference inside the published range",
          "[field][soc][long]") {
    struct R { std::vector<double> hum; double taw0, taw1, nmin_first5, nmin_last5; double c_res, n_res, w_res; };
    auto run = [](double straw_removed) {
        Field f(soc_config());
        tf::Plan p;
        p.n_kg = 170.0;
        p.p2o5_kg = 60.0;
        p.k2o_kg = 80.0;
        p.straw_removed = straw_removed;
        p.lime_every = 4;
        p.lime_t = 2.0;
        DailyWeatherSource& src = tf::weather30();
        R r{};
        r.taw0 = f.soc_view(0).taw_total_mm;
        double nm_prev = 0.0;
        for (std::int64_t d = 0; d < 30 * 365; ++d) {
            const DailyWeather& w = src.record(d);
            tf::apply_plan_day(f, w, p, nullptr);
            f.step_day(w);
            if ((d + 1) % 365 == 0) {
                r.hum.push_back(f.soc_view(0).hum_c_kg_ha);
                const double nm = f.nutrient_ledger(0).n_mineralised;
                const int y = static_cast<int>((d + 1) / 365);
                if (y == 5) r.nmin_first5 = nm;
                if (y == 25) nm_prev = nm;
                if (y == 30) r.nmin_last5 = nm - nm_prev;
            }
        }
        r.taw1 = f.soc_view(0).taw_total_mm;
        r.c_res = f.c_balance_residual(0);
        r.n_res = f.n_balance_residual(0);
        r.w_res = f.water_balance_residual_mm(0);
        return r;
    };
    const R left = run(0.0), removed = run(1.0);
    const double h0 = 58500.0;
    const double d_removed = (removed.hum.back() - h0) / 1000.0 / 30.0, d_left = (left.hum.back() - h0) / 1000.0 / 30.0;
    std::printf("S8 humus over 30 y: straw removed %.2f -> %.2f t C/ha (%.3f t/ha/yr), straw left %.2f -> %.2f (%.3f t/ha/yr)\n", h0 / 1000.0,
                removed.hum.back() / 1000.0, d_removed, h0 / 1000.0, left.hum.back() / 1000.0, d_left);
    CHECK(removed.hum.back() < h0);   // falls
    // a modelling study of straw export (recalled from a literature search, not checked against the primary paper) puts the loss
    // at 2.5-10.9 % of the initial stock in 50 years; the 30-year loss of this field must not exceed the upper end
    CHECK((h0 - removed.hum.back()) / h0 > 0.01);
    CHECK((h0 - removed.hum.back()) / h0 < 0.109);
    CHECK(left.hum.back() > removed.hum.back());
    // Powlson et al. UK trials of 17-22 years: topsoil carbon on average 6 % higher with incorporation, 0 to 20 % by site;
    // Han et al. 2018 (68 studies, 0-20 cm): +0.35 t C/ha/yr with incorporation, fading after 28-62 years.
    const double diff20 = (left.hum[19] - removed.hum[19]) / h0;
    const double rate20 = (left.hum[19] - removed.hum[19]) / 1000.0 / 20.0;
    std::printf("S8 difference after 20 years: %.1f %% of the initial stock, %.3f t C/ha/yr\n", 100.0 * diff20, rate20);
    CHECK(diff20 > 0.025);
    CHECK(diff20 < 0.20);
    CHECK(rate20 > 0.10);
    CHECK(rate20 < 0.40);
    // the slow decline: first decade faster than the third (the pool approaches its new equilibrium)
    CHECK((removed.hum[0] - removed.hum[9]) > (removed.hum[19] - removed.hum[28]));
    // falling humus lowers the water capacity and the mineralisation, rising humus raises both
    CHECK(removed.taw1 < removed.taw0);
    CHECK(left.taw1 > left.taw0);
    CHECK_THAT(removed.taw1 - removed.taw0, WithinAbs(1000.0 * 0.0116 * 100.0 * ((removed.hum.back() - h0) / 3.9e6) * (0.3 / 1.8) * 1.8, 1e-6));
    CHECK(removed.nmin_last5 < left.nmin_last5);
    // the three ledgers over 30 years
    for (const R* r : {&left, &removed}) {
        CHECK(std::fabs(r->c_res) < 1e-8);
        CHECK(std::fabs(r->n_res) < 1e-8);
        CHECK(std::fabs(r->w_res) < 1e-8);
    }
}
