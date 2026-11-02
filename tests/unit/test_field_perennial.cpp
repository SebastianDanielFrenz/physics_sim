// Hay H1: the perennial sward (crop file "perennial" block, meadow_grass.json). A grass field stays grass for 30 years with and without
// cuts, its biomass stays bounded, the nutrient / carbon / water books close, a cut puts the share it takes into the loose swath
// pool and sends the cell back to its regrowth state, and a crop without the block is untouched.
//
// Recorded sabotages (core/src/field/field.cpp, management.cpp; measured 2026-11-02, reverted):
//  - PN-a: the turnover branch removed from step_cell -> PN1 fails (the uncut sward ends in senescence, canopy and biomass collapse);
//  - PN-b: frost thinning the stand of a perennial again -> PN3 (the -25 degC night leaves the stand at 0.x and the sward does not recover);
//  - PN-c: perennial_cut without the regrowth_gdd reset -> PN2 fails (a cut sward keeps its stage).
//  - PN-d: perennial_cut also scaling the roots (n_root *= keep) -> PN2 fails (na.n_root == nb.n_root);
//  - PN-e: do_mow sending the cut into pool 0 (to_hay = false) -> PN2 and PN4 fail (straw pool touched, no hay);
//  - PN-f: step_hay without the rain re-wetting or without drying -> PN4 fails (moisture bands).
#include "field_test_support.h"
#include "ps/field/field_scenario.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdio>

using namespace ps;
using namespace ps::field;
namespace tf = test_field;
using Catch::Matchers::WithinAbs;

namespace {

FieldConfig grass_config() {
    static const ps::io::FieldScenario sc = ps::io::load_field_scenario(tf::data_dir() / "field_scenarios/wheat_managed_11y.json");
    FieldConfig cfg = sc.field;
    cfg.nx = 1;
    cfg.ny = 1;
    cfg.cell_soil.assign(1, static_cast<std::uint16_t>(0));
    cfg.crops.clear();
    cfg.crops.push_back(tf::crop_named("meadow_grass"));
    cfg.pathogens_enabled = false;
    cfg.weeds.clear();   // the scenario's volunteer wheat names a crop this palette does not have
    cfg.initial_day = -1;
    return cfg;
}

OperationEffect mow(double share) {
    OperationEffect e;
    e.mow = share;
    return e;
}

}  // namespace

TEST_CASE("PN0 meadow_grass loads and validates as a perennial", "[field][perennial]") {
    const CropDesc g = tf::crop_named("meadow_grass");
    CHECK(g.perennial.active);
    CHECK(crop_validation_error(g).empty());
    CHECK_FALSE(tf::crop_named("winter_wheat").perennial.active);
    CropDesc bad = g;
    bad.perennial.cycle_gdd = bad.perennial.regrowth_gdd;
    CHECK_FALSE(crop_validation_error(bad).empty());
    bad = g;
    bad.perennial.cc_floor = 0.0;
    CHECK_FALSE(crop_validation_error(bad).empty());
    FieldConfig cfg = grass_config();
    cfg.management_enabled = false;
    cfg.soc_enabled = false;
    CHECK_THROWS(Field(cfg));   // a perennial needs the management layer
}

TEST_CASE("PN1 thirty years of grass, cut and uncut: still grass, bounded, books closed", "[field][perennial][long]") {
    auto& src = tf::weather30();
    for (const bool cut : {true, false}) {
        Field f(grass_config());
        f.sow(0, 1);
        double max_bio = 0.0, min_cc_after_emerged = 1.0;
        int cuts = 0;
        std::int64_t pickup_day = -1;
        for (std::int64_t d = f.last_day() + 1; d < 30 * 365; ++d) {
            f.step_day(src.record(d));
            const CellView v = f.view(0);
            REQUIRE(v.crop_id == 1);   // a field left for years goes on being grass
            max_bio = std::max(max_bio, v.biomass_g_m2);
            if (v.stage >= 1) min_cc_after_emerged = std::min(min_cc_after_emerged, v.cc);
            if (cut && v.stage >= 3 && v.biomass_g_m2 > 150.0) {   // flowering: cut it
                f.apply(0, mow(1.0));
                ++cuts;
                pickup_day = d + 3;
            }
            if (cut && d == pickup_day) {
                OperationEffect bale;
                bale.hay_pickup = 0.9;   // 10 % stays and rots into the litter
                f.apply(0, bale);
            }
        }
        std::printf("PN1 %s: cuts %d, max biomass %.0f g/m2, min cc %.3f, N res %.3g P res %.3g K res %.3g C res %.3g water res %.3g\n",
                    cut ? "cut" : "uncut", cuts, max_bio, min_cc_after_emerged, f.n_balance_residual(0), f.p_balance_residual(0),
                    f.k_balance_residual(0), f.c_balance_residual(0), f.water_balance_residual_mm(0));
        CHECK(max_bio < 2500.0);   // 25 t DM/ha standing: far above any meadow
        CHECK(min_cc_after_emerged >= 0.08 - 1e-12);
        CHECK(std::abs(f.n_balance_residual(0)) < 1e-6);
        CHECK(std::abs(f.p_balance_residual(0)) < 1e-6);
        CHECK(std::abs(f.k_balance_residual(0)) < 1e-6);
        CHECK(std::abs(f.c_balance_residual(0)) < 1e-6);
        CHECK(std::abs(f.water_balance_residual_mm(0)) < 1e-6);
        if (cut) CHECK(cuts >= 40);   // unfertilised: the hay takes its N away, so fewer than three cuts a year
    }
}

TEST_CASE("PN2 a cut sends the cell back to regrowth and lays the cut biomass in the swath", "[field][perennial]") {
    auto& src = tf::weather30();
    Field f(grass_config());
    f.sow(0, 1);
    std::int64_t d = f.last_day() + 1;
    for (; d < 400; ++d) {
        f.step_day(src.record(d));
        if (f.view(0).stage >= 3) break;
    }
    const CellView before = f.view(0);
    REQUIRE(before.stage >= 3);
    const ManagementView mb = f.management_view(0);
    const NutrientView nb = f.nutrient_view(0);
    f.apply(0, mow(1.0));
    const CellView after = f.view(0);
    const ManagementView ma = f.management_view(0);
    const CropDesc g = tf::crop_named("meadow_grass");
    CHECK_THAT(after.gdd_sow, WithinAbs(g.perennial.regrowth_gdd, 1e-9));
    CHECK(after.stage < before.stage);
    CHECK_THAT(after.biomass_g_m2, WithinAbs(before.biomass_g_m2 * g.perennial.stubble_fraction, 1e-6));
    CHECK(after.cc >= g.perennial.cc_floor);
    CHECK(ma.straw_t_ha == mb.straw_t_ha);   // the loose straw pool is not touched: the cut grass lies in its own swath pool
    CHECK(ma.hay_dm_t_ha > 0.01);
    CHECK_THAT(ma.hay_moisture, WithinAbs(g.perennial.hay_fresh_moisture, 1e-9));
    const NutrientView na = f.nutrient_view(0);
    CHECK(na.n_root == nb.n_root);   // a mower does not touch the roots
    CHECK(na.n_crop < nb.n_crop);
    CHECK_THAT(ma.hay_n_kg_ha, WithinAbs(nb.n_crop - na.n_crop, 1e-9));
    // a half cut takes half of what is above the stubble
    Field h(grass_config());
    h.sow(0, 1);
    for (std::int64_t e = h.last_day() + 1; e <= d; ++e) h.step_day(src.record(e));
    const double b0 = h.view(0).biomass_g_m2;
    h.apply(0, mow(0.5));
    CHECK_THAT(h.view(0).biomass_g_m2, WithinAbs(b0 * (1.0 - 0.5 * (1.0 - g.perennial.stubble_fraction)), 1e-6));
}

TEST_CASE("PN3 frost thins the canopy, not the stand", "[field][perennial]") {
    auto& src = tf::weather30();
    Field f(grass_config());
    f.sow(0, 1);
    std::int64_t d = f.last_day() + 1;
    for (; d < 300; ++d) f.step_day(src.record(d));
    REQUIRE(f.view(0).stage >= 1);
    for (int i = 0; i < 3; ++i) f.step_day(tf::synth(f.last_day() + 1, -35.0, -30.0));
    const CellView v = f.view(0);
    CHECK(v.stand == 1.0);
    CHECK(v.cc >= 0.08 - 1e-12);
    CHECK(v.crop_id == 1);
}

TEST_CASE("PN4 the swath dries with the evaporative demand, is wetted by rain, is picked up with its mass and moisture, rots when left", "[field][perennial][hay]") {
    auto& src = tf::weather30();
    const CropDesc g = tf::crop_named("meadow_grass");
    Field f(grass_config());
    f.sow(0, 1);
    for (std::int64_t d = f.last_day() + 1; d < 400; ++d) {
        f.step_day(src.record(d));
        if (f.view(0).stage >= 3) break;
    }
    f.apply(0, mow(1.0));
    const double dm0 = f.management_view(0).hay_dm_t_ha;
    REQUIRE(dm0 > 0.01);
    // dry days: the moisture falls monotonically toward the equilibrium and the dry matter stays
    double prev = f.management_view(0).hay_moisture;
    for (int i = 0; i < 6; ++i) {
        f.step_day(tf::synth(f.last_day() + 1, 10.0, 24.0, 0.0, 0.0, 5.0));
        const double m = f.management_view(0).hay_moisture;
        CHECK(m < prev);
        CHECK(m >= g.perennial.hay_equilibrium_moisture - 1e-12);
        prev = m;
    }
    CHECK_THAT(f.management_view(0).hay_dm_t_ha, WithinAbs(dm0, 1e-12));
    CHECK(prev < 0.2);   // dry enough to bale after six sunny days
    // rain wets it again, but not above the cap
    f.step_day(tf::synth(f.last_day() + 1, 10.0, 16.0, 12.0, 0.0, 0.5));
    const double wet = f.management_view(0).hay_moisture;
    CHECK(wet > prev + 0.05);
    f.step_day(tf::synth(f.last_day() + 1, 10.0, 16.0, 80.0, 0.0, 0.1));
    CHECK(f.management_view(0).hay_moisture <= g.perennial.hay_max_moisture + 1e-9);
    // pick up 60 %: mass and moisture come back, the books follow
    const double n_removed_before = f.nutrient_ledger(0).n_removed;
    OperationEffect bale;
    bale.hay_pickup = 0.6;
    const HarvestResult r = f.apply_all(bale);
    CHECK_THAT(r.hay_dm_t_ha, WithinAbs(0.6 * dm0, 1e-9));
    CHECK_THAT(r.hay_moisture(), WithinAbs(f.management_view(0).hay_moisture, 1e-9));
    CHECK_THAT(f.management_view(0).hay_dm_t_ha, WithinAbs(0.4 * dm0, 1e-9));
    CHECK(f.nutrient_ledger(0).n_removed > n_removed_before);
    CHECK(std::abs(f.n_balance_residual(0)) < 1e-9);
    CHECK(std::abs(f.c_balance_residual(0)) < 1e-9);
    // the rest lies for the rot time and joins the loose litter
    const double straw0 = f.management_view(0).straw_t_ha;
    for (int i = 0; i < static_cast<int>(g.perennial.hay_rot_after_days) + 1; ++i) f.step_day(tf::synth(f.last_day() + 1, 10.0, 20.0, 0.0, 0.0, 3.0));
    CHECK(f.management_view(0).hay_dm_t_ha == 0.0);
    CHECK(f.management_view(0).straw_t_ha > straw0);
    CHECK(std::abs(f.n_balance_residual(0)) < 1e-9);
    CHECK(std::abs(f.c_balance_residual(0)) < 1e-9);
}

TEST_CASE("PN5 hay state exists, is hashed and checkpointed only with a perennial in the palette", "[field][perennial][hay]") {
    FieldConfig with = grass_config();
    FieldConfig without = grass_config();
    without.crops.clear();
    without.crops.push_back(tf::crop_named("winter_wheat"));
    Field a(with), b(without);
    a.sow(0, 1);
    b.sow(0, 1);
    CHECK(a.fold_hash(0) != b.fold_hash(0));
    // a mow changes the hash through the hay pool and a checkpoint restores it
    auto& src = tf::weather30();
    for (std::int64_t d = a.last_day() + 1; d < 300; ++d) a.step_day(src.record(d));
    const FieldCheckpoint cp = a.checkpoint();
    const std::uint64_t h0 = a.fold_hash(0);
    a.apply(0, mow(1.0));
    CHECK(a.fold_hash(0) != h0);
    a.restore(cp);
    CHECK(a.fold_hash(0) == h0);
    const std::uint64_t hb = b.fold_hash(0);
    b.apply(0, mow(1.0));   // a mow on an annual does nothing
    CHECK(b.fold_hash(0) == hb);
}
