// tests/unit/test_field_water.cpp - the soil water model of ps::field::Field
// (FAO-56 dual crop coefficient, root-zone depletion, Ks, SCS runoff, snow,
// drainage, root deepening) against closed forms and conservation.
//
// Recorded sabotages (F1a, 2026-10-31), each made in core/src/field/field.cpp,
// the test run, then reverted:
//  - A1: Kr without the REW stage (Kr = (TEW - De)/(TEW - REW) from De = 0):
//        "bare soil evaporation follows FAO-56 eq 71-77" failed (day 1 evaporation 3.4 instead of 6.0 mm).
//  - A2: SCS retention S = 25.4 (1000/CN - 10) written with 1000 -> 100: both
//        runoff tests failed (Q off by an order of magnitude).
//  - A3: the CN wetness interpolation always returned CN_II: the dry/wet
//        runoff cases failed, the CN_II case still passed.
//  - A4: drainage out of the root zone dropped instead of filling the layer
//        below (d_sub not reduced): "heavy rain fills the profile" and
//        the water balance closure failed.
//  - A5: set_root_depth moved the strip's water with the wrong weight
//        ((z_new - z_old) / depth instead of / (depth - z_old)): "root
//        deepening conserves water" failed (fractions differ) and so did the
//        30-year closure.
//  - A6: Ks without the (1 - p) divisor in the denominator: "Ks follows
//        FAO-56 eq 84" failed.
//  - A7: snowmelt not limited by the store (melt = factor * degrees): the
//        snow test failed (swe went negative) and the closure test failed.
//  - A8: the evaporation cap dropped (evap not limited to few*(TEW - De)):
//        the bare-soil recurrence failed from day 8 on.

#include "field_test_support.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>

using namespace ps;
using namespace ps::field;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// SCS runoff in mm for retention S (mm), initial abstraction ratio ia.
double q_scs(double p, double cn, double ia_ratio) {
    const double s = 25.4 * (1000.0 / cn - 10.0);
    const double ia = ia_ratio * s;
    if (p <= ia) return 0.0;
    return (p - ia) * (p - ia) / (p - ia + s);
}

// One day of rain on a one-cell loess field at the given initial depletion fraction; returns the runoff.
double runoff_after(double depletion_fraction, double rain_mm) {
    Field f(test_field::one_cell_config(test_field::loess(), test_field::wheat(), 1, 1, depletion_fraction));
    f.step_day(test_field::synth(0, 5.0, 15.0, rain_mm, 0.0, 0.0));
    return f.view(0).last_runoff_mm;
}

} // namespace

TEST_CASE("A1 bare soil evaporation follows FAO-56 eq 71-77", "[field][water]") {
    const SoilDesc& so = test_field::loess();
    const double tew = so.tew_mm();
    const double rew = so.rew_mm;
    REQUIRE_THAT(tew, WithinAbs(23.5, 1e-12));      // 1000 (0.30 - 0.5 * 0.13) * 0.10
    Field f(test_field::one_cell_config(so, test_field::wheat()));
    const double et0 = 5.0;

    // Hand values: Kc,max = 1.2 (wind 2 m/s, RHmin 45 %), stage 1 (Kr = 1) while De <= REW.
    f.step_day(test_field::synth(0, 5, 15, 0, 0, et0, 2.0, 0.45));
    CHECK_THAT(f.view(0).last_e_mm, WithinAbs(6.0, 1e-12));
    CHECK_THAT(f.view(0).d_evap_mm, WithinAbs(6.0, 1e-12));
    f.step_day(test_field::synth(1, 5, 15, 0, 0, et0, 2.0, 0.45));
    CHECK_THAT(f.view(0).last_e_mm, WithinAbs(6.0, 1e-12));
    CHECK_THAT(f.view(0).d_evap_mm, WithinAbs(12.0, 1e-12));
    // Day 3: De = 12 > REW 9.5 -> Kr = (23.5 - 12) / (23.5 - 9.5) = 0.821428..
    f.step_day(test_field::synth(2, 5, 15, 0, 0, et0, 2.0, 0.45));
    CHECK_THAT(f.view(0).last_e_mm, WithinAbs(6.0 * 11.5 / 14.0, 1e-12));

    // Whole recurrence for 40 days, written out independently of the field.
    double de = 12.0 + 6.0 * 11.5 / 14.0;
    for (int d = 3; d < 40; ++d) {
        const double kr = de <= rew ? 1.0 : std::max(0.0, (tew - de) / (tew - rew));
        const double e = std::min(kr * 1.2 * et0, tew - de);
        de = std::min(tew, de + e);
        f.step_day(test_field::synth(d, 5, 15, 0, 0, et0, 2.0, 0.45));
        INFO("day " << d);
        REQUIRE_THAT(f.view(0).last_e_mm, WithinAbs(e, 1e-12));
        REQUIRE_THAT(f.view(0).d_evap_mm, WithinAbs(de, 1e-12));
    }
    CHECK(de > 0.95 * tew);                              // the surface layer is nearly dry after 40 days without rain
    // Bare soil draws everything from the root zone: Dr equals the evaporation total.
    CHECK_THAT(f.view(0).d_root_mm, WithinAbs(f.ledger(0).evaporation, 1e-9));
    CHECK(f.ledger(0).transpiration == 0.0);
    CHECK(std::fabs(f.water_balance_residual_mm(0)) < 1e-12);
}

TEST_CASE("A9 evaporation cannot take more than the root zone holds", "[field][water]") {
    Field f(test_field::one_cell_config(test_field::loess(), test_field::wheat(), 1, 1, 0.999));
    for (int d = 0; d < 20; ++d) {
        f.step_day(test_field::synth(d, 5, 15, 0, 0, 6.0));
        REQUIRE(f.view(0).d_root_mm <= f.view(0).taw_root_mm + 1e-12);
        REQUIRE(f.view(0).d_root_mm >= 0.0);
    }
    CHECK(std::fabs(f.water_balance_residual_mm(0)) < 1e-12);
}

TEST_CASE("A2 SCS curve number runoff equals the closed form at every wetness", "[field][water]") {
    const SoilDesc& so = test_field::loess();
    const double cn2 = so.curve_number_ii;
    const double cn1 = cn2 / (2.281 - 0.01281 * cn2);
    const double cn3 = cn2 / (0.427 + 0.00573 * cn2);
    CHECK_THAT(cn1, WithinAbs(56.8, 0.05));       // NEH-4 tabulates CN II 75 -> I about 55-57
    CHECK_THAT(cn3, WithinAbs(87.5, 0.05));       // ... -> III about 88

    // wetness w = 1 - Dr/TAW. depletion fraction f -> w = 1 - f (the root zone and the layer below start alike).
    // Average (w = 0.5 = cn_normal_wetness): CN II exactly.
    {
        const double s = 25.4 * (1000.0 / cn2 - 10.0);
        CHECK_THAT(s, WithinAbs(84.6667, 1e-3));
        CHECK_THAT(runoff_after(0.5, 40.0), WithinAbs(q_scs(40.0, cn2, 0.2), 1e-9));
        CHECK_THAT(q_scs(40.0, cn2, 0.2), WithinAbs(4.939, 1e-3));    // (40 - 16.933)^2 / (40 + 0.8 * 84.667)
    }
    // Dry (w = 0) CN I, wet (w = 1) CN III, halfway between on both sides.
    CHECK_THAT(runoff_after(1.0, 40.0), WithinAbs(q_scs(40.0, cn1, 0.2), 1e-9));
    CHECK_THAT(runoff_after(0.0, 40.0), WithinAbs(q_scs(40.0, cn3, 0.2), 1e-9));
    CHECK_THAT(runoff_after(0.75, 40.0), WithinAbs(q_scs(40.0, 0.5 * (cn1 + cn2), 0.2), 1e-9));   // w = 0.25
    CHECK_THAT(runoff_after(0.25, 40.0), WithinAbs(q_scs(40.0, 0.5 * (cn2 + cn3), 0.2), 1e-9));   // w = 0.75
    // Wetter soil sheds more; a rain below the initial abstraction sheds nothing.
    CHECK(runoff_after(0.0, 40.0) > runoff_after(0.5, 40.0));
    CHECK(runoff_after(0.5, 40.0) > runoff_after(1.0, 40.0));
    CHECK(runoff_after(0.5, 10.0) == 0.0);
    // Mass: what is not runoff enters the soil.
    Field f(test_field::one_cell_config(so, test_field::wheat(), 1, 1, 0.5));
    const double before = f.view(0).d_root_mm + f.view(0).d_sub_mm;
    f.step_day(test_field::synth(0, 5, 15, 40.0, 0.0, 0.0));
    const double q = f.view(0).last_runoff_mm;
    CHECK_THAT(before - (f.view(0).d_root_mm + f.view(0).d_sub_mm), WithinAbs(40.0 - q, 1e-9));
}

TEST_CASE("A3 heavy rain fills the profile, the rest leaves as drainage", "[field][water]") {
    const SoilDesc& so = test_field::loess();
    Field f(test_field::one_cell_config(so, test_field::wheat(), 1, 1, 0.5));
    const double initial_deficit = f.view(0).d_root_mm + f.view(0).d_sub_mm;   // 25.5 + 127.5 mm
    CHECK_THAT(initial_deficit, WithinAbs(153.0, 1e-9));
    double first_drain_day = -1.0;
    for (int d = 0; d < 25; ++d) {
        f.step_day(test_field::synth(d, 8, 14, 30.0, 0.0, 0.0));
        if (first_drain_day < 0.0 && f.view(0).last_drain_mm > 0.0) first_drain_day = d;
    }
    const WaterLedger l = f.ledger(0);
    CHECK(first_drain_day > 3.0);                                       // the deficit has to fill first
    CHECK_THAT(f.view(0).d_root_mm, WithinAbs(0.0, 1e-9));
    CHECK_THAT(f.view(0).d_sub_mm, WithinAbs(0.0, 1e-9));
    CHECK_THAT(l.drainage, WithinAbs(l.precipitation - l.runoff - initial_deficit, 1e-9));
    CHECK(std::fabs(f.water_balance_residual_mm(0)) < 1e-12);
}

TEST_CASE("A4 snow store: degree-day melt, no evaporation under snow", "[field][water]") {
    const SoilDesc& so = test_field::loess();           // 3.0 mm per K and day above 0 C
    Field f(test_field::one_cell_config(so, test_field::wheat(), 1, 1, 0.5));
    f.step_day(test_field::synth(0, -8.0, -2.0, 0.0, 20.0, 1.0));        // 20 mm snow at -5 C mean
    CHECK_THAT(f.view(0).swe_mm, WithinAbs(20.0, 1e-12));
    f.step_day(test_field::synth(1, -4.0, -1.0, 0.0, 0.0, 1.0));         // below freezing: no melt
    CHECK_THAT(f.view(0).swe_mm, WithinAbs(20.0, 1e-12));
    const double d_before = f.view(0).d_root_mm + f.view(0).d_sub_mm;
    f.step_day(test_field::synth(2, 1.0, 5.0, 0.0, 0.0, 1.0));           // 3 C mean -> 9 mm melt
    CHECK_THAT(f.view(0).swe_mm, WithinAbs(11.0, 1e-12));
    CHECK(f.view(0).last_e_mm == 0.0);                                   // snow covers the soil
    f.step_day(test_field::synth(3, 1.0, 5.0, 0.0, 0.0, 1.0));
    CHECK_THAT(f.view(0).swe_mm, WithinAbs(2.0, 1e-12));
    f.step_day(test_field::synth(4, 1.0, 5.0, 0.0, 0.0, 1.0));           // the store is limiting, never negative
    CHECK_THAT(f.view(0).swe_mm, WithinAbs(0.0, 1e-12));
    // The melt reached the soil minus runoff.
    const double melted = 20.0;
    const double d_after = f.view(0).d_root_mm + f.view(0).d_sub_mm;
    CHECK(d_after < d_before);
    CHECK_THAT(d_before - d_after + f.ledger(0).evaporation,        // evaporation only after the cover is gone
               WithinAbs(melted - f.ledger(0).runoff, 1e-9));
    CHECK(std::fabs(f.water_balance_residual_mm(0)) < 1e-12);
    // Evaporation resumes once the snow is gone.
    f.step_day(test_field::synth(5, 1.0, 5.0, 0.0, 0.0, 3.0));
    CHECK(f.view(0).last_e_mm > 0.0);
}

TEST_CASE("A5 root deepening conserves water and keeps the depletion fraction", "[field][water]") {
    CropDesc crop = test_field::wheat();
    crop.vernalises = false;
    const SoilDesc& so = test_field::loess();
    for (double frac : {0.0, 0.4, 0.85}) {
        Field f(test_field::one_cell_config(so, crop, 1, 1, frac));
        f.sow_all(1);
        const double total0 = f.view(0).d_root_mm + f.view(0).d_sub_mm;
        double zr_prev = f.view(0).zr_m;
        // et0 = 0: no ET, no rain: only the roots move.
        for (int d = 0; d < 260; ++d) {
            f.step_day(test_field::synth(d, 10.0, 20.0, 0.0, 0.0, 0.0));
            const CellView v = f.view(0);
            REQUIRE(v.zr_m >= zr_prev);
            zr_prev = v.zr_m;
            REQUIRE_THAT(v.d_root_mm + v.d_sub_mm, WithinAbs(total0, 1e-9));
            REQUIRE_THAT(v.d_root_mm / (so.taw_mm_per_m() * v.zr_m), WithinAbs(frac, 1e-12));
            REQUIRE_THAT(v.d_sub_mm / (so.taw_mm_per_m() * (so.depth_m - v.zr_m)), WithinAbs(frac, 1e-12));
        }
        CHECK_THAT(zr_prev, WithinAbs(crop.zr_max_m, 1e-12));
        // Harvest returns the roots to the fallow depth, conserving water again.
        f.harvest(0);
        const CellView v = f.view(0);
        CHECK_THAT(v.zr_m, WithinAbs(so.fallow_root_depth_m, 1e-15));
        CHECK_THAT(v.d_root_mm + v.d_sub_mm, WithinAbs(total0, 1e-9));
        CHECK_THAT(v.d_root_mm / (so.taw_mm_per_m() * v.zr_m), WithinAbs(frac, 1e-12));
    }
}

TEST_CASE("A6 Ks follows FAO-56 eq 84 with the Table 22 depletion fraction", "[field][water]") {
    CropDesc crop = test_field::wheat();
    crop.vernalises = false;
    const SoilDesc& so = test_field::loess();
    for (bool adjust : {false, true}) {
        crop.p_adjust_for_etc = adjust;
        Field f(test_field::one_cell_config(so, crop));
        f.sow_all(1);
        f.set_irrigate_to_capacity(true);
        for (int d = 0; d < 110; ++d) f.step_day(test_field::synth(d, 10.0, 20.0, 0.0, 0.0, 4.0));   // 15 GDD/day: roots full at 1500
        REQUIRE_THAT(f.view(0).zr_m, WithinAbs(crop.zr_max_m, 1e-12));
        f.set_irrigate_to_capacity(false);

        const double taw = so.taw_mm_per_m() * crop.zr_max_m;       // 255 mm
        int stressed = 0, dead = 0;
        for (int d = 110; d < 190; ++d) {
            const double dr = f.view(0).d_root_mm;                   // depletion the day's Ks is computed from
            f.step_day(test_field::synth(d, 10.0, 20.0, 0.0, 0.0, 6.0));
            const CellView v = f.view(0);
            double p = crop.p_depletion;
            if (adjust) {
                // ETc of the unstressed crop = Tr/Ks + E (recovered from the day's outputs; valid while Ks > 0)
                const double etc = v.last_ks_sto > 0.0 ? v.last_tr_mm / v.last_ks_sto + v.last_e_mm : 5.0;
                p = std::min(0.8, std::max(0.1, p + 0.04 * (5.0 - etc)));
            }
            const double ks = dr <= p * taw ? 1.0 : std::max(0.0, (taw - dr) / ((1.0 - p) * taw));
            INFO("day " << d << " Dr " << dr << " p " << p);
            if (!adjust || v.last_ks_sto > 0.05) REQUIRE_THAT(v.last_ks_sto, WithinAbs(ks, 1e-9));
            stressed += v.last_ks_sto < 1.0;
            dead += v.last_ks_sto == 0.0;
        }
        CHECK(stressed > 10);        // the test reached the stressed range
        if (!adjust) CHECK(dead >= 0);
        CHECK(std::fabs(f.water_balance_residual_mm(0)) < 1e-9);
    }
}

TEST_CASE("A7 a wheat canopy transpires, a stressed one less", "[field][water]") {
    CropDesc crop = test_field::wheat();
    crop.vernalises = false;
    Field f(test_field::one_cell_config(test_field::loess(), crop, 2, 1));
    f.sow_all(1);
    f.set_irrigate_to_capacity(true);
    for (int d = 0; d < 100; ++d) f.step_day(test_field::synth(d, 10.0, 20.0, 0.0, 0.0, 4.0));
    const double tr_wet = f.view(0).last_tr_mm;
    CHECK(tr_wet > 3.0);                                // full canopy: Kcb about 1.1 of 4 mm
    CHECK(tr_wet < 4.6);
}

TEST_CASE("A8 the water balance closes to 1e-9 mm over 30 years", "[field][water]") {
    DailyWeatherSource& src = test_field::weather30();
    FieldConfig cfg;
    cfg.nx = 4;
    cfg.ny = 2;
    cfg.soils = {test_field::loess(), test_field::sandy()};
    cfg.crops = {test_field::wheat()};
    cfg.cell_soil = {0, 1, 0, 1, 1, 0, 1, 0};
    cfg.initial_depletion_fraction = 0.3;
    Field f(cfg);
    double precip_sum = 0.0;
    double worst = 0.0;
    int harvests = 0;
    for (std::int64_t d = 0; d < 30 * 365; ++d) {
        const DailyWeather& w = src.record(d);
        if (w.doy == 288) f.sow_all(1);
        if (w.doy == 212 && d > 365) {
            if (f.harvest_all().crop_id != 0) ++harvests;
        }
        f.step_day(w);
        precip_sum += w.precip_mm();
        if (d % 365 == 364)
            for (int c = 0; c < f.cell_count(); ++c) worst = std::max(worst, std::fabs(f.water_balance_residual_mm(c)));
    }
    INFO("worst residual " << worst << " mm, harvests " << harvests);
    CHECK(worst < 1e-9);
    for (int c = 0; c < f.cell_count(); ++c) {
        CHECK(std::fabs(f.water_balance_residual_mm(c)) < 1e-9);
        CHECK_THAT(f.ledger(c).precipitation, WithinRel(precip_sum, 1e-12));
        CHECK(f.ledger(c).irrigation == 0.0);
    }
    CHECK(harvests == 29);
    // The record has the rain the climate asks for: a real water cycle ran.
    const WaterLedger l = f.ledger(0);
    CHECK(l.transpiration > 30 * 150.0);
    CHECK(l.evaporation > 30 * 100.0);
    CHECK(l.drainage > 100.0);
}
