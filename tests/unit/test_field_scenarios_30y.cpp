// F1d part 7: the three 30-year field scenarios of data/field_scenarios (nutrients, management, pathogens, soil organic carbon).
//
//  wheat_continuous_30y.json            winter wheat every year, fertilised, straw returned
//  rotation_rape_wheat_barley_30y.json  rapeseed -> wheat -> barley (a non-cereal break every third year), same fertiliser logic
//  wheat_unfertilised_30y.json          winter wheat every year, no N, P or K (chalk kept), Broadbalk-style
//
// What the tests pin down:
//  Y1  all three load and run as pure functions of the file (two runs give the same state hash and the same harvests), 30 harvests
//      each, and the water, N, P2O5, K2O, CaO ledgers close to 1e-9 (mm, kg/ha), the residue carbon ledger to 1e-8 kg C/ha (its worst
//      measured residual is 1.5e-9 kg C/ha, i.e. 2.6e-14 of the humus stock: rounding of 11000 daily sums, not a leak);
//  Y2  1, 2 and 10 workers (cells stepped in disjoint ranges on threads) give the same state hash as the sequential step, on a
//      10-cell variant of each scenario whose cells alternate between the loess and the sandy loam (so cells differ);
//  Y3  mono-culture does not pay: continuous wheat (fertiliser refills the nutrients, so it does not decline with time) yields 80-90 %
//      of the wheat in the rotation, years 11-30 (the take-all margin of the F1d part 4 tests: AHDB / Farmers Weekly second and third
//      wheat 10-20 % below first wheat, Rothamsted take-all decline to 80-90 % of the early yields, Wikipedia 'Take-all' summary), and
//      the pathogen model alone accounts for 3-15 % of continuous wheat (with / without pathogens, same file);
//  Y4  unfertilised wheat: no N, P, K is applied in 30 years, the yield is far below the fertilised one, it declines, and the
//      humus falls where the fertilised field gains humus (more residue);
//  Y5  (tag [!mayfail], measured gap) the level of the unfertilised yield against the Broadbalk nil plot (plot 3): about 1 t/ha
//      at 85 % dry matter, 'Yields of continuous wheat given no fertilizer or manure (Nil plots 3) have remained at around 1 t ha-1
//      in general' (Rothamsted ERA, Perryman et al., dataset rbk1 02-OAWWYields). The model gives about 2.9 t/ha on the loess and
//      about 1.7 t/ha on the sandy loam, so the case fails by design and says so; Y5b is the sandy-loam case, the lower bound.
//
// Recorded sabotages (made in the source or the scenario file, this file run, reverted; 2026-10-31; cases that failed in brackets):
//  - SY-a: the pathogen yield loss left at zero (stress_rotation = 1 in the daily step): [Y3] (with / without pathogens = 1.0; the
//          ratio to the rotation alone, 0.94 then, would have passed the first version's 0.80-0.95 band, hence the 0.90 limit);
//  - SY-b: the unfertilised scenario with the first DAP and CAN dressings put back: [Y4] (applied N and P2O5 not zero);
//  - SY-c: step_soc reading the next cell's humus carbon (cells then depend on the order of stepping): [Y2];
//  - SY-d: an error of 1e-9 mm per day added to the booked transpiration: [Y1] (the water ledger limit is tight enough to see it);
//  - SY-e: the humus carbon not decaying (hum_c untouched in the mineralisation): [Y1 Y4] (the C ledger and the unfertilised humus no longer falls).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "field_test_support.h"
#include "ps/field/field_scenario.h"

#include <algorithm>
#include <barrier>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace ps;
using namespace ps::field;
namespace tf = ps::test_field;
using ps::io::FieldRunSummary;
using ps::io::FieldScenario;

namespace {

const char* const kCont = "wheat_continuous_30y.json";
const char* const kRot = "rotation_rape_wheat_barley_30y.json";
const char* const kUnf = "wheat_unfertilised_30y.json";
const std::uint64_t kHashSeed = 1469598103934665603ull;

FieldScenario load(const std::string& name) { return ps::io::load_field_scenario(tf::data_dir() / "field_scenarios" / name); }

// the runs of the shipped files, once per process
const FieldRunSummary& base_run(const std::string& name) {
    static std::map<std::string, FieldRunSummary> cache;
    auto it = cache.find(name);
    if (it == cache.end()) it = cache.emplace(name, ps::io::run_field_scenario(load(name))).first;
    return it->second;
}

std::map<int, double> grain_by_year(const FieldRunSummary& s) {
    std::map<int, double> y;
    for (const auto& h : s.harvests) y[h.year] = h.result.grain_dm_t_ha;
    return y;
}

double mean_years(const std::map<int, double>& y, int from, int to, int step = 1, int offset = 0) {
    double sum = 0.0;
    int n = 0;
    for (int k = from; k <= to; ++k)
        if ((k - from) % step == offset % step) {
            sum += y.at(k);
            ++n;
        }
    return sum / n;
}

// The runner's day loop on a Field built from `sc`, optionally with the cells stepped on `threads` threads (0: step_day).
struct Played {
    std::uint64_t hash = 0;
    std::vector<double> hum;            // humus carbon per cell at the end, kg C/ha
    double n_applied = 0.0, p_applied = 0.0, k_applied = 0.0, cao_applied = 0.0;   // cell 0, kg/ha
};

Played play(const FieldScenario& sc, int threads) {
    DailyWeatherSource src(sc.climate, sc.seed, sc.weather);
    Field f(sc.field);
    const std::int64_t n_days = static_cast<std::int64_t>(sc.years) * src.days_per_year();
    const int n = f.cell_count();
    std::barrier start(threads + 1), done(threads + 1);
    const DailyWeather* today = nullptr;
    bool stop = false;
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; ++t) {
        const int begin = n * t / threads, end = n * (t + 1) / threads;
        pool.emplace_back([&, begin, end] {
            for (;;) {
                start.arrive_and_wait();
                if (stop) return;
                f.step_cells(begin, end, *today);
                done.arrive_and_wait();
            }
        });
    }
    for (std::int64_t d = 0; d < n_days; ++d) {
        const DailyWeather& w = src.record(d);
        const int year = static_cast<int>(d / src.days_per_year());
        for (const ps::io::FieldOperationRule& rule : sc.rules) {
            if (rule.doy != w.doy || year < rule.first_year || year > rule.last_year || (year - rule.first_year) % rule.every_years != 0) continue;
            if (rule.op == "sow") f.sow_all(rule.crop_id, rule.density_scale);
            else if (rule.op == "apply" || rule.op == "operation") f.apply_all(rule.effect);
            else f.harvest_all(rule.straw_removed);
        }
        if (threads == 0) {
            f.step_day(w);
        } else {
            today = &w;
            start.arrive_and_wait();
            done.arrive_and_wait();
            f.finish_day(d);
        }
    }
    stop = true;
    if (threads > 0) start.arrive_and_wait();
    for (std::thread& th : pool) th.join();
    Played p;
    p.hash = f.fold_hash(kHashSeed);
    for (int c = 0; c < n; ++c) p.hum.push_back(f.soc_view(c).hum_c_kg_ha);
    const NutrientLedger nl = f.nutrient_ledger(0);
    p.n_applied = nl.n_fert_mineral + nl.n_fert_organic;
    p.p_applied = nl.p_applied;
    p.k_applied = nl.k_applied;
    p.cao_applied = nl.cao_applied;
    return p;
}

// 5 x 2 cells, soils alternating loess / sandy loam
FieldScenario ten_cells(FieldScenario sc) {
    sc.field.nx = 5;
    sc.field.ny = 2;
    sc.field.cell_soil.assign(10, 0);
    for (std::size_t i = 0; i < 10; ++i) sc.field.cell_soil[i] = static_cast<std::uint16_t>(i % 2);
    return sc;
}

}  // namespace

TEST_CASE("Y1 the 30-year scenarios are pure functions of their file and close every ledger", "[field][scenario30][long]") {
    for (const char* name : {kCont, kRot, kUnf}) {
        INFO(name);
        const FieldScenario sc = load(name);
        CHECK(sc.field.nutrients_enabled);
        CHECK(sc.field.management_enabled);
        CHECK(sc.field.pathogens_enabled);
        CHECK(sc.field.soc_enabled);
        const FieldRunSummary& a = base_run(name);
        const FieldRunSummary b = ps::io::run_field_scenario(sc);
        CHECK(a.state_hash == b.state_hash);
        REQUIRE(a.harvests.size() == b.harvests.size());
        REQUIRE(a.harvests.size() == 30);
        for (std::size_t i = 0; i < a.harvests.size(); ++i) {
            CHECK(a.harvests[i].day == b.harvests[i].day);
            CHECK(a.harvests[i].result.grain_dm_t_ha == b.harvests[i].result.grain_dm_t_ha);
            CHECK(a.harvests[i].year == static_cast<int>(i) + 1);
        }
        std::printf("Y1 %-40s water %.2e mm, N %.2e, P2O5 %.2e, K2O %.2e, CaO %.2e kg/ha, C %.2e kg C/ha (precipitation %.0f mm)\n", name,
                    a.max_balance_residual_mm, a.max_n_residual, a.max_p_residual, a.max_k_residual, a.max_cao_residual, a.max_c_residual,
                    a.total_precip_mm);
        CHECK(a.max_balance_residual_mm < 1e-9);
        CHECK(a.max_n_residual < 1e-9);
        CHECK(a.max_p_residual < 1e-9);
        CHECK(a.max_k_residual < 1e-9);
        CHECK(a.max_cao_residual < 1e-9);
        CHECK(a.max_c_residual < 1e-8);
        CHECK(a.total_precip_mm > 17000.0);   // 31 years of the home climate: the run is the long one
    }
}

TEST_CASE("Y2 1, 2 and 10 workers give the state of the sequential step on ten different cells", "[field][scenario30][determinism][long]") {
    for (const char* name : {kCont, kRot, kUnf}) {
        INFO(name);
        const FieldScenario sc = ten_cells(load(name));
        const Played seq = play(sc, 0);
        // the cells are not copies of each other, or the comparison would prove little
        REQUIRE(seq.hum.size() == 10);
        CHECK(seq.hum[0] != seq.hum[1]);
        CHECK(seq.hum[0] == seq.hum[2]);   // the same soil, the same weather: identical cells
        for (int threads : {1, 2, 10}) {
            INFO(threads << " threads");
            const Played t = play(sc, threads);
            CHECK(t.hash == seq.hash);
            CHECK(t.hum == seq.hum);
        }
    }
}

TEST_CASE("Y3 continuous wheat falls behind the rotation by the cited margin; fertiliser keeps it from declining", "[field][scenario30][long]") {
    const std::map<int, double> cont = grain_by_year(base_run(kCont)), rot = grain_by_year(base_run(kRot));
    // rotation: rapeseed in harvest years 1, 4, ..., wheat in 2, 5, ..., barley in 3, 6, ...
    const double wheat_rot = mean_years(rot, 11, 29, 3, 0);   // 11, 14, ..., 29 (the wheat years)
    const double wheat_cont = mean_years(cont, 11, 30);
    const double ratio = wheat_cont / wheat_rot;
    std::printf("Y3 wheat grain years 11-30: continuous %.2f t/ha dm, in the rotation %.2f, ratio %.3f; continuous years 1-10 %.2f, 21-30 %.2f\n",
                wheat_cont, wheat_rot, ratio, mean_years(cont, 1, 10), mean_years(cont, 21, 30));
    // the rotation's wheat years are the ones of harvest year 2 (mod 3)
    for (int y = 11; y <= 29; y += 3) CHECK(((y - 1) % 3) == 1);
    CHECK(ratio >= 0.80);
    CHECK(ratio <= 0.90);   // 10-20 % below first wheat; the break crop's N and the pathogens together
    // the pathogen model alone costs the continuous crop part of that: the same file without pathogens yields more
    FieldScenario nopath = load(kCont);
    nopath.field.pathogens_enabled = false;
    const double wheat_nopath = mean_years(grain_by_year(ps::io::run_field_scenario(nopath)), 11, 30);
    std::printf("Y3 continuous wheat years 11-30 without pathogens: %.2f t/ha dm, with/without %.3f\n", wheat_nopath, wheat_cont / wheat_nopath);
    CHECK(wheat_cont / wheat_nopath < 0.97);
    CHECK(wheat_cont / wheat_nopath > 0.85);
    // nutrients are refilled: the continuous crop does not decline over the decades (take-all decline is long past by year 11)
    CHECK(mean_years(cont, 21, 30) >= 0.9 * mean_years(cont, 1, 10));
    // the rotation does not decline either (wheat years)
    CHECK(mean_years(rot, 20, 29, 3, 0) >= 0.9 * mean_years(rot, 2, 11, 3, 0));
}

TEST_CASE("Y4 unfertilised wheat: nothing applied, a low yield that declines, a humus that falls", "[field][scenario30][long]") {
    const FieldScenario sc = load(kUnf);
    const Played p = play(sc, 0);
    CHECK(p.n_applied == 0.0);
    CHECK(p.p_applied == 0.0);
    CHECK(p.k_applied == 0.0);
    CHECK(p.cao_applied > 0.0);   // the chalk is kept, as on the Broadbalk nil plot (which still gets chalk and pesticides)
    const std::map<int, double> unf = grain_by_year(base_run(kUnf)), cont = grain_by_year(base_run(kCont));
    const double level = mean_years(unf, 11, 30), fert = mean_years(cont, 11, 30);
    const double early = mean_years(unf, 1, 5), late = mean_years(unf, 26, 30);
    std::printf("Y4 unfertilised grain: years 1-5 %.2f, 26-30 %.2f, 11-30 %.2f t/ha dm; fertilised continuous 11-30 %.2f (ratio %.3f)\n", early, late,
                level, fert, level / fert);
    CHECK(level / fert < 0.5);            // far below the fertilised crop
    CHECK(level > 0.15 * fert);           // but not a failed crop: soil N and deposition feed it
    CHECK(late < 0.95 * early);           // it declines
    CHECK(late > 0.5 * early);            // slowly
    // humus: falls without fertiliser, rises on the fertilised field (more residue returned)
    const FieldScenario scf = load(kCont);
    const Played pf = play(scf, 0);
    const double h0 = 58500.0;
    std::printf("Y4 humus after 31 years: unfertilised %.2f, fertilised %.2f t C/ha (start %.2f)\n", p.hum[0] / 1000.0, pf.hum[0] / 1000.0, h0 / 1000.0);
    CHECK(p.hum[0] < h0);
    CHECK(pf.hum[0] > h0);
    CHECK(p.hum[0] < pf.hum[0] - 3000.0);
}

TEST_CASE("Y5 the unfertilised level against the Broadbalk nil plot: about 1 t/ha at 85 % dry matter (loess)", "[field][scenario30][long][!mayfail]") {
    const std::map<int, double> unf = grain_by_year(base_run(kUnf));
    const double at85 = mean_years(unf, 11, 30) / 0.85;
    std::printf("Y5 loess, unfertilised, years 11-30: %.2f t/ha dm = %.2f t/ha at 85 %% dm (Broadbalk nil plot ~1)\n", mean_years(unf, 11, 30), at85);
    CHECK(at85 > 0.5);
    CHECK(at85 < 1.5);
}

TEST_CASE("Y5b the same on the sandy loam (1.0 % carbon): closer to Broadbalk, still above it", "[field][scenario30][long]") {
    FieldScenario sc = load(kUnf);
    sc.field.nx = 1;
    sc.field.ny = 1;
    sc.field.cell_soil.assign(1, 1);
    const std::map<int, double> sandy = grain_by_year(ps::io::run_field_scenario(sc));
    const std::map<int, double> loess = grain_by_year(base_run(kUnf));
    const double at85 = mean_years(sandy, 11, 30) / 0.85;
    std::printf("Y5b sandy loam, unfertilised, years 11-30: %.2f t/ha dm = %.2f t/ha at 85 %% dm (loess %.2f; Broadbalk nil plot ~1)\n",
                mean_years(sandy, 11, 30), at85, mean_years(loess, 11, 30) / 0.85);
    CHECK(mean_years(sandy, 11, 30) < mean_years(loess, 11, 30));
    CHECK(at85 > 1.0);   // the measured gap: the model's supply (humus mineralisation plus the 20 kg N/ha/yr deposition) feeds more than the Broadbalk soil does
}
