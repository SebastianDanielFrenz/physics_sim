// tests/unit/test_field_persistence.cpp - lazy stepping, checkpoint/restore,
// thread-partitioned stepping, and the allocation-free daily step.
//
// Recorded sabotages (F1a, 2026-10-31), each made in core/src/field/field.cpp,
// the test run, then reverted:
//  - P-a: advance_to stopping one day short: P1 failed;
//  - P-b: restore() keeping the current biomass vector: P2 failed (the first
//         attempt, keeping t_sen, did not fail: t_sen is zero at both
//         checkpoint times, so the test could not see it);
//  - P-c: step_cell reading its right neighbour's depletion (a cross-cell
//         dependence): P3 failed (the threaded runs differ from the
//         sequential one; with one thread it cannot);
//  - P-d: a heap allocation (new[] / delete[]) inside step_cell: P4 failed.
// F1b (2026-10-31), made in core/src/field/nutrients.cpp / field.cpp, then reverted:
//  - P-e: a heap vector built inside step_nutrients (size depending on the cell, so it cannot be elided): P9 failed
//         (P4 does not see it: it runs with nutrients off; a first attempt with new char[8] / delete[] was optimised away
//         by the compiler and failed nothing);
//  - P-f: step_nutrients reading its right neighbour's nitrate (a cross-cell dependence): P8 failed;
//  - P-g: restore() keeping the slow organic N vector of the current state: P7 failed.

#include "field_test_support.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <barrier>
#include <cmath>
#include <cstdlib>
#include <new>
#include <thread>

// --- allocation counter (this executable only; ps_field_tests does not link ps_core) -------------
namespace {
std::atomic<bool> g_count_allocations{false};
std::atomic<long> g_allocations{0};
} // namespace

void* operator new(std::size_t n) {
    if (g_count_allocations.load(std::memory_order_relaxed)) g_allocations.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

using namespace ps;
using namespace ps::field;

namespace {

FieldConfig mixed_config(int nx = 3, int ny = 2) {
    FieldConfig cfg;
    cfg.nx = nx;
    cfg.ny = ny;
    cfg.soils = {test_field::loess(), test_field::sandy()};
    cfg.crops = {test_field::wheat()};
    for (int i = 0; i < nx * ny; ++i) cfg.cell_soil.push_back(static_cast<std::uint16_t>(i % 2));
    cfg.initial_depletion_fraction = 0.3;
    return cfg;
}

// The test operations: drill on doy 288 (also while a crop stands: replaces it), harvest on doy 212 from year 1.
void apply_ops(Field& f, const DailyWeather& w) {
    if (w.doy == 288) f.sow_all(1);
    if (w.doy == 212 && w.day > 365) f.harvest_all();
}

// Day by day.
void play(Field& f, DailyWeatherSource& src, std::int64_t last) {
    for (std::int64_t d = f.last_day() + 1; d <= last; ++d) {
        const DailyWeather& w = src.record(d);
        apply_ops(f, w);
        f.step_day(w);
    }
}

// Jumps: advance_to between the days an operation happens on.
void play_jumping(Field& f, DailyWeatherSource& src, std::int64_t last) {
    while (f.last_day() < last) {
        std::int64_t next = f.last_day() + 1;
        while (next < last) {
            const int doy = src.record(next).doy;
            if (doy == 288 || doy == 212) break;
            ++next;
        }
        f.advance_to(next - 1, src);                 // one jump over the idle stretch
        const DailyWeather& w = src.record(next);
        apply_ops(f, w);
        f.step_day(w);
    }
}

bool same_state(const Field& a, const Field& b) {
    if (a.fold_hash(1469598103934665603ull) != b.fold_hash(1469598103934665603ull)) return false;
    for (int c = 0; c < a.cell_count(); ++c) {
        const CellView x = a.view(c), y = b.view(c);
        if (x.d_root_mm != y.d_root_mm || x.d_sub_mm != y.d_sub_mm || x.cc != y.cc || x.biomass_g_m2 != y.biomass_g_m2 ||
            x.grain_g_m2 != y.grain_g_m2 || x.gdd_sow != y.gdd_sow || x.swe_mm != y.swe_mm || x.zr_m != y.zr_m)
            return false;
    }
    return true;
}

} // namespace

TEST_CASE("P1 advance_to in jumps equals day-by-day stepping bit for bit", "[field][persistence]") {
    const std::int64_t last = 900;
    DailyWeatherSource src_a(test_field::farm_climate(), 7);
    Field a(mixed_config());
    play(a, src_a, last);

    // Fresh source, asked out of order first (a record is a pure function of the day).
    DailyWeatherSource src_b(test_field::farm_climate(), 7);
    (void)src_b.record(last);
    (void)src_b.record(300);
    Field b(mixed_config());
    play_jumping(b, src_b, last);
    CHECK(b.last_day() == last);
    CHECK(same_state(a, b));
    CHECK(a.view(0).crop_id == 1);                // the run ends with a crop standing (drilled on doy 288, not yet harvested)

    // One single jump over the whole span without operations.
    DailyWeatherSource src_c(test_field::farm_climate(), 7);
    Field c(mixed_config()), d(mixed_config());
    c.advance_to(400, src_c);
    DailyWeatherSource src_d(test_field::farm_climate(), 7);
    src_d.prefetch(0, 400);
    for (std::int64_t day = 0; day <= 400; ++day) d.step_day(src_d.record(day));
    CHECK(same_state(c, d));

    // No time travel, no skipped days.
    CHECK_THROWS_AS(c.advance_to(399, src_c), std::invalid_argument);
    c.advance_to(400, src_c);                     // already there: nothing happens
    CHECK(c.last_day() == 400);
    CHECK_THROWS_AS(c.step_day(src_c.record(402)), std::invalid_argument);
    CHECK_THROWS_AS(c.step_day(src_c.record(400)), std::invalid_argument);
}

TEST_CASE("P2 checkpoint restore is bit exact", "[field][persistence]") {
    DailyWeatherSource src(test_field::farm_climate(), 5);
    Field f(mixed_config());
    play(f, src, 500);
    const FieldCheckpoint cp = f.checkpoint();
    const std::uint64_t h_cp = f.fold_hash(1469598103934665603ull);
    play(f, src, 1000);
    const std::uint64_t h_end = f.fold_hash(1469598103934665603ull);
    CHECK(h_end != h_cp);

    f.restore(cp);
    CHECK(f.last_day() == 500);
    CHECK(f.fold_hash(1469598103934665603ull) == h_cp);
    play(f, src, 1000);                           // the replay, operations included
    CHECK(f.fold_hash(1469598103934665603ull) == h_end);

    // Into a different Field object of the same grid.
    Field g(mixed_config());
    g.restore(cp);
    play(g, src, 1000);
    CHECK(same_state(f, g));

    // A checkpoint of another grid is refused.
    Field other(mixed_config(2, 2));
    CHECK_THROWS_AS(other.restore(cp), std::invalid_argument);
}

TEST_CASE("P3 cells in disjoint ranges can step on several threads", "[field][persistence]") {
    DailyWeatherSource src(test_field::farm_climate(), 5);
    src.prefetch(0, 600);
    const FieldConfig cfg = mixed_config(8, 8);
    Field seq(cfg);
    for (std::int64_t d = 0; d <= 600; ++d) {
        apply_ops(seq, src.record(d));
        seq.step_day(src.record(d));
    }
    const std::uint64_t want = seq.fold_hash(1469598103934665603ull);

    for (int threads : {1, 2, 3, 10}) {
        Field f(cfg);
        const int n = f.cell_count();
        // Persistent workers released once per day by a barrier; the main thread applies the operations and
        // closes the day (the same split a game loop would use).
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
        for (std::int64_t d = 0; d <= 600; ++d) {
            const DailyWeather& w = src.record(d);
            apply_ops(f, w);
            today = &w;
            start.arrive_and_wait();
            done.arrive_and_wait();
            f.finish_day(d);
        }
        stop = true;
        start.arrive_and_wait();
        for (std::thread& th : pool) th.join();
        INFO(threads << " threads");
        CHECK(f.last_day() == 600);
        CHECK(f.fold_hash(1469598103934665603ull) == want);
        CHECK(same_state(f, seq));
    }
    Field bad(cfg);
    CHECK_THROWS_AS(bad.step_cells(-1, 4, src.record(0)), std::out_of_range);
    CHECK_THROWS_AS(bad.step_cells(0, 65, src.record(0)), std::out_of_range);
    CHECK_THROWS_AS(bad.finish_day(5), std::invalid_argument);
}

TEST_CASE("P4 the daily step allocates nothing once warm", "[field][persistence]") {
    DailyWeatherSource src(test_field::farm_climate(), 3);
    src.prefetch(0, 900);
    Field f(mixed_config(8, 8));
    play(f, src, 5);                              // warm
    g_allocations.store(0);
    g_count_allocations.store(true);
    for (std::int64_t d = 6; d <= 900; ++d) {
        const DailyWeather& w = src.record(d);
        apply_ops(f, w);                          // sow and harvest too
        f.step_day(w);
    }
    // and the queries a client calls every frame
    double acc = 0.0;
    for (int c = 0; c < f.cell_count(); ++c) acc += f.view(c).cc + f.water_balance_residual_mm(c) + f.storage_mm(c);
    g_count_allocations.store(false);
    CHECK(g_allocations.load() == 0);
    CHECK(std::isfinite(acc));
    // The counter does count: a vector made while armed registers.
    g_allocations.store(0);
    g_count_allocations.store(true);
    std::vector<double> probe(16, 1.0);
    g_count_allocations.store(false);
    CHECK(g_allocations.load() >= 1);
    CHECK(probe.size() == 16);
}

TEST_CASE("P5 the grid is aligned to the world axes", "[field][persistence]") {
    FieldConfig cfg = mixed_config(5, 3);
    cfg.cell_size_m = 4.0;
    cfg.origin_x_m = 100.0;
    cfg.origin_y_m = -50.0;
    Field f(cfg);
    CHECK(f.cell_at_world(100.0, -50.0) == 0);
    CHECK(f.cell_at_world(103.999, -46.001) == 0);
    CHECK(f.cell_at_world(104.0, -50.0) == 1);
    CHECK(f.cell_at_world(100.0, -46.0) == 5);
    CHECK(f.cell_at_world(119.9, -38.1) == 14);
    CHECK(f.cell_at_world(99.999, -50.0) == -1);
    CHECK(f.cell_at_world(120.0, -50.0) == -1);
    CHECK(f.cell_at_world(100.0, -38.0) == -1);
    CHECK(f.cell_at_world(100.0, -50.001) == -1);
    CHECK(f.cell_index(4, 2) == 14);
    CHECK(f.cell_area_m2() == 16.0);
    // soils follow the cell, not the order of stepping
    CHECK(f.view(0).soil == 0);
    CHECK(f.view(1).soil == 1);
}

// ---- with nutrients (F1b) -----------------------------------------------------------------------

namespace {

FieldConfig mixed_nutrient(int nx = 3, int ny = 2) {
    FieldConfig cfg = test_field::nutrient_config(test_field::loess(), test_field::wheat(), nx, ny);
    cfg.soils = {test_field::loess(), test_field::sandy()};
    cfg.cell_soil.clear();
    for (int i = 0; i < nx * ny; ++i) cfg.cell_soil.push_back(static_cast<std::uint16_t>(i % 2));
    cfg.initial_depletion_fraction = 0.3;
    return cfg;
}

// Sow/harvest as above plus applications on fixed days: slurry (surface ammonia clock), mineral N, lime, a half-field pass.
struct NEffects {   // built once (name lookups allocate strings); the daily path only copies PODs
    OperationEffect slurry = {1.0, test_field::deposit("cattle_slurry_75", "broadcast", 20.0)};
    OperationEffect can = test_field::n_dose("can_27", "broadcast", 60.0, 1.0);
    OperationEffect uan = test_field::n_dose("uan_28", "broadcast", 40.0, 0.5);
    OperationEffect lime = {1.0, test_field::deposit("quicklime_90", "broadcast", 1.0)};
};
const NEffects& n_effects() {
    static const NEffects e;
    return e;
}

void apply_ops_n(Field& f, const DailyWeather& w) {
    const NEffects& e = n_effects();
    if (w.doy == 288) f.sow_all(1);
    if (w.doy == 212 && w.day > 365) f.harvest_all(0.5);
    if (w.doy == 60) f.apply_all(e.slurry);
    if (w.doy == 90) f.apply_all(e.can);
    if (w.doy == 120) f.apply_all(e.uan);
    if (w.doy == 300 && w.day > 365) f.apply_all(e.lime);
}

void play_n(Field& f, DailyWeatherSource& src, std::int64_t last) {
    for (std::int64_t d = f.last_day() + 1; d <= last; ++d) {
        const DailyWeather& w = src.record(d);
        apply_ops_n(f, w);
        f.step_day(w);
    }
}

bool same_nutrients(const Field& a, const Field& b) {
    if (a.fold_hash(1469598103934665603ull) != b.fold_hash(1469598103934665603ull)) return false;
    for (int c = 0; c < a.cell_count(); ++c) {
        const NutrientView x = a.nutrient_view(c), y = b.nutrient_view(c);
        const NutrientLedger lx = a.nutrient_ledger(c), ly = b.nutrient_ledger(c);
        if (x.nmin != y.nmin || x.nh4 != y.nh4 || x.tan_surface != y.tan_surface || x.lime_surface != y.lime_surface ||
            x.ph != y.ph || x.p_test_mg100g != y.p_test_mg100g || x.k_test_mg100g != y.k_test_mg100g || x.n_crop != y.n_crop ||
            lx.n_leached != ly.n_leached || lx.n_volatilised != ly.n_volatilised || lx.n_denitrified != ly.n_denitrified ||
            lx.n_mineralised != ly.n_mineralised || lx.cao_lost != ly.cao_lost || lx.n_removed != ly.n_removed)
            return false;
    }
    return true;
}

} // namespace

TEST_CASE("P6 with nutrients: jumps equal day-by-day stepping, applications included", "[field][persistence][nutrients]") {
    const std::int64_t last = 900;
    DailyWeatherSource src_a(test_field::farm_climate(), 7);
    Field a(mixed_nutrient());
    play_n(a, src_a, last);

    DailyWeatherSource src_b(test_field::farm_climate(), 7);
    (void)src_b.record(last);
    Field b(mixed_nutrient());
    while (b.last_day() < last) {
        std::int64_t next = b.last_day() + 1;
        while (next < last) {
            const int doy = src_b.record(next).doy;
            if (doy == 288 || doy == 212 || doy == 60 || doy == 90 || doy == 120 || doy == 300) break;
            ++next;
        }
        b.advance_to(next - 1, src_b);
        const DailyWeather& w = src_b.record(next);
        apply_ops_n(b, w);
        b.step_day(w);
    }
    CHECK(same_nutrients(a, b));
    CHECK(a.nutrient_ledger(0).n_volatilised > 1.0);   // the surface-ammonia clock really ran
    CHECK(a.nutrient_ledger(0).n_fert_mineral > 50.0);
}

TEST_CASE("P7 with nutrients: checkpoint restore is bit exact", "[field][persistence][nutrients]") {
    DailyWeatherSource src(test_field::farm_climate(), 5);
    Field f(mixed_nutrient());
    play_n(f, src, 420);                                // inside a spring with a fresh slurry deposit and a standing crop
    const FieldCheckpoint cp = f.checkpoint();
    const std::uint64_t h_cp = f.fold_hash(1469598103934665603ull);
    play_n(f, src, 1000);
    const std::uint64_t h_end = f.fold_hash(1469598103934665603ull);
    CHECK(h_end != h_cp);

    f.restore(cp);
    CHECK(f.fold_hash(1469598103934665603ull) == h_cp);
    play_n(f, src, 1000);
    CHECK(f.fold_hash(1469598103934665603ull) == h_end);

    Field g(mixed_nutrient());
    g.restore(cp);
    play_n(g, src, 1000);
    CHECK(same_nutrients(f, g));

    // a checkpoint from a field without nutrients does not fit a field with them
    Field plain(mixed_config());
    CHECK_THROWS_AS(plain.restore(cp), std::invalid_argument);
    CHECK_THROWS_AS(f.restore(plain.checkpoint()), std::invalid_argument);
}

TEST_CASE("P8 with nutrients: threads give the same state as one", "[field][persistence][nutrients]") {
    DailyWeatherSource src(test_field::farm_climate(), 5);
    src.prefetch(0, 600);
    const FieldConfig cfg = mixed_nutrient(8, 8);
    Field seq(cfg);
    for (std::int64_t d = 0; d <= 600; ++d) {
        apply_ops_n(seq, src.record(d));
        seq.step_day(src.record(d));
    }
    const std::uint64_t want = seq.fold_hash(1469598103934665603ull);
    for (int threads : {1, 2, 3, 10}) {
        Field f(cfg);
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
        for (std::int64_t d = 0; d <= 600; ++d) {
            const DailyWeather& w = src.record(d);
            apply_ops_n(f, w);
            today = &w;
            start.arrive_and_wait();
            done.arrive_and_wait();
            f.finish_day(d);
        }
        stop = true;
        start.arrive_and_wait();
        for (std::thread& th : pool) th.join();
        INFO(threads << " threads");
        CHECK(f.fold_hash(1469598103934665603ull) == want);
        CHECK(same_nutrients(f, seq));
    }
}

TEST_CASE("P9 with nutrients: the daily step and the applications allocate nothing once warm", "[field][persistence][nutrients]") {
    DailyWeatherSource src(test_field::farm_climate(), 3);
    src.prefetch(0, 900);
    Field f(mixed_nutrient(8, 8));
    play_n(f, src, 5);
    g_allocations.store(0);
    g_count_allocations.store(true);
    for (std::int64_t d = 6; d <= 900; ++d) {
        const DailyWeather& w = src.record(d);
        apply_ops_n(f, w);
        f.step_day(w);
    }
    double acc = 0.0;
    for (int c = 0; c < f.cell_count(); ++c) {
        const NutrientView v = f.nutrient_view(c);
        acc += v.nmin + v.ph + f.n_balance_residual(c) + f.nutrient_ledger(c).n_leached;
    }
    g_count_allocations.store(false);
    CHECK(g_allocations.load() == 0);
    CHECK(std::isfinite(acc));
}
