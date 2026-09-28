// ps_audio A7: the time map / rate PLL, standalone (no physics).
// Sabotages and failing numbers: out/audio_a7_progress.md "Sabotages".
#include "ps_audio/halfband.h"
#include "ps_audio/spsc.h"
#include "ps_audio/time_map.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

using ps_audio::TimeMap;
using ps_audio::TimeMapSettings;
using ps::real;

namespace {

struct DriftResult {
    real max_lag = 0.0;        // intervals, after the first 30 s
    real min_lag = 1e9;
    real rate_end = 0.0;
    real max_rate_dev_late = 0.0;  // |rate - clock_ratio| over the last minute
    real mean_rate_late = 0.0;     // mean rate over the last minute
    std::uint64_t hitches = 0;
};

// Producer: snapshots every `interval` of simulation time, arriving every
// interval / clock_ratio seconds of audio time (clock_ratio > 1: the physics
// clock runs fast against the audio device). Consumer: advances every
// `control` output samples at 48 kHz, as the voice does.
DriftResult run_drift(real clock_ratio, real seconds) {
    TimeMapSettings s;
    TimeMap tm(s);
    const real fs = 48000.0;
    const int control = 32;
    const real dt = control / fs;
    const real arrival_dt = s.interval_s / clock_ratio;
    std::uint64_t k = 0;
    real next_arrival = 0.0;
    DriftResult r;
    std::uint64_t late = 0;
    const std::uint64_t steps = static_cast<std::uint64_t>(seconds / dt);
    for (std::uint64_t i = 0; i < steps; ++i) {
        const real t_audio = static_cast<real>(i) * dt;
        while (next_arrival <= t_audio) {
            tm.on_arrival(static_cast<real>(k) * s.interval_s, t_audio);
            ++k;
            next_arrival = static_cast<real>(k) * arrival_dt;
        }
        tm.advance(dt);
        if (t_audio > 30.0) {
            const real lag = tm.lag_intervals();
            if (lag > r.max_lag) r.max_lag = lag;
            if (lag < r.min_lag) r.min_lag = lag;
        }
        if (t_audio > seconds - 60.0) {
            const real d = std::fabs(tm.rate() - clock_ratio);
            if (d > r.max_rate_dev_late) r.max_rate_dev_late = d;
            r.mean_rate_late += tm.rate();
            ++late;
        }
    }
    r.mean_rate_late /= static_cast<real>(late);
    r.rate_end = tm.rate();
    r.hitches = tm.hitch_count();
    return r;
}

}  // namespace

TEST_CASE("A7 V12c: a 0.3 % clock mismatch keeps the read lag bounded over 10 simulated minutes", "[audio]") {
    // Catches: a rate loop that does not steer (lag drifts 0.3 % of 600 s =
    // 432 intervals), or a loop that oscillates / loses the bracket.
    for (real ratio : {1.003, 1.0 / 1.003}) {
        const DriftResult r = run_drift(ratio, 600.0);
        std::printf("A7 V12c: clock ratio %.6f: lag [%.4f, %.4f] intervals, late mean rate %.7f (max |rate - ratio| %.2e), hitches %llu\n",
                    ratio, r.min_lag, r.max_lag, r.mean_rate_late, r.max_rate_dev_late,
                    static_cast<unsigned long long>(r.hitches));
        CHECK(r.max_lag <= 2.0);
        CHECK(r.min_lag >= 0.0);
        // The rate jitters with the arrival quantisation (control grid 0.67 ms,
        // kp 1.78/s -> +-1.2e-3); only its mean has to match the clock ratio.
        CHECK(std::fabs(r.mean_rate_late - ratio) < 2e-5);
        CHECK(r.max_rate_dev_late < 3e-3);
        CHECK(r.hitches == 0);
    }
}

TEST_CASE("A7 T1: time map lock, hitch hold, resume jump without catch-up, rate clamp", "[audio]") {
    TimeMapSettings s;
    TimeMap tm(s);
    const real dt = 32.0 / 48000.0;
    real t_audio = 0.0;
    std::uint64_t k = 0;
    auto run = [&](real seconds, bool produce, real sim_step) {
        const std::uint64_t n = static_cast<std::uint64_t>(seconds / dt);
        real next = t_audio;
        for (std::uint64_t i = 0; i < n; ++i) {
            if (produce && t_audio >= next) {
                tm.on_arrival(static_cast<real>(k) * sim_step, t_audio);
                ++k;
                next += s.interval_s;
            }
            tm.advance(dt);
            t_audio += dt;
        }
    };
    run(20.0, true, s.interval_s);
    CHECK(tm.locked());
    CHECK(std::fabs(tm.rate() - 1.0) < 1e-3);
    CHECK(tm.lag_intervals() > 0.3);
    CHECK(tm.lag_intervals() < 1.7);
    const real rate_before = tm.rate();
    // 200 ms without snapshots: hitched, the read point stops at the newest.
    run(0.2, false, s.interval_s);
    CHECK(tm.hitched());
    CHECK(tm.hitch_count() == 1);
    CHECK(tm.read_time() == tm.newest());
    // Resume with the simulation 0.2 s further on: jump to the target lag, no catch-up.
    k += static_cast<std::uint64_t>(0.2 / s.interval_s);
    tm.on_arrival(static_cast<real>(k) * s.interval_s, t_audio);
    ++k;
    CHECK_FALSE(tm.hitched());
    CHECK(std::fabs(tm.lag_intervals() - s.target_lag_intervals) < 1e-9);
    CHECK(tm.rate() == rate_before);
    // A producer 5 % fast: the rate saturates at rate_max, never beyond.
    TimeMap fast(s);
    real ta = 0.0;
    std::uint64_t j = 0;
    real next = 0.0;
    for (int i = 0; i < 48000 * 20 / 32; ++i) {
        if (ta >= next) {
            fast.on_arrival(static_cast<real>(j++) * s.interval_s, ta);
            next += s.interval_s / 1.05;
        }
        fast.advance(dt);
        ta += dt;
        CHECK(fast.rate() <= s.rate_max);
        CHECK(fast.rate() >= s.rate_min);
    }
    CHECK(fast.rate() == s.rate_max);
    CHECK_FALSE(fast.locked());
}

TEST_CASE("A7 T2: SPSC queue and float ring capacity, order and all-or-nothing writes", "[audio]") {
    ps_audio::SpscQueue<int, 4> q;
    for (int i = 0; i < 4; ++i) CHECK(q.push(i));
    CHECK_FALSE(q.push(99));
    int v = -1;
    for (int i = 0; i < 4; ++i) {
        REQUIRE(q.pop(v));
        CHECK(v == i);
    }
    CHECK_FALSE(q.pop(v));
    ps_audio::SpscFloatRing ring(10);  // rounds to 16
    CHECK(ring.capacity() == 16);
    float buf[16] = {};
    for (int i = 0; i < 16; ++i) buf[i] = static_cast<float>(i);
    CHECK(ring.write(buf, 12));
    CHECK_FALSE(ring.write(buf, 5));
    float out[16] = {};
    CHECK(ring.read(out, 16) == 12);
    CHECK(out[11] == 11.0f);
}

TEST_CASE("A7 T3: half-band decimator taps and responses", "[audio]") {
    ps_audio::HalfbandDecimator d;
    real sum = 0.0;
    for (int k = 0; k < ps_audio::HalfbandDecimator::kTaps; ++k) {
        sum += d.tap(k);
        CHECK(d.tap(k) == d.tap(ps_audio::HalfbandDecimator::kTaps - 1 - k));  // symmetric
        if (k != ps_audio::HalfbandDecimator::kTaps / 2 && (k - ps_audio::HalfbandDecimator::kTaps / 2) % 2 == 0)
            CHECK(d.tap(k) == 0.0);  // half-band zeros
    }
    CHECK(std::fabs(sum - 1.0) < 1e-15);
    // DC passes with gain 1; the input Nyquist (+1, -1, ...) is removed.
    real y = 0.0;
    for (int i = 0; i < 64; ++i) y = d.push2(1.0, 1.0);
    CHECK(std::fabs(y - 1.0) < 1e-12);
    ps_audio::HalfbandDecimator n;
    for (int i = 0; i < 64; ++i) y = n.push2(1.0, -1.0);
    CHECK(std::fabs(y) < 1e-12);
}
