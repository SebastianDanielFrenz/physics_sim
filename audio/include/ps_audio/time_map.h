// ps_audio/time_map.h - the audio-clock -> simulation-time map of one voice
// (ps_audio A7, out/audio_plan.md section 4 "Time map"). Pure arithmetic, no
// threads, no allocation: the voice feeds it snapshot arrivals and advances it
// per sample; tests drive it standalone.
//
// The read point t_read (simulation time the voice renders at) advances at a
// rate r per audio second. A type-2 PLL, updated once per arrival, steers r so
// that at each arrival t_read sits 1.5 snapshot intervals behind the newest
// snapshot (it then sweeps from 1.5 to 0.5 intervals behind until the next
// arrival: on average one interval behind, always inside a bracketing pair).
// r is clamped to [rate_min, rate_max]. The rate only moves the READ POINT;
// pitch comes from the interpolated omega, never from r.
//
// Hitch: no arrival for more than hitch_intervals snapshot intervals of audio
// time -> hitched (t_read stops at the newest snapshot; the voice holds the
// last inputs). Resume: the next arrival jumps t_read to its target lag, with
// no catch-up and the loop integrator kept. reset_to() (pause resume) does the
// same jump.
//
// Deterministic: + - * / only.
#pragma once

#include "ps/types.h"

#include <cstdint>

namespace ps_audio {

using ps::real;

struct TimeMapSettings {
    real interval_s = 1.0 / 240.0;  // nominal snapshot interval (physics tick)
    real target_lag_intervals = 1.5;  // lag of t_read behind the newest snapshot at an arrival
    real natural_hz = 0.2;          // PLL natural frequency [A]
    real damping = 0.707;           // PLL damping ratio [A]
    real rate_min = 0.5;
    real rate_max = 1.02;
    real hitch_intervals = 2.0;     // no arrival for this many intervals -> hitch
    real lock_tolerance = 0.002;    // |r - 1| <= this and not hitched -> locked
};

class TimeMap {
public:
    explicit TimeMap(const TimeMapSettings& s = {});

    // A snapshot with simulation time t_sim arrived at audio time t_audio.
    void on_arrival(real t_sim, real t_audio);
    // An arrival while the voice is paused (frozen): records the newest
    // snapshot only - no steering - and re-seats at the first arrival after
    // reset_to_newest().
    void on_arrival_paused(real t_sim, real t_audio);
    // Advance the read point by dt_audio seconds of audio time (clamped at
    // the newest snapshot).
    void advance(real dt_audio);
    // Jump t_read to its target lag behind the newest snapshot (pause resume).
    void reset_to_newest();

    [[nodiscard]] real read_time() const { return t_read_; }
    [[nodiscard]] real rate() const { return rate_; }
    [[nodiscard]] real newest() const { return t_newest_; }
    [[nodiscard]] bool started() const { return started_; }
    [[nodiscard]] bool hitched() const { return hitched_; }
    [[nodiscard]] bool locked() const;
    [[nodiscard]] std::uint64_t hitch_count() const { return hitches_; }
    [[nodiscard]] std::uint64_t arrivals() const { return arrivals_; }
    // Lag of t_read behind the newest snapshot, in snapshot intervals.
    [[nodiscard]] real lag_intervals() const { return (t_newest_ - t_read_) / s_.interval_s; }
    // Audio time [s] since the last arrival (tracked by advance()).
    [[nodiscard]] real since_arrival() const { return since_arrival_; }
    [[nodiscard]] const TimeMapSettings& settings() const { return s_; }

private:
    TimeMapSettings s_;
    real kp_ = 0.0, ki_ = 0.0;
    real t_read_ = 0.0;
    real t_newest_ = 0.0;
    real rate_ = 1.0;
    real integ_ = 0.0;           // integral part of (rate - 1)
    real last_arrival_audio_ = 0.0;
    real since_arrival_ = 0.0;
    bool started_ = false;
    bool hitched_ = false;
    bool jump_pending_ = false;  // next arrival re-seats t_read
    std::uint64_t hitches_ = 0;
    std::uint64_t arrivals_ = 0;
};

}  // namespace ps_audio
