// ps_audio/time_map.cpp - see time_map.h.
#include "ps_audio/time_map.h"

namespace ps_audio {

namespace {
constexpr real kTwoPi = 6.283185307179586476925286766559;
}

TimeMap::TimeMap(const TimeMapSettings& s) : s_(s) {
    // Continuous-time loop on the arrival-sampled error e = target - t_read:
    // e'' + kp e' + ki e = 0 for a constant producer rate.
    const real wn = kTwoPi * s_.natural_hz;
    kp_ = 2.0 * s_.damping * wn;
    ki_ = wn * wn;
}

void TimeMap::on_arrival(real t_sim, real t_audio) {
    ++arrivals_;
    const real lag = s_.target_lag_intervals * s_.interval_s;
    if (!started_) {
        started_ = true;
        t_newest_ = t_sim;
        t_read_ = t_sim - lag;
        last_arrival_audio_ = t_audio;
        since_arrival_ = 0.0;
        return;
    }
    if (!(t_sim > t_newest_)) return;  // stale or duplicate snapshot: ignored
    t_newest_ = t_sim;
    const real target = t_sim - lag;
    if (hitched_ || jump_pending_) {
        // Resume: re-seat the read point, no catch-up, keep the rate estimate.
        if (target > t_read_) t_read_ = target;
        hitched_ = false;
        jump_pending_ = false;
    } else {
        const real e = target - t_read_;
        const real dt = t_audio - last_arrival_audio_;
        const real integ_new = integ_ + ki_ * e * dt;
        real rate = 1.0 + integ_new + kp_ * e;
        bool saturated = false;
        if (rate > s_.rate_max) { rate = s_.rate_max; saturated = e > 0.0; }
        if (rate < s_.rate_min) { rate = s_.rate_min; saturated = saturated || e < 0.0; }
        if (!saturated) integ_ = integ_new;  // anti-windup
        if (integ_ > s_.rate_max - 1.0) integ_ = s_.rate_max - 1.0;
        if (integ_ < s_.rate_min - 1.0) integ_ = s_.rate_min - 1.0;
        rate_ = rate;
    }
    last_arrival_audio_ = t_audio;
    since_arrival_ = 0.0;
}

void TimeMap::on_arrival_paused(real t_sim, real t_audio) {
    if (!started_) {
        on_arrival(t_sim, t_audio);
        return;
    }
    ++arrivals_;
    if (t_sim > t_newest_) t_newest_ = t_sim;
    last_arrival_audio_ = t_audio;
    since_arrival_ = 0.0;
}

void TimeMap::advance(real dt_audio) {
    if (!started_) return;
    since_arrival_ += dt_audio;
    if (!hitched_ && since_arrival_ > s_.hitch_intervals * s_.interval_s) {
        hitched_ = true;
        ++hitches_;
    }
    t_read_ += rate_ * dt_audio;
    if (t_read_ > t_newest_) t_read_ = t_newest_;
}

void TimeMap::reset_to_newest() {
    if (!started_) return;
    const real target = t_newest_ - s_.target_lag_intervals * s_.interval_s;
    if (target > t_read_) t_read_ = target;
    hitched_ = false;
    jump_pending_ = true;  // the next arrival re-seats again rather than steering
    since_arrival_ = 0.0;
}

bool TimeMap::locked() const {
    const real d = rate_ - 1.0;
    return started_ && !hitched_ && d <= s_.lock_tolerance && -d <= s_.lock_tolerance;
}

}  // namespace ps_audio
