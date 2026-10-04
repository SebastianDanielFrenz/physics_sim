// ps/environment/env_time.h - environment clock (weather plan section 2.1, K2).
//
// Environment time t_env (double seconds since the climate epoch) is a PURE,
// NON-ACCUMULATING function of the simulation tick:
//
//     t_env = anchor_env_s + scale * (double(tick - anchor_tick) / tick_hz)
//
// Nothing is ever added up tick by tick, so every client and the server get
// bit-identical values for the same tick regardless of how they got there
// (jumps, late joins, different frame pacing). A double resolves ~1e-7 s at
// 1e9 s (31 years), enough. scale 0 freezes environment time (weather, sun
// and surface condition stop); scale 1 is real time; a game "48-minute day"
// on a 24 h authored day is scale 30.
//
// Tyre/vehicle physics always run in SIM time; only weather/sun/surface run in
// ENV time (plan 2.1). This header has no dependency beyond ps/types.h.

#pragma once

#include "ps/types.h"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace ps::environment {

struct EnvClock {
    real anchor_env_s = 0.0;     // environment time at anchor_tick
    std::int64_t anchor_tick = 0;
    real tick_hz = 240.0;        // > 0, finite
    real scale = 1.0;            // >= 0, finite (0 = frozen)

    // Throws std::invalid_argument for a non-finite anchor, tick_hz <= 0 or
    // non-finite, scale < 0 or non-finite.
    void validate() const {
        if (!std::isfinite(anchor_env_s))
            throw std::invalid_argument("EnvClock: anchor_env_s must be finite");
        if (!(tick_hz > 0.0) || !std::isfinite(tick_hz))
            throw std::invalid_argument("EnvClock: tick_hz must be > 0 and finite");
        if (!(scale >= 0.0) || !std::isfinite(scale))
            throw std::invalid_argument("EnvClock: scale must be >= 0 and finite");
    }

    // The closed form. The (tick - anchor_tick) difference is taken in int64
    // and converted once, so the result for a given tick never depends on any
    // history.
    real env_time(std::int64_t tick) const {
        return anchor_env_s + scale * (static_cast<real>(tick - anchor_tick) / tick_hz);
    }

    // Jump: environment time is `env_s` at `tick` from now on.
    void set_time(real env_s, std::int64_t tick) {
        anchor_env_s = env_s;
        anchor_tick = tick;
    }

    // Change the scale without a jump: re-anchors at `tick` at the value the
    // OLD scale gave there, so t_env is continuous.
    void set_scale(real new_scale, std::int64_t tick) {
        anchor_env_s = env_time(tick);
        anchor_tick = tick;
        scale = new_scale;
    }
};

} // namespace ps::environment
