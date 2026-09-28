// ps_audio/halfband.h - 2:1 half-band FIR decimator (ps_audio A7), used when
// the load-time stability probe asks for two physics substeps per output
// sample. 31 taps, windowed sinc (Blackman), taps computed once with ps::math
// (deterministic across platforms), DC gain normalised to exactly the sum of
// the taps. Pass band to ~0.2 fs_out, about -58 dB from 0.3 fs_out [A: Blackman
// side lobes]; latency 7.5 output samples. PROVISIONAL (out/audio_a7_progress.md).
#pragma once

#include "ps/types.h"

#include <array>

namespace ps_audio {

using ps::real;

class HalfbandDecimator {
public:
    static constexpr int kTaps = 31;
    HalfbandDecimator();
    // Two input samples (in time order) -> one output sample.
    real push2(real x0, real x1);
    void reset();
    [[nodiscard]] real tap(int k) const { return taps_[static_cast<std::size_t>(k)]; }

private:
    std::array<real, kTaps> taps_{};
    std::array<real, kTaps> line_{};  // newest at index pos_
    int pos_ = 0;
    void push(real x);
};

}  // namespace ps_audio
