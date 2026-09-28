// ps_audio/halfband.cpp - see halfband.h.
#include "ps_audio/halfband.h"

#include "ps/math/transcendental.h"

namespace ps_audio {

namespace {
constexpr real kPi = 3.14159265358979323846264338327950288;
}

HalfbandDecimator::HalfbandDecimator() {
    // Built exactly symmetric with exact zeros: the centre tap is 0.5, the
    // even offsets are 0, and the odd offsets (0.5 * sinc(k / 2) = +-1/(pi k)
    // times a Blackman window) are scaled to sum to exactly 0.5 - so the DC
    // gain is 1 and the input Nyquist is rejected exactly.
    constexpr int half = kTaps / 2;
    taps_.fill(0.0);
    taps_[static_cast<std::size_t>(half)] = 0.5;
    real odd_sum = 0.0;
    for (int k = 1; k <= half; k += 2) {
        const real sign = ((k - 1) / 2) % 2 == 0 ? 1.0 : -1.0;
        const real x = static_cast<real>(half - k) / static_cast<real>(kTaps - 1);
        const real w = 0.42 - 0.5 * ps::math::cos(2.0 * kPi * x) + 0.08 * ps::math::cos(4.0 * kPi * x);
        const real t = sign / (kPi * static_cast<real>(k)) * w;
        taps_[static_cast<std::size_t>(half - k)] = t;
        taps_[static_cast<std::size_t>(half + k)] = t;
        odd_sum += 2.0 * t;
    }
    for (int k = 1; k <= half; k += 2) {
        taps_[static_cast<std::size_t>(half - k)] *= 0.5 / odd_sum;
        taps_[static_cast<std::size_t>(half + k)] = taps_[static_cast<std::size_t>(half - k)];
    }
}

void HalfbandDecimator::reset() {
    line_.fill(0.0);
    pos_ = 0;
}

void HalfbandDecimator::push(real x) {
    pos_ = pos_ == 0 ? kTaps - 1 : pos_ - 1;
    line_[static_cast<std::size_t>(pos_)] = x;
}

real HalfbandDecimator::push2(real x0, real x1) {
    push(x0);
    push(x1);
    real acc = 0.0;
    int j = pos_;
    for (int k = 0; k < kTaps; ++k) {
        acc += taps_[static_cast<std::size_t>(k)] * line_[static_cast<std::size_t>(j)];
        j = j + 1 == kTaps ? 0 : j + 1;
    }
    return acc;
}

}  // namespace ps_audio
