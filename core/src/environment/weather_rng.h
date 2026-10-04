// core/src/environment/weather_rng.h - INTERNAL counter-based randomness for
// the weather timeline (weather plan 3.1): every draw is a pure function of
// (seed, domain, a, b, slot), so a value never depends on evaluation order or
// on how often it was evaluated. Integer SplitMix64 mixing plus + - * / on
// doubles only (the same construction as ps/engine_cycle/combustion_variation.h,
// re-implemented so ps_environment does not depend on ps_engine_cycle).
#pragma once

#include "ps/environment/weather_desc.h"
#include "ps/math/transcendental.h"
#include "ps/types.h"

#include <cstdint>

namespace ps::environment::detail {

enum RngDomain : std::uint64_t {
    kDomainSegment = 1,    // a = segment index, slot = quantity
    kDomainCell = 2,       // a = segment index, b = cell index, slot = quantity
    kDomainLightning = 3,  // a = 60 s bin index, b = source key, slot = draw
};

// Slots of kDomainSegment.
enum SegmentSlot : std::uint64_t {
    kSlotRegime = 0, kSlotDuration, kSlotCloud, kSlotPrecip, kSlotWind, kSlotWindDir, kSlotTempOffset, kSlotRh,
    kSlotPressure
};
// Slots of kDomainCell.
enum CellSlot : std::uint64_t { kSlotGap = 0, kSlotCellDuration, kSlotPeak, kSlotHail, kSlotHailDiameter };

inline std::uint64_t splitmix64(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

inline std::uint64_t mix(std::uint64_t h, std::uint64_t v) {
    std::uint64_t s = h ^ (v * 0xd6e8feb86659fd93ull + 0x632be59bd9b4e019ull);
    return splitmix64(s);
}

inline std::uint64_t rng_key(std::uint64_t seed, std::uint64_t domain, std::uint64_t a, std::uint64_t b,
                             std::uint64_t slot) {
    return mix(mix(mix(mix(seed, domain), a), b), slot);
}

// Uniform in [0, 1) with 53 random bits (exact in double).
inline real unit_uniform(std::uint64_t bits) { return static_cast<real>(bits >> 11) * (1.0 / 9007199254740992.0); }

inline real rng_uniform(std::uint64_t key) {
    std::uint64_t s = key;
    return unit_uniform(splitmix64(s));
}

// Approximate standard normal: sqrt(3) * (sum of 4 uniforms - 2) (Irwin-Hall,
// variance 1, bounded to +-3.4641).
inline real rng_normal(std::uint64_t key) {
    std::uint64_t s = key ^ 0x2545f4914f6cdd1dull;
    real sum = 0.0;
    for (int i = 0; i < 4; ++i) sum += unit_uniform(splitmix64(s));
    return 1.7320508075688772 * (sum - 2.0);
}

inline real draw_lognormal(const LogNormalDesc& d, std::uint64_t key) {
    if (d.sigma_log == 0.0) return d.median;
    return d.median * ps::math::exp(d.sigma_log * rng_normal(key));
}

inline real draw_range(const RangeDesc& r, std::uint64_t key) {
    if (r.lo == r.hi) return r.lo;
    return r.lo + (r.hi - r.lo) * rng_uniform(key);
}

// Weibull: c * (-ln(1 - u))^(1/k); u in [0, 1) so 1 - u in (0, 1].
inline real draw_weibull(const WeibullDesc& w, std::uint64_t key) {
    if (w.c == 0.0) return 0.0;
    const real u = rng_uniform(key);
    const real e = -ps::math::log(1.0 - u);
    if (e == 0.0) return 0.0;
    return w.c * ps::math::pow(e, 1.0 / w.k);
}

// Poisson by inverse CDF with one uniform, capped at `cap`.
inline int draw_poisson(real mu, std::uint64_t key, int cap) {
    if (!(mu > 0.0)) return 0;
    const real u = rng_uniform(key);
    real p = ps::math::exp(-mu);
    real f = p;
    int k = 0;
    while (u >= f && k < cap) {
        ++k;
        p *= mu / static_cast<real>(k);
        f += p;
        if (p == 0.0) break;
    }
    return k;
}

} // namespace ps::environment::detail
