// engine_cycle/combustion_variation.h - seeded cycle-to-cycle combustion
// variation (ps_audio A7, out/audio_plan.md section 1 "Cycle-to-cycle
// variation"). AUDIO ONLY: the map generator (CycleEngine, map_generator.cpp)
// never calls it, so maps and their content hash are seed-independent (test
// "A7 V11" pins that). Header-only, part of ps_engine_cycle's public headers
// so the one sabotage that matters (the generator calling it) is a one-line
// edit, not a link change.
//
// Per cylinder and per combustion event: u_i = SplitMix64 outputs of a
// counter key (seed, cylinder, event index, stream), mapped to an approximate
// standard normal as sqrt(3) * (sum of 4 uniforms - 2) (variance 4/12 * 3 = 1,
// bounded to +-3.46 sigma). It perturbs the Wiebe burn duration (relative)
// and the start of burn (the spark-to-start-of-burn delay, degrees).
//
// Magnitudes are PROVISIONAL [A] (out/audio_a7_progress.md): target COV of
// IMEP about 2 % at part/full load rising to about 6 % at idle (Heywood 1988
// section 9.4 [A: page not verified]); sigma grows linearly from full load to
// idle_scale x at zero manifold pressure ratio.
//
// Deterministic: integer arithmetic plus + - * / on doubles; no statics.
#pragma once

#include "ps/types.h"

#include <cstdint>

namespace ps::engine_cycle {

struct CombustionVariationDesc {
    real burn_rel_sigma = 0.06;   // relative sigma of the Wiebe duration at full load [A]
    real start_sigma_deg = 1.0;   // sigma of the start-of-burn shift at full load [deg, A]
    real idle_scale = 3.0;        // sigma multiplier at p_im / p_amb -> 0 [A]
};

struct CombustionPerturbation {
    real burn_scale = 1.0;       // multiplies the burn duration (>= 0.3)
    real start_shift_deg = 0.0;  // added to the start of burn
};

inline std::uint64_t splitmix64_next(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

// Uniform in [0, 1) with 53 random bits (exact in double).
inline real unit_uniform(std::uint64_t bits) { return static_cast<real>(bits >> 11) * (1.0 / 9007199254740992.0); }

// Approximate standard normal for (seed, cylinder, event, stream).
inline real variation_normal(std::uint64_t seed, int cylinder, std::uint64_t event, int stream) {
    std::uint64_t s = seed;
    s ^= 0x632be59bd9b4e019ull * (static_cast<std::uint64_t>(cylinder) + 1u);
    s ^= 0x8cb92ba72f3d8dd7ull * (event + 1u);
    s ^= 0x2545f4914f6cdd1dull * (static_cast<std::uint64_t>(stream) + 1u);
    real sum = 0.0;
    for (int i = 0; i < 4; ++i) sum += unit_uniform(splitmix64_next(s));
    return 1.7320508075688772 * (sum - 2.0);
}

// load_ratio = p_im / p_amb (clamped to [0, 1]); scale multiplies both sigmas
// (0 = no variation).
inline CombustionPerturbation combustion_variation(const CombustionVariationDesc& d, std::uint64_t seed, int cylinder,
                                                   std::uint64_t event, real load_ratio, real scale) {
    const real lr = load_ratio < 0.0 ? 0.0 : (load_ratio > 1.0 ? 1.0 : load_ratio);
    const real k = scale * (d.idle_scale + (1.0 - d.idle_scale) * lr);
    CombustionPerturbation p;
    p.burn_scale = 1.0 + k * d.burn_rel_sigma * variation_normal(seed, cylinder, event, 0);
    if (p.burn_scale < 0.3) p.burn_scale = 0.3;
    p.start_shift_deg = k * d.start_sigma_deg * variation_normal(seed, cylinder, event, 1);
    return p;
}

} // namespace ps::engine_cycle
