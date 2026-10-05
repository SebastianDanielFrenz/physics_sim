// ps/environment/wind.h - gust synthesis (weather plan 3.8, W4).
//
// The turbulent part of the wind is a PURE FUNCTION of (seed, sim time,
// position, mean-wind state): a sum of 32 sinusoids per axis (Shinozuka &
// Deodatis 1991 spectral representation) whose amplitudes follow a Kaimal
// spectrum, with the frozen-turbulence (Taylor) shift so every point sees the
// same field advected along the mean wind - two cars at different places see
// coherent gusts, not two independent noises. No state, no accumulation, no
// allocation per evaluation; everything that is not a function of time and
// position is tabulated once (frequencies at construction, amplitudes once per
// tick by the Environment, phases once per window).
//
//   u'(t, x) = sum_k a_k cos(omega_k (t - s(x)) + phi_k),   s(x) = (x . e) / U_adv
//
// u along the mean wind e, v across it (horizontal), w vertical, each with its
// own independent frequency lines and phases (so the three components are
// uncorrelated over a long record).
//
// Spectrum [C IEC 61400-1, Kaimal model, via a secondary source (the standard
// text itself was not re-read, plan section 15); Kaimal et al. 1972, QJRMS 98 -
// M]: one-sided, per Hz,
//   S_k(f) = sigma_k^2 (4 L_k / U) / (1 + 6 f L_k / U)^(5/3),
//   sigma_v = 0.8 sigma_u, sigma_w = 0.5 sigma_u,
//   L_u = 8.1 Lambda, L_v = 2.7 Lambda, L_w = 0.66 Lambda, Lambda = 0.7 z (z <= 60 m).
// Its cumulative form is closed, int_0^f S df = sigma^2 (1 - (1 + 6 f L / U)^(-2/3)),
// so every component carries the EXACT variance of its frequency band
// (a_k^2 / 2 = sigma^2 [C(f_lo) - C(f_hi)] with C = (1 + 6 f L / U)^(-2/3)); no
// quadrature error. The 32 bands are log-spaced from f_min_hz (default 1/600 Hz,
// the 10-min-mean cutoff) with bands_per_octave per octave, each line sits at a
// seed-hashed place inside its band (in [25 %, 75 %], never at an edge). Variance
// below f_min (the weather's own mean-wind variation) and above the last band
// edge is not synthesised: for sigma_u at U = 10 m/s, z = 10 m that is 3.6 % +
// 0.3 % of the variance (sigma_u 2 % low), larger at low wind (test E-W4).
//
// Mean-wind coupling (the Environment's job, wind.h only provides the pieces):
//   sigma_u(t) = U(t) * gust_sigma_ratio(desc, G(t))
//   gust_sigma_ratio = I (G - 1) / (G_ref - 1),  I = c_u kappa / ln(z / z0)
//     [C WMO-No. 8 Vol. I ch. 5, eq. 5.A.1, c_u = 2.2, kappa = 0.4]; G = the
//     weather's published gust factor (regime/cell/preset), G_ref =
//     GustDesc::reference_gust_factor. [D]
//
// Time base (plan 3.8, owner question Q6): t is the SIM clock (real seconds),
// not environment time, so a car at 200 km/h meets turbulence at physical
// frequencies whatever the game's time scale. Phases are re-seeded per WINDOW
// of GustDesc::window_s sim seconds (counter-hash of (seed, window, axis,
// component)); the advection velocity (direction and speed of the frozen
// pattern) is constant within a window - the published mean wind at the
// window's start - which keeps the Taylor shift free of the frequency
// modulation a time-varying advection velocity would impose on a point far
// from the origin. At a window boundary the new field is crossfaded in over
// blend_s with cos/sin weights (the two fields are independent, so the
// variance is preserved); GustField::blend() gives the weights.
//
// Cost: evaluate = 96 ps::math::cos + 96 multiply-adds (+ the same again while
// blending); amplitudes = 99 ps::math::pow + 96 sqrt, once per tick.

#pragma once

#include "ps/environment/weather_desc.h"
#include "ps/math/vec3.h"
#include "ps/types.h"

#include <cstdint>
#include <limits>

namespace ps::environment {

// Cited constants (not data: they define the model).
inline constexpr real kWmoTurbulenceCu = 2.2;        // [C WMO-No. 8 eq. 5.A.1]
inline constexpr real kVonKarman = 0.4;              // [C WMO-No. 8 eq. 5.A.1]
inline constexpr real kKaimalLambdaPerHeight = 0.7;  // Lambda = 0.7 z, z <= 60 m [C IEC 61400-1 via secondary]
inline constexpr real kGustSigmaRatio[3] = {1.0, 0.8, 0.5};        // sigma_u : sigma_v : sigma_w [C IEC 61400-1]
inline constexpr real kGustLengthFactor[3] = {8.1, 2.7, 0.66};     // L_k / Lambda [C IEC 61400-1]

// I = c_u kappa / ln(z / z0): sigma_u / U for a gust factor equal to
// reference_gust_factor (about 0.1515 at z = 10 m, z0 = 0.03 m).
real gust_base_intensity(const GustDesc& d);
// sigma_u / U at a gust factor G >= 1: I (G - 1) / (G_ref - 1); 0 at G = 1.
real gust_sigma_ratio(const GustDesc& d, real gust_factor);

class GustField {
public:
    static constexpr int kComponents = 32;
    static constexpr int kAxes = 3;   // 0 = u (along the mean wind), 1 = v (across, horizontal), 2 = w (vertical)

    // Amplitudes of every line [m/s], per tick (depends on U and sigma_u).
    struct Amplitudes {
        real a[kAxes][kComponents] = {};
    };
    // Random phases of one window [rad], per window.
    struct Phases {
        std::int64_t window = std::numeric_limits<std::int64_t>::min();   // "not filled"
        real phi[kAxes][kComponents] = {};
    };
    // The crossfade between a window and its predecessor at a time within it.
    struct Blend {
        std::int64_t window = 0;
        real w_new = 1.0;    // weight of window `window`   (sin of the blend angle)
        real w_prev = 0.0;   // weight of window `window-1` (cos of it); 0 = no blending, window-1 not needed
    };

    // std::invalid_argument for an invalid desc (gust_desc_validation_error).
    GustField(const GustDesc& desc, std::uint64_t seed);

    const GustDesc& desc() const { return desc_; }
    std::uint64_t seed() const { return seed_; }

    // Band edges (32 bands: band k is [band_edge_hz(k), band_edge_hz(k + 1)]) and the line frequency in it.
    real band_edge_hz(int j) const { return edge_hz_[j]; }
    real line_hz(int axis, int k) const { return omega_[axis][k] / (2.0 * 3.14159265358979323846); }

    // Fills the amplitudes for a mean speed U (floored at min_advection_speed)
    // and the along-wind standard deviation sigma_u [m/s] (>= 0). sigma_u 0
    // gives all zeros.
    void amplitudes(real mean_speed_m_s, real sigma_u_m_s, Amplitudes& out) const;

    void fill_phases(std::int64_t window, Phases& out) const;

    // The gust vector [m/s] at horizontal position (x, y) and sim time t_gust,
    // in the frame the unit horizontal direction (dir_x, dir_y) the air moves
    // TOWARD is given in (u along it, v along (-dir_y, dir_x), w = +z).
    // advect_speed_m_s is floored at min_advection_speed.
    Vec3 evaluate(const Amplitudes& amps, const Phases& phases, real dir_x, real dir_y, real advect_speed_m_s,
                  real t_gust, real x, real y) const;

    // Window index and crossfade weights at a sim time.
    static std::int64_t window_index(real t_gust, real window_s);
    Blend blend(real t_gust) const;

private:
    GustDesc desc_;
    std::uint64_t seed_ = 0;
    real edge_hz_[kComponents + 1] = {};
    real omega_[kAxes][kComponents] = {};
};

} // namespace ps::environment
