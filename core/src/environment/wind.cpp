// ps/environment/wind.h implementation (weather plan 3.8, W4).

#include "ps/environment/wind.h"

#include "weather_rng.h"

#include "ps/math/transcendental.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace ps::environment {

namespace {

constexpr real kTwoPi = 2.0 * 3.14159265358979323846;
constexpr real kHalfPi = 0.5 * 3.14159265358979323846;

// Line position inside its band as a fraction of the band width: [25 %, 75 %],
// so a line never sits on a band edge (octave-band power estimates of a record
// then see exactly the lines of their bands).
constexpr real kLineFractionLo = 0.25;
constexpr real kLineFractionSpan = 0.5;

} // namespace

real gust_base_intensity(const GustDesc& d) {
    return kWmoTurbulenceCu * kVonKarman / math::log(d.reference_height_m / d.roughness_length_m);
}

real gust_sigma_ratio(const GustDesc& d, real gust_factor) {
    if (!(gust_factor > 1.0)) return 0.0;
    return gust_base_intensity(d) * (gust_factor - 1.0) / (d.reference_gust_factor - 1.0);
}

GustField::GustField(const GustDesc& desc, std::uint64_t seed) : desc_(desc), seed_(seed) {
    const std::string e = gust_desc_validation_error(desc);
    if (!e.empty()) throw std::invalid_argument("GustField: " + e);
    const real bpo = static_cast<real>(desc_.bands_per_octave);
    for (int j = 0; j <= kComponents; ++j)
        edge_hz_[j] = desc_.f_min_hz * math::pow(2.0, static_cast<real>(j) / bpo);
    for (int axis = 0; axis < kAxes; ++axis) {
        for (int k = 0; k < kComponents; ++k) {
            const real r = kLineFractionLo + kLineFractionSpan *
                           detail::rng_uniform(detail::rng_key(seed_, detail::kDomainGustFrequency,
                                                               static_cast<std::uint64_t>(axis),
                                                               static_cast<std::uint64_t>(k), 0));
            const real f = edge_hz_[k] + r * (edge_hz_[k + 1] - edge_hz_[k]);
            omega_[axis][k] = kTwoPi * f;
        }
    }
}

void GustField::amplitudes(real mean_speed_m_s, real sigma_u_m_s, Amplitudes& out) const {
    if (!(sigma_u_m_s > 0.0)) {
        for (int axis = 0; axis < kAxes; ++axis)
            for (int k = 0; k < kComponents; ++k) out.a[axis][k] = 0.0;
        return;
    }
    const real u = mean_speed_m_s > desc_.min_advection_speed_m_s ? mean_speed_m_s : desc_.min_advection_speed_m_s;
    const real lambda = kKaimalLambdaPerHeight * desc_.reference_height_m;
    for (int axis = 0; axis < kAxes; ++axis) {
        const real sigma = kGustSigmaRatio[axis] * sigma_u_m_s;
        const real var2 = 2.0 * sigma * sigma;
        const real c = 6.0 * kGustLengthFactor[axis] * lambda / u;
        // Cumulative variance complement C(f) = (1 + 6 f L / U)^(-2/3), decreasing in f.
        real prev = math::pow(1.0 + c * edge_hz_[0], -2.0 / 3.0);
        for (int k = 0; k < kComponents; ++k) {
            const real next = math::pow(1.0 + c * edge_hz_[k + 1], -2.0 / 3.0);
            out.a[axis][k] = std::sqrt(var2 * (prev - next));
            prev = next;
        }
    }
}

void GustField::fill_phases(std::int64_t window, Phases& out) const {
    out.window = window;
    const std::uint64_t w = static_cast<std::uint64_t>(window);
    for (int axis = 0; axis < kAxes; ++axis)
        for (int k = 0; k < kComponents; ++k)
            out.phi[axis][k] =
                kTwoPi * detail::rng_uniform(detail::rng_key(seed_, detail::kDomainGustPhase, w,
                                                             static_cast<std::uint64_t>(axis),
                                                             static_cast<std::uint64_t>(k)));
}

Vec3 GustField::evaluate(const Amplitudes& amps, const Phases& phases, real dir_x, real dir_y,
                         real advect_speed_m_s, real t_gust, real x, real y) const {
    const real adv = advect_speed_m_s > desc_.min_advection_speed_m_s ? advect_speed_m_s : desc_.min_advection_speed_m_s;
    // Frozen turbulence: the pattern at x is the pattern at the origin delayed by the travel time along the wind.
    const real tau = t_gust - (x * dir_x + y * dir_y) / adv;
    real s[kAxes];
    for (int axis = 0; axis < kAxes; ++axis) {
        real sum = 0.0;
        for (int k = 0; k < kComponents; ++k)
            sum += amps.a[axis][k] * math::cos(omega_[axis][k] * tau + phases.phi[axis][k]);
        s[axis] = sum;
    }
    return Vec3{s[0] * dir_x - s[1] * dir_y, s[0] * dir_y + s[1] * dir_x, s[2]};
}

std::int64_t GustField::window_index(real t_gust, real window_s) {
    return static_cast<std::int64_t>(std::floor(t_gust / window_s));
}

GustField::Blend GustField::blend(real t_gust) const {
    Blend b;
    b.window = window_index(t_gust, desc_.window_s);
    if (!(desc_.blend_s > 0.0)) return b;
    const real tin = t_gust - static_cast<real>(b.window) * desc_.window_s;
    if (!(tin < desc_.blend_s)) return b;
    const real s = tin / desc_.blend_s;            // [0, 1)
    const real sm = s * s * (3.0 - 2.0 * s);       // smoothstep
    real sn = 0.0, cs = 1.0;
    math::sincos(kHalfPi * sm, sn, cs);
    b.w_new = sn;
    b.w_prev = cs;
    return b;
}

} // namespace ps::environment
