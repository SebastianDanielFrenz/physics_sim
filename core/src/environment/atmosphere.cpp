// ps/environment/atmosphere.h implementation (weather plan W4).

#include "ps/environment/atmosphere.h"

#include "ps/environment/air_temperature.h"
#include "ps/math/transcendental.h"

#include <cmath>
#include <stdexcept>

namespace ps::environment {

namespace {

real clamp_altitude(real h) {
    if (h < kIsaMinAltitudeM) return kIsaMinAltitudeM;
    if (h > kIsaMaxAltitudeM) return kIsaMaxAltitudeM;
    return h;
}

// g0 / (R L): the barometric exponent of the lapse layer.
constexpr real kBarometricExponent = kIsaG0 / (kIsaGasConstantAir * kIsaLapseRate);

} // namespace

real isa_temperature_k(real sea_level_temperature_k, real altitude_m) {
    const real h = clamp_altitude(altitude_m);
    return sea_level_temperature_k - kIsaLapseRate * (h < kIsaTropopauseM ? h : kIsaTropopauseM);
}

real isa_pressure_pa(real sea_level_pressure_pa, real sea_level_temperature_k, real altitude_m) {
    const real h = clamp_altitude(altitude_m);
    const real t0 = sea_level_temperature_k;
    if (h <= kIsaTropopauseM) {
        const real t = t0 - kIsaLapseRate * h;
        return sea_level_pressure_pa * math::pow(t / t0, kBarometricExponent);
    }
    const real t11 = t0 - kIsaLapseRate * kIsaTropopauseM;
    const real p11 = sea_level_pressure_pa * math::pow(t11 / t0, kBarometricExponent);
    return p11 * math::exp(-kIsaG0 * (h - kIsaTropopauseM) / (kIsaGasConstantAir * t11));
}

real humid_air_density(real pressure_pa, real temperature_k, real vapour_pressure_pa) {
    const real e = vapour_pressure_pa;
    return (pressure_pa - e) / (kIsaGasConstantAir * temperature_k) + e / (kVapourGasConstant * temperature_k);
}

AtmosphereSample atmosphere_at(const SeaLevelAtmosphere& s, real altitude_m) {
    if (!std::isfinite(s.pressure_pa) || !(s.pressure_pa > 0.0))
        throw std::invalid_argument("atmosphere_at: sea-level pressure must be finite and > 0");
    if (!std::isfinite(s.temperature_k) || !(s.temperature_k > 100.0))
        throw std::invalid_argument("atmosphere_at: sea-level temperature must be finite and > 100 K");
    if (!std::isfinite(s.vapour_pressure_pa) || s.vapour_pressure_pa < 0.0)
        throw std::invalid_argument("atmosphere_at: vapour pressure must be finite and >= 0");
    if (!std::isfinite(altitude_m)) throw std::invalid_argument("atmosphere_at: altitude must be finite");

    AtmosphereSample a;
    a.altitude_m = clamp_altitude(altitude_m);
    a.temperature_k = isa_temperature_k(s.temperature_k, a.altitude_m);
    a.pressure_pa = isa_pressure_pa(s.pressure_pa, s.temperature_k, a.altitude_m);
    if (s.vapour_pressure_pa > 0.0) {
        // Constant vapour mole fraction, capped at saturation over water.
        real e = s.vapour_pressure_pa * (a.pressure_pa / s.pressure_pa);
        const real e_sat = 100.0 * saturation_vapour_pressure_hpa(a.temperature_k - 273.15);
        if (e > e_sat) e = e_sat;
        if (e > 0.99 * a.pressure_pa) e = 0.99 * a.pressure_pa;
        a.vapour_pressure_pa = e;
    }
    a.density_kg_m3 = humid_air_density(a.pressure_pa, a.temperature_k, a.vapour_pressure_pa);
    return a;
}

} // namespace ps::environment
