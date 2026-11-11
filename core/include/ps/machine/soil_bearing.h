// ps/machine/soil_bearing.h - what a wheel does to soft ground (soft soil plan, docs/farm/soft_soil.md in farming_sim_hud; physics_sim PLAN P8).
//
// A Bekker sinkage law for one wheel on soil, plus the two forces it gives the wheel: a motion resistance (the work of pressing the soil
// down) and a thrust limit (Mohr-Coulomb shear). Pure functions of the soil's parameters and the wheel's load, width and radius; no state,
// no allocation, deterministic (a fixed-step bisection, ps::math for the powers). The game supplies the parameters per place through
// ISoilSource::bearing(); a place that supplies none (valid = false: a road, a yard, frozen ground, an adapter with no soft soil) leaves the
// wheel exactly as it was.
//
// Sources: Bekker, "Theory of Land Locomotion" (1956): p = (k_c/b + k_phi) z^n; Wong, "Theory of Ground Vehicles" 4th ed. section 2.4: a rigid
// wheel's motion resistance R_c = b (k_c/b + k_phi) z^(n+1)/(n+1) and the contact length l = sqrt(D z - z^2) of its front half;
// Janosi and Hanamoto (1961) for the peak thrust A c + W tan(phi). [D] the wheel is treated as rigid: the tyre's own carcass deflection
// is the Pacejka model's, the soil only sinks under what the carcass presses on it.

#pragma once

#include "ps/math/transcendental.h"
#include "ps/types.h"

#include <algorithm>
#include <cmath>

namespace ps::machine {

struct SoilBearing {
    bool valid = false;     // false: this place has no soft soil, the wheel behaves as it always did
    real kc = 0.0;          // N/m^(n+1): the cohesive modulus
    real kphi = 0.0;        // N/m^(n+2): the frictional modulus
    real n = 1.0;           // the sinkage exponent
    real cohesion_pa = 0.0; // Mohr-Coulomb cohesion c
    real phi_rad = 0.0;     // internal friction angle
    bool operator==(const SoilBearing&) const = default;
};

struct WheelSoilResponse {
    real sinkage_m = 0.0;        // how deep the wheel sits in the soil
    real patch_length_m = 0.0;   // length of the contact patch (front half of the wheel)
    real resistance_n = 0.0;     // the compaction resistance, opposing the rolling
    real thrust_limit_n = 0.0;   // the most longitudinal force the soil can give this wheel
    bool bottomed = false;       // the wheel sank to its axle: the soil cannot carry it at all
};

// One wheel of load `fz` (N), tyre width `width` (m) and radius `radius` (m). With no load or invalid soil: all zero and an unlimited thrust.
[[nodiscard]] inline WheelSoilResponse wheel_soil_response(const SoilBearing& s, real fz, real width, real radius) {
    WheelSoilResponse out;
    out.thrust_limit_n = 1.0e30;
    if (!s.valid || !(fz > 0.0) || !(width > 0.0) || !(radius > 0.0)) return out;
    const real modulus = s.kc / width + s.kphi;
    const real d = 2.0 * radius;
    // the load a wheel sunk to z carries: pressure x patch area; strictly increasing in z, so a fixed bisection finds the root
    auto carried = [&](real z) { return modulus * ps::math::pow(z, s.n) * width * std::sqrt(std::max(real(0.0), d * z - z * z)); };
    if (carried(radius) < fz) { // the soil gives way before the axle: the wheel is down to its hub
        out.sinkage_m = radius;
        out.bottomed = true;
    } else {
        real lo = 0.0, hi = radius;
        for (int i = 0; i < 60; ++i) {
            const real mid = 0.5 * (lo + hi);
            (carried(mid) < fz ? lo : hi) = mid;
        }
        out.sinkage_m = 0.5 * (lo + hi);
    }
    const real z = out.sinkage_m;
    out.patch_length_m = std::sqrt(std::max(real(0.0), d * z - z * z));
    out.resistance_n = width * modulus * ps::math::pow(z, s.n + 1.0) / (s.n + 1.0);
    out.thrust_limit_n = width * out.patch_length_m * s.cohesion_pa + fz * ps::math::tan(s.phi_rad);
    return out;
}

} // namespace ps::machine
