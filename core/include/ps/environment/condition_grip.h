// ps/environment/condition_grip.h - the surface-condition friction factor
// (weather plan 5.2; W6). One per-tick immutable object: for each surface class
// it turns the class's water/soil/canopy state and the tyre profile's
// per-condition factors into the `lambda_mu` multiplier a wheel contact uses.
//
// Combination (plan 5.2 steps 1-3 and 6): a weight w of the wet layer from the
// class state, a wet-speed table built from the PIARC speed constant
// Sp = 14.2 + 89.7 MPD [C PIARC 1995] and the class's wet/damp levels, and the
// tyre profile's p[dry]/p[wet]/p[wet_grass]. The result is RELATIVE TO THE
// CLASS'S DRY LEVEL: the SurfaceTable lambda_mu carries the absolute dry value
// and multiplies this factor (plan 5.2 step 6). The tread factor (step 4) and
// aquaplaning (step 5) are W7/W8 and not here.
//
// The speed table uses ps::math::exp but is built once per grid (the class
// MTD and levels are static), never per substep; a lookup takes the nearest
// sample (round to the nearest of kSpeedSamples). No allocation, not thread
// safe.

#pragma once

#include "ps/environment/surface_condition.h"
#include "ps/environment/tyre_profile_desc.h"

#include <array>

namespace ps::environment {

class ConditionGrip {
public:
    static constexpr int kSpeedSamples = 33;
    static constexpr real kSpeedMaxMps = 55.6;   // 200 km/h [AS]

    void reset();
    // Static part (class MTD/friction, one ps::math::exp per grid point):
    // called once after the grid is built or restored. `profile` is the
    // environment's default tyre profile (per-wheel profiles are a later
    // increment); its factors apply to every wheel in W6.
    void build_static(const ConditionGrid& grid, const TyreConditionFactors& profile);
    // Per-tick part: the wet weight of each class from the state interpolated
    // at t_env. Cheap (no transcendental).
    void update_state(const ConditionGrid& grid, real t_env);

    bool valid() const { return valid_; }
    int class_count() const { return count_; }
    const TyreConditionFactors& profile() const { return profile_; }

    // Wet weight of one class [0, 1] from the last update_state (diagnostic and
    // the snapshot's condition label).
    real water_weight(int surface_id) const;
    // The friction factor of one wheel contact. 1.0 when the object is invalid,
    // the id is out of range, or the class is static (no state). The speed is
    // the contact slip speed (m/s), clamped to [0, kSpeedMaxMps].
    real factor(int surface_id, real speed_m_s) const;
    real factor(int surface_id, const TyreConditionFactors& p, real speed_m_s) const;

private:
    real table_[kMaxSurfaceClasses][kSpeedSamples]{};   // wet mu_rel vs speed
    real w_wet_[kMaxSurfaceClasses]{};                  // water/soil/canopy weight
    real w_grass_[kMaxSurfaceClasses]{};                // vegetated canopy weight
    SurfaceFrictionDesc friction_[kMaxSurfaceClasses]{};
    bool stateful_[kMaxSurfaceClasses]{};
    bool vegetated_[kMaxSurfaceClasses]{};
    TyreConditionFactors profile_{};
    int count_ = 0;
    bool valid_ = false;
};

} // namespace ps::environment
