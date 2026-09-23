// ps/vehicle/vehicle_desc.h — public, additive descriptor types for
// PLAN.md 5.1/P3: per-corner suspension + tyre + steering + brakes + a
// placeholder drive, assembled into one vehicle on a chassis rigid body.
//
// Pure data description; no Jolt type, no algorithm code (that lives in
// core/src/vehicle, internal - same D2/D10 pattern as
// ps/articulation/articulation_desc.h). A World consumes a VehicleDesc via
// World::create_vehicle (ps/world/world.h).

#pragma once

#include "ps/math/pose.h"
#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps/vehicle/tyre_desc.h"

#include <string>
#include <vector>

namespace ps::vehicle {

// A piecewise-linear table, e.g. damper force vs. velocity or camber vs.
// suspension travel (PLAN.md 5.1: "a kinematic table interface for camber/
// toe/track change vs travel"). Points must be sorted by x (ascending);
// evaluate() clamps outside the given range rather than extrapolating.
struct Curve1D {
    std::vector<real> x;
    std::vector<real> y;

    [[nodiscard]] real evaluate(real xv) const {
        if (x.empty()) return 0.0;
        if (xv <= x.front()) return y.front();
        if (xv >= x.back()) return y.back();
        for (std::size_t i = 1; i < x.size(); ++i) {
            if (xv <= x[i]) {
                const real t = (xv - x[i - 1]) / (x[i] - x[i - 1]);
                return y[i - 1] + t * (y[i] - y[i - 1]);
            }
        }
        return y.back();
    }
};

// Suspension travel is a scalar `x` in [0, travel_max] along `axis`
// (chassis-local): x = 0 is full extension (spring free length), x =
// travel_max is full bump. Wheel vertical position is measured kinematically
// each substep (a cylinder shape cast along `axis`, out/handover_p3.md
// decision 1), not integrated as a separate unsprung-mass state.
struct SuspensionDesc {
    Vec3 axis = Vec3{0.0, 0.0, -1.0}; // chassis-local, points from full extension toward full bump
    real travel_max = 0.09;           // m
    real spring_rate = 35000.0;       // N/m, used if spring_curve is empty
    Curve1D spring_curve;             // optional: force (N) vs compression x (m); overrides spring_rate if non-empty
    real spring_preload = 0.0;        // N, added to the spring force (ride-height setting)
    real damper_bump_rate = 3000.0;   // N*s/m for x increasing (bump), used if damper_curve empty
    real damper_rebound_rate = 4000.0;// N*s/m for x decreasing (rebound)
    Curve1D damper_curve;             // optional: force (N) vs compression rate xdot (m/s); overrides the two rates above if non-empty
    real bump_stop_rate = 400000.0;   // N/m, engages only past travel_max (soft/hard limit)
    // Kinematic tables vs. travel x (PLAN.md 5.1): start simple (a table),
    // double-wishbone/MacPherson-derived tables can replace the points
    // later without an interface change.
    Curve1D camber_vs_travel; // rad vs m; empty == 0 camber change with travel
    Curve1D toe_vs_travel;    // rad vs m; empty == 0 toe change with travel
    // Anti-roll bar: couples this wheel's compression to its pair's
    // (WheelDesc::arb_partner below); force = arb_rate * (x_self - x_partner).
    real arb_rate = 0.0; // N/m; 0 == no ARB on this corner
};

struct WheelDesc {
    std::string name; // e.g. "FL", "FR", "RL", "RR" - used in telemetry channel names
    Vec3 attachment_local = Vec3::zero(); // chassis-local, wheel centre at full extension (x=0)
    real static_camber = 0.0; // rad, added to camber_vs_travel(x)
    real static_toe = 0.0;    // rad, added to toe_vs_travel(x); + = toe-in for a front-left-style wheel (see steering.cpp for the sign convention applied per side)
    real wheel_radius = 0.31; // m
    real wheel_width = 0.205; // m (CylinderShape half_height*2)
    real wheel_mass = 18.0;   // kg (spin inertia only, PLAN.md section 6: unsprung body mass is not separately simulated - see out/handover_p3.md decision 1)
    real wheel_inertia = 0.9; // kg*m^2 about the spin axis (solid-disc-ish estimate unless overridden)

    SuspensionDesc suspension;
    int arb_partner = -1; // index into VehicleDesc::wheels of this wheel's anti-roll-bar pair, -1 == none

    TyreDesc tyre;

    bool steered = false;
    bool driven = false;
    bool is_front = true;  // picks BrakeDesc's front/rear split, SteeringDesc's Ackermann-vs-rear-steer formula, and telemetry grouping
};

// Ackermann steering geometry (PLAN.md 5.1). A single steer control channel
// ("steer", -1..1) maps to a rack position; each steered wheel's angle is
// then solved via Ackermann geometry so the front (or rear) wheels' turn
// centres coincide, using the vehicle's own track width and wheelbase
// (derived from wheel attachment points at build time, not stored here).
struct SteeringDesc {
    real max_wheel_angle = 0.5236; // rad (30 deg) at full steer input
    // Each steered wheel's angle is delta_avg + ackermann_fraction *
    // (delta_ackermann_geometric - delta_avg) (core/src/vehicle/
    // vehicle_instance.cpp's ackermann_wheel_angle): 0 = parallel steer
    // (every steered wheel gets the same angle, delta_avg), 1 = full
    // geometric Ackermann (each wheel's own turn centre coincides with the
    // others', so the INNER wheel - closer to the turn centre - gets the
    // larger-magnitude angle). NEGATIVE = anti-Ackermann, as used on some
    // race cars (e.g. some F1 setups): the OUTER wheel gets the larger-
    // magnitude angle instead, trading turn-centre coincidence for putting
    // the more heavily loaded outer tyre (which carries extra load from
    // cornering weight transfer) at a larger slip angle, closer to its own
    // (load-shifted) peak lateral-force slip angle. The magnitude |f| is
    // how far the wheel is pushed from parallel steer TOWARD (f>0) or AWAY
    // FROM (f<0) the geometric-Ackermann angle - e.g. f=-1 gives the inner
    // wheel the same angular offset from delta_avg as full Ackermann would,
    // but with the sign of that offset reversed, so the OUTER wheel ends up
    // with the larger magnitude. Validated/accepted range: [-1, 1]
    // (ps::io::load_vehicle_json rejects anything outside it) - 1 and -1
    // are each the "full" geometric effect in one direction or the other;
    // going further (e.g. -2) would overshoot the geometric correction by
    // more than the geometry itself justifies and is far more likely a
    // data-entry mistake than an intentional setting (no vehicle in this
    // project's data/vehicles/*.json needs more than +-1 - see
    // out/handover_anti_ackermann.md / vault PHYS-025 for the measured
    // effect on car_sedan.json's skidpad and limit-speed behaviour, and
    // ackermann_wheel_angle's own comment for the geometric-singularity
    // caveat this range does not remove: a vehicle with a very short
    // wheelbase relative to track can still see the underlying geometric
    // Ackermann term itself blow up near a tight-turn singularity,
    // independent of this fraction's sign).
    real ackermann_fraction = 1.0;
    real rear_steer_fraction = 0.0; // rear wheels steer this fraction of the equivalent front angle, opposite sign (0 = no rear steer)
    real steer_rate_limit = 0.0;    // rad/s; 0 == unlimited (instantaneous)
};

struct BrakeDesc {
    real max_torque_front = 2200.0; // N*m, per front wheel at brake=1
    real max_torque_rear = 1400.0;  // N*m, per rear wheel at brake=1
    real handbrake_rear_fraction = 1.0; // fraction of max_torque_rear the "handbrake" control channel commands
};

// Placeholder drive (PLAN.md M1: "throttle -> torque at the driven wheels
// from a simple torque curve, with a rev-like limit... clearly marked as a
// placeholder to be replaced in P4"). No gearbox/clutch: one fixed final
// drive ratio from a single notional "engine" speed derived from the
// driven wheels' own average angular speed.
//
// Because there is no clutch to slip/modulate torque during engagement,
// full throttle commanded before a driven wheel's tyre contact has settled
// to its load-bearing Fz (e.g. an instantaneous full-throttle command right
// at vehicle spawn, before the suspension has compressed) can spin that
// wheel up largely unopposed (reaction torque ~ Fx ~ Fz*mu, near zero while
// Fz is near zero) into the tyre's deep post-peak slip region before real
// grip develops - a genuine, understood limitation of this placeholder
// (no clutch/traction-control model), not a numerical bug. Measured
// (data/vehicles/car_sedan.json, out/handover_accel.md): self-recovers
// within several seconds as the car's own speed catches up to the wheel's
// surface speed (car keeps accelerating throughout, never stalls), but
// during the excursion Fx/Fz sits well below the tyre's achievable peak
// (~0.6 vs ~1.0 measured). A real full-throttle standing start from an
// ALREADY-SETTLED rest (the common case - see
// test_vehicle_accel_standing_start.cpp) does not trigger this. PLAN.md P4's
// real drivetrain (clutch engagement / launch control) is where this
// belongs properly; not addressed here.
struct DriveDesc {
    Curve1D torque_curve_nm_vs_rpm; // engine torque (N*m) vs rpm; required, non-empty
    real final_drive_ratio = 3.7;   // engine-to-wheel ratio (no gearbox in this placeholder)
    real rev_limit_rpm = 6500.0;    // torque is clamped to 0 above this (crude limiter, P4 replaces with a real one)
    real idle_rpm = 900.0;
    real drivetrain_efficiency = 0.9; // simple constant loss factor
};

struct VehicleDesc {
    std::string name;
    std::string source; // mass/geometry provenance (PLAN.md section 8)
    std::vector<WheelDesc> wheels;
    SteeringDesc steering;
    BrakeDesc brakes;
    DriveDesc drive;
};

} // namespace ps::vehicle
