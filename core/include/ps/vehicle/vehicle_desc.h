// ps/vehicle/vehicle_desc.h — public, additive descriptor types for
// PLAN.md 5.1/P3: per-corner suspension + tyre + steering + brakes + a
// placeholder drive, assembled into one vehicle on a chassis rigid body.
//
// Pure data description; no Jolt type, no algorithm code (that lives in
// core/src/vehicle, internal - same D2/D10 pattern as
// ps/articulation/articulation_desc.h). A World consumes a VehicleDesc via
// World::create_vehicle (ps/world/world.h).

#pragma once

#include <stdexcept>

#include "ps/drivetrain/brake_system_desc.h"
#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/math/pose.h"
#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps/aero/aerodynamics.h"
#include "ps/vehicle/curve1d.h" // Curve1D (moved to its own header, PHYS-033 I2 commit 5a, to
                                 // break the vehicle_desc.h <-> powertrain_desc.h cycle below)
#include "ps/vehicle/tyre_desc.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "ps/vehicle/drive_modes_desc.h"
#include "ps/vehicle/epb_desc.h"
#include "ps/vehicle/traction_control.h"

#include <optional>
#include <string>
#include <vector>

namespace ps::vehicle {

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

    // W6 (weather plan 7.5): path or name of a physics_sim.tyre_profile/1 file
    // for the condition grip. Parsed here; W6 resolves every wheel through the
    // environment's default_tyre_profile (per-wheel lookup is W7). Empty =
    // the environment default.
    std::string tyre_profile;

    bool steered = false;
    // PHYS-033 I2 (owner ruling Q7): "driven" is gone - which wheels are
    // driven is now determined by VehicleDesc::powertrain's graph (which
    // wheel hubs a differential/gear_pair/engine chain reaches), not a
    // per-wheel flag. See core/src/vehicle/vehicle_instance.cpp's P3 shim
    // (temporary, deleted in I2 commit 5b) for how the runtime still
    // derives an equivalent bool from the graph.
    bool is_front = true;  // picks brake_system's corner match, SteeringDesc's Ackermann-vs-rear-steer formula, and telemetry grouping
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

// PHYS-033 I2 (owner ruling Q7, no legacy converter): the P3 placeholder
// BrakeDesc (front/rear split) and DriveDesc (one notional-engine torque
// curve + a single final-drive ratio, "throttle -> torque at the driven
// wheels", PLAN.md M1) are both gone as of the /2 data format. They are
// replaced by drivetrain::BrakeSystemDesc (direct-torque friction rows per
// corner) and drivetrain::PowertrainDesc (the P4 I2 shaft-network graph -
// engine/gear_pair/differential components joined by shafts, PHYS-033 P4
// I2 plan section 2) below. DriveDesc's documented placeholder-drive
// limitation (no clutch/traction-control: an instantaneous full-throttle
// command before a driven wheel's tyre contact has settled can spin that
// wheel up largely unopposed into deep post-peak slip before recovering,
// out/handover_accel.md) still applies to I2's own torque_map engine kind
// (core/include/ps/drivetrain/powertrain_desc.h's EngineLimiterDesc
// comment: "I2 semantics == the P3 placeholder") - I3's real drivetrain is
// where it is finally addressed.
// Legacy drag plus reusable coefficient surfaces/wakes/fans (ps/aero/aerodynamics.h).
// PHYS-033 P4 I3 plan section 4.4/6.6: a single bluff-body drag element at
// the chassis CoM, F_drag = -0.5*rho*CdA*|v|*v (VehicleInstance::step_tick,
// plan section 6.6) - drag_area_m2 IS CdA (Cd already folded in), not a
// separate Cd/frontal-area pair. 0 == no aero drag (I2/P3 behaviour).
using AeroDesc = aero::Desc;

// Contents mass (racing_game PR #22, owner: "count fuel, which means live or semi-live updates"): with `enabled`,
// the chassis body carries the fuel tank and N2O bottle contents on top of its own (dry) mass. The chassis BodyDesc
// mass is the DRY mass; World::create_vehicle adds the initial contents at once and World::step re-applies dry +
// contents at the tick boundary whenever the contents moved by update_threshold_kg or more since the last update (a
// tank refill through set_fuel_tank_mass included). The inertia is rescaled with the mass as at creation (uniform
// density, centre of mass unchanged): the contents are not a point mass at the tank. Off (the default) = every
// vehicle bit-identical to before. Deterministic: a pure function of the simulated contents.
struct ContentsMassDesc {
    bool enabled = false;
    real update_threshold_kg = 0.1; // > 0: semi-live, the body mass is never more than this off the true contents
};
inline void validate_contents_mass(const ContentsMassDesc& d) {
    if (!(d.update_threshold_kg > real(0.0)) || !(d.update_threshold_kg < real(1000.0)))
        throw std::runtime_error("contents_mass.update_threshold_kg must be in (0, 1000)");
}

struct VehicleDesc {
    ContentsMassDesc contents_mass;
    TractionControlDesc traction_control;
    // Assists I1: the assist unit electronics part (grade + traction-control mode table), attached by the loader
    // from an `electronics` controller entry or the user layer. Absent = TC exactly as before.
    std::optional<AssistUnitDesc> assist_unit;
    // Assists I7: the drive_modes electronics part (Eco / Normal / Sport), a car-specific part attached like the
    // assist unit (at most one). Absent = no assist.drive_mode channel, every vehicle bit-identical.
    std::optional<DriveModesDesc> drive_modes;
    // Q3 / I8: the electronic parking brake part (kind `epb`), at most one. Drives brake_system.handbrake; channel `epb`
    // registered only with it. Its state is hashed only with it.
    std::optional<EpbDesc> epb;
    std::string name;
    std::string source; // mass/geometry provenance (PLAN.md section 8)
    std::vector<WheelDesc> wheels;
    SteeringDesc steering;
    drivetrain::BrakeSystemDesc brake_system;
    drivetrain::PowertrainDesc powertrain;
    AeroDesc aero;
    // PHYS-033 P4 I3 plan section 4.2/9.1: at most one manual_tcu in I3
    // (loader-enforced). Empty == no controller at all (e.g. a future
    // AI-driven or scripted vehicle) - VehicleInstance then leaves the
    // gearbox at its data-file initial_gear forever (no gear_request/
    // counter channel has any effect without a TCU to arbitrate them).
    std::vector<drivetrain::ControllerDesc> controllers;
};

} // namespace ps::vehicle
