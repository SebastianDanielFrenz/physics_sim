// ps/machine/linkage.h — a tractor's three-point linkage: two lower links, a top
// link and a lift (docs/data_formats/linkage.md).
//
// The links are massless and collapse onto the tractor's centre plane: the lower
// links into one link from the lower pivot to the middle of the lower pins, the
// top link from the top pivot to the top pin. That makes a planar four-bar with one
// degree of freedom, the lower link's angle. An implement hangs on it with its
// pitch given by the geometry; the lift holds that angle with a force it can run
// out of, and it can only lift. The rows live in core/src/machine/coupling_solver.h.

#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"

#include <string>

namespace ps::machine {

// The lift's three modes, in the order the mode channel counts them.
enum class LiftMode : unsigned char {
    Position = 0, // holds the lever's height
    Draft = 1,    // raises and lowers to hold the sensed draft at the setpoint; the lever is the deepest it goes
    Float = 2,    // no lift force: the implement follows the ground between the link stops
};

// The tractor's side, one block on a towing hitch point.
struct LinkageDesc {
    std::string source;
    Vec3 lower_pivot_local{}; // chassis-local, on the centre plane
    Vec3 top_pivot_local{};
    real min_angle_rad = 0.0, max_angle_rad = 0.0; // the lower link's stops, from the hitch point's pose
    real max_lift_force_n = 0.0; // at the lower pins, across the link
    real full_stroke_s = 0.0;    // stop to stop at the pump's flow
    real follow_slack_rad = 0.0; // how far the held angle may run below a link the ground holds up
    real draft_gain_per_s = 0.0; // lift rate over stroke rate, per relative draft error
    real draft_filter_s = 0.0;   // the sensing pin's lag
    real draft_band = 0.0;       // relative draft error inside which the lift holds still
    std::string position_channel = "lift";    // 0 lowest .. 1 highest
    std::string mode_channel = "lift_mode";   // LiftMode
    std::string draft_channel = "lift_draft"; // setpoint, newtons
};

void validate_linkage(const LinkageDesc& d); // std::invalid_argument

// The angle of a link in the chassis' x-z plane, pointing to the tractor's rear: 0 level, up positive.
[[nodiscard]] real linkage_angle(const Vec3& link_local);

// One coupling's linkage: the tractor's pivots and the implement's pins, in their chassis.
struct LinkageGeometry {
    Vec3 lower_pivot_local{}, top_pivot_local{}; // towing chassis
    Vec3 lower_pin_local{}, top_pin_local{};     // towed chassis
    real lower_length_m = 0.0, top_length_m = 0.0;
    real reference_angle_rad = 0.0; // the lower link's angle in the towing chassis at the hitch point's pose
    real min_angle_rad = 0.0, max_angle_rad = 0.0; // from that pose
    real max_lift_torque_nm = 0.0; // about the lower pivot
};

// Last tick (sim thread only). Everything here but `target` is derived.
struct LinkageState {
    LiftMode mode = LiftMode::Position;
    real angle_rad = 0.0;      // the lower link, from the hitch point's pose
    real position = 0.0;       // the same, 0 at the lower stop, 1 at the upper
    real target_rad = 0.0;     // what the lift is holding
    real lift_force_n = 0.0;   // at the lower pins, across the link, mean over the tick
    real lower_tension_n = 0.0; // both lower links together, along them, mean over the tick; pulled is positive
    real top_force_n = 0.0;    // the top link, mean over the tick; compressed is positive
    real sensed_draft_n = 0.0; // lower_tension_n through the sensing pin's lag
    bool saturated = false;    // the lift ran out of force in the last tick
};

} // namespace ps::machine
