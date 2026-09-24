// ps/drivetrain/powertrain_desc.h — public, additive descriptors for the
// P4 I2 shaft-network powertrain (PHYS-033 P4 I2 plan section 2). Pure
// data; no algorithm code, no Jolt/Godot type (same D2/D10 pattern as
// ps/articulation/articulation_desc.h) - the graph-building/solving
// machinery lives in core/src/drivetrain (internal, ps::drivetrain).
//
// I2 scope only (plan section 1): engine (kind torque_map, P3 semantics),
// gear_pair, differential (kind open), plus the implicit wheel hubs
// (WheelDesc, ps/vehicle/vehicle_desc.h). No clutch/gearbox/engine-idle
// (I3), no compliant shafts/LSD/locked diffs (I4).

#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h" // Curve1D (NOT vehicle_desc.h: VehicleDesc::powertrain
                                 // is a PowertrainDesc, so that include would be circular)

#include <string>
#include <variant>
#include <vector>

namespace ps::drivetrain {

// "component.port" (e.g. "final_drive.in") or "wheel:<name>" (is_wheel
// true, `component` holds the wheel's WheelDesc::name).
struct PortRef {
    std::string component;
    std::string port;
    bool is_wheel = false;
};

// Joins ports into one rigid node (I2: stiffness must be 0 - a compliant
// shaft is I4). PHYS-033 P4 I2 plan section 2.3.
struct ShaftDesc {
    std::vector<PortRef> ports;
    real stiffness = 0.0; // N*m/rad; I2 requires exactly 0 (rigid)
    real damping = 0.0;   // N*m*s/rad; unused while stiffness == 0
};

// I2 semantics == the P3 placeholder (vehicle_instance.cpp's pre-I2 drive):
// tau = throttle * wot(clamp(|rpm|, idle_rpm, limiter.rpm)), tau = 0 when
// |rpm| > limiter.rpm, always in the +omega direction. I3 extends this
// additively (idle control, stall, motoring, ECU) and changes these
// semantics when it rewrites the data - see PHYS-033 I3 plan section 0.
struct EngineLimiterDesc {
    std::string type = "hard_cut"; // I2: only "hard_cut" is accepted
    real rpm = 6500.0;
};

struct TorqueMapEngineDesc {
    vehicle::Curve1D wot_torque_nm_vs_rpm; // required, non-empty, x ascending
    real inertia_kgm2 = 0.0;               // >= 0; 0 allowed only if a rigid coupling eliminates the node
    real idle_rpm = 900.0;
    EngineLimiterDesc limiter;
};

// w_in = ratio * w_out (plan section 4.2's rigid-row convention).
struct GearPairDesc {
    real ratio = 1.0;
    real efficiency = 1.0;             // (0, 1]; < 1 needs a mesh-loss dissipative torque (section 5.4)
    real inertia_in_kgm2 = 0.0;
    real inertia_out_kgm2 = 0.0;
};

enum class DifferentialKind { Open }; // I4 adds Locked, ClutchLsd, Viscous, Torsen, Active

struct DifferentialDesc {
    DifferentialKind kind = DifferentialKind::Open;
    real ratio = 1.0;
    real efficiency = 1.0; // I2: must be exactly 1 (loader error otherwise - section 5.4's single-rigid-row-per-lossy-node limitation)
    // PHYS-033 I3 (plan section 4.1): additive, defaults reproduce I2 exactly
    // (carrier had no inertia field at all - Powertrain's constructor still
    // passes a hardcoded 0.0 for it, not these - and no rotor-reaction code
    // exists yet to read the axis). Both are wired in by commit 6.
    real carrier_inertia_kgm2 = 0.0;
    Vec3 carrier_axis_local = Vec3::zero();
};

// Closed, visited exhaustively (like ps::ShapeDesc) - adding a component
// type is a compile-checked change plus a data-format minor bump.
using ComponentParams = std::variant<TorqueMapEngineDesc, GearPairDesc, DifferentialDesc>;

struct ComponentDesc {
    std::string id; // unique, non-empty
    ComponentParams params;
};

enum class Compliance { Rigid }; // I4 adds Compliant

// PHYS-033 I3 (plan section 4.1): the gear_request channel's "no request"
// sentinel. Public because the adapter/scenario layer needs it too once
// wired (commit 6+) - out of forward AND reverse gear range for any
// gearbox this project ships (car: -1..6, tractor: -4..12).
inline constexpr real kGearRequestNone = real(-100.0);

struct PowertrainDesc {
    std::vector<ComponentDesc> components; // ARRAY order == evaluation order (PLAN.md D7)
    std::vector<ShaftDesc> shafts;
    Compliance compliance = Compliance::Rigid;
};

} // namespace ps::drivetrain
