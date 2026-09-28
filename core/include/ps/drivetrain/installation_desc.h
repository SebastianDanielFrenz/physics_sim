// ps/drivetrain/installation_desc.h - public descriptor for a vehicle's
// intake/exhaust INSTALLATION (ps_audio A4, out/audio_plan.md section 3.2,
// decision D-4/Q1). Pure data, no algorithm code: the mapping from this
// struct (plus a CycleModelDesc's intake/exhaust pipe blocks,
// ps/drivetrain/engine_cycle_desc.h) to a real PipeNetworkDesc
// (ps/engine_cycle/pipe_network.h) is A5's job - nothing reads this struct
// yet.
//
// Ownership split (Q1, owner-ruled 2026-09-28): the ENGINE file
// (engine_cycle_desc.h's IntakeCycleDesc/ExhaustCycleDesc) owns the
// throttle-body inlet through the first flange after the last
// engine-mounted part (plenum, runners, ports, primaries, collector, a
// manifold catalyst if engine-mounted). This installation file, referenced
// by the vehicle (or by the engine's own `default_installation`), owns the
// airbox/snorkel, the catalyst and everything downstream, and the outlet
// positions.
//
// Loader: core/src/io/vehicle_io.cpp's read_installation_object /
// ps::io::load_installation_json (format "physics_sim.installation/1").
// Positions (`inlet_position_m`, `position_m`) are audio-only - excluded
// from ps::drivetrain::engine_map_cache_key (map_cache.cpp) by construction
// of that function, never read from this struct.

#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"

#include <string>
#include <vector>

namespace ps::drivetrain {

struct InstallationSnorkelDesc {
    real length_m = 0.0;
    real d_m = 0.0;
};

struct InstallationIntakeDesc {
    real airbox_m3 = 0.0;
    bool has_snorkel = false;   // PROVISIONAL (A4): a plenum may instead open straight into the airbox (velocity stacks)
    InstallationSnorkelDesc snorkel;
    // Vehicle frame; audio-only (A8+), excluded from the map cache key.
    Vec3 inlet_position_m = Vec3::zero();
};

enum class ExhaustElementType { Catalyst, Pipe, ExpansionChamber, Outlet, Branch };

// One element of an installation's exhaust chain. Which fields are
// meaningful depends on `type` (see ExhaustElementType):
//  - Catalyst / Pipe / ExpansionChamber: length_m, d_m (> 0); Catalyst also
//    k_loss (> 0, a quasi-steady loss coefficient).
//  - Outlet: position_m only (vehicle frame; audio-only, excluded from the
//    map cache key). Terminates a chain - must be the last element.
//  - Branch (PROVISIONAL: out/audio_plan.md section 3.2 describes this only
//    in prose, "a list of branches after a junction element", with no
//    concrete JSON example): `branches` holds >= 2 parallel chains that
//    start right after this element, each its own ordered element list,
//    each normally ending in its own Outlet. Must be the last element of
//    its own chain (nothing follows a branch point in the SAME list).
struct ExhaustElementDesc {
    ExhaustElementType type = ExhaustElementType::Pipe;
    real length_m = 0.0;
    real d_m = 0.0;
    real k_loss = 0.0;
    Vec3 position_m = Vec3::zero();
    std::vector<std::vector<ExhaustElementDesc>> branches;
};

struct InstallationExhaustDesc {
    // Ordered chain from the engine's exhaust interface (the flange
    // CycleModelDesc's ExhaustCycleDesc names) to one or more outlets. Must
    // be non-empty and end in exactly one Outlet, or in a Branch whose own
    // sub-chains each end in exactly one Outlet.
    std::vector<ExhaustElementDesc> elements;
};

struct InstallationDesc {
    InstallationIntakeDesc intake;
    InstallationExhaustDesc exhaust;

    // PLAN.md section 8: provenance of every length/diameter/volume/loss
    // coefficient above. Required non-empty by the loader.
    std::string source;
};

} // namespace ps::drivetrain
