// ps/world/ambient.h — ambient atmospheric conditions (PHYS-033 P4 I7b C9,
// PLAN.md:578-584's engine-model requirement: "ambient density (altitude,
// from the P5 atmosphere)"). The P5 atmosphere model itself is not built yet
// (out/i7_plan_backup.md 3.6 "Ambient density hook") - this is the minimal
// additive seam that lets it plug in later without touching every consumer
// again: a plain {pressure, temperature} sample, an ISA sea-level default,
// and World::set_ambient (world.h) as the runtime hook. Plumbed to every
// vehicle's engine Sensors (core/src/drivetrain/engine.h) via
// ps::vehicle::VehicleTickEnv (core/src/vehicle/vehicle_instance.h) and
// ps::drivetrain::ComponentSubstepCtx (core/src/drivetrain/components.h) so
// a future simulated engine (I7c+) can model altitude effects on power.
//
// Deliberately NOT wired into WorldConfig::air_density (aero drag, PLAN.md
// P4 I3 plan section 4.5): this commit does not derive air_density from the
// sample, so aero drag is byte-for-byte unchanged by this header's
// existence. The graph engine (EngineTorqueMap) also ignores it entirely
// (owner ruling Q10, i7_plan_backup.md 3.6) - only a future simulated engine
// reads it. Never hashed: an ECU/environment INPUT, like a control channel
// (ps::World::set_control), not simulation state.

#pragma once

#include "ps/types.h"

namespace ps {

// Pa, K. Default = ISA (International Standard Atmosphere, ISO 2533:1975)
// sea-level values: 101325 Pa, 288.15 K (15 degC).
struct AmbientSample {
    real pressure_pa = 101325.0;
    real temperature_k = 288.15;
};

} // namespace ps
