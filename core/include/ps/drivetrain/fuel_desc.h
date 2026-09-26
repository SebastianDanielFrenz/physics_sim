// ps/drivetrain/fuel_desc.h — public, additive: fuel properties feeding a
// future engine/fuel_tank model (PHYS-033 P4 I7a). Pure data, no algorithm
// code (mirrors ps::vehicle::TyreDesc/ps::drivetrain::PowertrainEnergy's
// seam: a small standalone descriptor loaded from data/fuels/*.json via
// ps::io::load_fuel_json, core/src/io/fuel_io.cpp).
//
// Lives in ps::drivetrain (not ps::vehicle or a new ps::fuel namespace)
// because its first and only planned consumers are drivetrain types: a
// later commit references FuelDesc from TorqueMapEngineDesc (or a sibling
// engine descriptor) and a new fuel_tank IComponent, both already in
// ps::drivetrain - see core/src/drivetrain/ in CLAUDE.md's "Drivetrain"
// section. Nothing about a fuel's own properties is vehicle- or
// tyre-specific, so ps::vehicle would be the wrong home; a standalone
// ps::fuel namespace would just be one more namespace for one struct.

#pragma once

#include "ps/types.h"

#include <string>

namespace ps::drivetrain {

// Phase of matter the fuel is modelled as. Only Liquid exists today
// (physics_sim.fuel/1) - a future Gaseous phase (CNG/LPG/hydrogen) would
// need its own density convention (pressure/temperature-dependent, not a
// single kg/m^3) and is deliberately not modelled yet.
enum class FuelPhase {
    Liquid,
};

struct FuelDesc {
    std::string name;   // e.g. "gasoline", "diesel" - informational only
    FuelPhase phase = FuelPhase::Liquid;

    real lhv_j_kg = 0.0;       // lower heating value, J/kg (must be > 0, finite)
    real afr_stoich = 0.0;     // stoichiometric air/fuel mass ratio (must be > 0, finite)
    real density_kg_m3 = 0.0;  // liquid density at the cited reference condition (must be > 0, finite)

    // Provenance note (PLAN.md section 8's "source" convention, same as
    // TyreDesc::source/VehicleDesc::source): the exact publication/standard,
    // table or problem, and page/edition each numeric value above was read
    // from. Required, must be non-empty.
    std::string source;
};

} // namespace ps::drivetrain
