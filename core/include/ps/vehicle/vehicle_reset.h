// ps/vehicle/vehicle_reset.h — public options struct for World::reset_vehicle
// (PHYS-033 P4 I3 plan section 4.4/4.5): an out-of-band "teleport" of a
// vehicle's drivetrain state (e.g. the adapter's shadow-chassis flow, or a
// test harness dropping straight into an already-driving state) that sets
// the network directly - no shift sequence, no engagement/projection loss
// (that ledger term is `engagement_loss`; a reset's own rotational-KE delta
// books into the separate `external_reset` term instead, since this is not
// a physical clutch engagement).
//
// Pure data; no algorithm code (that lives in core/src/vehicle/vehicle_instance.*
// and core/src/drivetrain/powertrain.*, internal - same D2/D10 pattern as
// every other public descriptor in this project).

#pragma once

#include "ps/drivetrain/powertrain_state.h" // drivetrain::EngineState

namespace ps::vehicle {

struct VehicleResetOptions {
    // gear == kKeepGear leaves the gearbox at whatever position it is
    // currently in (still forced through reset_to - no shift sequence, no
    // grind event - just re-affirms the current gear's topology state).
    static constexpr int kKeepGear = 1000;
    int gear = kKeepGear;
    drivetrain::EngineState engine_state = drivetrain::EngineState::Running;
    bool hubs_roll_with_chassis = true; // hub omega from the chassis's current velocity (else 0)
    bool clutch_locked = true;          // engaged gear: engine speed = G * driven-hub speed (floored at idle)
};

} // namespace ps::vehicle
