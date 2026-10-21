// ps/drivetrain/powertrain_dyno.h - a full-pedal dyno sweep of a whole POWERTRAIN (EV, fuel-cell EV, hybrid, or a plain ICE
// car), from the same runtime Powertrain the car drives (powertrain dyno, 2026-10-19; docs/electric_drive.md "Envelope and
// dyno").
//
// Method: a chassis dyno. The vehicle's Powertrain is built without a chassis; every hub is held at the rolling speed of one
// road speed (omega_i = v / r_i, the tyres replaced by an implicit speed hold), the chosen axis shaft at the axis value
// (v from the kinematic ratio of that shaft at the held gear). Everything else runs as in the car, substep by substep at
// the car's 240/960 Hz: throttle stage, hybrid supervisor (split, engine start/stop, disconnect), motor law, the battery's
// discharge budget and its reservation, fuel-cell supply, gear and reduction efficiencies, clutch, engine (a turbo engine
// spools and its wastegate settles). The battery state of charge and the hydrogen fill are held at the spec's values for
// the whole point (the pack is not drained by the settle). The point settles like run_dyno_sweep's stepped kind
// (ps/drivetrain/engine_dyno.h): window means over settle_window_s, two consecutive passing window pairs.
//
// System output: the dyno's absorbed power at the hubs (the wheel power, every loss between the sources and the hubs
// included) and its torque referred to the axis shaft (wheel power / axis omega).
//
// Units: rpm [rev/min], torque [N*m], power [W], speed [m/s].
#pragma once

#include "ps/drivetrain/electric_envelope.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/jobs/job_system.h"
#include "ps/types.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/ambient.h"

#include <atomic>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace ps::drivetrain {

enum class PowertrainDynoAxis : std::uint8_t {
    EngineRpm = 0,        // gearbox input speed in the held gear (= engine speed with the clutch closed); needs a gearbox and gear != 0
    MotorRpm = 1,         // the shaft of motor `motor_index` (component order among the motors)
    GearboxOutputRpm = 2, // gearbox output shaft; needs a gearbox
    RoadSpeed = 3,        // values are road speed in m/s (the reference torque is then referred to the driven-wheel mean radius)
};

struct PowertrainDynoSpec {
    PowertrainDynoAxis axis = PowertrainDynoAxis::MotorRpm;
    std::vector<real> points;      // axis values, each finite and > 0; evaluated in this order
    int motor_index = 0;           // MotorRpm axis only: index into the vehicle's electric motors (component order)
    int gear = 1;                  // held gear (manual / sequential gearbox; ignored without a gearbox). Automatic, DCT and CVT
                                   // gearboxes are not supported yet (std::invalid_argument).
    real pedal = 1.0;              // the driver's pedal, 0..1 (the car's pedal curve applies; a drive_modes bend does not)
    real battery_soc = std::numeric_limits<real>::quiet_NaN();       // every pack; NaN = each pack's initial_soc
    real hydrogen_fraction = std::numeric_limits<real>::quiet_NaN(); // of every fuel cell's tank capacity; NaN = initial fill
    int drive_mode = 1;            // channel assist.drive_mode (hybrid supervisor: 0 Eco, 1 Normal, 2 Sport, 3 Racing)
    AmbientSample ambient{};       // the engine's air
    // Further channels set for the whole sweep (name, value), e.g. {"electric.ready", 1} or an assist channel. A name the
    // powertrain does not declare is ignored.
    std::vector<std::pair<std::string, real>> channels;
    // Settle (as DynoSweepSpec): window means; a pair passes when |dP| / window < settle_power_rel_per_s * |P| and (turbo)
    // |d boost| / window < settle_boost_bar_per_s. At least min_settle_s, at most max_settle_s.
    real settle_window_s = 0.5;
    real settle_power_rel_per_s = 0.002;
    real settle_boost_bar_per_s = 0.005;
    real min_settle_s = 1.0;
    real max_settle_s = 15.0;
};

struct PowertrainDynoMotor {
    real rpm = 0.0;
    real torque_nm = 0.0; // shaft torque (signed, + drives)
    real power_w = 0.0;   // mechanical
    real electrical_w = 0.0; // battery side (+ discharge)
    MotorLimit limit = MotorLimit::Idle; // at the last substep of the window
    bool crank_machine = false;          // a crank-mounted generator (crank_assist)
};

struct PowertrainDynoPoint {
    real axis_value = 0.0;
    real road_speed_mps = 0.0;
    real engine_rpm = 0.0;          // crank (0 without an engine)
    real gearbox_output_rpm = 0.0;  // 0 without a gearbox
    real engine_torque_nm = 0.0;    // net crank torque (Coulomb row included); 0 while off
    real engine_power_w = 0.0;
    real engine_command = 0.0;      // the throttle command the engine received (the hybrid split's ICE share)
    bool engine_running = false;
    int gear = 0;                   // the gear at the end of the window (the supervisor may hold neutral)
    std::vector<PowertrainDynoMotor> motors; // component order
    real electric_power_w = 0.0;    // sum of the motors' mechanical power
    real battery_power_w = 0.0;     // the packs' net discharge (motors' electrical minus fuel-cell supply)
    real fuel_cell_power_w = 0.0;
    real system_power_w = 0.0;      // absorbed at the hubs
    real system_wheel_torque_nm = 0.0; // sum of the hub torques
    real system_torque_nm = 0.0;    // system_power_w / axis omega (RoadSpeed: wheel torque at the driven-wheel mean radius)
    real boost_bar = 0.0;           // turbo engines: manifold gauge pressure
    bool battery_limited = false;   // some traction motor was limited by the pack (BatteryDischarge or Empty)
    bool converged = true;
    real settle_s = 0.0;
    // Hybrid supervisor state at the end of the window (0 / false without one).
    real hybrid_requested_w = 0.0, hybrid_available_w = 0.0;
    bool hybrid_engine_coupled = false, hybrid_charging = false;
};

struct PowertrainDynoResult {
    std::vector<PowertrainDynoPoint> points; // spec order; identical for any worker count
};

// `vehicle`: a loaded vehicle (ps::io::load_vehicle_json); only its powertrain, brake_system, wheels and controllers are used.
// `maps`: null = the engine's own; otherwise they replace a simulated engine's maps block (as run_dyno_sweep) and complete a
// maps_pending engine. A maps_pending engine without `maps` throws std::runtime_error. A bad spec or an unsupported
// configuration throws std::invalid_argument. `pool`: one job per point (nullptr = serially on the calling thread); each job
// builds its own Powertrain from the shared descriptor. `cancel`: read before each point, GenerationCancelled after the
// running points finished (run_dyno_sweep's semantics).
[[nodiscard]] PowertrainDynoResult run_powertrain_dyno(const vehicle::VehicleDesc& vehicle, EngineMapsPtr maps,
                                                       const PowertrainDynoSpec& spec, jobs::JobSystem* pool = nullptr,
                                                       const std::atomic<bool>* cancel = nullptr);

} // namespace ps::drivetrain
