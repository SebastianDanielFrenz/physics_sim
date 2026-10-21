// ps/drivetrain/electric_envelope.h - the electric machine and DC-supply laws as public, pure functions (powertrain dyno,
// 2026-10-19). These ARE the runtime laws: core/src/drivetrain/electric_drive.h's ElectricMotor::pre_solve, Battery and
// FuelCell call exactly these functions every substep (one implementation), so an envelope drawn from them is the torque
// the car's motor delivers under the same inputs. docs/electric_drive.md "Envelope and dyno".
//
// The motor law per substep h (pedal command c in [0, 1] after the traction ceiling, |omega| the shaft speed):
//   T     = peak_torque_nm * c
//   w_p   = |omega| + h * T / inertia_kgm2                   (rotor-inertia prediction of the end-of-substep speed)
//   T     = min(T, max_power_w / max(1, w_p))                 (constant-power corner)
//   E     = h * T * max(1, w_p) / efficiency                  (electrical energy wanted this substep)
//   T    *= granted / E                                        (the pack's grant; granted <= E)
//   T     = min(T, content * efficiency / (h * max(1, w_max))) (strict residual-energy bound near an empty pack)
// zero at or above max_rpm. Units: N*m, W, J, s, rad/s, rev/min.
#pragma once

#include "ps/drivetrain/electric_drive_desc.h"
#include "ps/types.h"

#include <cstdint>
#include <limits>

namespace ps::drivetrain {

// What limits a motor's drive torque at a point. Append-only values.
enum class MotorLimit : std::uint8_t {
    Idle = 0,             // not driving (pedal below the drive threshold, regenerating, not ready, neutral)
    PeakTorque = 1,       // the torque rating (constant-torque region)
    RatedPower = 2,       // the power rating (constant-power region)
    BatteryDischarge = 3, // the pack's discharge budget this substep (or the residual-energy bound)
    MaxRpm = 4,           // at or above max_rpm: zero torque
    Empty = 5,            // the pack granted nothing (empty, or no discharge budget)
};
[[nodiscard]] const char* to_string(MotorLimit l);

// The motor's request before the pack: torque after the power corner and the electrical energy it needs this substep.
struct MotorDriveRequest {
    real torque_nm = 0.0;
    real wanted_j = 0.0;
    bool power_limited = false; // the constant-power corner cut the torque
};
// abs_omega >= 0 [rad/s]; command = the pedal, in [0, 1]; h > 0 [s]. ceiling_scale in [0, 1] is the traction-control
// fraction of the torque the motor would deliver now (after the power corner), so it bites in the constant-power region too.
[[nodiscard]] MotorDriveRequest motor_drive_request(const ElectricMotorDesc& d, real abs_omega, real command, real h,
                                                    real ceiling_scale = real(1));

// The drive torque magnitude after the pack granted `granted_j` (<= request.wanted_j) and the residual-energy bound on the
// pack's current `content_j`. `limit` (optional) receives the classification.
[[nodiscard]] real motor_drive_torque(const ElectricMotorDesc& d, const MotorDriveRequest& request, real granted_j,
                                      real content_j, real h, MotorLimit* limit = nullptr);

// The drive-threshold test of the runtime motor: drives iff pedal > 0.02, regen <= 0.05, a direction and rpm < max_rpm.
[[nodiscard]] bool motor_drives(const ElectricMotorDesc& d, real pedal, real regen, real direction, real rpm);

struct MotorFullLoadPoint {
    real torque_nm = 0.0;    // shaft torque magnitude
    real power_w = 0.0;      // torque * omega
    real electrical_w = 0.0; // power / efficiency (battery side)
    MotorLimit limit = MotorLimit::Idle;
};
// Full pedal at `rpm` with `electrical_budget_w` available to this motor (the grant is min(budget * h, wanted)) and a pack
// `content_j` (default: unlimited). h = the car's substep (1/960 s). rpm >= 0.
[[nodiscard]] MotorFullLoadPoint motor_full_load_point(const ElectricMotorDesc& d, real rpm, real electrical_budget_w, real h,
                                                       real content_j = std::numeric_limits<real>::infinity());

// The battery's discharge budget for one substep: min(content, max_discharge_w * h) [J]. The runtime Battery calls this.
[[nodiscard]] real battery_discharge_budget_j(const BatteryDesc& b, real content_j, real h);
// The same at a state of charge, as power [W]: content = capacity_kwh * 3.6e6 * soc (the Battery constructor's expression).
[[nodiscard]] real battery_discharge_budget_w(const BatteryDesc& b, real soc, real h);
// The battery's charge acceptance for one substep: min(capacity - content, max_charge_w * h) [J].
[[nodiscard]] real battery_charge_budget_j(const BatteryDesc& b, real content_j, real h);

// The fuel cell's request for one substep before the pack's acceptance [J]: min(hydrogen * specific energy * efficiency,
// h * max_power_w * clamp((target_soc - soc) / 0.05, 0, 1)); 0 with no hydrogen. The runtime FuelCell calls this.
[[nodiscard]] real fuel_cell_request_j(const FuelCellDesc& f, real hydrogen_kg, real battery_soc, real h);
// What the stack actually adds to the pack (and to its discharge budget) this substep, as power [W]: the request limited
// by the pack's charge acceptance at that state of charge. The pack's own discharge budget is separate
// (battery_discharge_budget_w); the motors can draw the sum.
[[nodiscard]] real fuel_cell_supply_w(const FuelCellDesc& f, const BatteryDesc& b, real soc, real hydrogen_kg, real h);

} // namespace ps::drivetrain
