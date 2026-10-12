// assists I5: the rear-wing automation law of the assist unit (ps/vehicle/assist_unit_desc.h). Stateless; only + - * /
// and comparisons (no libm), so nothing here depends on the platform's math library.
#include "ps/vehicle/assist_unit_desc.h"

namespace ps::vehicle {

real aero_control_command(const AeroControlDesc& c, real max_offset_rad, real min_offset_rad, real forward_speed, real brake, bool launch,
                          real* target_deg_out) {
    real target = 0;
    if (launch) {
        target = c.launch_offset_deg;
    } else {
        const real ramp = (forward_speed - c.cruise_ramp_start_m_s) / (c.cruise_ramp_end_m_s - c.cruise_ramp_start_m_s);
        target = c.cruise_offset_deg * (ramp < real(0) ? real(0) : (ramp > real(1) ? real(1) : ramp));
        if (forward_speed >= c.airbrake_min_speed_m_s) {
            const real b = brake < real(0) ? real(0) : (brake > real(1) ? real(1) : brake);
            const real d = (b - c.airbrake_brake_start) / (c.airbrake_brake_full - c.airbrake_brake_start);
            const real demand = d < real(0) ? real(0) : (d > real(1) ? real(1) : d);
            target += (c.airbrake_offset_deg - target) * demand;
        }
    }
    if (target_deg_out) *target_deg_out = target;
    const real radians = target * real(3.141592653589793) / real(180);
    const real limit = radians >= real(0) ? max_offset_rad : -min_offset_rad;
    if (!(limit > real(0))) return real(0);
    const real cmd = radians / limit;
    return cmd < real(-1) ? real(-1) : (cmd > real(1) ? real(1) : cmd);
}

} // namespace ps::vehicle
