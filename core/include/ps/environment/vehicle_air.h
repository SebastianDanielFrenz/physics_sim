// ps/environment/vehicle_air.h - the air one vehicle sits in this tick (weather
// plan section 9, W4): atmosphere at the chassis altitude, humid-air density and
// the wind (weather mean + synthesised gust) at the chassis position.
//
// A plain, fixed-size, trivially copyable value, filled once per tick per
// vehicle by Environment::sample_air (World's freeze_tick_start_state stage),
// read by the vehicle job through vehicle::VehicleTickEnv, and published in
// VehicleSnapshot for HUDs. NEVER hashed: a pure function of the environment
// state (which is hashed) and the chassis position (which is body state).

#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"

namespace ps::environment {

struct VehicleAir {
    bool valid = false;                 // false = no environment atmosphere for this vehicle (consumers fall back to the World's own values)
    real altitude_m = 0.0;              // above sea level the sample is for (chassis z + the environment's altitude offset, clamped to the ISA range)
    real pressure_pa = 101325.0;
    real temperature_k = 288.15;
    real vapour_pressure_pa = 0.0;
    real density_kg_m3 = 1.225;         // humid-air density (what aero uses)
    Vec3 mean_wind_world;               // m/s, the direction the air moves TOWARD (World frame, Z up)
    Vec3 gust_world;                    // m/s, turbulent part at the chassis position
    Vec3 wind_world;                    // mean + gust (what aero uses)
};

} // namespace ps::environment
