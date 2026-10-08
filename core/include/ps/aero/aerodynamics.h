// Quasi-steady, deterministic coefficient model; SI, ISO +x forward/+y left/+z up.
#pragma once
#include "ps/math/pose.h"
#include "ps/aero/coefficient_map.h"
#include "ps/world/ids.h"
#include <array>
#include <span>
#include <string>
#include <vector>
#include <cstdint>
namespace ps::aero {
inline constexpr std::size_t max_surfaces=32, max_fans=8;
struct GroundEffectDesc {
    Vec3 probe_local{0,0,-1}; // points TOWARD nearby road, not global gravity
    real range_m=0, gain=0; // multiplier 1 + gain*(1-gap/range)^2; miss => 1
};
struct SurfaceDesc {
    std::string name;
    Vec3 position_local{}, chord_local{1,0,0}, normal_local{0,0,1};
    real area_m2=1, cl0=0, lift_slope_per_rad=4.5, stall_angle_rad=.3;
    real cd0=.02, induced_drag= .08, stalled_drag=1.2;
    real rest_angle_rad=0, min_offset_rad=0, max_offset_rad=0, rate_rad_s=1, max_lift_m=0;
    // Mechanical angle about normal x chord (+y by default); positive noses chord DOWN.
    // Command in [-1,1]: negative goes toward min_offset, positive toward max_offset.
    GroundEffectDesc ground;
};
struct BodyDesc {
    Vec3 position_local{}, drag_area_xyz_m2{}, moment_coeff{}, angular_damping{.1,.1,.1};
    real reference_area_m2=0, reference_length_m=1, side_force_slope=0;
};
struct WakeDesc {
    real length_m=0, radius_m=1, expansion=.12, deficit=.25, turbulence=.1;
};
struct FanDesc {
    std::string name;
    Vec3 position_local{}, direction_local{0,0,-1};
    real area_m2=1, pressure_pa=1000, seal_height_m=.1, probe_range_m=1;
    real max_power_w=10000, rate_per_s=1;
};
enum class MapScope { ReplaceBody, ReplacePassiveAero };
struct MapDesc {
    std::shared_ptr<const CoefficientMap> table;
    MapScope scope=MapScope::ReplaceBody;
    Vec3 ground_probe_local{0,0,-1}, ground_position_local{};
    real ground_range_m=0;
    std::size_t wing_surface=0; // actual actuator offset and lift, never the command
};
struct Desc {
    // Legacy isotropic CdA applies ONLY if body/surfaces are absent. Fans are independent.
    real drag_area_m2=0;
    BodyDesc body;
    std::vector<SurfaceDesc> surfaces;
    std::vector<FanDesc> fans;
    WakeDesc wake;
    MapDesc coefficient_map;
};
struct Environment { real air_density=1.225; Vec3 wind_world{}; };
struct WakeSource { BodyId owner{}; Vec3 position_world{}, velocity_world{}; WakeDesc desc; };
struct AirSample { Vec3 wind_world{}; real turbulence=0; };
struct GroundSample { bool hit=false; real clearance_m=0; };
struct State {
    std::array<real,max_surfaces> offset_rad{}, surface_command{};
    std::array<real,max_surfaces> lift_m{}; // actual derived height after the rate-limited offset advances
    std::array<real,max_fans> fan_level{}, fan_command{};
    real fan_energy_j=0; // prepaid energy reservoir; starts EMPTY, caller explicitly credits it
};
struct SurfaceTelemetry {
    Wrench wrench_world{}; real alpha_rad=0, beta_rad=0, cl=0, cd=0, dynamic_pressure_pa=0;
    real angle_rad=0, offset_rad=0, lift_m=0, ground_multiplier=1, clearance_m=-1, turbulence=0;
};
struct FanTelemetry {
    Vec3 force_world{};
    real level=0, power_w=0, energy_used_j=0, seal=0, clearance_m=-1;
};
struct Telemetry {
    Wrench wrench_world{};
    bool coefficient_map_active=false, coefficient_map_ground_missing=false;
    std::uint32_t coefficient_map_clamped_axes=0;
    std::array<real,6> coefficient_map_inputs{};
    std::array<real,6> coefficient_map_coefficients{};
    Vec3 body_relative_airflow_local{}, body_force_local{}, body_moment_local{};
    real body_airspeed_m_s=0, body_dynamic_pressure_pa=0, body_turbulence=0;
    std::array<SurfaceTelemetry,max_surfaces> surfaces{};
    std::array<FanTelemetry,max_fans> fans{};
    std::size_t surface_count=0, fan_count=0;
    real downforce_n=0, front_downforce_n=0, rear_downforce_n=0;
    // Standalone step groups by local x>=0/<0. World overrides with axle-equivalent loads
    // from the full local pitch moment when front/rear wheel groups both exist.
    bool axle_balance_valid=false;
    real front_balance=0, fan_power_w=0, fan_energy_used_j=0, fan_energy_remaining_j=0;
};
// Validation is shared by JSON loading and World::create_vehicle (also protects native descriptors).
void validate(const Desc&);
void validate(const Environment&);
void validate(const WakeDesc&);
AirSample sample_air(const Environment&, Vec3 point_world, BodyId exclude,
                     std::span<const WakeSource> wakes={}, std::span<const WakeSource> extra_wakes={});
// Ground span order: all surfaces, then fans, then optional map probe. Missing/short span means no ground.
// No allocation. Independent of wheels/backends: usable directly for an aircraft or any rigid body.
Telemetry step(const Desc&, State&, const Pose&, const Motion&, const Environment&, real dt,
               std::span<const GroundSample> ground={}, std::span<const WakeSource> wakes={},
               BodyId exclude={}, std::span<const WakeSource> extra_wakes={});
std::uint64_t fold_state_hash(const Desc&, const State&, std::uint64_t seed);
} // namespace ps::aero
