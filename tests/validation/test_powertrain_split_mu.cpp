// tests/validation/test_powertrain_split_mu.cpp — PHYS-033 P4 I4 C8 (plan
// section 4, Test 4): split-mu (asphalt left / low_mu right) straight-line
// launch and braking behaviour for each differential kind, cross-checked
// against an independent per-tick hub-torque/yaw-moment reconstruction from
// wheel telemetry and the drivetrain's own debug hub_shaft_torque (PHYS-033
// P4 I4 C8 part (a)).
//
// Ground: a local split_surface_mesh (asphalt y>0, low_mu y<0, tessellated
// into 20 m cells - see that function's own comment for why the original
// 4-triangle version was an invalid rig) - kept LOCAL to this file.
// car_sedan.json's front wheel attachment_local y is +-0.78 m, so FL/RL
// (y>0) sit on asphalt and FR/RR (y<0) sit on low_mu; only the FRONT axle
// (diff "diff", FL/FR) is driven, so only its torque split is under test -
// RL/RR still contribute their own fx/fy to the Mz cross-check. The diff's
// own left/right network ports carry zero inertia (core/src/drivetrain/
// powertrain.cpp), so a driven wheel's hub-node inertia is exactly its own
// WheelDesc::wheel_inertia - this is what makes the per-wheel hub-torque
// reconstruction T_i = J_i*domega_i/dt + r_i*fx_i valid against the debug
// hub_shaft_torque[i] value.
//
// Chassis/tick-rate rig matches test_vehicle_accel_standing_start.cpp
// (mass 1500 kg, half-extents {2.0, 0.4, 0.15}, origin height 0.6 m,
// 240/960 Hz) per the plan's own instruction - except the hub-torque
// cross-check, which runs at 960/960 Hz (one substep per tick; see its own
// comment).
//
// Diagnosis history (2026-09-26, replacing the C8 [!mayfail] explanations,
// which did not hold up under measurement):
//   - "diff hop" (locked launch), "drifts onto the wrong surface" (viscous
//     launch) and the locked braking run's lateral divergence were all a rig
//     artefact: the 4-triangle ground returned wrong wheel shape casts near
//     its 2000 m seam edge. Tessellating the ground removed all three.
//   - the clutch_lsd/torsen "~20 Hz limit cycle" is the engine's fuel-cut
//     rev limiter (60 ms cycle) running through the whole plan window at WOT
//     in 1st, not the one-substep capacity lag; the cases now launch at
//     throttle 0.6 and assert the limiter stays off.
//   - the wheelspin hub-torque cross-check compared last-substep samples
//     with a whole-tick velocity difference; it now runs at one substep per
//     tick for every kind.
//   - the locked launch's telemetry Mz formula omits the tyres' aligning
//     moment and rolling resistance; the net yaw moment is measured
//     separately, the literal formula stays [!mayfail] with its breakdown.

#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/validation/CMakeLists.txt)"
#endif

namespace {

constexpr std::size_t kFL = 0, kFR = 1;

// The plan's +-1000 m flat split ground, y > 0 asphalt and y < 0 low_mu,
// TESSELLATED into a regular grid of `cell` x `cell` quads (two CCW
// triangles each, viewed from +z - tests/support/vehicle_test_helpers.h's
// flat_surface_mesh() convention). Every vertex is shared by index between
// neighbouring quads (one grid), so Jolt welds every interior edge,
// including the y = 0 surface seam. `2*half_extent/cell` must be an even
// integer so the seam lies on a grid line.
//
// Why tessellated (2026-09-26): the C8 version was two quads - four
// triangles with ~2000 m edges, the y = 0 seam edge itself 2000 m long.
// Jolt's cylinder-vs-triangle shape cast loses precision near an edge that
// long. Probing car_sedan's front-wheel CylinderShape cast (r 0.317,
// half-width 0.1125, 0.18 m sweep, yaw -0.2 - the query
// WheelInstance::measure_contact makes) on a 0.01 m grid over x in [3, 7],
// y in [-0.3, 0.4] against the 4-triangle mesh gave a wrong result for 276
// of 28471 poses (~1%): phantom fraction-0 hits ~0.12 m ABOVE the plane,
// surface ids from the wrong side of the seam, and fraction errors of 0.02
// up to 0.23 (4 mm to 4 cm of compression). The same probe against the
// same split in 10-20 m cells, or against the 4-triangle mesh 30 m away from
// the seam, found none (worst |dfraction| 5.3e-4); 4-triangle meshes of
// half-extent 100/300/500/1000 m gave 0/6/80/276. A phantom fraction-0 hit
// puts the wheel at full bump within one 960 Hz substep - a ~100 m/s
// compression rate through the damper. That is the whole of the old "diff
// hop" (locked launch: FL Fz 4311.7 N -> 0, FR 3702.8 N -> 15429.1 N in one
// tick at t0+1.92 s) and "wrong surface" (viscous launch) events: both
// happened at chassis x ~ 3.79 m, with FL near the seam. On the tessellated
// ground both runs are clean over the whole 2 s window (max wheel Fz 4416 N).
// "rig ground validity" below pins this; the [.][diag] "legacy ground cast
// probe" case reproduces the old numbers.
MeshShape split_surface_mesh(real half_extent, real cell, SurfaceId asphalt_id, SurfaceId low_mu_id) {
    const int cells = static_cast<int>(std::lround(2.0 * half_extent / cell));
    if (cells <= 0 || cells % 2 != 0 || std::abs(cells * cell - 2.0 * half_extent) > 1e-9) {
        throw std::runtime_error("split_surface_mesh: 2*half_extent/cell must be an even integer");
    }
    const int nv = cells + 1;
    MeshShape m;
    m.vertices.reserve(static_cast<std::size_t>(nv) * nv);
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nv; ++i) m.vertices.push_back(Vec3{-half_extent + i * cell, -half_extent + j * cell, 0.0});
    }
    m.indices.reserve(static_cast<std::size_t>(cells) * cells * 6);
    m.triangle_surface_ids.reserve(static_cast<std::size_t>(cells) * cells * 2);
    for (int j = 0; j < cells; ++j) {
        const SurfaceId s = (j >= cells / 2) ? asphalt_id : low_mu_id; // row j spans y in [-he + j*cell, -he + (j+1)*cell]
        for (int i = 0; i < cells; ++i) {
            const auto a = static_cast<std::uint32_t>(j * nv + i);
            const std::uint32_t b = a + 1, c = a + static_cast<std::uint32_t>(nv) + 1, d = a + static_cast<std::uint32_t>(nv);
            m.indices.insert(m.indices.end(), {a, b, c, a, c, d});
            m.triangle_surface_ids.push_back(s);
            m.triangle_surface_ids.push_back(s);
        }
    }
    return m;
}

// The C8 4-triangle ground (two quads sharing the y = 0 edge by vertex
// index), kept ONLY for the [.][diag] "legacy ground cast probe" case that
// documents why the rig no longer uses it - see split_surface_mesh() above.
MeshShape legacy_two_quad_split_mesh(real half_extent, SurfaceId asphalt_id, SurfaceId low_mu_id) {
    MeshShape m;
    m.vertices = {
        Vec3{-half_extent, 0.0, 0.0},          // 0
        Vec3{half_extent, 0.0, 0.0},           // 1
        Vec3{half_extent, half_extent, 0.0},   // 2
        Vec3{-half_extent, half_extent, 0.0},  // 3
        Vec3{-half_extent, -half_extent, 0.0}, // 4
        Vec3{half_extent, -half_extent, 0.0},  // 5
    };
    m.indices = {0, 1, 2, 0, 2, 3, 0, 4, 5, 0, 5, 1};
    m.triangle_surface_ids = {asphalt_id, asphalt_id, low_mu_id, low_mu_id};
    return m;
}

constexpr real kGroundHalfExtent = 1000.0;
constexpr real kGroundCell = 20.0;

struct SplitMuRig {
    std::unique_ptr<World> world;
    BodyId chassis{};
    VehicleId vehicle{};
    SurfaceId asphalt_id = kInvalidSurfaceId;
    SurfaceId low_mu_id = kInvalidSurfaceId;
};

SplitMuRig build_split_mu_rig(const vehicle::VehicleDesc& desc, real tick_hz = 240.0, real substep_hz = 960.0) {
    SplitMuRig rig;
    WorldConfig cfg;
    cfg.tick_rate_hz = tick_hz;
    cfg.substep_rate_hz = substep_hz;
    rig.world = std::make_unique<World>(cfg);

    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    rig.asphalt_id = surfaces->id_for("asphalt");
    rig.low_mu_id = surfaces->id_for("low_mu");
    rig.world->set_surface_table(surfaces);

    BodyDesc ground;
    ground.shape = split_surface_mesh(kGroundHalfExtent, kGroundCell, rig.asphalt_id, rig.low_mu_id);
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    rig.world->create_body(ground);

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    rig.chassis = rig.world->create_body(chassis);

    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);
    return rig;
}

DifferentialDesc& diff_desc(vehicle::VehicleDesc& d, const std::string& id) {
    for (auto& c : d.powertrain.components) {
        if (c.id == id) return std::get<DifferentialDesc>(c.params);
    }
    throw std::runtime_error("no differential '" + id + "'");
}

// car_sedan.json with its front diff switched to `kind`, using this file's
// fixed per-kind parameters (clutch_lsd 60 N*m / 0.45 / 0.25, torsen TBR 3,
// viscous 25 N*m*s/rad - the plan's Test 4 values).
vehicle::VehicleDesc load_sedan(DifferentialKind kind) {
    vehicle::VehicleDesc d = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    DifferentialDesc& diff = diff_desc(d, "diff");
    diff.kind = kind;
    if (kind == DifferentialKind::ClutchLsd) {
        diff.preload_nm = 60.0;
        diff.drive_factor = 0.45;
        diff.coast_factor = 0.25;
    } else if (kind == DifferentialKind::Torsen) {
        diff.torque_bias_ratio = 3.0;
    } else if (kind == DifferentialKind::Viscous) {
        diff.viscous_nm_s_rad = 25.0;
    }
    return d;
}

void require_wheel_surfaces(const World& world, VehicleId v, SurfaceId asphalt_id, SurfaceId low_mu_id) {
    const std::array<SurfaceId, 4> expected = {asphalt_id, low_mu_id, asphalt_id, low_mu_id};
    for (std::size_t i = 0; i < 4; ++i) {
        const vehicle::WheelTelemetry& t = world.wheel_telemetry(v, i);
        INFO("wheel " << world.wheel_name(v, i) << " (index " << i << ") surface=" << t.surface
                       << " expected=" << expected[i] << " - split-mu rig validity guard failed: the car is not "
                          "straddling the y=0 surface split as intended");
        REQUIRE(t.surface == expected[i]);
    }
}

// One tick's worth of the quantities every split-mu case needs: front hub
// speeds/forces, the debug hub_shaft_torque cross-check value, the whole-
// vehicle yaw moment reconstructed from wheel telemetry (steer 0, so
// Mz = sum(x_i*fy_i - y_i*fx_i) over all four wheels per the plan) and its
// longitudinal part alone, the chassis yaw rate, and whether the engine's
// rev limiter was cutting fuel.
struct TickSample {
    real omega_fl = 0.0, omega_fr = 0.0;
    real fx_fl = 0.0, fx_fr = 0.0;
    real hub_fl = 0.0, hub_fr = 0.0;
    real mz = 0.0;
    real mz_fx = 0.0; // sum(-y_i*fx_i) only
    real yaw_rate = 0.0;
    bool limiter_active = false;
};

real yaw_moment_now(const World& w, VehicleId v, const vehicle::VehicleDesc& desc) {
    real mz = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        const vehicle::WheelTelemetry& t = w.wheel_telemetry(v, i);
        mz += desc.wheels[i].attachment_local.x * t.fy - desc.wheels[i].attachment_local.y * t.fx;
    }
    return mz;
}

real yaw_moment_fx_part_now(const World& w, VehicleId v, const vehicle::VehicleDesc& desc) {
    real mz = 0.0;
    for (std::size_t i = 0; i < 4; ++i) mz -= desc.wheels[i].attachment_local.y * w.wheel_telemetry(v, i).fx;
    return mz;
}

// Steps the world for `window_s` seconds, sampling every tick (see
// TickSample) and running the per-tick wheel-surface validity guard. Every
// launch case in this file shares it.
std::vector<TickSample> run_accel_window(World& w, VehicleId v, BodyId chassis, const vehicle::VehicleDesc& desc,
                                          SurfaceId asphalt_id, SurfaceId low_mu_id, real window_s) {
    std::vector<TickSample> out;
    const real dt = w.dt();
    const int n = static_cast<int>(std::lround(window_s / dt));
    out.reserve(static_cast<std::size_t>(n));

    for (int i = 0; i < n; ++i) {
        w.step();
        require_wheel_surfaces(w, v, asphalt_id, low_mu_id);

        TickSample s;
        const vehicle::WheelTelemetry& fl = w.wheel_telemetry(v, kFL);
        const vehicle::WheelTelemetry& fr = w.wheel_telemetry(v, kFR);
        s.omega_fl = fl.omega;
        s.omega_fr = fr.omega;
        s.fx_fl = fl.fx;
        s.fx_fr = fr.fx;
        s.mz = yaw_moment_now(w, v, desc);
        s.mz_fx = yaw_moment_fx_part_now(w, v, desc);
        s.yaw_rate = w.get_motion(chassis).angular.z;

        const drivetrain::PowertrainDebug dbg = w.powertrain_debug(v);
        s.hub_fl = dbg.hub_shaft_torque[kFL];
        s.hub_fr = dbg.hub_shaft_torque[kFR];

        const drivetrain::PowertrainSnapshot& ps = w.powertrain_state(v);
        REQUIRE(!ps.engines.empty());
        s.limiter_active = ps.engines[0].limiter_active;

        out.push_back(s);
    }
    return out;
}

// Settles a fresh rig for 1 s, selects 1st, applies `throttle` and runs
// run_accel_window() for `window_s`.
struct LaunchRun {
    SplitMuRig rig;
    std::vector<TickSample> samples;
};

LaunchRun run_launch(const vehicle::VehicleDesc& desc, real throttle, real window_s, real tick_hz = 240.0,
                     real substep_hz = 960.0) {
    LaunchRun run{build_split_mu_rig(desc, tick_hz, substep_hz), {}};
    World& w = *run.rig.world;
    ps_test::settle(w, 1.0);
    require_wheel_surfaces(w, run.rig.vehicle, run.rig.asphalt_id, run.rig.low_mu_id);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", throttle);
    run.samples =
        run_accel_window(w, run.rig.vehicle, run.rig.chassis, desc, run.rig.asphalt_id, run.rig.low_mu_id, window_s);
    return run;
}

// Cross-checks T_i = J_i*domega_i/dt + r_i*fx_i (reconstructed from
// consecutive per-tick telemetry samples) against the drivetrain's own
// debug hub_shaft_torque[i], for every consecutive sample pair from
// `start_index` onward, asserting the plan's 2% bound (margin 1 N*m).
// Only meaningful with ONE substep per tick - see the cross-check
// TEST_CASE's own comment.
struct CrossCheckStats {
    std::size_t checked = 0;
    real worst_abs_fl = 0.0;
    real worst_abs_fr = 0.0;
};

CrossCheckStats hub_torque_cross_check(const std::vector<TickSample>& samples, const vehicle::VehicleDesc& desc,
                                       real dt, std::size_t start_index) {
    const real j_fl = desc.wheels[kFL].wheel_inertia, r_fl = desc.wheels[kFL].wheel_radius;
    const real j_fr = desc.wheels[kFR].wheel_inertia, r_fr = desc.wheels[kFR].wheel_radius;
    CrossCheckStats stats;
    for (std::size_t i = std::max<std::size_t>(start_index, 1); i < samples.size(); ++i) {
        const TickSample& prev = samples[i - 1];
        const TickSample& cur = samples[i];
        const real t_fl = j_fl * (cur.omega_fl - prev.omega_fl) / dt + r_fl * cur.fx_fl;
        const real t_fr = j_fr * (cur.omega_fr - prev.omega_fr) / dt + r_fr * cur.fx_fr;
        stats.worst_abs_fl = std::max(stats.worst_abs_fl, std::abs(t_fl - cur.hub_fl));
        stats.worst_abs_fr = std::max(stats.worst_abs_fr, std::abs(t_fr - cur.hub_fr));
        ++stats.checked;
        INFO("hub-torque cross-check tick " << i << " (t0+" << (i + 1) * dt << "s): t_fl=" << t_fl
                                              << " hub_fl=" << cur.hub_fl << " t_fr=" << t_fr
                                              << " hub_fr=" << cur.hub_fr);
        CHECK(t_fl == Catch::Approx(cur.hub_fl).epsilon(0.02).margin(1.0));
        CHECK(t_fr == Catch::Approx(cur.hub_fr).epsilon(0.02).margin(1.0));
    }
    return stats;
}

std::size_t ticks_per_second(const World& w) { return static_cast<std::size_t>(std::lround(1.0 / w.dt())); }

// Mean total front force mean(fx_FL+fx_FR) over [t0+1, t0+2] of a WOT
// launch - used by the locked case to compare against the open case.
real mean_total_fx_last_second(DifferentialKind kind) {
    const LaunchRun run = run_launch(load_sedan(kind), 1.0, 2.0);
    const std::size_t start = ticks_per_second(*run.rig.world) - 1;
    real sum = 0.0;
    for (std::size_t i = start; i < run.samples.size(); ++i) sum += run.samples[i].fx_fl + run.samples[i].fx_fr;
    return sum / static_cast<real>(run.samples.size() - start);
}

// Accelerates a fresh split-mu rig to 15 m/s (full throttle, capped at 90 s
// of sim time), then applies brake 0.5 with the clutch pressed (fully
// disengaged, steer reset to 0 exactly as the plan's Mz formula assumes) and
// returns the mean yaw moment over the following 0.5 s.
//
// The launch phase runs a closed-loop steering correction, both a yaw-rate
// damping term AND a lateral-position term: steer = clamp(-4.0*yaw_rate -
// 0.3*(y - y0), -1, 1). An OPEN diff's Willis constraint splits torque 50/50
// regardless of grip ("one wheel on ice": low_mu is polished ice,
// lambda_mu=0.1, data/surfaces/surfaces.json) and a locked one pushes the
// asphalt wheel much harder; both produce a large continuous yaw moment
// under sustained full throttle. A rate-only correction leaves a small
// steady-state yaw-rate error against that bias, which integrates into
// metres of lateral drift over a launch that takes many seconds.
//
// Sign of the position term: steer > 0 yields yaw_rate > 0 (a left/
// counterclockwise turn viewed from above, test_vehicle_steer_mirror.cpp's
// convention), which increases y - so a NEGATIVE gain on (y - y0) is the
// stabilising direction. An earlier version used +0.02*(y - y0), a
// positive-feedback loop; the C8 fix to -0.3 is correct.
//
// The correction is turned off (steer forced back to 0) before the brake/
// measurement phase starts, matching the plan's own "Mz ... (steer 0)"
// formula, so it never affects any measured quantity.
real mean_mz_during_braking(DifferentialKind kind) {
    const vehicle::VehicleDesc desc = load_sedan(kind);
    SplitMuRig rig = build_split_mu_rig(desc);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 1.0);

    const real dt = w.dt();
    // 90 s: "open" genuinely needs slightly over 60 s to reach 15 m/s (half
    // its torque goes to the freely-spinning low-mu wheel the whole way;
    // measured ~14.5 m/s at t=58s, climbing steadily).
    const int max_ticks = static_cast<int>(std::lround(90.0 / dt));
    const real y0 = w.get_pose(rig.chassis).position.y;
    bool reached_15 = false;
    for (int i = 0; i < max_ticks; ++i) {
        const real yaw_rate = w.get_motion(rig.chassis).angular.z;
        const real y = w.get_pose(rig.chassis).position.y;
        w.set_control("steer", std::clamp(real(-4.0) * yaw_rate - real(0.3) * (y - y0), real(-1.0), real(1.0)));
        w.step();
        if (w.get_motion(rig.chassis).linear.length() >= 15.0) {
            reached_15 = true;
            break;
        }
    }
    INFO("differential kind " << static_cast<int>(kind) << " failed to reach 15 m/s within 90 s of full-throttle "
                                                             "split-mu launch");
    REQUIRE(reached_15);

    // Settle phase: coast (throttle 0) under the same closed-loop steer for
    // up to 5 s, until BOTH yaw rate and lateral offset are small, so the
    // steer-0 brake window starts with the car straddling the split. Checking
    // yaw rate alone is not enough - a car can have near-zero yaw rate while
    // still sitting metres away from y0.
    for (int i = 0; i < static_cast<int>(std::lround(5.0 / dt)); ++i) {
        const real yaw_rate = w.get_motion(rig.chassis).angular.z;
        const real y = w.get_pose(rig.chassis).position.y;
        if (std::abs(yaw_rate) < 1e-4 && std::abs(y - y0) < 0.2) break;
        w.set_control("throttle", 0.0);
        w.set_control("steer", std::clamp(real(-4.0) * yaw_rate - real(0.3) * (y - y0), real(-1.0), real(1.0)));
        w.step();
    }

    w.set_control("steer", 0.0);
    w.set_control("throttle", 0.0);
    w.set_control("brake", 0.5);
    w.set_control("clutch", 1.0); // pedal pressed -> fully disengaged via the vehicle's own engagement curve

    const int brake_ticks = static_cast<int>(std::lround(0.5 / dt));
    real sum_mz = 0.0;
    for (int i = 0; i < brake_ticks; ++i) {
        w.step();
        require_wheel_surfaces(w, rig.vehicle, rig.asphalt_id, rig.low_mu_id);
        sum_mz += yaw_moment_now(w, rig.vehicle, desc);
    }
    return sum_mz / static_cast<real>(brake_ticks);
}

// Casts car_sedan's front-wheel query shape (CylinderShape half-width
// 0.1125, r 0.317, rotated -0.2 rad about z, swept 0.18 m straight down from
// z0 = 0.4374 - WheelInstance::measure_contact's own query) over a 0.01 m
// grid, x in [3, 7], y in [-0.3, 0.4], against `ground` alone, and counts
// every pose whose result disagrees with the analytic answer for a flat
// plane at z = 0: a hit, not started penetrating, fraction (z0 - r)/0.18
// within 1e-3, and (for |y| >= 0.15, clear of the seam at the shape's
// footprint) the surface of that side.
struct CastProbeStats {
    int probed = 0;
    int wrong = 0;
    real worst_dfraction = 0.0;
};

CastProbeStats probe_wheel_casts(const MeshShape& ground, SurfaceId asphalt_id, SurfaceId low_mu_id,
                                 bool print_first_wrong) {
    World w{WorldConfig{}};
    BodyDesc g;
    g.shape = ground;
    g.motion = BodyMotionType::Static;
    w.create_body(g);

    const real z0 = 0.4374, radius = 0.317, sweep = 0.18;
    const real expected_fraction = (z0 - radius) / sweep;
    const Quat q = Quat::from_axis_angle(Vec3{0, 0, 1}, -0.2);
    const CylinderShape cyl{0.1125, radius};
    CastProbeStats st;
    for (int ix = 0; ix <= 400; ++ix) {
        for (int iy = 0; iy <= 70; ++iy) {
            const Vec3 p{3.0 + ix * 0.01, -0.3 + iy * 0.01, z0};
            const ShapeCastHit h = w.backend().shape_cast(cyl, Pose{p, q}, Vec3{0, 0, -sweep}, sweep);
            ++st.probed;
            const real d = h.hit ? std::abs(h.fraction - expected_fraction) : real(1.0);
            st.worst_dfraction = std::max(st.worst_dfraction, d);
            bool ok = h.hit && !h.started_penetrating && d <= 1e-3;
            if (std::abs(p.y) >= 0.15) ok = ok && h.surface == (p.y > 0.0 ? asphalt_id : low_mu_id);
            if (!ok) {
                ++st.wrong;
                if (print_first_wrong && st.wrong <= 10) {
                    std::cout << "  wrong cast at (" << p.x << ", " << p.y << "): hit=" << h.hit
                              << " fraction=" << h.fraction << " (expected " << expected_fraction
                              << ") started_penetrating=" << h.started_penetrating << " surface=" << h.surface
                              << "\n";
                }
            }
        }
    }
    return st;
}

} // namespace

// Pins the rig: the tessellated split ground must answer the front wheel's
// own shape cast exactly like an ideal plane over the region where the
// launch cases' front wheels travel near the seam (see split_surface_mesh()
// for the 4-triangle ground that did not - 276 wrong casts here).
TEST_CASE("split-mu rig ground validity: wheel shape casts near the seam match an ideal plane",
          "[validation][vehicle][powertrain][drivetrain]") {
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    const SurfaceId asph = surfaces->id_for("asphalt"), lmu = surfaces->id_for("low_mu");
    const CastProbeStats st =
        probe_wheel_casts(split_surface_mesh(kGroundHalfExtent, kGroundCell, asph, lmu), asph, lmu, true);
    INFO("probed=" << st.probed << " wrong=" << st.wrong << " worst|dfraction|=" << st.worst_dfraction);
    CHECK(st.probed == 401 * 71);
    CHECK(st.wrong == 0);
}

TEST_CASE("split-mu launch, differential kind open: low-mu wheel spins away, total front force ~ 2x its own",
          "[validation][vehicle][powertrain][drivetrain]") {
    const LaunchRun run = run_launch(load_sedan(DifferentialKind::Open), 1.0, 2.0);
    const auto& samples = run.samples;
    const std::size_t one_s = ticks_per_second(*run.rig.world);
    REQUIRE(samples.size() == 2 * one_s);

    const TickSample& at_1s = samples[one_s - 1];
    INFO("omega_FL@t0+1s=" << at_1s.omega_fl << " omega_FR@t0+1s=" << at_1s.omega_fr);
    REQUIRE(at_1s.omega_fl > 0.01); // guard the ratio below against a near-zero denominator
    CHECK(at_1s.omega_fr / at_1s.omega_fl >= 2.0);

    real sum_total = 0.0, sum_fr = 0.0;
    for (std::size_t i = one_s - 1; i < samples.size(); ++i) {
        sum_total += samples[i].fx_fl + samples[i].fx_fr;
        sum_fr += samples[i].fx_fr;
    }
    const real n = static_cast<real>(samples.size() - (one_s - 1));
    const real mean_total = sum_total / n;
    const real mean_fr = sum_fr / n;
    INFO("mean(fx_FL+fx_FR)=" << mean_total << " 2*mean(fx_FR)=" << 2.0 * mean_fr);
    CHECK(mean_total == Catch::Approx(2.0 * mean_fr).epsilon(0.05));
}

// Locked (spool) launch, the plan's slip and force checks plus the yaw
// moment the locked axle produces, measured two ways that do not depend on
// the literal telemetry formula (see the next case for that one):
//   - the diff's own contribution, sum(-y_i*fx_i) - measured -2439 N*m mean
//     over [t0+0.2, t0+1.0];
//   - the net yaw moment on the chassis, I_z*d(yaw_rate)/dt over the same
//     window - measured -152 N*m (I_z = 2080 kg*m^2 from the backend).
// Measured on the tessellated ground: max relative |omega_FL - omega_FR| 0,
// locked mean(fx_FL+fx_FR) 3928 N vs. open 725 N.
TEST_CASE("split-mu launch, differential kind locked: near-zero slip, much more total force than open, "
          "yaw moment opposes the mu split",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_sedan(DifferentialKind::Locked);
    const LaunchRun run = run_launch(desc, 1.0, 2.0);
    const World& w = *run.rig.world;
    const auto& samples = run.samples;
    const std::size_t one_s = ticks_per_second(w);

    real max_slip = 0.0;
    for (const auto& s : samples) {
        max_slip = std::max(max_slip, std::abs(s.omega_fl - s.omega_fr) / std::max(real(1.0), std::abs(s.omega_fl)));
    }
    INFO("max relative |omega_FL - omega_FR| = " << max_slip);
    CHECK(max_slip <= 1e-9);

    real sum_total = 0.0;
    for (std::size_t i = one_s - 1; i < samples.size(); ++i) sum_total += samples[i].fx_fl + samples[i].fx_fr;
    const real locked_mean_total = sum_total / static_cast<real>(samples.size() - (one_s - 1));
    const real open_mean_total = mean_total_fx_last_second(DifferentialKind::Open);
    INFO("locked mean(fx_FL+fx_FR)=" << locked_mean_total << " open mean(fx_FL+fx_FR)=" << open_mean_total);
    CHECK(locked_mean_total >= 2.0 * open_mean_total);

    // [t0+0.2, t0+1.0]
    const std::size_t idx_lo = static_cast<std::size_t>(std::lround(0.2 / w.dt())) - 1;
    const std::size_t idx_hi = one_s - 1;
    real sum_fx_part = 0.0;
    for (std::size_t i = idx_lo; i <= idx_hi; ++i) sum_fx_part += samples[i].mz_fx;
    const real mean_fx_part = sum_fx_part / static_cast<real>(idx_hi - idx_lo + 1);
    const real i_z = w.backend().get_inertia_world(run.rig.chassis)(2, 2);
    const real net_mz = i_z * (samples[idx_hi].yaw_rate - samples[idx_lo - 1].yaw_rate) /
                        (static_cast<real>(idx_hi - idx_lo + 1) * w.dt());
    INFO("mean sum(-y*fx) [t0+0.2, t0+1.0] = " << mean_fx_part << "  I_z*dr/dt = " << net_mz << " (I_z=" << i_z
                                               << ")");
    CHECK(mean_fx_part < -100.0);
    CHECK(net_mz < -100.0);
}

// [!mayfail] - the plan's literal yaw-moment check for the locked launch:
// mean Mz = sum(x_i*fy_i - y_i*fx_i) from wheel telemetry over
// [t0+0.2, t0+1.0] < -100 N*m. Measured -68.7 N*m (-66.6 at 960/960 Hz), so
// this fails. It is not a drivetrain or harness defect. The formula is the
// sum of two large, opposing parts:
//   - sum(-y_i*fx_i) = -2439 N*m: the locked diff pushing the asphalt (left)
//     wheel harder, turning the car clockwise;
//   - sum(x_i*fy_i) = +2370 N*m (front +852, rear +1518): the lateral tyre
//     forces that build up as soon as the car starts to yaw, resisting it.
// At steer 0 the plan's -100 threshold assumes that second part is small; on
// this rig it cancels 97% of the first. The formula also leaves out two terms
// telemetry does not carry: the tyres' aligning moments (~-83 N*m) and the
// rolling-resistance forces. With them the net yaw moment is
// I_z*d(yaw_rate)/dt = -152 N*m, which the case above asserts. The literal
// check is kept as the plan wrote it, not loosened.
TEST_CASE("split-mu launch, differential kind locked: plan's telemetry yaw-moment formula < -100 N*m",
          "[validation][vehicle][powertrain][drivetrain][!mayfail]") {
    const LaunchRun run = run_launch(load_sedan(DifferentialKind::Locked), 1.0, 2.0);
    const World& w = *run.rig.world;
    const std::size_t idx_lo = static_cast<std::size_t>(std::lround(0.2 / w.dt())) - 1;
    const std::size_t idx_hi = ticks_per_second(w) - 1;
    real sum_mz = 0.0, sum_fx_part = 0.0;
    for (std::size_t i = idx_lo; i <= idx_hi; ++i) {
        sum_mz += run.samples[i].mz;
        sum_fx_part += run.samples[i].mz_fx;
    }
    const real n = static_cast<real>(idx_hi - idx_lo + 1);
    INFO("mean Mz [t0+0.2, t0+1.0] = " << sum_mz / n << " = sum(-y*fx) " << sum_fx_part / n << " + sum(x*fy) "
                                       << (sum_mz - sum_fx_part) / n);
    CHECK(sum_mz / n < -100.0);
}

TEST_CASE("split-mu braking from 15 m/s, open: yaw moment favours the higher-mu (asphalt) side",
          "[validation][long][vehicle][powertrain][drivetrain]") {
    const real mz_open = mean_mz_during_braking(DifferentialKind::Open);
    INFO("mean Mz (open, braking, first 0.5 s) = " << mz_open);
    CHECK(mz_open > 100.0);
}

// The C8 [!mayfail] here (a large lateral excursion during the launch and a
// divergence in the settle phase, ending ~5.5 m off the split) was the
// 4-triangle ground's cast artefact, which the car hits every time a wheel
// crosses the seam. On the tessellated ground, with the same controller,
// "locked" reaches 15 m/s after ~14 s with a peak |y| of ~2.7 m during the
// launch, and the settle phase converges to y = 0.04 m in 462 ticks.
TEST_CASE("split-mu braking from 15 m/s, locked: yaw moment favours the higher-mu (asphalt) side",
          "[validation][vehicle][powertrain][drivetrain]") {
    const real mz_locked = mean_mz_during_braking(DifferentialKind::Locked);
    INFO("mean Mz (locked, braking, first 0.5 s) = " << mz_locked);
    CHECK(mz_locked > 100.0);
}

// clutch_lsd and torsen saturation checks, over the plan's window
// [t0+1, t0+2], at throttle 0.6 rather than WOT.
//
// At WOT in 1st the engine reaches its fuel-cut rev limiter (6800 rpm,
// i4_2l_na.json) at t0+0.93 s and then cycles on it every ~60 ms for the
// whole window (18 cuts). Each cut swings the engine from drive to motoring
// torque, so the hub torques reverse, the clutch_lsd switches between its
// drive and coast factors, and the friction row sticks through each
// reversal - while stuck it transfers less than its capacity, which is the
// correct Coulomb behaviour. The ~20 Hz "limit cycle" the C8 comment
// described is this cycle: T_high/T_low measured 0.35-1.27x the clutch_lsd
// bound and 0.34-3.9x TBR for torsen at WOT. It is not the one-substep
// capacity lag:
//   - with the limiter raised out of reach (or at throttle 0.6), both
//     identities hold to 2e-4 (clutch_lsd 0.9999-1.0002, torsen
//     0.9998-1.0003 of nominal) across the whole window;
//   - halving the lag (1920 Hz substeps) leaves the WOT cycle unchanged
//     (19 cuts, 59.5 ms period);
//   - the lag loop's gain is below 1: the lagged capacity feeds back only
//     through T_axle, which the split does not change directly, with a gain
//     of at most drive_factor = 0.45 (clutch_lsd) or
//     (TBR-1)/(TBR+1) = 0.5 (torsen).
// At throttle 0.6 the low-mu wheel still spins hard (omega_FR ~75 rad/s vs.
// omega_FL ~6.5 at t0+2 s), so the row is genuinely slipping throughout.
// Each case asserts the limiter never fires in the checked window.
TEST_CASE("split-mu launch, clutch_lsd(preload=60, drive_factor=0.45, coast_factor=0.25): "
          "torque split saturates near preload + drive_factor*(T_high+T_low) while the row slips",
          "[validation][vehicle][powertrain][drivetrain]") {
    const LaunchRun run = run_launch(load_sedan(DifferentialKind::ClutchLsd), 0.6, 2.0);
    const auto& samples = run.samples;
    const std::size_t one_s = ticks_per_second(*run.rig.world);

    std::size_t slipping_checked = 0;
    for (std::size_t i = one_s - 1; i < samples.size(); ++i) {
        const real t_high = std::max(std::abs(samples[i].hub_fl), std::abs(samples[i].hub_fr));
        const real t_low = std::min(std::abs(samples[i].hub_fl), std::abs(samples[i].hub_fr));
        const real bound = 60.0 + 0.45 * (t_high + t_low);
        const real diff_t = t_high - t_low;
        INFO("tick " << i << " t_high=" << t_high << " t_low=" << t_low << " bound=" << bound);
        REQUIRE_FALSE(samples[i].limiter_active);
        CHECK(diff_t <= 1.02 * bound);
        // "Where the row slips" - both hubs carrying a real (non-idle) share
        // of torque is this test's proxy for the clutch pack actually
        // sliding under the strong asphalt/low_mu split.
        if (t_low > 5.0) {
            CHECK(diff_t >= 0.95 * bound);
            ++slipping_checked;
        }
    }
    INFO("slipping samples checked: " << slipping_checked << " / " << (samples.size() - (one_s - 1)));
    CHECK(slipping_checked > 0);
}

// See the clutch_lsd case above for why throttle 0.6.
TEST_CASE("split-mu launch, torsen(torque_bias_ratio=3): torque split approaches TBR once the low-mu wheel spins",
          "[validation][vehicle][powertrain][drivetrain]") {
    const LaunchRun run = run_launch(load_sedan(DifferentialKind::Torsen), 0.6, 2.0);
    const auto& samples = run.samples;
    const std::size_t one_s = ticks_per_second(*run.rig.world);

    std::size_t spinning_checked = 0;
    for (std::size_t i = one_s - 1; i < samples.size(); ++i) {
        REQUIRE_FALSE(samples[i].limiter_active);
        const real t_high = std::max(std::abs(samples[i].hub_fl), std::abs(samples[i].hub_fr));
        const real t_low = std::min(std::abs(samples[i].hub_fl), std::abs(samples[i].hub_fr));
        if (t_low <= 5.0) continue; // "once the low wheel spins" - needs a real torque on both hubs
        const real ratio = t_high / t_low;
        INFO("tick " << i << " t_high=" << t_high << " t_low=" << t_low << " ratio=" << ratio);
        CHECK(ratio <= 1.02 * 3.0);
        CHECK(ratio >= 0.95 * 3.0);
        ++spinning_checked;
    }
    INFO("spinning samples checked: " << spinning_checked << " / " << (samples.size() - (one_s - 1)));
    CHECK(spinning_checked > 0);
}

// The C8 [!mayfail] here ("drifts onto the wrong surface") was the
// 4-triangle ground's cast artefact (see split_surface_mesh()); on the
// tessellated ground the run stays on its surfaces for the whole window.
TEST_CASE("split-mu launch, viscous(25 Nm.s/rad): torque split difference equals 2*c*|domega|",
          "[validation][vehicle][powertrain][drivetrain]") {
    const LaunchRun run = run_launch(load_sedan(DifferentialKind::Viscous), 1.0, 2.0);
    const auto& samples = run.samples;
    const std::size_t one_s = ticks_per_second(*run.rig.world);

    real sum_diff = 0.0, sum_expected = 0.0;
    for (std::size_t i = one_s - 1; i < samples.size(); ++i) {
        const real t_high = std::max(std::abs(samples[i].hub_fl), std::abs(samples[i].hub_fr));
        const real t_low = std::min(std::abs(samples[i].hub_fl), std::abs(samples[i].hub_fr));
        const real domega = std::abs(samples[i].omega_fl - samples[i].omega_fr);
        sum_diff += (t_high - t_low);
        sum_expected += 2.0 * 25.0 * domega;
    }
    const real n = static_cast<real>(samples.size() - (one_s - 1));
    const real mean_diff = sum_diff / n;
    const real mean_expected = sum_expected / n;
    INFO("mean(T_high-T_low)=" << mean_diff << " mean(2*c*|domega|)=" << mean_expected);
    CHECK(mean_diff == Catch::Approx(mean_expected).epsilon(0.05).margin(1.0));
}

// Hub-torque cross-check: T_i = J_i*domega_i/dt + r_i*fx_i vs. the
// drivetrain's own debug hub_shaft_torque[i], within the plan's 2% (margin
// 1 N*m), for every kind, every tick over [t0+0.2, t0+2], WOT.
//
// Runs at 960/960 Hz - ONE substep per tick. wheel_telemetry and
// powertrain_debug expose the LAST substep's fx and shaft torque, while
// J*domega/dt over a tick is that tick's MEAN hub torque minus mean r*fx.
// The two agree only when neither changes within a tick. At 240/960 Hz they
// do change: the rev limiter's fuel cuts (every kind at WOT, from t0+0.93 s)
// and the free-spinning low-mu wheel (open/clutch_lsd/torsen) - C8 measured
// errors of 53-74 N*m. With one substep per tick the sample IS the tick's
// value, and the identity holds to rounding: worst 3.6e-11 N*m (open),
// <= 4.5e-13 N*m for the other four kinds, over 1728 ticks each.
//
// Starts at t0+0.2 s: the first ~0.1 s holds the N->1 engagement
// (t0+0.084-0.097 s), a momentum-conserving topology switch whose velocity
// jump is not a finite shaft torque (up to 134 N*m there), and the lossy
// (efficiency 0.97) diff's mesh-loss reconstruction, lagged one substep,
// which is off by up to 33 N*m while the torque ramps (0.08 N*m with
// efficiency 1).
TEST_CASE("split-mu launch, hub-torque cross-check at one substep per tick (all kinds): "
          "T=J*domega/dt+r*fx matches debug hub_shaft_torque within 2%",
          "[validation][long][vehicle][powertrain][drivetrain]") {
    for (const DifferentialKind kind : {DifferentialKind::Open, DifferentialKind::Locked, DifferentialKind::Viscous,
                                        DifferentialKind::ClutchLsd, DifferentialKind::Torsen}) {
        const vehicle::VehicleDesc desc = load_sedan(kind);
        const LaunchRun run = run_launch(desc, 1.0, 2.0, 960.0, 960.0);
        const World& w = *run.rig.world;
        REQUIRE(w.dt() == Catch::Approx(1.0 / 960.0));
        const std::size_t start_idx = static_cast<std::size_t>(std::lround(0.2 / w.dt()));
        const CrossCheckStats stats = hub_torque_cross_check(run.samples, desc, w.dt(), start_idx);
        INFO("kind=" << static_cast<int>(kind) << " checked=" << stats.checked << " worst_abs_fl="
                     << stats.worst_abs_fl << " worst_abs_fr=" << stats.worst_abs_fr);
        CHECK(stats.checked == run.samples.size() - start_idx);
    }
}

// Reproduces the 4-triangle ground's cast errors (split_surface_mesh()'s
// comment): half-extent 100/300/500/1000 m -> 0/6/80/276 wrong casts (worst
// |dfraction| 5.2e-4/0.19/0.23/0.67) vs. 0 (5.3e-4) for the tessellated
// ground, at the time of writing (Jolt v5.6.0).
TEST_CASE("split-mu diagnostic: legacy ground cast probe", "[.][diag]") {
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    const SurfaceId asph = surfaces->id_for("asphalt"), lmu = surfaces->id_for("low_mu");
    for (const real he : {100.0, 300.0, 500.0, 1000.0}) {
        std::cout << "legacy 4-triangle ground, half-extent " << he << " m:\n";
        const CastProbeStats st = probe_wheel_casts(legacy_two_quad_split_mesh(he, asph, lmu), asph, lmu, true);
        std::cout << "  probed=" << st.probed << " wrong=" << st.wrong << " worst|dfraction|=" << st.worst_dfraction
                  << "\n";
    }
    const CastProbeStats st =
        probe_wheel_casts(split_surface_mesh(kGroundHalfExtent, kGroundCell, asph, lmu), asph, lmu, true);
    std::cout << "tessellated ground (" << kGroundCell << " m cells): probed=" << st.probed << " wrong=" << st.wrong
              << " worst|dfraction|=" << st.worst_dfraction << "\n";
}

// Prints the locked car's lateral position/yaw rate through
// mean_mz_during_braking()'s launch and settle phases (same controller).
TEST_CASE("split-mu braking diagnostic: locked launch-to-15 trace", "[.][diag]") {
    const vehicle::VehicleDesc desc = load_sedan(DifferentialKind::Locked);
    SplitMuRig rig = build_split_mu_rig(desc);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 1.0);
    const real dt = w.dt();
    const real y0 = w.get_pose(rig.chassis).position.y;
    bool reached_15 = false;
    int reach_i = -1;
    real max_abs_y = 0.0;
    for (int i = 0; i < static_cast<int>(std::lround(60.0 / dt)); ++i) {
        const real yaw_rate = w.get_motion(rig.chassis).angular.z;
        const real y = w.get_pose(rig.chassis).position.y;
        max_abs_y = std::max(max_abs_y, std::abs(y - y0));
        w.set_control("steer", std::clamp(real(-4.0) * yaw_rate - real(0.3) * (y - y0), real(-1.0), real(1.0)));
        w.step();
        if (i % 24 == 0) {
            std::cerr << "launch i=" << i << " t=" << (1.0 + (i + 1) * dt) << " y=" << y << " yaw_rate=" << yaw_rate
                      << " speed=" << w.get_motion(rig.chassis).linear.length() << "\n";
        }
        if (w.get_motion(rig.chassis).linear.length() >= 15.0) {
            reached_15 = true;
            reach_i = i;
            break;
        }
    }
    std::cerr << "reached_15=" << reached_15 << " at i=" << reach_i << " max|y-y0|=" << max_abs_y << "\n";
    for (int i = 0; i < static_cast<int>(std::lround(5.0 / dt)); ++i) {
        const real yaw_rate = w.get_motion(rig.chassis).angular.z;
        const real y = w.get_pose(rig.chassis).position.y;
        if (std::abs(yaw_rate) < 1e-4 && std::abs(y - y0) < 0.2) {
            std::cerr << "settle converged at i=" << i << " y=" << y << " yaw_rate=" << yaw_rate << "\n";
            break;
        }
        w.set_control("throttle", 0.0);
        w.set_control("steer", std::clamp(real(-4.0) * yaw_rate - real(0.3) * (y - y0), real(-1.0), real(1.0)));
        w.step();
        if (i % 12 == 0) std::cerr << "settle i=" << i << " y=" << y << " yaw_rate=" << yaw_rate << "\n";
    }
    std::cerr << "final y=" << w.get_pose(rig.chassis).position.y << " yaw_rate=" << w.get_motion(rig.chassis).angular.z
              << "\n";
}
