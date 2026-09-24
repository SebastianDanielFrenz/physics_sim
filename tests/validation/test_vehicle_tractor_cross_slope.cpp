// tests/validation/test_vehicle_tractor_cross_slope.cpp — PHYS-033 I1 spec
// test 11: a tractor parked on a cross slope, handbrake only, must not slide
// or yaw (owner-reported: it does, on data/vehicles/tractor.json as shipped
// before this file's H0 finding). See the I1 plan (S3, "tractor.json
// geometry is inconsistent") and CLAUDE.md's "Vehicle system" section.
//
// F15 (I1 plan): tractor.json's rear wheel attachment z (-0.725) is
// inconsistent with the wheel radii the rig file (data/models/tractor/
// tractor.rig.json) actually places on the ground - front wheel centres sit
// at 0.8 m (== front radius), rear at 1.025 m (== rear radius), both
// measured from the ground plane at rest. In the CHASSIS frame that puts
// the "ground under this wheel" offset at attachment.z - radius: front
// -0.5 - 0.8 = -1.30, rear -0.725 - 1.025 = -1.75 - NOT equal, so a level
// chassis cannot touch both axles at once; the tractor sits about
// atan(0.45/2.80) = 9.1 deg nose-down at rest. Consistent data needs
// rear z = -0.5 + (1.025 - 0.8) = -0.275 (matches front's -1.30 ground
// offset: -0.275 - 1.025 = -1.30).
//
// The acceptance cases below run the STOCK (uncorrected) tractor.json and
// are therefore expected to fail today - [!mayfail] until the coordinator's
// data-fix commit (I1 plan section 7, commit G1) lands and this tag is
// removed.

#include "vehicle_test_helpers.h"

#include "ps/math/transcendental.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

using namespace ps;

namespace {

// Relative orientation of the chassis w.r.t. the tilted ground plane -> a
// (yaw, pitch, roll) triple in radians. Diagnostic only (printf'd), so a
// small-angle-friendly quaternion decomposition is enough; not used by any
// CHECK/REQUIRE.
struct Attitude { real yaw = 0.0, pitch = 0.0, roll = 0.0; };

Attitude attitude_relative_to_ground(const Pose& chassis_pose, const Quat& ground_tilt) {
    const Quat rel = ground_tilt.inverse() * chassis_pose.orientation;
    const Vec3 local_x = rel.rotate(Vec3::unit_x());
    const Vec3 local_y = rel.rotate(Vec3::unit_y());
    Attitude a;
    a.yaw = ps::math::atan2(local_x.y, local_x.x);
    a.pitch = ps::math::asin(std::clamp(-local_x.z, real(-1.0), real(1.0)));
    a.roll = ps::math::asin(std::clamp(local_y.z, real(-1.0), real(1.0)));
    return a;
}

Quat ground_tilt_for(real slope_grade, const Vec3& tilt_axis) {
    const real theta = ps::math::atan(slope_grade);
    return Quat::from_axis_angle(tilt_axis.normalized(), theta);
}

// --- Acceptance cases (spec test 11) ---------------------------------------

void run_acceptance_case(real slope_grade, const Vec3& tilt_axis, const char* label) {
    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, slope_grade, 4, tilt_axis);
    rig.world->set_control("handbrake", 1.0);
    ps_test::settle(*rig.world, 2.0);

    const Pose p0 = rig.world->get_pose(rig.chassis);
    const Quat tilt = ground_tilt_for(slope_grade, tilt_axis);
    const Attitude att0 = attitude_relative_to_ground(p0, tilt);

    const std::size_t wc = rig.world->vehicle_wheel_count(rig.vehicle);
    std::vector<real> max_alpha(wc, 0.0), max_kappa(wc, 0.0), min_fz(wc, std::numeric_limits<real>::max());

    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(30.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        for (std::size_t w = 0; w < wc; ++w) {
            const vehicle::WheelTelemetry& t = rig.world->wheel_telemetry(rig.vehicle, w);
            max_alpha[w] = std::max(max_alpha[w], std::abs(t.slip_angle));
            max_kappa[w] = std::max(max_kappa[w], std::abs(t.slip_ratio));
            min_fz[w] = std::min(min_fz[w], t.fz);
        }
    }

    const Pose p1 = rig.world->get_pose(rig.chassis);
    const Attitude att1 = attitude_relative_to_ground(p1, tilt);
    const real drift = (p1.position - p0.position).length();
    const real yaw_change = att1.yaw - att0.yaw;

    INFO(label << ": drift=" << drift << " m, yaw_change=" << yaw_change
               << " rad, pitch0=" << att0.pitch << " rad");
    for (std::size_t w = 0; w < wc; ++w) {
        INFO("wheel " << w << ": max|alpha|=" << max_alpha[w] << " max|kappa|=" << max_kappa[w]
                       << " min_fz=" << min_fz[w]);
        CHECK(max_alpha[w] < 0.05);
        CHECK(max_kappa[w] < 0.05);
        CHECK(min_fz[w] > 0.0);
    }
    CHECK(drift < 0.001);
    CHECK(std::abs(yaw_change) < 1e-3);
}

} // namespace

// PHYS-033 I1 commit G1 fixed data/vehicles/tractor.json's rear attachment z
// (F15/H0 - see that commit's message and the file's own updated "source"
// field). [!mayfail] removed: measured drift is now 5.3e-5 m (cross slope)
// and 2.7e-4 m (longitudinal), both comfortably under the 1 mm/1e-3 rad
// bounds.
TEST_CASE("Tractor holds a 10% cross slope on the handbrake alone", "[validation][vehicle][long]") {
    run_acceptance_case(0.10, Vec3::unit_x(), "10% cross slope");
}

TEST_CASE("Tractor holds a 15% longitudinal slope on the handbrake alone", "[validation][vehicle][long]") {
    run_acceptance_case(0.15, Vec3::unit_y(), "15% longitudinal slope");
}

// Control: the CAR on the same 10% cross slope with its handbrake. If this
// also slides, the cause is generic to the vehicle system (e.g. F16's
// chassis-axis suspension force, or the wheel-centre contact point, F5),
// not specific to the tractor's own data - see I1 plan section 6.
TEST_CASE("Diagnostic: car on a 10% cross slope with handbrake (control for the tractor case)",
          "[.][diag][vehicle]") {
    const vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.10, 4, Vec3::unit_x());
    rig.world->set_control("handbrake", 1.0);
    ps_test::settle(*rig.world, 2.0);
    const Pose p0 = rig.world->get_pose(rig.chassis);
    ps_test::settle(*rig.world, 10.0);
    const Pose p1 = rig.world->get_pose(rig.chassis);
    const real drift = (p1.position - p0.position).length();
    std::printf("[diag] car on 10%% cross slope, handbrake only: drift over 10s = %.5f m\n", static_cast<double>(drift));
}

// --- Section 6: tractor cross-slope A/B hypothesis diagnostics -------------
//
// [.] keeps this out of default runs and CTest discovery (I1 plan section
// 6.1). Run by hand:
//   .\out\build\release\tests\validation\ps_validation_tests.exe "[diag]" -s
// Run once against W0 (today's one-tick-delayed chassis extrapolation) and
// again after W1 lands (I1 plan commit D) - the diff between the two runs
// IS the spec's "W1 on/off" A/B. No permanent runtime switch is added.
namespace {

struct Variant {
    const char* name;
    bool fix_geometry = false;
    Vec3 box{1.2, 0.35, 0.9};
    real brake = 0.0;
    real handbrake = 1.0;
    real sigma_y_scale = 1.0;
    real front_qsy_override = -1.0; // < 0 == leave qsy1/qsy4 as loaded (H6)
    real drop_height = 0.0;         // extra spawn height above the baseline origin (H3)
};

void run_variant(const Variant& v) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
    if (v.fix_geometry) {
        for (auto& w : desc.wheels) {
            if (!w.is_front) w.attachment_local.z = -0.275; // F15's consistent value
        }
    }
    for (auto& w : desc.wheels) {
        w.tyre.relaxation_length_y *= v.sigma_y_scale;
        if (v.front_qsy_override >= 0.0 && w.is_front) {
            w.tyre.qsy1 = v.front_qsy_override;
            w.tyre.qsy4 = v.front_qsy_override;
        }
    }

    const real origin_height = 1.28 + v.drop_height; // I1 plan: 1.30 - 0.02 static compression
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 6000.0, v.box, origin_height, 0.10, 4, Vec3::unit_x());
    rig.world->set_control("handbrake", v.handbrake);
    rig.world->set_control("brake", v.brake);

    ps_test::settle(*rig.world, 2.0);
    const Pose p0 = rig.world->get_pose(rig.chassis);
    const Quat tilt = ground_tilt_for(0.10, Vec3::unit_x());
    const Attitude att0 = attitude_relative_to_ground(p0, tilt);

    const std::size_t wc = rig.world->vehicle_wheel_count(rig.vehicle);
    std::vector<real> fz_min(wc, std::numeric_limits<real>::max()), fz_max(wc, std::numeric_limits<real>::min()),
        max_alpha(wc, 0.0), max_kappa(wc, 0.0);
    real pitch_sum = 0.0, roll_sum = 0.0, max_roll_rate = 0.0;
    real dv_err_max = 0.0;
    int n_samples = 0;

    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(10.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        for (std::size_t w = 0; w < wc; ++w) {
            const vehicle::WheelTelemetry& t = rig.world->wheel_telemetry(rig.vehicle, w);
            fz_min[w] = std::min(fz_min[w], t.fz);
            fz_max[w] = std::max(fz_max[w], t.fz);
            max_alpha[w] = std::max(max_alpha[w], std::abs(t.slip_angle));
            max_kappa[w] = std::max(max_kappa[w], std::abs(t.slip_ratio));
        }
        const Pose p = rig.world->get_pose(rig.chassis);
        const Attitude att = attitude_relative_to_ground(p, tilt);
        pitch_sum += att.pitch;
        roll_sum += att.roll;
        const Vec3 local_roll_axis = p.orientation.rotate(Vec3::unit_x());
        max_roll_rate = std::max(max_roll_rate, std::abs(dot(rig.world->get_motion(rig.chassis).angular, local_roll_axis)));
        ++n_samples;
    }

    const Pose p1 = rig.world->get_pose(rig.chassis);
    const Attitude att1 = attitude_relative_to_ground(p1, tilt);
    const real drift = (p1.position - p0.position).length();
    const real yaw_change = att1.yaw - att0.yaw;
    const real mean_pitch_deg = (pitch_sum / n_samples) * real(180.0) / real(3.14159265358979323846);
    const real mean_roll_deg = (roll_sum / n_samples) * real(180.0) / real(3.14159265358979323846);

    std::printf("%-40s drift=%8.4f yaw=%8.4f pitch=%7.2fdeg roll=%7.2fdeg roll_rate_max=%7.3f dv_err_max=%8.5f\n",
                v.name, static_cast<double>(drift), static_cast<double>(yaw_change),
                static_cast<double>(mean_pitch_deg), static_cast<double>(mean_roll_deg),
                static_cast<double>(max_roll_rate), static_cast<double>(dv_err_max));
    for (std::size_t w = 0; w < wc; ++w) {
        std::printf("    wheel%zu Fz[%8.1f,%8.1f] max|alpha|=%6.3f max|kappa|=%6.3f\n", w,
                    static_cast<double>(fz_min[w]), static_cast<double>(fz_max[w]),
                    static_cast<double>(max_alpha[w]), static_cast<double>(max_kappa[w]));
    }
}

} // namespace

TEST_CASE("tractor cross-slope A/B diagnostics", "[.][diag][vehicle]") {
    // H0: flat ground, no controls - isolates the pure geometry-induced
    // self-propulsion (spec section 6.2) from anything slope-related.
    {
        vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
        auto rig = ps_test::build_vehicle_rig_from_desc(desc, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 0.0, 4, Vec3::unit_x());
        ps_test::settle(*rig.world, 1.0);
        const Pose p0 = rig.world->get_pose(rig.chassis);
        const Motion m0 = rig.world->get_motion(rig.chassis);
        const Quat flat = ground_tilt_for(0.0, Vec3::unit_x());
        const Attitude att0 = attitude_relative_to_ground(p0, flat);
        ps_test::settle(*rig.world, 4.0);
        const Motion m1 = rig.world->get_motion(rig.chassis);
        const real dvx = (m1.linear - m0.linear).length() / 4.0;
        std::printf("%-40s pitch0=%7.2fdeg forward_accel_est=%8.4f m/s^2\n", "H0a flat, no controls, uncorrected geom",
                    static_cast<double>(att0.pitch * real(180.0) / real(3.14159265358979323846)), static_cast<double>(dvx));
    }
    {
        vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
        for (auto& w : desc.wheels) if (!w.is_front) w.attachment_local.z = -0.275;
        auto rig = ps_test::build_vehicle_rig_from_desc(desc, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 0.0, 4, Vec3::unit_x());
        ps_test::settle(*rig.world, 1.0);
        const Pose p0 = rig.world->get_pose(rig.chassis);
        const Motion m0 = rig.world->get_motion(rig.chassis);
        const Quat flat = ground_tilt_for(0.0, Vec3::unit_x());
        const Attitude att0 = attitude_relative_to_ground(p0, flat);
        ps_test::settle(*rig.world, 4.0);
        const Motion m1 = rig.world->get_motion(rig.chassis);
        const real dvx = (m1.linear - m0.linear).length() / 4.0;
        std::printf("%-40s pitch0=%7.2fdeg forward_accel_est=%8.4f m/s^2\n", "H0b flat, no controls, fixed geom",
                    static_cast<double>(att0.pitch * real(180.0) / real(3.14159265358979323846)), static_cast<double>(dvx));
    }

    std::printf("--- baseline: handbrake only, current data, box {1.2,0.35,0.9}, 10%% cross slope ---\n");
    run_variant(Variant{"baseline (handbrake only)"});
    run_variant(Variant{"H0b fix_geometry", true});
    run_variant(Variant{"H1 small box (I_xx small)", false, Vec3{1.2, 0.3, 0.3}});
    run_variant(Variant{"H2 all four brakes + handbrake", false, Vec3{1.2, 0.35, 0.9}, 1.0, 1.0});
    run_variant(Variant{"H3 drop 0.05m extra", false, Vec3{1.2, 0.35, 0.9}, 0.0, 1.0, 1.0, -1.0, 0.05});
    run_variant(Variant{"H4 sigma_y halved", false, Vec3{1.2, 0.35, 0.9}, 0.0, 1.0, 0.5});
    run_variant(Variant{"H6 front qsy=0", false, Vec3{1.2, 0.35, 0.9}, 0.0, 1.0, 1.0, 0.0});
    run_variant(Variant{"H0b+H2 fixed geom + all brakes", true, Vec3{1.2, 0.35, 0.9}, 1.0, 1.0});
}
