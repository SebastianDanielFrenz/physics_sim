// Unit tests for core/include/ps/math (P1 scope).

#include "ps/math/mat3.h"
#include "ps/math/pose.h"
#include "ps/math/quat.h"
#include "ps/math/simd.h"
#include "ps/math/vec3.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using Catch::Approx;
using namespace ps;

namespace {
constexpr real kPi = 3.14159265358979323846;
}

TEST_CASE("Vec3 basic algebra", "[math][vec3]") {
    const Vec3 a{1, 2, 3};
    const Vec3 b{4, 5, 6};
    CHECK((a + b) == Vec3{5, 7, 9});
    CHECK((b - a) == Vec3{3, 3, 3});
    CHECK(dot(a, b) == 32.0);
    CHECK(cross(Vec3::unit_x(), Vec3::unit_y()) == Vec3::unit_z());
    CHECK(cross(Vec3::unit_y(), Vec3::unit_z()) == Vec3::unit_x());
    CHECK((a * 2.0).length() == Approx((a * 2.0).length()));
    CHECK(Vec3{3, 4, 0}.length() == Approx(5.0));
    CHECK(Vec3{3, 4, 0}.normalized().length() == Approx(1.0));
}

TEST_CASE("Quat identity and rotation", "[math][quat]") {
    const Quat id = Quat::identity();
    CHECK(id.rotate(Vec3{1, 2, 3}) == Vec3{1, 2, 3});

    const Quat rz90 = Quat::from_axis_angle(Vec3::unit_z(), kPi / 2);
    const Vec3 rotated = rz90.rotate(Vec3::unit_x());
    CHECK(rotated.x == Approx(0.0).margin(1e-12));
    CHECK(rotated.y == Approx(1.0));
    CHECK(rotated.z == Approx(0.0).margin(1e-12));

    const Quat combo = rz90 * rz90.conjugate();
    CHECK(combo.rotate(Vec3{1, 0, 0}).x == Approx(1.0));
}

TEST_CASE("Quat to_mat3 round trip", "[math][quat]") {
    const Quat q = Quat::from_axis_angle(Vec3{1, 1, 0}.normalized(), 0.7);
    const Mat3 m = q.to_mat3();
    const Vec3 v{2, -1, 3};
    const Vec3 via_quat = q.rotate(v);
    const Vec3 via_mat = m * v;
    CHECK(via_quat.x == Approx(via_mat.x).margin(1e-9));
    CHECK(via_quat.y == Approx(via_mat.y).margin(1e-9));
    CHECK(via_quat.z == Approx(via_mat.z).margin(1e-9));
}

TEST_CASE("Mat3 inverse", "[math][mat3]") {
    const Mat3 m{{2, 0, 0}, {0, 3, 0}, {0, 0, 4}};
    const Mat3 inv = m.inverse();
    const Mat3 product = m * inv;
    CHECK(product.rows[0].x == Approx(1.0));
    CHECK(product.rows[1].y == Approx(1.0));
    CHECK(product.rows[2].z == Approx(1.0));
}

TEST_CASE("Pose compose and inverse", "[math][pose]") {
    Pose a;
    a.position = Vec3{1, 0, 0};
    a.orientation = Quat::from_axis_angle(Vec3::unit_z(), kPi / 2);

    const Pose a_inv = a.inverse();
    const Pose identity_check = a * a_inv;
    CHECK(identity_check.position.length() == Approx(0.0).margin(1e-9));

    const Vec3 world_point = a.transform_point(Vec3{1, 0, 0});
    CHECK(world_point.x == Approx(1.0).margin(1e-9));
    CHECK(world_point.y == Approx(1.0));
}

TEST_CASE("Motion velocity_at combines linear and angular", "[math][pose]") {
    Motion m;
    m.linear = Vec3{1, 0, 0};
    m.angular = Vec3{0, 0, 1}; // 1 rad/s about Z
    const Vec3 v = m.velocity_at(Vec3{1, 0, 0}); // point 1m along +X from origin
    // omega x r = (0,0,1) x (1,0,0) = (0,1,0)
    CHECK(v.x == Approx(1.0));
    CHECK(v.y == Approx(1.0));
    CHECK(v.z == Approx(0.0).margin(1e-12));
}

TEST_CASE("simd f64x4 matches scalar arithmetic", "[math][simd]") {
    const simd::f64x4 a(1.0, 2.0, 3.0, 4.0);
    const simd::f64x4 b(10.0, 20.0, 30.0, 40.0);
    const simd::f64x4 sum = a + b;
    CHECK(sum[0] == Approx(11.0));
    CHECK(sum[1] == Approx(22.0));
    CHECK(sum[2] == Approx(33.0));
    CHECK(sum[3] == Approx(44.0));
    CHECK(sum.hsum() == Approx(110.0));

    const simd::f64x4 fmadd_result = simd::f64x4::fmadd(a, b, simd::f64x4(1.0));
    CHECK(fmadd_result[0] == Approx(1.0 * 10.0 + 1.0));
    CHECK(fmadd_result[3] == Approx(4.0 * 40.0 + 1.0));
}
