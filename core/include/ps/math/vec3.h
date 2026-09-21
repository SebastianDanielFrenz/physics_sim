// ps/math/vec3.h — 3D vector in ps::real (double), ISO 8855 frame (PLAN.md D4).
//
// Plain, dependency-free value type. No Jolt/Godot type appears here
// (PLAN.md D1/D2). Kept trivially copyable so it is safe to place in
// cache-line-aligned per-job output slots (PLAN.md D8).

#pragma once

#include "ps/types.h"

#include <cmath>

namespace ps {

struct Vec3 {
    real x = 0.0;
    real y = 0.0;
    real z = 0.0;

    constexpr Vec3() = default;
    constexpr Vec3(real x_, real y_, real z_) : x(x_), y(y_), z(z_) {}

    static constexpr Vec3 zero() { return Vec3{0.0, 0.0, 0.0}; }
    static constexpr Vec3 unit_x() { return Vec3{1.0, 0.0, 0.0}; }
    static constexpr Vec3 unit_y() { return Vec3{0.0, 1.0, 0.0}; }
    static constexpr Vec3 unit_z() { return Vec3{0.0, 0.0, 1.0}; }

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(real s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(real s) const { return {x / s, y / s, z / s}; }

    constexpr Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(real s) { x *= s; y *= s; z *= s; return *this; }
    constexpr Vec3& operator/=(real s) { x /= s; y /= s; z /= s; return *this; }

    constexpr bool operator==(const Vec3& o) const { return x == o.x && y == o.y && z == o.z; }
    constexpr bool operator!=(const Vec3& o) const { return !(*this == o); }

    constexpr real operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }

    [[nodiscard]] constexpr real length_squared() const { return x * x + y * y + z * z; }
    [[nodiscard]] real length() const { return std::sqrt(length_squared()); }

    // Returns a zero vector for a near-zero-length input rather than NaN;
    // callers on hot paths that already guarantee non-zero length may prefer
    // to divide by length() directly.
    [[nodiscard]] Vec3 normalized() const {
        const real len = length();
        if (len < real(1e-12)) return Vec3::zero();
        return *this / len;
    }
};

constexpr Vec3 operator*(real s, const Vec3& v) { return v * s; }

constexpr real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

inline real distance(const Vec3& a, const Vec3& b) { return (a - b).length(); }

inline Vec3 lerp(const Vec3& a, const Vec3& b, real t) { return a + (b - a) * t; }

} // namespace ps
