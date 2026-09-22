// ps/math/quat.h — unit quaternion orientation in ps::real.
//
// Hamilton convention, (x, y, z, w) storage, right-handed rotations
// consistent with the ISO 8855 core frame (PLAN.md D4).

#pragma once

#include "ps/math/mat3.h"
#include "ps/math/transcendental.h"
#include "ps/math/vec3.h"

#include <cmath> // std::sqrt only (IEEE-exact; PLAN.md D7 exempts it, see ps/math/transcendental.h)

namespace ps {

struct Quat {
    real x = 0.0;
    real y = 0.0;
    real z = 0.0;
    real w = 1.0;

    constexpr Quat() = default;
    constexpr Quat(real x_, real y_, real z_, real w_) : x(x_), y(y_), z(z_), w(w_) {}

    static constexpr Quat identity() { return Quat{0, 0, 0, 1}; }

    static Quat from_axis_angle(const Vec3& axis, real angle_rad) {
        const Vec3 n = axis.normalized();
        const real half = angle_rad * real(0.5);
        real s, c;
        ps::math::sincos(half, s, c);
        return Quat{n.x * s, n.y * s, n.z * s, c};
    }

    [[nodiscard]] constexpr real length_squared() const { return x * x + y * y + z * z + w * w; }
    [[nodiscard]] real length() const { return std::sqrt(length_squared()); }

    [[nodiscard]] Quat normalized() const {
        const real len = length();
        if (len < real(1e-12)) return Quat::identity();
        const real inv = real(1.0) / len;
        return Quat{x * inv, y * inv, z * inv, w * inv};
    }

    [[nodiscard]] constexpr Quat conjugate() const { return Quat{-x, -y, -z, w}; }
    // Unit quaternions: inverse == conjugate.
    [[nodiscard]] constexpr Quat inverse() const { return conjugate(); }

    constexpr Quat operator*(const Quat& o) const {
        return Quat{
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w,
            w * o.w - x * o.x - y * o.y - z * o.z,
        };
    }

    // Rotates v by this quaternion (v' = q * v * q^-1), expanded to avoid
    // constructing an intermediate pure quaternion.
    [[nodiscard]] constexpr Vec3 rotate(const Vec3& v) const {
        const Vec3 qv{x, y, z};
        const Vec3 t = cross(qv, v) * real(2.0);
        return v + t * w + cross(qv, t);
    }

    [[nodiscard]] Mat3 to_mat3() const {
        const real xx = x * x, yy = y * y, zz = z * z;
        const real xy = x * y, xz = x * z, yz = y * z;
        const real wx = w * x, wy = w * y, wz = w * z;
        return Mat3{
            Vec3{1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)},
            Vec3{2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)},
            Vec3{2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)},
        };
    }
};

inline Quat slerp(const Quat& a, const Quat& b, real t) {
    real cos_half_theta = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    Quat b2 = b;
    if (cos_half_theta < 0) {
        b2 = Quat{-b.x, -b.y, -b.z, -b.w};
        cos_half_theta = -cos_half_theta;
    }
    if (cos_half_theta > real(0.9995)) {
        // Nearly parallel: linear interpolation avoids a 0/0 division below.
        return Quat{
            a.x + (b2.x - a.x) * t,
            a.y + (b2.y - a.y) * t,
            a.z + (b2.z - a.z) * t,
            a.w + (b2.w - a.w) * t,
        }.normalized();
    }
    const real half_theta = ps::math::acos(cos_half_theta);
    const real sin_half_theta = ps::math::sin(half_theta);
    const real ra = ps::math::sin((1 - t) * half_theta) / sin_half_theta;
    const real rb = ps::math::sin(t * half_theta) / sin_half_theta;
    return Quat{
        a.x * ra + b2.x * rb,
        a.y * ra + b2.y * rb,
        a.z * ra + b2.z * rb,
        a.w * ra + b2.w * rb,
    };
}

} // namespace ps
