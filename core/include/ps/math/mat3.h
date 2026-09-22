// ps/math/mat3.h — 3x3 matrix in ps::real, row-major storage.
//
// Used for inertia tensors and rotation matrices derived from Quat. Not a
// hot-path SIMD type in P1 (see ps/math/simd.h for that skeleton).

#pragma once

#include "ps/math/vec3.h"

namespace ps {

struct Mat3 {
    // rows[i] is row i: (m[i][0], m[i][1], m[i][2])
    Vec3 rows[3];

    constexpr Mat3() : rows{Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}} {}
    constexpr Mat3(const Vec3& r0, const Vec3& r1, const Vec3& r2) : rows{r0, r1, r2} {}

    static constexpr Mat3 identity() { return Mat3{}; }
    static constexpr Mat3 zero() { return Mat3{Vec3::zero(), Vec3::zero(), Vec3::zero()}; }

    static constexpr Mat3 diagonal(real xx, real yy, real zz) {
        return Mat3{Vec3{xx, 0, 0}, Vec3{0, yy, 0}, Vec3{0, 0, zz}};
    }

    constexpr real operator()(int r, int c) const { return rows[r][c]; }

    constexpr Vec3 col(int c) const { return Vec3{rows[0][c], rows[1][c], rows[2][c]}; }

    constexpr Vec3 operator*(const Vec3& v) const {
        return Vec3{dot(rows[0], v), dot(rows[1], v), dot(rows[2], v)};
    }

    constexpr Mat3 operator*(const Mat3& o) const {
        const Vec3 c0 = o.col(0);
        const Vec3 c1 = o.col(1);
        const Vec3 c2 = o.col(2);
        return Mat3{
            Vec3{dot(rows[0], c0), dot(rows[0], c1), dot(rows[0], c2)},
            Vec3{dot(rows[1], c0), dot(rows[1], c1), dot(rows[1], c2)},
            Vec3{dot(rows[2], c0), dot(rows[2], c1), dot(rows[2], c2)},
        };
    }

    constexpr Mat3 operator+(const Mat3& o) const {
        return Mat3{rows[0] + o.rows[0], rows[1] + o.rows[1], rows[2] + o.rows[2]};
    }

    constexpr Mat3 operator-(const Mat3& o) const {
        return Mat3{rows[0] - o.rows[0], rows[1] - o.rows[1], rows[2] - o.rows[2]};
    }

    constexpr Mat3 operator*(real s) const { return Mat3{rows[0] * s, rows[1] * s, rows[2] * s}; }

    [[nodiscard]] constexpr Mat3 transposed() const {
        return Mat3{col(0), col(1), col(2)};
    }

    [[nodiscard]] constexpr real trace() const { return rows[0].x + rows[1].y + rows[2].z; }

    [[nodiscard]] constexpr real determinant() const {
        return rows[0].x * (rows[1].y * rows[2].z - rows[1].z * rows[2].y)
             - rows[0].y * (rows[1].x * rows[2].z - rows[1].z * rows[2].x)
             + rows[0].z * (rows[1].x * rows[2].y - rows[1].y * rows[2].x);
    }

    // Returns identity for a near-singular matrix rather than dividing by
    // ~0 (callers that already know the matrix is invertible, e.g. a valid
    // inertia tensor, may skip this guard by reading determinant() first).
    [[nodiscard]] constexpr Mat3 inverse() const {
        const real det = determinant();
        if (det > real(-1e-12) && det < real(1e-12)) return Mat3::identity();
        const real inv_det = real(1.0) / det;
        const Vec3 c0{
            (rows[1].y * rows[2].z - rows[1].z * rows[2].y) * inv_det,
            (rows[0].z * rows[2].y - rows[0].y * rows[2].z) * inv_det,
            (rows[0].y * rows[1].z - rows[0].z * rows[1].y) * inv_det,
        };
        const Vec3 c1{
            (rows[1].z * rows[2].x - rows[1].x * rows[2].z) * inv_det,
            (rows[0].x * rows[2].z - rows[0].z * rows[2].x) * inv_det,
            (rows[0].z * rows[1].x - rows[0].x * rows[1].z) * inv_det,
        };
        const Vec3 c2{
            (rows[1].x * rows[2].y - rows[1].y * rows[2].x) * inv_det,
            (rows[0].y * rows[2].x - rows[0].x * rows[2].y) * inv_det,
            (rows[0].x * rows[1].y - rows[0].y * rows[1].x) * inv_det,
        };
        // c0/c1/c2 are the inverse matrix's COLUMNS; transpose into rows.
        return Mat3{Vec3{c0.x, c1.x, c2.x}, Vec3{c0.y, c1.y, c2.y}, Vec3{c0.z, c1.z, c2.z}};
    }
};

// Cross-product (skew-symmetric) matrix: skew(v) * w == cross(v, w).
constexpr Mat3 skew(const Vec3& v) {
    return Mat3{
        Vec3{0, -v.z, v.y},
        Vec3{v.z, 0, -v.x},
        Vec3{-v.y, v.x, 0},
    };
}

} // namespace ps
