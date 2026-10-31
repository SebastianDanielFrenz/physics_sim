#include "coupling_solver.h"

#include "../drivetrain/dense.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cstddef>

namespace ps::machine {

namespace {

constexpr real kTwoPi = 6.283185307179586476925286766559;
constexpr std::size_t kMaxRows = 6;

// Rate = linear . (va - vb) + angular_a . wa + angular_b . wb; force on a = lambda linear, torque on a = lambda
// angular_a, and on b -lambda linear, lambda angular_b.
struct Row {
    Vec3 linear{}, angular_a{}, angular_b{};
    Vec3 inv_a{}, inv_b{}; // the inverse inertias times angular_a, angular_b
    real position = 0.0, rate = 0.0;
    int side = 0; // 0 both ways; +1 an upper stop, may only push down; -1 a lower stop
};

[[nodiscard]] Vec3 unit(int k) { return k == 0 ? Vec3::unit_x() : (k == 1 ? Vec3::unit_y() : Vec3::unit_z()); }

// yaw, pitch, roll of R = Rz(yaw) Ry(pitch) Rx(roll).
[[nodiscard]] std::array<real, 3> cardan_angles(const Mat3& r) {
    return {math::atan2(r(1, 0), r(0, 0)), math::asin(std::clamp(-r(2, 0), real(-1.0), real(1.0))), math::atan2(r(2, 1), r(2, 2))};
}

struct HitchBasis {
    std::array<real, 3> angle{};
    std::array<Vec3, 3> dual{};
};

[[nodiscard]] HitchBasis hitch_basis(const SocketBody& a, const SocketBody& b) {
    HitchBasis out;
    out.angle = cardan_angles((a.hitch_frame.conjugate() * b.hitch_frame).to_mat3());
    const Vec3 e1 = a.hitch_frame.rotate(Vec3::unit_z());
    const Vec3 e3 = b.hitch_frame.rotate(Vec3::unit_x());
    const Vec3 e2 = cross(e1, e3).normalized();
    const Vec3 c23 = cross(e2, e3);
    const real volume = dot(e1, c23);
    out.dual = {c23 / volume, cross(e3, e1) / volume, cross(e1, e2) / volume};
    return out;
}

using Mat = drivetrain::SmallMat<kMaxRows, kMaxRows>;
using Vec = drivetrain::SmallVec<kMaxRows>;

} // namespace

HitchMeasure measure_hitch(const SocketBody& a, const SocketBody& b, const HitchClassDesc& hitch) {
    HitchMeasure m;
    m.error = a.socket_position - b.socket_position;
    m.error_rate = (a.linear + cross(a.angular, a.socket_offset)) - (b.linear + cross(b.angular, b.socket_offset));
    if (has_rotational_rows(hitch)) {
        const HitchBasis basis = hitch_basis(a, b);
        m.angle = basis.angle;
        for (std::size_t k = 0; k < 3; ++k) m.angle_rate[k] = dot(basis.dual[k], b.angular - a.angular);
    }
    return m;
}

SocketSolve solve_socket_coupling(const SocketBody& a, const SocketBody& b, const HitchClassDesc& hitch, real h, const LatchTarget& latch) {
    SocketSolve out;
    const real inv_mass = a.inv_mass + b.inv_mass;

    std::array<Row, kMaxRows> rows;
    std::size_t n = 0;

    out.error = a.socket_position - b.socket_position;
    const Vec3 e_rate = (a.linear + cross(a.angular, a.socket_offset)) - (b.linear + cross(b.angular, b.socket_offset));
    for (int k = 0; k < 3; ++k) {
        Row& r = rows[n++];
        r.linear = unit(k);
        r.angular_a = cross(a.socket_offset, r.linear);
        r.angular_b = cross(r.linear, b.socket_offset);
        r.position = out.error[k] - latch.error[k];
        r.rate = e_rate[k] - latch.error_rate[k];
    }

    if (has_rotational_rows(hitch)) {
        const auto [angle, dual] = hitch_basis(a, b);
        out.angle = angle;
        const Vec3 omega_rel = b.angular - a.angular;
        const std::array<const HitchAxis*, 3> axes = {&hitch.yaw, &hitch.pitch, &hitch.roll};
        for (std::size_t k = 0; k < 3; ++k) {
            const HitchAxis& axis = *axes[k];
            const real rate = dot(dual[k], omega_rel);
            real position = 0.0, target_rate = 0.0;
            int side = 0;
            if (axis.mode == HitchAxisMode::Free) continue;
            if (axis.mode == HitchAxisMode::Locked) {
                position = out.angle[k] - latch.angle[k];
                target_rate = latch.angle_rate[k];
            } else if (out.angle[k] + h * rate > axis.max_rad) {
                position = out.angle[k] - axis.max_rad;
                side = 1;
            } else if (out.angle[k] + h * rate < axis.min_rad) {
                position = out.angle[k] - axis.min_rad;
                side = -1;
            } else {
                continue;
            }
            Row& r = rows[n++];
            r.angular_a = -dual[k];
            r.angular_b = dual[k];
            r.position = position;
            r.rate = rate - target_rate;
            r.side = side;
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        rows[i].inv_a = a.inv_inertia_world * rows[i].angular_a;
        rows[i].inv_b = b.inv_inertia_world * rows[i].angular_b;
    }

    const real omega = kTwoPi * hitch.frequency_hz;
    const real g = h * omega * omega + 2.0 * hitch.damping_ratio * omega;
    const real scale = -1.0 / (1.0 + h * g);

    Mat m;
    Vec lambda;
    for (;;) {
        Vec rhs;
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j <= i; ++j) {
                m(i, j) = m(j, i) = inv_mass * dot(rows[i].linear, rows[j].linear) + dot(rows[i].angular_a, rows[j].inv_a) + dot(rows[i].angular_b, rows[j].inv_b);
            }
            rhs[i] = scale * (omega * omega * rows[i].position + g * rows[i].rate);
        }
        Mat l;
        drivetrain::cholesky_factor(m, n, l);
        drivetrain::cholesky_solve(l, n, rhs, lambda);
        // Each pass removes a row, so this ends after three solves at most.
        std::size_t kept = n;
        for (std::size_t i = n; i-- > 3;) {
            if (static_cast<real>(rows[i].side) * lambda[i] > 0.0) {
                for (std::size_t j = i + 1; j < kept; ++j) rows[j - 1] = rows[j];
                --kept;
            }
        }
        if (kept == n) break;
        n = kept;
    }

    out.rows = static_cast<int>(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.force_on_a += rows[i].linear * lambda[i];
        out.torque_on_a += rows[i].angular_a * lambda[i];
        out.torque_on_b += rows[i].angular_b * lambda[i];
        real rate_new = rows[i].rate;
        for (std::size_t j = 0; j < n; ++j) rate_new += h * m(i, j) * lambda[j];
        out.work_j += h * lambda[i] * rate_new;
    }
    return out;
}

} // namespace ps::machine
