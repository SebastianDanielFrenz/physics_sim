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

[[nodiscard]] HitchBasis hitch_basis(const Quat& a, const Quat& b) {
    HitchBasis out;
    out.angle = cardan_angles((a.conjugate() * b).to_mat3());
    const Vec3 e1 = a.rotate(Vec3::unit_z());
    const Vec3 e3 = b.rotate(Vec3::unit_x());
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
        const HitchBasis basis = hitch_basis(a.hitch_frame, b.hitch_frame);
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
        const auto [angle, dual] = hitch_basis(a.hitch_frame, b.hitch_frame);
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

namespace {

// What every linkage row needs from the bodies, worked out once.
struct LinkagePoints {
    Vec3 ra_lower, ra_top, rb_lower, rb_top; // pivots and pins from each centre of mass
    Vec3 lower, top;                         // pivot - pin, world
    Vec3 y;                                  // the towing chassis' lateral axis
    real lower_length = 0.0, top_length = 0.0, angle = 0.0;
};

[[nodiscard]] LinkagePoints linkage_points(const LinkageBody& a, const LinkageBody& b, const LinkageGeometry& g) {
    LinkagePoints p;
    p.ra_lower = a.orientation.rotate(g.lower_pivot_local);
    p.ra_top = a.orientation.rotate(g.top_pivot_local);
    p.rb_lower = b.orientation.rotate(g.lower_pin_local);
    p.rb_top = b.orientation.rotate(g.top_pin_local);
    p.lower = (a.position + p.ra_lower) - (b.position + p.rb_lower);
    p.top = (a.position + p.ra_top) - (b.position + p.rb_top);
    p.lower_length = p.lower.length();
    p.top_length = p.top.length();
    p.y = a.orientation.rotate(Vec3::unit_y());
    const Vec3 link_local = a.orientation.conjugate().rotate(p.lower * real(-1.0));
    p.angle = math::atan2(link_local.z, -link_local.x) - g.reference_angle_rad;
    return p;
}

[[nodiscard]] Row distance_row(const Vec3& separation, real length, real rest, const Vec3& ra, const Vec3& rb) {
    Row r;
    r.linear = separation / length;
    r.angular_a = cross(ra, r.linear);
    r.angular_b = cross(r.linear, rb);
    r.position = length - rest;
    return r;
}

// Across the link at the pin; the other point of the row is the towing chassis point coincident with the pin.
[[nodiscard]] Row lateral_row(const LinkagePoints& p) {
    Row r;
    r.linear = p.y;
    r.angular_a = cross(p.ra_lower - p.lower, p.y);
    r.angular_b = cross(p.y, p.rb_lower);
    r.position = dot(p.y, p.lower);
    return r;
}

// The lower link's angle row, lambda the torque about the pivot (see the header).
[[nodiscard]] Row lift_row(const LinkagePoints& p) {
    const Vec3 t = cross(p.y, p.lower * real(-1.0)) / p.lower_length;
    const real inv_l = real(1.0) / p.lower_length;
    Row r;
    r.linear = t * -inv_l;
    r.angular_a = cross(p.ra_lower, t) * -inv_l - p.y;
    r.angular_b = cross(p.rb_lower, t) * inv_l;
    return r;
}

template <typename Body>
[[nodiscard]] real row_rate(const Row& r, const Body& a, const Body& b) {
    return dot(r.linear, a.linear - b.linear) + dot(r.angular_a, a.angular) + dot(r.angular_b, b.angular);
}

} // namespace

LinkageMeasure measure_linkage(const LinkageBody& a, const LinkageBody& b, const LinkageGeometry& g) {
    LinkageMeasure m;
    const LinkagePoints p = linkage_points(a, b, g);
    const Row lower = distance_row(p.lower, p.lower_length, g.lower_length_m, p.ra_lower, p.rb_lower);
    const Row top = distance_row(p.top, p.top_length, g.top_length_m, p.ra_top, p.rb_top);
    const Row lateral = lateral_row(p);
    m.error = Vec3{lower.position, top.position, lateral.position};
    m.error_rate = Vec3{row_rate(lower, a, b), row_rate(top, a, b), row_rate(lateral, a, b)};
    const HitchBasis basis = hitch_basis(a.hitch_frame, b.hitch_frame);
    m.angle = basis.angle;
    for (std::size_t k = 0; k < 3; ++k) m.angle_rate[k] = dot(basis.dual[k], b.angular - a.angular);
    m.lift_angle = p.angle;
    m.lift_rate = row_rate(lift_row(p), a, b);
    return m;
}

LinkageSolve solve_linkage(const LinkageBody& a, const LinkageBody& b, const LinkageGeometry& g, const HitchClassDesc& hitch, real h, const LiftCommand& lift, const LatchTarget& latch) {
    LinkageSolve out;
    const real inv_mass = a.inv_mass + b.inv_mass;
    const LinkagePoints p = linkage_points(a, b, g);
    out.lift_angle = p.angle;

    std::array<Row, kMaxRows> rows;
    std::size_t n = 0;
    rows[n++] = distance_row(p.lower, p.lower_length, g.lower_length_m, p.ra_lower, p.rb_lower);
    rows[n++] = distance_row(p.top, p.top_length, g.top_length_m, p.ra_top, p.rb_top);
    rows[n++] = lateral_row(p);
    out.wrench.error = Vec3{rows[0].position, rows[1].position, rows[2].position};
    for (int k = 0; k < 3; ++k) {
        const auto i = static_cast<std::size_t>(k);
        rows[i].rate = row_rate(rows[i], a, b) - latch.error_rate[k];
        rows[i].position -= latch.error[k];
    }

    const auto [angle, dual] = hitch_basis(a.hitch_frame, b.hitch_frame);
    out.wrench.angle = angle;
    const Vec3 omega_rel = b.angular - a.angular;
    for (const std::size_t k : {std::size_t{0}, std::size_t{2}}) {
        const HitchAxis& axis = k == 0 ? hitch.yaw : hitch.roll;
        const real rate = dot(dual[k], omega_rel);
        real position = 0.0, target_rate = 0.0;
        int side = 0;
        if (axis.mode == HitchAxisMode::Free) continue;
        if (axis.mode == HitchAxisMode::Locked) {
            position = angle[k] - latch.angle[k];
            target_rate = latch.angle_rate[k];
        } else if (angle[k] + h * rate > axis.max_rad) {
            position = angle[k] - axis.max_rad;
            side = 1;
        } else if (angle[k] + h * rate < axis.min_rad) {
            position = angle[k] - axis.min_rad;
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

    // At most one row on the link's angle. A stop is mechanical, so it goes before the lift's own row; at the lower stop
    // a lift asked to raise goes first, and the stop takes over if it runs out of force.
    Row angle_row = lift_row(p);
    const real angle_rate = row_rate(angle_row, a, b);
    std::size_t lift_index = kMaxRows, stop_index = kMaxRows;
    const bool on_lower_stop = p.angle + h * angle_rate < g.min_angle_rad;
    const bool raising_off_stop = on_lower_stop && lift.active && lift.target_rad > g.min_angle_rad;
    if (p.angle + h * angle_rate > g.max_angle_rad) {
        angle_row.position = p.angle - g.max_angle_rad;
        angle_row.rate = angle_rate;
        angle_row.side = 1;
        stop_index = n;
        rows[n++] = angle_row;
    } else if (on_lower_stop && !raising_off_stop) {
        angle_row.position = p.angle - g.min_angle_rad;
        angle_row.rate = angle_rate;
        angle_row.side = -1;
        stop_index = n;
        rows[n++] = angle_row;
    } else if (lift.active) {
        angle_row.position = p.angle - lift.target_rad;
        angle_row.rate = angle_rate - lift.target_rate;
        angle_row.side = -1; // it only lifts
        lift_index = n;
        rows[n++] = angle_row;
    }

    for (std::size_t i = 0; i < n; ++i) {
        rows[i].inv_a = a.inv_inertia_world * rows[i].angular_a;
        rows[i].inv_b = b.inv_inertia_world * rows[i].angular_b;
    }

    const real omega = kTwoPi * hitch.frequency_hz;
    const real g_rate = h * omega * omega + 2.0 * hitch.damping_ratio * omega;
    const real scale = -1.0 / (1.0 + h * g_rate);

    Mat m;
    Vec lambda;
    real held_lift = 0.0; // a lift out of force: a known torque, its row out of the matrix
    Row held_row{};
    for (;;) {
        Vec rhs;
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j <= i; ++j) {
                m(i, j) = m(j, i) = inv_mass * dot(rows[i].linear, rows[j].linear) + dot(rows[i].angular_a, rows[j].inv_a) + dot(rows[i].angular_b, rows[j].inv_b);
            }
            rhs[i] = scale * (omega * omega * rows[i].position + g_rate * rows[i].rate);
            if (held_lift != 0.0) {
                rhs[i] -= held_lift * (inv_mass * dot(rows[i].linear, held_row.linear) + dot(rows[i].angular_a, held_row.inv_a) + dot(rows[i].angular_b, held_row.inv_b));
            }
        }
        Mat l;
        drivetrain::cholesky_factor(m, n, l);
        drivetrain::cholesky_solve(l, n, rhs, lambda);
        // The links and locked rows stay; each pass removes a row, so this ends.
        std::size_t kept = n;
        bool replaced = false;
        for (std::size_t i = n; i-- > 3;) {
            const bool saturated = i == lift_index && lambda[i] > g.max_lift_torque_nm;
            if (!saturated && static_cast<real>(rows[i].side) * lambda[i] <= 0.0) continue;
            if (saturated) {
                held_lift = g.max_lift_torque_nm;
                held_row = rows[i];
                out.saturated = true;
                if (on_lower_stop) {
                    rows[i].position = p.angle - g.min_angle_rad;
                    rows[i].rate = angle_rate;
                    stop_index = i;
                    lift_index = kMaxRows;
                    replaced = true;
                    continue;
                }
            }
            if (i == lift_index) lift_index = kMaxRows;
            if (i == stop_index) stop_index = kMaxRows;
            if (lift_index != kMaxRows && lift_index > i) --lift_index;
            if (stop_index != kMaxRows && stop_index > i) --stop_index;
            for (std::size_t j = i + 1; j < kept; ++j) rows[j - 1] = rows[j];
            --kept;
        }
        if (kept == n && !replaced) break;
        n = kept;
    }

    out.wrench.rows = static_cast<int>(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.wrench.force_on_a += rows[i].linear * lambda[i];
        out.wrench.torque_on_a += rows[i].angular_a * lambda[i];
        out.wrench.torque_on_b += rows[i].angular_b * lambda[i];
        real rate_new = rows[i].rate;
        for (std::size_t j = 0; j < n; ++j) rate_new += h * m(i, j) * lambda[j];
        out.wrench.work_j += h * lambda[i] * rate_new;
    }
    out.wrench.force_on_a += held_row.linear * held_lift;
    out.wrench.torque_on_a += held_row.angular_a * held_lift;
    out.wrench.torque_on_b += held_row.angular_b * held_lift;

    out.lower_tension_n = -lambda[0];
    out.top_compression_n = lambda[1];
    out.lift_torque_nm = held_lift;
    out.lift_acting = held_lift != 0.0 || lift_index != kMaxRows;
    if (lift_index != kMaxRows) out.lift_torque_nm = lambda[lift_index];
    // What a lift out of force did not manage is the stop's: together they are what raising would take.
    if (stop_index != kMaxRows && held_lift != 0.0) out.lift_torque_nm += lambda[stop_index];
    return out;
}

} // namespace ps::machine
