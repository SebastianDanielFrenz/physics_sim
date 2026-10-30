#include "coupling_solver.h"

namespace ps::machine {

namespace {

constexpr real kTwoPi = 6.283185307179586476925286766559;

[[nodiscard]] Mat3 socket_inverse_mass(const SocketBody& b) {
    const Mat3 r = skew(b.socket_offset);
    return Mat3::diagonal(b.inv_mass, b.inv_mass, b.inv_mass) + r * b.inv_inertia_world * r.transposed();
}

} // namespace

SocketSolve solve_socket_coupling(const SocketBody& a, const SocketBody& b, const CouplingDesc& desc, real h) {
    const Vec3 e = a.socket_position - b.socket_position;
    const Vec3 edot_free = (a.linear + cross(a.angular, a.socket_offset)) - (b.linear + cross(b.angular, b.socket_offset));
    const Mat3 w = socket_inverse_mass(a) + socket_inverse_mass(b);

    Vec3 force;
    if (desc.frequency_hz > 0.0) {
        const real omega = kTwoPi * desc.frequency_hz;
        const real g = h * omega * omega + 2.0 * desc.damping_ratio * omega;
        force = (w.inverse() * (e * (omega * omega) + edot_free * g)) * (-1.0 / (1.0 + h * g));
    } else {
        const real g = h * desc.stiffness_n_m + desc.damping_n_s_m;
        const Mat3 lhs = Mat3::identity() + w * (h * g);
        force = lhs.inverse() * (e * desc.stiffness_n_m + edot_free * g) * real(-1.0);
    }

    SocketSolve out;
    out.force_on_a = force;
    out.error = e;
    out.work_j = h * dot(force, edot_free + w * force * h);
    return out;
}

} // namespace ps::machine
