#include "frame_convert_core.h"

#include <cmath>

namespace ps_godot {

Vec3f iso_to_godot_dir(const ps::Vec3& sim_dir) {
    // godot.x = -sim.y ; godot.y = sim.z ; godot.z = -sim.x
    // (derivation: sim right = -sim.y since sim "left" is +y; Godot right
    // is +x, so godot.x = sim_right = -sim.y. Godot up (+y) = sim up
    // (sim.z). Sim forward (+x) is Godot's -z, so godot.z = -sim.x.
    // det of this map is +1 - see frame_convert_core.h - so it is a
    // rotation, never a mirror.)
    return Vec3f{static_cast<float>(-sim_dir.y), static_cast<float>(sim_dir.z), static_cast<float>(-sim_dir.x)};
}

ps::Vec3 godot_to_iso_dir(const Vec3f& godot_dir) {
    // Inverse of the above: sim.x = -godot.z ; sim.y = -godot.x ; sim.z = godot.y
    return ps::Vec3{static_cast<ps::real>(-godot_dir.z), static_cast<ps::real>(-godot_dir.x), static_cast<ps::real>(godot_dir.y)};
}

Vec3f iso_to_godot_position(const ps::Vec3& sim_pos_absolute, const ps::Vec3& origin_m) {
    // Subtract in ps::real (double), in sim axes, BEFORE the basis change
    // and BEFORE the float cast - this is the entire origin-rebasing
    // mechanism (PLAN.md P2). iso_to_godot_dir's float cast then only ever
    // sees a camera-relative magnitude, never the absolute one.
    const ps::Vec3 relative = sim_pos_absolute - origin_m;
    return iso_to_godot_dir(relative);
}

ps::Vec3 godot_to_iso_position(const Vec3f& godot_pos, const ps::Vec3& origin_m) {
    return godot_to_iso_dir(godot_pos) + origin_m;
}

Basis3f iso_to_godot_basis(const ps::Quat& sim_rot) {
    // Rotate the sim's own local unit axes by sim_rot to get their world
    // directions in SIM axes, then map each direction into Godot axes.
    // Composing two proper rotations (sim_rot, then the fixed basis change)
    // always yields a valid orthonormal right-handed basis.
    const ps::Vec3 local_x_world_sim = sim_rot.rotate(ps::Vec3::unit_x());
    const ps::Vec3 local_y_world_sim = sim_rot.rotate(ps::Vec3::unit_y());
    const ps::Vec3 local_z_world_sim = sim_rot.rotate(ps::Vec3::unit_z());
    return Basis3f{
        iso_to_godot_dir(local_x_world_sim),
        iso_to_godot_dir(local_y_world_sim),
        iso_to_godot_dir(local_z_world_sim),
    };
}

ps::Quat godot_to_iso_rotation(const Basis3f& godot_basis) {
    // Map each Godot-basis column back into sim axes; the three results are
    // the sim-frame rotation matrix's columns (local x/y/z axes in sim
    // world space) - i.e. exactly q_sim's rotation matrix. Convert that
    // 3x3 (given as columns) to a quaternion via the standard
    // trace/largest-diagonal method (Shepperd 1978) - not a core
    // algorithm, so implemented locally here rather than touching
    // ps::Quat/ps::Mat3 (adapter's own lane only).
    const ps::Vec3 cx = godot_to_iso_dir(godot_basis.col_x);
    const ps::Vec3 cy = godot_to_iso_dir(godot_basis.col_y);
    const ps::Vec3 cz = godot_to_iso_dir(godot_basis.col_z);

    const ps::real m00 = cx.x, m10 = cx.y, m20 = cx.z;
    const ps::real m01 = cy.x, m11 = cy.y, m21 = cy.z;
    const ps::real m02 = cz.x, m12 = cz.y, m22 = cz.z;

    const ps::real trace = m00 + m11 + m22;
    ps::real x, y, z, w;
    if (trace > 0.0) {
        const ps::real s = 0.5 / std::sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (m21 - m12) * s;
        y = (m02 - m20) * s;
        z = (m10 - m01) * s;
    } else if (m00 > m11 && m00 > m22) {
        const ps::real s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const ps::real s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    } else {
        const ps::real s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }
    return ps::Quat{x, y, z, w}.normalized();
}

} // namespace ps_godot
