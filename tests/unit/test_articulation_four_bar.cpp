// White-box test for core/src/articulation/loop_closure.h: a Grashof
// crank-rocker four-bar linkage (PLAN.md P1.5a acceptance).
//
// Ground link A-B (length d), crank A-P1 (length a, driven), coupler
// P1-P2 (length b), rocker B-P2 (length c). Modelled as an open chain
// (crank, then coupler as its child) plus ONE Point loop closure pinning
// the coupler's far end to a separately-grounded rocker's far end - see
// loop_closure.h's file comment for the constrained-dynamics/projection
// algorithm this exercises.
//
// Grashof crank-rocker check (Wong-style link naming, standard
// four-bar theory): shortest + longest <= sum of the other two, AND the
// shortest link is the crank (adjacent to ground) - a=2 (crank, shortest),
// b=5 (coupler, longest), c=4 (rocker), d=4 (ground): 2+5=7 <= 4+4=8. Ok.
//
// Closed-form position check: with the sim's own rotate() convention
// (Quat::from_axis_angle(unit_y, theta) applied to the rod direction
// (0,0,-1) works out, by direct expansion of quat.h's rotate(), to
// (-sin(theta), 0, -cos(theta)) - re-derived in this comment, not assumed)
// crank tip P1(theta_crank) = a*(-sin(theta_crank), 0, -cos(theta_crank)),
// rocker tip P2(theta_rocker) = B + c*(-sin(theta_rocker), 0, -cos(theta_rocker)).
// Given theta_crank, P1 is known; P2 is the standard two-circle
// intersection of radius b about P1 and radius c about B, and
// theta_rocker = atan2(-(P2-B).x, -(P2-B).z) inverts the direction formula
// above.

#include "articulation/loop_closure.h"
#include "articulation/model.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using Catch::Approx;
using namespace ps;
using namespace ps::articulation;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kA = 2.0; // crank
constexpr double kB = 5.0; // coupler
constexpr double kC = 4.0; // rocker
constexpr double kD = 4.0; // ground

ArticulationDesc make_desc(real crank_mass) {
    ArticulationDesc desc;
    desc.root.floating = false;
    desc.root.pose = Pose::identity();

    LinkDesc crank;
    crank.parent = -1;
    crank.joint = JointType::Revolute;
    crank.axis = Vec3::unit_y();
    crank.mass = crank_mass;
    crank.com = Vec3{0, 0, -kA / 2.0};
    crank.inertia = Mat3::diagonal(crank_mass * kA * kA / 12.0, crank_mass * kA * kA / 12.0, 1e-4);
    desc.links.push_back(crank);

    LinkDesc coupler;
    coupler.parent = 0;
    coupler.joint = JointType::Revolute;
    coupler.joint_frame_in_parent.position = Vec3{0, 0, -kA};
    coupler.axis = Vec3::unit_y();
    coupler.mass = 1.0;
    coupler.com = Vec3{0, 0, -kB / 2.0};
    coupler.inertia = Mat3::diagonal(1.0 * kB * kB / 12.0, 1.0 * kB * kB / 12.0, 1e-4);
    desc.links.push_back(coupler);

    LinkDesc rocker;
    rocker.parent = -1;
    rocker.joint = JointType::Revolute;
    rocker.joint_frame_in_parent.position = Vec3{kD, 0, 0};
    rocker.axis = Vec3::unit_y();
    rocker.mass = 1.0;
    rocker.com = Vec3{0, 0, -kC / 2.0};
    rocker.inertia = Mat3::diagonal(1.0 * kC * kC / 12.0, 1.0 * kC * kC / 12.0, 1e-4);
    desc.links.push_back(rocker);

    LoopClosureDesc lc;
    lc.type = LoopClosureType::Point;
    lc.link_a = 1; // coupler
    lc.anchor_a = Vec3{0, 0, -kB};
    lc.link_b = 2; // rocker
    lc.anchor_b = Vec3{0, 0, -kC};
    desc.loop_closures.push_back(lc);

    return desc;
}

Vec3 rod_direction(double theta) { return Vec3{-std::sin(theta), 0.0, -std::cos(theta)}; }

// Closed-form rocker angle for a given crank angle (two-circle
// intersection); branch is selected by proximity to a supplied guess
// point, so repeated calls trace a continuous solution.
double closed_form_rocker_angle(double theta_crank, Vec3& branch_hint /* previous P2, updated in place */) {
    const Vec3 A{0, 0, 0};
    const Vec3 B{kD, 0, 0};
    const Vec3 P1 = A + kA * rod_direction(theta_crank);

    const Vec3 d_vec = B - P1;
    const double d = d_vec.length();
    const double a_dist = (kB * kB - kC * kC + d * d) / (2.0 * d);
    const double h2 = kB * kB - a_dist * a_dist;
    const double h = h2 > 0.0 ? std::sqrt(h2) : 0.0;
    const Vec3 mid = P1 + d_vec * (a_dist / d);
    const Vec3 perp = Vec3{-d_vec.z, 0, d_vec.x} / d;

    const Vec3 cand1 = mid + perp * h;
    const Vec3 cand2 = mid - perp * h;
    const Vec3 p2 = (distance(cand1, branch_hint) < distance(cand2, branch_hint)) ? cand1 : cand2;
    branch_hint = p2;

    const Vec3 rel = (p2 - B) / kC;
    return std::atan2(-rel.x, -rel.z);
}

} // namespace

TEST_CASE("Four-bar linkage: loop-closure solve matches closed-form kinematics", "[unit][articulation][four_bar]") {
    // A crank mass ~1e8x the coupler/rocker makes the mass-weighted
    // projection (loop_closure.h's project_onto_constraints) leave the
    // crank angle untouched to well under 1e-9 rad while it converges the
    // coupler/rocker angles onto the constraint manifold - i.e. this
    // exercises the SAME production position-projection code the dynamic
    // stepping path uses, with the crank acting as a prescribed input.
    const Model model = build_model(make_desc(1.0e8));
    std::vector<int> link_to_body = {0, 1, 2};
    const std::vector<LoopClosureRuntime> closures = build_loop_closure_runtime(model, link_to_body);

    ArticulationState st;
    st.q.assign(static_cast<std::size_t>(model.num_q), real(0.0));
    st.qd.assign(static_cast<std::size_t>(model.num_v), real(0.0));

    // Two-circle intersection has two solutions (the two four-bar assembly
    // modes); seed the closed-form branch tracker from the SIMULATION's
    // own converged assembly mode at the first sample (whichever branch
    // Newton's method reaches from the q=0 initial guess) rather than an
    // arbitrary point, so the comparison tracks the same branch throughout
    // - both are equally valid solutions of the closure equations (as the
    // closure-error check below confirms independent of which one this
    // is), only one of them is what THIS simulation run exhibits.
    Vec3 branch_hint{kD - kC, 0, 0};
    double max_closure_error_mm = 0.0;

    for (int sample = 0; sample <= 24; ++sample) {
        const double theta_crank = -kPi + (2.0 * kPi) * sample / 24.0;
        st.q[0] = theta_crank; // crank is st.q[0] (only 1-dof joint declared before it)

        for (int iter = 0; iter < 40; ++iter) project_onto_constraints(model, st, closures);

        if (sample == 0) {
            const std::vector<Pose> poses = forward_kinematics_poses(model, st);
            branch_hint = poses[1].transform_point(Vec3{0, 0, -kB}); // coupler's far end, i.e. the simulation's own P2
        }

        const std::vector<real> g = constraint_error(model, st, closures);
        const double closure_mm = Vec3{g[0], g[1], g[2]}.length() * 1000.0;
        max_closure_error_mm = std::max(max_closure_error_mm, closure_mm);

        const double theta_rocker_sim = st.q[2]; // rocker's own dof index (q_index 2: crank=0, coupler=1, rocker=2)
        const double theta_rocker_closed = closed_form_rocker_angle(theta_crank, branch_hint);

        double diff = std::fmod(theta_rocker_sim - theta_rocker_closed + kPi, 2.0 * kPi);
        if (diff < 0) diff += 2.0 * kPi;
        diff -= kPi;

        INFO("sample=" << sample << " theta_crank=" << theta_crank << " sim=" << theta_rocker_sim
                        << " closed_form=" << theta_rocker_closed << " diff=" << diff << " closure_mm=" << closure_mm);
        CHECK(std::abs(diff) < 1e-6);
    }

    INFO("max_closure_error_mm=" << max_closure_error_mm);
    CHECK(max_closure_error_mm < 0.1);
}
