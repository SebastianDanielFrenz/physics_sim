// White-box tests for core/src/articulation (internal spatial algebra +
// RNEA/CRBA/ABA), included via a private include path (see
// tests/unit/CMakeLists.txt) since this is not public ps_core API.
//
// PLAN.md P1.5a acceptance: "ABA accelerations equal the CRBA solve
// M^-1(tau-C) to 1e-9 relative, on random trees (seeded, deterministic)
// with mixed revolute/prismatic joints and a floating base."

#include "articulation/aba.h"
#include "articulation/crba.h"
#include "articulation/model.h"
#include "articulation/rnea.h"
#include "articulation/small_matrix.h"
#include "articulation/spatial.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>
#include <vector>

using Catch::Approx;
using namespace ps;
using namespace ps::articulation;

namespace {

// Plain Gauss elimination with partial pivoting, N up to a few dozen -
// test-only helper, deliberately independent of SmallMatrix (kMaxJointDof
// == 6) since a random tree's total DoF count exceeds that.
std::vector<real> solve_linear(std::vector<real> a, std::vector<real> b, int n) {
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        real best = std::abs(a[static_cast<std::size_t>(col * n + col)]);
        for (int r = col + 1; r < n; ++r) {
            const real v = std::abs(a[static_cast<std::size_t>(r * n + col)]);
            if (v > best) { best = v; pivot = r; }
        }
        if (pivot != col) {
            for (int c = 0; c < n; ++c) std::swap(a[static_cast<std::size_t>(col * n + c)], a[static_cast<std::size_t>(pivot * n + c)]);
            std::swap(b[static_cast<std::size_t>(col)], b[static_cast<std::size_t>(pivot)]);
        }
        const real p = a[static_cast<std::size_t>(col * n + col)];
        for (int r = col + 1; r < n; ++r) {
            const real f = a[static_cast<std::size_t>(r * n + col)] / p;
            for (int c = col; c < n; ++c) a[static_cast<std::size_t>(r * n + c)] -= f * a[static_cast<std::size_t>(col * n + c)];
            b[static_cast<std::size_t>(r)] -= f * b[static_cast<std::size_t>(col)];
        }
    }
    std::vector<real> x(static_cast<std::size_t>(n), real(0.0));
    for (int r = n - 1; r >= 0; --r) {
        real sum = b[static_cast<std::size_t>(r)];
        for (int c = r + 1; c < n; ++c) sum -= a[static_cast<std::size_t>(r * n + c)] * x[static_cast<std::size_t>(c)];
        x[static_cast<std::size_t>(r)] = sum / a[static_cast<std::size_t>(r * n + r)];
    }
    return x;
}

ArticulationDesc make_random_tree(std::mt19937& rng, int num_links, bool floating) {
    std::uniform_real_distribution<real> unit(-1.0, 1.0);
    std::uniform_real_distribution<real> pos01(0.2, 1.5);
    std::uniform_int_distribution<int> joint_pick(0, 1); // Revolute or Prismatic only (Fixed covered separately)

    ArticulationDesc desc;
    desc.root.floating = floating;
    desc.root.pose = Pose::identity();
    desc.root.mass = pos01(rng) * 5.0;
    desc.root.com = Vec3{unit(rng), unit(rng), unit(rng)} * 0.1;
    desc.root.inertia = Mat3::diagonal(pos01(rng), pos01(rng), pos01(rng));

    for (int i = 0; i < num_links; ++i) {
        LinkDesc ld;
        ld.parent = (i == 0) ? -1 : std::uniform_int_distribution<int>(0, i - 1)(rng);
        ld.joint = (joint_pick(rng) == 0) ? JointType::Revolute : JointType::Prismatic;
        ld.axis = Vec3{unit(rng), unit(rng), unit(rng)}.normalized();
        ld.joint_frame_in_parent.position = Vec3{unit(rng), unit(rng), unit(rng)} * 0.3;
        ld.mass = pos01(rng) * 2.0;
        ld.com = Vec3{unit(rng), unit(rng), unit(rng)} * 0.05;
        ld.inertia = Mat3::diagonal(pos01(rng) * 0.1, pos01(rng) * 0.1, pos01(rng) * 0.1);
        ld.q0 = unit(rng) * 0.3;
        ld.qd0 = unit(rng) * 0.3;
        desc.links.push_back(ld);
    }
    return desc;
}

ArticulationState make_state(const Model& model, const ArticulationDesc& desc, std::mt19937& rng) {
    std::uniform_real_distribution<real> unit(-1.0, 1.0);
    ArticulationState st;
    st.q.assign(static_cast<std::size_t>(model.num_q), real(0.0));
    st.qd.assign(static_cast<std::size_t>(model.num_v), real(0.0));
    for (auto& v : st.q) v = unit(rng) * 0.4;
    for (auto& v : st.qd) v = unit(rng) * 0.4;
    if (model.floating_root) {
        st.root_pose.position = Vec3{unit(rng), unit(rng), unit(rng)} * 0.2;
        st.root_pose.orientation = Quat::from_axis_angle(Vec3{unit(rng), unit(rng), unit(rng)}.normalized(), unit(rng) * 0.5);
    }
    (void)desc;
    return st;
}

} // namespace

TEST_CASE("Spatial transform preserves power (motion.force invariant)", "[unit][articulation][spatial]") {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<real> unit(-1.0, 1.0);
    for (int trial = 0; trial < 50; ++trial) {
        const Vec3 axis = Vec3{unit(rng), unit(rng), unit(rng)}.normalized();
        const Quat q = Quat::from_axis_angle(axis, unit(rng) * 3.0);
        const Vec3 r{unit(rng), unit(rng), unit(rng)};
        const spatial::SpatialTransform X{q.to_mat3().transposed(), r};

        const spatial::SpatialVector m{{unit(rng), unit(rng), unit(rng)}, {unit(rng), unit(rng), unit(rng)}};
        const spatial::SpatialVector f{{unit(rng), unit(rng), unit(rng)}, {unit(rng), unit(rng), unit(rng)}};

        const real before = spatial::spatial_dot(m, f);
        const real after = spatial::spatial_dot(X.apply_motion(m), X.apply_force(f));
        CHECK(after == Approx(before).margin(1e-10));

        // Round trip through inverse() recovers the original vectors.
        const spatial::SpatialVector m_back = X.inverse().apply_motion(X.apply_motion(m));
        CHECK(m_back.top.x == Approx(m.top.x).margin(1e-10));
        CHECK(m_back.bottom.z == Approx(m.bottom.z).margin(1e-10));
    }
}

TEST_CASE("Spatial inertia transform stays dynamically consistent", "[unit][articulation][spatial]") {
    std::mt19937 rng(777);
    std::uniform_real_distribution<real> unit(-1.0, 1.0);
    std::uniform_real_distribution<real> pos(0.3, 2.0);
    for (int trial = 0; trial < 20; ++trial) {
        const real mass = pos(rng);
        const Vec3 com{unit(rng) * 0.3, unit(rng) * 0.3, unit(rng) * 0.3};
        const Mat3 ic = Mat3::diagonal(pos(rng), pos(rng), pos(rng));
        const spatial::SpatialInertia ia = spatial::SpatialInertia::from_com(mass, com, ic);

        const Vec3 axis = Vec3{unit(rng), unit(rng), unit(rng)}.normalized();
        const Quat q = Quat::from_axis_angle(axis, unit(rng) * 2.0);
        const Vec3 r{unit(rng), unit(rng), unit(rng)};
        const spatial::SpatialTransform X{q.to_mat3().transposed(), r};

        const spatial::SpatialInertia ib = X.apply_inertia(ia);
        const spatial::SpatialVector v{{unit(rng), unit(rng), unit(rng)}, {unit(rng), unit(rng), unit(rng)}};

        // I_B * X.apply_motion(v) must equal X.apply_force(I_A * v).
        const spatial::SpatialVector lhs = ib * X.apply_motion(v);
        const spatial::SpatialVector rhs = X.apply_force(ia * v);
        CHECK(lhs.top.x == Approx(rhs.top.x).margin(1e-9));
        CHECK(lhs.top.y == Approx(rhs.top.y).margin(1e-9));
        CHECK(lhs.top.z == Approx(rhs.top.z).margin(1e-9));
        CHECK(lhs.bottom.x == Approx(rhs.bottom.x).margin(1e-9));
        CHECK(lhs.bottom.y == Approx(rhs.bottom.y).margin(1e-9));
        CHECK(lhs.bottom.z == Approx(rhs.bottom.z).margin(1e-9));
    }
}

TEST_CASE("ABA matches the CRBA solve M^-1(tau - C) to 1e-9 relative", "[unit][articulation][aba][crba]") {
    // Seeded, deterministic random trees (PLAN.md P1.5a): fixed and
    // floating roots, mixed revolute/prismatic joints.
    for (const bool floating : {false, true}) {
        std::mt19937 rng(floating ? 2026u : 4242u);
        for (int num_links : {1, 3, 6}) {
            for (int sample = 0; sample < 5; ++sample) {
                const ArticulationDesc desc = make_random_tree(rng, num_links, floating);
                const Model model = build_model(desc);
                ArticulationState st = make_state(model, desc, rng);

                std::uniform_real_distribution<real> unit(-1.0, 1.0);
                std::vector<real> tau(static_cast<std::size_t>(model.num_v));
                for (auto& t : tau) t = unit(rng) * 2.0;
                const Vec3 gravity{0, 0, -9.81};

                AbaInput ain;
                ain.model = &model;
                ain.state = &st;
                ain.tau = tau;
                ain.gravity = gravity;
                const AbaOutput aout = aba(ain);

                const MassMatrix M = crba(model, st);

                RneaInput cin;
                cin.model = &model;
                cin.state = &st;
                cin.qdd.assign(static_cast<std::size_t>(model.num_v), real(0.0));
                cin.gravity = gravity;
                const RneaOutput cout_ = rnea(cin);

                std::vector<real> rhs(static_cast<std::size_t>(model.num_v));
                for (int i = 0; i < model.num_v; ++i) rhs[static_cast<std::size_t>(i)] = tau[static_cast<std::size_t>(i)] - cout_.tau[static_cast<std::size_t>(i)];

                std::vector<real> m_flat(static_cast<std::size_t>(model.num_v) * static_cast<std::size_t>(model.num_v));
                for (int r = 0; r < model.num_v; ++r)
                    for (int c = 0; c < model.num_v; ++c) m_flat[static_cast<std::size_t>(r * model.num_v + c)] = M.at(r, c);

                const std::vector<real> qdd_check = solve_linear(m_flat, rhs, model.num_v);

                for (int i = 0; i < model.num_v; ++i) {
                    const real a = aout.qdd[static_cast<std::size_t>(i)];
                    const real b = qdd_check[static_cast<std::size_t>(i)];
                    const real scale = std::max(real(1.0), std::abs(b));
                    INFO("floating=" << floating << " num_links=" << num_links << " sample=" << sample << " dof=" << i
                                      << " aba=" << a << " crba_solve=" << b);
                    CHECK(std::abs(a - b) < 1e-9 * scale);
                }
            }
        }
    }
}
