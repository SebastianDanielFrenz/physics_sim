// PHYS-009 Step 2 rule 3: "keep the old dense implementation as a test-only
// reference, and add a test comparing the fast ABA against it (and against
// CRBA) to 1e-12 relative on random trees."
//
// Two levels: (1) the isolated transform_inertia_to_parent_fast vs.
// _reference functions on synthetic (xup, symmetric Mat6) inputs, and (2) a
// full aba() (fast, production) vs. aba_reference() (dense reference path)
// comparison on the SAME random trees test_articulation_core.cpp's
// ABA-vs-CRBA cross-check already uses, plus that same CRBA solve as a
// third, independent check.

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

// Same helpers as test_articulation_core.cpp (duplicated rather than
// shared across translation units - both are small, self-contained, and
// each file's independence matters more here than avoiding the repeat).
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
    std::uniform_int_distribution<int> joint_pick(0, 1);

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

ArticulationState make_state(const Model& model, std::mt19937& rng) {
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
    return st;
}

// 1e-12 RELATIVE (PLAN.md/PHYS-009 Step 2 rule 3's own wording), but two
// algebraically-equivalent, differently-ORDERED floating-point paths
// (dense 6x6 multiply vs. block Mat3 congruence) can disagree in their
// low-order bits well past 1e-12 *relative* precisely where an entry is
// itself the result of near-cancellation of larger terms (observed: an
// entry around -8.3e-5 that is a difference of terms of order 1, so its
// OWN absolute error is still ~1e-13, tiny in absolute terms, but that is
// ~1e-9 relative to the ~1e-4 result - relative tolerance alone is the
// wrong tool exactly at these near-zero crossings). A fixed absolute
// margin alongside the relative epsilon (Catch2 Approx combines them with
// OR) keeps the check meaningful everywhere: still 1e-12 relative for
// entries of order 1 (the common case, where the margin term is
// negligible), while not manufacturing a false failure at a near-zero
// entry whose absolute error is genuinely tiny.
void check_close(real a, real b, const char* what) {
    INFO(what << ": fast=" << a << " reference/expected=" << b);
    CHECK(a == Approx(b).epsilon(1e-12).margin(1e-12));
}

} // namespace

TEST_CASE("transform_inertia_to_parent_fast matches the dense reference to 1e-12 relative", "[unit][articulation][aba][fast]") {
    std::mt19937 rng(0xF457);
    std::uniform_real_distribution<real> unit(-1.0, 1.0);
    std::uniform_real_distribution<real> pos(0.2, 2.0);

    for (int trial = 0; trial < 200; ++trial) {
        // A random SYMMETRIC Mat6 "IA-like" operand (every operand this
        // transform ever actually sees, per aba.cpp's pass 2, is a
        // symmetric articulated-body inertia).
        Mat6 child;
        for (int r = 0; r < 6; ++r) {
            for (int c = r; c < 6; ++c) {
                const real v = unit(rng) * pos(rng);
                child.m[r][c] = v;
                child.m[c][r] = v;
            }
        }

        const Vec3 axis = Vec3{unit(rng), unit(rng), unit(rng)}.normalized();
        const Quat q = Quat::from_axis_angle(axis, unit(rng) * 3.0);
        const Vec3 r{unit(rng), unit(rng), unit(rng)};
        const spatial::SpatialTransform xup{q.to_mat3(), r};

        const Mat6 fast = transform_inertia_to_parent_fast(xup, child);
        const Mat6 reference = transform_inertia_to_parent_reference(xup, child);

        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                check_close(fast.m[i][j], reference.m[i][j], "Mat6 entry");
            }
        }
    }
}

TEST_CASE("Full aba() (fast) matches aba_reference() (dense) to 1e-12 relative, and both match CRBA", "[unit][articulation][aba][crba][fast]") {
    for (const bool floating : {false, true}) {
        std::mt19937 rng(floating ? 0xABCD1u : 0xABCD2u);
        for (int num_links : {1, 3, 6, 10}) {
            for (int sample = 0; sample < 5; ++sample) {
                const ArticulationDesc desc = make_random_tree(rng, num_links, floating);
                const Model model = build_model(desc);
                ArticulationState st = make_state(model, rng);

                std::uniform_real_distribution<real> unit(-1.0, 1.0);
                std::vector<real> tau(static_cast<std::size_t>(model.num_v));
                for (auto& t : tau) t = unit(rng) * 2.0;
                const Vec3 gravity{0, 0, -9.81};

                AbaInput ain;
                ain.model = &model;
                ain.state = &st;
                ain.tau = tau;
                ain.gravity = gravity;

                const AbaOutput fast_out = aba(ain);
                const AbaOutput ref_out = aba_reference(ain);

                REQUIRE(fast_out.qdd.size() == ref_out.qdd.size());
                for (std::size_t i = 0; i < fast_out.qdd.size(); ++i) {
                    check_close(fast_out.qdd[i], ref_out.qdd[i], "qdd (fast vs reference)");
                }

                // Independent third check: CRBA + RNEA solve M^-1(tau - C),
                // same as test_articulation_core.cpp's own cross-check -
                // both fast and reference ABA outputs must agree with it.
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
                const std::vector<real> qdd_crba = solve_linear(m_flat, rhs, model.num_v);

                for (int i = 0; i < model.num_v; ++i) {
                    INFO("num_links=" << num_links << " floating=" << floating << " dof=" << i);
                    CHECK(fast_out.qdd[static_cast<std::size_t>(i)] == Approx(qdd_crba[static_cast<std::size_t>(i)]).epsilon(1e-9).margin(1e-9));
                }
            }
        }
    }
}
