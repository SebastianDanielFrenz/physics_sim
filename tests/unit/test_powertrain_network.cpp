// tests/unit/test_powertrain_network.cpp — white-box tests for
// ps::drivetrain (PHYS-033 P4 I2 plan section 11.2). core/src is on this
// target's include path (tests/unit/CMakeLists.txt), like the other
// white-box tests (test_articulation_core.cpp etc.) - ps::drivetrain is
// internal, not part of ps_core's public API.
//
// Commit 1: case (g), Cholesky against a reference solve. Commit 3 (this
// landing) adds cases (a)-(d) and (f): analytic two-body lock, gear-chain
// power with mesh loss, open differential, and a test-only two-topology-
// state engagement. Case (h) (relaxation) lands with commit 4.
//
// Cases (a)-(d) verify energy/momentum identities INLINE, directly off
// ShaftNetwork's own public state (omega/row_lambda/row_velocity) - the
// full ps::drivetrain::EnergyLedger type does not exist until commit 6;
// until then, the underlying algebraic identity (plan section 8) is
// exactly as checkable this way, since it is a property of the linearly-
// implicit solve itself, not of any bookkeeping wrapper around it.

#include "drivetrain/dense.h"
#include "drivetrain/network.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>

using namespace ps;
using namespace ps::drivetrain;

namespace {
real sign_of(real x) { return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0); }
} // namespace

namespace {

// Reference solve for a small SPD system via plain Gauss-Jordan with
// partial pivoting (an INDEPENDENT method from cholesky_factor/solve under
// test - not itself part of the production code path) - n <= 8, so a
// simple O(n^3) elimination is more than accurate enough as a cross-check.
template <std::size_t N>
SmallVec<N> reference_solve(const SmallMat<N, N>& A, std::size_t n, const SmallVec<N>& b) {
    // Augmented matrix [A | b] as plain arrays (small, stack only).
    std::array<std::array<real, N + 1>, N> aug{};
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) aug[i][j] = A(i, j);
        aug[i][n] = b[i];
    }
    for (std::size_t col = 0; col < n; ++col) {
        std::size_t pivot = col;
        real best = std::abs(aug[col][col]);
        for (std::size_t r = col + 1; r < n; ++r) {
            if (std::abs(aug[r][col]) > best) { best = std::abs(aug[r][col]); pivot = r; }
        }
        std::swap(aug[col], aug[pivot]);
        const real d = aug[col][col];
        for (std::size_t j = col; j <= n; ++j) aug[col][j] /= d;
        for (std::size_t r = 0; r < n; ++r) {
            if (r == col) continue;
            const real f = aug[r][col];
            if (f == real(0.0)) continue;
            for (std::size_t j = col; j <= n; ++j) aug[r][j] -= f * aug[col][j];
        }
    }
    SmallVec<N> x;
    x.set_size(n);
    for (std::size_t i = 0; i < n; ++i) x[i] = aug[i][n];
    return x;
}

} // namespace

TEST_CASE("cholesky_factor/cholesky_solve match a reference Gauss-Jordan solve", "[unit][drivetrain][dense]") {
    ps_test::SplitMix64 rng(0xC0FFEE1234ULL);
    constexpr std::size_t kMaxN = 8;

    for (int trial = 0; trial < 2000; ++trial) {
        const std::size_t n = 1 + static_cast<std::size_t>(rng.next_u64() % kMaxN); // 1..8

        // Random SPD: A = M*M^T + 0.5*I (plan section 11.1 batch 2's own
        // construction), M entries in [-1, 1].
        std::array<std::array<real, kMaxN>, kMaxN> M{};
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j) M[i][j] = rng.next_uniform(-1.0, 1.0);

        SmallMat<kMaxN, kMaxN> A;
        A.set_size(n, n);
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                real s = 0.0;
                for (std::size_t k = 0; k < n; ++k) s += M[i][k] * M[j][k];
                A(i, j) = s + (i == j ? real(0.5) : real(0.0));
            }
        }

        SmallVec<kMaxN> b;
        b.set_size(n);
        for (std::size_t i = 0; i < n; ++i) b[i] = rng.next_uniform(-10.0, 10.0);

        SmallMat<kMaxN, kMaxN> L;
        int nonspd_hits = 0;
        const bool ok = cholesky_factor(A, n, L, &nonspd_hits);
        REQUIRE(ok);
        REQUIRE(nonspd_hits == 0);

        SmallVec<kMaxN> x;
        cholesky_solve(L, n, b, x);

        const SmallVec<kMaxN> x_ref = reference_solve(A, n, b);
        for (std::size_t i = 0; i < n; ++i) {
            const real scale = std::max(real(1.0), std::abs(x_ref[i]));
            INFO("trial=" << trial << " n=" << n << " i=" << i);
            CHECK(std::abs(x[i] - x_ref[i]) <= real(1e-13) * scale);
        }

        // L*L^T reproduces A itself (the factorisation identity), not just
        // "solves the same system" - independent of the solve step above.
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j <= i; ++j) {
                real s = 0.0;
                for (std::size_t k = 0; k <= j; ++k) s += L(i, k) * L(j, k);
                CHECK(std::abs(s - A(i, j)) <= real(1e-12) * std::max(real(1.0), std::abs(A(i, j))));
            }
        }
    }
}

TEST_CASE("cholesky_factor regularises a merely-PSD matrix instead of producing a NaN",
          "[unit][drivetrain][dense]") {
    // A rank-deficient PSD matrix (one row duplicated into another, plan
    // section 11.1 batch 3's "degenerate" construction): [[1,1],[1,1]] is
    // PSD (eigenvalues 0 and 2) but not PD - cholesky_factor must
    // regularise rather than take sqrt of a non-positive diagonal.
    SmallMat<8, 8> A;
    A.set_size(2, 2);
    A(0, 0) = 1.0; A(0, 1) = 1.0;
    A(1, 0) = 1.0; A(1, 1) = 1.0;

    SmallMat<8, 8> L;
    int nonspd_hits = 0;
    const bool ok = cholesky_factor(A, 2, L, &nonspd_hits);
    CHECK_FALSE(ok);
    CHECK(nonspd_hits >= 1);
    CHECK(std::isfinite(L(1, 1)));
    CHECK(L(1, 1) > 0.0);
}

// --- Case (f): null-space basis matches the plan's worked examples exactly ---
TEST_CASE("build_nullspace matches the plan's car/tractor worked examples exactly", "[unit][drivetrain][network]") {
    // Car: N0={engine.crank,final_drive.in} (J=0), N1={final_drive.out,diff.carrier} (J=0),
    // N2=wheel:FL (J=0.905), N3=wheel:FR (J=0.905). Gear row [N0:1,N1:-10],
    // diff row [N1:1,N2:-0.5,N3:-0.5]. Expected T = [[5,5],[0.5,0.5],[1,0],[0,1]]
    // (plan section 4.3's own worked example).
    auto run = [](real ratio, real expected_n0_col0) {
        std::vector<ConstraintRow> rows(2);
        rows[0].terms = {{0, real(1.0)}, {1, -ratio}};
        rows[1].terms = {{1, real(1.0)}, {2, real(-0.5)}, {3, real(-0.5)}};
        std::vector<real> inertia = {0.0, 0.0, 0.905, 0.905};
        NullspaceBasis basis;
        const int n = build_nullspace(rows, inertia, 4, basis);
        REQUIRE(n == 2);
        REQUIRE(basis.free_nodes.size() == 2u);
        CHECK(basis.free_nodes[0] == 2);
        CHECK(basis.free_nodes[1] == 3);
        CHECK(basis.at(0, 0) == Catch::Approx(expected_n0_col0));
        CHECK(basis.at(0, 1) == Catch::Approx(expected_n0_col0));
        CHECK(basis.at(1, 0) == Catch::Approx(0.5));
        CHECK(basis.at(1, 1) == Catch::Approx(0.5));
        CHECK(basis.at(2, 0) == Catch::Approx(1.0));
        CHECK(basis.at(2, 1) == Catch::Approx(0.0));
        CHECK(basis.at(3, 0) == Catch::Approx(0.0));
        CHECK(basis.at(3, 1) == Catch::Approx(1.0));

        // G*T == 0 exactly (within float noise).
        for (const ConstraintRow& row : rows) {
            for (int col = 0; col < n; ++col) {
                real s = 0.0;
                for (const auto& [node, coef] : row.terms) s += coef * basis.at(node, col);
                CHECK(std::abs(s) < 1e-12);
            }
        }

        // Determinism: a second independent build is bit-identical.
        NullspaceBasis basis2;
        build_nullspace(rows, inertia, 4, basis2);
        CHECK(basis.T == basis2.T);
    };
    run(10.0, 5.0); // car_sedan final_drive_ratio = 10.0 (plan's own example number)
    run(15.0, 7.5); // tractor final_drive_ratio = 15.0
}

// --- Case (a): two-inertia lock via a friction row ---
TEST_CASE("network: two inertias lock through a friction row (momentum-conserving)", "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId n1 = nb.add_node(0.2);
    const NodeId n2 = nb.add_node(1.5);
    const FrictionRowId row = nb.add_friction_row(n1, n2);
    ShaftNetwork net = nb.build("test-a");

    const real J1 = 0.2, J2 = 1.5, cap = 150.0;
    const real w1_0 = 300.0, w2_0 = 0.0;
    net.set_omega(n1, w1_0);
    net.set_omega(n2, w2_0);

    const real h = 1.0 / 960.0;
    const real momentum0 = J1 * w1_0 + J2 * w2_0;

    Assembly assembly;
    int lock_substep = -1;
    for (int s = 0; s < 2000; ++s) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        assembly.set_friction_capacity(row, cap, cap);

        const real w1_before = net.omega(n1), w2_before = net.omega(n2);
        const real ke_before = 0.5 * (J1 * w1_before * w1_before + J2 * w2_before * w2_before);

        net.solve(h, assembly);

        const real w1_after = net.omega(n1), w2_after = net.omega(n2);
        const real ke_after = 0.5 * (J1 * w1_after * w1_after + J2 * w2_after * w2_after);
        const real momentum_after = J1 * w1_after + J2 * w2_after;
        CHECK(std::abs(momentum_after - momentum0) <= 1e-12 * std::abs(momentum0));

        // Ledger identity (plan section 8), tau0=d=0 everywhere here:
        // dKE = h*v'*lambda - integrator_loss.
        const real v_after = net.row_velocity(row);
        const real lambda = net.row_lambda(row);
        const real integrator_loss = 0.5 * (J1 * (w1_after - w1_before) * (w1_after - w1_before) +
                                             J2 * (w2_after - w2_before) * (w2_after - w2_before));
        const real predicted = h * v_after * lambda - integrator_loss;
        CHECK(std::abs((ke_after - ke_before) - predicted) <= 1e-9 * std::max(real(1.0), std::abs(ke_before)));

        if (lock_substep < 0 && net.row_stuck(row)) lock_substep = s;
    }

    REQUIRE(lock_substep >= 0);
    const real t_star = (w1_0 - w2_0) * J1 * J2 / (cap * (J1 + J2));
    const real expected_substep = t_star / h;
    CHECK(std::abs(static_cast<real>(lock_substep) - expected_substep) <= 1.0);

    const real common = momentum0 / (J1 + J2);
    CHECK(net.omega(n1) == Catch::Approx(common).margin(1e-9));
    CHECK(net.omega(n2) == Catch::Approx(common).margin(1e-9));

    // Stays stuck for 1000 more substeps.
    for (int s = 0; s < 1000; ++s) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        assembly.set_friction_capacity(row, cap, cap);
        net.solve(h, assembly);
        REQUIRE(net.row_stuck(row));
        CHECK(std::abs(net.omega(n1) - net.omega(n2)) < 1e-9);
    }
}

// --- Case (b): gear-chain power with mesh loss, test-only (plan section 5.4) ---
TEST_CASE("network: gear-chain power reproduces ratio*efficiency*torque with lagged mesh loss",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId in_node = nb.add_node(0.0);   // zero-inertia "engine" = gear.in, sharing one node
    const NodeId load_node = nb.add_node(2.0); // gear.out == the load inertia directly
    const real ratio = 10.0, eta = 0.9;
    RigidRow gear_row;
    gear_row.terms = {{static_cast<int>(in_node.index), real(1.0)}, {static_cast<int>(load_node.index), -ratio}};
    nb.add_rigid_row(0, gear_row);
    ShaftNetwork net = nb.build("test-b");

    net.set_omega(in_node, ratio * 5.0); // consistent with load starting at 5 rad/s
    net.set_omega(load_node, 5.0);

    const real h = 1.0 / 960.0;
    const real tau_e = 100.0;
    real lambda_mesh_prev = 0.0;
    Assembly assembly;

    for (int s = 0; s < 20; ++s) {
        const real w_in_old = net.omega(in_node);
        const real w_load_old = net.omega(load_node);
        const real loss_term = -(1.0 - eta) * std::abs(lambda_mesh_prev) * sign_of(w_in_old);

        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        assembly.torque(in_node, tau_e + loss_term, 0.0);

        net.solve(h, assembly);

        const real w_in_new = net.omega(in_node);
        const real w_load_new = net.omega(load_node);

        // Reconstruct lambda_mesh (plan section 5.4), EXCLUDING the loss
        // term (J_in == 0, so the first term vanishes identically; no
        // other rows touch this node).
        lambda_mesh_prev = (0.0 * (w_in_new - w_in_old)) / h - tau_e;

        const real gear_loss = -h * w_in_new * loss_term;

        if (s >= 1) { // "from the second substep on"
            const real delta_w_load = w_load_new - w_load_old;
            const real j_dw_dt = 2.0 * delta_w_load / h;
            CHECK(j_dw_dt == Catch::Approx(ratio * eta * tau_e).epsilon(1e-12));

            const real expected_loss = (1.0 - eta) * tau_e * std::abs(w_in_new) * h;
            CHECK(gear_loss == Catch::Approx(expected_loss).epsilon(1e-9));
        }
    }
}

// --- Case (c): open differential ---
TEST_CASE("network: open differential splits torque, holds a locked hub", "[unit][drivetrain][network]") {
    SECTION("identical hubs under identical (tau0,d) get bit-identical delta") {
        NetworkBuilder nb;
        const NodeId carrier = nb.add_node(0.0);
        const NodeId left = nb.add_node(1.0);
        const NodeId right = nb.add_node(1.0);
        RigidRow diff_row;
        diff_row.terms = {{static_cast<int>(carrier.index), real(1.0)}, {static_cast<int>(left.index), real(-0.5)},
                           {static_cast<int>(right.index), real(-0.5)}};
        nb.add_rigid_row(0, diff_row);
        ShaftNetwork net = nb.build("test-c1");

        net.set_omega(left, 10.0);
        net.set_omega(right, 10.0);
        net.set_omega(carrier, 10.0);

        const real h = 1.0 / 960.0;
        Assembly assembly;
        for (int s = 0; s < 50; ++s) {
            assembly.begin_substep(net.node_count());
            assembly.begin_substep_rows(net.friction_row_count());
            assembly.torque(left, -20.0, 5.0); // identical implicit tyre-like (tau0,d)
            assembly.torque(right, -20.0, 5.0);
            net.solve(h, assembly);
            CHECK(net.omega(left) == net.omega(right)); // bit-exact by symmetry
        }
    }

    SECTION("one hub locked by a large-capacity brake: carrier is half the free hub's speed") {
        NetworkBuilder nb;
        const NodeId carrier = nb.add_node(0.0);
        const NodeId left = nb.add_node(1.0);
        const NodeId right = nb.add_node(1.0);
        RigidRow diff_row;
        diff_row.terms = {{static_cast<int>(carrier.index), real(1.0)}, {static_cast<int>(left.index), real(-0.5)},
                           {static_cast<int>(right.index), real(-0.5)}};
        nb.add_rigid_row(0, diff_row);
        const FrictionRowId brake = nb.add_friction_row(left);
        ShaftNetwork net = nb.build("test-c2");

        net.set_omega(left, 0.0);
        net.set_omega(right, 20.0);
        net.set_omega(carrier, 10.0);

        const real h = 1.0 / 960.0;
        Assembly assembly;
        for (int s = 0; s < 500; ++s) {
            assembly.begin_substep(net.node_count());
            assembly.begin_substep_rows(net.friction_row_count());
            assembly.set_friction_capacity(brake, 1e6, 1e6);
            net.solve(h, assembly);
        }
        CHECK(std::abs(net.omega(left)) < 1e-9);
        CHECK(net.omega(carrier) == Catch::Approx(0.5 * net.omega(right)).margin(1e-9));
    }
}

// --- Case (d): topology switch, momentum-conserving engagement (test-only two-state builder) ---
TEST_CASE("network: topology switch conserves momentum and books the exact engagement loss",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId n1 = nb.add_node(0.3);
    const NodeId n2 = nb.add_node(0.7);
    // State 0: free (no rows). State 1: dog engaged, w1 == w2.
    RigidRow dog_row;
    dog_row.terms = {{static_cast<int>(n1.index), real(1.0)}, {static_cast<int>(n2.index), real(-1.0)}};
    nb.add_rigid_row(1, dog_row);
    ShaftNetwork net = nb.build("test-d");

    const real J1 = 0.3, J2 = 0.7;
    const real w1_0 = 40.0, w2_0 = -15.0;
    net.set_omega(n1, w1_0);
    net.set_omega(n2, w2_0);

    const real h = 1.0 / 960.0;
    Assembly assembly;
    for (int s = 0; s < 10; ++s) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        net.solve(h, assembly);
    }
    // Still free: no drift (no torque applied).
    CHECK(net.omega(n1) == Catch::Approx(w1_0));
    CHECK(net.omega(n2) == Catch::Approx(w2_0));

    const real w1_before = net.omega(n1), w2_before = net.omega(n2);
    const real ke_before = 0.5 * (J1 * w1_before * w1_before + J2 * w2_before * w2_before);
    const real momentum_before = J1 * w1_before + J2 * w2_before;

    net.set_topology_state(1);
    assembly.begin_substep(net.node_count());
    assembly.begin_substep_rows(net.friction_row_count());
    net.solve(h, assembly);

    const real nu = momentum_before / (J1 + J2);
    CHECK(net.omega(n1) == Catch::Approx(nu).epsilon(1e-12));
    CHECK(net.omega(n2) == Catch::Approx(nu).epsilon(1e-12));

    const real momentum_after = J1 * net.omega(n1) + J2 * net.omega(n2);
    CHECK(std::abs(momentum_after - momentum_before) <= 1e-12 * std::abs(momentum_before));

    const real ke_after = 0.5 * (J1 + J2) * nu * nu;
    const real integrator_loss = ke_before - ke_after;
    const real expected_loss = 0.5 * (J1 * J2 / (J1 + J2)) * (w1_before - w2_before) * (w1_before - w2_before);
    CHECK(integrator_loss == Catch::Approx(expected_loss).epsilon(1e-12));

    // Stays engaged for further substeps.
    for (int s = 0; s < 20; ++s) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        net.solve(h, assembly);
        CHECK(net.omega(n1) == Catch::Approx(net.omega(n2)).epsilon(1e-12));
    }
}

// --- PHYS-033 I3 plan section 5.5: a friction row's b-side ratio -----------
// A synchro row's relative velocity is omega(a) - ratio*omega(b), not
// omega(a) - omega(b). Derivation of the quantity this test checks: with no
// rigid rows in this 2-node cluster, the basis is identity (n=2), so
// B = [1, -ratio] and Y = A^-1 B^T = [1/J1, -ratio/J2]^T (A = diag(J1,J2),
// d=0 here). So one substep's impulse gives J1*Domega1 = h*lambda and
// J2*Domega2 = -ratio*h*lambda, i.e. J2*Domega2 = -ratio*J1*Domega1: the
// row's own action alone (no external torque) conserves
// M_r := ratio*J1*omega1 + J2*omega2, NOT the plain momentum J1*w1+J2*w2
// (that formula is the special case ratio=1, matching the existing "two
// inertias lock" test above).
TEST_CASE("network: a friction row's ratio generalises its coupling (synchro row shape)", "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId n1 = nb.add_node(0.014); // gearbox input shaft, plan section 10.1's sedan_6mt J1
    const NodeId n2 = nb.add_node(0.905); // a hub-sized inertia standing in for the gearbox output
    const FrictionRowId row = nb.add_friction_row(n1, n2);
    ShaftNetwork net = nb.build("test-ratio");

    const real J1 = 0.014, J2 = 0.905, ratio = 2.05, cap = 500.0; // ratio = sedan_6mt's own 2nd-gear ratio
    const real w1_0 = 0.0, w2_0 = 200.0;                          // engine idle-ish input, wheel-side spinning
    net.set_omega(n1, w1_0);
    net.set_omega(n2, w2_0);
    const real M_r = ratio * J1 * w1_0 + J2 * w2_0;

    const real h = 1.0 / 960.0;
    Assembly assembly;
    bool locked = false;
    for (int s = 0; s < 2000; ++s) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        assembly.set_friction_row_ratio(row, ratio);
        assembly.set_friction_capacity(row, cap, cap);

        net.solve(h, assembly);

        const real m_now = ratio * J1 * net.omega(n1) + J2 * net.omega(n2);
        CHECK(std::abs(m_now - M_r) <= 1e-9 * std::max(real(1.0), std::abs(M_r)));

        // The row's own reported velocity must be the RATIO'd relative
        // velocity (plan section 5.5), not the plain difference.
        const real v_expected = net.omega(n1) - ratio * net.omega(n2);
        CHECK(net.row_velocity(row) == Catch::Approx(v_expected).margin(1e-9));

        if (net.row_stuck(row)) { locked = true; break; }
    }
    REQUIRE(locked);

    // Locked: omega1 == ratio*omega2 (NOT omega1 == omega2 - that would be
    // the ratio=1 default, and catching a regression to it is this test's
    // whole point), and the closed form from M_r above.
    const real w2_expected = M_r / (ratio * ratio * J1 + J2);
    const real w1_expected = ratio * w2_expected;
    CHECK(net.omega(n2) == Catch::Approx(w2_expected).epsilon(1e-9));
    CHECK(net.omega(n1) == Catch::Approx(w1_expected).epsilon(1e-9));
    CHECK(net.omega(n1) == Catch::Approx(ratio * net.omega(n2)).epsilon(1e-9));

    // Stays locked, still ratio'd, for more substeps.
    for (int s = 0; s < 20; ++s) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        assembly.set_friction_row_ratio(row, ratio);
        assembly.set_friction_capacity(row, cap, cap);
        net.solve(h, assembly);
        REQUIRE(net.row_stuck(row));
        CHECK(net.omega(n1) == Catch::Approx(ratio * net.omega(n2)).epsilon(1e-9));
    }

    // A DEFAULT (un-set) row still behaves as ratio == 1.0 exactly (I2
    // compatibility - begin_substep_rows' own default, not this test
    // re-asserting it): build a fresh, otherwise-identical network and never
    // call set_friction_row_ratio.
    NetworkBuilder nb2;
    const NodeId m1 = nb2.add_node(J1);
    const NodeId m2 = nb2.add_node(J2);
    const FrictionRowId row2 = nb2.add_friction_row(m1, m2);
    ShaftNetwork net2 = nb2.build("test-ratio-default");
    net2.set_omega(m1, w1_0);
    net2.set_omega(m2, w2_0);
    Assembly assembly2;
    for (int s = 0; s < 2000 && !net2.row_stuck(row2); ++s) {
        assembly2.begin_substep(net2.node_count());
        assembly2.begin_substep_rows(net2.friction_row_count());
        assembly2.set_friction_capacity(row2, cap, cap);
        net2.solve(h, assembly2);
    }
    REQUIRE(net2.row_stuck(row2));
    CHECK(net2.omega(m1) == Catch::Approx(net2.omega(m2)).epsilon(1e-9)); // ratio 1.0, not 2.05
}

// --- PHYS-033 I3 plan section 13.3(a): clutch lock-up, literal criterion ---
// Engine node (J=0.16) under a constant drive torque, load node (J=30)
// under an implicit viscous load (d=2 N*m*s/rad, torque = -d*omega,
// linearised around the substep's own omega_old exactly as a real
// component's pre_solve would), coupled by a friction row whose KINETIC
// capacity ramps linearly (c(t) = 300*t) and static = 1.15*kinetic - the
// literal setup the plan's coordinator ruling Q-I3-1 asks for at network
// level ("no torque spike above capacity or steady").
TEST_CASE("network: clutch lock-up under a ramping capacity - no spike above capacity or steady",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId engine = nb.add_node(0.16);
    const NodeId load = nb.add_node(30.0);
    // a = load, b = engine (not the other way around): with coefficient +1
    // on the "a" side (network.cpp's B-matrix build), this makes the row's
    // lambda enter the ENGINE's own balance as "J_e*wdot_e = others - lambda"
    // (coefficient -1 on the "b" side) - i.e. lambda is "torque the row
    // REMOVES from the engine", matching the plan's own sign convention for
    // its lambda_ref = tau - J_e*wdot formula (section 13.3) exactly.
    const FrictionRowId row = nb.add_friction_row(load, engine);
    ShaftNetwork net = nb.build("test-lockup");

    const real J_e = 0.16, J_L = 30.0, tau_e = 100.0, d_load = 2.0;
    net.set_omega(engine, 314.0);
    net.set_omega(load, 0.0);

    const real h = 1.0 / 960.0;
    Assembly assembly;
    int lock_substep = -1;
    real t = 0.0;
    real max_lambda_pre_lock = 0.0;
    for (int s = 0; s < 20000 && lock_substep < 0; ++s) {
        t += h;
        const real cap_kinetic = 300.0 * t;
        const real cap_static = 1.15 * cap_kinetic;

        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        assembly.set_friction_capacity(row, cap_static, cap_kinetic);
        assembly.torque(engine, tau_e, 0.0);
        const real wl_old = net.omega(load);
        assembly.torque(load, -d_load * wl_old, d_load); // tau0 - d*(w'-w_old) = -d*w' at the implicit solution

        net.solve(h, assembly);

        // Exact LCP property (plan section 13.3): while slipping, |lambda|
        // sits exactly at the currently-active capacity - static on the
        // first substep after a stuck one, kinetic otherwise; here the row
        // starts Free (never stuck yet), so it is kinetic throughout the
        // slipping phase.
        if (!net.row_stuck(row)) {
            CHECK(std::abs(net.row_lambda(row)) <= cap_static * (1.0 + 1e-9));
            CHECK(std::abs(std::abs(net.row_lambda(row)) - cap_kinetic) <= 1e-12 * std::max(real(1.0), cap_kinetic));
            max_lambda_pre_lock = std::max(max_lambda_pre_lock, std::abs(net.row_lambda(row)));
        } else {
            lock_substep = s;
        }
    }
    REQUIRE(lock_substep >= 0);
    REQUIRE(lock_substep > 0); // did not lock instantly (cap(0) == 0)

    const real lambda_at_lock = std::abs(net.row_lambda(row));
    // No spike ON the lock substep itself, relative to the capacity level
    // that was active going into it (Q-I3-1's literal 1.05x ratio).
    CHECK(lambda_at_lock <= 1.05 * max_lambda_pre_lock);

    // Post-lock: an INDEPENDENT scalar recursion of the exact same discrete
    // linearly-implicit-Euler update ShaftNetwork::solve applies (plain
    // arithmetic here, no call into network.h/box_lcp.h/dense.h) - the
    // plan's own reference (section 13.3), but evaluated discretely rather
    // than from the continuous ODE's closed form, since the plan's 1e-9
    // tolerance is tight enough that it can only mean "matches this
    // substep's own implicit solve", not a continuous approximation of it:
    //   m_load = J_L + h*d_load           (A's load entry, d_engine == 0)
    //   nu_free_load   = J_L*w_load_old/m_load     (tau0_load==d_load*w_load_old cancels exactly)
    //   nu_free_engine = w_engine_old + h*tau_e/J_e
    //   w_scalar = h*(1/m_load + 1/J_e)   ("row a"=load: +1, "row b"=engine: -1)
    //   lambda_ref = (nu_free_load - nu_free_engine) / w_scalar
    //   w_load_new_ref   = nu_free_load   + h*lambda_ref/m_load
    //   w_engine_new_ref = nu_free_engine - h*lambda_ref/J_e
    real max_lambda_post_lock = lambda_at_lock;
    real w_engine_ref = net.omega(engine), w_load_ref = net.omega(load); // == each other: just locked
    for (int s = 0; s < 960; ++s) { // 1 s at h = 1/960
        t += h;
        const real cap_kinetic = 300.0 * t;
        const real cap_static = 1.15 * cap_kinetic; // capacity keeps ramping but is now far above what's needed

        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        assembly.set_friction_capacity(row, cap_static, cap_kinetic);
        assembly.torque(engine, tau_e, 0.0);
        const real wl_old = net.omega(load);
        assembly.torque(load, -d_load * wl_old, d_load);

        net.solve(h, assembly);
        REQUIRE(net.row_stuck(row)); // never unlocks again in this test

        const real m_load = J_L + h * d_load;
        const real nu_free_load = J_L * w_load_ref / m_load;
        const real nu_free_engine = w_engine_ref + h * tau_e / J_e;
        const real w_scalar = h * (real(1.0) / m_load + real(1.0) / J_e);
        // box_lcp.h's own convention: v = W*lambda + q, v == 0 (stuck) means
        // lambda = -q/W, not +q/W (box_lcp.h:6-9).
        const real lambda_ref = -(nu_free_load - nu_free_engine) / w_scalar;
        w_load_ref = nu_free_load + h * lambda_ref / m_load;
        w_engine_ref = nu_free_engine - h * lambda_ref / J_e;

        CHECK(net.omega(engine) == Catch::Approx(w_engine_ref).epsilon(1e-9));
        CHECK(net.omega(load) == Catch::Approx(w_load_ref).epsilon(1e-9));
        const real lambda_now = net.row_lambda(row);
        CHECK(std::abs(lambda_now - lambda_ref) <= 1e-9 * std::max(real(1.0), std::abs(lambda_ref)));
        max_lambda_post_lock = std::max(max_lambda_post_lock, std::abs(lambda_now));
    }
    // Q-I3-1's literal criterion: after lock, torque never exceeds 1.05x
    // the level at the moment of lock (which is itself >= the eventual
    // steady value here, since the engine decelerates into lock - the
    // "drop, then ring" shape plan section 13.3 describes, not a spike).
    CHECK(max_lambda_post_lock <= 1.05 * lambda_at_lock);
}
