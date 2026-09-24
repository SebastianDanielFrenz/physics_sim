// core/src/drivetrain/box_lcp.h — exact implicit Coulomb friction as a
// small box-LCP (PHYS-033 P4 I2 plan section 5.5, owner ruling Q1: F1
// accepted). Finite, capped block principal pivoting (Judice-Pires) with a
// Murty single-pivot fallback. Internal (ps::drivetrain).
//
// Problem: find lambda in [-c, c] (c > 0) with v = W*lambda + q, where
//   lambda_i = -c_i requires v_i >= 0;
//   lambda_i = +c_i requires v_i <= 0;
//   -c_i < lambda_i < c_i requires v_i = 0.
// These are the KKT conditions of min 1/2 lambda^T W lambda + q^T lambda on
// the box. For W > 0 the solution is unique.
//
// Callers (network.h) must pre-filter rows with c_i == 0 (lambda = 0, not
// "stuck" - plan section 11.1's own "separate caller-level case") before
// calling solve_box_lcp: every c_i passed in here is assumed > 0.

#pragma once

#include "dense.h"

#include "ps/types.h"

#include <array>
#include <cstdint>

namespace ps::drivetrain {

constexpr int kMaxLcpK = 8; // matches network.h's kMaxRows

enum class LcpSet : std::uint8_t { Lo, Free, Hi };

struct LcpResult {
    int iterations = 0;
    bool cap_hit = false;
};

// Per-cluster (or per-test-case) scratch, sized to kMaxLcpK - a member of
// network.h's Cluster in production (PLAN.md D8: no allocation inside the
// job), or a local stack variable in tests.
struct LcpWorkspace {
    SmallMat<kMaxLcpK, kMaxLcpK> Wr; // W + eps*I
    SmallMat<kMaxLcpK, kMaxLcpK> Wff;
    SmallMat<kMaxLcpK, kMaxLcpK> Lff;
    SmallVec<kMaxLcpK> rhs_f;
    SmallVec<kMaxLcpK> lam_f;
    SmallVec<kMaxLcpK> lam;
    SmallVec<kMaxLcpK> v;
    std::array<int, kMaxLcpK> free_idx{};
    real tol_v = 0.0;
};

// Builds ws.Wr = W + eps*I (eps = 1e-12*trace(W)/k, section 5.5's own
// epsilon-regularisation - makes a merely-PSD W with duplicate rows
// positive definite) and ws.tol_v = 1e-12*vscale, where
// vscale = max(max_i|q_i|, max_i sum_j|Wr_ij|*c_j, 1e-300). `W`/`q`/`c` are
// row-major/plain arrays of length k*k / k / k.
void build_regularized_problem(const real* W, const real* q, const real* c, int k, LcpWorkspace& ws);

// Solves the free block for the GIVEN (fixed, not iterated) partition
// `sets`, filling ws.lam (bound values from `c`, free values from the
// Cholesky solve of Wr[F][F]) and ws.v = Wr*lam + q. Appends the ascending
// list of infeasible indices into `infeasible_out` (caller-sized to at
// least k) and returns how many. This is the EXACT step solve_box_lcp's
// own pivoting loop performs each iteration - exposed so a brute-force
// cross-check (tests/unit/test_powertrain_friction_lcp.cpp, plan section
// 11.1) can evaluate an arbitrary candidate partition through the same
// code path, not a reimplementation of it.
//
// Feasibility (section 5.5): a Free index is infeasible if lam_i is
// outside [-c_i - tol_l(i), c_i + tol_l(i)] (tol_l(i) = 1e-12*c_i); a Lo
// index is infeasible if v_i < -ws.tol_v; a Hi index is infeasible if
// v_i > +ws.tol_v.
int evaluate_partition(const real* q, const real* c, int k, const LcpSet* sets, LcpWorkspace& ws, int* infeasible_out);

// Finite capped block principal pivoting (section 5.5). `sets_inout` is
// both the warm start (previous substep's sets; a new/reactivated row
// should be passed in as Free) and, on return, the sets the solution
// settled in. `lambda_out` receives the solution, always box-feasible
// (clamped to [-c, c]) whether or not the pivot cap (2k+2) was hit.
LcpResult solve_box_lcp(const real* W, const real* q, const real* c, int k, LcpSet* sets_inout, real* lambda_out,
                         LcpWorkspace& ws);

} // namespace ps::drivetrain
