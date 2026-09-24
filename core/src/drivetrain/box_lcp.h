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
    // true only if stage 3 (exhaustive active-set enumeration, the final
    // fallback - see solve_box_lcp's own comment) ALSO fails to find a
    // feasible partition. Mathematically this should be unreachable for an
    // SPD W (build_regularized_problem's epsilon regularisation guarantees
    // this): stage 3 enumerates every one of the 3^k possible partitions in
    // a fixed order and returns the first feasible one, and for an SPD W
    // the box-LCP solution is unique, so at least one partition must
    // qualify. Reaching cap_hit==true is therefore a genuine "report, do
    // not force" signal (PHYS-033 P4 I2 plan section 14 stop condition 1),
    // not an expected/accepted outcome the way stage-1/stage-2 cap hits
    // were before stage 3 existed.
    bool cap_hit = false;
    // Which stage actually produced the returned lambda: 1 = block
    // principal pivoting converged within its own 2k+2 cap; 2 = stage 1's
    // cap was exceeded but Murty's least-index fallback converged within
    // its own 2^k cap; 3 = stage 2's cap was ALSO exceeded and exhaustive
    // enumeration (stage 3) had to run. Lets callers/telemetry
    // (ps::drivetrain::ShaftNetwork::lcp_stage3_hits, surfaced as the
    // World telemetry channel `powertrain.lcp_stage3` once the network is
    // wired into World - PHYS-033 P4 I2 commit 5b) count stage-3 usage
    // precisely instead of inferring it from `iterations` alone.
    int stage_used = 1;
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

// Three-stage solve (section 5.5; stage 3 added per coordinator ruling on
// PHYS-033 P4 I2 commit 2's cap-hit finding - the standard-LCP
// reformulation of a BOX LCP is only P0, not P, so Murty's finite-
// termination guarantee does not carry over to stage 2 either, and a
// cap-hit "last feasible iterate" is not an acceptable production answer:
// it can violate the stick/slip complementarity conditions the solver
// exists to enforce). `sets_inout` is both the warm start (previous
// substep's sets; a new/reactivated row should be passed in as Free) and,
// on return, the sets the solution settled in. `lambda_out` receives the
// solution, always box-feasible (clamped to [-c, c]).
//   Stage 1: block principal pivoting (Judice-Pires), capped at 2k+2.
//   Stage 2: Murty's LEAST-index single-pivot method, capped at 2^k
//            further pivots (finite for a P-matrix; not a full guarantee
//            here, see above, but resolves the vast majority of stage-1
//            cap hits in practice).
//   Stage 3: exhaustive active-set enumeration - every one of the 3^k
//            {Lo, Free, Hi} partitions in a FIXED (row-index-major,
//            ternary-counter) order, returning the FIRST one that
//            satisfies evaluate_partition's feasibility check. For an SPD
//            W (guaranteed by build_regularized_problem's epsilon
//            regularisation) the box-LCP solution is unique up to the
//            same tolerances evaluate_partition already uses, so exactly
//            one partition qualifies (mod tie-breaks) and the fixed order
//            makes the result deterministic. Allocates nothing (plain
//            stack-local fixed-size scratch, reusing `ws`'s existing
//            SmallMat/SmallVec-backed Cholesky) - k <= kMaxLcpK (8) means
//            at most 3^8 = 6561 candidates, each one Cholesky solve of at
//            most an 8x8 system, so this is a bounded but not cheap
//            fallback - stage 3 running at all is itself the signal
//            something upstream needs attention, not routine.
LcpResult solve_box_lcp(const real* W, const real* q, const real* c, int k, LcpSet* sets_inout, real* lambda_out,
                         LcpWorkspace& ws);

} // namespace ps::drivetrain
