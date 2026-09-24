#include "box_lcp.h"

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {

void build_regularized_problem(const real* W, const real* q, const real* c, int k, LcpWorkspace& ws) {
    real trace = 0.0;
    for (int i = 0; i < k; ++i) trace += W[i * k + i];
    const real eps = real(1e-12) * trace / static_cast<real>(k);

    ws.Wr.set_size(static_cast<std::size_t>(k), static_cast<std::size_t>(k));
    for (int i = 0; i < k; ++i) {
        for (int j = 0; j < k; ++j) {
            ws.Wr(static_cast<std::size_t>(i), static_cast<std::size_t>(j)) =
                W[i * k + j] + (i == j ? eps : real(0.0));
        }
    }

    real vscale = 0.0;
    for (int i = 0; i < k; ++i) vscale = std::max(vscale, std::abs(q[i]));
    for (int i = 0; i < k; ++i) {
        real rowsum = 0.0;
        for (int j = 0; j < k; ++j) rowsum += std::abs(ws.Wr(static_cast<std::size_t>(i), static_cast<std::size_t>(j))) * c[j];
        vscale = std::max(vscale, rowsum);
    }
    vscale = std::max(vscale, real(1e-300));
    ws.tol_v = real(1e-12) * vscale;
}

int evaluate_partition(const real* q, const real* c, int k, const LcpSet* sets, LcpWorkspace& ws, int* infeasible_out) {
    ws.lam.set_size(static_cast<std::size_t>(k));
    for (int i = 0; i < k; ++i) {
        if (sets[i] == LcpSet::Lo) ws.lam[static_cast<std::size_t>(i)] = -c[i];
        else if (sets[i] == LcpSet::Hi) ws.lam[static_cast<std::size_t>(i)] = c[i];
        // Free entries are filled by the solve below.
    }

    int nf = 0;
    for (int i = 0; i < k; ++i) {
        if (sets[i] == LcpSet::Free) ws.free_idx[static_cast<std::size_t>(nf++)] = i;
    }

    if (nf > 0) {
        ws.Wff.set_size(static_cast<std::size_t>(nf), static_cast<std::size_t>(nf));
        for (int a = 0; a < nf; ++a) {
            for (int b = 0; b < nf; ++b) {
                ws.Wff(static_cast<std::size_t>(a), static_cast<std::size_t>(b)) =
                    ws.Wr(static_cast<std::size_t>(ws.free_idx[static_cast<std::size_t>(a)]),
                          static_cast<std::size_t>(ws.free_idx[static_cast<std::size_t>(b)]));
            }
        }
        ws.rhs_f.set_size(static_cast<std::size_t>(nf));
        for (int a = 0; a < nf; ++a) {
            const int i = ws.free_idx[static_cast<std::size_t>(a)];
            real s = q[i];
            for (int j = 0; j < k; ++j) {
                if (sets[j] != LcpSet::Free) s += ws.Wr(static_cast<std::size_t>(i), static_cast<std::size_t>(j)) * ws.lam[static_cast<std::size_t>(j)];
            }
            ws.rhs_f[static_cast<std::size_t>(a)] = -s;
        }
        // Wff is a principal submatrix of Wr (which is SPD by construction
        // - W + eps*I with eps from build_regularized_problem), so it is
        // itself SPD; cholesky_factor should never need to regularise here
        // (that telemetry belongs to network.h's own main-system Cholesky,
        // section 5.2's nonspd_hits, not this inner free-block solve).
        cholesky_factor(ws.Wff, static_cast<std::size_t>(nf), ws.Lff);
        cholesky_solve(ws.Lff, static_cast<std::size_t>(nf), ws.rhs_f, ws.lam_f);
        for (int a = 0; a < nf; ++a) {
            ws.lam[static_cast<std::size_t>(ws.free_idx[static_cast<std::size_t>(a)])] = ws.lam_f[static_cast<std::size_t>(a)];
        }
    }

    ws.v.set_size(static_cast<std::size_t>(k));
    for (int i = 0; i < k; ++i) {
        real s = q[i];
        for (int j = 0; j < k; ++j) s += ws.Wr(static_cast<std::size_t>(i), static_cast<std::size_t>(j)) * ws.lam[static_cast<std::size_t>(j)];
        ws.v[static_cast<std::size_t>(i)] = s;
    }

    int ni = 0;
    for (int i = 0; i < k; ++i) {
        const real tol_l = real(1e-12) * c[i];
        bool infeasible = false;
        if (sets[i] == LcpSet::Free) {
            const real li = ws.lam[static_cast<std::size_t>(i)];
            if (li < -c[i] - tol_l || li > c[i] + tol_l) infeasible = true;
        } else if (sets[i] == LcpSet::Lo) {
            if (ws.v[static_cast<std::size_t>(i)] < -ws.tol_v) infeasible = true;
        } else { // Hi
            if (ws.v[static_cast<std::size_t>(i)] > ws.tol_v) infeasible = true;
        }
        if (infeasible) infeasible_out[ni++] = i;
    }
    return ni;
}

namespace {
void flip(LcpSet* sets, int i, const SmallVec<kMaxLcpK>& lam, const real* c) {
    switch (sets[i]) {
        case LcpSet::Free:
            sets[i] = (lam[static_cast<std::size_t>(i)] < -c[i]) ? LcpSet::Lo : LcpSet::Hi;
            break;
        case LcpSet::Lo:
            sets[i] = LcpSet::Free;
            break;
        case LcpSet::Hi:
            sets[i] = LcpSet::Free;
            break;
    }
}
} // namespace

LcpResult solve_box_lcp(const real* W, const real* q, const real* c, int k, LcpSet* sets_inout, real* lambda_out,
                         LcpWorkspace& ws) {
    build_regularized_problem(W, q, c, k, ws);

    const int cap = 2 * k + 2;
    const int pbar = 3;
    int p = pbar;
    int ninf = k + 1;

    std::array<int, kMaxLcpK> infeasible{};
    int iterations = 0;
    for (int it = 1; it <= cap; ++it) {
        iterations = it;
        const int ni = evaluate_partition(q, c, k, sets_inout, ws, infeasible.data());

        if (ni == 0) {
            for (int i = 0; i < k; ++i) lambda_out[i] = std::clamp(ws.lam[static_cast<std::size_t>(i)], -c[i], c[i]);
            return {iterations, false};
        }

        if (ni < ninf) {
            ninf = ni;
            p = pbar;
            for (int t = 0; t < ni; ++t) flip(sets_inout, infeasible[static_cast<std::size_t>(t)], ws.lam, c);
        } else if (p > 0) {
            --p;
            for (int t = 0; t < ni; ++t) flip(sets_inout, infeasible[static_cast<std::size_t>(t)], ws.lam, c);
        } else {
            const int istar = infeasible[static_cast<std::size_t>(ni - 1)]; // ascending list -> max index last
            flip(sets_inout, istar, ws.lam, c);
        }
    }

    for (int i = 0; i < k; ++i) lambda_out[i] = std::clamp(ws.lam[static_cast<std::size_t>(i)], -c[i], c[i]);
    return {iterations, true};
}

} // namespace ps::drivetrain
