#include "network.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace ps::drivetrain {
namespace {

// Build-time union-find (plan section 4.4) - plain std::vector, fine at
// construction time (D8 only forbids allocation inside the per-substep job).
class UnionFind {
public:
    explicit UnionFind(int n) : parent_(static_cast<std::size_t>(n)) { std::iota(parent_.begin(), parent_.end(), 0); }
    int find(int x) {
        while (parent_[static_cast<std::size_t>(x)] != x) {
            parent_[static_cast<std::size_t>(x)] = parent_[static_cast<std::size_t>(parent_[static_cast<std::size_t>(x)])];
            x = parent_[static_cast<std::size_t>(x)];
        }
        return x;
    }
    void unite(int a, int b) {
        // "Smaller root port index wins" (plan section 4.1 step 3) - here
        // applied to node indices, giving a deterministic root regardless
        // of union order.
        const int ra = find(a), rb = find(b);
        if (ra == rb) return;
        if (ra < rb) parent_[static_cast<std::size_t>(rb)] = ra;
        else parent_[static_cast<std::size_t>(ra)] = rb;
    }

private:
    std::vector<int> parent_;
};

template <typename T>
std::uint64_t fnv1a(std::uint64_t h, const T& v) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}

} // namespace

ShaftNetwork NetworkBuilder::build(const std::string& vehicle_name) const {
    const int node_count = static_cast<int>(node_inertia_.size());
    if (node_count > kMaxNetworkNodes) {
        throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain network has " + std::to_string(node_count) +
                                  " nodes, exceeding kMaxNetworkNodes (" + std::to_string(kMaxNetworkNodes) + ")");
    }
    if (static_cast<int>(friction_rows_.size()) > kMaxNetworkFrictionRows) {
        throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain network has " +
                                  std::to_string(friction_rows_.size()) + " friction rows, exceeding kMaxNetworkFrictionRows (" +
                                  std::to_string(kMaxNetworkFrictionRows) + ")");
    }

    // Union nodes: every rigid row of EVERY state (plan 4.4 - clusters
    // never change with topology), plus friction rows between two
    // non-ground nodes.
    UnionFind uf(std::max(node_count, 1));
    for (const auto& rows : rows_by_state_) {
        for (const RigidRow& row : rows) {
            if (row.terms.empty()) continue;
            const int first = row.terms.front().first;
            for (std::size_t t = 1; t < row.terms.size(); ++t) uf.unite(first, row.terms[t].first);
        }
    }
    for (const FrictionRowDesc& fd : friction_rows_) {
        if (fd.b.valid()) uf.unite(fd.a.index, fd.b.index);
    }

    // Group nodes by cluster root, name each cluster by its lowest node
    // (plan 4.4), solve in ascending order - achieved here by sorting
    // cluster roots ascending and numbering clusters in that order.
    std::vector<int> root_of_node(static_cast<std::size_t>(node_count));
    for (int i = 0; i < node_count; ++i) root_of_node[static_cast<std::size_t>(i)] = uf.find(i);

    std::vector<int> distinct_roots;
    for (int i = 0; i < node_count; ++i) {
        const int r = root_of_node[static_cast<std::size_t>(i)];
        if (std::find(distinct_roots.begin(), distinct_roots.end(), r) == distinct_roots.end()) distinct_roots.push_back(r);
    }
    std::sort(distinct_roots.begin(), distinct_roots.end());

    ShaftNetwork net;
    net.node_inertia_ = node_inertia_;
    net.omega_.assign(static_cast<std::size_t>(node_count), real(0.0));
    net.node_cluster_.assign(static_cast<std::size_t>(node_count), -1);
    net.node_local_.assign(static_cast<std::size_t>(node_count), -1);

    net.clusters_.resize(distinct_roots.size());
    for (std::size_t ci = 0; ci < distinct_roots.size(); ++ci) {
        const int root = distinct_roots[ci];
        Cluster& c = net.clusters_[ci];
        for (int i = 0; i < node_count; ++i) {
            if (root_of_node[static_cast<std::size_t>(i)] != root) continue;
            if (c.node_count >= kMaxClusterNodes) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': a powertrain cluster exceeds kMaxClusterNodes (" +
                                          std::to_string(kMaxClusterNodes) + ")");
            }
            c.global_node[static_cast<std::size_t>(c.node_count)] = i;
            net.node_cluster_[static_cast<std::size_t>(i)] = static_cast<int>(ci);
            net.node_local_[static_cast<std::size_t>(i)] = c.node_count;
            ++c.node_count;
        }
    }

    // Per-state null-space basis, per cluster.
    net.current_state_ = 0;
    for (int state = 0; state < num_states_; ++state) {
        const std::vector<RigidRow>& state_rows = state < static_cast<int>(rows_by_state_.size()) ? rows_by_state_[static_cast<std::size_t>(state)] : std::vector<RigidRow>{};
        for (std::size_t ci = 0; ci < net.clusters_.size(); ++ci) {
            Cluster& c = net.clusters_[ci];
            if (state >= kMaxStates) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain has more than kMaxStates (" +
                                          std::to_string(kMaxStates) + ") topology states");
            }

            // Remap this state's rigid rows into cluster-local node indices
            // (only rows touching this cluster's nodes - by construction of
            // the union-find above, every row's nodes are ALL in the SAME
            // cluster).
            std::vector<ConstraintRow> local_rows;
            for (const RigidRow& row : state_rows) {
                if (row.terms.empty()) continue;
                if (net.node_cluster_[static_cast<std::size_t>(row.terms.front().first)] != static_cast<int>(ci)) continue;
                ConstraintRow lr;
                for (const auto& [gnode, coef] : row.terms) {
                    lr.terms.emplace_back(net.node_local_[static_cast<std::size_t>(gnode)], coef);
                }
                local_rows.push_back(std::move(lr));
            }

            std::vector<real> local_inertia(static_cast<std::size_t>(c.node_count));
            for (int local = 0; local < c.node_count; ++local) local_inertia[static_cast<std::size_t>(local)] = node_inertia_[static_cast<std::size_t>(c.global_node[static_cast<std::size_t>(local)])];

            NullspaceBasis basis;
            const int n = build_nullspace(local_rows, local_inertia, c.node_count, basis);
            if (n > kMaxDofs) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': a powertrain cluster's reduced DOF (" + std::to_string(n) +
                                          ") exceeds kMaxDofs (" + std::to_string(kMaxDofs) + ")");
            }

            // Validate: the reduced mass matrix T^T * diag(J) * T must be
            // invertible without regularisation (plan section 4.3).
            if (n > 0) {
                SmallMat<kMaxDofs, kMaxDofs> M;
                M.zero();
                M.set_size(static_cast<std::size_t>(n), static_cast<std::size_t>(n));
                for (int local = 0; local < c.node_count; ++local) {
                    const real J = local_inertia[static_cast<std::size_t>(local)];
                    if (J == real(0.0)) continue;
                    for (int a = 0; a < n; ++a) {
                        const real ta = basis.at(local, a);
                        if (ta == real(0.0)) continue;
                        for (int b = 0; b < n; ++b) M(static_cast<std::size_t>(a), static_cast<std::size_t>(b)) += J * ta * basis.at(local, b);
                    }
                }
                SmallMat<kMaxDofs, kMaxDofs> L;
                int hits = 0;
                cholesky_factor(M, static_cast<std::size_t>(n), L, &hits);
                if (hits > 0) {
                    throw std::runtime_error("vehicle '" + vehicle_name +
                                              "': cluster contains a reduced coordinate with zero inertia "
                                              "(e.g. an unconnected zero-inertia port)");
                }
            } else if (c.node_count > 0) {
                // n == 0 with nodes present: every node is rigidly pinned
                // (e.g. a single massless node with no free DOF) - only
                // valid if none of them is a hub (checked by the caller,
                // which knows which nodes are hubs; NetworkBuilder itself
                // has no notion of "hub").
            }

            c.basis[static_cast<std::size_t>(state)] = std::move(basis);
            c.state_count = state + 1;
        }
    }

    // Friction rows: assign each to its cluster (both endpoints already
    // guaranteed same cluster by the union-find above).
    net.friction_row_desc_ = friction_rows_;
    net.friction_row_cluster_.resize(friction_rows_.size());
    net.row_lambda_.assign(friction_rows_.size(), real(0.0));
    net.row_velocity_.assign(friction_rows_.size(), real(0.0));
    net.row_stuck_.assign(friction_rows_.size(), false);
    net.row_stuck_prev_.assign(friction_rows_.size(), false);
    net.row_set_.assign(friction_rows_.size(), LcpSet::Free);

    for (std::size_t rid = 0; rid < friction_rows_.size(); ++rid) {
        const FrictionRowDesc& fd = friction_rows_[rid];
        const int cluster_idx = net.node_cluster_[fd.a.index];
        net.friction_row_cluster_[rid] = cluster_idx;
        Cluster& c = net.clusters_[static_cast<std::size_t>(cluster_idx)];
        if (c.row_count >= kMaxRows) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': a powertrain cluster exceeds kMaxRows (" +
                                      std::to_string(kMaxRows) + ") friction rows");
        }
        c.local_rows[static_cast<std::size_t>(c.row_count)] = FrictionRowId{static_cast<std::uint16_t>(rid)};
        ++c.row_count;
    }

    return net;
}

void ShaftNetwork::solve(real h, const Assembly& assembly) {
    // Warm-start sets persist across substeps per friction row (plan
    // section 5.5, row_set_ member) - read into each cluster's own
    // lcp_sets scratch before calling solve_box_lcp, written back after.
    for (Cluster& c : clusters_) {
        const NullspaceBasis& basis = c.basis[static_cast<std::size_t>(current_state_)];
        const int n = basis.rank;

        std::array<real, kMaxClusterNodes> m{}, rhs{};
        for (int local = 0; local < c.node_count; ++local) {
            const int global = c.global_node[static_cast<std::size_t>(local)];
            const real J = node_inertia_[static_cast<std::size_t>(global)];
            const real w_old = omega_[static_cast<std::size_t>(global)];
            const NodeId nid{static_cast<std::uint16_t>(global)};
            const real d = assembly.d(nid);
            const real tau0 = assembly.tau0(nid);
            m[static_cast<std::size_t>(local)] = J + h * d;
            rhs[static_cast<std::size_t>(local)] = J * w_old + h * (tau0 + d * w_old);
        }

        c.A.zero();
        c.A.set_size(static_cast<std::size_t>(n), static_cast<std::size_t>(n));
        c.r.zero();
        c.r.set_size(static_cast<std::size_t>(n));
        for (int local = 0; local < c.node_count; ++local) {
            for (int a = 0; a < n; ++a) {
                const real ta = basis.at(local, a);
                if (ta == real(0.0)) continue;
                c.r[static_cast<std::size_t>(a)] += rhs[static_cast<std::size_t>(local)] * ta;
                for (int b = 0; b < n; ++b) {
                    c.A(static_cast<std::size_t>(a), static_cast<std::size_t>(b)) += m[static_cast<std::size_t>(local)] * ta * basis.at(local, b);
                }
            }
        }

        int hits = 0;
        cholesky_factor(c.A, static_cast<std::size_t>(n), c.L, &hits);
        c.nonspd_hits += hits;
        cholesky_solve(c.L, static_cast<std::size_t>(n), c.r, c.nu_free);

        std::array<int, kMaxRows> active_local{};
        int k = 0;
        for (int lr = 0; lr < c.row_count; ++lr) {
            const FrictionRowId rid = c.local_rows[static_cast<std::size_t>(lr)];
            const real ci = row_stuck_prev_[rid.index] ? assembly.cap_static(rid) : assembly.cap_kinetic(rid);
            if (ci > real(0.0)) {
                active_local[static_cast<std::size_t>(k)] = lr;
                c.cap_packed[static_cast<std::size_t>(k)] = ci;
                ++k;
            } else {
                row_lambda_[rid.index] = 0.0;
                row_stuck_[rid.index] = false;
            }
        }

        c.nu.set_size(static_cast<std::size_t>(n));
        if (k > 0) {
            c.B.zero();
            c.B.set_size(static_cast<std::size_t>(k), static_cast<std::size_t>(n));
            for (int a = 0; a < k; ++a) {
                const FrictionRowId rid = c.local_rows[static_cast<std::size_t>(active_local[static_cast<std::size_t>(a)])];
                const FrictionRowDesc& fd = friction_row_desc_[rid.index];
                const int local_a = node_local_[fd.a.index];
                for (int col = 0; col < n; ++col) c.B(static_cast<std::size_t>(a), static_cast<std::size_t>(col)) += basis.at(local_a, col);
                if (fd.b.valid()) {
                    // PHYS-033 I3 (plan section 5.5): a synchro row's b-side
                    // carries a per-substep ratio (v = omega(a) -
                    // ratio*omega(b)); every I2 row's ratio defaults to
                    // exactly 1.0 (Assembly::begin_substep_rows), so this is
                    // bit-identical to the old unconditional "-=" for them.
                    const real ratio = assembly.ratio_b(rid);
                    const int local_b = node_local_[fd.b.index];
                    for (int col = 0; col < n; ++col) c.B(static_cast<std::size_t>(a), static_cast<std::size_t>(col)) -= ratio * basis.at(local_b, col);
                }
            }

            c.Y.zero();
            c.Y.set_size(static_cast<std::size_t>(n), static_cast<std::size_t>(k));
            for (int a = 0; a < k; ++a) {
                SmallVec<kMaxDofs> bcol;
                bcol.set_size(static_cast<std::size_t>(n));
                for (int col = 0; col < n; ++col) bcol[static_cast<std::size_t>(col)] = c.B(static_cast<std::size_t>(a), static_cast<std::size_t>(col));
                SmallVec<kMaxDofs> ycol;
                cholesky_solve(c.L, static_cast<std::size_t>(n), bcol, ycol);
                for (int col = 0; col < n; ++col) c.Y(static_cast<std::size_t>(col), static_cast<std::size_t>(a)) = ycol[static_cast<std::size_t>(col)];
            }

            for (int a = 0; a < k; ++a) {
                for (int b = 0; b < k; ++b) {
                    real s = 0.0;
                    for (int col = 0; col < n; ++col) s += c.B(static_cast<std::size_t>(a), static_cast<std::size_t>(col)) * c.Y(static_cast<std::size_t>(col), static_cast<std::size_t>(b));
                    c.w_packed[static_cast<std::size_t>(a * k + b)] = h * s;
                }
            }
            for (int a = 0; a < k; ++a) {
                for (int b = a + 1; b < k; ++b) {
                    const real avg = real(0.5) * (c.w_packed[static_cast<std::size_t>(a * k + b)] + c.w_packed[static_cast<std::size_t>(b * k + a)]);
                    c.w_packed[static_cast<std::size_t>(a * k + b)] = avg;
                    c.w_packed[static_cast<std::size_t>(b * k + a)] = avg;
                }
            }
            for (int a = 0; a < k; ++a) {
                real s = 0.0;
                for (int col = 0; col < n; ++col) s += c.B(static_cast<std::size_t>(a), static_cast<std::size_t>(col)) * c.nu_free[static_cast<std::size_t>(col)];
                c.q_packed[static_cast<std::size_t>(a)] = s;
            }
            for (int a = 0; a < k; ++a) {
                const FrictionRowId rid = c.local_rows[static_cast<std::size_t>(active_local[static_cast<std::size_t>(a)])];
                c.lcp_sets[static_cast<std::size_t>(a)] = row_set_[rid.index];
            }

            const LcpResult lr_result =
                solve_box_lcp(c.w_packed.data(), c.q_packed.data(), c.cap_packed.data(), k, c.lcp_sets.data(), c.lambda_packed.data(), c.lcp_ws);
            if (lr_result.cap_hit) ++friction_cap_hits_;
            if (lr_result.stage_used >= 3) ++lcp_stage3_hits_;

            for (int col = 0; col < n; ++col) {
                real s = c.nu_free[static_cast<std::size_t>(col)];
                for (int a = 0; a < k; ++a) s += h * c.Y(static_cast<std::size_t>(col), static_cast<std::size_t>(a)) * c.lambda_packed[static_cast<std::size_t>(a)];
                c.nu[static_cast<std::size_t>(col)] = s;
            }

            for (int a = 0; a < k; ++a) {
                const FrictionRowId rid = c.local_rows[static_cast<std::size_t>(active_local[static_cast<std::size_t>(a)])];
                row_lambda_[rid.index] = c.lambda_packed[static_cast<std::size_t>(a)];
                row_stuck_[rid.index] = (c.lcp_sets[static_cast<std::size_t>(a)] == LcpSet::Free);
                row_set_[rid.index] = c.lcp_sets[static_cast<std::size_t>(a)];
            }
        } else {
            for (int col = 0; col < n; ++col) c.nu[static_cast<std::size_t>(col)] = c.nu_free[static_cast<std::size_t>(col)];
        }

        c.w_new_local.set_size(static_cast<std::size_t>(c.node_count));
        for (int local = 0; local < c.node_count; ++local) {
            real s = 0.0;
            for (int col = 0; col < n; ++col) s += basis.at(local, col) * c.nu[static_cast<std::size_t>(col)];
            c.w_new_local[static_cast<std::size_t>(local)] = s;
            omega_[static_cast<std::size_t>(c.global_node[static_cast<std::size_t>(local)])] = s;
        }

        // Velocity for EVERY row in this cluster (active or not, plan
        // section 5.2: "v_a = B_a nu" - here read directly off the already-
        // computed w_new_local, algebraically identical and cheaper).
        for (int lr = 0; lr < c.row_count; ++lr) {
            const FrictionRowId rid = c.local_rows[static_cast<std::size_t>(lr)];
            const FrictionRowDesc& fd = friction_row_desc_[rid.index];
            const int local_a = node_local_[fd.a.index];
            real v = c.w_new_local[static_cast<std::size_t>(local_a)];
            if (fd.b.valid()) v -= assembly.ratio_b(rid) * c.w_new_local[static_cast<std::size_t>(node_local_[fd.b.index])];
            row_velocity_[rid.index] = v;
        }
    }

    for (std::size_t i = 0; i < row_stuck_.size(); ++i) row_stuck_prev_[i] = row_stuck_[i];
}

std::uint64_t ShaftNetwork::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (real w : omega_) h = fnv1a(h, w);
    for (std::size_t i = 0; i < row_stuck_.size(); ++i) {
        const std::uint8_t stuck = row_stuck_[i] ? 1 : 0;
        h = fnv1a(h, stuck);
        const std::uint8_t set_val = static_cast<std::uint8_t>(row_set_[i]);
        h = fnv1a(h, set_val);
    }
    h = fnv1a(h, current_state_);
    return h;
}

} // namespace ps::drivetrain
