// core/src/drivetrain/network.h — the shaft network runtime (PHYS-033 P4
// I2 plan sections 3-5): reduced coordinates via a per-topology-state
// null-space basis (nullspace.h), linearly-implicit Euler per substep,
// friction rows solved as a box-LCP (box_lcp.h). Internal (ps::drivetrain).
//
// Build time (NetworkBuilder, sim thread, outside any job - ordinary
// std::vector allocation is fine here, PLAN.md D8 only forbids allocation
// INSIDE the per-substep job): declare nodes/inertias, rigid rows per
// topology state, friction rows; NetworkBuilder::build() unions nodes into
// clusters, computes each cluster's null-space basis per state (deterministic
// Gauss-Jordan, nullspace.h) and validates every cluster's reduced mass
// matrix is invertible.
//
// Per substep (ShaftNetwork::solve, called from inside the vehicle job):
// every Cluster's own fixed-capacity SmallMat/SmallVec workspace (sized at
// construction to kMaxDofs/kMaxClusterNodes/kMaxRows) is reused - nothing
// is resized. I2 has exactly one topology state (plan section 1); the
// state-indexed shape (topology_state parameter, one NullspaceBasis per
// state) exists so I3 (clutch/gearbox engagement) can add more without
// changing this file's own interface (PHYS-033 I3 plan section 0.5 builds
// on this).

#pragma once

#include "box_lcp.h"
#include "dense.h"
#include "nullspace.h"

#include "ps/types.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ps::drivetrain {

// D8 fixed capacities (PHYS-033 P4 I2 plan section 3). Exceeding any of
// these is a build-time error in Powertrain's constructor, naming the
// vehicle - never a silent truncation.
constexpr int kMaxDofs = 12;
constexpr int kMaxClusterNodes = 16;
constexpr int kMaxRows = 12;
constexpr int kMaxStates = 128;
// Additive to the plan's own table: bounds on the WHOLE network (every
// cluster's nodes/rows combined) - generous relative to car/tractor's
// actual 6-8 nodes and <=6 friction rows, so this is not expected to be a
// practical limit before I4/I5 (more components, more brakes/rows).
constexpr int kMaxNetworkNodes = 48;
constexpr int kMaxNetworkFrictionRows = 24;

struct NodeId {
    std::uint16_t index = 0xFFFFu;
    [[nodiscard]] bool valid() const { return index != 0xFFFFu; }
    friend bool operator==(NodeId a, NodeId b) { return a.index == b.index; }
    friend bool operator!=(NodeId a, NodeId b) { return a.index != b.index; }
};
// Sentinel meaning "the ground/chassis" wherever a friction row's "other
// side" is not a network node (e.g. a wheel brake, or the engine's own
// stiction-to-ground in I3) - NOT a valid index into any node array.
inline constexpr NodeId kGround{};

struct FrictionRowId {
    std::uint16_t index = 0;
};

// A rigid kinematic constraint row (plan section 4.2): "sum coef*omega(node)
// == 0", in GLOBAL node indices. Reused from nullspace.h's ConstraintRow
// (build time only).
using RigidRow = ConstraintRow;

// One cluster: a set of nodes plus every rigid row and friction row that
// touches only nodes within it (plan section 4.4 - clusters never change
// with topology state, since they are formed from the union of EVERY
// state's rows). `global_node[local]` maps a cluster-local index to the
// network's global NodeId::index.
struct Cluster {
    std::array<int, kMaxClusterNodes> global_node{};
    int node_count = 0;

    // One null-space basis per topology state actually used (I2: exactly
    // 1, index 0). Sized to kMaxStates so I3 can add engagement states
    // without touching this struct's layout; only `state_count` are valid.
    std::array<NullspaceBasis, kMaxStates> basis{};
    int state_count = 0;

    std::array<FrictionRowId, kMaxRows> local_rows{};
    int row_count = 0;

    // --- per-substep scratch (touched inside the job; fixed capacity,
    // never resized after construction - PLAN.md D8). ---
    SmallMat<kMaxDofs, kMaxDofs> A, L;
    SmallVec<kMaxDofs> r, nu_free, nu;
    SmallMat<kMaxRows, kMaxDofs> B;
    SmallMat<kMaxDofs, kMaxRows> Y;
    // box_lcp.h's calling convention is a PACKED row-major array (stride
    // k, not kMaxRows - see build_regularized_problem's `W[i*k+j]`), so
    // these cannot be SmallMat<kMaxRows,kMaxRows> (fixed stride kMaxRows) -
    // packed scratch instead, filled/read with an explicit stride of the
    // cluster's own active row count k each substep.
    std::array<real, kMaxRows * kMaxRows> w_packed{};
    std::array<real, kMaxRows> q_packed{};
    std::array<real, kMaxRows> cap_packed{};
    std::array<real, kMaxRows> lambda_packed{};
    std::array<LcpSet, kMaxRows> lcp_sets{};
    LcpWorkspace lcp_ws;
    SmallVec<kMaxClusterNodes> w_new_local;
    int nonspd_hits = 0; // cumulative telemetry (plan section 5.2/8) - should stay 0
};

// Per-node terms assembled by every component's pre_solve() each substep
// (plan section 5.1), cleared by begin_substep(). tau(w') ~= tau0 - d*(w'
// - w_old); d is the caller-supplied max(-dtau/dw, 0) (a negative slope
// stays explicit).
class Assembly {
public:
    void begin_substep(int node_count) {
        tau0_.set_size(static_cast<std::size_t>(node_count));
        d_.set_size(static_cast<std::size_t>(node_count));
        tau0_.zero();
        d_.zero();
    }
    void torque(NodeId n, real tau0, real d) {
        tau0_[n.index] += tau0;
        d_[n.index] += std::max(d, real(0.0));
    }
    [[nodiscard]] real tau0(NodeId n) const { return tau0_[n.index]; }
    [[nodiscard]] real d(NodeId n) const { return d_[n.index]; }

    void set_friction_capacity(FrictionRowId r, real cap_static, real cap_kinetic) {
        cap_static_[r.index] = cap_static;
        cap_kinetic_[r.index] = cap_kinetic;
    }
    void begin_substep_rows(int row_count) {
        cap_static_.set_size(static_cast<std::size_t>(row_count));
        cap_kinetic_.set_size(static_cast<std::size_t>(row_count));
        ratio_b_.set_size(static_cast<std::size_t>(row_count));
        cap_static_.zero();
        cap_kinetic_.zero();
        // PHYS-033 I3 (plan section 5.5): default 1.0 reproduces every I2
        // friction row's "v = omega(a) - omega(b)" definition exactly (a
        // gear/synchro row's caller overrides this per substep before
        // solve() - Assembly::begin_substep_rows always runs before any
        // component's pre_solve, plan section 7.2 step 4).
        for (std::size_t i = 0; i < ratio_b_.size(); ++i) ratio_b_[i] = real(1.0);
    }
    [[nodiscard]] real cap_static(FrictionRowId r) const { return cap_static_[r.index]; }
    [[nodiscard]] real cap_kinetic(FrictionRowId r) const { return cap_kinetic_[r.index]; }

    // PHYS-033 I3 (plan section 5.5): a synchro row's relative velocity is
    // omega(a) - ratio*omega(b), not omega(a) - omega(b) - the gearbox
    // updates `ratio` to the candidate gear's r_target each substep it is
    // in Synchronising. Every OTHER row (clutch, brakes, engine-to-ground)
    // never calls this, so its ratio stays the begin_substep_rows() default
    // of 1.0.
    void set_friction_row_ratio(FrictionRowId r, real ratio_b) { ratio_b_[r.index] = ratio_b; }
    [[nodiscard]] real ratio_b(FrictionRowId r) const { return ratio_b_[r.index]; }

private:
    SmallVec<kMaxNetworkNodes> tau0_;
    SmallVec<kMaxNetworkNodes> d_;
    SmallVec<kMaxNetworkFrictionRows> cap_static_;
    SmallVec<kMaxNetworkFrictionRows> cap_kinetic_;
    SmallVec<kMaxNetworkFrictionRows> ratio_b_;
};

// Build-time-only descriptor for one friction row (plan section 4.2): the
// relative velocity is omega(a) - omega(b) (b == kGround means omega(a)
// alone, e.g. a wheel brake).
struct FrictionRowDesc {
    NodeId a;
    NodeId b = kGround;
};

class ShaftNetwork {
public:
    [[nodiscard]] real omega(NodeId n) const { return omega_[n.index]; }
    // Build-time-constant node inertia J_i (PHYS-033 P4 I2 commit 5b): read
    // by Powertrain/components.cpp for the energy ledger's integrator_loss/
    // rotor_ke terms (plan section 8) and by GearPair's mesh-loss
    // reconstruction (section 5.4), neither of which has any other way to
    // see a node's own J.
    [[nodiscard]] real node_inertia(NodeId n) const { return node_inertia_[n.index]; }
    // Initial-condition / test seeding only (never called mid-substep from
    // inside the job on the hot path - Powertrain seeds hub omega once at
    // construction from the wheel's existing spin state, commit 5b).
    // Writes into an already-sized vector element: no allocation.
    void set_omega(NodeId n, real w) { omega_[n.index] = w; }
    [[nodiscard]] int node_count() const { return static_cast<int>(omega_.size()); }
    [[nodiscard]] int friction_row_count() const { return static_cast<int>(row_lambda_.size()); }

    [[nodiscard]] real row_lambda(FrictionRowId r) const { return row_lambda_[r.index]; }
    [[nodiscard]] real row_velocity(FrictionRowId r) const { return row_velocity_[r.index]; }
    [[nodiscard]] bool row_stuck(FrictionRowId r) const { return row_stuck_[r.index]; }
    // A friction row's own (a, b) endpoints (b == kGround for a wheel brake
    // row) - commit 5b's GearPair::post_solve needs this to find the
    // "Sigma(rows at a) coef*lambda_row" term in its mesh-torque
    // reconstruction (plan section 5.4); every other I2 caller only ever
    // needs row_lambda/row_velocity/row_stuck above.
    [[nodiscard]] NodeId friction_row_a(FrictionRowId r) const { return friction_row_desc_[r.index].a; }
    [[nodiscard]] NodeId friction_row_b(FrictionRowId r) const { return friction_row_desc_[r.index].b; }
    [[nodiscard]] int nonspd_hits() const {
        int total = 0;
        for (const Cluster& c : clusters_) total += c.nonspd_hits;
        return total;
    }
    [[nodiscard]] int friction_cap_hits() const { return friction_cap_hits_; }
    // Cumulative count of box_lcp.cpp's stage 3 (exhaustive active-set
    // enumeration) being invoked - i.e. LcpResult::stage_used >= 3 -
    // across every cluster's solve() calls (PHYS-033 P4 I2, coordinator
    // ruling: stage 3 exists because a box LCP's standard-LCP
    // reformulation is only P0, so stage 2's Murty fallback is not itself
    // guaranteed finite). Should stay 0 in ordinary operation - reaching
    // stage 3 at all is the "report, do not force" signal, not friction_
    // cap_hits (which now only fires if stage 3 ALSO fails, mathematically
    // unreachable for an SPD Wr). Surfaced as the World telemetry channel
    // `powertrain.lcp_stage3` once ShaftNetwork is wired into World
    // (commit 5b/6 - not yet done as of this commit).
    [[nodiscard]] int lcp_stage3_hits() const { return lcp_stage3_hits_; }

    // Topology switch (I3+; exercised here only by the test-only two-state
    // builder in test 11.2(d)). Clusters/bases for every declared state are
    // already precomputed at build time (plan section 2.4 step 4: "Tstate
    // precomputed and cached") - switching is just changing which
    // precomputed basis solve() reads next; the existing omega_ (this
    // tick's w_old) is reused as-is, which is exactly what makes an
    // engagement a momentum-conserving inelastic projection (plan section
    // 3.2) with no extra code.
    void set_topology_state(int state) { current_state_ = state; }
    [[nodiscard]] int topology_state() const { return current_state_; }

    // World::reset_vehicle (plan section 4.5): clears every friction row's
    // stick flags and LCP warm-start set back to their build-time defaults
    // (row_lambda_/row_velocity_ to 0, row_stuck_/row_stuck_prev_ to false,
    // row_set_ to LcpSet::Free - matches NetworkBuilder::build's own initial
    // values). Writes into already-sized vectors: no allocation. Does NOT
    // touch omega_ (the caller sets whichever nodes it wants directly via
    // set_omega) or the topology state (set separately via
    // set_topology_state) or the cumulative friction_cap_hits_/
    // lcp_stage3_hits_ diagnostic counters (not "stick flags/sets").
    void clear_friction_row_state() {
        std::fill(row_lambda_.begin(), row_lambda_.end(), real(0.0));
        std::fill(row_velocity_.begin(), row_velocity_.end(), real(0.0));
        std::fill(row_stuck_.begin(), row_stuck_.end(), false);
        std::fill(row_stuck_prev_.begin(), row_stuck_prev_.end(), false);
        std::fill(row_set_.begin(), row_set_.end(), LcpSet::Free);
    }

    // Solves every cluster (ascending order, plan section 4.4) for one
    // substep of size h, using `asm`'s per-node tau0/d and per-row
    // capacities (already filled by the caller's pre_solve pass). Reads
    // and updates `omega_` in place (the network's own persistent state).
    void solve(real h, const Assembly& assembly);

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    friend class NetworkBuilder;

private:
    std::vector<real> node_inertia_; // J_i, global node index -> inertia (build time constant)
    std::vector<real> omega_;        // global node index -> omega (network state, PERSISTS across substeps)
    std::vector<Cluster> clusters_;
    std::vector<int> node_cluster_;      // global node -> cluster index
    std::vector<int> node_local_;        // global node -> local index within its cluster
    std::vector<FrictionRowDesc> friction_row_desc_; // global friction row id -> (a,b)
    std::vector<int> friction_row_cluster_;          // global friction row id -> cluster index
    std::vector<real> row_lambda_;
    std::vector<real> row_velocity_;
    std::vector<bool> row_stuck_prev_;
    std::vector<bool> row_stuck_;
    std::vector<LcpSet> row_set_; // warm start (previous substep's BPP partition) per friction row
    int friction_cap_hits_ = 0;
    int lcp_stage3_hits_ = 0;
    int current_state_ = 0; // I2: always 0
};

// Build-time graph builder (Powertrain's constructor, sim thread).
class NetworkBuilder {
public:
    // Adds a node with the given inertia (>= 0), returns its NodeId.
    NodeId add_node(real inertia) {
        NodeId id{static_cast<std::uint16_t>(node_inertia_.size())};
        node_inertia_.push_back(inertia);
        return id;
    }

    // Registers a rigid constraint row for topology state `state` (I2:
    // always 0). `state` must be < kMaxStates.
    void add_rigid_row(int state, RigidRow row) {
        if (static_cast<int>(rows_by_state_.size()) <= state) rows_by_state_.resize(static_cast<std::size_t>(state) + 1);
        rows_by_state_[static_cast<std::size_t>(state)].push_back(std::move(row));
        num_states_ = std::max(num_states_, state + 1);
    }

    // Registers a friction row (relative velocity omega(a) - omega(b), b ==
    // kGround meaning just omega(a)). Returns its FrictionRowId.
    FrictionRowId add_friction_row(NodeId a, NodeId b = kGround) {
        FrictionRowId id{static_cast<std::uint16_t>(friction_rows_.size())};
        friction_rows_.push_back(FrictionRowDesc{a, b});
        return id;
    }

    // Builds the ShaftNetwork: unions nodes into clusters (every state's
    // rigid rows, plus friction rows between two non-ground nodes),
    // computes each cluster's null-space basis per state, and validates
    // every basis's reduced mass matrix is invertible (Cholesky succeeds
    // cleanly). Throws std::runtime_error (naming `vehicle_name`) on any
    // build-time error (plan section 4.3's validation, section 2.4 step 7).
    ShaftNetwork build(const std::string& vehicle_name) const;

private:
    std::vector<real> node_inertia_;
    std::vector<std::vector<RigidRow>> rows_by_state_;
    std::vector<FrictionRowDesc> friction_rows_;
    int num_states_ = 1;
};

} // namespace ps::drivetrain
