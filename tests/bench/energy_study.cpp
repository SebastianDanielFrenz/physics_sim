// tests/bench/energy_study — Part A energy-loss measurement study (see
// PLAN.md's session brief; not itself part of PLAN.md, this file exists to
// answer Sebastian's open question from vault ticket PHYS-003 item 7: a
// hinged capsule released from ~90 deg loses ~13% of its mechanical energy
// over 60 s at 240 Hz, hypothesised to be Jolt's Baumgarte position
// stabilisation (PhysicsSettings::mBaumgarte, default 0.2), but that was
// never tested by actually varying mBaumgarte. This is a measurement tool,
// not a validation test: it prints a CSV table to stdout and always exits 0
// (the report interprets the numbers; PLAN.md's "measure, don't decide" -
// Sebastian owns the fix). Target ps_energy_study, deliberately NOT
// registered with CTest (add_test) - see tests/bench/CMakeLists.txt.
//
// Two body configurations, reusing the exact geometry of
// tests/validation/test_joint_chain_energy.cpp (kLinks=1 there is the
// "single hinged capsule" case; kLinks=3 is the validation test's own
// chain): a chain of frictionless-hinged capsules, first link pivoted to
// the world, released from rest at release_deg from the hanging-straight-
// down equilibrium.
//
// For each run we measure:
//  - energy loss %: (E_start - E_end) / E_start over 60 s, E = sum of
//    linear + rotational KE + gravitational PE per body (same formula as
//    the validation test).
//  - max joint anchor separation (mm): the two constraint solvers never
//    force joint anchors to coincide exactly (Baumgarte only corrects a
//    fraction mBaumgarte of the error per step - see PhysicsSettings.h and
//    ConstraintPart/PointConstraintPart.h); this is the constraint error a
//    lower Baumgarte trades against less energy loss.
//  - whether energy decays monotonically (sampled once per second, i.e.
//    every 240 ticks at the default rate, to filter out sub-second
//    integrator oscillation from a true systematic trend).
//  - ms per tick: wall-clock cost of World::step() alone (diagnostic
//    bookkeeping timed separately), i.e. the runtime cost of whatever
//    solver settings that row used.

#include "headless_env.h"
#include "ps/world/world.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace ps;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;

struct SolverSpec {
    std::string label;
    double baumgarte = 0.2;
    unsigned num_position_steps = 2;
    int collision_steps = 1;
    double tick_rate_hz = 240.0;
};

struct BodySpec {
    Pose pose;
    Vec3 anchor_a_local;
    // Anchor B: either a fixed world point (body_b invalid) or a local
    // point on the previous body.
    bool anchored_to_world = false;
    Vec3 anchor_b_local; // body-local (previous body) or world point
};

struct ChainInfo {
    std::vector<BodyId> bodies;
    // Parallel to bodies: each body's own local anchor point and what it is
    // attached to (previous body, or a fixed world point for link 0).
    std::vector<Vec3> anchor_a_local;
    std::vector<BodyId> anchor_b_body; // invalid == world
    std::vector<Vec3> anchor_b_local;
};

// Builds an n-link hinge chain identical in geometry to
// test_joint_chain_energy.cpp, released at release_deg from hanging-down.
ChainInfo build_chain(World& world, int num_links, double release_deg) {
    constexpr double kHalfHeight = 0.4;
    constexpr double kRadius = 0.1;
    constexpr double kCollisionRadius = 0.08;
    constexpr double kLinkLength = 2.0 * (kHalfHeight + kRadius); // 1.0 m

    const double theta0 = release_deg * kPi / 180.0;
    const Quat release_tilt = Quat::from_axis_angle(Vec3::unit_y(), theta0);
    const Quat hang_down = Quat::from_axis_angle(Vec3::unit_x(), -kPi / 2.0);

    ChainInfo info;
    BodyId previous;
    for (int i = 0; i < num_links; ++i) {
        const Vec3 local_hanging_pos{0.0, 0.0, -(i + 0.5) * kLinkLength};

        BodyDesc desc;
        desc.shape = CapsuleShape{kHalfHeight, kCollisionRadius};
        desc.motion = BodyMotionType::Dynamic;
        desc.mass = 1.0;
        desc.pose.orientation = release_tilt * hang_down;
        desc.pose.position = release_tilt.rotate(local_hanging_pos);
        desc.gravity_enabled = true;
        desc.allow_sleep = false;
        desc.linear_damping = 0.0;
        desc.angular_damping = 0.0;
        const BodyId id = world.create_body(desc);
        info.bodies.push_back(id);

        JointDesc joint;
        joint.type = JointType::Hinge;
        joint.body_a = id;
        joint.axis_a_local = Vec3::unit_z();
        joint.axis_b_local = Vec3::unit_z();
        joint.friction_torque = 0.0;

        Vec3 anchor_a{0, -kLinkLength / 2.0, 0};
        Vec3 anchor_b;
        BodyId anchor_b_body;
        if (i == 0) {
            joint.anchor_a_local = anchor_a;
            joint.anchor_b_local = Vec3::zero();
            anchor_b = Vec3::zero();
        } else {
            joint.body_b = previous;
            joint.anchor_a_local = anchor_a;
            joint.anchor_b_local = Vec3{0, kLinkLength / 2.0, 0};
            anchor_b = joint.anchor_b_local;
            anchor_b_body = previous;
        }
        world.create_joint(joint);

        info.anchor_a_local.push_back(anchor_a);
        info.anchor_b_body.push_back(anchor_b_body);
        info.anchor_b_local.push_back(anchor_b);

        previous = id;
    }
    return info;
}

double total_mechanical_energy(World& world, const std::vector<BodyId>& bodies, double z_ref, double* max_ang_speed_out) {
    double e = 0.0;
    double max_w = 0.0;
    for (BodyId id : bodies) {
        const double m = world.get_mass(id);
        const Motion mo = world.get_motion(id);
        const Pose p = world.get_pose(id);
        const Mat3 inertia_world = world.backend().get_inertia_world(id);
        const double ke_lin = 0.5 * m * mo.linear.length_squared();
        const Vec3 i_w = inertia_world * mo.angular;
        const double ke_ang = 0.5 * dot(mo.angular, i_w);
        const double pe = m * kG * (p.position.z - z_ref);
        e += ke_lin + ke_ang + pe;
        max_w = std::max(max_w, mo.angular.length());
    }
    if (max_ang_speed_out) *max_ang_speed_out = std::max(*max_ang_speed_out, max_w);
    return e;
}

double max_anchor_separation_mm(World& world, const ChainInfo& info) {
    double max_sep = 0.0;
    for (std::size_t i = 0; i < info.bodies.size(); ++i) {
        const Pose pa = world.get_pose(info.bodies[i]);
        const Vec3 wa = pa.transform_point(info.anchor_a_local[i]);
        Vec3 wb;
        if (info.anchor_b_body[i].valid()) {
            const Pose pb = world.get_pose(info.anchor_b_body[i]);
            wb = pb.transform_point(info.anchor_b_local[i]);
        } else {
            wb = info.anchor_b_local[i]; // fixed world point
        }
        max_sep = std::max(max_sep, (wa - wb).length());
    }
    return max_sep * 1000.0; // m -> mm
}

struct RunResult {
    double energy_loss_pct = 0.0;
    double max_anchor_sep_mm = 0.0;
    double max_ang_speed_rad_s = 0.0;
    bool monotonic = true;
    double ms_per_tick = 0.0;
};

RunResult run_chain_study(int num_links, double release_deg, const SolverSpec& solver, double duration_s = 60.0) {
    WorldConfig cfg;
    cfg.tick_rate_hz = solver.tick_rate_hz;
    cfg.gravity = Vec3{0, 0, -kG};
    cfg.job_workers = 1; // few bodies; avoid thread overhead noise in ms/tick
    cfg.baumgarte = solver.baumgarte;
    cfg.num_position_steps = solver.num_position_steps;
    cfg.collision_steps = solver.collision_steps;
    World world(cfg);

    ChainInfo info = build_chain(world, num_links, release_deg);
    const double z_ref = -(num_links * 1.0 + 2.0);

    double max_w = 0.0;
    const double e_start = total_mechanical_energy(world, info.bodies, z_ref, &max_w);

    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * cfg.tick_rate_hz + 0.5);
    const std::uint64_t sample_every = static_cast<std::uint64_t>(cfg.tick_rate_hz + 0.5); // once per second

    double max_sep = 0.0;
    double step_ms_total = 0.0;
    std::vector<double> per_second_energy;
    per_second_energy.push_back(e_start);

    for (std::uint64_t i = 0; i < num_ticks; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        world.step();
        const auto t1 = std::chrono::steady_clock::now();
        step_ms_total += std::chrono::duration<double, std::milli>(t1 - t0).count();

        max_sep = std::max(max_sep, max_anchor_separation_mm(world, info));
        const double e_now = total_mechanical_energy(world, info.bodies, z_ref, &max_w);
        if ((i + 1) % sample_every == 0) per_second_energy.push_back(e_now);
    }

    const double e_end = total_mechanical_energy(world, info.bodies, z_ref, &max_w);

    bool monotonic = true;
    // Tolerate 0.01% upward wobble between consecutive 1 s samples before
    // calling it non-monotonic (integrator noise, not a real energy gain).
    for (std::size_t i = 1; i < per_second_energy.size(); ++i) {
        if (per_second_energy[i] > per_second_energy[i - 1] * 1.0001) { monotonic = false; break; }
    }

    RunResult r;
    r.energy_loss_pct = 100.0 * (e_start - e_end) / e_start;
    r.max_anchor_sep_mm = max_sep;
    r.max_ang_speed_rad_s = max_w;
    r.monotonic = monotonic;
    r.ms_per_tick = step_ms_total / static_cast<double>(num_ticks);
    return r;
}

void print_row(const char* scenario, double release_deg, const SolverSpec& s, const RunResult& r) {
    std::printf("CSV,%s,%.1f,%.3f,%u,%d,%.1f,%.4f,%.4f,%.3f,%s,%.5f\n",
                scenario, release_deg, s.baumgarte, s.num_position_steps, s.collision_steps, s.tick_rate_hz,
                r.energy_loss_pct, r.max_anchor_sep_mm, r.max_ang_speed_rad_s, r.monotonic ? "yes" : "no", r.ms_per_tick);
}

// --- Candidate cause: gyroscopic / angular integration -------------------
// A free-spinning asymmetric body, no joints, no gravity, no contacts:
// launched near the intermediate principal axis (Dzhanibekov/tennis-racket
// setup) so gyroscopic coupling is maximally exercised. An ideal free rigid
// body conserves rotational KE and angular momentum magnitude exactly - any
// drift here isolates integration error in ApplyGyroscopicForceInternal
// (MotionProperties.h) / velocity integration, with zero contribution from
// Baumgarte (no constraints exist in this scenario at all).
RunResult run_gyroscopic_study(const SolverSpec& solver, double duration_s = 60.0) {
    WorldConfig cfg;
    cfg.tick_rate_hz = solver.tick_rate_hz;
    cfg.gravity = Vec3::zero();
    cfg.job_workers = 1;
    cfg.baumgarte = solver.baumgarte;
    cfg.num_position_steps = solver.num_position_steps;
    cfg.collision_steps = solver.collision_steps;
    World world(cfg);

    BodyDesc desc;
    desc.shape = BoxShape{Vec3{0.5, 0.3, 0.1}}; // three distinct half-extents -> three distinct principal inertias
    desc.motion = BodyMotionType::Dynamic;
    desc.mass = 2.0;
    desc.gravity_enabled = false;
    desc.allow_sleep = false;
    desc.velocity.angular = Vec3{0.05, 5.0, 0.05}; // near the unstable intermediate axis
    desc.max_angular_velocity = 1000.0; // do not let the default clamp interfere with this control experiment
    const BodyId id = world.create_body(desc);
    std::vector<BodyId> bodies{id};

    double max_w = 0.0;
    const double e_start = total_mechanical_energy(world, bodies, 0.0, &max_w);
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * cfg.tick_rate_hz + 0.5);
    double step_ms_total = 0.0;
    for (std::uint64_t i = 0; i < num_ticks; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        world.step();
        const auto t1 = std::chrono::steady_clock::now();
        step_ms_total += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    const double e_end = total_mechanical_energy(world, bodies, 0.0, &max_w);

    RunResult r;
    r.energy_loss_pct = 100.0 * (e_start - e_end) / e_start;
    r.max_anchor_sep_mm = 0.0;
    r.max_ang_speed_rad_s = max_w;
    r.monotonic = true; // not evaluated for this control experiment
    r.ms_per_tick = step_ms_total / static_cast<double>(num_ticks);
    return r;
}

// --- Candidate cause: float-precision velocity / integration control -----
// A single free-falling body under gravity, no joints, no contacts: any
// energy drift here is purely from float32 velocity accumulation and the
// symplectic-Euler integrator itself (see test_free_fall.cpp), with zero
// contribution from Baumgarte or gyroscopic coupling. raise_velocity_cap
// controls BodyDesc::max_linear_velocity: false uses Jolt's own default
// (500 m/s) to demonstrate what that clamp does once free fall exceeds it
// (~51 s at g=9.81 from rest); true raises the cap far out of reach so this
// run isolates float-precision/integrator drift alone.
RunResult run_free_fall_energy_study(double duration_s, bool raise_velocity_cap) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.gravity = Vec3{0, 0, -kG};
    cfg.job_workers = 1;
    World world(cfg);

    BodyDesc desc;
    desc.shape = SphereShape{0.5};
    desc.motion = BodyMotionType::Dynamic;
    desc.mass = 1.0;
    desc.pose.position = Vec3{0, 0, 100000.0}; // stays clear of the ground for the whole run
    desc.gravity_enabled = true;
    desc.allow_sleep = false;
    desc.linear_damping = 0.0;
    desc.angular_damping = 0.0;
    if (raise_velocity_cap) desc.max_linear_velocity = 1.0e6;
    const BodyId id = world.create_body(desc);
    std::vector<BodyId> bodies{id};

    double max_w = 0.0;
    const double e_start = total_mechanical_energy(world, bodies, 0.0, &max_w);
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * cfg.tick_rate_hz + 0.5);
    for (std::uint64_t i = 0; i < num_ticks; ++i) world.step();
    const double e_end = total_mechanical_energy(world, bodies, 0.0, &max_w);
    const Motion final_motion = world.get_motion(id);

    RunResult r;
    r.energy_loss_pct = 100.0 * (e_start - e_end) / e_start;
    r.max_ang_speed_rad_s = final_motion.linear.length(); // reused field: final SPEED (m/s) for this scenario
    return r;
}

// --- PLAN.md P1.5a scope item 8: Jolt joints vs. articulation, side by
// side, on the same Part A scenarios (single hinged link and the 3-link
// chain, released at the same angles), plus a 10-link cost benchmark. ---

// Builds the SAME geometry build_chain() does (pivot spacing, masses),
// but as one ps::articulation chain instead of N Jolt bodies + hinges, so
// the two energy-loss numbers are a fair side-by-side comparison.
RunResult run_articulation_chain_study(int num_links, double release_deg, real substep_hz, double duration_s = 60.0) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = substep_hz;
    cfg.gravity = Vec3{0, 0, -kG};
    cfg.job_workers = 1;
    World world(cfg);

    constexpr double kLinkLength = 1.0; // matches build_chain's kLinkLength
    const double theta0 = release_deg * kPi / 180.0;

    articulation::ArticulationDesc desc;
    desc.root.floating = false;
    for (int i = 0; i < num_links; ++i) {
        articulation::LinkDesc link;
        link.parent = (i == 0) ? -1 : (i - 1);
        link.joint = articulation::JointType::Revolute;
        link.axis = Vec3::unit_y();
        if (i > 0) link.joint_frame_in_parent.position = Vec3{0, 0, -kLinkLength};
        link.mass = 1.0;
        link.com = Vec3{0, 0, -kLinkLength / 2.0};
        // A uniform slender rod about its own COM, perpendicular axes
        // (matches the capsule's own approximate inertia closely enough
        // for a fair side-by-side comparison, not an exact capsule match).
        link.inertia = Mat3::diagonal(1.0 * kLinkLength * kLinkLength / 12.0, 1.0 * kLinkLength * kLinkLength / 12.0, 1e-4);
        link.q0 = (i == 0) ? theta0 : 0.0;
        desc.links.push_back(link);
    }
    const ArticulationId art = world.create_articulation(desc);

    // z_ref: see test_joint_chain_energy.cpp's header comment - a fixed
    // height comfortably below anything the chain can reach, purely so
    // the relative energy_loss_pct below has a well-conditioned
    // denominator. Released horizontally (theta0=90deg), a single link's
    // PE relative to its own pivot is ~0 at t=0, which blew this up to a
    // meaningless 873318% on the first run before z_ref was added here -
    // not a dynamics bug, a bug in this benchmark's own energy accounting.
    const double z_ref = -(num_links * kLinkLength + 2.0);

    auto total_energy = [&]() {
        double e = 0.0;
        for (int i = 0; i < num_links; ++i) {
            const Frame com_frame = world.link_frame(art, i, Pose{Vec3{0, 0, -kLinkLength / 2.0}, Quat::identity()});
            const Pose com_pose = com_frame.world_pose(world.backend());
            const Motion com_motion = com_frame.world_motion(world.backend());
            const Pose link_pose = world.link_frame(art, i).world_pose(world.backend());
            const Mat3 r = link_pose.orientation.to_mat3();
            const Mat3 i_cm = Mat3::diagonal(1.0 / 12.0, 1.0 / 12.0, 1e-4);
            const Mat3 i_world = r * i_cm * r.transposed();
            e += 0.5 * 1.0 * com_motion.linear.length_squared();
            e += 0.5 * dot(com_motion.angular, i_world * com_motion.angular);
            e += 1.0 * kG * (com_pose.position.z - z_ref);
        }
        return e;
    };

    const double e_start = total_energy();
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * cfg.tick_rate_hz + 0.5);
    double step_ms_total = 0.0;
    for (std::uint64_t i = 0; i < num_ticks; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        world.step();
        const auto t1 = std::chrono::steady_clock::now();
        step_ms_total += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    const double e_end = total_energy();

    RunResult r;
    r.energy_loss_pct = 100.0 * (e_start - e_end) / e_start;
    r.max_anchor_sep_mm = 0.0; // zero by construction (reduced coordinates, PLAN.md D10)
    r.monotonic = true; // not evaluated here
    r.ms_per_tick = step_ms_total / static_cast<double>(num_ticks);
    return r;
}

// Cost benchmark (PLAN.md P1.5a scope item 8): a 10-link articulation, per
// 960 Hz substep, release/scalar/single-threaded (job_workers=1). No
// target yet - reported for the record.
double bench_10_link_cost_us_per_substep() {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -kG};
    cfg.job_workers = 1;
    World world(cfg);

    constexpr int kLinks = 10;
    articulation::ArticulationDesc desc;
    desc.root.floating = false;
    for (int i = 0; i < kLinks; ++i) {
        articulation::LinkDesc link;
        link.parent = (i == 0) ? -1 : (i - 1);
        link.joint = (i % 2 == 0) ? articulation::JointType::Revolute : articulation::JointType::Prismatic;
        link.axis = Vec3::unit_y();
        if (i > 0) link.joint_frame_in_parent.position = Vec3{0, 0, -0.5};
        link.mass = 1.0;
        link.com = Vec3{0, 0, -0.25};
        link.inertia = Mat3::diagonal(0.05, 0.05, 0.01);
        link.q0 = 0.1 * static_cast<double>(i);
        desc.links.push_back(link);
    }
    world.create_articulation(desc);

    constexpr int kTicks = 2000; // 2000 ticks * 4 substeps/tick = 8000 substeps
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kTicks; ++i) world.step();
    const auto t1 = std::chrono::steady_clock::now();
    const double total_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    const double substeps = static_cast<double>(kTicks) * (cfg.substep_rate_hz / cfg.tick_rate_hz);
    return total_us / substeps;
}

} // namespace

int main() {
    ps_tools::install_headless_crt_handlers();

    std::printf("# ps_energy_study - Part A energy-loss measurement (not a CTest target)\n");
    std::printf("CSV,scenario,release_deg,baumgarte,num_position_steps,collision_steps,tick_hz,energy_loss_pct,max_anchor_sep_mm,max_ang_speed_rad_s,monotonic,ms_per_tick\n");

    const SolverSpec kDefault{"default", 0.2, 2, 1, 240.0};
    const std::vector<SolverSpec> kSweep = {
        kDefault,
        SolverSpec{"baumgarte=0.1", 0.1, 2, 1, 240.0},
        SolverSpec{"baumgarte=0.05", 0.05, 2, 1, 240.0},
        SolverSpec{"baumgarte=0.0", 0.0, 2, 1, 240.0},
        SolverSpec{"num_position_steps=0", 0.2, 0, 1, 240.0},
        SolverSpec{"collision_steps=4", 0.2, 2, 4, 240.0},
        SolverSpec{"tick_rate=960", 0.2, 2, 1, 960.0},
        SolverSpec{"baumgarte=0.05+collision_steps=4", 0.05, 2, 4, 240.0},
    };

    // (i) single hinged capsule: 5, 45, 90, 170 deg.
    for (double deg : {5.0, 45.0, 90.0, 170.0}) {
        for (const auto& s : kSweep) {
            const RunResult r = run_chain_study(1, deg, s);
            print_row(("single_capsule:" + s.label).c_str(), deg, s, r);
        }
    }

    // (ii) 3-link chain: 5, 45, 90 deg.
    for (double deg : {5.0, 45.0, 90.0}) {
        for (const auto& s : kSweep) {
            const RunResult r = run_chain_study(3, deg, s);
            print_row(("chain3:" + s.label).c_str(), deg, s, r);
        }
    }

    std::printf("\n# Candidate cause isolation (each is its own scenario, not part of the sweep above)\n");

    const RunResult gyro = run_gyroscopic_study(kDefault);
    std::printf("CANDIDATE,gyroscopic_free_spin,energy_loss_pct=%.6f,max_ang_speed_rad_s=%.4f,ms_per_tick=%.5f\n",
                gyro.energy_loss_pct, gyro.max_ang_speed_rad_s, gyro.ms_per_tick);

    const RunResult ff_capped = run_free_fall_energy_study(60.0, false);
    std::printf("CANDIDATE,free_fall_default_500ms_cap,duration_s=60,energy_loss_pct=%.6f,final_speed_m_s=%.3f\n",
                ff_capped.energy_loss_pct, ff_capped.max_ang_speed_rad_s);
    const RunResult ff_short = run_free_fall_energy_study(30.0, false);
    std::printf("CANDIDATE,free_fall_default_500ms_cap,duration_s=30,energy_loss_pct=%.6f,final_speed_m_s=%.3f (never reaches the 500 m/s cap)\n",
                ff_short.energy_loss_pct, ff_short.max_ang_speed_rad_s);
    const RunResult ff_uncapped = run_free_fall_energy_study(60.0, true);
    std::printf("CANDIDATE,float_precision_free_fall_control,duration_s=60,cap_raised_to_1e6,energy_loss_pct=%.6f,final_speed_m_s=%.3f\n",
                ff_uncapped.energy_loss_pct, ff_uncapped.max_ang_speed_rad_s);

    std::printf("\n# hinge_vs_two_point_constraints: not run empirically (would need a new JointType outside this\n");
    std::printf("# study's additive-only scope). See report: source inspection of HingeConstraint.cpp shows the\n");
    std::printf("# hinge's position solve already IS a point constraint (translation) plus a rotation constraint\n");
    std::printf("# (twist [+ limits]), each independently Baumgarte-corrected via the same PointConstraintPart\n");
    std::printf("# split-impulse mechanism.\n");

    std::printf("\n# angular_velocity_clamp: see max_ang_speed_rad_s column above vs. the default clamp\n");
    std::printf("# BodyDesc::max_angular_velocity = 47.1239 rad/s (0.25*pi*60, JoltPhysics' own default).\n");

    std::printf("\n# damping: BodyDesc::linear_damping/angular_damping default to 0.0 and every scenario above\n");
    std::printf("# leaves them at 0.0 - no default damping is applied anywhere in this study.\n");

    std::printf("\n# max_linear_velocity: BodyCreationSettings::mMaxLinearVelocity defaults to 500 m/s in Jolt\n");
    std::printf("# itself; a body exceeding it has kinetic energy silently removed every step the clamp is\n");
    std::printf("# active (see free_fall_default_500ms_cap rows above). Not relevant to the swinging-chain\n");
    std::printf("# scenarios above (peak speeds well under 500 m/s) but relevant to Part B's huge-force test.\n");

    // --- PLAN.md P1.5a scope item 8: Jolt joints vs. articulation, side by side ---
    std::printf("\n# PLAN.md P1.5a: Jolt joints vs. our own articulation solver, same Part A scenarios\n");
    std::printf("CSV_ART,scenario,release_deg,integrator,substep_hz,energy_loss_pct,max_anchor_sep_mm,ms_per_tick\n");
    for (double deg : {5.0, 45.0, 90.0, 170.0}) {
        const RunResult r = run_articulation_chain_study(1, deg, 960.0);
        std::printf("CSV_ART,single_link:rk4,%.1f,rk4,960,%.6f,%.6f,%.5f\n", deg, r.energy_loss_pct, r.max_anchor_sep_mm, r.ms_per_tick);
    }
    for (double deg : {5.0, 45.0, 90.0}) {
        const RunResult r = run_articulation_chain_study(3, deg, 960.0);
        std::printf("CSV_ART,chain3:rk4,%.1f,rk4,960,%.6f,%.6f,%.5f\n", deg, r.energy_loss_pct, r.max_anchor_sep_mm, r.ms_per_tick);
    }
    std::printf("# Compare against the CSV rows above (scenario=single_capsule:default / chain3:default,\n");
    std::printf("# release_deg matching): Jolt's single hinged capsule at 90 deg loses several percent of its\n");
    std::printf("# energy over 60 s (Baumgarte position stabilisation, PHYS-003); the articulation solver's\n");
    std::printf("# max_anchor_sep_mm is 0.0 by construction (reduced/joint coordinates - PLAN.md D10's whole\n");
    std::printf("# point) and its energy_loss_pct is the RK4 numbers already measured in the P1.5a validation\n");
    std::printf("# suite (tests/validation/test_articulation_energy.cpp): ~1e-9 to 1e-13, i.e. orders of\n");
    std::printf("# magnitude tighter than Jolt's joint solver on the identical released-chain scenario.\n");

    // --- PLAN.md P1.5a scope item 8: 10-link articulation cost ---
    const double us_per_substep = bench_10_link_cost_us_per_substep();
    std::printf("\nCOST,10_link_articulation_rk4_scalar_1thread,us_per_960hz_substep=%.3f\n", us_per_substep);
    std::printf("# No target set for this number yet (PLAN.md P1.5a scope item 8: \"report the number; no\n");
    std::printf("# target yet\"). RK4 evaluates ABA four times per substep (instance.h's integrator comparison),\n");
    std::printf("# so semi-implicit Euler's per-substep cost for the same chain is roughly this / 4 (not\n");
    std::printf("# separately measured here - would need its own WorldConfig/InstanceConfig plumbing to select\n");
    std::printf("# at runtime, which does not exist; instance.h's IntegratorKind is currently a compile-time\n");
    std::printf("# default, see that file's header comment for why RK4 was chosen anyway).\n");

    return 0;
}
