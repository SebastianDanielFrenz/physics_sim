// core/src/articulation/contact.cpp — see contact.h for the two strategies'
// shapes; this file has the actual force law and the ProxyBody bookkeeping.
//
// Compliant normal force: Hunt & Crossley, "Coefficient of Restitution
// Interpreted as Damping in Vibroimpact", ASME J. Applied Mechanics 42(2),
// 1975 - a nonlinear viscoelastic model Fn = k*x^n + b*x^n*xdot (x =
// penetration, xdot = closing speed), clamped to Fn >= 0 so the contact
// never pulls (unlike a linear Kelvin-Voigt model, which can go negative
// and "stick" the surfaces together at the point of maximum penetration).
// n = 1.5 (Hertzian contact exponent) is this file's default, matching
// Hunt-Crossley's own choice for a sphere-like local contact geometry.
//
// Compliant friction: a velocity-regularised Coulomb law,
// Ft = -mu*Fn * vt/(|vt|+eps), which is 0 at vt=0 (no discontinuity to
// numerically stick/slip-chatter on) and -> the ordinary Coulomb law
// mu*Fn opposing relative tangential velocity as |vt| grows past eps - the
// same style of smoothing Drumwright & Shell, "Extending Open Dynamics
// Engine for Robotics Simulation", SIMPACT 2010, and Chrono's DEM-P
// contact model use (a tanh-based regulariser is equivalent in shape; the
// rational form here needs no transcendental call, which PLAN.md D7 avoids
// entirely off the platform libm anyway).

#include "contact.h"

#include "ps/math/transcendental.h"
#include "system.h"

#include <algorithm>
#include <cmath>

namespace ps::articulation {

namespace {
Motion shape_point_motion(const Pose& link_pose, const Motion& link_motion, const Pose& shape_offset) {
    const Vec3 world_offset = link_pose.transform_direction(shape_offset.position);
    return Motion{link_motion.velocity_at(world_offset), link_motion.angular};
}
} // namespace

ContactCoupling::ContactCoupling(const ArticulationDesc& desc, const Instance& inst, IRigidBackend& backend)
    : strategy_(desc.contact_strategy), backend_(backend) {
    for (std::size_t i = 0; i < desc.links.size(); ++i) {
        const LinkDesc& link = desc.links[i];
        if (!link.contact.enabled) continue;

        LinkContact lc;
        lc.link_index = static_cast<int>(i);
        lc.shape = link.contact.shape;
        lc.shape_offset = link.contact.shape_offset;
        lc.material = link.contact.material;
        lc.link_mass = link.mass;

        if (strategy_ == ContactStrategy::ProxyBody) {
            const Pose link_pose = inst.link_world_pose(lc.link_index);
            const Motion link_motion = inst.link_world_motion(lc.link_index);
            const Motion shape_motion = shape_point_motion(link_pose, link_motion, lc.shape_offset);

            BodyDesc bd;
            bd.shape = lc.shape;
            bd.motion = BodyMotionType::Dynamic;
            bd.mass = std::max(real(1e-6), link.mass * lc.material.proxy_mass_scale);
            bd.pose = link_pose * lc.shape_offset;
            bd.velocity = shape_motion;
            bd.friction = lc.material.friction;
            bd.restitution = 0.0; // no bounce double-counting - the articulation's own dynamics supply the "bounce"
            // Gravity IS enabled on the proxy, even though the link already
            // carries its own weight through ABA (instance.cpp's gravity.h) -
            // this looks like double-counting but is required for the
            // impulse-readback to work at all (root cause of the ~15 cm
            // settle error this fixes - see post_backend_step's comment and
            // out/handover_p15b.md): a resting rigid body's contact normal
            // force is a STEADY-STATE holding force, and Jolt's velocity
            // solver only produces a comparably large impulse in response to
            // a comparably large INCOMING velocity each step - normally
            // supplied by gravity being integrated into the body every step.
            // With gravity disabled, a proxy sitting at rest (near-zero
            // commanded velocity every tick, since the link itself is
            // already near-equilibrium) hands Jolt nothing to resist, so the
            // velocity solver contributes ~nothing and the ENTIRE holding
            // force has to come from the much weaker penetration/baumgarte
            // position corrector instead - which only grows to mg once the
            // proxy has sunk deep enough (measured/derived: penetration ~=
            // mass*|gravity|*dt / (baumgarte*mass) + penetration_slop, ~10-12
            // cm at this rig's defaults - matches the observed ~15 cm within
            // the approximation). Enabling gravity here lets the proxy rest
            // exactly like any other Jolt dynamic body (see
            // test_resting_contact.cpp: plain Jolt bodies settle sub-cm) -
            // post_backend_step then subtracts the proxy's own known gravity
            // contribution back out of the readback so the wrench fed back
            // to the link is the CONTACT reaction alone, not gravity+contact.
            bd.gravity_enabled = true;
            bd.allow_sleep = false;
            // Impact scenarios (PLAN.md P1.5b measurement 5: 5 and 20 m/s)
            // need this - a Discrete proxy at 20 m/s against a thin static
            // mesh can tunnel exactly like any other fast Discrete body
            // (i_rigid_backend.h's MotionQuality doc comment).
            bd.motion_quality = MotionQuality::LinearCast;

            lc.proxy_body = backend.create_body(bd);
            lc.proxy_mass = bd.mass;
            lc.last_commanded_motion = shape_motion;
        }

        links_.push_back(std::move(lc));
    }
}

ContactCoupling::~ContactCoupling() {
    if (strategy_ != ContactStrategy::ProxyBody) return;
    for (auto& lc : links_) {
        if (lc.proxy_body.valid()) backend_.destroy_body(lc.proxy_body);
    }
}

void ContactCoupling::evaluate_substep_contacts(const Instance& inst, IRigidBackend& backend, real substep_dt, real tick_dt,
                                                 std::vector<Wrench>& out_wrench, bool apply_reaction_to_other) {
    // Only Compliant ever has a link with contact enabled AND is wired as
    // an IContactSubstepSource (World::create_articulation only calls
    // Instance::set_contact_source for Compliant) - this is never actually
    // invoked for ProxyBody/None, but the guard keeps this function correct
    // in isolation too.
    if (strategy_ != ContactStrategy::Compliant) return;

    for (auto& lc : links_) {
        const Pose link_pose = inst.link_world_pose(lc.link_index);
        const Motion link_motion = inst.link_world_motion(lc.link_index);
        const Pose shape_pose = link_pose * lc.shape_offset;

        backend.collide_shape_into(lc.shape, shape_pose, lc.hits_scratch);
        for (const ShapeContactHit& hit : lc.hits_scratch) {
            if (hit.penetration <= real(0.0)) continue;

            const Pose other_pose = backend.get_pose(hit.body);
            const Motion other_motion = backend.get_motion(hit.body);
            const Vec3 v_link = link_motion.velocity_at(hit.point - link_pose.position);
            const Vec3 v_other = other_motion.velocity_at(hit.point - other_pose.position);
            const Vec3 v_rel = v_link - v_other;
            const real vn = dot(v_rel, hit.normal);
            const Vec3 vt_vec = v_rel - hit.normal * vn;
            const real vt = vt_vec.length();

            // Hunt-Crossley normal force (see this file's header comment):
            // x^n via ps::math::pow (PLAN.md D7 - never the platform
            // libm), clamped so the contact never pulls.
            const real xn = math::pow(hit.penetration, lc.material.stiffness_exponent);
            const real closing_speed = std::max(real(0.0), -vn);
            real fn_mag = std::max(real(0.0), lc.material.stiffness * xn + lc.material.damping * xn * closing_speed);

            // PHYS-011: an earlier "energy-safety clamp" used to live here -
            // capping the impulse at 2*link_mass*closing_speed (the elastic
            // bound), using the INSTANT's own closing_speed. Removed: it was
            // never the fix for the real energy-gain bug (the handover
            // already reported it "did not fix" that), and instrumented
            // tracing here during this fix showed it was actively
            // counter-productive - closing_speed goes to ~0 at the point of
            // MAXIMUM penetration (the turning point of the bounce, where
            // the restoring force needs to be LARGEST), so this clamp forced
            // fn_mag down to near-zero at exactly the moment the spring
            // should be pushing hardest, then let it act at full strength
            // everywhere else - a nonphysical distortion of the force
            // profile, not a genuine elastic-bound safety valve (a real one
            // would bound the TOTAL impulse over the whole contact event
            // against the IMPACT closing speed, not re-evaluate against the
            // current, by-then-much-smaller closing speed every substep).
            // The actual fix for the energy-gain bug is evaluating this
            // force at each RK4 stage's own state (see contact_source.h's
            // header comment and instance.cpp's evaluate_stage_ext_force_into) -
            // once that was in place, this clamp's only remaining effect was
            // to suppress the correct restoring force near peak penetration,
            // which is what let each bounce come out very slightly
            // super-elastic instead of properly dissipative.

            Vec3 ft_vec = Vec3::zero();
            if (vt > real(1e-9)) {
                const real ft_mag = lc.material.friction * fn_mag * (vt / (vt + lc.material.friction_regularization_speed));
                ft_vec = vt_vec * (-ft_mag / vt);
            }

            const Vec3 force = hit.normal * fn_mag + ft_vec;
            const Vec3 tau_link = cross(hit.point - link_pose.position, force);
            const int body_index = inst.body_index_for_link(lc.link_index);
            if (body_index >= 0) out_wrench[static_cast<std::size_t>(body_index)] += Wrench{force, tau_link};

            // Reaction on the other body (PLAN.md P1.5b: "reaction forces
            // go to the other body via the backend when it is dynamic").
            // apply_wrench accumulates once per backend.step() call
            // (i_rigid_backend.h's own doc comment), not once per substep,
            // so each of this tick's num_substeps calls contributes
            // 1/num_substeps of this instant's force - by the time
            // backend.step() actually runs, the accumulated total is this
            // tick's TIME-AVERAGE reaction force, which is the physically
            // correct quantity for a single discrete backend step covering
            // the whole tick. A zero/negative mass reads as static or
            // kinematic (get_mass's own doc comment) - neither should be
            // pushed.
            // apply_reaction_to_other: only the k1 (substep-start) RK4 stage
            // may call this (contact_source.h's header comment) - k2/k3/k4
            // recompute the LINK's own contact force at their offset states
            // (the actual bug fix) but must not ALSO re-drive the other
            // body's accumulated wrench, or it would receive 4x the correct
            // reaction every substep.
            if (apply_reaction_to_other && backend.get_mass(hit.body) > real(0.0)) {
                const Vec3 tau_other = cross(hit.point - other_pose.position, -force);
                backend.apply_wrench(hit.body, Wrench{-force, tau_other} * (substep_dt / tick_dt));
            }
        }
    }
}

void ContactCoupling::pre_backend_step(const Instance& inst, IRigidBackend& backend) {
    if (strategy_ != ContactStrategy::ProxyBody) return;

    for (auto& lc : links_) {
        const Pose link_pose = inst.link_world_pose(lc.link_index);
        const Motion link_motion = inst.link_world_motion(lc.link_index);
        const Pose shape_pose = link_pose * lc.shape_offset;
        const Motion shape_motion = shape_point_motion(link_pose, link_motion, lc.shape_offset);

        backend.set_pose(lc.proxy_body, shape_pose);
        backend.set_motion(lc.proxy_body, shape_motion);
        lc.last_commanded_motion = shape_motion;
    }
}

void ContactCoupling::post_backend_step(System& sys, ArticulationId id, IRigidBackend& backend, real dt, const Vec3& gravity) {
    if (strategy_ != ContactStrategy::ProxyBody) return; // Compliant already fully applied itself above

    for (auto& lc : links_) {
        const Motion after = backend.get_motion(lc.proxy_body);
        const Vec3 dv_linear = after.linear - lc.last_commanded_motion.linear;
        const Vec3 dv_angular = after.angular - lc.last_commanded_motion.angular;

        // The proxy now has real gravity (see the constructor's comment on
        // why) plus contact, so dv_linear reflects BOTH over this step -
        // impulse/dt = mass*dv_linear/dt is the TOTAL force the proxy felt,
        // gravity included. Subtract the proxy's own known gravity
        // contribution (mass*gravity) back out so the wrench fed back to
        // the link is the CONTACT reaction alone - the link already gets
        // its own gravity separately through ABA (instance.cpp), so leaving
        // gravity in here would double-count it. This is also exactly what
        // makes a resting equilibrium readable at all: at rest, dv_linear is
        // itself close to zero (gravity and contact cancel inside the
        // proxy's own step, same as any ordinary resting Jolt body) - the
        // subtraction is what turns that near-zero total back into the
        // correct near-mg upward contact force. Angular has no such
        // subtraction (uniform gravity applied at the COM has zero torque
        // about it).
        //
        // I_world*dw (angular) uses inertia_world barely changing over one
        // 240 Hz tick, so this first-order approximation is accurate to
        // O(dt); the proxy is rebuilt from the real link state every tick
        // anyway, so no error accumulates in the proxy itself.
        const Mat3 inertia_world = backend.get_inertia_world(lc.proxy_body);
        const Vec3 force = dv_linear * (lc.proxy_mass / dt) - gravity * lc.proxy_mass;
        const Wrench wrench{force, (inertia_world * dv_angular) * (real(1.0) / dt)};

        const Pose proxy_pose = backend.get_pose(lc.proxy_body);
        sys.accumulate_external_wrench(ArticulationLinkRef{id.index, lc.link_index}, proxy_pose, wrench);
    }
}

} // namespace ps::articulation
