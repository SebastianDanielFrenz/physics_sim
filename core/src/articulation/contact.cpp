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
#include <cstdio>

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
            bd.gravity_enabled = false; // the link already carries its own weight through ABA (instance.cpp's gravity.h)
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
                                                 std::vector<Wrench>& out_wrench) {
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

            // Energy-safety clamp (found empirically - see
            // test_articulation_contact.cpp's history/this file's git log):
            // explicit evaluation of a stiff penalty spring at a HIGH
            // closing speed can, within a single substep, compute a force
            // whose impulse (fn_mag*substep_dt) reverses the closing
            // velocity into a separating one faster than a perfectly
            // elastic (restitution=1) bounce would - injecting energy at
            // every impact instead of removing it, observed as a 1 kg mass
            // dropped from 4.7 m gaining height on every bounce instead of
            // settling. Capping the impulse at 2*m*closing_speed (the
            // elastic bound) is a standard practical safety valve for
            // penalty-based contact at high impact speed (the alternative -
            // an implicit/backward-Euler contact solve - is not attempted
            // here). Uses the link's own mass as an effective mass, ignoring
            // the rest of the chain's inertia coupling - an approximation,
            // reasonable for the P1.5b scenarios (each contacts through one
            // end-effector link), not exact for a general articulated
            // effective mass (Featherstone's operational-space inertia).
            if (closing_speed > real(0.0)) {
                const real max_impulse = real(2.0) * lc.link_mass * closing_speed;
                fn_mag = std::min(fn_mag, max_impulse / substep_dt);
            }

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
            if (backend.get_mass(hit.body) > real(0.0)) {
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

void ContactCoupling::post_backend_step(System& sys, ArticulationId id, IRigidBackend& backend, real dt) {
    if (strategy_ != ContactStrategy::ProxyBody) return; // Compliant already fully applied itself above

    for (auto& lc : links_) {
        const Motion after = backend.get_motion(lc.proxy_body);
        const Vec3 dv_linear = after.linear - lc.last_commanded_motion.linear;
        const Vec3 dv_angular = after.angular - lc.last_commanded_motion.angular;

        // The proxy has no gravity and no other force applied to it (see
        // the constructor), so any velocity change across backend.step()
        // came from contact impulses alone - impulse = mass*dv (linear),
        // I_world*dw (angular; I_world barely changes over one 240 Hz tick,
        // so this first-order approximation is accurate to O(dt) and the
        // proxy is rebuilt from the real link state every tick anyway, so
        // no error accumulates in the proxy itself).
        const Mat3 inertia_world = backend.get_inertia_world(lc.proxy_body);
        const Wrench wrench{dv_linear * (lc.proxy_mass / dt), (inertia_world * dv_angular) * (real(1.0) / dt)};

        const Pose proxy_pose = backend.get_pose(lc.proxy_body);
        sys.accumulate_external_wrench(ArticulationLinkRef{id.index, lc.link_index}, proxy_pose, wrench);
    }
}

} // namespace ps::articulation
