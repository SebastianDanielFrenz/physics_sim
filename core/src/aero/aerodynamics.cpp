#include "ps/aero/aerodynamics.h"
#include "ps/math/transcendental.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace ps::aero {
namespace {
void check(bool ok, const char* message) { if(!ok) throw std::invalid_argument(message); }
bool finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
bool nonnegative(real v) { return std::isfinite(v)&&v>=0; }
bool positive(real v) { return std::isfinite(v)&&v>0; }
real slew(real value,real target,real rate,real dt) { return value+std::clamp(target-value,-rate*dt,rate*dt); }
real sq(real x) { return x*x; }
bool explicit_model(const Desc& d) { return d.body.reference_area_m2>0 || d.body.drag_area_xyz_m2.length_squared()>0 || !d.surfaces.empty(); }
GroundSample ground_at(std::span<const GroundSample> g,std::size_t i) {
    if(i<g.size()&&g[i].hit&&nonnegative(g[i].clearance_m)) return g[i];
    return {};
}
void add(Telemetry& t,Vec3 force,Vec3 intrinsic_torque,Vec3 arm,const Quat& rotation) {
    t.wrench_world.force+=force; t.wrench_world.torque+=cross(arm,force)+intrinsic_torque;
    const real down=-rotation.inverse().rotate(force).z;
    t.downforce_n+=down;
    if(rotation.inverse().rotate(arm).x>=0) t.front_downforce_n+=down; else t.rear_downforce_n+=down;
}
}
void validate(const Environment& e) {
    check(nonnegative(e.air_density)&&e.air_density<=100,"aero density must be finite and in [0,100] kg/m3");
    check(finite(e.wind_world),"aero wind must be finite");
}
void validate(const WakeDesc& w) {
    check(nonnegative(w.length_m)&&positive(w.radius_m)&&nonnegative(w.expansion),"invalid aero wake geometry");
    check(nonnegative(w.deficit)&&w.deficit<=.75&&nonnegative(w.turbulence)&&w.turbulence<=1,"invalid aero wake deficit/turbulence");
}
void validate(const Desc& d) {
    check(nonnegative(d.drag_area_m2),"aero drag_area_m2 must be finite and nonnegative");
    check(!(d.drag_area_m2>0&&(explicit_model(d)||d.body.position_local.length_squared()>0||d.body.moment_coeff.length_squared()>0||d.body.side_force_slope>0)),"aero legacy drag_area_m2 cannot coexist with explicit body/surfaces");
    const auto& b=d.body;
    check(finite(b.position_local)&&finite(b.drag_area_xyz_m2)&&finite(b.moment_coeff)&&finite(b.angular_damping),"aero body vectors must be finite");
    check(b.drag_area_xyz_m2.x>=0&&b.drag_area_xyz_m2.y>=0&&b.drag_area_xyz_m2.z>=0&&b.angular_damping.x>=0&&b.angular_damping.y>=0&&b.angular_damping.z>=0,"aero drag and damping must be nonnegative");
    check(nonnegative(b.reference_area_m2)&&positive(b.reference_length_m)&&nonnegative(b.side_force_slope),"invalid aero body dimensions");
    check(d.surfaces.size()<=max_surfaces&&d.fans.size()<=max_fans,"aero surface/fan count exceeds limits");
    for(std::size_t i=0;i<d.surfaces.size();++i) {
        const auto& s=d.surfaces[i];
        check(!s.name.empty()&&finite(s.position_local)&&finite(s.chord_local)&&finite(s.normal_local),"invalid aero surface name/vectors");
        for(std::size_t j=0;j<i;++j) check(s.name!=d.surfaces[j].name,"duplicate aero surface name");
        check(std::abs(s.chord_local.length()-1)<1e-6&&std::abs(s.normal_local.length()-1)<1e-6&&std::abs(dot(s.chord_local,s.normal_local))<1e-6,"aero chord/normal must be orthonormal unit vectors");
        check(positive(s.area_m2)&&std::isfinite(s.cl0)&&nonnegative(s.lift_slope_per_rad)&&positive(s.stall_angle_rad)&&s.stall_angle_rad<1.5,"invalid aero lift parameters");
        check(nonnegative(s.cd0)&&nonnegative(s.induced_drag)&&nonnegative(s.stalled_drag),"invalid aero drag coefficients");
        check(nonnegative(s.max_lift_m),"aero max_lift_m must be nonnegative");
        check(std::isfinite(s.rest_angle_rad)&&std::isfinite(s.min_offset_rad)&&std::isfinite(s.max_offset_rad)&&s.min_offset_rad<=0&&s.max_offset_rad>=0&&positive(s.rate_rad_s),"invalid aero actuator angles/rate");
        check(finite(s.ground.probe_local)&&std::abs(s.ground.probe_local.length()-1)<1e-6&&nonnegative(s.ground.range_m)&&nonnegative(s.ground.gain)&&s.ground.gain<=3,"invalid aero ground effect");
    }
    for(std::size_t i=0;i<d.fans.size();++i) {
        const auto& f=d.fans[i];
        check(!f.name.empty()&&finite(f.position_local)&&finite(f.direction_local)&&std::abs(f.direction_local.length()-1)<1e-6,"invalid aero fan vectors/name");
        for(std::size_t j=0;j<i;++j) check(f.name!=d.fans[j].name,"duplicate aero fan name");
        check(positive(f.area_m2)&&nonnegative(f.pressure_pa)&&positive(f.seal_height_m)&&positive(f.probe_range_m)&&f.probe_range_m>=f.seal_height_m&&positive(f.max_power_w)&&positive(f.rate_per_s),"invalid aero fan pressure/geometry/power/rate");
    }
    const auto& map=d.coefficient_map;
    if(map.table) {
        check(map.scope==MapScope::ReplaceBody||map.scope==MapScope::ReplacePassiveAero,"invalid coefficient scope");
        check(finite(map.ground_position_local)&&finite(map.ground_probe_local)&&std::abs(map.ground_probe_local.length()-1)<1e-6,"invalid coefficient ground probe");
        check(nonnegative(map.ground_range_m),"invalid coefficient ground range");
        check(map.table->data().axes[3].empty()||map.ground_range_m>0,"ride height map requires a ground probe");
        check(map.table->data().axes[4].empty()||map.wing_surface<d.surfaces.size(),"invalid coefficient wing surface");
    }
    validate(d.wake);
}
AirSample sample_air(const Environment& e,Vec3 p,BodyId exclude,std::span<const WakeSource> wakes,std::span<const WakeSource> extra) {
    Vec3 entrained{}; real weight=0, turbulence_complement=1;
    auto accumulate=[&](const WakeSource& w) {
        if((exclude.valid()&&w.owner==exclude)||w.desc.length_m<=0) return;
        const Vec3 v=w.velocity_world-e.wind_world; const real speed=v.length();
        if(speed<.5) return;
        const Vec3 downstream=-v/speed, delta=p-w.position_world;
        const real distance=dot(delta,downstream);
        if(distance<=0||distance>=w.desc.length_m) return;
        const real radius=w.desc.radius_m+w.desc.expansion*distance;
        const real radial=(delta-downstream*distance).length()/radius;
        if(radial>=1) return;
        // Smooth finite cone (zero at source and cone edges), decaying downstream.
        const real onset=std::min(real(1),distance/w.desc.radius_m);
        const real shape=sq(1-sq(radial))*sq(1-distance/w.desc.length_m)*onset*onset*(3-2*onset);
        const real strength=w.desc.deficit*shape;
        entrained+=v*strength; weight+=strength;
        turbulence_complement*=1-w.desc.turbulence*shape;
    };
    for(const auto& w:wakes) accumulate(w);
    for(const auto& w:extra) accumulate(w);
    // Convex bounded entrainment: overlapping wakes cannot exceed 75% of source speed.
    return {e.wind_world+entrained/std::max(real(1),weight/.75),1-turbulence_complement};
}
Telemetry step(const Desc& d,State& state,const Pose& pose,const Motion& motion,const Environment& env,real dt,
               std::span<const GroundSample> ground,std::span<const WakeSource> wakes,BodyId exclude,std::span<const WakeSource> extra) {
    Telemetry t; t.surface_count=d.surfaces.size(); t.fan_count=d.fans.size();
    const Quat& rotation=pose.orientation;
    auto relative=[&](Vec3 local) {
        const Vec3 arm=rotation.rotate(local);
        const auto air=sample_air(env,pose.position+arm,exclude,wakes,extra);
        return rotation.inverse().rotate(motion.velocity_at(arm)-air.wind_world);
    };
    for(std::size_t i=0;i<d.surfaces.size();++i) {
        const auto& s=d.surfaces[i]; const real cmd=std::clamp(state.surface_command[i],real(-1),real(1));
        const real target=cmd>=0?cmd*s.max_offset_rad:-cmd*s.min_offset_rad;
        state.offset_rad[i]=slew(state.offset_rad[i],target,s.rate_rad_s,dt);
    }
    const auto& map=d.coefficient_map;
    const auto map_ground=ground_at(ground,d.surfaces.size()+d.fans.size());
    const bool needs_ground=map.table&&!map.table->data().axes[3].empty();
    t.coefficient_map_ground_missing=needs_ground&&!map_ground.hit;
    const bool use_map=map.table&&!t.coefficient_map_ground_missing;
    t.coefficient_map_active=use_map;
    const auto& b=d.body;
    const Vec3 point=use_map?map.table->data().reference_point_local:b.position_local;
    const Vec3 bv=relative(point);
    const real speed=bv.length(), q=.5*env.air_density*speed*speed;
    t.body_relative_airflow_local=bv; t.body_airspeed_m_s=speed; t.body_dynamic_pressure_pa=q;
    t.body_turbulence=sample_air(env,pose.transform_point(point),exclude,wakes,extra).turbulence;
    Vec3 body_force{};
    Vec3 legacy_force_world{};
    if(!use_map&&!explicit_model(d)) {
        // Preserve the legacy world-space arithmetic exactly when wind/wakes are zero.
        const auto air=sample_air(env,pose.position,exclude,wakes,extra);
        const Vec3 velocity=motion.linear-air.wind_world;
        legacy_force_world=velocity*(-real(.5)*env.air_density*d.drag_area_m2*velocity.length());
        body_force=rotation.inverse().rotate(legacy_force_world);
    }
    else if(!use_map) {
        body_force={-.5*env.air_density*b.drag_area_xyz_m2.x*bv.x*std::abs(bv.x),
                    -.5*env.air_density*b.drag_area_xyz_m2.y*bv.y*std::abs(bv.y),
                    -.5*env.air_density*b.drag_area_xyz_m2.z*bv.z*std::abs(bv.z)};
        if(speed>1e-9) {
            const Vec3 dir=bv/speed;
            const Vec3 side=(Vec3::unit_y()-dir*dir.y).normalized();
            body_force+=side*(-q*b.reference_area_m2*b.side_force_slope*ps::math::atan2(bv.y,std::abs(bv.x)+1e-9));
        }
    }
    const Vec3 omega=rotation.inverse().rotate(motion.angular);
    const real damp_scale=b.reference_length_m/(2*std::max(real(1),speed));
    Vec3 body_torque{};
    if(!use_map) body_torque=(b.moment_coeff-Vec3{b.angular_damping.x*omega.x,b.angular_damping.y*omega.y,b.angular_damping.z*omega.z}*damp_scale)*(q*b.reference_area_m2*b.reference_length_m);
    if(use_map) {
        constexpr real deg=180/3.14159265358979323846;
        const real horizontal=std::hypot(bv.x,bv.y);
        t.coefficient_map_inputs={horizontal>1e-12?ps::math::atan2(bv.y,bv.x)*deg:real(0),
            speed>1e-12?ps::math::atan2(bv.z,horizontal)*deg:real(0),speed,map_ground.clearance_m,
            map.table->data().axes[4].empty()?real(0):state.offset_rad[map.wing_surface]*deg};
        const auto sample=map.table->evaluate(t.coefficient_map_inputs);
        t.coefficient_map_clamped_axes=sample.clamped_axes;t.coefficient_map_coefficients=sample.coefficients;
        const auto& c=sample.coefficients;const auto& data=map.table->data();const real scale=q*data.area_m2;
        if(scale==0) { body_force={}; body_torque={}; }
        else {
            body_force=Vec3{c[0],c[1],c[2]}*scale;
            body_torque=Vec3{c[3]*data.lengths_m.x,c[4]*data.lengths_m.y,c[5]*data.lengths_m.z}*scale;
        }
    }
    t.body_force_local=body_force; t.body_moment_local=body_torque;
    if(use_map||explicit_model(d)) add(t,rotation.rotate(body_force),rotation.rotate(body_torque),rotation.rotate(point),rotation);
    else add(t,legacy_force_world,{}, {},rotation);
    for(std::size_t i=0;i<d.surfaces.size();++i) {
        const auto& s=d.surfaces[i]; auto& st=t.surfaces[i];

        st.offset_rad=state.offset_rad[i]; st.angle_rad=s.rest_angle_rad+st.offset_rad;
        const Vec3 span=cross(s.normal_local,s.chord_local);
        const Quat incidence=Quat::from_axis_angle(span,st.angle_rad);
        const Vec3 chord=incidence.rotate(s.chord_local), normal=incidence.rotate(s.normal_local);
        st.lift_m=s.max_offset_rad>0 ? s.max_lift_m*std::clamp(st.offset_rad/s.max_offset_rad,real(0),real(1)) : real(0);
        if(use_map&&map.scope==MapScope::ReplacePassiveAero) continue;
        const Vec3 arm=rotation.rotate(s.position_local+s.normal_local*st.lift_m);
        const auto air=sample_air(env,pose.position+arm,exclude,wakes,extra);
        const Vec3 v=rotation.inverse().rotate(motion.velocity_at(arm)-air.wind_world);
        const real vlen=v.length(); st.turbulence=air.turbulence;
        const auto g=ground_at(ground,i); if(g.hit) st.clearance_m=g.clearance_m;
        if(g.hit&&s.ground.range_m>0) st.ground_multiplier=1+s.ground.gain*sq(std::max(real(0),1-g.clearance_m/s.ground.range_m));
        if(vlen<1e-9) continue;
        const Vec3 direction=v/vlen;
        st.alpha_rad=ps::math::atan2(-dot(v,normal),dot(v,chord));
        st.beta_rad=ps::math::asin(std::clamp(dot(direction,span),real(-1),real(1)));
        const real alpha=st.alpha_rad, a=std::abs(alpha);
        // Linear attached lift smoothly blends to a flat-plate sin(2 alpha) at stall.
        // Reverse flow has no camber bonus; spanwise flow suppresses lift continuously.
        const real x=std::clamp((a-s.stall_angle_rad)/(s.stall_angle_rad*.5),real(0),real(1));
        const real blend=x*x*(3-2*x);
        const real attached=s.cl0+s.lift_slope_per_rad*alpha;
        const real plate=ps::math::sin(2*alpha);
        const real span_factor=std::max(real(0),1-sq(dot(direction,span)));
        st.cl=((1-blend)*attached+blend*plate)*span_factor*st.ground_multiplier*(1-.5*air.turbulence);
        st.cd=s.cd0+s.induced_drag*sq(st.cl)+s.stalled_drag*blend*sq(ps::math::sin(alpha));
        st.dynamic_pressure_pa=.5*env.air_density*vlen*vlen;
        const Vec3 lift_dir=(normal-direction*dot(normal,direction)).normalized();
        const Vec3 f=rotation.rotate((lift_dir*st.cl-direction*st.cd)*(st.dynamic_pressure_pa*s.area_m2));
        st.wrench_world={f,cross(arm,f)};
        add(t,f,{},arm,rotation);
    }
    for(std::size_t i=0;i<d.fans.size();++i) {
        const auto& f=d.fans[i];
        state.fan_level[i]=slew(state.fan_level[i],std::clamp(state.fan_command[i],real(0),real(1)),f.rate_per_s,dt);
        const real requested=f.max_power_w*state.fan_level[i];
        const real power=dt>0?std::min(requested,std::max(real(0),state.fan_energy_j)/dt):real(0);
        const real used=power*dt; state.fan_energy_j=std::max(real(0),state.fan_energy_j-used);
        t.fan_power_w+=power; t.fan_energy_used_j+=used;
        const auto g=ground_at(ground,d.surfaces.size()+i);
        // A leaking/open skirt loses suction, but the running motor still consumes energy.
        const real seal=g.hit?sq(std::max(real(0),1-g.clearance_m/f.seal_height_m)):real(0);
        const real pressure_fraction=power/f.max_power_w;
        const Vec3 force=rotation.rotate(f.direction_local)*(f.pressure_pa*f.area_m2*seal*pressure_fraction);
        auto& ft=t.fans[i]; ft.force_world=force; ft.level=state.fan_level[i]; ft.power_w=power;
        ft.energy_used_j=used; ft.seal=seal; ft.clearance_m=g.hit?g.clearance_m:real(-1);
        add(t,force,{},rotation.rotate(f.position_local),rotation);
    }
    if(std::abs(t.downforce_n)>1e-9) t.front_balance=t.front_downforce_n/t.downforce_n;
    t.fan_energy_remaining_j=state.fan_energy_j;
    return t;
}
std::uint64_t fold_state_hash(const Desc& d,const State& s,std::uint64_t h) {
    auto fold=[&](real v) { const auto* bytes=reinterpret_cast<const unsigned char*>(&v); for(std::size_t i=0;i<sizeof(v);++i) { h^=bytes[i]; h*=1099511628211ULL; } };
    if(d.coefficient_map.table) {
        const auto& m=d.coefficient_map;auto hash=m.table->content_hash();
        for(unsigned i=0;i<8;++i){h^=(hash>>(i*8))&255;h*=1099511628211ULL;}
        fold(static_cast<real>(m.scope));fold(static_cast<real>(m.wing_surface));
        fold(m.ground_range_m);for(auto v:{m.ground_position_local,m.ground_probe_local}){fold(v.x);fold(v.y);fold(v.z);}
    }
    for(std::size_t i=0;i<d.surfaces.size();++i) fold(s.offset_rad[i]);
    for(std::size_t i=0;i<d.fans.size();++i) fold(s.fan_level[i]);
    if(!d.fans.empty()) fold(s.fan_energy_j);
    return h;
}
} // namespace ps::aero
