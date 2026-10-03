#include "aero_io.h"
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <algorithm>
namespace ps::io {
namespace {
using json=nlohmann::json;
void keys(const json& j,std::initializer_list<const char*> allowed) {
    if(!j.is_object()) throw std::invalid_argument("aero entries must be objects");
    for(auto it=j.begin();it!=j.end();++it) {
        if(std::find(allowed.begin(),allowed.end(),it.key())==allowed.end())
            throw std::invalid_argument("unknown aero key: "+it.key());
    }
}
Vec3 vec(const json& j,const char* key,Vec3 fallback) {
    if(!j.contains(key)) return fallback;
    const auto& v=j.at(key);
    if(!v.is_array()||v.size()!=3) throw std::invalid_argument(std::string("aero ")+key+" must be a 3-vector");
    return {v[0].get<real>(),v[1].get<real>(),v[2].get<real>()};
}
#define NUMBER(field) d.field=j.value(#field,d.field)
#define VECTOR(field) d.field=vec(j,#field,d.field)
aero::BodyDesc body(const json& j) {
    keys(j,{"position_local","drag_area_xyz_m2","moment_coeff","angular_damping","reference_area_m2","reference_length_m","side_force_slope"});
    aero::BodyDesc d;
    VECTOR(position_local); VECTOR(drag_area_xyz_m2); VECTOR(moment_coeff); VECTOR(angular_damping);
    NUMBER(reference_area_m2); NUMBER(reference_length_m); NUMBER(side_force_slope); return d;
}
aero::GroundEffectDesc ground(const json& j) {
    keys(j,{"probe_local","range_m","gain"}); aero::GroundEffectDesc d;
    VECTOR(probe_local); NUMBER(range_m); NUMBER(gain); return d;
}
aero::SurfaceDesc surface(const json& j) {
    keys(j,{"name","position_local","chord_local","normal_local","area_m2","cl0","lift_slope_per_rad","stall_angle_rad","cd0","induced_drag","stalled_drag","rest_angle_rad","min_offset_rad","max_offset_rad","rate_rad_s","max_lift_m","ground"});
    aero::SurfaceDesc d; d.name=j.at("name").get<std::string>();
    VECTOR(position_local); VECTOR(chord_local); VECTOR(normal_local);
    NUMBER(area_m2); NUMBER(cl0); NUMBER(lift_slope_per_rad); NUMBER(stall_angle_rad);
    NUMBER(cd0); NUMBER(induced_drag); NUMBER(stalled_drag);
    NUMBER(rest_angle_rad); NUMBER(min_offset_rad); NUMBER(max_offset_rad); NUMBER(rate_rad_s); NUMBER(max_lift_m);
    if(j.contains("ground")) d.ground=ground(j.at("ground")); return d;
}
aero::FanDesc fan(const json& j) {
    keys(j,{"name","position_local","direction_local","area_m2","pressure_pa","seal_height_m","probe_range_m","max_power_w","rate_per_s"});
    aero::FanDesc d; d.name=j.at("name").get<std::string>();
    VECTOR(position_local); VECTOR(direction_local); NUMBER(area_m2); NUMBER(pressure_pa);
    NUMBER(seal_height_m); NUMBER(probe_range_m); NUMBER(max_power_w); NUMBER(rate_per_s); return d;
}
aero::WakeDesc wake(const json& j) {
    keys(j,{"length_m","radius_m","expansion","deficit","turbulence"}); aero::WakeDesc d;
    NUMBER(length_m); NUMBER(radius_m); NUMBER(expansion); NUMBER(deficit); NUMBER(turbulence); return d;
}
#undef NUMBER
#undef VECTOR
}
aero::Desc read_aero(const json& j) {
    keys(j,{"drag_area_m2","body","surfaces","fans","wake"}); aero::Desc d;
    d.drag_area_m2=j.value("drag_area_m2",real(0));
    if(j.contains("body")) d.body=body(j.at("body"));
    if(j.contains("wake")) d.wake=wake(j.at("wake"));
    if(j.contains("surfaces")) {
        if(!j.at("surfaces").is_array()) throw std::invalid_argument("aero surfaces must be an array");
        for(const auto& s:j.at("surfaces")) d.surfaces.push_back(surface(s));
    }
    if(j.contains("fans")) {
        if(!j.at("fans").is_array()) throw std::invalid_argument("aero fans must be an array");
        for(const auto& f:j.at("fans")) d.fans.push_back(fan(f));
    }
    aero::validate(d); return d;
}
} // namespace ps::io
