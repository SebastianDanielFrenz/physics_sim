#include "aero_io.h"
#include "ps/io/aero_coefficient_map_io.h"
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
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
namespace {
void bind_coefficient_map(const json& ref,const std::string& base_dir,aero::Desc& d) {
        keys(ref,{"file","scope","ground_probe","wing_surface"});
        const auto path=std::filesystem::path(base_dir)/ref.at("file").get<std::string>();
        if(std::filesystem::file_size(path)>256*1024*1024)throw std::invalid_argument("coefficient file too large");
        std::ifstream stream(path);if(!stream)throw std::invalid_argument("cannot read coefficient map");
        std::vector<std::set<std::string>> object_keys;
        auto cb=[&](int,json::parse_event_t e,json& parsed) {
            if(e==json::parse_event_t::object_start)object_keys.emplace_back();
            if(e==json::parse_event_t::key&&!object_keys.back().insert(parsed.get<std::string>()).second)throw std::invalid_argument("duplicate coefficient JSON key");
            if(e==json::parse_event_t::object_end)object_keys.pop_back();return true;
        };
        const auto m=json::parse(stream,cb);
        keys(m,{"schema","frame","units","reference","provenance","axes","coefficients"});
        if(m.at("schema")!="physics_sim.aero_coefficients/1"||m.at("frame")!="ISO_BODY_X_FORWARD_Y_LEFT_Z_UP"||m.at("units")!="SI_DEGREES_DIMENSIONLESS_COEFFICIENTS")throw std::invalid_argument("unsupported coefficient schema/frame/units");
        aero::CoefficientData data;
        const auto& reference=m.at("reference");keys(reference,{"area_m2","lengths_m","point_local_m"});
        data.area_m2=reference.at("area_m2").get<real>();data.lengths_m=vec(reference,"lengths_m",{});data.reference_point_local=vec(reference,"point_local_m",{});
        if(!reference.contains("point_local_m")||!reference.contains("lengths_m"))throw std::invalid_argument("missing coefficient reference dimensions");
        const auto& prov=m.at("provenance");keys(prov,{"geometry_sha256","solver","source"});
        data.geometry_sha256=prov.at("geometry_sha256").get<std::string>();data.solver=prov.at("solver").get<std::string>();data.source=prov.at("source").get<std::string>();
        const auto& axes=m.at("axes");keys(axes,{"yaw_deg","pitch_deg","speed_m_s","ride_height_m","wing_offset_deg"});
        const char* names[]={"yaw_deg","pitch_deg","speed_m_s","ride_height_m","wing_offset_deg"};
        for(std::size_t i=0;i<5;++i) {
            if(i<3||axes.contains(names[i])) {data.axes[i]=axes.at(names[i]).get<std::vector<real>>();if(data.axes[i].empty())throw std::invalid_argument("empty coefficient axis");}
        }
        const auto& rows=m.at("coefficients");if(!rows.is_array()||rows.size()>1000000)throw std::invalid_argument("invalid coefficient rows");
        for(const auto& row:rows)if(!row.is_array()||row.size()!=6)throw std::invalid_argument("coefficient rows require six entries");
        data.rows=rows.get<std::vector<std::array<real,6>>>();
        d.coefficient_map.table=aero::CoefficientMap::create(std::move(data));
        const auto scope=ref.at("scope").get<std::string>();
        if(scope=="replace_body")d.coefficient_map.scope=aero::MapScope::ReplaceBody;
        else if(scope=="replace_passive_aero")d.coefficient_map.scope=aero::MapScope::ReplacePassiveAero;
        else throw std::invalid_argument("invalid coefficient replacement scope");
        if(ref.contains("ground_probe")) {
            const auto& g=ref.at("ground_probe");keys(g,{"position_local_m","direction_local","range_m","missing"});
            if(g.at("missing")!="authored_fallback")throw std::invalid_argument("ground map requires authored_fallback");
            d.coefficient_map.ground_position_local=vec(g,"position_local_m",{});
            d.coefficient_map.ground_probe_local=vec(g,"direction_local",{0,0,-1});
            d.coefficient_map.ground_range_m=g.at("range_m").get<real>();
        }
        if(ref.contains("wing_surface")) {
            auto name=ref.at("wing_surface").get<std::string>();
            auto it=std::find_if(d.surfaces.begin(),d.surfaces.end(),[&](const auto& s){return s.name==name;});
            if(it==d.surfaces.end())throw std::invalid_argument("coefficient wing_surface not found");
            d.coefficient_map.wing_surface=static_cast<std::size_t>(it-d.surfaces.begin());
        } else if(!d.coefficient_map.table->data().axes[4].empty()) throw std::invalid_argument("wing map requires wing_surface");
        if(d.coefficient_map.table->data().axes[3].empty()==ref.contains("ground_probe"))throw std::invalid_argument("ground_probe must exist exactly for ride-height maps");
        if(d.coefficient_map.table->data().axes[4].empty()&&ref.contains("wing_surface"))throw std::invalid_argument("wing_surface without wing axis");
}
}
aero::Desc read_aero(const json& j,const std::string& base_dir) {
    keys(j,{"drag_area_m2","body","surfaces","fans","wake","coefficient_map"}); aero::Desc d;
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
    if(j.contains("coefficient_map")) bind_coefficient_map(j.at("coefficient_map"),base_dir,d);
    aero::validate(d); return d;
}
void load_aero_coefficient_map_reference(const std::string& selection_path,aero::Desc& authored) {
    if(std::filesystem::file_size(selection_path)>1024*1024)throw std::invalid_argument("coefficient selection too large");
    std::ifstream stream(selection_path);if(!stream)throw std::invalid_argument("cannot read coefficient selection");
    std::vector<std::set<std::string>> object_keys;
    auto cb=[&](int,json::parse_event_t e,json& parsed) {
        if(e==json::parse_event_t::object_start)object_keys.emplace_back();
        if(e==json::parse_event_t::key&&!object_keys.back().insert(parsed.get<std::string>()).second)throw std::invalid_argument("duplicate coefficient selection key");
        if(e==json::parse_event_t::object_end)object_keys.pop_back();return true;
    };
    auto j=json::parse(stream,cb);keys(j,{"enabled","coefficient_map"});
    auto candidate=authored;candidate.coefficient_map={};
    if(j.at("enabled").get<bool>())bind_coefficient_map(j.at("coefficient_map"),std::filesystem::path(selection_path).parent_path().string(),candidate);
    aero::validate(candidate);authored=std::move(candidate);
}
} // namespace ps::io
