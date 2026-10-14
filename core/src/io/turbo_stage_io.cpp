// turbo_stage_io.cpp - see ps/io/turbo_stage_io.h.
#include "ps/io/turbo_stage_io.h"

#include "turbo_stage_json.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

namespace ps::io {
namespace {
using json = nlohmann::json;
namespace ml = ps::drivetrain::meanline;

// Same messages as vehicle_io.cpp's require_number / require_positive.
real require_number(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key)) throw std::runtime_error(ctx + ": '" + std::string(key) + "' is required");
    if (!j.at(key).is_number()) throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be a number");
    const real v = j.at(key).get<real>();
    if (!std::isfinite(v)) throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be finite");
    return v;
}
real require_positive(const json& j, const char* key, const std::string& ctx) {
    const real v = require_number(j, key, ctx);
    if (!(v > real(0.0))) throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be > 0, got " + std::to_string(v));
    return v;
}

json parse_text(const std::string& text, const std::string& label) {
    json j = json::parse(text, nullptr, false);
    if (j.is_discarded()) throw std::runtime_error("malformed JSON in " + label);
    return j;
}

std::string hex64(std::uint64_t v) {
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));
    return buf;
}

json compressor_json(const ml::CompressorPoint& p, ml::ShockModel model) {
    json o;
    o["status"] = ml::to_string(p.status);
    if (p.status == ml::CompressorStatus::InletChoke) return o;
    o["inlet_mach"] = p.inlet_mach;
    if (p.status == ml::CompressorStatus::ExitChokeOrNoPositiveHead) return o;
    o["pressure_ratio"] = p.pressure_ratio;
    o["outlet_total_k"] = p.outlet_total_k;
    o["specific_work_j_kg"] = p.specific_work_j_kg;
    o["shaft_power_w"] = p.shaft_power_w;
    o["efficiency"] = p.efficiency;
    o["shock_model"] = model == ml::ShockModel::InletNormalEntropy ? "inlet_normal_entropy" : "unsupported";
    o["shock_pressure_retention"] = p.shock_pressure_retention;
    o["shock_entropy_j_kg_k"] = p.shock_entropy_j_kg_k;
    o["inlet_relative_mach"] = p.inlet_relative_mach;
    o["exit_relative_mach"] = p.exit_relative_mach;
    o["incidence_risk"] = p.incidence_risk;
    o["continuity_residual_kg_s"] = p.continuity_residual_kg_s;
    o["energy_residual_w"] = p.energy_residual_w;
    return o;
}

json turbine_json(const ml::TurbinePoint& p) {
    return json{{"status", "ok"}, {"mass_flow_kg_s", p.mass_flow_kg_s}, {"specific_work_j_kg", p.specific_work_j_kg},
                {"shaft_power_w", p.shaft_power_w}, {"specific_torque_nm_per_kg_s", p.specific_torque_nm_per_kg_s},
                {"outlet_total_k", p.outlet_total_k}, {"efficiency", p.efficiency}, {"energy_residual_w", p.energy_residual_w},
                {"choked", p.choked}};
}

json audit_json(const ml::StageAudit& a) {
    return json{{"cell_centers_checked", a.cell_centers_checked}, {"maximum_relative_error", a.maximum_relative_error},
                {"coverage_boundary_cells", a.coverage_boundary_cells}};
}
} // namespace

drivetrain::meanline::StageGeometry parse_turbo_geometry_json(const std::string& text, const std::string& label) {
    json j = parse_text(text, label);
    if (j.is_object() && j.value("format", std::string()) == "physics_sim.turbo_stage_maps/1") {
        if (!j.contains("geometry") || !j.at("geometry").is_object()) throw std::runtime_error(label + ": stage map has no geometry");
        j = j.at("geometry");
    }
    if (!j.is_object() || j.value("format", std::string()) != "physics_sim.turbo_geometry/1")
        throw std::runtime_error(label + ": expected physics_sim.turbo_geometry/1");
    ml::StageGeometry g;
    try {
        const json& c = j.at("compressor");
        const json& t = j.at("turbine");
        const auto known = [&](const json& o, std::initializer_list<const char*> keys) {
            for (const auto& item : o.items()) {
                bool ok = false;
                for (const char* k : keys) ok = ok || item.key() == k;
                if (!ok) throw std::runtime_error(label + ": unknown geometry key '" + item.key() + "'");
            }
        };
        known(c, {"inducer_mm", "exducer_mm", "hub_mm", "exit_width_mm", "blades", "exit_angle_deg", "inlet_angle_deg", "blockage",
                  "passage_loss", "incidence_loss", "diffuser_loss", "shock_model"});
        known(t, {"diameter_mm", "effective_throat_mm2", "nozzle_angle_deg", "nozzle_efficiency", "rotor_velocity_retention"});
        auto& gc = g.compressor;
        gc.inducer_mm = require_number(c, "inducer_mm", label);
        gc.exducer_mm = require_number(c, "exducer_mm", label);
        gc.hub_mm = require_number(c, "hub_mm", label);
        gc.exit_width_mm = require_number(c, "exit_width_mm", label);
        const real blades = require_number(c, "blades", label);
        if (blades != std::floor(blades) || blades < 3 || blades > 1000) throw std::invalid_argument("invalid compressor blade geometry");
        gc.blades = int(blades);
        gc.exit_angle_deg = require_number(c, "exit_angle_deg", label);
        gc.inlet_angle_deg = require_number(c, "inlet_angle_deg", label);
        gc.blockage = require_number(c, "blockage", label);
        gc.passage_loss = require_number(c, "passage_loss", label);
        gc.incidence_loss = require_number(c, "incidence_loss", label);
        gc.diffuser_loss = require_number(c, "diffuser_loss", label);
        const std::string shock = c.value("shock_model", std::string("unsupported"));
        if (shock == "inlet_normal_entropy") gc.shock_model = ml::ShockModel::InletNormalEntropy;
        else if (shock == "unsupported") gc.shock_model = ml::ShockModel::Unsupported;
        else throw std::invalid_argument("unknown compressor shock model");
        auto& gt = g.turbine;
        gt.diameter_mm = require_number(t, "diameter_mm", label);
        gt.effective_throat_mm2 = require_number(t, "effective_throat_mm2", label);
        gt.nozzle_angle_deg = require_number(t, "nozzle_angle_deg", label);
        gt.nozzle_efficiency = require_number(t, "nozzle_efficiency", label);
        gt.rotor_velocity_retention = require_number(t, "rotor_velocity_retention", label);
        if (j.contains("air")) g.air = {require_number(j.at("air"), "r", label), require_number(j.at("air"), "gamma", label)};
        else g.air = {287.05, 1.4};
        if (j.contains("exhaust")) g.exhaust = {require_number(j.at("exhaust"), "r", label), require_number(j.at("exhaust"), "gamma", label)};
        else g.exhaust = {287.05, 1.33};
        g.source = j.value("source", std::string());
        if (g.source.empty() || !j.contains("provenance") || !j.at("provenance").is_object() || j.at("provenance").empty())
            throw std::invalid_argument("source and provenance required");
    } catch (const json::exception& e) {
        throw std::runtime_error(label + ": invalid turbo geometry (" + e.what() + ")");
    }
    g.profile_json = j.dump();
    ml::validate(g);
    return g;
}

drivetrain::meanline::StageGeometry load_turbo_geometry_json(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open file: " + path);
    std::ostringstream text;
    text << in.rdbuf();
    return parse_turbo_geometry_json(text.str(), path);
}

drivetrain::meanline::Sampling parse_turbo_stage_sampling_json(const std::string& text, const std::string& label) {
    const json sj = parse_text(text, label);
    if (!sj.is_object() || sj.value("format", std::string()) != "physics_sim.turbo_stage_maps/1")
        throw std::runtime_error(label + ": expected physics_sim.turbo_stage_maps/1");
    try {
        return {sj.at("corrected_rpm").get<std::vector<real>>(), sj.at("compressor_corrected_flow_kg_s").get<std::vector<real>>(),
                sj.at("turbine_expansion_ratio").get<std::vector<real>>()};
    } catch (const json::exception& e) {
        throw std::runtime_error(label + ": invalid stage-map axes (" + e.what() + ")");
    }
}

std::string turbo_stage_maps_to_json(const drivetrain::meanline::StageMapResult& r, std::uint64_t cache_key) {
    const json profile = json::parse(r.geometry.profile_json);
    json out;
    out["format"] = "physics_sim.turbo_stage_maps/1";
    out["solver"] = ml::kSolverVersion;
    out["source"] = r.geometry.source;
    out["provenance"] = profile.at("provenance");
    out["geometry"] = profile;
    out["reference_pressure_bar"] = ml::kReferencePa / 1e5;
    out["compressor_reference_temperature_k"] = ml::kReferenceK;
    out["turbine_reference_temperature_k"] = ml::kTurbineReferenceK;
    out["corrected_rpm"] = r.axes.corrected_rpm;
    out["compressor_corrected_flow_kg_s"] = r.axes.flow_kg_s;
    out["turbine_expansion_ratio"] = r.axes.expansion_ratio;
    json comp = json::array(), turb = json::array();
    for (const auto& p : r.compressor) comp.push_back(compressor_json(p, r.geometry.compressor.shock_model));
    for (const auto& p : r.turbine) turb.push_back(turbine_json(p));
    out["compressor"] = std::move(comp);
    out["turbine"] = std::move(turb);
    const std::string shock = r.geometry.compressor.shock_model == ml::ShockModel::InletNormalEntropy ? "inlet_normal_entropy" : "unsupported";
    out["validation"] = {{"manufacturer_calibrated", false},
                         {"compressor_shock", shock + "; assumed inlet shock only; supersonic outlet/passage not resolved"},
                         {"surge", "incidence risk only; unsteady surge not resolved"},
                         {"turbine", "impulse-stage approximation; reaction/volute losses not resolved"},
                         {"coverage", "invalid/choked points are explicit; no extrapolation is certified"}};
    if (r.adaptive) {
        out["sampling"] = {{"method", "adaptive_cell_centers"}, {"target_relative_error", r.sampling.target_relative_error},
                           {"passes", r.sampling.passes}, {"budget_limited", r.sampling.budget_limited},
                           {"audit", {{"compressor", audit_json(r.sampling.compressor)}, {"turbine", audit_json(r.sampling.turbine)}}},
                           {"certified_global_error_bound", false}};
    }
    out["generator"] = {{"implementation", "ps::drivetrain::meanline (C++ port of tools/turbo_sim/meanline.py)"},
                        {"cache_key", hex64(cache_key)}};
    std::uint64_t h = 1469598103934665603ULL;
    for (unsigned char b : out.dump()) { h ^= b; h *= 1099511628211ULL; }
    out["content_hash"] = "fnv1a64:" + hex64(h);
    return out.dump();
}

void write_turbo_stage_maps_json(const std::string& path, const drivetrain::meanline::StageMapResult& r, std::uint64_t cache_key) {
    const std::string text = turbo_stage_maps_to_json(r, cache_key);
    const std::filesystem::path target(path);
    std::error_code ec;
    if (target.has_parent_path()) std::filesystem::create_directories(target.parent_path(), ec);
    std::ostringstream suffix;
    suffix << ".tmp" << std::this_thread::get_id();
    const std::filesystem::path tmp = target.string() + suffix.str();
    {
        std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
        if (!o) throw std::runtime_error("cannot write file: " + tmp.string());
        o << text;
        if (!o) throw std::runtime_error("cannot write file: " + tmp.string());
    }
    std::filesystem::rename(tmp, target, ec);
    if (ec) {
        std::filesystem::remove(tmp, ec);
        throw std::runtime_error("cannot replace file: " + path);
    }
}

std::shared_ptr<drivetrain::TurboStageMaps> turbo_stage_maps_from_json(const std::string& text, int units, const std::string& label) {
    const json sj = parse_text(text, label);
    if (!sj.is_object() || sj.value("format", std::string()) != "physics_sim.turbo_stage_maps/1")
        throw std::runtime_error(label + ": expected physics_sim.turbo_stage_maps/1");
    return detail::stage_maps_from_json(sj, units, label);
}

namespace detail {
// Moved unchanged from vehicle_io.cpp's performance_map block (2026-10-14).
std::shared_ptr<drivetrain::TurboStageMaps> stage_maps_from_json(const json& sj, int count, const std::string& stage_path) {
    auto sm = std::make_shared<drivetrain::TurboStageMaps>();
    sm->source=sj.at("source").get<std::string>();
    sm->solver=sj.at("solver").get<std::string>();
    if(sm->source.empty() || (sm->solver!="meanline-3" && sm->solver!="meanline-4") || !sj.at("provenance").is_object())
        throw std::runtime_error(stage_path+": unsupported solver or missing provenance");
    sm->reference_pa=require_positive(sj,"reference_pressure_bar",stage_path)*real(1e5);
    sm->compressor_reference_k=require_positive(sj,"compressor_reference_temperature_k",stage_path);
    sm->turbine_reference_k=require_positive(sj,"turbine_reference_temperature_k",stage_path);
    const auto& gas=sj.at("geometry").at("exhaust");
    sm->exhaust_r=require_positive(gas,"r",stage_path);
    sm->exhaust_gamma=require_positive(gas,"gamma",stage_path);
    if(!(sm->exhaust_gamma>1 && sm->exhaust_gamma<=1.67)) throw std::runtime_error(stage_path+": invalid exhaust gamma");
    const auto& air=sj.at("geometry").at("air");
    const real air_r=require_positive(air,"r",stage_path), air_gamma=require_positive(air,"gamma",stage_path);
    if(!(air_gamma>1 && air_gamma<=1.67)) throw std::runtime_error(stage_path+": invalid air gamma");
    sm->compressor_cp=air_r*air_gamma/(air_gamma-real(1));
    sm->air_kappa=(air_gamma-real(1))/air_gamma; // stage seam S0: the affinity extension's isentropic exponent
    sm->units=count;
    const auto speeds=sj.at("corrected_rpm").get<std::vector<real>>();
    const auto flows=sj.at("compressor_corrected_flow_kg_s").get<std::vector<real>>();
    const auto expansions=sj.at("turbine_expansion_ratio").get<std::vector<real>>();
    sm->compressor_pr.x=sm->compressor_work_j_kg.x=sm->turbine_flow_kg_s.x=sm->turbine_work_j_kg.x=sm->turbine_torque_nm_per_kg_s.x=speeds;
    sm->compressor_pr.y=sm->compressor_work_j_kg.y=flows;
    for(real er:expansions) {
        const real chi=sm->turbine_coordinate(er);
        sm->turbine_flow_kg_s.y.push_back(chi); sm->turbine_work_j_kg.y.push_back(chi);
        sm->turbine_torque_nm_per_kg_s.y.push_back(chi);
    }
    if(sj.at("compressor").size()!=speeds.size()*flows.size() || sj.at("turbine").size()!=speeds.size()*expansions.size())
        throw std::runtime_error(stage_path+": inconsistent map dimensions");
    for(const auto& point:sj.at("compressor")) {
        const bool valid=point.at("status").get<std::string>()=="ok";
        const real pr=valid ? point.at("pressure_ratio").get<real>():real(1);
        const real work=valid ? point.at("specific_work_j_kg").get<real>():real(0);
        if(!std::isfinite(pr) || !std::isfinite(work) || pr<1 || work<0)
            throw std::runtime_error(stage_path+": invalid compressor point");
        sm->compressor_valid.push_back(valid);
        sm->compressor_pr.values.push_back(pr); sm->compressor_work_j_kg.values.push_back(work);
    }
    for(const auto& point:sj.at("turbine")) {
        const real flow=point.at("mass_flow_kg_s").get<real>(), work=point.at("specific_work_j_kg").get<real>();
        if(!std::isfinite(flow) || !std::isfinite(work) || flow<0 || point.at("status").get<std::string>()!="ok")
            throw std::runtime_error(stage_path+": invalid turbine point");
        const real torque=point.at("specific_torque_nm_per_kg_s").get<real>();
        if(!std::isfinite(torque)) throw std::runtime_error(stage_path+": invalid turbine torque");
        sm->turbine_torque_nm_per_kg_s.values.push_back(torque);
        sm->turbine_flow_kg_s.values.push_back(flow); sm->turbine_work_j_kg.values.push_back(work);
    }
    for(const auto* table:{&sm->compressor_pr,&sm->compressor_work_j_kg,&sm->turbine_flow_kg_s,&sm->turbine_work_j_kg,&sm->turbine_torque_nm_per_kg_s})
        if(const auto error=table->validate();!error.empty()) throw std::runtime_error(stage_path+": "+error);
    if(speeds.front()<0 || flows.front()<0 || expansions.front()!=1)
        throw std::runtime_error(stage_path+": invalid map axis domain (turbine must start at expansion ratio 1)");
    return sm;
}
} // namespace detail

} // namespace ps::io
