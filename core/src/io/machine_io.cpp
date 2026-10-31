#include "ps/io/machine_io.h"

#include "ps/environment/json_extends.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <filesystem>
#include <initializer_list>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;

void reject_unknown(const json& j, std::initializer_list<const char*> allowed, const std::string& ctx) {
    for (auto it = j.begin(); it != j.end(); ++it) {
        if (std::none_of(allowed.begin(), allowed.end(), [&](const char* a) { return it.key() == a; })) {
            throw std::runtime_error(ctx + ": unknown field '" + it.key() + "'");
        }
    }
}

[[nodiscard]] real number(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key) || !j.at(key).is_number()) throw std::runtime_error(ctx + ": '" + key + "' is required and must be a number");
    return j.at(key).get<real>();
}

[[nodiscard]] std::string text(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key) || !j.at(key).is_string() || j.at(key).get_ref<const std::string&>().empty()) {
        throw std::runtime_error(ctx + ": '" + key + "' is required and must be a string");
    }
    return j.at(key).get<std::string>();
}

// Every block of numbers says where they come from.
[[nodiscard]] std::string tagged_source(const json& j, const std::string& ctx) {
    std::string s = text(j, "source", ctx);
    for (const char* tag : {"[C]", "[D]", "[M]", "[AS]"}) {
        if (s.rfind(tag, 0) == 0) return s;
    }
    throw std::runtime_error(ctx + ": 'source' must start with [C], [D], [M] or [AS]");
}

[[nodiscard]] const json& object(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key) || !j.at(key).is_object()) throw std::runtime_error(ctx + ": '" + key + "' is required and must be an object");
    return j.at(key);
}

[[nodiscard]] machine::DraftCoefficients read_coefficients(const json& j, const std::string& ctx) {
    reject_unknown(j, {"a", "b", "c"}, ctx);
    return machine::DraftCoefficients{number(j, "a", ctx), number(j, "b", ctx), number(j, "c", ctx)};
}

[[nodiscard]] machine::ToolDraftDesc read_draft(const json& j, const std::string& ctx) {
    reject_unknown(j, {"source", "width_unit", "tool_spacing_m", "depth_term", "primary", "secondary", "texture_factor", "moisture_factor", "compaction_gain", "lateral_ratio", "creep_speed_m_s"}, ctx);
    machine::ToolDraftDesc d;
    d.source = tagged_source(j, ctx);
    const std::string unit = text(j, "width_unit", ctx);
    if (unit != "metre" && unit != "tool") throw std::runtime_error(ctx + ": 'width_unit' must be \"metre\" or \"tool\"");
    d.unit = unit == "metre" ? machine::DraftWidthUnit::Metre : machine::DraftWidthUnit::Tool;
    if (d.unit == machine::DraftWidthUnit::Tool) {
        d.tool_spacing_m = number(j, "tool_spacing_m", ctx);
    } else if (j.contains("tool_spacing_m")) {
        throw std::runtime_error(ctx + ": 'tool_spacing_m' belongs to width_unit \"tool\"");
    }
    if (!j.contains("depth_term") || !j.at("depth_term").is_boolean()) throw std::runtime_error(ctx + ": 'depth_term' is required and must be true or false");
    d.depth_term = j.at("depth_term").get<bool>();
    d.primary = read_coefficients(object(j, "primary", ctx), ctx + ": primary");
    d.secondary = read_coefficients(object(j, "secondary", ctx), ctx + ": secondary");
    const json& texture = object(j, "texture_factor", ctx);
    reject_unknown(texture, {"fine", "medium", "coarse"}, ctx + ": texture_factor");
    d.texture_factor = {number(texture, "fine", ctx), number(texture, "medium", ctx), number(texture, "coarse", ctx)};
    const json& moisture = object(j, "moisture_factor", ctx);
    reject_unknown(moisture, {"wetness", "factor"}, ctx + ": moisture_factor");
    if (!moisture.contains("wetness") || !moisture.contains("factor")) throw std::runtime_error(ctx + ": moisture_factor needs 'wetness' and 'factor'");
    d.moisture_factor.x = moisture.at("wetness").get<std::vector<real>>();
    d.moisture_factor.y = moisture.at("factor").get<std::vector<real>>();
    d.compaction_gain = number(j, "compaction_gain", ctx);
    d.lateral_ratio = number(j, "lateral_ratio", ctx);
    d.creep_speed_m_s = number(j, "creep_speed_m_s", ctx);
    return d;
}

[[nodiscard]] machine::ToolSoilDesc read_soil(const json& j, const std::string& ctx) {
    reject_unknown(j, {"source", "suction_ratio", "bearing_n_m2", "gauge_n_m2", "hard_n_m2", "hard_friction", "damping_n_s_m2"}, ctx);
    machine::ToolSoilDesc s;
    s.source = tagged_source(j, ctx);
    s.suction_ratio = number(j, "suction_ratio", ctx);
    s.bearing_n_m2 = number(j, "bearing_n_m2", ctx);
    s.gauge_n_m2 = number(j, "gauge_n_m2", ctx);
    s.hard_n_m2 = number(j, "hard_n_m2", ctx);
    s.hard_friction = number(j, "hard_friction", ctx);
    s.damping_n_s_m2 = number(j, "damping_n_s_m2", ctx);
    return s;
}

[[nodiscard]] machine::ToolPtoDesc read_pto(const json& j, const std::string& ctx) {
    reject_unknown(j, {"source", "nominal_rpm", "inertia_kgm2", "no_load_kw", "kw_per_m", "specific_energy_mj_m3", "min_speed_fraction"}, ctx);
    machine::ToolPtoDesc p;
    p.present = true;
    p.source = tagged_source(j, ctx);
    p.nominal_rpm = number(j, "nominal_rpm", ctx);
    p.inertia_kgm2 = number(j, "inertia_kgm2", ctx);
    p.no_load_kw = number(j, "no_load_kw", ctx);
    p.kw_per_m = number(j, "kw_per_m", ctx);
    p.specific_energy_mj_m3 = number(j, "specific_energy_mj_m3", ctx);
    p.min_speed_fraction = number(j, "min_speed_fraction", ctx);
    return p;
}

[[nodiscard]] Vec3 socket(const MachineSockets& sockets, const std::string& name, const std::string& ctx) {
    const auto it = sockets.find(name);
    if (it == sockets.end()) throw std::runtime_error(ctx + ": the model has no socket '" + name + "'");
    return it->second;
}

[[nodiscard]] machine::ToolDesc read_tool(const json& j, const MachineSockets& sockets, const std::string& path) {
    if (!j.is_object()) throw std::runtime_error(path + ": every tool must be an object");
    machine::ToolDesc t;
    t.name = text(j, "name", path + ": tool");
    const std::string ctx = path + ": tool '" + t.name + "'";
    reject_unknown(j, {"name", "kind", "operation", "edge_left", "edge_right", "range", "draft", "soil", "pto"}, ctx);
    const std::string kind = j.contains("kind") ? text(j, "kind", ctx) : std::string("soil");
    if (kind != "soil" && kind != "applicator") throw std::runtime_error(ctx + ": 'kind' must be \"soil\" or \"applicator\"");
    t.kind = kind == "soil" ? machine::ToolKind::Soil : machine::ToolKind::Applicator;
    t.operation = text(j, "operation", ctx);
    t.edge_left_local = socket(sockets, text(j, "edge_left", ctx), ctx);
    t.edge_right_local = socket(sockets, text(j, "edge_right", ctx), ctx);
    t.width_m = (t.edge_left_local - t.edge_right_local).length();

    const json& range = object(j, "range", ctx);
    const std::string range_ctx = ctx + ": range";
    const bool applicator = t.kind == machine::ToolKind::Applicator;
    if (applicator) {
        for (const char* key : {"draft", "soil", "pto"}) {
            if (j.contains(key)) throw std::runtime_error(ctx + ": an applicator has no '" + key + "' block");
        }
        reject_unknown(range, {"source", "speed_min_km_h", "speed_max_km_h", "overspeed_quality_per_km_h", "quality_floor"}, range_ctx);
    } else {
        reject_unknown(range, {"source", "depth_min_m", "depth_nominal_m", "depth_max_m", "speed_min_km_h", "speed_max_km_h", "overspeed_quality_per_km_h", "quality_floor"}, range_ctx);
    }
    t.range_source = tagged_source(range, range_ctx);
    if (!applicator) {
        t.depth_min_m = number(range, "depth_min_m", range_ctx);
        t.depth_nominal_m = number(range, "depth_nominal_m", range_ctx);
        t.depth_max_m = number(range, "depth_max_m", range_ctx);
    }
    t.speed_min_km_h = number(range, "speed_min_km_h", range_ctx);
    t.speed_max_km_h = number(range, "speed_max_km_h", range_ctx);
    t.overspeed_quality_per_km_h = number(range, "overspeed_quality_per_km_h", range_ctx);
    t.quality_floor = number(range, "quality_floor", range_ctx);

    if (applicator) return t;
    t.draft = read_draft(object(j, "draft", ctx), ctx + ": draft");
    t.soil = read_soil(object(j, "soil", ctx), ctx + ": soil");
    if (j.contains("pto")) t.pto = read_pto(object(j, "pto", ctx), ctx + ": pto");
    return t;
}

} // namespace

machine::MachineDesc load_machine_json(const std::string& path, const MachineSockets& sockets) {
    const json j = load_json_with_extends(std::filesystem::path(path));
    if (j.value("format", std::string()) != "physics_sim.machine/1") throw std::runtime_error(path + ": 'format' must be \"physics_sim.machine/1\"");
    reject_unknown(j, {"format", "name", "source", "tools"}, path);
    machine::MachineDesc m;
    m.name = text(j, "name", path);
    m.source = text(j, "source", path);
    if (!j.contains("tools") || !j.at("tools").is_array()) throw std::runtime_error(path + ": 'tools' is required and must be an array");
    for (const json& t : j.at("tools")) m.tools.push_back(read_tool(t, sockets, path));
    machine::validate_machine(m);
    return m;
}

machine::ToolPowerSpec load_tool_power_json(const std::string& path) {
    const json j = load_json_with_extends(std::filesystem::path(path));
    if (j.value("format", std::string()) != "physics_sim.tool_power/1") throw std::runtime_error(path + ": 'format' must be \"physics_sim.tool_power/1\"");
    reject_unknown(j, {"format", "source", "tractive_ratio", "pto_share", "reserve_factor", "soils"}, path);
    machine::ToolPowerSpec s;
    s.source = tagged_source(j, path);
    s.tractive_ratio = number(j, "tractive_ratio", path);
    s.pto_share = number(j, "pto_share", path);
    s.reserve_factor = number(j, "reserve_factor", path);
    if (!j.contains("soils") || !j.at("soils").is_array() || j.at("soils").empty()) throw std::runtime_error(path + ": 'soils' is required and must be a list");
    for (const json& c : j.at("soils")) {
        const std::string ctx = path + ": soils";
        reject_unknown(c, {"texture", "wetness", "tilled", "compaction"}, ctx);
        machine::SoilSample soil;
        soil.workable = true;
        const std::string texture = text(c, "texture", ctx);
        if (texture == "fine") soil.texture = machine::SoilTexture::Fine;
        else if (texture == "medium") soil.texture = machine::SoilTexture::Medium;
        else if (texture == "coarse") soil.texture = machine::SoilTexture::Coarse;
        else throw std::runtime_error(ctx + ": 'texture' must be \"fine\", \"medium\" or \"coarse\"");
        soil.wetness = number(c, "wetness", ctx);
        soil.tilled = number(c, "tilled", ctx);
        soil.compaction = number(c, "compaction", ctx);
        s.soils.push_back(soil);
    }
    return s;
}

} // namespace ps::io
