#include "ps/io/hitch_io.h"

#include "ps/environment/json_extends.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <initializer_list>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;

constexpr real kDeg = 3.14159265358979323846 / 180.0;

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

[[nodiscard]] machine::HitchAxis read_axis(const json& root, const char* name, const std::string& path) {
    const std::string ctx = path + ": " + name;
    if (!root.contains(name) || !root.at(name).is_object()) throw std::runtime_error(ctx + " is required and must be an object");
    const json& a = root.at(name);
    if (!a.contains("mode") || !a.at("mode").is_string()) throw std::runtime_error(ctx + ": 'mode' must be \"free\", \"limited\" or \"locked\"");
    const std::string mode = a.at("mode").get<std::string>();
    machine::HitchAxis out;
    if (mode == "free" || mode == "locked") {
        reject_unknown(a, {"mode"}, ctx);
        out.mode = mode == "free" ? machine::HitchAxisMode::Free : machine::HitchAxisMode::Locked;
    } else if (mode == "limited") {
        reject_unknown(a, {"mode", "min_deg", "max_deg"}, ctx);
        out.mode = machine::HitchAxisMode::Limited;
        out.min_rad = number(a, "min_deg", ctx) * kDeg;
        out.max_rad = number(a, "max_deg", ctx) * kDeg;
    } else {
        throw std::runtime_error(ctx + ": 'mode' must be \"free\", \"limited\" or \"locked\"");
    }
    return out;
}

} // namespace

machine::HitchClassDesc load_hitch_class_json(const std::string& path) {
    const json root = load_json_with_extends(std::filesystem::path(path));
    reject_unknown(root, {"format", "extends", "name", "source", "yaw", "pitch", "roll", "frequency_hz", "damping_ratio", "latch"}, path);
    if (!root.contains("format") || root.at("format") != "physics_sim.hitch_class/1") throw std::runtime_error(path + ": 'format' must be \"physics_sim.hitch_class/1\"");
    if (!root.contains("name") || !root.at("name").is_string()) throw std::runtime_error(path + ": 'name' is required");
    if (!root.contains("source") || !root.at("source").is_string()) throw std::runtime_error(path + ": 'source' is required");

    machine::HitchClassDesc d;
    d.name = root.at("name").get<std::string>();
    d.source = root.at("source").get<std::string>();
    d.yaw = read_axis(root, "yaw", path);
    d.pitch = read_axis(root, "pitch", path);
    d.roll = read_axis(root, "roll", path);
    d.frequency_hz = number(root, "frequency_hz", path);
    d.damping_ratio = number(root, "damping_ratio", path);

    if (!root.contains("latch") || !root.at("latch").is_object()) throw std::runtime_error(path + ": 'latch' is required and must be an object");
    const json& latch = root.at("latch");
    const std::string latch_ctx = path + ": latch";
    reject_unknown(latch, {"time_s", "max_error_m", "max_angle_deg", "max_speed_m_s"}, latch_ctx);
    d.latch_time_s = number(latch, "time_s", latch_ctx);
    d.max_latch_error_m = number(latch, "max_error_m", latch_ctx);
    d.max_latch_angle_rad = number(latch, "max_angle_deg", latch_ctx) * kDeg;
    d.max_latch_speed_m_s = number(latch, "max_speed_m_s", latch_ctx);

    machine::validate_hitch_class(d);
    return d;
}

} // namespace ps::io
