#include "ps/io/electronics_io.h"

#include "engine_parts.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;

[[nodiscard]] bool is_assist_name(const std::string& n) {
    for (const char* a : kElectronicsAssistNames) {
        if (n == a) return true;
    }
    return false;
}

void reject_unknown(const json& j, std::initializer_list<const char*> allowed, const std::string& path) {
    for (auto it = j.begin(); it != j.end(); ++it) {
        if (std::none_of(allowed.begin(), allowed.end(), [&](const char* a) { return it.key() == a; })) {
            throw std::runtime_error(path + ": unknown electronics field '" + it.key() + "'");
        }
    }
}

[[nodiscard]] real require_positive(const json& j, const char* key, const std::string& path) {
    if (!j.contains(key) || !j.at(key).is_number()) throw std::runtime_error(path + ": '" + key + "' is required and must be a number");
    const real v = j.at(key).get<real>();
    if (!std::isfinite(v) || !(v > real(0.0))) throw std::runtime_error(path + ": '" + key + "' must be finite and > 0");
    return v;
}

} // namespace

ElectronicsPart load_electronics_json(const std::string& path) {
    const json j = parts::load_part(path);
    if (!j.is_object()) throw std::runtime_error(path + ": an electronics part must be a JSON object");
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(path + ": missing required 'format' field (expected \"physics_sim.electronics/1\")");
    }
    if (j.at("format").get<std::string>() != "physics_sim.electronics/1") {
        throw std::runtime_error(path + ": unsupported 'format' \"" + j.at("format").get<std::string>() +
                                 "\" (expected \"physics_sim.electronics/1\")");
    }
    if (!j.contains("kind") || !j.at("kind").is_string()) {
        throw std::runtime_error(path + ": 'kind' is required (\"launch_control\" or \"assist_fitment\")");
    }
    const std::string kind = j.at("kind").get<std::string>();

    ElectronicsPart part;
    if (j.contains("name") && j.at("name").is_string()) part.name = j.at("name").get<std::string>();
    if (j.contains("source") && j.at("source").is_string()) part.source = j.at("source").get<std::string>();

    if (kind == "launch_control") {
        reject_unknown(j, {"format", "kind", "name", "source", "extends", "rpm", "max_speed_mps", "channel", "fitted"}, path);
        part.kind = ElectronicsKind::LaunchControl;
        part.launch.rpm = require_positive(j, "rpm", path);
        part.launch.max_speed_mps = require_positive(j, "max_speed_mps", path);
        if (j.contains("channel")) {
            if (!j.at("channel").is_string() || j.at("channel").get<std::string>().empty()) {
                throw std::runtime_error(path + ": 'channel' must be a non-empty string");
            }
            part.launch.channel = j.at("channel").get<std::string>();
        }
        if (j.contains("fitted")) {
            if (!j.at("fitted").is_boolean()) throw std::runtime_error(path + ": 'fitted' must be a boolean");
            part.launch.fitted = j.at("fitted").get<bool>();
        }
    } else if (kind == "assist_fitment") {
        reject_unknown(j, {"format", "kind", "name", "source", "extends", "fits"}, path);
        part.kind = ElectronicsKind::AssistFitment;
        if (!j.contains("fits") || !j.at("fits").is_array() || j.at("fits").empty()) {
            throw std::runtime_error(path + ": 'fits' is required and must be a non-empty array of assist names");
        }
        for (const json& f : j.at("fits")) {
            if (!f.is_string() || !is_assist_name(f.get<std::string>())) {
                throw std::runtime_error(path + ": 'fits' entries must be one of auto_clutch, auto_blip, auto_shift, traction_control, anti_lag");
            }
            const std::string n = f.get<std::string>();
            if (std::find(part.fits.begin(), part.fits.end(), n) != part.fits.end()) {
                throw std::runtime_error(path + ": duplicate assist '" + n + "' in 'fits'");
            }
            part.fits.push_back(n);
        }
    } else {
        throw std::runtime_error(path + ": unknown electronics kind \"" + kind + "\" (expected \"launch_control\" or \"assist_fitment\")");
    }
    return part;
}

} // namespace ps::io
