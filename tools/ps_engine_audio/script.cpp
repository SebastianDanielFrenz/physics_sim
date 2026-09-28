#include "script.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ps_tools {

namespace {
using json = nlohmann::json;

const char* kFormat = "physics_sim.engine_audio_script/1";

json read_json_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("load_engine_audio_script: could not open '" + path + "'");
    json j;
    in >> j;
    return j;
}

std::optional<ps::real> opt_number(const json& ej, const char* key, const std::string& ctx) {
    if (!ej.contains(key)) return std::nullopt;
    if (!ej.at(key).is_number()) throw std::runtime_error(ctx + ": '" + key + "' must be a number");
    return ej.at(key).get<ps::real>();
}

} // namespace

EngineAudioScript load_engine_audio_script(const std::string& path) {
    const json j = read_json_file(path);
    if (!j.is_object() || !j.contains("format") || j.at("format") != kFormat) {
        throw std::runtime_error("load_engine_audio_script('" + path + "'): 'format' must be \"" +
                                  std::string(kFormat) + "\"");
    }
    EngineAudioScript s;
    if (j.contains("source")) {
        if (!j.at("source").is_string()) throw std::runtime_error(path + ": 'source' must be a string");
        s.source = j.at("source").get<std::string>();
    }
    if (j.contains("duration_s")) {
        if (!j.at("duration_s").is_number()) throw std::runtime_error(path + ": 'duration_s' must be a number");
        s.duration_s = j.at("duration_s").get<ps::real>();
    }
    if (!j.contains("events") || !j.at("events").is_array() || j.at("events").empty()) {
        throw std::runtime_error(path + ": 'events' must be a non-empty array");
    }

    static const std::vector<std::string> kEventKeys = {"t", "starter", "ignition", "throttle", "dyno_scale"};
    ps::real last_t = -1.0;
    bool first = true;
    for (const auto& ej : j.at("events")) {
        if (!ej.is_object()) throw std::runtime_error(path + ": each event must be an object");
        for (auto it = ej.begin(); it != ej.end(); ++it) {
            bool known = false;
            for (const auto& k : kEventKeys) known = known || (it.key() == k);
            if (!known) throw std::runtime_error(path + ": event has unknown key '" + it.key() + "'");
        }
        if (!ej.contains("t") || !ej.at("t").is_number()) {
            throw std::runtime_error(path + ": every event requires a numeric 't'");
        }
        ScriptEvent e;
        e.t = ej.at("t").get<ps::real>();
        if (!first && !(e.t >= last_t)) {
            throw std::runtime_error(path + ": event 't' values must be non-decreasing (got " + std::to_string(e.t) +
                                      " after " + std::to_string(last_t) + ")");
        }
        last_t = e.t;
        first = false;
        e.starter = opt_number(ej, "starter", path);
        e.ignition = opt_number(ej, "ignition", path);
        e.throttle = opt_number(ej, "throttle", path);
        e.dyno_scale = opt_number(ej, "dyno_scale", path);
        if (!e.starter && !e.ignition && !e.throttle && !e.dyno_scale) {
            throw std::runtime_error(path + ": event at t=" + std::to_string(e.t) +
                                      " sets none of starter/ignition/throttle/dyno_scale");
        }
        s.events.push_back(e);
    }
    return s;
}

} // namespace ps_tools
