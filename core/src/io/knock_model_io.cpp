// core/src/io/knock_model_io.cpp - see ps/io/knock_model_io.h.
#include "ps/io/knock_model_io.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

namespace ps::io {

KnockModelFile load_knock_model_json(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("knock model: cannot open '" + path + "'");
    nlohmann::json j;
    try {
        in >> j;
    } catch (...) { // TOOL-019: never touch the caught exception
        throw std::runtime_error("knock model: '" + path + "' is not valid JSON");
    }
    const std::string where = "knock model '" + path + "': ";
    if (!j.is_object()) throw std::runtime_error(where + "the file must be a JSON object");
    if (j.value("format", std::string()) != "physics_sim.knock_model/1") throw std::runtime_error(where + "format must be \"physics_sim.knock_model/1\"");
    if (j.value("model", std::string()) != "douaud_eyzat") throw std::runtime_error(where + "model must be \"douaud_eyzat\"");
    if (!j.contains("source") || !j.at("source").is_string() || j.at("source").get<std::string>().empty()) {
        throw std::runtime_error(where + "'source' is required");
    }
    static const char* const kKeys[] = {"format", "model", "version", "source", "prefactor_s", "octane_exponent",
                                        "octane_index_k", "prefactor_scale", "pressure_exponent", "activation_k",
                                        "dilution_exponent", "end_burn_fraction", "charge_heat_fraction"};
    for (auto it = j.begin(); it != j.end(); ++it) {
        bool known = false;
        for (const char* k : kKeys) known = known || it.key() == k;
        if (!known) throw std::runtime_error(where + "unknown key '" + it.key() + "'");
    }
    KnockModelFile f;
    if (!j.contains("version") || !j.at("version").is_number_integer()) throw std::runtime_error(where + "'version' must be an integer");
    f.version = j.at("version").get<int>();
    if (f.version != static_cast<int>(engine_cycle::kKnockModelVersion)) {
        throw std::runtime_error(where + "version " + std::to_string(f.version) + " is not the compiled knock model version " +
                                 std::to_string(engine_cycle::kKnockModelVersion));
    }
    auto param = [&](const char* key, bool allow_zero = false) {
        if (!j.contains(key) || !j.at(key).is_object()) throw std::runtime_error(where + "'" + key + "' must be {\"value\", \"source\"}");
        const nlohmann::json& p = j.at(key);
        for (auto it = p.begin(); it != p.end(); ++it) {
            if (it.key() != "value" && it.key() != "source") throw std::runtime_error(where + key + ": unknown key '" + it.key() + "'");
        }
        if (!p.contains("value") || !p.at("value").is_number()) throw std::runtime_error(where + key + ".value must be a number");
        if (!p.contains("source") || !p.at("source").is_string() || p.at("source").get<std::string>().empty()) {
            throw std::runtime_error(where + key + ".source is required");
        }
        const double v = p.at("value").get<double>();
        if (allow_zero ? !(v >= 0.0) : !(v > 0.0)) throw std::runtime_error(where + key + (allow_zero ? ".value must be >= 0" : ".value must be > 0"));
        return v;
    };
    f.prefactor_s = param("prefactor_s");
    f.octane_exponent = param("octane_exponent");
    f.params.octane_index_k = param("octane_index_k");
    f.params.prefactor_scale = param("prefactor_scale");
    f.params.pressure_exponent = param("pressure_exponent");
    f.params.activation_k = param("activation_k");
    f.params.dilution_exponent = param("dilution_exponent", true);
    f.params.end_burn_fraction = param("end_burn_fraction"); // knock model v5
    if (!(f.params.end_burn_fraction < 1.0)) throw std::runtime_error(where + "end_burn_fraction.value must be in (0, 1)");
    f.params.charge_heat_fraction = param("charge_heat_fraction", true); // knock model v6
    if (!(f.params.charge_heat_fraction < 1.0)) throw std::runtime_error(where + "charge_heat_fraction.value must be in [0, 1)");
    if (!(f.params.octane_index_k <= 1.0)) throw std::runtime_error(where + "octane_index_k.value must be in (0, 1]");
    return f;
}

} // namespace ps::io
