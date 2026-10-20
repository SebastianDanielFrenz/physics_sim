// core/src/io/burn_law_io.cpp - see ps/io/burn_law_io.h.
#include "ps/io/burn_law_io.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

namespace ps::io {

namespace {

struct ParamKey {
    const char* key;
    real engine_cycle::BurnLawParams::*field;
};

constexpr ParamKey kParams[] = {
    {"reference_rpm", &engine_cycle::BurnLawParams::reference_rpm},
    {"reference_unburned_temperature_k", &engine_cycle::BurnLawParams::reference_unburned_temperature_k},
    {"reference_unburned_pressure_pa", &engine_cycle::BurnLawParams::reference_unburned_pressure_pa},
    {"reference_residual_fraction", &engine_cycle::BurnLawParams::reference_residual_fraction},
    {"development_reference_deg", &engine_cycle::BurnLawParams::development_reference_deg},
    {"rapid_reference_deg", &engine_cycle::BurnLawParams::rapid_reference_deg},
    {"scale", &engine_cycle::BurnLawParams::scale},
    {"rpm_exponent", &engine_cycle::BurnLawParams::rpm_exponent},
    {"flame_speed_exponent", &engine_cycle::BurnLawParams::flame_speed_exponent},
    {"flame_speed_pressure_factor", &engine_cycle::BurnLawParams::flame_speed_pressure_factor},
    {"pressure_exponent", &engine_cycle::BurnLawParams::pressure_exponent},
    {"mk_phi_m", &engine_cycle::BurnLawParams::mk_phi_m},
    {"mk_b_m_m_s", &engine_cycle::BurnLawParams::mk_b_m_m_s},
    {"mk_b_2_m_s", &engine_cycle::BurnLawParams::mk_b_2_m_s},
    {"mk_alpha_0", &engine_cycle::BurnLawParams::mk_alpha_0},
    {"mk_alpha_phi", &engine_cycle::BurnLawParams::mk_alpha_phi},
    {"mk_beta_0", &engine_cycle::BurnLawParams::mk_beta_0},
    {"mk_beta_phi", &engine_cycle::BurnLawParams::mk_beta_phi},
    {"mk_t0_k", &engine_cycle::BurnLawParams::mk_t0_k},
    {"mk_p0_pa", &engine_cycle::BurnLawParams::mk_p0_pa},
    {"mk_phi_min", &engine_cycle::BurnLawParams::mk_phi_min},
    {"mk_phi_max", &engine_cycle::BurnLawParams::mk_phi_max},
    {"dilution_k", &engine_cycle::BurnLawParams::dilution_k},
    {"dilution_exponent", &engine_cycle::BurnLawParams::dilution_exponent},
    {"min_dilution_factor", &engine_cycle::BurnLawParams::min_dilution_factor},
    {"min_flame_speed_m_s", &engine_cycle::BurnLawParams::min_flame_speed_m_s},
    {"compression_exponent", &engine_cycle::BurnLawParams::compression_exponent},
};

bool tagged(const std::string& s) {
    for (const char* t : {"[C]", "[D]", "[M]", "[AS]"}) {
        if (s.find(t) != std::string::npos) return true;
    }
    return false;
}

} // namespace

BurnLawFile load_burn_law_json(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("burn law: cannot open '" + path + "'");
    nlohmann::json j;
    try {
        in >> j;
    } catch (...) { // TOOL-019: never touch the caught exception
        throw std::runtime_error("burn law: '" + path + "' is not valid JSON");
    }
    const std::string where = "burn law '" + path + "': ";
    if (!j.is_object()) throw std::runtime_error(where + "the file must be a JSON object");
    if (j.value("format", std::string()) != "physics_sim.burn_law/1") throw std::runtime_error(where + "format must be \"physics_sim.burn_law/1\"");
    if (j.value("model", std::string()) != "metghalchi_keck_wiebe") throw std::runtime_error(where + "model must be \"metghalchi_keck_wiebe\"");
    if (!j.contains("source") || !j.at("source").is_string() || j.at("source").get<std::string>().empty()) {
        throw std::runtime_error(where + "'source' is required");
    }
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        bool known = k == "format" || k == "model" || k == "version" || k == "source";
        for (const ParamKey& p : kParams) known = known || k == p.key;
        if (!known) throw std::runtime_error(where + "unknown key '" + k + "'");
    }
    BurnLawFile f;
    if (!j.contains("version") || !j.at("version").is_number_integer()) throw std::runtime_error(where + "'version' must be an integer");
    f.version = j.at("version").get<int>();
    if (f.version != static_cast<int>(engine_cycle::kBurnLawVersion)) {
        throw std::runtime_error(where + "version " + std::to_string(f.version) + " is not the compiled burn law version " +
                                 std::to_string(engine_cycle::kBurnLawVersion));
    }
    for (const ParamKey& pk : kParams) {
        if (!j.contains(pk.key) || !j.at(pk.key).is_object()) {
            throw std::runtime_error(where + "'" + pk.key + "' must be {\"value\", \"source\"}");
        }
        const nlohmann::json& p = j.at(pk.key);
        for (auto it = p.begin(); it != p.end(); ++it) {
            if (it.key() != "value" && it.key() != "source") throw std::runtime_error(where + pk.key + ": unknown key '" + it.key() + "'");
        }
        if (!p.contains("value") || !p.at("value").is_number()) throw std::runtime_error(where + pk.key + ".value must be a number");
        if (!p.contains("source") || !p.at("source").is_string() || !tagged(p.at("source").get<std::string>())) {
            throw std::runtime_error(where + pk.key + ".source is required and must carry a [C]/[D]/[M]/[AS] tag");
        }
        f.params.*(pk.field) = p.at("value").get<double>();
    }
    const std::string err = engine_cycle::validate_burn_law(f.params);
    if (!err.empty()) throw std::runtime_error(where + err);
    return f;
}

} // namespace ps::io
