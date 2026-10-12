// core/src/io/engine_parts.cpp - see engine_parts.h (engine garage P2).

#include "engine_parts.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace ps::io::parts {

namespace {

namespace fs = std::filesystem;

void rebase_key(json& obj, const char* key, const fs::path& from_dir, const fs::path& to_dir) {
    auto it = obj.find(key);
    if (it == obj.end() || !it->is_string()) return; // a wrong type is the reader's rejection
    const std::string& v = it->get_ref<const std::string&>();
    if (v.empty() || v == "none") return;
    *it = rebase_relative_path(v, from_dir, to_dir);
}

std::string resolve_path(const std::string& base_dir, const std::string& p) {
    const fs::path fp(p);
    if (fp.is_absolute()) return p;
    return (fs::path(base_dir) / fp).string();
}

bool ends_with_source(const std::string& k) {
    return k.size() > 7 && k.compare(k.size() - 7, 7, "_source") == 0;
}

void require_part_format(const json& j, const std::string& kind, const std::string& path) {
    const std::string expected = "physics_sim." + kind + "/1";
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(path + ": missing required 'format' field (expected \"" + expected + "\")");
    }
    const std::string actual = j.at("format").get<std::string>();
    if (actual != expected) throw std::runtime_error(path + ": unsupported 'format' \"" + actual + "\" (expected \"" + expected + "\")");
}

// ---- ECU part (physics_sim.ecu/1): a patch restricted to the ECU-owned engine paths --------------------------
constexpr const char* kEcuTopKeys[] = {"idle_rpm",        "stall_rpm",         "fire_rpm",
                                       "start_complete_rpm", "dfco_enable_rpm", "dfco_reenable_rpm",
                                       "idle_control",    "limiter",           "throttle_actuator_tau_s",
                                       "switching"}; // engine garage P9: the cam-profile switching rule
constexpr const char* kEcuBlockKeys[] = {"lambda_target", "spark_advance_deg", "ignition_strategy", "knock_margin_deg"}; // engine "ecu" object (P7: + knock opt-in)
constexpr const char* kEcuCycleKeys[] = {"spark_advance_deg", "cam"};         // engine "cycle" object
constexpr const char* kCamKeys[] = {"intake_lift", "exhaust_lift", "profiles"}; // engine "cycle" object (P9: + profiles)

template <std::size_t N>
bool in_list(const std::string& k, const char* const (&list)[N]) {
    return std::any_of(std::begin(list), std::end(list), [&](const char* x) { return k == x; });
}

// The ECU-owned field an engine object carries inline, or "" when none.
std::string inline_ecu_field(const json& j) {
    for (const char* k : kEcuTopKeys)
        if (j.contains(k)) return k;
    if (j.contains("ecu") && j.at("ecu").is_object()) {
        for (const char* k : kEcuBlockKeys)
            if (j.at("ecu").contains(k)) return std::string("ecu.") + k;
    }
    if (j.contains("cycle") && j.at("cycle").is_object()) {
        for (const char* k : kEcuCycleKeys)
            if (j.at("cycle").contains(k)) return std::string("cycle.") + k;
    }
    return {};
}

std::string inline_cam_field(const json& j) {
    if (j.contains("cycle") && j.at("cycle").is_object()) {
        for (const char* k : kCamKeys)
            if (j.at("cycle").contains(k)) return std::string("cycle.") + k;
    }
    return {};
}

// Validates an ECU part and returns the patch to merge over the engine object.
json ecu_patch(const json& part, const std::string& path) {
    require_part_format(part, "ecu", path);
    json patch = json::object();
    for (auto it = part.begin(); it != part.end(); ++it) {
        const std::string& k = it.key();
        if (k == "format" || k == "name" || k == "source") continue; // the part's own metadata
        if (in_list(k, kEcuTopKeys) || ends_with_source(k)) {
            patch[k] = it.value();
        } else if (k == "ecu" || k == "cycle") {
            if (!it.value().is_object()) throw std::runtime_error(path + ": '" + k + "' must be an object");
            for (auto jt = it.value().begin(); jt != it.value().end(); ++jt) {
                const std::string& kk = jt.key();
                const bool ok = (k == "ecu" ? in_list(kk, kEcuBlockKeys) : in_list(kk, kEcuCycleKeys)) || ends_with_source(kk);
                if (!ok) {
                    throw std::runtime_error(path + ": '" + k + "." + kk + "' is not an ECU field (an ECU part may set " + k +
                                             (k == "ecu" ? ".lambda_target, ecu.spark_advance_deg, ecu.ignition_strategy, ecu.knock_margin_deg" : ".spark_advance_deg, cycle.cam") +
                                             " and *_source notes)");
                }
            }
            patch[k] = it.value();
        } else {
            throw std::runtime_error(path + ": '" + k + "' is not an ECU field (docs/data_formats/ecu.md lists the fields)");
        }
    }
    return patch;
}

int fits_int(const json& fits, const char* key, const std::string& ctx) {
    const json& v = fits.at(key);
    if (!v.is_number_integer() || v.get<int>() < 1) throw std::runtime_error(ctx + ": fits." + key + " must be a positive integer");
    return v.get<int>();
}

void check_fits_range(const json& fits, const char* key, const json& cycle, const char* engine_key, const std::string& ctx) {
    if (!fits.contains(key)) return;
    const json& r = fits.at(key);
    if (!r.is_array() || r.size() != 2 || !r[0].is_number() || !r[1].is_number() || !(r[0].get<double>() <= r[1].get<double>())) {
        throw std::runtime_error(ctx + ": fits." + key + " must be [min, max] with min <= max");
    }
    if (!cycle.contains(engine_key) || !cycle.at(engine_key).is_number()) return; // the reader rejects the engine
    const double d = cycle.at(engine_key).get<double>();
    if (d < r[0].get<double>() || d > r[1].get<double>()) {
        throw std::runtime_error(ctx + ": the cam does not fit this engine: cycle." + engine_key + " " + std::to_string(d) +
                                 " mm is outside fits." + key + " [" + std::to_string(r[0].get<double>()) + ", " +
                                 std::to_string(r[1].get<double>()) + "]");
    }
}

// Validates a cam part against the engine's cycle block and returns the cycle patch.
json cam_patch(const json& part, const std::string& path, const json& engine_cycle) {
    require_part_format(part, "cam", path);
    json patch = json::object();
    for (auto it = part.begin(); it != part.end(); ++it) {
        const std::string& k = it.key();
        if (k == "format" || k == "name" || k == "source" || k == "fits") continue;
        if (in_list(k, kCamKeys) || ends_with_source(k)) {
            patch[k] = it.value();
        } else {
            throw std::runtime_error(path + ": '" + k + "' is not a cam field (a cam part sets intake_lift, exhaust_lift, profiles, "
                                     "fits)");
        }
    }
    // Engine garage P9: a cam part is the WHOLE cam. A profile cam carries its lifts inside each profile, so it removes the
    // engine's single lifts (RFC 7396 null); a single-lift cam removes an engine's profiles. A merge then never leaves
    // both; the engine reader's "profiles and intake_lift/exhaust_lift together" rejection fires only for an engine file
    // that writes both itself.
    if (part.contains("profiles")) {
        if (part.contains("intake_lift") || part.contains("exhaust_lift")) {
            throw std::runtime_error(path + ": a cam with 'profiles' carries intake_lift/exhaust_lift inside each profile, not at "
                                     "the top level");
        }
        for (const char* k : {"intake_lift", "exhaust_lift"}) {
            if (engine_cycle.contains(k)) patch[k] = nullptr;
        }
    } else if (engine_cycle.contains("profiles")) {
        patch["profiles"] = nullptr;
    }
    if (part.contains("fits")) {
        const json& fits = part.at("fits");
        if (!fits.is_object()) throw std::runtime_error(path + ": 'fits' must be an object");
        for (auto it = fits.begin(); it != fits.end(); ++it) {
            const std::string& k = it.key();
            if (k != "intake_valves" && k != "exhaust_valves" && k != "intake_valve_d_mm" && k != "exhaust_valve_d_mm" &&
                k != "family" && k != "source") {
                throw std::runtime_error(path + ": fits: unknown key '" + k + "'");
            }
        }
        if (fits.contains("family") && !fits.at("family").is_string()) throw std::runtime_error(path + ": fits.family must be a string");
        for (const char* k : {"intake_valves", "exhaust_valves"}) {
            if (!fits.contains(k)) continue;
            const int n = fits_int(fits, k, path);
            if (engine_cycle.contains(k) && engine_cycle.at(k).is_number_integer() && engine_cycle.at(k).get<int>() != n) {
                throw std::runtime_error(path + ": the cam does not fit this engine: it is for " + std::to_string(n) + " " + k +
                                         ", the engine's cycle block has " + std::to_string(engine_cycle.at(k).get<int>()));
            }
        }
        check_fits_range(fits, "intake_valve_d_mm", engine_cycle, "intake_valve_d_mm", path);
        check_fits_range(fits, "exhaust_valve_d_mm", engine_cycle, "exhaust_valve_d_mm", path);
    }
    return patch;
}

// A slot value from the engine file itself: absent, or a non-empty string.
bool engine_slot_string(const json& j, const char* key, const std::string& ctx, std::string& out) {
    if (!j.contains(key)) return false;
    if (!j.at(key).is_string() || j.at(key).get<std::string>().empty()) {
        throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be a non-empty path string");
    }
    out = j.at(key).get<std::string>();
    return true;
}

} // namespace

void rebase_engine_paths(json& j, const fs::path& from_dir, const fs::path& to_dir) {
    for (const char* k : {"fuel", "default_thermal", "default_ecu", "default_cam", "default_forced_induction",
                          "default_turbo_configuration"}) {
        rebase_key(j, k, from_dir, to_dir);
    }
    if (j.contains("fuels") && j.at("fuels").is_array()) {
        for (auto& e : j.at("fuels")) {
            if (e.is_string() && !e.get_ref<const std::string&>().empty()) e = rebase_relative_path(e.get<std::string>(), from_dir, to_dir);
        }
    }
    if (j.contains("cycle") && j.at("cycle").is_object()) rebase_key(j.at("cycle"), "default_installation", from_dir, to_dir);
}

void rebase_turbo_configuration_paths(json& j, const fs::path& from_dir, const fs::path& to_dir) {
    rebase_key(j, "turbo", from_dir, to_dir);
}

void rebase_turbo_hardware_paths(json& j, const fs::path& from_dir, const fs::path& to_dir) {
    rebase_key(j, "performance_map", from_dir, to_dir);
}

void rebase_n2o_bottle_paths(json& j, const fs::path& from_dir, const fs::path& to_dir) {
    rebase_key(j, "oxidizer", from_dir, to_dir);
}

void rebase_nitrous_kit_paths(json& j, const fs::path& from_dir, const fs::path& to_dir) {
    rebase_key(j, "default_controller", from_dir, to_dir);
}

json load_part(const std::string& path, const ExtendsRebase& rebase) {
    return load_json_with_extends(fs::path(path), 8, rebase);
}

bool is_engine_slot(const std::string& name) { return in_list(name, kEngineSlots); }

SlotMap read_component_slots(const json& cj, const std::string& base_dir, const std::string& ctx, bool inline_engine) {
    if (cj.contains("turbo_configuration")) {
        throw std::runtime_error(ctx + ": 'turbo_configuration' was renamed 'forced_induction' (engine garage P2; a path or \"none\")");
    }
    SlotMap slots;
    for (const char* s : kEngineSlots) {
        if (!cj.contains(s)) continue;
        const json& v = cj.at(s);
        if (inline_engine && std::string(s) == "ecu" && v.is_object()) continue; // the inline engine's own ECU block
        if (!v.is_string() || v.get<std::string>().empty()) {
            throw std::runtime_error(ctx + ": '" + std::string(s) + "' must be a non-empty path string" +
                                     (std::string(s) == "forced_induction" ? " or \"none\"" : ""));
        }
        const std::string value = v.get<std::string>();
        if (value == "none") {
            if (std::string(s) != "forced_induction") {
                throw std::runtime_error(ctx + ": '" + std::string(s) + "': \"none\" is only allowed for 'forced_induction'");
            }
            slots[s] = value;
        } else {
            slots[s] = resolve_path(base_dir, value);
        }
    }
    return slots;
}

void erase_component_slots(json& cj) {
    for (const char* s : kEngineSlots) {
        if (!cj.contains(s)) continue;
        if (std::string(s) == "ecu" && cj.at(s).is_object()) continue;
        cj.erase(s);
    }
}

AssembledEngine assemble_engine(json engine, const std::string& engine_dir, const std::string& ctx, const SlotMap& overrides) {
    AssembledEngine a;
    json& j = engine;
    if (!j.is_object()) throw std::runtime_error(ctx + ": an engine must be a JSON object");

    // ---- the renamed key and unknown default_* slots ----------------------------------------------------------
    if (j.contains("default_turbo_configuration")) {
        throw std::runtime_error(ctx + ": 'default_turbo_configuration' was renamed 'default_forced_induction' (engine garage P2; "
                                 "a path or \"none\")");
    }
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (k.rfind("default_", 0) != 0) continue;
        if (k == "default_ecu" || k == "default_cam" || k == "default_forced_induction" || k == "default_thermal") continue;
        throw std::runtime_error(ctx + ": unknown slot '" + k + "' (engine slots: default_ecu, default_cam, default_forced_induction, "
                                 "default_thermal; default_installation lives in the cycle block)");
    }
    for (const auto& kv : overrides) {
        if (!is_engine_slot(kv.first)) {
            throw std::runtime_error(ctx + ": unknown slot '" + kv.first + "' (engine slots: ecu, cam, forced_induction, installation, thermal)");
        }
    }
    const std::string kind = j.value("kind", std::string("torque_map"));

    // ---- engine-level slots ---------------------------------------------------------------------------------
    std::string default_ecu, default_cam, default_fi, default_thermal;
    const bool has_default_ecu = engine_slot_string(j, "default_ecu", ctx, default_ecu);
    const bool has_default_cam = engine_slot_string(j, "default_cam", ctx, default_cam);
    const bool has_default_fi = engine_slot_string(j, "default_forced_induction", ctx, default_fi);
    const bool has_default_thermal = engine_slot_string(j, "default_thermal", ctx, default_thermal);
    // Inline content and a slot for the same part in one engine object is ambiguous.
    if (has_default_ecu) {
        const std::string f = inline_ecu_field(j);
        if (!f.empty()) {
            throw std::runtime_error(ctx + ": 'default_ecu' and the inline ECU field '" + f + "' - give the ECU either inline or as a "
                                     "part file, not both (a child engine removes an inherited field with null)");
        }
    }
    if (has_default_cam) {
        const std::string f = inline_cam_field(j);
        if (!f.empty()) {
            throw std::runtime_error(ctx + ": 'default_cam' and the inline cam field '" + f + "' - give the cam either inline or as a "
                                     "part file, not both (a child engine removes an inherited field with null)");
        }
    }
    j.erase("default_ecu");
    j.erase("default_cam");
    j.erase("default_forced_induction");
    j.erase("default_thermal");

    auto pick = [&](const char* slot, bool has_default, const std::string& default_value, std::string& origin) -> std::string {
        const auto it = overrides.find(slot);
        if (it != overrides.end()) {
            origin = slot;
            return it->second;
        }
        if (has_default) {
            origin = std::string("default_") + slot;
            return default_value == "none" ? default_value : resolve_path(engine_dir, default_value);
        }
        return {};
    };

    // ---- ECU: merge the winning part over the engine's own ECU content ---------------------------------------
    std::string origin;
    const std::string ecu_path = pick("ecu", has_default_ecu, default_ecu, origin);
    if (!ecu_path.empty()) {
        if (ecu_path == "none") throw std::runtime_error(ctx + ": '" + origin + "': \"none\" is only allowed for forced induction");
        const json patch = ecu_patch(load_part(ecu_path), ecu_path);
        if (patch.contains("cycle") && !(j.contains("cycle") && j.at("cycle").is_object())) {
            throw std::runtime_error(ctx + ": the ECU part '" + ecu_path + "' sets cycle fields but the engine has no 'cycle' block");
        }
        json_merge_patch(j, patch);
    }

    // ---- cam: merge the winning part's lifts into the cycle block --------------------------------------------
    const std::string cam_path = pick("cam", has_default_cam, default_cam, origin);
    if (!cam_path.empty()) {
        if (cam_path == "none") throw std::runtime_error(ctx + ": '" + origin + "': \"none\" is only allowed for forced induction");
        if (!(j.contains("cycle") && j.at("cycle").is_object())) {
            throw std::runtime_error(ctx + ": a cam part ('" + origin + "') needs an engine with a 'cycle' block");
        }
        const json patch = cam_patch(load_part(cam_path), cam_path, j.at("cycle"));
        json_merge_patch(j.at("cycle"), patch);
    }

    // ---- forced induction -------------------------------------------------------------------------------------
    const std::string fi = pick("forced_induction", has_default_fi, default_fi, origin);
    if (!fi.empty()) {
        a.forced_induction_origin = origin;
        if (fi == "none") {
            a.forced_induction_none = true;
        } else {
            if (kind != "simulated") {
                throw std::runtime_error(ctx + ": '" + origin + "' is only allowed on a simulated engine (a torque-map engine has no "
                                         "air path to boost)");
            }
            a.has_forced_induction = true;
            a.forced_induction_path = fi;
        }
    }

    // ---- installation and thermal: paths for the existing loaders -----------------------------------------------
    const std::string inst = pick("installation", false, std::string(), origin);
    if (inst == "none") throw std::runtime_error(ctx + ": 'installation': \"none\" is only allowed for forced induction");
    a.installation_path = inst;
    const std::string thermal = pick("thermal", has_default_thermal, default_thermal, origin);
    if (thermal == "none") throw std::runtime_error(ctx + ": '" + origin + "': \"none\" is only allowed for forced induction");
    a.thermal_path = thermal;

    a.j = std::move(engine);
    return a;
}

} // namespace ps::io::parts
