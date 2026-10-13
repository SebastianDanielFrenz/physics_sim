// Loader of physics_sim.limit_policy/1 (increment L0). Every rejection message names the file and the key. No exception object
// is caught here (clang-cl ASan catch caveat, CLAUDE.md).
#include "ps/io/limit_policy_io.h"

#include "../drivetrain/limit_policy_table.h"
#include "ps/environment/json_extends.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace ps::io {
namespace {

using json = nlohmann::json;
using ps::drivetrain::LimitPolicy;
using ps::drivetrain::RefuseAt;
using ps::drivetrain::WarnAt;
namespace pt = ps::drivetrain::policy_table;

constexpr const char* kFormat = "physics_sim.limit_policy/1";

[[noreturn]] void fail(const std::string& origin, const std::string& key, const std::string& what) {
    throw std::runtime_error(origin + ": " + (key.empty() ? "" : key + ": ") + what);
}

[[nodiscard]] bool is_group(const std::string& g) {
    for (const pt::Row& r : pt::kRows) {
        if (g == r.group) return true;
    }
    return false;
}

[[nodiscard]] const pt::Row* find_row(const std::string& group, const std::string& name) {
    for (const pt::Row& r : pt::kRows) {
        if (group == r.group && name == r.name) return &r;
    }
    return nullptr;
}

[[nodiscard]] bool has_source_tag(const std::string& s) {
    return s.rfind("[C]", 0) == 0 || s.rfind("[D]", 0) == 0 || s.rfind("[AS]", 0) == 0 || s.rfind("[M]", 0) == 0;
}

// Structure and types of one (merged or layer) object; writes every value it holds into `out`. `require_format`: the base
// file / merged result must name its format, a user layer need not.
void read_object(const json& root, const std::string& origin, bool require_format, LimitPolicy& out) {
    if (!root.is_object()) fail(origin, "", "the top level must be an object");
    if (root.contains("format")) {
        if (!root["format"].is_string() || root["format"].get<std::string>() != kFormat) {
            fail(origin, "format", std::string("must be \"") + kFormat + "\"");
        }
    } else if (require_format) {
        fail(origin, "format", std::string("is required (\"") + kFormat + "\")");
    }
    for (auto it = root.begin(); it != root.end(); ++it) {
        const std::string& key = it.key();
        const json& val = it.value();
        if (key == "format" || key == "extends") continue;
        if (val.is_null()) continue; // a user layer's RFC 7396 delete: the merged result lacks the entry, so it keeps the compiled default
        if (key == "source") {
            if (!val.is_string()) fail(origin, "source", "must be a string");
            continue;
        }
        if (key == "refuse_at") {
            if (!val.is_string()) fail(origin, key, "must be a string (\"exceeded\", \"limiting\" or \"never\")");
            const std::string s = val.get<std::string>();
            if (s == "exceeded") out.refuse_at = RefuseAt::Exceeded;
            else if (s == "limiting") out.refuse_at = RefuseAt::Limiting;
            else if (s == "never") out.refuse_at = RefuseAt::Never;
            else fail(origin, key, "\"" + s + "\" is not one of \"exceeded\", \"limiting\", \"never\"");
            continue;
        }
        if (key == "warn_at") {
            if (!val.is_string()) fail(origin, key, "must be a string (\"near\", \"limiting\", \"exceeded\" or \"never\")");
            const std::string s = val.get<std::string>();
            if (s == "near") out.warn_at = WarnAt::Near;
            else if (s == "limiting") out.warn_at = WarnAt::Limiting;
            else if (s == "exceeded") out.warn_at = WarnAt::Exceeded;
            else if (s == "never") out.warn_at = WarnAt::Never;
            else fail(origin, key, "\"" + s + "\" is not one of \"near\", \"limiting\", \"exceeded\", \"never\"");
            continue;
        }
        if (!is_group(key)) fail(origin, key, "unknown key");
        if (!val.is_object()) fail(origin, key, "must be an object of entries");
        for (auto e = val.begin(); e != val.end(); ++e) {
            const std::string path = key + "/" + e.key();
            const pt::Row* row = find_row(key, e.key());
            if (row == nullptr) fail(origin, path, "unknown key");
            const json& ev = e.value();
            if (ev.is_null()) continue;
            const json* number = &ev;
            if (ev.is_object()) {
                for (auto f = ev.begin(); f != ev.end(); ++f) {
                    if (f.key() != "value" && f.key() != "source") fail(origin, path + "/" + f.key(), "unknown key (only \"value\" and \"source\")");
                }
                if (!ev.contains("value")) fail(origin, path, "the entry has no \"value\"");
                number = &ev["value"];
                if (ev.contains("source")) {
                    if (!ev["source"].is_string()) fail(origin, path + "/source", "must be a string");
                    if (!has_source_tag(ev["source"].get<std::string>())) {
                        fail(origin, path + "/source", "must start with [C], [D], [AS] or [M]");
                    }
                }
            }
            if (!number->is_number()) fail(origin, path, "must be a number or an object {\"value\": number, \"source\": text}");
            out.*row->member = number->get<real>();
        }
    }
}

[[nodiscard]] LimitPolicy build(const json& merged, const std::string& origin) {
    LimitPolicy p; // compiled defaults for everything the file leaves out
    read_object(merged, origin, /*require_format=*/true, p);
    const std::string problem = validate_limit_policy(p);
    if (!problem.empty()) fail(origin, "", problem);
    return p;
}

[[nodiscard]] json read_file_json(const std::string& path) {
    return ps::io::load_json_with_extends(std::filesystem::path(path)); // throws naming the file on I/O, parse and extends errors
}

} // namespace

LimitPolicy load_limit_policy_json(const std::string& path, const std::string& user_path) {
    json merged = read_file_json(path);
    std::string origin = path;
    if (!user_path.empty()) {
        const json user = read_file_json(user_path);
        LimitPolicy scratch;
        read_object(user, user_path, /*require_format=*/false, scratch); // the layer's own structure first: errors name ITS file
        ps::io::json_merge_patch(merged, user);
        origin = path + " (+ user layer " + user_path + ")";
    }
    return build(merged, origin);
}

LimitPolicy load_limit_policy_json_from_string(const std::string& text, const std::string& virtual_path, const std::string& user_text) {
    json root = json::parse(text, nullptr, /*allow_exceptions=*/false);
    if (root.is_discarded()) fail(virtual_path, "", "not valid JSON");
    json merged = ps::io::resolve_json_extends(std::move(root), std::filesystem::path(virtual_path), 8, {});
    std::string origin = virtual_path;
    if (!user_text.empty()) {
        json user = json::parse(user_text, nullptr, false);
        if (user.is_discarded()) fail("user layer", "", "not valid JSON");
        user = ps::io::resolve_json_extends(std::move(user), std::filesystem::path(virtual_path), 8, {});
        LimitPolicy scratch;
        read_object(user, "user layer", false, scratch);
        ps::io::json_merge_patch(merged, user);
        origin = virtual_path + " (+ user layer)";
    }
    return build(merged, origin);
}

} // namespace ps::io
