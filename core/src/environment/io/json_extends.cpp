// core/src/environment/io/json_extends.cpp - see ps/environment/json_extends.h.

#include "ps/environment/json_extends.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ps::io {

using nlohmann::json;

void json_merge_patch(json& target, const json& patch) {
    if (!patch.is_object()) {
        target = patch;
        return;
    }
    if (!target.is_object()) target = json::object();
    for (auto it = patch.begin(); it != patch.end(); ++it) {
        if (it.value().is_null()) {
            target.erase(it.key());
        } else {
            // operator[] creates a null member when absent; a null target is
            // not an object, so the recursive call starts from {} (RFC 7396).
            json_merge_patch(target[it.key()], it.value());
        }
    }
}

namespace {

json read_json_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error(path.string() + ": cannot open file");
    std::ostringstream ss;
    ss << in.rdbuf();
    // allow_exceptions = false: a parse error yields a discarded value, so no
    // exception object is ever caught here (clang-cl ASan catch caveat,
    // CLAUDE.md "Windows ASan caveat").
    json j = json::parse(ss.str(), nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) throw std::runtime_error(path.string() + ": invalid JSON");
    return j;
}

std::filesystem::path canonical_of(const std::filesystem::path& path) {
    std::error_code ec;
    std::filesystem::path canon = std::filesystem::weakly_canonical(path, ec);
    if (ec) canon = path.lexically_normal();
    return canon;
}

json resolve(const std::filesystem::path& path, int depth, int max_depth, std::vector<std::filesystem::path>& stack,
             const ExtendsRebase& rebase, const std::filesystem::path& root_dir, const json* in_memory) {
    const std::filesystem::path canon = canonical_of(path);
    for (const auto& p : stack) {
        if (p == canon) {
            std::string chain;
            for (const auto& q : stack) chain += q.filename().string() + " -> ";
            chain += canon.filename().string();
            throw std::runtime_error(path.string() + ": extends cycle (" + chain + ")");
        }
    }
    if (depth > max_depth)
        throw std::runtime_error(path.string() + ": extends chain deeper than " + std::to_string(max_depth) + " parents");

    json j = in_memory != nullptr ? *in_memory : read_json_file(path);
    if (!j.is_object()) throw std::runtime_error(path.string() + ": top level must be a JSON object");

    auto it = j.find("extends");
    std::filesystem::path parent_path;
    const bool has_parent = it != j.end();
    if (has_parent) {
        if (!it->is_string() || it->get_ref<const std::string&>().empty())
            throw std::runtime_error(path.string() + ": \"extends\" must be a non-empty string");
        parent_path = path.parent_path() / it->get_ref<const std::string&>();
        j.erase("extends");
    }
    // Garage P2: an ancestor's own relative paths are re-expressed against the
    // root file's directory before it is merged (the root itself is depth 0).
    if (depth > 0 && rebase) rebase(j, path.parent_path(), root_dir);
    if (!has_parent) return j;

    stack.push_back(canon);
    json merged = resolve(parent_path, depth + 1, max_depth, stack, rebase, root_dir, nullptr);
    stack.pop_back();

    json_merge_patch(merged, j);
    return merged;
}

} // namespace

json load_json_with_extends(const std::filesystem::path& path, int max_depth) {
    std::vector<std::filesystem::path> stack;
    return resolve(path, 0, max_depth, stack, ExtendsRebase{}, path.parent_path(), nullptr);
}

json load_json_with_extends(const std::filesystem::path& path, int max_depth, const ExtendsRebase& rebase) {
    std::vector<std::filesystem::path> stack;
    return resolve(path, 0, max_depth, stack, rebase, path.parent_path(), nullptr);
}

json resolve_json_extends(json root, const std::filesystem::path& virtual_path, int max_depth, const ExtendsRebase& rebase) {
    std::vector<std::filesystem::path> stack;
    return resolve(virtual_path, 0, max_depth, stack, rebase, virtual_path.parent_path(), &root);
}

std::string rebase_relative_path(const std::string& value, const std::filesystem::path& from_dir,
                                 const std::filesystem::path& to_dir) {
    const std::filesystem::path v(value);
    if (v.is_absolute()) return value;
    std::error_code ec;
    const std::filesystem::path target = std::filesystem::absolute(from_dir / v, ec).lexically_normal();
    if (ec) return (from_dir / v).lexically_normal().generic_string();
    const std::filesystem::path base = std::filesystem::absolute(to_dir.empty() ? std::filesystem::path(".") : to_dir, ec)
                                           .lexically_normal();
    if (ec) return target.generic_string();
    const std::filesystem::path rel = target.lexically_relative(base);
    if (rel.empty()) return target.generic_string();
    return rel.generic_string();
}

} // namespace ps::io
