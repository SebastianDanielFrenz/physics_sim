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

json resolve(const std::filesystem::path& path, int depth, int max_depth, std::vector<std::filesystem::path>& stack) {
    std::error_code ec;
    std::filesystem::path canon = std::filesystem::weakly_canonical(path, ec);
    if (ec) canon = path.lexically_normal();
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

    json j = read_json_file(path);
    if (!j.is_object()) throw std::runtime_error(path.string() + ": top level must be a JSON object");

    auto it = j.find("extends");
    if (it == j.end()) return j;
    if (!it->is_string() || it->get_ref<const std::string&>().empty())
        throw std::runtime_error(path.string() + ": \"extends\" must be a non-empty string");
    const std::filesystem::path parent_path = path.parent_path() / it->get_ref<const std::string&>();
    j.erase("extends");

    stack.push_back(canon);
    json merged = resolve(parent_path, depth + 1, max_depth, stack);
    stack.pop_back();

    json_merge_patch(merged, j);
    return merged;
}

} // namespace

json load_json_with_extends(const std::filesystem::path& path, int max_depth) {
    std::vector<std::filesystem::path> stack;
    return resolve(path, 0, max_depth, stack);
}

} // namespace ps::io
