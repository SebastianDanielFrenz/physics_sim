#include "ps/io/surface_table.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace ps::io {

SurfaceTable::SurfaceTable(const std::string& surfaces_json_path) {
    std::ifstream in(surfaces_json_path);
    if (!in) throw std::runtime_error("cannot open surface table file: " + surfaces_json_path);

    nlohmann::json j;
    try {
        in >> j;
    } catch (const nlohmann::json::parse_error& e) {
        throw std::runtime_error("malformed JSON in " + surfaces_json_path + ": " + e.what());
    }

    if (!j.contains("surfaces") || !j.at("surfaces").is_object()) {
        throw std::runtime_error("surface table " + surfaces_json_path + " has no 'surfaces' object");
    }

    for (const auto& [name, _] : j.at("surfaces").items()) names_.push_back(name);
    std::sort(names_.begin(), names_.end()); // see surface_table.h: alphabetical, not file order

    if (names_.size() > static_cast<std::size_t>(kInvalidSurfaceId)) {
        throw std::runtime_error("surface table " + surfaces_json_path + " has too many surfaces (max " +
                                  std::to_string(static_cast<unsigned>(kInvalidSurfaceId)) + ")");
    }
    for (std::size_t i = 0; i < names_.size(); ++i) {
        name_to_id_[names_[i]] = static_cast<SurfaceId>(i);
    }
}

SurfaceId SurfaceTable::id_for(const std::string& name) const {
    const auto it = name_to_id_.find(name);
    return it == name_to_id_.end() ? kInvalidSurfaceId : it->second;
}

const std::string& SurfaceTable::name_for(SurfaceId id) const {
    static const std::string kUnknown = "<unknown surface id>";
    if (id >= names_.size()) return kUnknown;
    return names_[id];
}

} // namespace ps::io
