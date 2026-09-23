#include "tir_parser.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace ps::vehicle {
namespace {

std::string trim(const std::string& s) {
    std::size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    std::size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string to_upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

// key -> pointer-to-member, so parsing is one table lookup instead of a
// long if/else chain. Only the keys tyre_desc.h models are listed here (see
// this file's header comment).
const std::unordered_map<std::string, real TyreDesc::*>& key_table() {
    static const std::unordered_map<std::string, real TyreDesc::*> table = {
        {"UNLOADED_RADIUS", &TyreDesc::unloaded_radius},
        {"WIDTH", &TyreDesc::width},
        {"RIM_RADIUS", &TyreDesc::rim_radius},
        {"FNOMIN", &TyreDesc::fz0},
        {"VERTICAL_STIFFNESS", &TyreDesc::vertical_stiffness},
        {"LONGVL", &TyreDesc::v0},
        {"QSY1", &TyreDesc::qsy1},
        {"QSY2", &TyreDesc::qsy2},
        {"QSY4", &TyreDesc::qsy4},
        {"LAMBDA_MUX", &TyreDesc::lambda_mux},
        {"LAMBDA_MUY", &TyreDesc::lambda_muy},
        {"PCX1", &TyreDesc::pcx1}, {"PDX1", &TyreDesc::pdx1}, {"PDX2", &TyreDesc::pdx2}, {"PDX3", &TyreDesc::pdx3},
        {"PEX1", &TyreDesc::pex1}, {"PEX2", &TyreDesc::pex2}, {"PEX3", &TyreDesc::pex3}, {"PEX4", &TyreDesc::pex4},
        {"PKX1", &TyreDesc::pkx1}, {"PKX2", &TyreDesc::pkx2}, {"PKX3", &TyreDesc::pkx3},
        {"PHX1", &TyreDesc::phx1}, {"PHX2", &TyreDesc::phx2},
        {"PVX1", &TyreDesc::pvx1}, {"PVX2", &TyreDesc::pvx2},
        {"PCY1", &TyreDesc::pcy1}, {"PDY1", &TyreDesc::pdy1}, {"PDY2", &TyreDesc::pdy2}, {"PDY3", &TyreDesc::pdy3},
        {"PEY1", &TyreDesc::pey1}, {"PEY2", &TyreDesc::pey2}, {"PEY3", &TyreDesc::pey3}, {"PEY4", &TyreDesc::pey4},
        {"PKY1", &TyreDesc::pky1}, {"PKY2", &TyreDesc::pky2},
        {"PHY1", &TyreDesc::phy1}, {"PHY2", &TyreDesc::phy2}, {"PHY3", &TyreDesc::phy3},
        {"PVY1", &TyreDesc::pvy1}, {"PVY2", &TyreDesc::pvy2}, {"PVY3", &TyreDesc::pvy3}, {"PVY4", &TyreDesc::pvy4},
        {"RBX1", &TyreDesc::rbx1}, {"RBX2", &TyreDesc::rbx2}, {"RCX1", &TyreDesc::rcx1}, {"RHX1", &TyreDesc::rhx1},
        {"RBY1", &TyreDesc::rby1}, {"RBY2", &TyreDesc::rby2}, {"RBY3", &TyreDesc::rby3}, {"RCY1", &TyreDesc::rcy1},
        {"RHY1", &TyreDesc::rhy1}, {"RHY2", &TyreDesc::rhy2},
        {"RVY1", &TyreDesc::rvy1}, {"RVY2", &TyreDesc::rvy2}, {"RVY3", &TyreDesc::rvy3},
        {"RVY4", &TyreDesc::rvy4}, {"RVY5", &TyreDesc::rvy5}, {"RVY6", &TyreDesc::rvy6},
        {"QBZ1", &TyreDesc::qbz1}, {"QBZ2", &TyreDesc::qbz2}, {"QBZ3", &TyreDesc::qbz3},
        {"QCZ1", &TyreDesc::qcz1},
        {"QDZ1", &TyreDesc::qdz1}, {"QDZ2", &TyreDesc::qdz2},
        {"QEZ1", &TyreDesc::qez1}, {"QEZ2", &TyreDesc::qez2},
        {"QHZ1", &TyreDesc::qhz1}, {"QHZ2", &TyreDesc::qhz2}, {"QHZ3", &TyreDesc::qhz3},
        {"QSX1", &TyreDesc::qsx1}, {"QSX2", &TyreDesc::qsx2}, {"QSX3", &TyreDesc::qsx3},
    };
    return table;
}

} // namespace

TyreDesc parse_tir_file(const std::string& path, const std::string& source_note) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open .tir file: " + path);

    TyreDesc desc;
    if (!source_note.empty()) desc.source = source_note;

    const auto& table = key_table();
    std::string line;
    while (std::getline(in, line)) {
        std::string s = trim(line);
        if (s.empty() || s[0] == '$' || s[0] == '!' || s[0] == '[') continue; // comment/blank/section header
        const std::size_t eq = s.find('=');
        if (eq == std::string::npos) continue;
        std::string key = to_upper(trim(s.substr(0, eq)));
        std::string value_part = trim(s.substr(eq + 1));
        // Strip an inline '$'-prefixed comment after the value.
        const std::size_t dollar = value_part.find('$');
        if (dollar != std::string::npos) value_part = trim(value_part.substr(0, dollar));
        std::istringstream vs(value_part);
        real value = 0.0;
        if (!(vs >> value)) continue; // not a numeric value (e.g. a string field like a name/version) - ignored

        const auto it = table.find(key);
        if (it != table.end()) desc.*(it->second) = value;
    }
    return desc;
}

} // namespace ps::vehicle
