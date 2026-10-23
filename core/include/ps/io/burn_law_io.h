// ps/io/burn_law_io.h - the burn law (design C, vault PHYS-079): load a physics_sim.burn_law/1 file
// (data/burn_laws/*.json) into its parameter set. Every parameter is {"value": number, "source": string} with the
// source's [C]/[D]/[M]/[AS] tag; unknown keys (top level and inside a parameter), a missing parameter or source, a
// wrong format/model/version and values that fail validate_burn_law are rejected (std::runtime_error naming the file
// and the key). data/burn_laws/default.json equals the compiled defaults (tests/unit/test_burn_law.cpp).
#pragma once

#include "ps/engine_cycle/burn_law.h"

#include <string>

namespace ps::io {

struct BurnLawFile {
    engine_cycle::BurnLawParams params;
    int version = 0;
};

[[nodiscard]] BurnLawFile load_burn_law_json(const std::string& path);

} // namespace ps::io
