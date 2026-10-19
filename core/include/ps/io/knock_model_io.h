// ps/io/knock_model_io.h - knock model v3/v4: load a physics_sim.knock_model/1 file (data/knock_models/*.json) into the
// replay's parameter set. Every parameter is {"value": number, "source": string}; unknown keys, a missing source, a
// wrong model/format/version and non-positive values are rejected (std::runtime_error naming the file and key).
#pragma once

#include "ps/engine_cycle/knock_model.h"

#include <string>

namespace ps::io {

struct KnockModelFile {
    engine_cycle::KnockModelParams params;
    double prefactor_s = 0.0;     // the literature A it was fitted with (must equal kDouaudEyzatA_s)
    double octane_exponent = 0.0; // the literature ON exponent (must equal kDouaudEyzatOnExponent)
    int version = 0;
};

[[nodiscard]] KnockModelFile load_knock_model_json(const std::string& path);

} // namespace ps::io
