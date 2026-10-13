// ps/io/limit_policy_io.h - loader of physics_sim.limit_policy/1 (increment L0, docs/data_formats/limit_policy.md): the
// refusal limits and advice thresholds of the engine garage as a data file (ps/drivetrain/limit_policy.h).
//
// A file may be partial: every entry it omits keeps the compiled default (data/limit_policies/default.json lists them all, so a
// derived file is usually `extends` + the few entries it changes). `extends` chains resolve through
// ps::io::load_json_with_extends (RFC 7396 merge, bottom-up); the optional user layer is merged last, over the whole chain.
// Unknown keys, wrong types, a source text without a [C]/[D]/[AS]/[M] tag and values outside their domain
// (validate_limit_policy) are rejected with the file path and the key, e.g. "<path>: turbo/gate_near_opening: must be <= 1".
#pragma once

#include "ps/drivetrain/limit_policy.h"

#include <string>

namespace ps::io {

// Throws std::runtime_error on any problem. `user_path` empty = no user layer.
[[nodiscard]] drivetrain::LimitPolicy load_limit_policy_json(const std::string& path, const std::string& user_path = {});

// The same from text (tests, tools): `virtual_path` stands for the file (its directory is the base of "extends", its name
// appears in messages). `user_text` empty = no user layer (its `extends` is relative to virtual_path's directory too).
[[nodiscard]] drivetrain::LimitPolicy load_limit_policy_json_from_string(const std::string& text, const std::string& virtual_path = "<string>",
                                                                         const std::string& user_text = {});

} // namespace ps::io
