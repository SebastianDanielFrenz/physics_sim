// ps/environment/json_extends.h - the "extends" override rule shared by the
// environment data formats (weather plan section 7; owner rule "modular part
// files with overrides, bottom-up, user last").
//
// Every environment format accepts `"extends": "<relative path>"`: the file's
// object is applied OVER its parent's as a JSON Merge Patch [C RFC 7396] - an
// object member merges recursively, `null` deletes the member, anything else
// (including an array) replaces. Chains resolve bottom-up: the deepest parent
// first, each descendant patched over it, the file the user named last. The
// parent path is relative to the file that names it. A cycle and a chain
// deeper than `max_depth` parents are errors.
//
// This is the first implementation of the override rule; later formats (and
// the vehicle loader's tyre_profile references) reuse it.
//
// The functions use nlohmann::json in their signatures; a consumer must link
// nlohmann_json itself (ps_environment_io links it PRIVATE, like ps_core).

#pragma once

#include <nlohmann/json_fwd.hpp>

#include <filesystem>

namespace ps::io {

// RFC 7396 MergePatch(target, patch), in place:
//   patch not an object            -> target = patch
//   patch an object                -> target (made an object if it is not) gets
//                                     each member: null erases, otherwise
//                                     target[name] = MergePatch(target[name], value)
void json_merge_patch(nlohmann::json& target, const nlohmann::json& patch);

// Reads `path`, resolves its `extends` chain and returns the merged object
// (without the `extends` key). Throws std::runtime_error (message names the
// file) on an unreadable/invalid file, a non-object top level, a non-string or
// empty `extends`, a cycle or a depth beyond `max_depth`.
nlohmann::json load_json_with_extends(const std::filesystem::path& path, int max_depth = 8);

} // namespace ps::io
