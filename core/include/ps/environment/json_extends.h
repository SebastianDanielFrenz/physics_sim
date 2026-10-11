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
#include <functional>
#include <string>

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

// Engine garage P2: relative paths per file. A part format whose keys hold
// relative paths (an engine's "fuels", a turbo configuration's "turbo", ...)
// passes a rebase callback: before an ANCESTOR's object is merged, the
// callback rewrites each of that object's path-valued keys from
// `ancestor_dir` to `root_dir` (rebase_relative_path below), so the merged
// object resolves every path against the root file's directory, exactly as
// if the key had been written there. The root file's own keys are never
// passed to the callback - a file without "extends" is returned unchanged.
// An empty callback behaves like the two-argument overload.
using ExtendsRebase = std::function<void(nlohmann::json& ancestor, const std::filesystem::path& ancestor_dir,
                                         const std::filesystem::path& root_dir)>;
nlohmann::json load_json_with_extends(const std::filesystem::path& path, int max_depth, const ExtendsRebase& rebase);

// The same chain resolution for an object already in memory (e.g. parsed from
// a string): `virtual_path` stands for the file the object would be (its
// parent directory is the base of "extends" and of the rebase; its name only
// appears in messages and never equals a real file in the cycle check).
nlohmann::json resolve_json_extends(nlohmann::json root, const std::filesystem::path& virtual_path, int max_depth,
                                    const ExtendsRebase& rebase);

// A relative path written in a file in `from_dir`, re-expressed relative to
// `to_dir` (forward slashes, lexically normalised). An absolute `value` is
// returned unchanged; when no relative form exists (another drive) the
// absolute path is returned.
std::string rebase_relative_path(const std::string& value, const std::filesystem::path& from_dir,
                                 const std::filesystem::path& to_dir);

} // namespace ps::io
