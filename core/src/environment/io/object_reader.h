// core/src/environment/io/object_reader.h - INTERNAL: typed access to one JSON
// object with unknown-key rejection, shared by the ps_environment_io loaders.
// Never calls json::get<T>() on an unchecked value and never catches an
// exception (clang-cl ASan catch caveat, CLAUDE.md).
#pragma once

#include "ps/environment/weather_desc.h"
#include "ps/types.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace ps::io::detail {

using nlohmann::json;
using ps::real;
using ps::environment::RangeDesc;


// Typed access to one JSON object with unknown-key rejection. Never uses
// json::get<T>() on an unchecked value (no exception is caught anywhere in
// this file - clang-cl ASan catch caveat).
class ObjectReader {
public:
    ObjectReader(const json& obj, std::string context, const std::string& origin)
        : obj_(obj), ctx_(std::move(context)), origin_(origin) {
        if (!obj.is_object()) fail("must be an object");
    }

    [[noreturn]] void fail(const std::string& msg) const {
        throw std::runtime_error(origin_ + ": " + (ctx_.empty() ? "" : ctx_ + ": ") + msg);
    }

    bool has(const char* key) const { return obj_.contains(key); }

    real number(const char* key, bool required, real fallback) {
        used_.insert(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            if (required) fail(std::string("missing required field \"") + key + "\"");
            return fallback;
        }
        if (!it->is_number()) fail(std::string("\"") + key + "\" must be a number");
        const real v = it->get<real>();
        if (!std::isfinite(v)) fail(std::string("\"") + key + "\" must be finite");
        return v;
    }

    int integer(const char* key, bool required, int fallback) {
        used_.insert(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            if (required) fail(std::string("missing required field \"") + key + "\"");
            return fallback;
        }
        if (!it->is_number()) fail(std::string("\"") + key + "\" must be an integer");
        const real v = it->get<real>();
        if (!std::isfinite(v) || v != std::floor(v) || std::fabs(v) > 1e9)
            fail(std::string("\"") + key + "\" must be an integer");
        return static_cast<int>(v);
    }

    std::string string(const char* key, bool required, const std::string& fallback) {
        used_.insert(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            if (required) fail(std::string("missing required field \"") + key + "\"");
            return fallback;
        }
        if (!it->is_string()) fail(std::string("\"") + key + "\" must be a string");
        return it->get<std::string>();
    }

    const json* child(const char* key) {
        used_.insert(key);
        auto it = obj_.find(key);
        return it == obj_.end() ? nullptr : &*it;
    }

    void skip(const char* key) { used_.insert(key); }

    // [lo, hi]: an array of two finite numbers.
    RangeDesc range(const char* key) {
        used_.insert(key);
        auto it = obj_.find(key);
        if (it == obj_.end()) fail(std::string("missing required field \"") + key + "\"");
        if (!it->is_array() || it->size() != 2 || !(*it)[0].is_number() || !(*it)[1].is_number())
            fail(std::string("\"") + key + "\" must be an array of two numbers [lo, hi]");
        RangeDesc r;
        r.lo = (*it)[0].get<real>();
        r.hi = (*it)[1].get<real>();
        if (!std::isfinite(r.lo) || !std::isfinite(r.hi)) fail(std::string("\"") + key + "\" must be finite");
        return r;
    }

    // A required child object (null not allowed).
    const json& object(const char* key) {
        const json* c = child(key);
        if (!c) fail(std::string("missing required field \"") + key + "\"");
        if (!c->is_object()) fail(std::string("\"") + key + "\" must be an object");
        return *c;
    }

    // Keys named by the format but not implemented yet: a clear error rather
    // than a silent drop.
    void reserve_for_later(const char* key, const char* increment) {
        used_.insert(key);
        if (obj_.contains(key))
            fail(std::string("\"") + key + "\" is part of physics_sim.climate/1 but is not implemented before " +
                 increment + "; remove it");
    }

    void finish() const {
        for (auto it = obj_.begin(); it != obj_.end(); ++it)
            if (!used_.count(it.key())) fail("unknown key \"" + it.key() + "\"");
    }

    const std::string& context() const { return ctx_; }

private:
    const json& obj_;
    std::string ctx_;
    const std::string& origin_;
    std::set<std::string> used_;
};

} // namespace ps::io::detail
