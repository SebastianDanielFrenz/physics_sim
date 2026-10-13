// core/src/drivetrain/limit_policy.cpp - increment L0: LimitPolicy defaults, severity rules and validation
// (ps/drivetrain/limit_policy.h; the entry table is limit_policy_table.h).
#include "ps/drivetrain/limit_policy.h"

#include "limit_policy_table.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace ps::drivetrain {

namespace {
namespace pt = policy_table;

// 2 bytes of enums padded to the doubles' alignment, then exactly one double per table row: a field added to LimitPolicy
// without a row in limit_policy_table.h (or the reverse) fails here.
static_assert(sizeof(LimitPolicy) == sizeof(real) + pt::kRowCount * sizeof(real),
              "LimitPolicy and policy_table::kRows disagree: every field needs exactly one row (and the reverse)");

[[nodiscard]] std::string fmt(real v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
    return buf;
}

[[nodiscard]] const pt::Row& row_of(real LimitPolicy::*m) {
    for (const pt::Row& r : pt::kRows) {
        if (r.member == m) return r;
    }
    return pt::kRows[0]; // unreachable: every kOrders member is a field (a test iterates them)
}
} // namespace

LimitPolicy::LimitPolicy() : refuse_at(RefuseAt::Exceeded), warn_at(WarnAt::Near) {
    for (const pt::Row& r : pt::kRows) this->*r.member = r.def;
}

const LimitPolicy& default_limit_policy() {
    static const LimitPolicy kDefault{};
    return kDefault;
}

bool LimitPolicy::warns(LimitSeverity s) const {
    switch (warn_at) {
    case WarnAt::Near: return true;
    case WarnAt::Limiting: return s >= LimitSeverity::Limiting;
    case WarnAt::Exceeded: return s >= LimitSeverity::Exceeded;
    case WarnAt::Never: return false;
    }
    return true;
}

bool LimitPolicy::refuses(LimitSeverity s) const {
    switch (refuse_at) {
    case RefuseAt::Exceeded: return s >= LimitSeverity::Exceeded;
    case RefuseAt::Limiting: return s >= LimitSeverity::Limiting;
    case RefuseAt::Never: return false;
    }
    return s >= LimitSeverity::Exceeded;
}

const char* to_string(RefuseAt r) {
    switch (r) {
    case RefuseAt::Exceeded: return "exceeded";
    case RefuseAt::Limiting: return "limiting";
    case RefuseAt::Never: return "never";
    }
    return "unknown";
}

const char* to_string(WarnAt w) {
    switch (w) {
    case WarnAt::Near: return "near";
    case WarnAt::Limiting: return "limiting";
    case WarnAt::Exceeded: return "exceeded";
    case WarnAt::Never: return "never";
    }
    return "unknown";
}

std::string validate_limit_policy(const LimitPolicy& p) {
    for (const pt::Row& r : pt::kRows) {
        const real v = p.*r.member;
        const std::string key = std::string(r.group) + "/" + r.name;
        if (!std::isfinite(v)) return key + ": must be a finite number";
        if (!(v > real(0.0))) return key + ": must be > 0 (is " + fmt(v) + ")";
        if (v < r.lo) return key + ": must be >= " + fmt(r.lo) + " (is " + fmt(v) + ")";
        if (v > r.hi) return key + ": must be <= " + fmt(r.hi) + " (is " + fmt(v) + ")";
    }
    for (const pt::Order& o : pt::kOrders) {
        const real a = p.*o.lo;
        const real b = p.*o.hi;
        if (o.strict ? !(a < b) : !(a <= b)) {
            const pt::Row& ra = row_of(o.lo);
            const pt::Row& rb = row_of(o.hi);
            return std::string(ra.group) + "/" + ra.name + " (" + fmt(a) + ") must be " + (o.strict ? "below " : "at most ") + rb.group +
                   "/" + rb.name + " (" + fmt(b) + ")";
        }
    }
    return {};
}

bool has_refusing_finding(const std::vector<LimitFinding>& findings, const LimitPolicy* policy) {
    const LimitPolicy& pol = resolve_limit_policy(policy);
    for (const LimitFinding& f : findings) {
        if (pol.refuses(f.severity)) return true;
    }
    return false;
}

} // namespace ps::drivetrain
