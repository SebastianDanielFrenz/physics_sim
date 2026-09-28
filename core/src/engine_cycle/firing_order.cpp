#include "ps/engine_cycle/firing_order.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace ps::engine_cycle {

void validate_firing_order(const std::vector<int>& firing_order, int cylinders) {
    if (static_cast<int>(firing_order.size()) != cylinders) {
        throw std::invalid_argument("firing_order: expected " + std::to_string(cylinders) + " entries, got " +
                                     std::to_string(firing_order.size()));
    }
    std::vector<bool> seen(static_cast<std::size_t>(cylinders) + 1, false);
    for (std::size_t i = 0; i < firing_order.size(); ++i) {
        const int v = firing_order[i];
        if (v < 1 || v > cylinders) {
            throw std::invalid_argument("firing_order[" + std::to_string(i) + "]: cylinder number " + std::to_string(v) +
                                         " is out of range [1, " + std::to_string(cylinders) + "]");
        }
        if (seen[static_cast<std::size_t>(v)]) {
            throw std::invalid_argument("firing_order: cylinder " + std::to_string(v) +
                                         " appears more than once - firing_order must be a permutation of 1.." +
                                         std::to_string(cylinders));
        }
        seen[static_cast<std::size_t>(v)] = true;
    }
}

void validate_firing_angles(const std::vector<real>& firing_angles_deg, int cylinders) {
    if (firing_angles_deg.empty()) return; // empty = "derive the default", always valid
    if (static_cast<int>(firing_angles_deg.size()) != cylinders) {
        throw std::invalid_argument("firing_angles_deg: expected " + std::to_string(cylinders) +
                                     " entries (one per cylinder) or none, got " + std::to_string(firing_angles_deg.size()));
    }
    for (std::size_t i = 0; i < firing_angles_deg.size(); ++i) {
        const real v = firing_angles_deg[i];
        if (!std::isfinite(v) || !(v >= real(0.0)) || !(v < real(720.0))) {
            throw std::invalid_argument("firing_angles_deg[" + std::to_string(i) + "] (cylinder " + std::to_string(i + 1) +
                                         "): must be finite and in [0, 720), got " + std::to_string(v));
        }
    }
    constexpr real kMinSeparationDeg = real(1e-9);
    for (std::size_t i = 0; i < firing_angles_deg.size(); ++i) {
        for (std::size_t j = i + 1; j < firing_angles_deg.size(); ++j) {
            if (std::abs(firing_angles_deg[i] - firing_angles_deg[j]) < kMinSeparationDeg) {
                throw std::invalid_argument("firing_angles_deg: cylinders " + std::to_string(i + 1) + " and " +
                                             std::to_string(j + 1) + " share the same firing angle (" +
                                             std::to_string(firing_angles_deg[i]) + " deg)");
            }
        }
    }
}

std::vector<real> resolve_firing_angles_deg(const std::vector<int>& firing_order, const std::vector<real>& firing_angles_deg,
                                             int cylinders) {
    if (!firing_angles_deg.empty()) return firing_angles_deg;

    std::vector<real> phi(static_cast<std::size_t>(cylinders), real(0.0));
    const real step_deg = real(720.0) / real(cylinders);
    for (std::size_t i = 0; i < firing_order.size(); ++i) {
        const int cylinder = firing_order[i]; // 1-based
        phi[static_cast<std::size_t>(cylinder - 1)] = real(i) * step_deg;
    }
    return phi;
}

} // namespace ps::engine_cycle
