// tests/support/splitmix64.h — deterministic RNG for test generators.
//
// PHYS-033 P4 I2 plan section 11.1: "a deterministic splitmix64, NOT
// std::uniform_real_distribution (its output is implementation-defined)".
// Shared by the box-LCP brute-force cross-check (test 12) and the Cholesky/
// network white-box cases (test 11.2(g) and friends) so every generated
// case is reproducible byte-for-byte across compilers/platforms - the
// standard library's own distributions are explicitly NOT portable that
// way (D7).

#pragma once

#include <cstdint>

namespace ps_test {

class SplitMix64 {
public:
    explicit SplitMix64(std::uint64_t seed) : state_(seed) {}

    std::uint64_t next_u64() {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }

    // u01 = (x >> 11) * 2^-53 (plan section 11.1's own formula).
    double next_u01() { return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0); }

    double next_uniform(double lo, double hi) { return lo + (hi - lo) * next_u01(); }

private:
    std::uint64_t state_;
};

} // namespace ps_test
