// core/src/math/detmath/exp_data.h — table for exp()/pow() (shared, like
// upstream). Ported from musl libm v1.2.5, src/math/exp_data.h (Copyright
// (c) 2018, Arm Limited; SPDX-License-Identifier: MIT). See common.h for
// the porting rules and THIRD_PARTY_NOTICES.md for the licence.
#pragma once

#include <cstdint>

namespace ps::math::detmath {

inline constexpr int kExpTableBits = 7;
inline constexpr int kExpPolyOrder = 5;

struct ExpData {
    double invln2N;
    double shift;
    double negln2hiN;
    double negln2loN;
    double poly[4]; // last four poly coefficients
    double exp2_shift;
    double exp2_poly[5];
    std::uint64_t tab[2 * (1 << kExpTableBits)];
};

extern const ExpData exp_data;

} // namespace ps::math::detmath
