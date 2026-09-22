// core/src/math/detmath/pow_data.h — table for the log part of pow().
// Ported from musl libm v1.2.5, src/math/pow_data.h (Copyright (c) 2018,
// Arm Limited; SPDX-License-Identifier: MIT). See common.h for the
// porting rules and THIRD_PARTY_NOTICES.md for the licence.
#pragma once

namespace ps::math::detmath {

inline constexpr int kPowLogTableBits = 7;
inline constexpr int kPowLogPolyOrder = 8;

struct PowLogTabEntry { double invc, pad, logc, logctail; };

struct PowLogData {
    double ln2hi;
    double ln2lo;
    double poly[kPowLogPolyOrder - 1]; // first coefficient is 1
    PowLogTabEntry tab[1 << kPowLogTableBits];
};

extern const PowLogData pow_log_data;

} // namespace ps::math::detmath
