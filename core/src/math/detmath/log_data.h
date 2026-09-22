// core/src/math/detmath/log_data.h — table for log(). Ported from musl
// libm v1.2.5, src/math/log_data.h (Copyright (c) 2018, Arm Limited;
// SPDX-License-Identifier: MIT). See common.h for the porting rules and
// THIRD_PARTY_NOTICES.md for the licence.
//
// Field order here matches this port's log_data.cpp initializer, not
// upstream's struct (which interleaves them differently) - purely an
// internal implementation detail, since nothing outside log.cpp reads this
// struct's fields directly.
#pragma once

namespace ps::math::detmath {

inline constexpr int kLogTableBits = 7;
inline constexpr int kLogPolyOrder = 6;
inline constexpr int kLogPoly1Order = 12;

struct LogTabEntry { double invc, logc; };
struct LogTab2Entry { double chi, clo; };

struct LogData {
    double ln2hi;
    double ln2lo;
    double poly1[kLogPoly1Order - 1];
    double poly[kLogPolyOrder - 1]; // first coefficient (-0.5) folded into poly[0]
    LogTabEntry tab[1 << kLogTableBits];
    LogTab2Entry tab2[1 << kLogTableBits]; // always present: this port forces __FP_FAST_FMA=0 (no compiler builtins, PLAN.md P1.5a Step 1 rule 2)
};

extern const LogData log_data;

} // namespace ps::math::detmath
