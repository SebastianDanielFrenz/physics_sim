// core/src/math/detmath/rem_pio2.h — argument reduction mod pi/2.
// Ported from musl libm v1.2.5, src/math/__rem_pio2.c and
// src/math/__rem_pio2_large.c (origin: FreeBSD e_rem_pio2.c / k_rem_pio2.c).
// See common.h for the porting rules and THIRD_PARTY_NOTICES.md for the
// licence.
#pragma once

namespace ps::math::detmath {

// Returns the remainder of x mod pi/2 in y[0]+y[1] (y[0] the leading part,
// y[1] the correction), and n such that x ~= n*pi/2 + y[0]+y[1] (only the
// low 3 bits of n are meaningful to callers, per fdlibm convention).
// Caller must handle the no-reduction-needed case (|x| <~ pi/4) itself.
int rem_pio2(double x, double* y);

// Large-argument tail of rem_pio2 (musl's own __rem_pio2_large): x[] holds
// nx 24-bit chunks of the (already-shifted) input, y[] receives 2 (prec=1,
// double precision) result chunks; returns n&7.
int rem_pio2_large(double* x, double* y, int e0, int nx, int prec);

} // namespace ps::math::detmath
