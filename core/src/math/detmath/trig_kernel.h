// core/src/math/detmath/trig_kernel.h — sin/cos/tan polynomial kernels on
// [-pi/4, pi/4]. Ported from musl libm v1.2.5, src/math/__sin.c,
// src/math/__cos.c, src/math/__tan.c (origin: FreeBSD k_sin.c/k_cos.c/
// k_tan.c). See common.h for the porting rules.
#pragma once

namespace ps::math::detmath {

// sin(x+y), x in [-pi/4,pi/4]; iy!=0 means y is a meaningful tail of x.
double kernel_sin(double x, double y, int iy);
// cos(x+y), x in [-pi/4,pi/4].
double kernel_cos(double x, double y);
// tan (odd==0) or -1/tan (odd!=0) of (x+y), x in [-pi/4,pi/4].
double kernel_tan(double x, double y, int odd);

} // namespace ps::math::detmath
