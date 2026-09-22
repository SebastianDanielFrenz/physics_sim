// ps/math/transcendental.h — deterministic double-precision transcendental
// functions (PLAN.md D7, vault PHYS-009).
//
// core/ must never call the platform libm's sin/cos/tan/asin/acos/atan/
// atan2/exp/log/pow (or sincos): ucrtbase (Windows) and glibc (Linux)
// round these differently (confirmed by a first-diverging-value trace in
// P1.5a: cos(0.69999833602557...) differs by 1 ulp between them with
// identical input bits), which broke Windows/Linux state-hash equality for
// any scenario with a revolute-joint articulation. Use these instead -
// bit-identical on every platform/compiler this project builds for,
// because they never call the platform libm (ported from musl libm, MIT;
// see THIRD_PARTY_NOTICES.md). A CI grep (tools/ci.ps1) enforces this for
// everything under core/ except core/src/math/detmath itself.
//
// sqrt is exempt (PLAN.md D7: "sqrt is IEEE-exact and may stay"), as are
// floor/scalbn/fabs/copysign (see core/src/math/detmath/common.h's file
// comment for why) - std::sqrt/std::fabs/... stay fine to call directly.
#pragma once

#include "ps/types.h"

namespace ps::math {

real sin(real x);
real cos(real x);
void sincos(real x, real& s, real& c);
real tan(real x);
real asin(real x);
real acos(real x);
real atan(real x);
real atan2(real y, real x);
real exp(real x);
real log(real x);
real pow(real x, real y);

} // namespace ps::math
