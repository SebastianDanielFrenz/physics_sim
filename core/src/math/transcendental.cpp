// core/src/math/transcendental.cpp — thin forwarder from the public
// ps::math API to the internal ps::math::detmath port. See
// ps/math/transcendental.h and detmath/detmath.h for the rationale
// (PLAN.md D7, vault PHYS-009).

#include "ps/math/transcendental.h"
#include "detmath/detmath.h"

namespace ps::math {

real sin(real x) { return detmath::sin(x); }
real cos(real x) { return detmath::cos(x); }
void sincos(real x, real& s, real& c) { detmath::sincos(x, &s, &c); }
real tan(real x) { return detmath::tan(x); }
real asin(real x) { return detmath::asin(x); }
real acos(real x) { return detmath::acos(x); }
real atan(real x) { return detmath::atan(x); }
real atan2(real y, real x) { return detmath::atan2(y, x); }
real exp(real x) { return detmath::exp(x); }
real log(real x) { return detmath::log(x); }
real pow(real x, real y) { return detmath::pow(x, y); }

} // namespace ps::math
