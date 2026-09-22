// core/src/math/detmath/detmath.h — internal deterministic double-precision
// transcendental functions (PLAN.md D7, vault PHYS-009). Ported from musl
// libm v1.2.5 (MIT; see THIRD_PARTY_NOTICES.md for the licence text and
// exact source commit). Bit-identical across every platform/compiler this
// project builds for, because it never calls the platform libm.
//
// Internal only: core code should include ps/math/transcendental.h and call
// ps::math::sin/cos/... instead (that header's .cpp is a thin forwarder to
// this namespace). This header exists so the accuracy/golden-table tests
// (white-box, like test_articulation_core.cpp) can call the ported
// functions directly.
#pragma once

namespace ps::math::detmath {

double sin(double x);
double cos(double x);
void sincos(double x, double* s, double* c);
double tan(double x);
double asin(double x);
double acos(double x);
double atan(double x);
double atan2(double y, double x);
double exp(double x);
double log(double x);
double pow(double x, double y);

} // namespace ps::math::detmath
