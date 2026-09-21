// core/src/math is header-only (vec3/quat/mat3/pose/simd are templates or
// small value types with no state to define out-of-line). This translation
// unit exists so the build actually compiles every math header at least
// once even before world/force/vehicle code includes them, and hosts a few
// compile-time sanity checks.

#include "ps/math/mat3.h"
#include "ps/math/pose.h"
#include "ps/math/quat.h"
#include "ps/math/simd.h"
#include "ps/math/vec3.h"

namespace ps::math_check {

static_assert(sizeof(Vec3) == 3 * sizeof(real));
static_assert(sizeof(Quat) == 4 * sizeof(real));

constexpr Vec3 kX = Vec3::unit_x();
constexpr Vec3 kY = Vec3::unit_y();
static_assert(cross(kX, kY) == Vec3::unit_z());
static_assert(dot(kX, kY) == 0.0);

constexpr Mat3 kId = Mat3::identity();
static_assert(kId.trace() == 3.0);

} // namespace ps::math_check
