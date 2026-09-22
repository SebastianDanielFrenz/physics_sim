// adapters/godot/tests/test_origin_rebase.cpp — numeric acceptance test for
// PLAN.md P2's origin-rebasing requirement: "no visible jitter at +-50 km
// (camera-relative error < 0.1 mm, checked numerically)".
//
// Plain main(), no framework - see test_input_map.cpp's file comment for
// why. Also Godot-free (origin_rebase.h and frame_convert_core.h take no
// godot_cpp include), so this exercises exactly the numeric core the real
// adapter's frame_convert.h wraps, with no engine needed to run it.

#include "../src/frame_convert_core.h"
#include "../src/origin_rebase.h"

#include <cstdio>
#include <cmath>

using namespace ps_godot;

namespace {

int g_failures = 0;

void check(bool cond, const char* what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++g_failures;
    } else {
        std::printf("ok: %s\n", what);
    }
}

// Round-trips an absolute sim position through iso_to_godot_position (which
// rebases against `origin`, converts basis, and casts to float32 - exactly
// what a Node3D transform receives every frame) and back through
// godot_to_iso_position, and returns the absolute error in metres.
double roundtrip_error_m(const ps::Vec3& absolute, const ps::Vec3& origin) {
    const Vec3f godot_relative = iso_to_godot_position(absolute, origin);
    const ps::Vec3 recovered = godot_to_iso_position(godot_relative, origin);
    return (recovered - absolute).length();
}

} // namespace

int main() {
    // --- OriginRebase bookkeeping --------------------------------------
    {
        OriginRebase rebase(500.0);
        check(rebase.origin() == ps::Vec3::zero(), "origin_rebase: starts at zero");
        check(rebase.update(ps::Vec3{100.0, 0.0, 0.0}) == false, "origin_rebase: no rebase inside threshold");
        check(rebase.origin() == ps::Vec3::zero(), "origin_rebase: origin unchanged inside threshold");
        check(rebase.update(ps::Vec3{600.0, 0.0, 0.0}) == true, "origin_rebase: rebases once past threshold");
        check(rebase.origin() == ps::Vec3{600.0, 0.0, 0.0}, "origin_rebase: snaps exactly to the focus point");
        check(rebase.update(ps::Vec3{600.5, 0.0, 0.0}) == false, "origin_rebase: no re-rebase for a small move right after");
    }

    // --- The actual +-50 km acceptance numbers -------------------------
    // Without origin rebasing (origin fixed at world zero), a position near
    // +50 km loses float32 precision on the order of the value's own ULP:
    // for a float magnitude of ~5e4, the ULP is 2^(16-23) ~= 0.0039 m, i.e.
    // roughly 4 mm - already far above the 0.1 mm acceptance bound, and
    // this check proves that number rather than asserting it from theory.
    // Uses a deliberately non-round fractional position: an exact multiple
    // of a power of two (e.g. plain 50000.0) is itself exactly
    // float32-representable and would round-trip losslessly, hiding the
    // effect this control case exists to demonstrate.
    {
        const ps::Vec3 far_point{50000.123456789, 0.0, 10.0987654321};
        const double err_no_rebase = roundtrip_error_m(far_point, ps::Vec3::zero());
        std::printf("info: round-trip error at 50 km with NO origin rebase: %.6f m\n", err_no_rebase);
        check(err_no_rebase > 1.0e-4, "control: confirms the no-rebase case is worse than the 0.1 mm bound (motivates rebasing)");
    }
    {
        // With the origin rebased near the focus point (as the adapter does
        // every time the camera/tracked vehicle drifts past the threshold,
        // OriginRebase above), the same point - and everything within
        // OriginRebase's threshold of it - round-trips to well under 0.1 mm.
        const ps::Vec3 far_origin{50000.0, 0.0, 0.0};
        const double max_err_camera_relative_m = [&] {
            double worst = 0.0;
            // Sample points scattered within +-500 m of the origin (the
            // default rebase threshold), the actual working range the
            // camera/vehicle would occupy before the next rebase fires.
            const ps::Vec3 offsets[] = {
                {0.0, 0.0, 0.0}, {500.0, 0.0, 0.0}, {-500.0, 500.0, 0.0}, {123.456, -87.65, 3.21}, {-499.9, -499.9, 12.3},
            };
            for (const ps::Vec3& off : offsets) {
                const ps::Vec3 abs_pos = far_origin + off;
                const double err = roundtrip_error_m(abs_pos, far_origin);
                if (err > worst) worst = err;
            }
            return worst;
        }();
        std::printf("info: max round-trip error within 500 m of a rebased origin at 50 km: %.9f m\n", max_err_camera_relative_m);
        check(max_err_camera_relative_m < 1.0e-4, "acceptance: camera-relative error < 0.1 mm at +-50 km after origin rebase");
    }
    {
        // Same check at -50 km and on a different axis, and combined with a
        // non-trivial altitude, to rule out a sign or single-axis bug in
        // the basis-change formulas.
        const ps::Vec3 far_origin{-12000.0, -50000.0, 250.0};
        const ps::Vec3 abs_pos = far_origin + ps::Vec3{42.0, -17.5, 1.9};
        const double err = roundtrip_error_m(abs_pos, far_origin);
        std::printf("info: round-trip error near (-12000,-50000,250) origin, off by (42,-17.5,1.9): %.9f m\n", err);
        check(err < 1.0e-4, "acceptance: camera-relative error < 0.1 mm at -50 km, y-axis and altitude offset");
    }

    // --- Orientation round-trip (sanity: basis change is invertible) ---
    {
        const ps::Quat q = ps::Quat::from_axis_angle(ps::Vec3{1.0, 2.0, 3.0}, 0.7);
        const Basis3f b = iso_to_godot_basis(q);
        const ps::Quat back = godot_to_iso_rotation(b);
        // Quaternions q and -q represent the same rotation; compare via the
        // rotated vector, not the raw components.
        const ps::Vec3 v{0.3, -0.6, 0.9};
        const ps::Vec3 a = q.rotate(v);
        const ps::Vec3 c = back.rotate(v);
        const double err = (a - c).length();
        std::printf("info: orientation round-trip error: %.9f\n", err);
        check(err < 1.0e-6, "orientation: round-trip through the Godot basis change recovers the same rotation");
    }

    if (g_failures == 0) {
        std::printf("test_origin_rebase: all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_origin_rebase: %d check(s) FAILED\n", g_failures);
    return g_failures;
}
