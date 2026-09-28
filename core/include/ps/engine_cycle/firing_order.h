// ps/engine_cycle/firing_order.h — firing-angle derivation (ps_audio A1,
// out/audio_plan.md section 1: "Each cylinder k gets a firing angle phi_k
// in crank degrees, where 0 is cylinder 1's firing TDC ... phi_k comes from
// firing_angles_deg if given. Otherwise it is even spacing 720/n in firing
// order."). Pure functions, no I/O, no exceptions caught/translated here -
// core/src/io/vehicle_io.cpp's loader calls these and wraps any thrown
// std::invalid_argument into its own "<ctx>: ..." message; a direct C++
// caller (tests, a future A5 CycleEngine) sees the message below unwrapped.
//
// Not yet wired into the map generator or the single-cylinder cycle
// stepper (CycleModelDesc::firing_order's own header comment) - A5 adds the
// multi-cylinder CycleEngine that actually phases each cylinder's state by
// this derivation's result.
//
// DESIGN NOTE (not an owner ruling - the plan text pins this down
// unambiguously enough that it did not need one; recorded here so the
// reasoning is not lost): an EXPLICIT firing_angles_deg is per CYLINDER
// NUMBER (index 0 = cylinder 1's own phi), not per firing_order POSITION.
// The plan's own sentence already indexes phi by cylinder number k before
// saying "phi_k comes from firing_angles_deg if given" - so the given array
// must use that same indexing to satisfy "if given" being a drop-in
// replacement for the derived phi_k. Only the DEFAULT (absent
// firing_angles_deg) actually consults firing_order's sequence.

#pragma once

#include "ps/types.h"

#include <string>
#include <vector>

namespace ps::engine_cycle {

// Non-throwing form: returns an empty string when firing_order is a valid
// permutation of 1..cylinders, else the exact message validate_firing_order
// below would throw. core/src/io/vehicle_io.cpp's loader uses this form (not
// try/catch + e.what()) because catching an exception by reference and then
// reading it is the clang-cl 19.1.5 Windows ASan crash pattern (vault
// TOOL-019, CLAUDE.md's "Windows ASan caveat") - core/ must never do that.
// validate_firing_order is implemented in terms of this, so the two can
// never disagree on message text.
[[nodiscard]] std::string firing_order_error(const std::vector<int>& firing_order, int cylinders);

// Throws std::invalid_argument if firing_order is not a permutation of
// 1..cylinders (wrong length, a value outside [1, cylinders], or a repeated
// value). Pure - never touches CycleModelDesc or anything else. For a
// direct C++ caller (tests, a future A5 CycleEngine) that wants the
// exception-throwing form; core/'s own loader uses firing_order_error
// instead (see above).
void validate_firing_order(const std::vector<int>& firing_order, int cylinders);

// Non-throwing form of validate_firing_angles below - same rationale/rule
// as firing_order_error above.
[[nodiscard]] std::string firing_angles_error(const std::vector<real>& firing_angles_deg, int cylinders);

// Throws std::invalid_argument if firing_angles_deg (when non-empty) is not
// exactly one value per cylinder, every value finite and in [0, 720), no
// two cylinders sharing (within 1e-9 deg) the same angle. An EMPTY
// firing_angles_deg is always valid (it means "derive the default").
void validate_firing_angles(const std::vector<real>& firing_angles_deg, int cylinders);

// Resolves the per-cylinder firing angle in crank degrees: index 0 =
// cylinder 1's angle, ..., index cylinders-1 = the last cylinder's angle.
//   - firing_angles_deg non-empty: returned as-is (validate_firing_angles
//     first if the input is untrusted - this function does not itself
//     validate, so tests can sabotage inputs directly).
//   - firing_angles_deg empty: cylinder firing_order[i] gets angle
//     i * 720/cylinders (validate_firing_order first, for the same reason).
// Precondition (not checked here): firing_order.size() == cylinders and
// firing_angles_deg.empty() || firing_angles_deg.size() == cylinders.
[[nodiscard]] std::vector<real> resolve_firing_angles_deg(const std::vector<int>& firing_order,
                                                           const std::vector<real>& firing_angles_deg, int cylinders);

} // namespace ps::engine_cycle
