// ps/drivetrain/engine_cycle_desc.h - public descriptor for the 0D crank-angle
// cycle model that generates a simulated engine's runtime maps (PHYS-033 P4
// I7c C15a, out/i7c_plan.md section 4). Pure data, no algorithm code: the
// model itself lives in core/src/engine_cycle (internal, ps::engine_cycle).
// No loader reads this yet (C16a); defaults below are the plan's Tier-2
// values, each labelled with its status.
//
// Crank-angle convention: degrees, 0 = gas-exchange TDC, 360 = firing TDC
// (TRUE TDC - with a pin offset the crank is then asin(e/(l+r)) past the
// cylinder axis, see core/src/engine_cycle/geometry.h). "ATDC" angles in the
// API are relative to firing TDC.
//
// Stroke convention [A, i7c plan section 3]: stroke_m is 2*crank radius. With
// a pin offset the true piston travel is slightly longer; the compression
// ratio is applied geometrically to the TRUE swept volume.

#pragma once

#include "ps/drivetrain/installation_desc.h"
#include "ps/drivetrain/table2d.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <optional>
#include <string>
#include <vector>

namespace ps::drivetrain {

// One valve's lift profile. lift_m_vs_deg.x is crank degrees, y is lift in
// metres (both ends must be 0 - the lift is 0 outside the digitised span).
// At cam phase 0 the profile's point x sits at crank angle x + phase_ref_deg
// (gas-exchange TDC = 0). An intake advance moves the profile earlier, an
// exhaust retard later. The span (x.back() - x.front()) must be < 720.
struct ValveLiftProfile {
    vehicle::Curve1D lift_m_vs_deg;
    real phase_ref_deg = 0.0;
};

// Tier 1b: cam phaser schedule over (rpm, p_im Pa). Consumed by the map
// generator (C16a), which turns it into each point's OperatingPoint; the
// cycle model itself takes the phase from the OperatingPoint.
struct CamSchedule {
    Table2D intake_advance_deg;
    Table2D exhaust_retard_deg;
};

// cv of one species group, linear in T: cv(T) = cv_a + cv_b*T, so the
// sensible internal energy u(T) = cv_a*T + cv_b*T^2/2 is closed-form.
// gamma_flow is the constant ratio of specific heats the valve-flow function
// uses for gas of this group (i7c plan section 3: fresh 1.40, burned 1.30).
struct GasGroupDesc {
    real cv_a_j_kgk = 0.0;
    real cv_b_j_kgk2 = 0.0;
    real r_j_kgk = 0.0;
    real gamma_flow = 1.4;
};

// Defaults DERIVED (C15a) from NIST-JANAF Shomate coefficients (Chase 1998,
// NIST Chemistry WebBook, public domain) by a least-squares linear fit of the
// mixture cv:
// - fresh: air as 1 O2 : 3.773 N2 (fuel vapour neglected - ASSUMED, it raises
//   the real unburned cv by a few percent), fitted over 300-1500 K (the
//   unburned-gas range), max relative error 1.7 %;
// - burned: stoichiometric iso-octane products 8 CO2 + 9 H2O + 47.16 N2
//   (iso-octane as the gasoline surrogate is ASSUMED), fitted over 300-3000 K,
//   max relative error 10 % at the range ends (H2O's 500-1700 K Shomate range
//   extrapolated to 300 K).
// gamma_flow 1.40/1.30 are the i7c plan's Tier-2 classes [A].
struct GasPropertiesDesc {
    GasGroupDesc fresh{659.6157, 0.1924821, 288.2124, 1.40};
    GasGroupDesc burned{817.5120, 0.1513793, 290.6604, 1.30};
};

// D13 friction correlation - descriptor only; C15a does not evaluate it
// (friction is C16a's job, i7c plan section 5).
struct FrictionModelDesc {
    enum class Model { None, PolynomialRpm, PolynomialPistonSpeed };
    Model model = Model::None;
    real a_pa = 0.0;  // constant term
    real b_pa = 0.0;  // per krpm (PolynomialRpm) or per m/s (PolynomialPistonSpeed)
    real c_pa = 0.0;  // per krpm^2 or per (m/s)^2
};

// ---- ps_audio A4 (out/audio_plan.md section 3.1): the cycle block's
// optional `intake`/`exhaust` pipe description, engine-owned per Q1's split
// (the throttle-body inlet through the first flange after the last
// engine-mounted part). Pure data - the mapping to a real PipeNetworkDesc
// (ps/engine_cycle/pipe_network.h) is A5's job; nothing reads these structs
// yet. See ps/io/vehicle_io.cpp's read_intake_block/read_exhaust_block for
// the loader's rules (both optional as a PAIR, cylinder-coverage checks,
// runner_*/back_pressure_pa_vs_exh_kg_s rejected together with these).

// One intake runner (or a tapered set shared by several cylinders).
// `cylinders` is empty for the JSON shorthand "cylinders": "all" (every
// engine cylinder shares this one runner description); otherwise it lists
// the 1-based cylinder numbers this runner serves, and every runner
// together must cover each cylinder exactly once (mirrors CycleModelDesc's
// own `banks` rule).
struct RunnerPipeDesc {
    std::vector<int> cylinders; // empty = "all"
    real length_m = 0.0;
    real d_in_m = 0.0;
    real d_out_m = 0.0;
};

struct IntakeCycleDesc {
    real throttle_bore_m = 0.0;
    real plenum_m3 = 0.0;
    std::vector<RunnerPipeDesc> runners;
    // Descriptive tag for the split interface point (plan section 3.3);
    // "throttle_inlet" in every example so far. Free text, not an enum -
    // only required non-empty.
    std::string interface;
    std::string source; // PLAN.md section 8: citation tier for every length/diameter above
    // Forced induction: the throttle is fed by a compressor (after its
    // intercooler), not by the snorkel/airbox. Set by the engine loader from
    // the engine's default_forced_induction, never read from the cycle
    // block itself. The map generator then supplies the throttle at every
    // grid node (max(ambient, 1.05 x node pressure) at ISA temperature)
    // instead of failing on a node the ambient path cannot reach. A
    // generator input: folded into the C16b cache key when true.
    bool compressor_supply = false;
};

// One exhaust primary (straight, untapered - unlike an intake runner).
// `cylinders` follows RunnerPipeDesc's own "all"/coverage convention.
struct PrimaryPipeDesc {
    std::vector<int> cylinders; // empty = "all"
    real length_m = 0.0;
    real d_m = 0.0;
};

// One entry of a merge's "from" list: either a cylinder's own primary
// (is_cylinder true, `cylinder` the 1-based number) or a NAMED reference to
// an earlier merge's `into` (is_cylinder false, `name`).
struct MergeSourceRef {
    bool is_cylinder = false;
    int cylinder = 0;
    std::string name;
};

// One junction where two or more primaries/earlier merges join into one
// pipe named `into`, which a LATER merge (or the exhaust `interface`, for
// the last merge in the list) refers back to.
struct MergeCycleDesc {
    std::vector<MergeSourceRef> from;
    std::string into;
    real length_m = 0.0;
    real d_m = 0.0;
};

struct ExhaustCycleDesc {
    std::vector<PrimaryPipeDesc> primaries;
    std::vector<MergeCycleDesc> merges;
    // Descriptive tag for the split interface point; must equal the LAST
    // merge's `into` (the loader checks this) - "flange" in every example
    // so far, or the engine's own manifold-catalyst outlet name if it
    // carries one (plan section 3.3).
    std::string interface;
    std::string source; // PLAN.md section 8: citation tier for every length/diameter above
};

// ps_audio A4: the "resolution" sub-block's cell-size knobs (plan section
// 3.1's defaults). Meaningful only together with intake/exhaust.
struct ResolutionCycleDesc {
    real power_dx_m = 0.05;
    real audio_dx_exhaust_m = 0.032;
    real audio_dx_intake_m = 0.017;
};

struct CycleModelDesc {
    // ---- Tier 1 (cited geometry; [AS] where uncited) ----
    real bore_m = 0.0;
    real stroke_m = 0.0;          // 2 * crank radius (convention A, see file comment)
    real rod_m = 0.0;
    real pin_offset_m = 0.0;      // piston-pin (or crank) offset e, positive toward the thrust side
    real compression_ratio = 0.0; // geometric, on the true swept volume
    int cylinders = 4;
    int cycle = 4;                // strokes; only 4 is supported

    ValveLiftProfile intake_lift;
    ValveLiftProfile exhaust_lift;
    int intake_valves = 2;
    int exhaust_valves = 2;
    real intake_valve_d_m = 0.0;  // valve head (inner seat) diameter
    real exhaust_valve_d_m = 0.0;

    real runner_length_m = 0.0;
    real runner_diameter_m = 0.0; // also the exhaust port -> reservoir orifice area (plan section 3)
    real intake_port_m3 = 0.0;
    real exhaust_port_m3 = 0.0;

    CamSchedule cam;                               // Tier 1b (generator input)
    vehicle::Curve1D back_pressure_pa_vs_exh_kg_s; // Tier 1b; empty = p_amb. Engine exhaust mass flow.

    // Tier 1b (I7c C17a follow-up): the ECU's spark schedule, deg BTDC over
    // x = rpm, y = intake manifold pressure (Pa) - the same axes and units as
    // SimulatedEngineDesc::ecu.spark_advance_deg, which the loader fills from
    // this table (one source of truth). Empty = mbt_tracking: every grid
    // node runs at the MBT Wiebe start and the generated maps are
    // bit-identical to an engine without this field. Non-empty: every grid
    // node's cycle runs with its Wiebe start at -(schedule) + ignition delay
    // (deg ATDC), and eta_i is stored as the MBT-equivalent value
    // eta_gross(scheduled) / eta_spark(scheduled - theta_mbt), so the
    // runtime's eta_i * eta_spark reproduces the cycle at every node.
    Table2D spark_advance_deg;
    // Tier 2: flame-development (ignition) delay, deg crank: the Wiebe
    // combustion start lies this many degrees AFTER the spark. >= 0; 0 =
    // spark and Wiebe start coincide (the pre-follow-up behaviour).
    real ignition_delay_deg = 0.0;

    // Engine garage P7 (C17b part 1): the generated knock limit theta_kl(rpm,
    // p_im). Filled by the loader from the ECU's ignition_strategy
    // "knock_limited" (+ knock_margin_deg) and the engine's fuels list - one
    // entry per listed fuel, in list order, each with the fuel's "knock"
    // block. enabled == false (every engine that does not opt in): the
    // generator computes no knock map and the cache key does not see this
    // block at all, so its maps and key are bit-identical to before P7.
    struct KnockFuel {
        std::string name; // the fuel's name (diagnostic; folded into the key)
        real ron = 0.0;
        real mon = 0.0;
    };
    struct KnockLimit {
        bool enabled = false;
        real margin_deg = 0.0;          // subtracted from the knock-safe advance at knock-limited nodes, [0, 10]
        std::vector<KnockFuel> fuels;   // >= 1 when enabled
    } knock;

    // ---- Tier 2 (frozen literature; every value carries its citation in the JSON source) ----
    real wiebe_a = 5.0;           // Heywood 1988 section 9.2 [A: page]
    real wiebe_m = 2.0;
    real burn_duration_deg = 60.0;  // [AS-literature-range]
    real ca50_mbt_deg_atdc = 8.0;   // MBT rule (plan D11)
    real woschni_c1_gx = 6.18;      // Heywood 1988 eq. 12.19ff (Woschni 1967)
    real woschni_c1_cc = 2.28;
    real woschni_c2 = 3.24e-3;      // m/(s*K), after spark
    real wall_temperature_k = 450.0;  // [AS]
    vehicle::Curve1D cd_vs_l_over_d;  // discharge coefficient on the curtain area pi*D*L
    real head_area_factor = 1.1;      // [AS] pent roof
    real combustion_eff_lean = 0.98;  // eta_c for lambda >= 1; eta_c = this*lambda below 1 (plan D14)
    GasPropertiesDesc gas;
    FrictionModelDesc friction;       // D13 (not evaluated in C15a)

    // Exhaust reservoir gas temperature used only for backflow from the
    // reservoir into the exhaust port [AS, C15a PROVISIONAL].
    real exhaust_backflow_temperature_k = 800.0;

    // ---- Generator grid (C16a; PHYS-033 section 5.2 defaults) ----
    std::vector<real> grid_rpm;
    std::vector<real> grid_p_im;
    // Opt-in interpolation-driven sampling. Bounds default to the authored grid.
    // Each generated table gets its own reduced axis (eta_i remains air mass).
    struct PressureSampling {
        bool enabled = false;
        bool allow_budget_limit = false;
        real min_pa = 0, max_pa = 0;
        real relative_error = 0.01;
        std::size_t max_points = 256;
        int max_passes = 8;
    } pressure_sampling;

    // The cycle block's own citation (PLAN.md section 8; required by the
    // loader, ps/io/vehicle_io.cpp's read_cycle_block). Carried through into
    // the generated maps' SimulatedEngineMapsDesc::source note (I7c C16b
    // review fix N4) so a generated engine's provenance survives; empty when
    // a CycleModelDesc is built directly in C++ (tests/fixtures), not
    // through the loader.
    std::string source;

    // ---- ps_audio A1 (out/audio_plan.md section 1/3, D-1/D-5): firing
    // phasing + exhaust-bank geometry. Descriptor only - NEITHER the map
    // generator (map_generator.cpp) NOR the single-cylinder cycle stepper
    // (cycle_stepper.h) read these yet (A5 wires a real multi-cylinder
    // CycleEngine to them); an engine's generated maps and state_hash are
    // therefore unaffected by these fields today, and they are deliberately
    // OUTSIDE engine_map_cache_key (map_cache.cpp) - H3's own rule ("every
    // field the generator will read must be in the key") does not yet apply
    // to them. See ps::engine_cycle::resolve_firing_angles_deg
    // (ps/engine_cycle/firing_order.h) for the derivation these feed, and
    // ps/io/vehicle_io.cpp's read_cycle_block for the loader/defaults/
    // rejections.
    //
    // 1-based cylinder numbers, a permutation of 1..cylinders, in FIRING
    // ORDER (cylinder firing_order[0] fires first). Default [1, 3, 4, 2]
    // only when cylinders == 4 (the common inline-4 order); the loader
    // requires it explicitly for any other cylinder count (a cross-plane
    // and a flat-plane V8 differ, and there is no single "common" order).
    std::vector<int> firing_order;

    // Optional. Empty = derive even 720/cylinders spacing in firing_order's
    // order (cylinder firing_order[i] gets angle i * 720/cylinders).
    // Non-empty = one EXPLICIT angle per cylinder, in CYLINDER-NUMBER order
    // (index 0 = cylinder 1's angle, ..., index cylinders-1 = the last
    // cylinder's), crank degrees, 0 = cylinder 1's firing TDC - required
    // for odd-fire engines (a V-twin, some V6), where no even-spacing rule
    // applies.
    std::vector<real> firing_angles_deg;

    // Groups of 1-based cylinder numbers assigning each cylinder to an
    // exhaust bank (audio source positions + bank topology, A8+; NOT used
    // for firing timing, which the two fields above already fully carry).
    // Empty = one bank containing every cylinder (the loader's default).
    std::vector<std::vector<int>> banks;

    // Bank geometry (the V angle between banks), degrees. Does not affect
    // firing timing. 0 for an inline/single-bank engine (the default).
    real bank_angle_deg = 0.0;

    // ---- ps_audio A4 (out/audio_plan.md section 3.1/3.2/3.3): optional as
    // a PAIR (both present or both absent) - see the structs above and
    // ps/io/vehicle_io.cpp's read_cycle_block. Absent: today's lumped path
    // runs unchanged, bit-identical maps/cache key (D5). Present:
    // runner_length_m/runner_diameter_m are unused (the loader rejects the
    // corresponding JSON keys), and default_installation_path/installation
    // below become meaningful.
    std::optional<IntakeCycleDesc> intake;
    std::optional<ExhaustCycleDesc> exhaust;

    // Absolute path to the engine file's OWN `default_installation` (given
    // relative to the engine file's own directory), empty when not given.
    // Provenance only - excluded from engine_map_cache_key, like every
    // other path/position (map_cache.cpp; plan section 3.3 "positions are
    // excluded from the key").
    std::string default_installation_path;

    // The RESOLVED installation this particular vehicle-engine pairing
    // uses: the vehicle powertrain component's own "installation" override
    // if given, else default_installation_path - loaded and validated by
    // ps::io::load_vehicle_json once both the engine file and (if given)
    // the override are visible (read_cycle_block itself only resolves
    // default_installation_path; it cannot see the vehicle's override).
    // nullopt iff intake/exhaust are both absent. installation_path is its
    // resolved absolute path (provenance only, excluded from the key like
    // default_installation_path above); `installation`'s ACOUSTIC content
    // (element types/lengths/diameters/volumes/loss coefficients, never
    // positions) is what the key hashes (map_cache.cpp).
    std::optional<InstallationDesc> installation;
    std::string installation_path;

    // Cell-size knobs; only meaningful (and only read by the loader's CFL
    // check) together with intake/exhaust. Defaults apply even when the
    // JSON "resolution" key is absent, exactly as if it had been given with
    // every field at its default.
    ResolutionCycleDesc resolution;
};

} // namespace ps::drivetrain
