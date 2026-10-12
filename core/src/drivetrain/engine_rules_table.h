// core/src/drivetrain/engine_rules_table.h - engine garage P4: the ONE table of every number the static buildability rules
// (ps/drivetrain/engine_rules.h, check_buildable) use. INTERNAL (the docs list the values in docs/data_formats/engine.md
// "Buildability rules"; a game does not tune them). Same layout as limit_thresholds.h.
//
// Every entry carries a source tag: [C] cited, [D] derived from data/code/mathematics in this repo, [AS] assumed (no source was
// available locally; the reasoning is in the entry), [M] measured on a shipped engine. Nothing here is a fitted constant of the
// physics - the rules only decide which engines the garage refuses before a map job starts.
//
// Sources that could NOT be read locally: Heywood, J. B., Internal Combustion Engine Fundamentals (1988), has no scan or text
// layer in this repository or the vault (the repo's own data/engines/toyota_a25a_fks_gen.json already marks its Table 6.2 valve
// ranges "[recalled, page to verify]"). Every Heywood-derived number is therefore tagged [AS] with the recalled statement, never
// [C]; no page numbers are claimed.
//
// kAllEngineRuleThresholds lists every entry so a test can assert that none is untagged and none is unused.
#pragma once

#include "ps/types.h"

#include <cstddef>

namespace ps::drivetrain::rules {

struct Threshold {
    const char* name;
    real value;
    const char* source; // starts with "[C]", "[D]", "[AS]" or "[M]"
};

// --- Valve layouts ----------------------------------------------------------------------------------------------------------
// The layouts that exist as production technology (design section 6.2): intake x exhaust valves per cylinder.
//   1 x 1 (2V wedge/hemi: nearly every pre-1990 engine), 2 x 1 (3V, Mercedes M112), 2 x 2 (4V pent-roof: the modern default),
//   3 x 2 (5V: Audi 20V, Yamaha). Anything else is not a production layout.
inline constexpr Threshold kLayoutSource{"valve_layouts", 4.0, "[AS] design section 6.2: 2V (1+1), 3V (2+1), 4V (2+2), 5V (3+2); each is a production layout (5V: Audi 20V, Yamaha; 3V: Mercedes M112)"};

// --- Intake valve head diameter over bore, per number of INTAKE valves -------------------------------------------------------
// Heywood 1988 section 6.3 states ranges for the valve diameter over the bore; the book is not readable here, so these are the
// recalled order of magnitude, widened until every shipped engine passes with room: 1 intake valve (2V) about 0.4-0.5 of the bore
// (a Ford Kent 38 mm in an 80 mm bore is 0.475, recalled), 2 intake valves (4V) 0.33-0.43 (the Toyota A25A-FKS 0.36, EPA/SAE 2019-
// 01-0249 gives the cam but the repo's own [AS] says 0.35-0.37 of Heywood Table 6.2; the hyper 38/92 = 0.413 [M]; a Honda K20A-like
// 35/86 = 0.407, recalled), 3 intake valves (5V) 0.25-0.35. The widened bands below leave the hyper at 0.413 inside the 4V band
// (owner requirement: the hyper's 0.41 x bore valves pass).
inline constexpr Threshold kIntake1Min{"intake_d_over_bore_1v_min", 0.34, "[AS] Heywood 1988 sec. 6.3 recalled (2-valve heads ~0.4-0.5 of bore), widened by 15 %"};
inline constexpr Threshold kIntake1Max{"intake_d_over_bore_1v_max", 0.52, "[AS] Heywood 1988 sec. 6.3 recalled; just above the Ford Kent 0.475 (recalled)"};
inline constexpr Threshold kIntake2Min{"intake_d_over_bore_2v_min", 0.28, "[AS] 20 % below the A25A-FKS 0.36 (repo [AS], Heywood Table 6.2 recalled 0.35-0.37)"};
inline constexpr Threshold kIntake2Max{"intake_d_over_bore_2v_max", 0.44, "[AS] widened over the hyper 0.413 [M] and the Honda K20A-like 0.407 (recalled)"};
inline constexpr Threshold kIntake3Min{"intake_d_over_bore_3v_min", 0.22, "[AS] 5-valve heads (3 intake) recalled ~0.28-0.33, widened"};
inline constexpr Threshold kIntake3Max{"intake_d_over_bore_3v_max", 0.34, "[AS] 5-valve heads recalled ~0.28-0.33, widened"};

// --- Exhaust over intake valve diameter (same layout, per valve) -------------------------------------------------------------
inline constexpr Threshold kExhaustRatioMin{"exhaust_over_intake_min", 0.78, "[AS] design section 6.2 (production range 0.78-0.90)"};
inline constexpr Threshold kExhaustRatioMax{"exhaust_over_intake_max", 0.90, "[AS] design section 6.2; the hyper (34/38 = 0.895) is the closest shipped engine"};

// --- Packaging --------------------------------------------------------------------------------------------------------------
// Row rule (a side with one or two valves): the valves of one side sit in a row across the head, centred about a quarter of the
// bore off the bore axis, so the row fits the chord of the bore circle there: 2*sqrt((B/2)^2 - (B/4)^2) = sqrt(3)/2 * B =
// 0.8660254 B. n * d must not exceed it (a design-doc example: 2 x 37.2 mm in an 86 mm bore = 0.865 B).
inline constexpr Threshold kRowChordFraction{"valve_row_chord_fraction", 0.8660254037844386, "[D] chord of the bore circle at a row offset of B/4 (valve row centreline [AS] at a quarter bore)"};
// Area rule: the valve heads are circles inside the bore circle; sum(d^2)/B^2 cannot exceed the densest packing of N equal circles in
// a circle, which is a mathematical result, N = 2: 1/2; 3: 3/(1+2/sqrt3)^2 = 0.6462; 4: 4/(1+sqrt2)^2 = 0.6863; 5: r/R = 0.37019
// -> 0.6852 (valid for equal circles; a mixed intake/exhaust size set packs no denser than the same area in equal circles [AS]).
// A real head also needs webs between the seats, the spark plug and the injector; the rule accepts this share of the ideal.
inline constexpr Threshold kPackDensity2{"pack_density_n2", 0.5, "[D] 2 equal circles in a circle: 1/2"};
inline constexpr Threshold kPackDensity3{"pack_density_n3", 0.6462, "[D] 3 equal circles in a circle: 3/(1+2/sqrt3)^2"};
inline constexpr Threshold kPackDensity4{"pack_density_n4", 0.6863, "[D] 4 equal circles in a circle: 4/(1+sqrt2)^2"};
inline constexpr Threshold kPackDensity5{"pack_density_n5", 0.6852, "[D] 5 equal circles in a circle: r/R = 0.37019 (Graham et al.)"};
inline constexpr Threshold kPackShare{"pack_share_of_ideal", 0.95, "[AS] the hyper packs 0.89 of the ideal 4-circle density [M]; 0.95 leaves it 6 % of room and still refuses valves with no web between the seats"};

// --- Connecting rod: ratio = rod length / crank radius (stroke / 2), Heywood's R = l/a ---------------------------------------
// The colloquial 'rod ratio' l / stroke (1.5-2) is half this number; the garage design (section 6.3) states 3.0-4.2 for l/a.
inline constexpr Threshold kRodRatioMin{"rod_over_crank_radius_min", 3.0, "[AS] design section 6.3; Heywood sec. 2.1 recalls R = 3-4 for small and medium engines. The Toyota A25A-FKS (155.1 / 51.7) sits exactly on it [M]"};
inline constexpr Threshold kRodRatioMax{"rod_over_crank_radius_max", 4.2, "[AS] design section 6.3"};

// --- Mean piston speed at the limiter: the P3b thresholds in limit_thresholds.h (kPistonSpeedNear/Limiting/Exceeded) are reused,
// not copied; the static rule's severities are mapped from them in engine_rules.cpp (>= Exceeded: Exceeded, >= Limiting: Near).

// --- Compression ratio, by aspiration ---------------------------------------------------------------------------------------
inline constexpr Threshold kCrNaMin{"cr_na_min", 8.0, "[AS] production gasoline engines since the 1980s run 8-14"};
inline constexpr Threshold kCrNaMax{"cr_na_max", 14.5, "[AS] Mazda SkyActiv-G 14.0 (recalled); the A25A-FKS runs 13.0 [C] SAE 2019-01-0249 via the repo data"};
inline constexpr Threshold kCrFiMin{"cr_forced_min", 7.0, "[AS] low-compression turbo engines (recalled 7.5-8.5)"};
inline constexpr Threshold kCrFiMax{"cr_forced_max", 10.5, "[AS] design section 6.3: no knock model, so forced induction is limited to <= 10.5"};

// --- Grid coverage ----------------------------------------------------------------------------------------------------------
// A lookup beyond the last node holds the last row (silent clamp). Relative overshoot of the requirement over the last node:
// any overshoot is Near; beyond kGridOvershootLimiting the held row is a visible error; beyond kGridOvershootExceeded the maps
// do not represent the engine.
inline constexpr Threshold kGridOvershootLimiting{"grid_overshoot_limiting", 0.05, "[AS] 5 % past the last node: clamped numbers start to matter (the shipped demo I4: limiter 6800 over a 6500 grid = 4.6 %)"};
inline constexpr Threshold kGridOvershootExceeded{"grid_overshoot_exceeded", 0.15, "[AS] 15 % past the last node"};
// The garage helpers' constants (boost_grid_p_im / extend_grid_rpm).
inline constexpr Threshold kBoostGridMargin{"boost_grid_margin_bar", 0.2, "[AS] design section 7.3: rows up to (1 + target + 0.2) bar absolute"};
inline constexpr Threshold kBoostGridAmbientBar{"boost_grid_ambient_bar", 1.0, "[AS] design section 7.3: '1 bar absolute' (the ISA 1.01325 is within the margin)"};
inline constexpr Threshold kBoostGridStepPa{"boost_grid_step_pa", 60000.0, "[M] the hyper's boosted rows (100 -> 160 -> 220 kPa) are spaced 60 kPa; the last step is 80 kPa"};
inline constexpr Threshold kBoostGridMinLastStepPa{"boost_grid_min_last_step_pa", 30000.0, "[AS] half a row pitch: a shorter last step is merged into the top node"};
inline constexpr Threshold kBoostGridAmbientNodeGapPa{"boost_grid_ambient_node_gap_pa", 2000.0, "[AS] engine garage P5: when the last node at or below ambient is more than 2 kPa under it, boost_grid_p_im adds a node at ambient; measured on the A25A (last node 97.27 kPa): without it the plenum at wide-open throttle (~101 kPa) interpolates toward the first boosted row and SimulatedEngine rejects the maps ('steady torque is not non-decreasing in plate position') at Preview quality; the hyper and the demo I4 (last node 100 kPa, 1.3 kPa under) need no extra node"};
inline constexpr Threshold kNaGridTopFraction{"na_grid_top_fraction_of_ambient", 0.95, "[AS] the generator throws for a node the engine cannot reach with the throttle fully open; the NA hyper's plenum peaks at 97873 Pa (0.966 of ISA ambient) at 8000 rpm [M], so ambient_grid_p_im caps the top node at 0.95 of ambient"};
inline constexpr Threshold kRelTolerance{"relative_tolerance", 1e-9, "[AS] comparison slack so a value exactly on a bound (the A25A's rod ratio 3.0000) passes"};

// --- Engine garage P9: discrete cam-profile switching (VTEC-style) ----------------------------------------------------------
// Switch speed over the limiter: Honda's own figures put the switch at 0.60-0.72 of the limit - B16A ~4800-5200 rpm of 8000
// [C] global.honda B16A page; K20A DC5 6000 of 8400 rpm [C] Honda Integra fact book 2001.07; FD2 5800 of 8400 [C] Civic Type R
// fact book 2007.03; S2000 F20C ~6000 of 9000 [C] hondanews S2000 powertrain. The band is that range widened [AS].
inline constexpr Threshold kCamSwitchFractionMin{"cam_switch_over_limiter_min", 0.45, "[AS] Honda VTEC 0.60-0.72 of the limiter (B16A, K20A DC5/FD2, F20C; sources in the comment above), widened"};
inline constexpr Threshold kCamSwitchFractionMax{"cam_switch_over_limiter_max", 0.85, "[AS] Honda VTEC 0.60-0.72 of the limiter, widened; above it the high cam serves a sliver of the band"};
// Hysteresis rpm_on - rpm_off: no OEM value found (Hondata exposes engage/disengage rpm without stock numbers [C] hondata.com
// FlashPro 'VTEC parameters' help page).
inline constexpr Threshold kCamHysteresisMin{"cam_switch_hysteresis_min_rpm", 50.0, "[AS] below ~50 rpm a WOT pull's speed ripple toggles the solenoid (no OEM hysteresis found)"};
inline constexpr Threshold kCamHysteresisMax{"cam_switch_hysteresis_max_rpm", 1000.0, "[AS] beyond ~1000 rpm the engine runs the wrong cam for a large part of a lift (no OEM hysteresis found)"};
// Peak lift over valve head diameter: the curtain area pi*d*L equals the seat area pi*d^2/4 at L/d = 0.25; production and
// street-tuning cams stay below ~0.35 (JUN K20 Type 0 high lobe 12.2 mm on a 35 mm valve = 0.35 [C] junauto.co.jp, vendor
// data; the 35 mm valve itself is a vendor listing).
inline constexpr Threshold kCamLiftOverValveMax{"cam_peak_lift_over_valve_d_max", 0.40, "[AS] 0.35 seen on an aftermarket K20 race cam (junauto.co.jp), widened; beyond it more lift adds no flow area and stresses the valve train"};

// --- Engine garage P10: continuous intake lift (Valvetronic-style) -----------------------------------------------------------
// Minimum lift: BMW's second Valvetronic generation runs 0.18 mm at minimum [C] fenderbender.com 'How to repair BMW Valvetronic
// systems' (Scot Manna 2017); the full lift of the N52 is 9.9 mm [C] mymotorlist.com/engines/bmw/n52b30 (secondary page), so the
// minimum node is 0.018 of full lift. Below ~0.1 mm the valve barely leaves its seat (seat and lash tolerances) [AS]; a minimum
// node above a quarter of full lift leaves the light-load range to the throttle, which the strategy is meant to avoid [AS].
inline constexpr Threshold kLiftMinPeakMinMm{"lift_min_node_peak_min_mm", 0.1, "[AS] below ~0.1 mm the valve barely leaves its seat (BMW's second-generation minimum is 0.18 mm, fenderbender.com Scot Manna 2017)"};
inline constexpr Threshold kLiftMinPeakMaxFraction{"lift_min_node_peak_max_fraction_of_full", 0.25, "[AS] BMW N52 0.18 of 9.9 mm = 0.018 (fenderbender.com; mymotorlist.com); above a quarter of full lift the throttle sets the light loads"};

inline constexpr Threshold kAllEngineRuleThresholds[] = {
    kLayoutSource,  kIntake1Min,   kIntake1Max,    kIntake2Min,     kIntake2Max,   kIntake3Min,
    kIntake3Max,    kExhaustRatioMin, kExhaustRatioMax, kRowChordFraction, kPackDensity2, kPackDensity3,
    kPackDensity4,  kPackDensity5, kPackShare,     kRodRatioMin,    kRodRatioMax,  kCrNaMin,
    kCrNaMax,       kCrFiMin,      kCrFiMax,       kGridOvershootLimiting, kGridOvershootExceeded, kBoostGridMargin,
    kBoostGridAmbientBar, kBoostGridStepPa, kBoostGridMinLastStepPa, kBoostGridAmbientNodeGapPa, kNaGridTopFraction, kRelTolerance,
    kCamSwitchFractionMin, kCamSwitchFractionMax, kCamHysteresisMin, kCamHysteresisMax, kCamLiftOverValveMax,
    kLiftMinPeakMinMm, kLiftMinPeakMaxFraction,
};
inline constexpr std::size_t kAllEngineRuleThresholdCount = sizeof(kAllEngineRuleThresholds) / sizeof(kAllEngineRuleThresholds[0]);

} // namespace ps::drivetrain::rules
