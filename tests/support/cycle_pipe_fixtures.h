// tests/support/cycle_pipe_fixtures.h - a GENERIC pipe-bearing I4 for the
// ps_audio A5 CycleEngine / generator-v2 tests. Every value below is ASSUMED
// ([AS]): the base engine is tests/support/cycle_fixtures.h's generic 2.0 L
// I4 (itself [AS]); the pipes and the installation are typical of a 2.0 L NA
// road car, chosen for the tests, never fitted to anything. Not a shipped
// engine (A6 authors real pipe data).
//
// Topology: airbox (10 L) fed by a 0.4 m x 70 mm snorkel -> 60 mm throttle ->
// 2.5 L plenum -> four 0.35 m runners tapering 45 -> 40 mm (the lumped
// fixture's 350 x 40 mm runner) -> intake ports; exhaust ports -> four 0.6 m
// x 38 mm primaries -> 4-2-1 (1+4 and 2+3, the pairs 360 deg apart in the
// 1-3-4-2 firing order) -> 0.35 m x 45 mm secondaries -> 0.3 m x 50 mm
// collector to the flange -> installation: 0.4 m pipe, a 0.16 m x 100 mm
// catalyst (k_loss 1.5), 1.2 m pipe, a 0.35 m x 120 mm expansion chamber,
// a 0.5 m tailpipe, outlet.

#pragma once

#include "cycle_fixtures.h"

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/installation_desc.h"

#include <string>
#include <vector>

namespace ps::test {

inline drivetrain::InstallationDesc make_generic_installation() {
    using drivetrain::ExhaustElementDesc;
    using drivetrain::ExhaustElementType;
    drivetrain::InstallationDesc inst;
    inst.source = "test fixture [AS]";
    inst.intake.airbox_m3 = 0.010;
    inst.intake.has_snorkel = true;
    inst.intake.snorkel.length_m = 0.4;
    inst.intake.snorkel.d_m = 0.07;
    auto elem = [](ExhaustElementType t, real l, real d, real k) {
        ExhaustElementDesc e;
        e.type = t;
        e.length_m = l;
        e.d_m = d;
        e.k_loss = k;
        return e;
    };
    inst.exhaust.elements = {
        elem(ExhaustElementType::Pipe, 0.4, 0.05, 0.0),
        elem(ExhaustElementType::Catalyst, 0.16, 0.10, 1.5),
        elem(ExhaustElementType::Pipe, 1.2, 0.05, 0.0),
        elem(ExhaustElementType::ExpansionChamber, 0.35, 0.12, 0.0),
        elem(ExhaustElementType::Pipe, 0.5, 0.05, 0.0),
        elem(ExhaustElementType::Outlet, 0.0, 0.0, 0.0),
    };
    return inst;
}

inline drivetrain::MergeSourceRef merge_cyl(int c) {
    drivetrain::MergeSourceRef r;
    r.is_cylinder = true;
    r.cylinder = c;
    return r;
}

inline drivetrain::MergeSourceRef merge_name(const std::string& n) {
    drivetrain::MergeSourceRef r;
    r.is_cylinder = false;
    r.name = n;
    return r;
}

// runner_len_m / primary_len_m let V8 sweep the pipe lengths.
inline drivetrain::CycleModelDesc make_pipe_i4_cycle_desc(real runner_len_m = 0.35, real primary_len_m = 0.60) {
    drivetrain::CycleModelDesc d = make_generic_i4_cycle_desc();
    d.runner_length_m = 0.0;  // the loader zeroes them for a pipe engine
    d.runner_diameter_m = 0.0;
    d.firing_order = {1, 3, 4, 2};

    drivetrain::IntakeCycleDesc in;
    in.throttle_bore_m = 0.060;
    in.plenum_m3 = 2.5e-3;
    drivetrain::RunnerPipeDesc r;  // cylinders empty = "all"
    r.length_m = runner_len_m;
    r.d_in_m = 0.045;
    r.d_out_m = 0.040;
    in.runners = {r};
    in.interface = "throttle_inlet";
    in.source = "test fixture [AS]";
    d.intake = in;

    drivetrain::ExhaustCycleDesc ex;
    drivetrain::PrimaryPipeDesc p;
    p.length_m = primary_len_m;
    p.d_m = 0.038;
    ex.primaries = {p};
    drivetrain::MergeCycleDesc m14;
    m14.from = {merge_cyl(1), merge_cyl(4)};
    m14.into = "y14";
    m14.length_m = 0.35;
    m14.d_m = 0.045;
    drivetrain::MergeCycleDesc m23;
    m23.from = {merge_cyl(2), merge_cyl(3)};
    m23.into = "y23";
    m23.length_m = 0.35;
    m23.d_m = 0.045;
    drivetrain::MergeCycleDesc mc;
    mc.from = {merge_name("y14"), merge_name("y23")};
    mc.into = "flange";
    mc.length_m = 0.30;
    mc.d_m = 0.050;
    ex.merges = {m14, m23, mc};
    ex.interface = "flange";
    ex.source = "test fixture [AS]";
    d.exhaust = ex;

    d.installation = make_generic_installation();
    d.grid_rpm = {1500.0, 3000.0, 4500.0, 6000.0};
    d.grid_p_im = {30000.0, 60000.0, 90000.0};
    return d;
}

// ps_audio A8 V2: one BANK of a V8, as its own complete 4-cylinder "half
// engine" reusing make_pipe_i4_cycle_desc()'s exact merge-tree topology -
// see out/audio_a8_progress.md for why: CycleModelDesc has exactly one
// ExhaustCycleDesc/flange (ExhaustCycleDesc::interface "must equal the LAST
// merge's into", cycle_engine.cpp), so a literal single-engine two-bank
// V8 with two never-merging tailpipes cannot be built as one
// CycleModelDesc. Two independent bank fixtures, each its own engine/voice,
// give exactly what V2 needs (each bank's own outlet order content, and -
// run together with the crank pre-roll below - the combined 8-cylinder
// order content) without inventing a second topology.
//
// firing_angles_deg is indexed by CYLINDER NUMBER (firing_order.h's own
// design note), so it is given directly here rather than through
// firing_order (which stays the inherited {1,3,4,2} permutation - only used
// to derive angles when firing_angles_deg is empty, ignored otherwise).
//
// Cross-plane bank: cyl1@0, cyl2@90, cyl3@360, cyl4@450 - sorted firing
// order is 1,2,3,4 with alternating 90/270 deg gaps (90, 270, 90, 270), the
// real cross-plane V8's per-bank pattern (a "180 deg + 90 deg crank"
// bank). Both banks use the SAME shape; bank B's crank_angle_deg is preset
// 180 deg from bank A's (the test's job, not this fixture's) so the two
// banks' events interleave into the full V8's even 90-deg-apart firing
// (bank A at {0,90,360,450}, bank B at {0,90,360,450}+180 =
// {180,270,540,630} - together every 90 deg, 0..630).
inline drivetrain::CycleModelDesc make_v8_bank_cross_plane_cycle_desc(real runner_len_m = 0.35,
                                                                      real primary_len_m = 0.60) {
    drivetrain::CycleModelDesc d = make_pipe_i4_cycle_desc(runner_len_m, primary_len_m);
    d.firing_angles_deg = {0.0, 90.0, 360.0, 450.0};
    return d;
}

// Flat-plane bank: fires evenly (180 deg apart, cyl1@0/cyl3@180/cyl4@360/
// cyl2@540 via the default firing_order {1,3,4,2} with no explicit
// firing_angles_deg) - identical to the plain I4 fixture, an explicit
// wrapper only for naming symmetry with the cross-plane bank above. The
// full V8's two flat-plane banks interleave 90 deg apart (bank B pre-rolled
// 90 deg from bank A: {0,180,360,540}+90 = {90,270,450,630}, together every
// 90 deg with bank A's {0,180,360,540}).
inline drivetrain::CycleModelDesc make_v8_bank_flat_plane_cycle_desc(real runner_len_m = 0.35,
                                                                     real primary_len_m = 0.60) {
    return make_pipe_i4_cycle_desc(runner_len_m, primary_len_m);
}

} // namespace ps::test
