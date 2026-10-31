// ps/machine/tool.h — a machine's working tools (docs/data_formats/machine.md).
//
// A tool is a working edge between two sockets of the machine's model. Pulled through
// soil it takes the ASABE D497 draft form
//
//     D = F_i (A + B S + C S^2) W T     newtons
//
// F_i the soil texture factor, S km/h, W the width in the coefficients' unit, T the
// depth in centimetres (1 for a tool whose table row has no depth term), raised by
// moisture and compaction. The force acts at the edge inside the vehicle's substep and
// reaches the tractor through the hitch rows, so what the tractor feels is the
// force itself: nothing in here knows about a tractor's power.
//
// A part names sockets and never carries a coordinate. The model owns where the edge
// is and how wide it is, the machine part how it behaves, and the caller binds the two
// (io::load_machine_json).

#pragma once

#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/limit_policy.h"
#include "ps/machine/soil_source.h"
#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps/vehicle/curve1d.h"

#include <array>
#include <string>
#include <vector>

namespace ps::machine {

struct DraftCoefficients {
    real a = 0.0, b = 0.0, c = 0.0; // newtons per width unit (per cm of depth with a depth term), S in km/h
};

enum class DraftWidthUnit : unsigned char { Metre, Tool };

struct ToolDraftDesc {
    std::string source;
    DraftWidthUnit unit = DraftWidthUnit::Metre;
    real tool_spacing_m = 0.0;  // DraftWidthUnit::Tool: the width in tools is the edge's width over this
    bool depth_term = true;     // false: T = 1
    DraftCoefficients primary;  // untilled soil
    DraftCoefficients secondary; // freshly tilled soil; between them by SoilSample::tilled
    std::array<real, 3> texture_factor{1.0, 1.0, 1.0}; // indexed by SoilTexture
    vehicle::Curve1D moisture_factor;                  // over SoilSample::wetness
    real compaction_gain = 0.0;  // draft + this share at SoilSample::compaction 1
    real lateral_ratio = 0.0;    // side force over draft, sliding sideways
    real creep_speed_m_s = 0.0;  // the forces build up from standstill over this speed
};

// What the soil does to the tool across its travel.
struct ToolSoilDesc {
    std::string source;
    real suction_ratio = 0.0;     // pulled in, over draft
    real bearing_n_m2 = 0.0;      // soil under the tool, per metre of width per metre of depth
    real gauge_n_m2 = 0.0;        // a roller or gauge wheel past the set depth, same unit; 0: none
    real hard_n_m2 = 0.0;         // ground it cannot enter (SoilSample::workable false)
    real hard_friction = 0.0;     // scraping over that ground
    real damping_n_s_m2 = 0.0;    // per metre of width
};

struct ToolPtoDesc {
    std::string source;
    bool present = false;
    real nominal_rpm = 0.0;
    real inertia_kgm2 = 0.0;      // at the tractor's shaft
    real no_load_kw = 0.0;
    real kw_per_m = 0.0;          // with the rotor running, per metre of width
    real specific_energy_mj_m3 = 0.0; // per cubic metre of soil worked
    real min_speed_fraction = 0.0; // below this share of nominal the tool does no work
};

// What a tool does with the ground. A Soil tool is pulled through it (draft, soil force, depth); an Applicator puts something
// on it without touching it: a sprayer, a spreader, a slurry splash plate. It has no draft, no soil force, no depth and no
// PTO, and works while it is switched on (World::set_tool_enabled), the ground under it is workable and it is moving forward.
enum class ToolKind : unsigned char { Soil, Applicator };

struct ToolDesc {
    ToolKind kind = ToolKind::Soil;
    std::string name;
    std::string operation; // the farm's record for this work; ps_core only carries the name
    Vec3 edge_left_local{}, edge_right_local{}; // chassis-local, from the model's sockets
    real width_m = 0.0;                         // their distance
    real depth_min_m = 0.0;     // shallower does not count as worked
    real depth_nominal_m = 0.0; // where the gauge is set to begin with
    real depth_max_m = 0.0;     // the tines' length: draft stops growing
    real speed_min_km_h = 0.0;  // the shop's range: below it only time is lost
    real speed_max_km_h = 0.0;  // above it quality falls
    real overspeed_quality_per_km_h = 0.0;
    real quality_floor = 0.0;
    std::string range_source;   // the depth, speed and quality numbers
    ToolDraftDesc draft;
    ToolSoilDesc soil;
    ToolPtoDesc pto;
};

struct MachineDesc {
    std::string name;
    std::string source;
    std::vector<ToolDesc> tools;
};

void validate_machine(const MachineDesc& d); // std::invalid_argument

// The width term W of the draft form.
[[nodiscard]] real tool_draft_width(const ToolDesc& tool);
// The draft force at a speed and depth, with no regard for standstill.
[[nodiscard]] real tool_draft_n(const ToolDesc& tool, const SoilSample& soil, real speed_km_h, real depth_m);
// 1 up to the tool's top speed, then down to the floor. Slower never hurts.
[[nodiscard]] real tool_quality(const ToolDesc& tool, real speed_km_h);
// The rotor's power with soil going through it at this speed and depth; speed 0 gives the no-load power.
[[nodiscard]] real tool_pto_power_w(const ToolDesc& tool, real speed_m_s, real depth_m);

// Last tick (sim thread only). Derived, not hashed.
struct ToolState {
    bool working = false;     // this tick counted as worked ground
    real depth_m = 0.0;
    real speed_m_s = 0.0;
    real draft_n = 0.0;       // mean over the tick, along the tool
    real vertical_n = 0.0;    // mean over the tick, up on the tool
    real quality = 1.0;
    real worked_area_m2 = 0.0; // cumulative
    real pto_speed_fraction = 0.0; // of nominal
    real pto_torque_nm = 0.0;      // at the tractor's shaft
    real gauge_depth_m = 0.0;
    bool enabled = false;     // Applicator only: switched on (World::set_tool_enabled)
};

// --- What a machine needs of a tractor, for a shop's "needs about N kW". ---

// physics_sim.tool_power/1. The soils and the efficiencies of an estimate.
struct ToolPowerSpec {
    std::string source;
    real tractive_ratio = 0.0;     // drawbar power over PTO-equivalent power
    real pto_share = 0.0;          // PTO power over net engine power
    real reserve_factor = 1.0;     // the engine is not bought to run at its limit
    std::vector<SoilSample> soils; // the sweep's soil cases
};

struct ToolPowerPoint {
    real speed_km_h = 0.0;
    SoilSample soil;
    real draft_n = 0.0;
    real drawbar_kw = 0.0;
    real pto_kw = 0.0;    // the rotor's own
    real engine_kw = 0.0; // net engine power this point takes, no reserve
};

struct ToolPowerSweep {
    std::vector<ToolPowerPoint> points; // ascending speed, within it the spec's soil order
    real needed_kw = 0.0;               // the worst point with the reserve: what the shop quotes
};

// Sweeps the tool's speed range in `speed_steps` steps at its nominal depth.
[[nodiscard]] ToolPowerSweep tool_power_sweep(const ToolDesc& tool, const ToolPowerSpec& spec, int speed_steps = 5);

// A tool against a tractor, as limit findings.
struct ToolTractorFit {
    real engine_kw = 0.0;        // rated net power
    real pto_torque_nm = 0.0;    // the PTO clutch at the shaft, 0: no PTO
};

[[nodiscard]] std::vector<drivetrain::LimitFinding> check_tool_limits(const ToolDesc& tool, const ToolPowerSweep& sweep, const ToolTractorFit& tractor,
                                                                      const drivetrain::LimitPolicy* policy = nullptr);

} // namespace ps::machine
