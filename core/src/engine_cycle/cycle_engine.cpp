// engine_cycle/cycle_engine.cpp - see cycle_engine.h.
#include "ps/engine_cycle/cycle_engine.h"

#include "ps/engine_cycle/firing_order.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/geometry.h"
#include "ps/engine_cycle/valve_flow.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ps::engine_cycle {
namespace {

using drivetrain::CycleModelDesc;
using drivetrain::ExhaustElementDesc;
using drivetrain::ExhaustElementType;

[[noreturn]] void fail(const std::string& what) { throw std::invalid_argument("CycleEngine: " + what); }

inline void fnv_fold(std::uint64_t& h, real v) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    for (int k = 0; k < 8; ++k) {
        h ^= (bits >> (8 * k)) & 0xffu;
        h *= 1099511628211ull;
    }
}

real area_of(real d) { return 0.25 * kPi * d * d; }

// The template entry (runner / primary) that covers 1-based cylinder c.
template <class T>
const T& entry_for_cylinder(const std::vector<T>& entries, int c, const char* what) {
    for (const T& e : entries) {
        if (e.cylinders.empty()) return e;
        for (int k : e.cylinders)
            if (k == c) return e;
    }
    fail(std::string("no ") + what + " covers cylinder " + std::to_string(c));
}

struct WgBuilder {
    WaveguideNetworkDesc* wd = nullptr;
    GasStateDesc gas;

    int add_node(WgNodeDesc::Kind kind, const std::string& name) {
        WgNodeDesc n;
        n.kind = kind;
        n.name = name;
        wd->nodes.push_back(n);
        return static_cast<int>(wd->nodes.size()) - 1;
    }
    int add_segment(int a, int b, real len, real d, const std::string& name) {
        WgSegmentDesc s;
        s.name = name;
        s.length_m = len;
        s.diameter_m = d;
        s.node_a = a;
        s.node_b = b;
        s.loss = WgLoss::Kirchhoff;
        s.gas = gas;
        s.mean_velocity_m_s = 0.0;
        wd->segments.push_back(s);
        return static_cast<int>(wd->segments.size()) - 1;
    }

    // One installation chain from Junction `from`. Returns false when the
    // chain held no segment before its outlet (the caller decides).
    bool add_chain(const std::vector<ExhaustElementDesc>& chain, int from, real flow_fraction,
                   const std::string& prefix) {
        int cur = from;
        int last_seg = -1;
        for (std::size_t i = 0; i < chain.size(); ++i) {
            const ExhaustElementDesc& e = chain[i];
            const std::string nm = prefix + "element " + std::to_string(i + 1);
            switch (e.type) {
                case ExhaustElementType::Pipe:
                case ExhaustElementType::ExpansionChamber: {
                    const int j = add_node(WgNodeDesc::Kind::Junction, nm + " end");
                    last_seg = add_segment(cur, j, e.length_m, e.d_m, nm);
                    cur = j;
                    break;
                }
                case ExhaustElementType::Catalyst: {
                    const int r = add_node(WgNodeDesc::Kind::Resistance, nm + " substrate");
                    wd->nodes[static_cast<std::size_t>(r)].k_loss = e.k_loss;
                    wd->nodes[static_cast<std::size_t>(r)].flow_fraction = flow_fraction;
                    add_segment(cur, r, 0.5 * e.length_m, e.d_m, nm + " inlet half");
                    const int j = add_node(WgNodeDesc::Kind::Junction, nm + " end");
                    last_seg = add_segment(r, j, 0.5 * e.length_m, e.d_m, nm + " outlet half");
                    cur = j;
                    break;
                }
                case ExhaustElementType::Outlet: {
                    if (last_seg < 0) return false;
                    WgNodeDesc& n = wd->nodes[static_cast<std::size_t>(cur)];
                    n.kind = WgNodeDesc::Kind::Open;
                    n.radiation = true;
                    n.name = nm + " (outlet)";
                    wd->segments[static_cast<std::size_t>(last_seg)].end_b = EndCorrection::Unflanged;
                    return true;
                }
                case ExhaustElementType::Branch: {
                    const real nb = static_cast<real>(e.branches.size());
                    for (std::size_t b = 0; b < e.branches.size(); ++b) {
                        const std::string bp = nm + " branch " + std::to_string(b + 1) + " ";
                        if (!add_chain(e.branches[b], cur, flow_fraction / nb, bp))
                            fail("installation " + bp + "needs a pipe, catalyst or chamber before its outlet");
                    }
                    return true;
                }
            }
        }
        fail("installation chain '" + prefix + "' does not end in an outlet or branch");
    }
};

} // namespace

std::vector<real> engine_firing_phases_deg(const CycleModelDesc& desc) {
    std::vector<int> order = desc.firing_order;
    if (order.empty()) {
        if (desc.cylinders != 4) fail("firing_order is required for a pipe engine with other than 4 cylinders");
        order = {1, 3, 4, 2};
    }
    validate_firing_order(order, desc.cylinders);
    validate_firing_angles(desc.firing_angles_deg, desc.cylinders);
    const std::vector<real> ang = resolve_firing_angles_deg(order, desc.firing_angles_deg, desc.cylinders);
    std::vector<real> ph(ang.size());
    for (std::size_t c = 0; c < ang.size(); ++c) ph[c] = wrap720(ang[c] - ang[0]);
    return ph;
}

PipeNetworkDesc make_engine_pipe_network_desc(const CycleModelDesc& desc, const GasStateDesc& ambient,
                                              real p_plenum_init_pa, real dx_m, EngineNetworkLayout* layout) {
    return make_engine_pipe_network_desc(desc, ambient, p_plenum_init_pa, dx_m, dx_m, layout);
}

PipeNetworkDesc make_engine_pipe_network_desc(const CycleModelDesc& desc, const GasStateDesc& ambient,
                                              real p_plenum_init_pa, real dx_intake_m, real dx_exhaust_m,
                                              EngineNetworkLayout* layout) {
    if (!desc.intake.has_value() || !desc.exhaust.has_value()) fail("the desc has no intake/exhaust pipe blocks");
    const drivetrain::IntakeCycleDesc& in = *desc.intake;
    const drivetrain::ExhaustCycleDesc& ex = *desc.exhaust;
    const int ncyl = desc.cylinders;
    if (!(dx_intake_m > 0.0) || !(dx_exhaust_m > 0.0)) fail("dx must be > 0");
    EngineNetworkLayout lay;

    PipeNetworkDesc nd;
    nd.gas = desc.gas;
    nd.ambient = ambient;
    const GasStateDesc intake_gas{p_plenum_init_pa, ambient.t_k, 0.0};
    const GasStateDesc exhaust_gas{ambient.p_pa, desc.exhaust_backflow_temperature_k, 1.0};

    // --- Intake ---------------------------------------------------------
    const bool inst = desc.installation.has_value();
    const bool snorkel = inst && desc.installation->intake.has_snorkel;
    if (snorkel) {
        VolumeDesc ab;
        ab.name = "airbox";
        ab.volume_m3 = desc.installation->intake.airbox_m3;
        ab.initial = ambient;
        nd.volumes.push_back(ab);
        lay.airbox_volume = 0;
        PipeDesc sn;
        sn.name = "snorkel";
        sn.length_m = desc.installation->intake.snorkel.length_m;
        sn.d_start_m = sn.d_end_m = desc.installation->intake.snorkel.d_m;
        sn.dx_target_m = dx_intake_m;
        sn.max_wave_speed_m_s = kIntakeMaxWaveSpeed;
        sn.start = PipeEndDesc{PipeEndDesc::Kind::Open, -1, EndCorrection::Unflanged};
        sn.end = PipeEndDesc{PipeEndDesc::Kind::Volume, lay.airbox_volume, EndCorrection::Unflanged};
        sn.initial = ambient;
        nd.pipes.push_back(sn);
    }
    {
        VolumeDesc pl;
        pl.name = "plenum";
        pl.volume_m3 = in.plenum_m3;
        pl.initial = intake_gas;
        nd.volumes.push_back(pl);
        lay.plenum_volume = static_cast<int>(nd.volumes.size()) - 1;
    }
    lay.throttle_cd_area_geom_m2 = area_of(in.throttle_bore_m);  // Cd 1 [AS]
    {
        OrificeDesc th;
        th.name = "throttle";
        th.volume_a = lay.plenum_volume;
        th.volume_b = lay.airbox_volume;  // -1 = ambient
        th.cd_area_m2 = 1e-9;             // the caller sets its own
        th.max_wave_speed_m_s = kIntakeMaxWaveSpeed;
        nd.orifices.push_back(th);
        lay.throttle_orifice = 0;
    }
    lay.intake_port0 = 0;
    lay.exhaust_port0 = ncyl;
    for (int c = 1; c <= ncyl; ++c) {
        PortDesc p;
        p.name = "intake port " + std::to_string(c);
        p.volume_m3 = desc.intake_port_m3;
        nd.ports.push_back(p);
    }
    for (int c = 1; c <= ncyl; ++c) {
        PortDesc p;
        p.name = "exhaust port " + std::to_string(c);
        p.volume_m3 = desc.exhaust_port_m3;
        nd.ports.push_back(p);
    }
    for (int c = 1; c <= ncyl; ++c) {
        const drivetrain::RunnerPipeDesc& r = entry_for_cylinder(in.runners, c, "runner");
        PipeDesc p;
        p.name = "runner " + std::to_string(c);
        p.length_m = r.length_m;
        p.d_start_m = r.d_in_m;
        p.d_end_m = r.d_out_m;
        p.dx_target_m = dx_intake_m;
        p.max_wave_speed_m_s = kIntakeMaxWaveSpeed;
        p.start = PipeEndDesc{PipeEndDesc::Kind::Volume, lay.plenum_volume, EndCorrection::Unflanged};  // [AS]
        p.end = PipeEndDesc{PipeEndDesc::Kind::Port, lay.intake_port0 + c - 1, EndCorrection::None};
        p.initial = intake_gas;
        nd.pipes.push_back(p);
    }

    // --- Exhaust ----------------------------------------------------------
    // One junction volume per merge (in merge order); its volume is set once
    // every connected pipe is known.
    const int nm = static_cast<int>(ex.merges.size());
    if (nm == 0 && ncyl != 1) fail("a multi-cylinder exhaust needs merges");
    const int j0 = static_cast<int>(nd.volumes.size());
    std::vector<real> junction_area(static_cast<std::size_t>(nm), 0.0);
    for (int m = 0; m < nm; ++m) {
        VolumeDesc v;
        v.name = "junction " + ex.merges[static_cast<std::size_t>(m)].into;
        v.initial = exhaust_gas;
        nd.volumes.push_back(v);
    }
    // The merge consuming a cylinder / a named merge output.
    auto consumer_of_cyl = [&](int c) {
        for (int m = 0; m < nm; ++m)
            for (const drivetrain::MergeSourceRef& f : ex.merges[static_cast<std::size_t>(m)].from)
                if (f.is_cylinder && f.cylinder == c) return m;
        return -1;
    };
    auto consumer_of_name = [&](const std::string& n) {
        for (int m = 0; m < nm; ++m)
            for (const drivetrain::MergeSourceRef& f : ex.merges[static_cast<std::size_t>(m)].from)
                if (!f.is_cylinder && f.name == n) return m;
        return -1;
    };
    // Flange end: a waveguide Junction (installation) or an open end.
    WgBuilder wg;
    wg.wd = &nd.waveguide;
    wg.gas = exhaust_gas;
    PipeEndDesc flange_end{PipeEndDesc::Kind::Open, -1, EndCorrection::Unflanged};
    if (inst) {
        const int j = wg.add_node(WgNodeDesc::Kind::Junction, "flange");
        if (wg.add_chain(desc.installation->exhaust.elements, j, 1.0, "exhaust ")) {
            lay.flange_node = j;
            flange_end = PipeEndDesc{PipeEndDesc::Kind::Waveguide, j, EndCorrection::None};
        } else {
            nd.waveguide = WaveguideNetworkDesc{};  // the installation is just an outlet at the flange
        }
    }
    auto end_into = [&](int m, real area) {
        junction_area[static_cast<std::size_t>(m)] += area;
        return PipeEndDesc{PipeEndDesc::Kind::Volume, j0 + m, EndCorrection::None};
    };
    for (int c = 1; c <= ncyl; ++c) {
        const drivetrain::PrimaryPipeDesc& pr = entry_for_cylinder(ex.primaries, c, "primary");
        PipeDesc p;
        p.name = "primary " + std::to_string(c);
        p.length_m = pr.length_m;
        p.d_start_m = p.d_end_m = pr.d_m;
        p.dx_target_m = dx_exhaust_m;
        p.max_wave_speed_m_s = kExhaustMaxWaveSpeed;
        p.start = PipeEndDesc{PipeEndDesc::Kind::Port, lay.exhaust_port0 + c - 1, EndCorrection::None};
        if (nm == 0) {
            p.end = flange_end;
        } else {
            const int m = consumer_of_cyl(c);
            if (m < 0) fail("cylinder " + std::to_string(c) + " feeds no merge");
            p.end = end_into(m, area_of(pr.d_m));
        }
        p.initial = exhaust_gas;
        nd.pipes.push_back(p);
    }
    for (int m = 0; m < nm; ++m) {
        const drivetrain::MergeCycleDesc& md = ex.merges[static_cast<std::size_t>(m)];
        PipeDesc p;
        p.name = "merge " + md.into;
        p.length_m = md.length_m;
        p.d_start_m = p.d_end_m = md.d_m;
        p.dx_target_m = dx_exhaust_m;
        p.max_wave_speed_m_s = kExhaustMaxWaveSpeed;
        junction_area[static_cast<std::size_t>(m)] += area_of(md.d_m);
        p.start = PipeEndDesc{PipeEndDesc::Kind::Volume, j0 + m, EndCorrection::None};
        if (m == nm - 1) {
            if (md.into != ex.interface) fail("the last merge's into must be the exhaust interface");
            p.end = flange_end;
        } else {
            const int next = consumer_of_name(md.into);
            if (next <= m) fail("merge '" + md.into + "' is not consumed by a later merge");
            p.end = end_into(next, area_of(md.d_m));
        }
        p.initial = exhaust_gas;
        nd.pipes.push_back(p);
    }
    for (int m = 0; m < nm; ++m)
        nd.volumes[static_cast<std::size_t>(j0 + m)].volume_m3 =
            kJunctionLengthM * junction_area[static_cast<std::size_t>(m)];  // PROVISIONAL P8 [AS]

    if (layout != nullptr) *layout = lay;
    return nd;
}

namespace {

std::uint64_t validated_step_count(const CycleModelDesc& desc, const OperatingPoint& op,
                                   const CycleEngineSettings& settings) {
    if (!desc.intake.has_value() || !desc.exhaust.has_value()) fail("the desc has no intake/exhaust pipe blocks");
    if (desc.cylinders < 1 || desc.cylinders > kCycleEngineMaxCylinders)
        fail("cylinders must be 1.." + std::to_string(kCycleEngineMaxCylinders));
    if (!(op.rpm > 0.0 && op.p_im_pa > 0.0 && op.t_im_k > 0.0 && op.p_amb_pa > 0.0))
        fail("operating point rpm, p_im, t_im and p_amb must be > 0");
    if (!(settings.h_s > 0.0) || settings.settling_cycles < 0) fail("h_s must be > 0 and settling_cycles >= 0");
    const real nn = std::ceil(120.0 / (op.rpm * settings.h_s));
    return nn < 1.0 ? 1u : static_cast<std::uint64_t>(nn);
}

EngineCoreSettings core_settings(const CycleModelDesc& desc, const CycleEngineSettings& settings) {
    const real dx = settings.dx_m > 0.0 ? settings.dx_m : desc.resolution.power_dx_m;
    EngineCoreSettings c;
    c.dx_intake_m = dx;
    c.dx_exhaust_m = dx;
    c.reservoir_ports = settings.reservoir_ports;
    return c;
}

} // namespace

EngineCore::EngineCore(const CycleModelDesc& desc, real lhv_j_kg, real afr_st, const OperatingPoint& op, real h,
                       const EngineCoreSettings& settings)
    : cylinders_(desc.cylinders), h_(h) {
    if (!desc.intake.has_value() || !desc.exhaust.has_value()) fail("the desc has no intake/exhaust pipe blocks");
    if (cylinders_ < 1 || cylinders_ > kCycleEngineMaxCylinders)
        fail("cylinders must be 1.." + std::to_string(kCycleEngineMaxCylinders));
    if (!(op.p_im_pa > 0.0 && op.t_im_k > 0.0 && op.p_amb_pa > 0.0 && h > 0.0))
        fail("operating point p_im, t_im, p_amb and the step h must be > 0");
    phase_ = engine_firing_phases_deg(desc);

    const CycleStepper proto(desc, lhv_j_kg, afr_st);
    steppers_.assign(static_cast<std::size_t>(cylinders_), proto);
    for (int c = 0; c < cylinders_; ++c) steppers_[static_cast<std::size_t>(c)].reset_phased(op, phase_[static_cast<std::size_t>(c)]);

    if (settings.reservoir_ports) {
        for (int c = 1; c <= cylinders_; ++c) {
            const real ai = area_of(entry_for_cylinder(desc.intake->runners, c, "runner").d_out_m);
            const real ae = area_of(entry_for_cylinder(desc.exhaust->primaries, c, "primary").d_m);
            steppers_[static_cast<std::size_t>(c - 1)].set_port_reservoirs(ai, ae);  // Cd 1 [AS]
        }
        return;
    }

    const GasStateDesc ambient{op.p_amb_pa, op.t_im_k, 0.0};
    const PipeNetworkDesc nd = make_engine_pipe_network_desc(desc, ambient, op.p_im_pa, settings.dx_intake_m,
                                                             settings.dx_exhaust_m, &layout_);
    net_ = std::make_unique<PipeNetwork>(nd, h_);
    cda_max_ = std::min(layout_.throttle_cd_area_geom_m2, net_->orifice_max_cd_area(layout_.throttle_orifice));
    // Feed-forward start: VE 0.9 [A] at the target pressure through a steady orifice.
    const GasModel& g = net_->gas();
    const real v_d = area_of(desc.bore_m) * desc.stroke_m;
    const real m_cyc = 0.9 * op.p_im_pa / (g.fresh.r * op.t_im_k) * v_d * static_cast<real>(cylinders_);
    const real rpm_ff = op.rpm > 0.0 ? op.rpm : 1.0;  // A7: a live engine may start at rest
    const real t_cycle = 120.0 / rpm_ff;
    const real pr = std::min(op.p_im_pa / op.p_amb_pa, 0.999);
    const real g_unit = op.p_amb_pa / std::sqrt(g.fresh.r * op.t_im_k) * proto.model().psi.eval(pr, 0.0);
    cda_ = std::clamp(m_cyc / t_cycle / g_unit, 1e-9, cda_max_);
    v_intake_ = desc.intake->plenum_m3;
    for (int c = 1; c <= cylinders_; ++c) {
        const drivetrain::RunnerPipeDesc& rp = entry_for_cylinder(desc.intake->runners, c, "runner");
        v_intake_ += kPi * rp.length_m / 12.0 * (rp.d_in_m * rp.d_in_m + rp.d_in_m * rp.d_out_m + rp.d_out_m * rp.d_out_m);
    }
    net_->set_orifice_cd_area(layout_.throttle_orifice, cda_);

    // A7: radiating ends (FV ends open to ambient, then waveguide Open nodes).
    const auto is_intake_volume = [&](const PipeEndDesc& e) {
        return e.kind == PipeEndDesc::Kind::Volume && (e.index == layout_.airbox_volume || e.index == layout_.plenum_volume);
    };
    for (std::size_t p = 0; p < nd.pipes.size(); ++p) {
        const PipeDesc& pd = nd.pipes[p];
        for (int side = 0; side < 2; ++side) {
            const PipeEndDesc& e = side == 0 ? pd.start : pd.end;
            if (e.kind != PipeEndDesc::Kind::Open) continue;
            RadiatingEnd r;
            r.kind = RadiatingEnd::Kind::PipeEnd;
            r.name = pd.name + (side == 0 ? " (start)" : " (end)");
            r.index = static_cast<int>(p);
            r.at_start = side == 0;
            r.area_m2 = area_of(side == 0 ? pd.d_start_m : pd.d_end_m);
            r.intake = is_intake_volume(side == 0 ? pd.end : pd.start);
            rad_.push_back(r);
        }
    }
    for (std::size_t n = 0; n < nd.waveguide.nodes.size(); ++n) {
        const WgNodeDesc& wn = nd.waveguide.nodes[n];
        if (wn.kind != WgNodeDesc::Kind::Open) continue;
        RadiatingEnd r;
        r.kind = RadiatingEnd::Kind::WaveguideOpen;
        r.name = wn.name;
        r.index = static_cast<int>(n);
        // A10: an Open node's own desc carries no diameter (it is the far end
        // of the segment that terminates there, e.g. WgBuilder::add_chain's
        // Outlet case just relabels the last Junction node) - find that
        // segment for display purposes only; radiating_outflow() itself reads
        // the network's live state, not area_m2, so this never affected the
        // physics, only the printed source list.
        for (const WgSegmentDesc& seg : nd.waveguide.segments) {
            if (seg.node_a == static_cast<int>(n) || seg.node_b == static_cast<int>(n)) {
                r.area_m2 = area_of(seg.diameter_m);
                break;
            }
        }
        rad_.push_back(r);
    }
}

EngineCore::~EngineCore() = default;

real EngineCore::radiating_outflow(int k) const {
    const RadiatingEnd& r = rad_[static_cast<std::size_t>(k)];
    if (r.kind == RadiatingEnd::Kind::WaveguideOpen) return net_->waveguide().node_outflow(r.index);
    const int i = r.at_start ? 0 : net_->cell_count(r.index) - 1;
    const real u = net_->cell(r.index, i).u;
    return (r.at_start ? -u : u) * r.area_m2;
}

void EngineCore::set_throttle_cd_area(real cd_area_m2) {
    cda_ = std::clamp(cd_area_m2, 1e-9, cda_max_);
    if (net_) net_->set_orifice_cd_area(layout_.throttle_orifice, cda_);
}

void EngineCore::exchange() {
    if (!net_) return;
    const std::size_t nc = static_cast<std::size_t>(cylinders_);
    const GasModel& g = steppers_[0].model().gas;
    const real vip = steppers_[0].model().v_ip, vep = steppers_[0].model().v_ep;
    for (std::size_t c = 0; c < nc; ++c) {
        const real* y = steppers_[c].state().y;
        const CvState ip = cv_state(g, y[kIpM], y[kIpU], y[kIpMb], vip);
        const CvState ep = cv_state(g, y[kEpM], y[kEpU], y[kEpMb], vep);
        net_->set_port_state(layout_.intake_port0 + static_cast<int>(c), PortState{ip.p, ip.t, ip.x});
        net_->set_port_state(layout_.exhaust_port0 + static_cast<int>(c), PortState{ep.p, ep.t, ep.x});
    }
    net_->step();
    const real inv_h = 1.0 / h_;
    for (std::size_t c = 0; c < nc; ++c) {
        const PortFlux& fi = net_->port_flux(layout_.intake_port0 + static_cast<int>(c));
        const PortFlux& fe = net_->port_flux(layout_.exhaust_port0 + static_cast<int>(c));
        steppers_[c].set_port_sources(PortSource{-fi.mass_kg * inv_h, -fi.energy_j * inv_h, -fi.burned_kg * inv_h},
                                      PortSource{-fe.mass_kg * inv_h, -fe.energy_j * inv_h, -fe.burned_kg * inv_h});
    }
}

real EngineCore::total_mass() const {
    real m = net_ ? net_->total_mass() : 0.0;
    for (const CycleStepper& s : steppers_) {
        const real* y = s.state().y;
        m += y[kCylM] + y[kIpM] + y[kEpM];
    }
    return m;
}

std::uint64_t EngineCore::fold_state_hash(std::uint64_t h) const {
    if (net_) h = net_->fold_state_hash(h);
    for (const CycleStepper& s : steppers_) {
        for (real v : s.state().y) fnv_fold(h, v);
        fnv_fold(h, s.crank_deg());
    }
    fnv_fold(h, cda_);
    return h;
}

CycleEngine::CycleEngine(const CycleModelDesc& desc, real lhv_j_kg, real afr_st, const OperatingPoint& op,
                         const CycleEngineSettings& settings)
    : op_(op),
      settings_(settings),
      n_(validated_step_count(desc, op, settings)),
      core_(desc, lhv_j_kg, afr_st, op, 120.0 / (op.rpm * static_cast<real>(n_)), core_settings(desc, settings)) {}

CycleEngine::~CycleEngine() = default;

void CycleEngine::step_all(std::uint64_t k) {
    core_.exchange();
    const int nc = core_.cylinders();
    for (int c = 0; c < nc; ++c) core_.cylinder(c).step_phased(k, n_, core_.phase_deg(c));
}

CycleEngineResult CycleEngine::run() {
    if (used_) throw std::logic_error("CycleEngine: run() is single use");
    used_ = true;
    const std::size_t nc = static_cast<std::size_t>(core_.cylinders());
    PipeNetwork* const net = core_.network();
    const EngineNetworkLayout& layout = core_.layout();
    const int cycles = settings_.settling_cycles + 1;
    const real p_t = op_.p_im_pa;
    const PsiPair psi = core_.cylinder(0).model().psi;
    const real r_amb = core_.cylinder(0).model().gas.fresh.r;
    real air_prev = 0.0, air_last = 0.0;
    real m0 = 0.0, x0 = 0.0, pl_mean = p_t;
    std::uint64_t k = 0;
    std::array<real, CycleEngineResult::kTraceCycles> trace_p{}, trace_c{};
    for (int cyc = 0; cyc < cycles; ++cyc) {
        const bool last = cyc == cycles - 1;
        for (std::size_t c = 0; c < nc; ++c) {
            CycleStepper& s = core_.cylinder(static_cast<int>(c));
            s.reset_accumulators();
            if (last) s.reset_peak_pressure();
        }
        if (last) {
            m0 = core_.total_mass();
            x0 = net ? (net->exchange().ambient_mass_kg + net->exchange().waveguide_mass_kg +
                         net->exchange().source_mass_kg)
                      : 0.0;
        }
        real sum_pl = 0.0, sum_up = 0.0, sum_g = 0.0;
        for (std::uint64_t i = 0; i < n_; ++i, ++k) {
            if (net) {
                // Unit-Cd*A throttle flow at this step's start states (what the
                // network's explicit orifice update sees).
                const CvState pl = net->volume_state(layout.plenum_volume);
                if (layout.airbox_volume >= 0) {
                    const CvState ab = net->volume_state(layout.airbox_volume);
                    sum_g += orifice_flow(psi, 1.0, ab.p, ab.r, ab.t, ab.x, pl.p, pl.r, pl.t, pl.x).mdot;
                } else {
                    sum_g += orifice_flow(psi, 1.0, op_.p_amb_pa, r_amb, op_.t_im_k, 0.0, pl.p, pl.r, pl.t, pl.x).mdot;
                }
            }
            step_all(k);
            if (net) {
                sum_pl += net->volume_state(layout.plenum_volume).p;
                sum_up += layout.airbox_volume >= 0 ? net->volume_state(layout.airbox_volume).p : op_.p_amb_pa;
            }
        }
        real air = 0.0;
        for (std::size_t c = 0; c < nc; ++c) air += core_.cylinder(static_cast<int>(c)).state().y[kAccIvFresh];
        air_prev = air_last;
        if (cyc < CycleEngineResult::kTraceCycles) {
            trace_p[static_cast<std::size_t>(cyc)] = net ? sum_pl / static_cast<real>(n_) : p_t;
            trace_c[static_cast<std::size_t>(cyc)] = core_.throttle_cd_area();
        }
        air_last = air;
        if (net) {
            const real rn = static_cast<real>(n_);
            pl_mean = sum_pl / rn;
            const real p_up = sum_up / rn;
            if (!last) {
                // Throttle trim (PROVISIONAL P11): the flow the engine will draw
                // at the target (measured valve flow scaled by p_t / p_mean,
                // speed-density) plus half the mass that refills the intake
                // volume from its cycle-mean pressure to the target within
                // one cycle, over the measured mean unit-Cd*A orifice flow
                // moved to the target by the steady Psi ratio. Both terms
                // vanish only at p_mean == p_t (an end-of-cycle sample in the
                // refill term left a +2.5..3 % fixed-point bias at 6000 rpm,
                // where the sample sits in a pulsation trough).
                const real t_cycle = 120.0 / op_.rpm;
                real m_eng = 0.0;
                for (std::size_t c = 0; c < nc; ++c) m_eng += core_.cylinder(static_cast<int>(c)).state().y[kAccIvMass];
                const real flow_req = m_eng / t_cycle * (p_t / pl_mean) +
                                      0.5 * (p_t - pl_mean) * core_.intake_volume_m3() / (r_amb * op_.t_im_k) / t_cycle;  // gain 0.5 [A]
                const real psi_bar = psi.eval(std::clamp(pl_mean / p_up, 0.0, 0.999), 0.0);
                const real psi_t = psi.eval(std::clamp(p_t / p_up, 0.0, 0.999), 0.0);
                const real g_t = sum_g / rn * (psi_t / psi_bar);
                real cda = g_t > 0.0 ? flow_req / g_t : core_.throttle_cd_area_max();
                cda = std::clamp(cda, 0.5 * core_.throttle_cd_area(), 2.0 * core_.throttle_cd_area());
                core_.set_throttle_cd_area(cda);  // clamp(cda, 1e-9, max), pushed to the orifice
            }
        }
    }

    CycleEngineResult r;
    r.cylinders = core_.cylinders();
    r.steps_per_cycle = n_;
    r.h_s = core_.h();
    r.p_target_pa = p_t;
    r.p_plenum_mean_pa = pl_mean;
    r.plenum_error_rel = (pl_mean - p_t) / p_t;
    r.trace_plenum_mean_pa = trace_p;
    r.trace_cd_area_m2 = trace_c;
    r.throttle_cd_area_m2 = core_.throttle_cd_area();
    r.throttle_cd_area_max_m2 = core_.throttle_cd_area_max();
    r.throttle_saturated = net && core_.throttle_cd_area() >= core_.throttle_cd_area_max() && r.plenum_error_rel < -settings_.trim_tolerance;
    r.air_change_rel = air_last > 0.0 ? std::fabs(air_last - air_prev) / air_last : 0.0;

    real fuel = 0.0, res_in = 0.0, ev_m = 0.0, ev_mt = 0.0;
    PointResult& mr = r.mean;
    for (std::size_t c = 0; c < nc; ++c) {
        const CycleStepper& s = core_.cylinder(static_cast<int>(c));
        const real* y = s.state().y;
        fuel += y[kAccFuel];
        res_in += y[kAccRunnerMass] - y[kAccResMass];
        ev_m += y[kAccEvOutMass];
        ev_mt += y[kAccEvOutMassT];
        r.air_kg[c] = y[kAccIvFresh];
        mr.w_gross_j += y[kAccWorkGross];
        mr.w_pump_j += y[kAccWork] - y[kAccWorkGross];
        mr.m_air_j_cyc += y[kAccIvFresh];
        mr.m_fuel_kg += s.m_fuel();
        mr.residual_frac += s.residual_fraction_ivc();
        mr.h_exh_j += y[kAccEvEnthalpy];
        mr.p_max_pa = std::max(mr.p_max_pa, s.p_max());
    }
    const real inv = 1.0 / static_cast<real>(nc);
    mr.w_gross_j *= inv;
    mr.w_pump_j *= inv;
    mr.m_air_j_cyc *= inv;
    mr.m_fuel_kg *= inv;
    mr.residual_frac *= inv;
    mr.h_exh_j *= inv;
    mr.t_exh_k = ev_m > 0.0 ? ev_mt / ev_m : 0.0;
    mr.ca50_deg = core_.cylinder(0).wiebe().angle_at_fraction(0.5) - 360.0;
    mr.steps = k;

    const real x1 = net ? (net->exchange().ambient_mass_kg + net->exchange().waveguide_mass_kg +
                            net->exchange().source_mass_kg)
                         : 0.0;
    const real exch = net ? (x1 - x0) : res_in;
    r.mass_closure_kg = (core_.total_mass() - m0) - exch - fuel;
    r.mass_scale_kg = mr.m_air_j_cyc * static_cast<real>(nc);
    return r;
}

} // namespace ps::engine_cycle
