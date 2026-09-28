// tests/bench/bench_audio_voice.cpp - ps_audio A7 budget measurement (plain
// main, NOT CTest-registered: a measurement tool whose numbers feed the A7
// report and the A9 brief). Links ps_engine_cycle only.
//
// Part 1 ("split"): the audio-grade fixture I4 (tests/support/
// cycle_pipe_fixtures.h, dx 32 mm exhaust / 17 mm intake, h = 1/48000 s)
// stepped exactly like CycleEngine::step_all, timing the cylinders (4 RK4
// steppers), the PipeNetwork step (FV + waveguide) and - separately - a
// stand-alone copy of the same waveguide, so FV = network - waveguide.
//
// Usage: bench_audio_voice [rpm] [seconds]
#include "cycle_pipe_fixtures.h"
#include "ps/engine_cycle/cycle_engine.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/pipe_network.h"
#include "ps/engine_cycle/waveguide_network.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace ps;
using namespace ps::engine_cycle;
using Clock = std::chrono::steady_clock;

int main(int argc, char** argv) {
    const real rpm = argc > 1 ? std::atof(argv[1]) : 3000.0;
    const real seconds = argc > 2 ? std::atof(argv[2]) : 1.0;
    const real fs = 48000.0;
    const drivetrain::CycleModelDesc desc = test::make_pipe_i4_cycle_desc();
    OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = 60000.0;
    op.t_im_k = 298.15;
    op.p_amb_pa = 101325.0;
    op.lambda = 1.0;
    op.spark_deg_atdc = -20.0;
    const std::uint64_t n = static_cast<std::uint64_t>(120.0 / (rpm / fs) + 0.5);  // steps per cycle
    const real h = 120.0 / (rpm * static_cast<real>(n));

    const std::vector<real> phase = engine_firing_phases_deg(desc);
    const CycleStepper proto(desc, 43.0e6, 14.6);
    std::vector<CycleStepper> cyl(4, proto);
    for (int c = 0; c < 4; ++c) cyl[c].reset_phased(op, phase[c]);

    EngineNetworkLayout lay;
    const GasStateDesc amb{op.p_amb_pa, op.t_im_k, 0.0};
    PipeNetworkDesc nd = make_engine_pipe_network_desc(desc, amb, op.p_im_pa, desc.resolution.audio_dx_intake_m, &lay);
    for (PipeDesc& p : nd.pipes)
        if (p.max_wave_speed_m_s > kIntakeMaxWaveSpeed) p.dx_target_m = desc.resolution.audio_dx_exhaust_m;
    PipeNetwork net(nd, h);
    net.set_orifice_cd_area(lay.throttle_orifice, 4e-4);
    WaveguideNetworkDesc wd = nd.waveguide;
    wd.gas = nd.gas;
    WaveguideNetwork wg(wd, h);

    int cells_in = 0, cells_ex = 0;
    for (int p = 0; p < net.pipe_count(); ++p)
        (nd.pipes[static_cast<std::size_t>(p)].max_wave_speed_m_s > kIntakeMaxWaveSpeed ? cells_ex : cells_in) += net.cell_count(p);

    const std::uint64_t steps = static_cast<std::uint64_t>(seconds * fs);
    double t_cyl = 0.0, t_net = 0.0, t_wg = 0.0;
    const GasModel& g = cyl[0].model().gas;
    const real vip = cyl[0].model().v_ip, vep = cyl[0].model().v_ep;
    for (std::uint64_t k = 0; k < steps; ++k) {
        const auto a = Clock::now();
        for (int c = 0; c < 4; ++c) {
            const real* y = cyl[c].state().y;
            const CvState ip = cv_state(g, y[kIpM], y[kIpU], y[kIpMb], vip);
            const CvState ep = cv_state(g, y[kEpM], y[kEpU], y[kEpMb], vep);
            net.set_port_state(lay.intake_port0 + c, PortState{ip.p, ip.t, ip.x});
            net.set_port_state(lay.exhaust_port0 + c, PortState{ep.p, ep.t, ep.x});
        }
        net.step();
        for (int c = 0; c < 4; ++c) {
            const PortFlux& fi = net.port_flux(lay.intake_port0 + c);
            const PortFlux& fe = net.port_flux(lay.exhaust_port0 + c);
            cyl[c].set_port_sources(PortSource{-fi.mass_kg / h, -fi.energy_j / h, -fi.burned_kg / h},
                                    PortSource{-fe.mass_kg / h, -fe.energy_j / h, -fe.burned_kg / h});
        }
        const auto b = Clock::now();
        for (int c = 0; c < 4; ++c) cyl[c].step_phased(k, n, phase[c]);
        const auto e = Clock::now();
        wg.step();
        const auto f = Clock::now();
        t_net += std::chrono::duration<double>(b - a).count();
        t_cyl += std::chrono::duration<double>(e - b).count();
        t_wg += std::chrono::duration<double>(f - e).count();
    }
    const double us = 1e6 / static_cast<double>(steps);
    const double budget = 1e6 / fs;
    std::printf("audio-grade fixture I4 at %.0f rpm, h = %.3f us (%llu steps/cycle), %llu steps\n", rpm, h * 1e6,
                static_cast<unsigned long long>(n), static_cast<unsigned long long>(steps));
    std::printf("cells: intake %d, exhaust %d, total %d; waveguide segments %d\n", cells_in, cells_ex,
                net.total_cell_count(), wg.segment_count());
    std::printf("per sample: cylinders %.3f us, network %.3f us (FV %.3f + WG %.3f), total %.3f us; "
                "real-time budget %.3f us -> %.2f of one core\n",
                t_cyl * us, t_net * us, (t_net - t_wg) * us, t_wg * us, (t_cyl + t_net) * us, budget,
                (t_cyl + t_net) * us / budget);
    std::printf("FV per cell-step %.1f ns; cylinder per step %.3f us\n",
                (t_net - t_wg) * us * 1000.0 / net.total_cell_count(), t_cyl * us / 4.0);
    std::printf("check: plenum p %.0f Pa, cyl1 p %.0f Pa\n", net.volume_state(lay.plenum_volume).p,
                cv_state(g, cyl[0].state().y[kCylM], cyl[0].state().y[kCylU], cyl[0].state().y[kCylMb],
                         cyl[0].model().geom.eval(cyl[0].crank_deg()).v).p);
    return 0;
}
