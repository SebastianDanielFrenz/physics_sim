// core/src/drivetrain/throttle_stage.h — the throttle-command stage between
// the pedal and an engine (PHYS-033 P4 I7a C3, plan section 3.4). A TYPED
// Powertrain member (like engine_/clutch_/gearbox_ below - see
// components.h's own header comment for why these live outside the generic
// IComponent catalogue), not a generic IComponent: it sits strictly BEFORE
// the engine's own pre_solve in Powertrain::pre_solve_components's fixed
// substep order (plan section 7.2 step 4, "engine"), and has no shaft-
// network port/node/row of its own - a pure control-signal seam.
//
// Arithmetic (plan section 3.4.4, verbatim):
//   pedal'          = has_curve ? curve(pedal) : pedal            // plain copy when absent
//   command         = hand lever present ? max(clamp(hand,0,1), clamp(pedal',0,1)) : pedal'
//   floor, ceiling  = electronic ? (ctx.tcu_throttle_floor, ctx.tcu_throttle_ceiling) : (0, 1)
//
// This is the ONLY place the pedal/hand-lever combination happens; the
// engine (engine_torque_map.cpp) never reads ctx.throttle or a hand-lever
// channel directly any more - it only ever sees this stage's ThrottleCommand
// (engine.h). Floor/ceiling are carried through to the engine, which still
// applies them AFTER its own governor (unchanged from before this commit) -
// see engine_torque_map.cpp's own comment for why that clamp order must not
// move.
//
// "Which TCU intervention lives where" (plan section 3.4.5, the loader's own
// rejection list in vehicle_io.cpp encodes the same table):
//   pedal_map / linkage_curve        -> this stage (has_curve/curve above)
//   hand lever                        -> this stage (hand_lever_channel_slot_)
//   auto-blip floor / pedal ceiling   -> TCU -> this stage, ELECTRONIC ONLY
//                                        (a mechanical linkage has no wire
//                                        back to the pedal, so it cannot
//                                        carry a blip or a post-shift ramp -
//                                        floor/ceiling are fixed 0/1 instead)
//   upshift ignition cut              -> TCU -> engine ignition directly;
//                                        works on BOTH kinds (it never goes
//                                        through the throttle path at all)
//   idle PI / limiter / DFCO /
//   governor / starter                -> engine management; unchanged,
//                                        works on BOTH kinds

#pragma once

#include "components.h" // ComponentSubstepCtx
#include "engine.h"      // ThrottleCommand

#include "ps/drivetrain/powertrain_desc.h" // ThrottleInput
#include "ps/vehicle/curve1d.h"
#include "ps/vehicle/drive_modes_desc.h" // pedal_bend_apply (assists I7)

#include <algorithm>

namespace ps::drivetrain {

class ThrottleStage {
public:
    // Build-time only (Powertrain's constructor), never per-tick/substep.
    // `hand_lever_channel_slot` is the already-resolved index into
    // ComponentSubstepCtx::channels (-1 = no hand lever for this stage).
    void configure(ThrottleInput kind, bool has_curve, const vehicle::Curve1D& curve, int hand_lever_channel_slot) {
        kind_ = kind;
        has_curve_ = has_curve;
        curve_ = curve;
        hand_lever_channel_slot_ = hand_lever_channel_slot;
    }

    [[nodiscard]] bool has_hand_lever() const { return hand_lever_channel_slot_ >= 0; }

    // The raw hand-lever channel value this substep (0 when this stage has
    // no hand lever) - Powertrain::governor_hand_throttle()'s replacement
    // source (PowertrainSnapshot::hand_throttle), kept separate from
    // evaluate()'s combined command since the two are different published
    // quantities (see powertrain_state.h's own field comments).
    [[nodiscard]] real raw_hand_lever(const ComponentSubstepCtx& ctx) const {
        if (hand_lever_channel_slot_ < 0 || ctx.channels == nullptr || hand_lever_channel_slot_ >= ctx.channel_count) {
            return real(0.0);
        }
        return ctx.channels[hand_lever_channel_slot_];
    }

    [[nodiscard]] ThrottleCommand evaluate(const ComponentSubstepCtx& ctx) const {
        real unused_raw_hand_lever = real(0.0);
        return evaluate(ctx, unused_raw_hand_lever);
    }

    // Same as evaluate(ctx), additionally writing raw_hand_lever(ctx) into
    // out_raw_hand_lever (0 without a hand lever) - one read of the channel
    // serving both the command and Powertrain's published hand_throttle.
    [[nodiscard]] ThrottleCommand evaluate(const ComponentSubstepCtx& ctx, real& out_raw_hand_lever) const {
        real pedal_prime = has_curve_ ? curve_.evaluate(ctx.throttle) : ctx.throttle;
        if (ctx.drive_bend_x > real(0.0)) { // assists I7: the drive mode's one-point bend, on top of the car's curve
            pedal_prime = vehicle::pedal_bend_apply(vehicle::PedalBend{ctx.drive_bend_x, ctx.drive_bend_y}, pedal_prime);
        }
        real command = pedal_prime;
        out_raw_hand_lever = raw_hand_lever(ctx);
        if (has_hand_lever()) {
            const real hand = out_raw_hand_lever;
            command = std::max(std::clamp(hand, real(0.0), real(1.0)), std::clamp(pedal_prime, real(0.0), real(1.0)));
        }
        ThrottleCommand out;
        out.command = command;
        if (kind_ == ThrottleInput::Electronic) {
            out.floor = ctx.tcu_throttle_floor;
            out.ceiling = std::min(std::min(ctx.tcu_throttle_ceiling,ctx.traction_throttle_ceiling),ctx.launch_throttle_ceiling); // launch_*: assists I4, 1 = none
            out.floor = std::min(out.floor,out.ceiling); // safety intervention takes priority over auto-blip
        } else {
            out.floor = real(0.0);
            out.ceiling = real(1.0);
        }
        return out;
    }

private:
    ThrottleInput kind_ = ThrottleInput::Electronic;
    bool has_curve_ = false;
    vehicle::Curve1D curve_;
    int hand_lever_channel_slot_ = -1; // into ComponentSubstepCtx::channels
};

} // namespace ps::drivetrain
