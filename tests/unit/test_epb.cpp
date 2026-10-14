// tests/unit/test_epb.cpp - queue item Q3 / assists I8 (docs/next_steps_brief.md 2.1): the Racing drive mode (DriveMode
// Racing = 3, appended) and the electronic parking brake (electronics kind `epb`, channel `epb`, per-drive-mode `epb_mode`
// park_only | handbrake).
//
// Owner rule, verbatim (relayed by the racing_game cockpit plan): "No, unless special modes are used (racing, drift), the
// parking brake should refuse to engage while speed is significant."
//
// Law under test (VehicleInstance::step_tick, docs/data_formats/electronics.md "epb"):
//   level = channel epb >= 0.5; speed = |chassis velocity| at the tick start; mode = the effective drive mode's epb_mode,
//   else the part's epb_mode.
//   - level 0: released (state 0), refusal cleared.
//   - a refused request stays dropped while the level is held (in every mode): it engages only on a NEW rising edge.
//   - handbrake mode: state 2 = the rear handbrake rows driven at full (handbrake = max(channel, 1)) at any speed.
//   - park_only: a rising edge below max_engage_speed_mps latches (state 1, the same rows at full); at or above it the request
//     is refused (state 0, brakes untouched, snapshot epb_refused). A latched brake stays latched. A handbrake-mode
//     application that meets park_only by a mode switch is a new request: latched below the threshold, else refused.
//
// BANDS COMMITTED BEFORE THE FIRST RUN (all [AS] unless marked):
//   L1  the shipped part data/electronics/epb_generic.json: kind Epb, max_engage_speed_mps 2.0, epb_mode park_only;
//       rejections (unknown field, missing / non-positive / non-finite max_engage_speed_mps, unknown epb_mode) name the field;
//       a drive_modes epb_mode other than park_only / handbrake is rejected
//   L2  car_hyper_n2o's part: modes eco/normal/sport/racing (bits 0b1111); Racing: TC default sport, LC default true and
//       allowed, nitrous allowed, auto-shift up 4500 / 8100, down 2500 / 5500 rpm, close 0.04 s, epb_mode handbrake; the
//       other three modes epb_mode park_only. car_hyper's new part: the same four modes, Normal without any override.
//       A car with an epb part but no handbrake rows, and a car with two epb parts, fail to load
//   C1  channel `epb` registered (default 0) only on a car with the part (car_hyper, car_hyper_n2o); car_sedan has none;
//       snapshot assist_fitted_bits bit 7 = EPB fitted (0 on the sedan), assist_active_bits bit 7 = applied
//   E1  park_only (Normal) refuse / drop / engage on car_hyper_n2o in neutral on flat asphalt:
//       a rising edge at 2.5 m/s is refused in that tick (epb_refused, state 0); at 1.5 m/s it latches in that tick (state 1).
//       Refused at 10 m/s: the chassis position and velocity are bit-identical, every tick for 1 s, to the same run without
//       the epb write (brakes untouched). Level held while the car is braked to rest (service brake 1): state stays 0 every
//       tick (dropped, not pended). Level 0 clears the refusal; a new rising edge at rest latches (state 1) in that tick
//   E2  handbrake mode (Racing) at 20 m/s: state 2 and no refusal in the tick of the rising edge; the chassis position and
//       velocity are bit-identical every tick for 2 s to the run with the handbrake channel at 1 instead; speed after 2 s at
//       least 1 m/s below the free-coast run (non-vacuous)
//   E3  mode switch with the level held: Racing state 2 at 20 m/s -> Normal: state 0 and refused in that tick; at rest
//       Normal 1 -> Racing 2 -> Normal 1 (each in the tick of the switch); refused in Normal at 10 m/s -> Racing: stays 0
//       (dropped) until the level falls and rises again, then 2
//   E4  1 vs 10 workers: equal state hash every tick through a run with EPB requests, a refusal and mode switches; zero own
//       allocation per tick (debug_alloc)
//   E5  EPB state is hashed on a car with the part: two runs that differ only in a latched-at-rest EPB have different hashes
//       from that tick on (and equal before)
//   R1  car_hyper with its new part in Normal vs the part removed ("none"): chassis pose and velocity bit-identical every tick
//       of a 3 s WOT auto-shift launch (Normal overrides nothing); Racing selected: the TC channel is written to 3 (sport)
//       and assist.launch_control to 1 by the mode change
//
// SABOTAGES (temporary env-switched edits in core/src, reverted; recorded below after the runs):
//   1 park_only speed gate ignored             -> E1 must fail
//   2 a refused request pends (engages once slow) -> E1 must fail
//   3 handbrake mode does not apply the rows     -> E2 must fail
//   4 a mode switch keeps the EPB state          -> E3 must fail
//   5 EPB state not folded into the hash         -> E5 must fail
