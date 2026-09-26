"""Render an engine-rpm trace (CSV) as a crude rotary-engine exhaust sound (WAV).

Dev-only listening aid for the driveline shuffle after a throttle lift-off
(tests/validation/test_vehicle_787b_shuffle.cpp's "[.][diag]" CSV case writes
out/787b_shuffle_liftoff_<mass>[_rigid].csv). Python standard library only
(csv, wave, math, struct, argparse, os) - nothing to install.

Usage:
    python tools/plots/rpm_to_wav.py out/787b_shuffle_liftoff_1500.csv
    python tools/plots/rpm_to_wav.py in.csv out.wav --rotors 4 --pre-hold 1.0

Input columns: "t" (s) and "engine_rpm" are required; "engine_load" (0..1,
the effective combustion throttle) is used when present, else 1.

What is physical and what is not:
- Firing frequency = rotors * rpm / 60. A Wankel rotor makes one power pulse
  per eccentric-shaft revolution (Wikipedia, "Wankel engine"), and the R26B's
  four rotors fire 1-3-2-4 at 90 deg shaft spacing (SAE 920309, Table 1), so
  the pulses are evenly spaced at 4 per shaft revolution. The phase is
  integrated from the interpolated rpm, so any rpm oscillation (the shuffle)
  becomes a frequency modulation of the firing tone.
- Everything else is an ARBITRARY listening choice, not data: the pulse shape
  (a fast-rise/exponential-decay blowdown pulse plus one damped pipe
  resonance), the per-rotor amplitude pattern, the load-to-loudness map and
  the overrun floor.

EXAGGERATED, NOT PHYSICAL (off by default): --fm-reference REF.csv
--fm-exaggerate G replaces the rpm by ref + G*(rpm - ref), with ref the
reference trace's engine_rpm interpolated at each sample time (typically the
matching Rigid control CSV). G > 1 magnifies only the compliant-minus-rigid
difference (the shuffle) so it can be heard; it is a listening aid, never a
simulation result. Without --fm-reference the output is unchanged.
"""

import argparse
import csv
import math
import os
import struct
import wave

# Arbitrary sound-design constants (see the module docstring).
PULSE_TAU_S = 0.00035          # blowdown pulse decay time
RESONANCE_HZ = 1700.0          # one exhaust-pipe resonance
RESONANCE_TAU_S = 0.0015       # its decay time
RESONANCE_GAIN = 0.6
ROTOR_GAIN = (1.0, 0.94, 0.97, 0.91)  # small rotor-to-rotor differences give 1st-order texture
OVERRUN_FLOOR = 0.18           # loudness at zero load (pumping only, no combustion)
PULSE_LIFETIME_S = 0.008
FADE_S = 0.05


def read_trace(path):
    t, rpm, load = [], [], []
    with open(path, newline="") as f:
        rows = csv.DictReader(f)
        for row in rows:
            t.append(float(row["t"]))
            rpm.append(float(row["engine_rpm"]))
            load.append(float(row["engine_load"]) if "engine_load" in row and row["engine_load"] != "" else 1.0)
    if len(t) < 2:
        raise SystemExit("need at least two samples in " + path)
    return t, rpm, load


def interp(ts, ys, x, hint):
    """Linear interpolation with a moving index hint (x increases monotonically)."""
    i = hint
    while i + 1 < len(ts) - 1 and ts[i + 1] <= x:
        i += 1
    if x <= ts[0]:
        return ys[0], 0
    if x >= ts[-1]:
        return ys[-1], len(ts) - 2
    a = (x - ts[i]) / (ts[i + 1] - ts[i])
    return ys[i] + a * (ys[i + 1] - ys[i]), i


def pulse(age_s):
    """Pressure pulse of one firing event, age_s seconds after it started."""
    blow = (age_s / PULSE_TAU_S) * math.exp(1.0 - age_s / PULSE_TAU_S)
    ring = RESONANCE_GAIN * math.exp(-age_s / RESONANCE_TAU_S) * math.sin(2.0 * math.pi * RESONANCE_HZ * age_s)
    return blow + ring


def render(t, rpm, load, rotors, pre_hold_s, rate):
    t0 = t[0] - pre_hold_s
    duration = t[-1] - t0
    n = int(duration * rate)
    out = [0.0] * n
    phase = 0.0            # in firing periods
    event = 0
    active = []            # (start sample, amplitude)
    hint = 0
    lifetime = int(PULSE_LIFETIME_S * rate)
    for k in range(n):
        tk = t0 + k / rate
        r, hint = interp(t, rpm, tk, hint)
        ld, _ = interp(t, load, tk, hint)
        f_fire = rotors * max(r, 0.0) / 60.0
        phase += f_fire / rate
        while phase >= 1.0:
            phase -= 1.0
            loud = OVERRUN_FLOOR + (1.0 - OVERRUN_FLOOR) * min(max(ld, 0.0), 1.0)
            active.append((k, loud * ROTOR_GAIN[event % len(ROTOR_GAIN)]))
            event += 1
        active = [p for p in active if k - p[0] < lifetime]
        s = 0.0
        for start, amp in active:
            s += amp * pulse((k - start) / rate)
        out[k] = s
    # Remove DC, fade the ends, normalise to 0.9 full scale.
    mean = sum(out) / n
    out = [v - mean for v in out]
    fade = int(FADE_S * rate)
    for k in range(min(fade, n)):
        g = k / fade
        out[k] *= g
        out[n - 1 - k] *= g
    peak = max(abs(v) for v in out) or 1.0
    return [0.9 * v / peak for v in out]


def write_wav(path, samples, rate):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(b"".join(struct.pack("<h", int(round(v * 32767))) for v in samples))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv")
    ap.add_argument("wav", nargs="?", help="default: <repo>/out/<csv name>.wav")
    ap.add_argument("--rotors", type=int, default=4)
    ap.add_argument("--pre-hold", type=float, default=1.0, help="seconds of the first sample held before the trace starts")
    ap.add_argument("--rate", type=int, default=44100)
    ap.add_argument("--fm-reference", help="EXAGGERATION ONLY: reference CSV (e.g. the Rigid control) for --fm-exaggerate")
    ap.add_argument("--fm-exaggerate", type=float, default=1.0,
                    help="EXAGGERATION ONLY: rpm := ref + G*(rpm - ref); needs --fm-reference; not physical")
    args = ap.parse_args()
    t, rpm, load = read_trace(args.csv)
    if args.fm_reference is not None:
        rt, rrpm, _ = read_trace(args.fm_reference)
        hint = 0
        exaggerated = []
        for tk, r in zip(t, rpm):
            ref, hint = interp(rt, rrpm, tk, hint)
            exaggerated.append(ref + args.fm_exaggerate * (r - ref))
        rpm = exaggerated
        print("EXAGGERATED (not physical): rpm = ref + %.2f*(rpm - ref), ref = %s" % (args.fm_exaggerate, args.fm_reference))
    elif args.fm_exaggerate != 1.0:
        raise SystemExit("--fm-exaggerate needs --fm-reference")
    wav = args.wav
    if wav is None:
        repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        wav = os.path.join(repo, "out", os.path.splitext(os.path.basename(args.csv))[0] + ".wav")
    os.makedirs(os.path.dirname(os.path.abspath(wav)), exist_ok=True)
    samples = render(t, rpm, load, args.rotors, args.pre_hold, args.rate)
    write_wav(wav, samples, args.rate)
    print("wrote %s (%.2f s, firing %.0f-%.0f Hz)" % (os.path.abspath(wav), len(samples) / args.rate,
                                                      args.rotors * min(rpm) / 60.0, args.rotors * max(rpm) / 60.0))


if __name__ == "__main__":
    main()
