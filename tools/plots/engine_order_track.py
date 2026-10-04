"""Engine-order tracking of a rendered engine WAV against its EngineInput CSV.

Dev tool (numpy; not part of any build). Used for the crank-synchronous audio
bank measurement (out/audio_bank_crank_sync_progress.md):

  python engine_order_track.py stats  <wav> <input.csv> <t0> <t1> [--nfft 8192] [--orders 1,4]
  python engine_order_track.py compare <bank.wav> <nobank.wav> <input.csv> <t0> <t1> [--nfft 8192]
  python engine_order_track.py sweep  <template.csv> <out.csv> [--rpm0 3000] [--rpm1 8000] [--seconds 25] [--hold 2]
  python engine_order_track.py cut    <wav> <out.wav> <t0> <t1>

stats [--detrend S] [--bins N]: Hann STFT (nfft points, hop 480 = 10 ms), for each order k the band
k*rpm/60 +- 0.25*rpm/60 (rpm interpolated from the CSV at the frame centre);
order amplitude = sqrt(sum |X|^2) over the band [dB]. Reports per order: sd,
min - median, max frame-to-frame jump (all dB), and the pitch-track residual:
the band's peak frequency (parabolic interpolation on log magnitude) minus
k*rpm/60, rms [Hz]. Plus the 25 ms broadband RMS envelope sd [dB].

sweep: a steady-then-ramping rpm input script built from the template row
closest to 7000 rpm with no cut flags (pressures, lambda, spark held), the
crank angle integrated from omega so the voice's phase correction stays locked.
"""
import argparse
import csv
import math
import sys
import wave

import numpy as np


def read_wav(path):
    with wave.open(path) as w:
        n = w.getnframes()
        fs = w.getframerate()
        ch = w.getnchannels()
        width = w.getsampwidth()
        raw = w.readframes(n)
    if width == 2:
        x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    elif width == 4:
        x = np.frombuffer(raw, dtype="<f4").astype(np.float64)
    else:
        raise SystemExit("unsupported sample width %d" % width)
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    return x, fs


def read_csv(path):
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        rows = list(r)
    return rows


def stats(args):
    x, fs = read_wav(args.wav)
    rows = read_csv(args.csv)
    t_in = np.array([float(r["t_sim_s"]) for r in rows])
    rpm_in = np.array([float(r["omega_rad_s"]) for r in rows]) * 60.0 / (2.0 * math.pi)
    nfft = args.nfft
    hop = 480
    win = np.hanning(nfft)
    i0 = int(args.t0 * fs)
    i1 = int(args.t1 * fs)
    starts = np.arange(i0, i1 - nfft, hop)
    centres = (starts + nfft / 2) / fs
    rpm = np.interp(centres, t_in, rpm_in)
    frames = np.stack([x[s:s + nfft] * win for s in starts])
    spec = np.abs(np.fft.rfft(frames, axis=1)) ** 2
    df = fs / nfft
    out = {}
    for k in args.orders:
        amp = np.empty(len(starts))
        resid = np.empty(len(starts))
        for i in range(len(starts)):
            fk = k * rpm[i] / 60.0
            bw = 0.25 * rpm[i] / 60.0
            lo = max(1, int(math.floor((fk - bw) / df)))
            hi = int(math.ceil((fk + bw) / df))
            band = spec[i, lo:hi + 1]
            amp[i] = 10.0 * math.log10(max(band.sum(), 1e-30))
            j = int(np.argmax(band))
            if 0 < j < len(band) - 1:
                a, b, c = (math.log(max(v, 1e-30)) for v in band[j - 1:j + 2])
                den = a - 2 * b + c
                d = 0.5 * (a - c) / den if den != 0 else 0.0
            else:
                d = 0.0
            resid[i] = (lo + j + d) * df - fk
        if args.bins > 0:
            edges = np.linspace(rpm.min(), rpm.max(), args.bins + 1)
            idx = np.clip(np.digitize(rpm, edges) - 1, 0, args.bins - 1)
            print("  order %g level per rpm bin: %s" % (k, " ".join(
                "%.0f:%.1f" % (0.5 * (edges[b] + edges[b + 1]), np.median(amp[idx == b])) for b in range(args.bins))))
        if args.detrend > 0:
            # remove the slow level trend (a centred moving average of
            # `detrend` seconds): what remains is modulation and steps
            n = max(1, int(round(args.detrend * fs / hop)))
            pad = np.pad(amp, (n // 2, n - 1 - n // 2), mode="edge")
            amp = amp - np.convolve(pad, np.ones(n) / n, mode="valid")
        med = float(np.median(amp))
        out[k] = dict(sd=float(amp.std()), min_rel=float(amp.min() - med), max_jump=float(np.abs(np.diff(amp)).max()),
                      pitch_rms=float(np.sqrt(np.mean(resid ** 2))), median=med)
    seg = x[i0:i1]
    w25 = int(0.025 * fs)
    n25 = len(seg) // w25
    env = 10 * np.log10(np.maximum((seg[:n25 * w25].reshape(n25, w25) ** 2).mean(axis=1), 1e-30))
    print("%s [%g, %g] s, nfft %d, rpm %.0f..%.0f" % (args.wav, args.t0, args.t1, nfft, rpm.min(), rpm.max()))
    for k, s in out.items():
        print("  order %g: sd %.2f dB, min %.2f dB below median, max jump %.2f dB, pitch residual %.3f Hz rms (median %.1f dB)"
              % (k, s["sd"], -s["min_rel"], s["max_jump"], s["pitch_rms"], s["median"]))
    print("  25 ms envelope sd %.2f dB" % env.std())
    return out


def compare(args):
    """Bank (b) vs no bank (c) over one window: exits 1 unless, for every
    order, the 0.5 s-detrended level sd of (b) is within 0.1 dB of (c), its
    max frame-to-frame jump is <= max(0.5, (c) + 0.2) dB and its dip below
    median <= max(1, (c) + 0.5) dB ((c) itself dips 2.2 dB on order 1 in the
    slow sweep, where order 1 is weak).
    The old sample-time playback measured 3.1 dB sd / 5.7 dB jumps / 16.8 dB
    dips on order 4 there; (c) itself 0.03 dB sd."""
    import contextlib
    import io
    res = {}
    for tag, path in (("b", args.wav_b), ("c", args.wav_c)):
        ns = argparse.Namespace(wav=path, csv=args.csv, t0=args.t0, t1=args.t1, nfft=args.nfft, orders=args.orders,
                                detrend=0.5, bins=0)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            res[tag] = stats(ns)
        print(buf.getvalue(), end="")
    ok = True
    for k in args.orders:
        b, c = res["b"][k], res["c"][k]
        checks = [("sd", b["sd"] <= c["sd"] + 0.1), ("max jump", b["max_jump"] <= max(0.5, c["max_jump"] + 0.2)),
                  ("dip", -b["min_rel"] <= max(1.0, -c["min_rel"] + 0.5))]
        for name, good in checks:
            ok &= good
            print("  order %g %s: %s" % (k, name, "ok" if good else "FAIL"))
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def sweep(args):
    rows = read_csv(args.template)
    cand = [r for r in rows if int(float(r["flags"])) == 0 and float(r["throttle"]) > 0.9]
    if not cand:
        cand = rows
    best = min(cand, key=lambda r: abs(float(r["omega_rad_s"]) * 60 / (2 * math.pi) - 7000))
    fields = list(rows[0].keys())
    dt = 1.0 / 240.0
    n = int(round((args.hold + args.seconds) / dt))
    crank = 0.0
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for i in range(n):
            t = (i + 1) * dt
            u = min(max((t - args.hold) / args.seconds, 0.0), 1.0)
            rpm = args.rpm0 + (args.rpm1 - args.rpm0) * u
            omega = rpm * 2 * math.pi / 60
            crank = (crank + omega * dt * 180 / math.pi) % 720.0
            r = dict(best)
            r["t_sim_s"] = "%.9f" % t
            r["omega_rad_s"] = "%.9f" % omega
            r["crank_angle_deg"] = "%.9f" % crank
            r["flags"] = "0"
            w.writerow([r[k] for k in fields])
    print("wrote %s: %d rows, template row t=%s rpm %.0f" % (args.out, n, best["t_sim_s"],
                                                           float(best["omega_rad_s"]) * 60 / (2 * math.pi)))


def cut(args):
    with wave.open(args.wav) as w:
        p = w.getparams()
        fs = w.getframerate()
        w.setpos(int(args.t0 * fs))
        data = w.readframes(int((args.t1 - args.t0) * fs))
    with wave.open(args.out, "wb") as o:
        o.setparams(p)
        o.writeframes(data)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("stats")
    s.add_argument("wav")
    s.add_argument("csv")
    s.add_argument("t0", type=float)
    s.add_argument("t1", type=float)
    s.add_argument("--nfft", type=int, default=8192)
    s.add_argument("--orders", type=lambda v: [float(o) for o in v.split(",")], default=[1.0, 4.0])
    s.add_argument("--detrend", type=float, default=0.0, help="subtract a moving average of this many seconds")
    s.add_argument("--bins", type=int, default=0, help="print the median order level in this many rpm bins")
    s.set_defaults(fn=stats)
    m = sub.add_parser("compare")
    m.add_argument("wav_b")
    m.add_argument("wav_c")
    m.add_argument("csv")
    m.add_argument("t0", type=float)
    m.add_argument("t1", type=float)
    m.add_argument("--nfft", type=int, default=8192)
    m.add_argument("--orders", type=lambda v: [float(o) for o in v.split(",")], default=[1.0, 4.0])
    m.set_defaults(fn=compare)
    w = sub.add_parser("sweep")
    w.add_argument("template")
    w.add_argument("out")
    w.add_argument("--rpm0", type=float, default=3000)
    w.add_argument("--rpm1", type=float, default=8000)
    w.add_argument("--seconds", type=float, default=25)
    w.add_argument("--hold", type=float, default=2)
    w.set_defaults(fn=sweep)
    c = sub.add_parser("cut")
    c.add_argument("wav")
    c.add_argument("out")
    c.add_argument("t0", type=float)
    c.add_argument("t1", type=float)
    c.set_defaults(fn=cut)
    a = ap.parse_args()
    r = a.fn(a)
    return r if isinstance(r, int) else 0


if __name__ == "__main__":
    sys.exit(main())
