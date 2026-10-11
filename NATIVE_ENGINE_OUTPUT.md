# Native live engine output

The Windows spatial endpoint can consume VehicleVoice directly through an exclusive native pump. This removes render-frame scheduling from Runner-to-device delivery; it does not remove synthesis CPU contention or stalled physics input publication.

## Integration

Open PsSpatialAudio at the voice sample rate and channel count, then attach once:

```cpp
bool attached = spatial->attach_native_source(
    voice->native_output()->acquire(), 12, 1.0f / 15.0f);
```

15 Pa is the default voice full-scale pressure; use the configured full-scale value if different. The lease is passed directly so a failed attachment releases it. Attachment rejects inactive endpoints, mismatched format, duplicate attachments and diagnostic PCM mode.

PsSimulation also exposes `connect_vehicle_voice_spatial(vehicle_name, spatial, latency_ms=12)` to GDScript; it applies the configured voice normalization.

After attachment, stop the old render-frame `read_audio`/`push_audio` path. `get_frames_needed()` returns zero while attached, manual pushes are rejected, and Runner rejects any competing consumer. Continue calling `set_positions`, `set_reference_distance_m`, and `set_reflections_gain` with current camera-relative data. Set normalized composite volume through `set_native_gain`; gain changes are smoothed over 10 ms.

Input publication remains separate: call VehicleVoice::push_tick from the physics update, with a single producer. Do not also publish from `_process`. Stop/join that publisher before replacing or destroying the voice. PsSimulation already publishes from its physics post-step callback. No Godot object API belongs on the pump thread.

`detach_native_source()` stops and joins the producer and releases the exclusive lease before legacy fallback reads resume. `close()` joins the endpoint before releasing its pump. Voice stop/destruction invalidates its bridge before stopping/destroying Runner, so an attached source cannot access freed memory. Restarting a stopped voice creates a new bridge; reconnect it. Endpoint failure releases the pump too.

The target latency accepts 5–50 ms (default 12). The queue grows to accommodate the observed device quantum plus 2 ms, with a hard 50 ms frame bound. This is a native producer fix, rather than relying on a large render-fed queue. Runner's existing four 256-frame blocks remain separate buffering.

## Diagnostics

`get_output_diagnostics()` returns cumulative counters; sample deltas during the reported traffic hitch:

- `native_pump_active`, `source_connected`, `queued_frames`: routing and queue health.
- `source_short_reads`, `source_missing_frames`: Runner supplied fewer frames than requested. These include routine producer polling and do not alone prove an audible dropout.
- `output_short_reads`, `output_missing_frames`: device requested frames absent from the pump queue. Rising missing frames indicate output starvation.
- `endpoint_short_reads`, `endpoint_missing_frames`: endpoint shortages, including legacy delivery and startup.
- `runner_mean_us_per_frame`, `runner_max_block_ms`, `runner_block_budget_ms`, `runner_blocks_rendered`: synthesis throughput and stalls. At 48 kHz the sample budget is 20.833 us and a 256-frame block has 5.333 ms.
- `runner_rejected_reads`, `rejected_manual_pushes`: competing legacy consumers or double feeding.

Compare output shortages against synthesis timing and physics publication: a render hitch should no longer interrupt delivery; CPU starvation can still interrupt Runner. Positions and gain can lag a stalled render update, but audio delivery continues.

The existing Godot AudioStreamGenerator fallback is still render-fed. This commit does not supply a Godot audio-mixer callback stream. On fallback, detach first, then use the existing reader; it retains the original hitch limitation. The reusable ps_audio::OutputPump and OutputSource interfaces provide a seam for a future native mixer callback, whose callback must be its only consumer and must stop before pump destruction.

## Verification

Focused native output, threaded Runner, and zero-allocation checks: 32 assertions in six cases passed. Coverage includes exclusive ownership, independent delivery across a 126 ms main-thread pause, starvation counters, bounded queue, and voice-owner teardown with a live pump. Changed Godot adapter translation units compile independently; no game source or running game DLL was replaced.

Actual hypercar synthesis benchmark at 48 kHz measured 15.682, 11.183 and 13.703 us/frame at 950, 3000 and 7500 RPM; longest measured block 4.293 ms. This quiet-machine benchmark does not establish margin under 1,300+ NPC CPU contention. Windows hardware endpoint behavior still needs an integrated road retest. Long sanitizer checks remain queued for the overnight owner.

## Baking the audio bank (`ps_engine_tools`, engine garage P6)

The crank-synchronous PSAB0002 bank (`ps_audio/audio_bank.h`) is baked by a library, not a tool source file:
`engine_tools/` builds the STATIC target `ps_engine_tools` (links `ps_core` and `ps_audio`; neither links it, so
`ps_audio_standalone` stays free of `ps_core`). Header `ps_engine_tools/audio_bank_prep.h`:

```cpp
ps_engine_tools::AudioBankPrepOptions o;
o.cache_dir = "out/engine_cache";            // bank lives in <cache_dir>/audio/<key>.psab; empty = no disk access
o.workers = 0;                               // 0 = half the hardware threads; 1 = the calling thread renders; bytes do not depend on it
o.cancel = &flag;                            // std::atomic<bool>*  (and/or o.cancel_poll, a thread-safe std::function<bool()>)
o.on_progress = [](size_t done, size_t total) { /* ... */ };
auto r = ps_engine_tools::prepare_engine_audio_bank(sim_desc, /*forced_induction*/ nullptr, o);
// r.bank (AudioBankPtr), r.cache_hit, r.key, r.path, r.seconds, r.bytes, r.removed_energy_fraction
```

- Input is the RESOLVED `SimulatedEngineDesc` (complete maps, `cycle` block, as any loader returns it, a garage edit
  included); `forced_induction` non-null replaces `desc.turbo_pair` for the bake, null uses the desc as it is.
  `prepare_engine_audio_bank_from_files(engine_path, turbo_path, o)` loads with `ps::io::load_engine_json` first
  (tools and the Godot adapter hold paths). Bake from Full maps; a Sketch/Preview engine gets its own key and a
  worse bank.
- Cost: 10 planes (5 rpm rows x 2 pedals), 5256 render ticks in total [M]; demo_i4_pipes 2.8 s on 10 workers /
  20 s on one, hyper V8 4.5 s warm [M]. A cache hit costs the operating-point settles (0.1 s [M]) and no progress call.
- Cancel (mirrors the engine-map generator, `MapGenerationControl`): the flag is read before every operating point,
  before each 200-frame tick of every plane and once more right before the file is published; once set, the call
  throws `ps::drivetrain::GenerationCancelled` after the running ticks finished (latency one tick) and writes no
  cache file and no temp file. A flag never observed means a normal return (a request, not a guarantee).
- Progress: `on_progress(done, total)` once per finished tick, `done` = 1, 2, ..., `total` (strictly rising by 1;
  `total` = 5256 for the shipped grid), always on the calling thread - workers only bump an atomic and the calling
  thread polls it (a burst arrives as consecutive calls). Not called for a cache hit. The callback may allocate and
  be slow, must not throw.
- Cache key = FNV-1a over the generator identity (`kEngineAudioBankSettings`), `engine_map_cache_key` of the cycle
  block with its first fuel (free text blanked), and the 10 operating points (manifold/charge pressure, charge
  temperature, lambda, throttle position, spark advance) the recording takes from the MVEM engine. It therefore
  moves with an ECU, cam, map, throttle, turbo or fuel change exactly when that reaches the sound, whichever part
  file or user edit it came from, and not with free text, positions or the worker count. `engine_audio_bank_key`
  computes it without recording (about 0.1-0.3 s [M]) - "does this edit need a rebake?".
- Tests: `tests/unit/test_audio_bank_prep.cpp` (`[audio][engine_tools]`); the bank bytes are pinned against the
  pre-move generator (demo_i4_pipes and the turbo hyper V8).
