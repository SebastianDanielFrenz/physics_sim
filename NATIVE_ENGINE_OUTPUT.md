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
