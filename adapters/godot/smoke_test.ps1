#Requires -Version 5.1
<#
.SYNOPSIS
    PLAN.md P2/M1 headless smoke test: builds the ps_godot_adapter
    GDExtension and its plain-C++ tests (input mapping, origin rebasing,
    haptics rumble shaping), runs them, runs the demo Godot
    project headless (fails if Godot prints any error), then runs it again
    with --m1-autopilot (main.gd's scripted throttle/brake/steer sequence)
    and asserts the car moved sensibly (distance, top speed, upright, no
    NaN).

.DESCRIPTION
    Deliberately NOT wired into tools/ci.ps1 (that file is another agent's
    lane right now - see this adapter's final report for the exact lines to
    add there later). Uses its own CMake binary directory
    (out/build/godot-adapter), separate from the debug/release/asan
    directories tools/ci.ps1 uses, so this script never races a concurrent
    build in those directories.

.PARAMETER QuitAfterFrames
    How many process frames the demo runs headless before quitting
    (Godot's own --quit-after flag). Default 300 (~5 s at 60 fps) - enough
    for PsSimulation to load the scenario, run several hundred sim ticks on
    its own thread, and for the HUD to print at least one measured-tick-rate
    line.

.PARAMETER SkipBuild
    Skip the CMake configure/build step (use an already-built
    out/build/godot-adapter + demo/bin/libps_godot.dll - faster iteration).

.PARAMETER Vehicle
    Which vehicle main.gd's --m1-autopilot drive spawns and checks against
    (main.gd's own --m1-vehicle= cmdline arg, forwarded verbatim when this
    is not "car_sedan"). Default "car_sedan" (unchanged behaviour). The M1
    driving-check thresholds below are per-vehicle (tractor demo, 2026-09-28)
    - see $script:VehicleThresholds - because the tractor's 12F/4R gearbox,
    diesel engine + governor and much lower top speed do not reach the
    sedan-tuned bars (max_speed_kmh>15, max_gear>=2) within the same ~10s
    scripted autopilot sequence (main.gd's _run_autopilot is not itself
    vehicle-aware - it drives generic throttle/brake/shift_up_count control
    channels the same way regardless of which vehicle is spawned).
#>
[CmdletBinding()]
param(
    [int]$QuitAfterFrames = 300,
    [switch]$SkipBuild,
    [string]$Vehicle = 'car_sedan'
)

# Tractor demo (2026-09-28), PROVISIONAL - see this script's own
# .PARAMETER Vehicle comment and out/tractor_demo_progress.md for the
# alternatives considered (a per-vehicle table here, matching main.gd's own
# VEHICLE_TABLE pattern, vs. a wholly separate looser tractor-only check;
# chose the table for symmetry and because it keeps one shared pass/fail
# code path). MinGear is deliberately 1 (not 2) for the tractor: its 12F/4R
# gearbox's ratio spacing/synchro timing is untested against this exact
# scripted 10s sequence, so requiring only "the drivetrain engaged and moved
# under power" rather than "shifted at least once" avoids a flaky gate until
# an owner test drive can tune this properly.
$script:VehicleThresholds = @{
    'car_sedan' = @{ MinDistanceM = 4.0; MinSpeedKmhFloor = 3.0; MinSpeedKmhGate = 15.0; MinGear = 2 }
    'tractor'   = @{ MinDistanceM = 1.0; MinSpeedKmhFloor = 0.5; MinSpeedKmhGate = 2.0;  MinGear = 1 }
}
$script:VehicleGate = $script:VehicleThresholds[$Vehicle]
if (-not $script:VehicleGate) {
    Write-Host "smoke_test.ps1: no threshold entry for vehicle '$Vehicle' - falling back to car_sedan's" -ForegroundColor Yellow
    $script:VehicleGate = $script:VehicleThresholds['car_sedan']
}

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot | Split-Path -Parent
$adapterDir = Join-Path $repoRoot 'adapters\godot'
$demoDir = Join-Path $adapterDir 'demo'
$buildDir = Join-Path $repoRoot 'out\build\godot-adapter'

$script:Failures = New-Object System.Collections.Generic.List[string]

function Report-Fail {
    param([string]$Message)
    Write-Host "FAIL: $Message" -ForegroundColor Red
    $script:Failures.Add($Message)
}

function Report-Ok {
    param([string]$Message)
    Write-Host "ok: $Message" -ForegroundColor Green
}

# Enter-VsDevShell / Find-GodotConsoleExe / cmake configure+build wrappers
# now live in godot_common.ps1, shared with the repo-root drive.ps1 -
# behaviour/output below is unchanged, only the duplicated function bodies
# moved.
. (Join-Path $PSScriptRoot 'godot_common.ps1')

Write-Host "=== PLAN.md P2 headless smoke test ===" -ForegroundColor Cyan
Write-Host "repo root: $repoRoot"
Write-Host "build dir: $buildDir"

if (-not $SkipBuild) {
    Enter-VsDevShell

    Write-Host "`n-- configure --" -ForegroundColor Cyan
    $configureExitCode = Invoke-AdapterCMakeConfigure -RepoRoot $repoRoot -BuildDir $buildDir
    if ($configureExitCode -ne 0) { Report-Fail "cmake configure failed (exit $configureExitCode)"; }

    if ($script:Failures.Count -eq 0) {
        Write-Host "`n-- build --" -ForegroundColor Cyan
        $buildExitCode = Invoke-AdapterCMakeBuild -BuildDir $buildDir -Targets @('ps_godot_adapter', 'ps_godot_test_input_map', 'ps_godot_test_origin_rebase', 'ps_godot_test_haptics', 'ps_godot_test_render_interp')
        if ($buildExitCode -ne 0) { Report-Fail "cmake build failed (exit $buildExitCode)" }
    }
}

if ($script:Failures.Count -eq 0) {
    Report-Ok "build"
}

# ---------------------------------------------------------------------------
# Plain-C++ tests (PLAN.md P2: input mapping + origin rebasing, tested with
# synthetic input - see adapters/godot/tests/).
# ---------------------------------------------------------------------------
if ($script:Failures.Count -eq 0) {
    foreach ($testName in @('ps_godot_test_input_map', 'ps_godot_test_origin_rebase', 'ps_godot_test_haptics', 'ps_godot_test_render_interp')) {
        $exe = Get-ChildItem -Path $buildDir -Recurse -Filter "$testName.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $exe) {
            Report-Fail "$testName.exe not found under $buildDir"
            continue
        }
        Write-Host "`n-- running $testName --" -ForegroundColor Cyan
        $output = & $exe.FullName 2>&1
        $output | ForEach-Object { Write-Host $_ }
        if ($LASTEXITCODE -eq 0) {
            Report-Ok "$testName (exit 0)"
        } else {
            Report-Fail "$testName exited $LASTEXITCODE"
        }
    }
}

# ---------------------------------------------------------------------------
# Headless Godot run.
# ---------------------------------------------------------------------------
if ($script:Failures.Count -eq 0) {
    $dll = Join-Path $demoDir 'bin\libps_godot.dll'
    if (-not (Test-Path $dll)) {
        Report-Fail "libps_godot.dll not found at $dll after build"
    } else {
        Report-Ok "libps_godot.dll present at $dll"
    }
}

# A fresh checkout/worktree has no adapters/godot/demo/.godot cache (gitignored,
# generated by Godot's own project import) - without it, GDExtension classes
# (PsSimulation) are not registered yet and a plain `--headless` game run fails
# with "Cannot get class 'PsSimulation'" even though the DLL built and loaded
# fine (tractor demo, 2026-09-28: hit exactly this in a brand-new worktree that
# had never run Godot before). One editor-mode headless pass imports the
# project and writes the cache; safe/idempotent to run every time.
if ($script:Failures.Count -eq 0 -and -not (Test-Path (Join-Path $demoDir '.godot'))) {
    $godotExe = Find-GodotConsoleExe
    Write-Host "`n-- first run in this checkout: importing Godot project (writes .godot cache) --" -ForegroundColor Cyan
    & $godotExe --headless --editor --path $demoDir --quit-after 60 2>&1 | ForEach-Object { Write-Host $_ }
    if (-not (Test-Path (Join-Path $demoDir '.godot\extension_list.cfg'))) {
        Report-Fail "project import did not produce .godot/extension_list.cfg - GDExtension classes will not be registered"
    } else {
        Report-Ok "project import produced .godot cache"
    }
}

if ($script:Failures.Count -eq 0) {
    $godotExe = Find-GodotConsoleExe
    Write-Host "`n-- headless run: $godotExe --quit-after $QuitAfterFrames --" -ForegroundColor Cyan

    # stdout/stderr captured to SEPARATE files, not merged with '*>': found
    # by actually running this (see this adapter's report) that '*>' against
    # this native console executable intermittently drops the stdout half
    # entirely (a PowerShell native-command stream-merging quirk, not a
    # Godot or extension bug - reproduced with and without extension errors
    # present). print()'d evidence (the HUD's tick-rate line below) lives on
    # stdout, so losing it silently would make this smoke test unreliable.
    $stdoutFile = Join-Path $buildDir 'smoke_test_godot_stdout.log'
    $stderrFile = Join-Path $buildDir 'smoke_test_godot_stderr.log'
    # Godot routinely prints non-fatal WARNINGs to stderr (e.g. a
    # colinear look_at() hint during a hard manoeuvre) - with this script's
    # $ErrorActionPreference = 'Stop', PowerShell would otherwise promote
    # any such stderr line into a terminating NativeCommandError even
    # though it is redirected straight to a file, not a PowerShell stream.
    # Below distinguishes real errors from warnings by TEXT PATTERN
    # (ERROR/SCRIPT ERROR/...), same as this script already did before -
    # this guard only stops PowerShell itself from aborting early.
    $previousEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $godotExe --headless --path $demoDir --quit-after $QuitAfterFrames 1>$stdoutFile 2>$stderrFile
    $godotExit = $LASTEXITCODE
    $ErrorActionPreference = $previousEap
    $logContent = @(Get-Content $stdoutFile) + @(Get-Content $stderrFile)
    $logContent | ForEach-Object { Write-Host $_ }

    $errorLines = $logContent | Select-String -Pattern 'ERROR|SCRIPT ERROR|Unhandled exception|Segmentation fault'
    if ($errorLines) {
        Report-Fail "Godot printed error line(s):`n$($errorLines -join "`n")"
    } else {
        Report-Ok "no ERROR/SCRIPT ERROR lines in Godot output"
    }
    if ($godotExit -ne 0) {
        Report-Fail "Godot exited with code $godotExit"
    } else {
        Report-Ok "Godot exited 0"
    }

    $tickLine = $logContent | Select-String -Pattern 'measured sim tick rate|sim thread running' | Select-Object -Last 1
    if ($tickLine) {
        Report-Ok "HUD reported the sim thread ticking: $tickLine"
    } else {
        Report-Fail "no tick-rate line found in Godot output - PsSimulation/HUD may not have started"
    }
}

# ---------------------------------------------------------------------------
# PLAN.md M1 driving check: loads the test ground, spawns the car, applies
# a scripted throttle/brake/steer sequence (main.gd's --m1-autopilot path)
# and asserts the car moved sensibly. main.gd self-quits about 9.2 real
# seconds after start (its own scripted sequence's length) - Start-Process
# with a generous hard timeout is a safety net in case that self-quit ever
# fails to fire (e.g. a crash before reaching it), so this script can never
# hang indefinitely.
# ---------------------------------------------------------------------------
if ($script:Failures.Count -eq 0) {
    $godotExe = Find-GodotConsoleExe
    Write-Host "`n-- M1 driving check: $godotExe -- --m1-autopilot --" -ForegroundColor Cyan

    $apStdout = Join-Path $buildDir 'smoke_test_m1_autopilot_stdout.log'
    $apStderr = Join-Path $buildDir 'smoke_test_m1_autopilot_stderr.log'
    # --m1-force-rebase-test: forces one origin rebase partway through the
    # run and checks the chase camera never JUMPS toward/away from the
    # chassis in a single frame afterward - regression coverage for the
    # chase-cam origin-rebase bug (chase_cam.gd's header comment; the
    # placeholder drive alone never covers the 500 m a real rebase needs,
    # see main.gd's own comment on this flag).
    $apArgs = @('--headless', '--path', $demoDir, '--quit-after', '100000', '--', '--m1-autopilot', '--m1-force-rebase-test')
    if ($Vehicle -ne 'car_sedan') {
        $apArgs += "--m1-vehicle=$Vehicle"
    }

    $proc = Start-Process -FilePath $godotExe -ArgumentList $apArgs `
        -RedirectStandardOutput $apStdout -RedirectStandardError $apStderr `
        -NoNewWindow -PassThru
    $finished = $proc.WaitForExit(40000)
    if (-not $finished) {
        Report-Fail "M1 autopilot run did not quit within 40s - killing it (self-quit never fired?)"
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }

    $apLogContent = @(Get-Content $apStdout -ErrorAction SilentlyContinue) + @(Get-Content $apStderr -ErrorAction SilentlyContinue)
    $apLogContent | ForEach-Object { Write-Host $_ }

    $apErrorLines = $apLogContent | Select-String -Pattern 'ERROR|SCRIPT ERROR|Unhandled exception|Segmentation fault'
    if ($apErrorLines) {
        Report-Fail "M1 autopilot run printed error line(s):`n$($apErrorLines -join "`n")"
    } else {
        Report-Ok "M1 autopilot run: no ERROR/SCRIPT ERROR lines"
    }
    # Fall diagnostic (src/fall_detector.h) must stay silent on a normal drive.
    if ($apLogContent | Select-String -SimpleMatch 'ps_godot FALL DETECTED') {
        Report-Fail "M1 autopilot run: fall detector fired on a normal drive"
    } else {
        Report-Ok "M1 autopilot run: fall detector silent"
    }

    $summaryLine = $apLogContent | Select-String -Pattern 'ps_godot M1 smoke:' | Select-Object -Last 1
    if (-not $summaryLine) {
        Report-Fail "M1 autopilot run: no 'ps_godot M1 smoke:' summary line found - main.gd's autopilot path may not have run"
    } else {
        Report-Ok "M1 autopilot summary: $summaryLine"
        # PHYS-033 P4 I3 (C8) section 12.4: new fields appended after nan=
        # (max_gear/stalled/engine_state) - the old prefix (distance_m/
        # max_speed_kmh/upright/nan) keeps its meaning, so this regex still
        # matches a summary line from before this change if one ever shows
        # up (e.g. a bisect), just without the new groups.
        $m = [regex]::Match($summaryLine.ToString(), 'distance_m=([\-0-9.]+)\s+max_speed_kmh=([\-0-9.]+)\s+upright=(true|false)\s+nan=(true|false)\s+max_gear=([\-0-9]+)\s+stalled=(true|false)\s+engine_state=(\w+)', [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
        if (-not $m.Success) {
            Report-Fail "M1 autopilot run: could not parse the summary line: $summaryLine"
        } else {
            $distanceM = [double]$m.Groups[1].Value
            $maxSpeedKmh = [double]$m.Groups[2].Value
            $upright = $m.Groups[3].Value -eq 'True'
            $hasNan = $m.Groups[4].Value -eq 'True'
            $maxGear = [int]$m.Groups[5].Value
            $stalled = $m.Groups[6].Value -eq 'True'
            $engineState = $m.Groups[7].Value

            # Thresholds are deliberately modest, not "a car should floor
            # it" numbers: main.gd's default spawn point (test_ground.json
            # "spawn_main") sits on the open procedural terrain's default
            # ("dirt") surface, not a paved test feature (see main.gd's own
            # comment on spawn_id for a found-but-not-investigated anomaly
            # at the paved spawn points). distance/speed>3 predate the I3
            # drivetrain (PLAN.md P3's torque-curve placeholder) and are
            # kept as a loose "the car moved at all" floor; max_speed_kmh>15
            # and the max_gear/stalled/engine_state checks below are I3's
            # own drivetrain gates (PLAN.md 12.4: "1st at 0.7 throttle on
            # asphalt reaches ~40 km/h by 2.5 s").
            $gate = $script:VehicleGate
            if ($hasNan) { Report-Fail "M1 autopilot: NaN observed in chassis state" } else { Report-Ok "M1 autopilot: no NaN observed" }
            if ($upright) { Report-Ok "M1 autopilot: chassis stayed upright" } else { Report-Fail "M1 autopilot: chassis did not stay upright" }
            if ($distanceM -gt $gate.MinDistanceM) { Report-Ok "M1 autopilot [$Vehicle]: distance travelled $distanceM m (> $($gate.MinDistanceM) m)" } else { Report-Fail "M1 autopilot [$Vehicle]: distance travelled only $distanceM m (expected > $($gate.MinDistanceM) m)" }
            if ($maxSpeedKmh -gt $gate.MinSpeedKmhFloor) { Report-Ok "M1 autopilot [$Vehicle]: reached $maxSpeedKmh km/h (> $($gate.MinSpeedKmhFloor) km/h)" } else { Report-Fail "M1 autopilot [$Vehicle]: only reached $maxSpeedKmh km/h (expected > $($gate.MinSpeedKmhFloor) km/h)" }
            if ($maxSpeedKmh -gt $gate.MinSpeedKmhGate) { Report-Ok "M1 autopilot [$Vehicle]: reached $maxSpeedKmh km/h (> $($gate.MinSpeedKmhGate) km/h drivetrain gate)" } else { Report-Fail "M1 autopilot [$Vehicle]: only reached $maxSpeedKmh km/h (expected > $($gate.MinSpeedKmhGate) km/h with the real drivetrain)" }
            if ($maxGear -ge $gate.MinGear) { Report-Ok "M1 autopilot [$Vehicle]: reached gear $maxGear (>= $($gate.MinGear))" } else { Report-Fail "M1 autopilot [$Vehicle]: never got past gear $maxGear (expected >= $($gate.MinGear))" }
            if (-not $stalled) { Report-Ok "M1 autopilot: engine never stalled" } else { Report-Fail "M1 autopilot: engine entered Stalled during the run" }
            if ($engineState -eq 'running') { Report-Ok "M1 autopilot: engine state at end = running" } else { Report-Fail "M1 autopilot: engine state at end was '$engineState', expected 'running'" }
        }
    }

    # Chase-cam origin-rebase regression check (main.gd's
    # --m1-force-rebase-test, chase_cam.gd's header comment).
    $cameraLine = $apLogContent | Select-String -Pattern 'ps_godot M1 camera check: rebase_forced=' | Select-Object -Last 1
    if (-not $cameraLine) {
        Report-Fail "M1 autopilot run: no 'ps_godot M1 camera check: rebase_forced=' summary line found - --m1-force-rebase-test may not have run"
    } else {
        Report-Ok "M1 camera check summary: $cameraLine"
        $cm = [regex]::Match($cameraLine.ToString(), 'rebase_forced=(true|false)\s+max_frame_jump_m=([\-0-9.]+)\s+jump_bound_m=([\-0-9.]+)\s+max_cam_chassis_dist_m=([\-0-9.]+)\s+max_cam_chassis_dist_after_settle_m=([\-0-9.]+)\s+pass=(true|false)', [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
        if (-not $cm.Success) {
            Report-Fail "M1 autopilot run: could not parse the camera check summary line: $cameraLine"
        } else {
            $rebaseForced = $cm.Groups[1].Value -eq 'True'
            $maxFrameJumpM = [double]$cm.Groups[2].Value
            $jumpBoundM = [double]$cm.Groups[3].Value
            $maxCamDistM = [double]$cm.Groups[4].Value
            $maxCamDistAfterSettleM = [double]$cm.Groups[5].Value
            $cameraPass = $cm.Groups[6].Value -eq 'True'

            if ($rebaseForced) { Report-Ok "M1 camera check: forced rebase fired (via a teleport far enough to make the chase cam's own rebase_focus call cross threshold)" } else { Report-Fail "M1 camera check: forced rebase never fired - test did not exercise the rebase path" }
            # Primary signal: a single-target exponential lag can only ever
            # shrink camera-to-chassis distance frame over frame (up to
            # sub-metre drift) - a multi-metre one-frame INCREASE is
            # specifically the origin-rebase discontinuity (main.gd's own
            # comment on max_frame_jump_m), not the teleport's own
            # (legitimate, expected) catch-up lag.
            $dm = [regex]::Match($cameraLine.ToString(), 'max_render_desync_m=([\-0-9.]+)\s+desync_bound_m=([\-0-9.]+)')
            if ($dm.Success) {
                # Drawn chassis vs. its current pose in the same frame - a
                # ~500 m value means a node placed itself before the frame's
                # origin rebase (cameras must process first, chase_cam.gd).
                Report-Ok "M1 camera check: max rendered-vs-current chassis desync $($dm.Groups[1].Value) m (bound $($dm.Groups[2].Value) m; folded into pass=)"
            } else {
                Report-Fail "M1 camera check: no max_render_desync_m field in: $cameraLine"
            }
            if ($cameraPass) {
                Report-Ok "M1 camera check: no frame-to-frame camera jump beyond $maxFrameJumpM m (bound $jumpBoundM m) - raw peak distance during the teleport's own catch-up lag was $maxCamDistM m, settled to $maxCamDistAfterSettleM m"
            } else {
                Report-Fail "M1 camera check: chase camera jumped $maxFrameJumpM m in a single frame (bound $jumpBoundM m) - origin-rebase desync (chase_cam.gd's `_smoothed_pos` not shifted with the origin?); raw peak distance $maxCamDistM m, $maxCamDistAfterSettleM m after settling"
            }
        }
    }
}

Write-Host "`n=== summary ===" -ForegroundColor Cyan
if ($script:Failures.Count -eq 0) {
    Write-Host "PASS: headless smoke test" -ForegroundColor Green
    exit 0
} else {
    Write-Host "FAIL: $($script:Failures.Count) failure(s):" -ForegroundColor Red
    $script:Failures | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 1
}
