#Requires -Version 5.1
<#
.SYNOPSIS
    Builds the ps_godot_adapter GDExtension (incrementally) and launches
    Godot on the demo project - "build it and go drive" in one command,
    from either cmd.exe or PowerShell (see drive.cmd, the thin wrapper
    next to this file).

.DESCRIPTION
    Shares its build directory (out/build/godot-adapter) and DLL output
    (adapters/godot/demo/bin/libps_godot.dll) with
    adapters/godot/smoke_test.ps1; both dot-source the helpers in
    adapters/godot/godot_common.ps1 (VS dev-env entry, Godot executable
    lookup, cmake configure/build wrappers) instead of duplicating them.

    Unlike smoke_test.ps1 (which always reconfigures), the default build
    here is INCREMENTAL: cmake configure only runs the first time (no
    out/build/godot-adapter/build.ninja yet); afterwards a plain
    `cmake --build` is enough even after a CMakeLists.txt edit, because
    Ninja's own generated build re-runs cmake itself when needed.

    Before building, checks whether libps_godot.dll is currently locked
    (a running Godot instance likely still has the demo open) and stops
    with a clear message instead of letting the build fail deep inside
    link.exe/lld-link with a cryptic "cannot open ... for writing" error.
    If the build itself fails, the script stops before ever launching
    Godot, so you never end up driving against a stale DLL.

    PASS-THROUGH CONVENTION for extra Godot arguments (e.g. main.gd's
    own --m1-autopilot): this script intentionally does NOT use a formal
    PowerShell param() block for them. A literal "--" cannot be bound by
    PowerShell's own parameter binder when named parameters are declared
    (confirmed by testing: it throws "AmbiguousParameter" no matter where
    the "--" appears on the command line) - a real PowerShell limitation,
    not a choice here. So $args is parsed by hand below instead. In
    practice all of the following work identically, from either shell:
        drive.ps1 --m1-autopilot
        drive.ps1 -SkipBuild --m1-autopilot
        drive.ps1 -SkipBuild -- --m1-autopilot
        drive.cmd -- --m1-autopilot
        drive.cmd -DryRun -- --headless --quit-after 300
    A literal "--" is accepted as an optional visual separator (and is
    itself swallowed, never forwarded to Godot) but is never required:
    this script's own switches are always single words
    (-SkipBuild/-NoConsole/-Editor/-DryRun, one or two leading dashes),
    Godot's own flags conventionally start with "--", and anything this
    script does not recognise is forwarded as-is - so the two never
    collide. Forwarded args are placed after Godot's own "--" on the
    launched command line (Godot's own convention for user args).

.PARAMETER -SkipBuild
    Skip the configure/build step entirely and launch whatever is
    already built. Errors if libps_godot.dll does not exist yet.

.PARAMETER -NoConsole
    Launch the non-console Godot executable (no extra terminal window;
    Godot's own stdout/stderr will not print into this window).

.PARAMETER -Editor
    Open the Godot editor on the demo project instead of running it.
    Forwarded Godot args are ignored in this mode (a warning is printed
    if any were given).

.PARAMETER -VR
    SteamVR/OpenXR driver's-eye view: starts Godot with --xr-mode on and
    the Vulkan (Mobile) renderer. Start SteamVR streaming and wear the
    headset first; keyboard input needs the Godot desktop window focused.
    F9 recenters. Without a runtime/headset the demo stays on the flat screen.

.PARAMETER -DryRun
    Print exactly what would happen (whether a configure/build would
    run, and the resolved Godot command line) and do nothing - no
    build, no launch.
#>

$ErrorActionPreference = 'Stop'

# Resolve everything relative to this script's own location, not the
# caller's current directory (works from any cwd, and from a double-click
# where cwd may be unpredictable).
$repoRoot   = $PSScriptRoot
$adapterDir = Join-Path $repoRoot 'adapters\godot'
$demoDir    = Join-Path $adapterDir 'demo'
$buildDir   = Join-Path $repoRoot 'out\build\godot-adapter'
$dllPath    = Join-Path $demoDir 'bin\libps_godot.dll'

. (Join-Path $adapterDir 'godot_common.ps1')

# ---------------------------------------------------------------------------
# Hand-rolled argument parsing - see the "PASS-THROUGH CONVENTION" note
# above for why this isn't a formal param() block.
# ---------------------------------------------------------------------------
$SkipBuild = $false
$NoConsole = $false
$Editor    = $false
$DryRun    = $false
$ShowHelp  = $false
$VR        = $false
$GodotArgs = New-Object System.Collections.Generic.List[string]
$sawSeparator = $false

foreach ($a in $args) {
    if ($sawSeparator) { $GodotArgs.Add($a); continue }
    switch -Regex ($a) {
        '^--?SkipBuild$' { $SkipBuild = $true; continue }
        '^--?NoConsole$' { $NoConsole = $true; continue }
        '^--?Editor$'    { $Editor    = $true; continue }
        '^--?DryRun$'    { $DryRun    = $true; continue }
        '^--?VR$'        { $VR        = $true; continue }
        '^(--?help|-h|/\?)$' { $ShowHelp = $true; continue }
        '^--$'           { $sawSeparator = $true; continue }
        default          { $GodotArgs.Add($a) }
    }
}

# ---------------------------------------------------------------------------
# -help / --help / -h: usage, this script's switches and the demo's own
# --m1-* flags (main.gd's _ready). Vehicles and spawn points are listed
# from the data files so the text cannot go stale.
# ---------------------------------------------------------------------------
if ($ShowHelp) {
    $vehicles = (Get-ChildItem (Join-Path $repoRoot 'data\vehicles'), (Join-Path $repoRoot 'tests\support\fixtures\vehicles') -Filter '*.json' |
        ForEach-Object { $_.BaseName }) -join ', '
    $groundJson = Join-Path $repoRoot 'data\tracks\test_ground\test_ground.json'
    $spawns = if (Test-Path $groundJson) {
        ([regex]::Matches((Get-Content $groundJson -Raw), '"(spawn_[a-z0-9_]+)"') |
            ForEach-Object { $_.Groups[1].Value } | Sort-Object -Unique) -join ', '
    } else { '(test_ground.json not found)' }
    # Wrap a comma list to ~70 columns under the help text's value column.
    $wrap = {
        param([string]$list)
        $lines = @(); $line = ''
        foreach ($item in ($list -split ', ')) {
            if ($line.Length -gt 0 -and ($line.Length + $item.Length + 2) -gt 70) { $lines += $line + ','; $line = $item }
            elseif ($line.Length -gt 0) { $line += ', ' + $item }
            else { $line = $item }
        }
        $lines += $line
        $lines -join ("`n" + (' ' * 29))
    }
    $vehicles = & $wrap $vehicles
    $spawns = & $wrap $spawns
    @"
drive - build the Godot adapter (incrementally) and launch the demo

USAGE
  drive [switches] [--] [demo/Godot args...]      (cmd.exe: drive.cmd, PowerShell: .\drive.ps1)

SWITCHES (one or two leading dashes)
  -help, -h        show this text and exit
  -SkipBuild       launch the existing DLL without building
  -NoConsole       use the non-console Godot executable (no log window)
  -Editor          open the Godot editor on the demo instead of running it
  -DryRun          print what would be built/launched, do nothing
  -VR              SteamVR/OpenXR driver's-eye view (Godot --xr-mode on); start
                   SteamVR with the headset first. Falls back to the flat screen
                   if OpenXR does not start. F9 recenters the seat.

Anything else is forwarded to the demo (placed after Godot's own "--").
A literal "--" is optional; everything after it is forwarded unchanged.

DEMO FLAGS (main.gd)
  --m1-vehicle=NAME          vehicle file data/vehicles/NAME.json, else
                             tests/support/fixtures/vehicles/NAME.json (default car_sedan;
                             a value with a / or ending in .json is a repo-relative path)
                             $vehicles
                             chassis and art come from main.gd's VEHICLE_TABLE: only
                             car_sedan and tractor have entries, every other file uses
                             the sedan's (right for car_sedan_*, not for tractor_fuel)
  --m1-spawn=ID              spawn point (default spawn_straight_car / _tractor)
                             $spawns
  --m1-autopilot             scripted drive, prints a smoke summary and quits
  --m1-force-rebase-test     with --m1-autopilot: force an origin rebase and check the camera
  --m1-haptics-cruise        scripted cruise for the rumble check, then quits
  --m1-screenshot=PATH.png   save one screenshot and quit (needs a window, not --headless)
  --m1-screenshot-view=V     drive (default) | cockpit | side | three_quarter | wheels
  --m1-screenshot-delay=S    simulated seconds before the screenshot (default 3)

IN GAME
  V / Y(joy) cycles chase -> cockpit -> free camera; the HUD lists the other keys.

EXAMPLES
  drive
  drive --m1-vehicle=car_sedan_pipes
  drive -SkipBuild --m1-vehicle=tractor --m1-spawn=spawn_skidpad_tractor
  drive -VR --m1-vehicle=car_sedan_pipes
  drive -DryRun -- --headless --quit-after 300
"@ | Write-Host
    exit 0
}

Write-Host "=== drive: build + launch the ps_godot demo ===" -ForegroundColor Cyan
Write-Host "repo root: $repoRoot"
Write-Host "build dir: $buildDir"
Write-Host "demo dir:  $demoDir"
if ($GodotArgs.Count -gt 0) {
    Write-Host "forwarding to Godot: $($GodotArgs -join ' ')"
}
if ($DryRun) {
    Write-Host "(dry run - nothing will actually be built or launched)" -ForegroundColor Yellow
}

# ---------------------------------------------------------------------------
# Locked-DLL check - before touching the build at all.
# ---------------------------------------------------------------------------
if (-not $SkipBuild) {
    $dllLocked = Test-DllLocked -Path $dllPath
    if ($dllLocked) {
        if ($DryRun) {
            Write-Host "note: $dllPath appears to be locked (likely a running Godot instance has the demo open) - a real run would stop here. Close Godot and re-run." -ForegroundColor Yellow
        } else {
            throw "libps_godot.dll appears to be locked - a Godot process likely already has the demo open:`n  $dllPath`nClose Godot and re-run drive.ps1 / drive.cmd."
        }
    }
}

# ---------------------------------------------------------------------------
# Build (incremental: configure only if out/build/godot-adapter has never
# been configured before).
# ---------------------------------------------------------------------------
$buildElapsed = $null
if (-not $SkipBuild) {
    $needsConfigure = -not (Test-AdapterConfigured -BuildDir $buildDir)
    if ($DryRun) {
        $what = if ($needsConfigure) { 'configure + build' } else { 'build (already configured)' }
        Write-Host "`n-- would $what target ps_godot_adapter in $buildDir --" -ForegroundColor Cyan
    } else {
        Enter-VsDevShell
        $sw = [System.Diagnostics.Stopwatch]::StartNew()

        if ($needsConfigure) {
            Write-Host "`n-- configure --" -ForegroundColor Cyan
            $configureExitCode = Invoke-AdapterCMakeConfigure -RepoRoot $repoRoot -BuildDir $buildDir
            if ($configureExitCode -ne 0) {
                throw "cmake configure failed (exit $configureExitCode) - not launching Godot"
            }
        }

        Write-Host "`n-- build --" -ForegroundColor Cyan
        $buildExitCode = Invoke-AdapterCMakeBuild -BuildDir $buildDir -Targets @('ps_godot_adapter')
        $sw.Stop()
        $buildElapsed = $sw.Elapsed

        if ($buildExitCode -ne 0) {
            throw "cmake build failed (exit $buildExitCode) after $($buildElapsed.ToString('mm\:ss\.ff')) - not launching Godot on a stale DLL"
        }
        if (-not (Test-Path $dllPath)) {
            throw "build reported success but $dllPath is still missing - not launching Godot"
        }
        Write-Host "build ok in $($buildElapsed.ToString('mm\:ss\.ff'))" -ForegroundColor Green
    }
} else {
    Write-Host "`n-- skipping build (-SkipBuild) --" -ForegroundColor Cyan
    if (-not $DryRun -and -not (Test-Path $dllPath)) {
        throw "-SkipBuild given but $dllPath does not exist yet - build at least once first (run drive.ps1 without -SkipBuild)"
    }
}

# ---------------------------------------------------------------------------
# Launch.
# ---------------------------------------------------------------------------
$godotExe = if ($NoConsole) { Find-GodotExe } else { Find-GodotConsoleExe }

$launchArgs = New-Object System.Collections.Generic.List[string]
$launchArgs.Add('--path')
$launchArgs.Add($demoDir)
if ($VR) {
    # The project keeps xr/openxr/enabled=false; this turns OpenXR on for this run only.
    $launchArgs.Add('--xr-mode')
    $launchArgs.Add('on')
    # Vulkan for VR: SteamVR's OpenXR wants OpenGL >= 4.3, Godot's GL
    # Compatibility renderer makes a 3.3 context ("trying anyway") - head
    # tracking worked but no picture reached the headset (AMD, 2026-09-30).
    $launchArgs.Add('--rendering-method')
    $launchArgs.Add('mobile')
}

if ($Editor) {
    $launchArgs.Add('--editor')
    if ($GodotArgs.Count -gt 0) {
        Write-Host "note: -Editor ignores forwarded Godot args: $($GodotArgs -join ' ')" -ForegroundColor Yellow
    }
} elseif ($GodotArgs.Count -gt 0) {
    $launchArgs.Add('--')
    foreach ($a in $GodotArgs) { $launchArgs.Add($a) }
}

$launchArgsArr = $launchArgs.ToArray()

Write-Host "`n-- launch --" -ForegroundColor Cyan
Write-Host "$godotExe $($launchArgsArr -join ' ')"

if ($DryRun) {
    Write-Host "`n(dry run - not launching Godot)" -ForegroundColor Yellow
    exit 0
}

& $godotExe @launchArgsArr
exit $LASTEXITCODE
