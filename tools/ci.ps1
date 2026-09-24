#Requires -Version 5.1
<#
.SYNOPSIS
    physics_sim CI script. Configures, builds and tests every required leg,
    greps for Godot includes leaking into core/, and prints a clear per-leg
    PASS/FAIL summary with per-stage timing. Exits non-zero if anything
    failed.

.DESCRIPTION
    THREE LEVELS - pick the smallest one that fits (CLAUDE.md "CI"):

      tools\ci.ps1 -Affected [-Base <ref>]   PER CHANGE. Maps the changed
          paths (working tree + untracked vs <ref>, default HEAD) to test
          topics via $script:PathRules below, then builds ONE Windows preset
          (release) and runs only those topics' tests, the two greps, the
          topics' walltime benches (alone), and the hash compare for the
          topics' scenarios. Adds the Linux legs only for determinism- or
          platform-sensitive changes (see $script:SensitivePatterns) or with
          -Linux. Foundational paths and ANY path the table does not know
          fall back to the default fast run - never to nothing. A
          tests/{unit,validation,determinism}/CMakeLists.txt edit that only
          adds/removes a bare test_*.cpp/.h line (registering a new test
          file) is the one CMakeLists.txt exception: it contributes no
          topics of its own and does NOT force the default fast run - the
          new test file's own Catch2 tags (matched via the ordinary
          'testfile' rule) decide what runs. Any other change to that file
          (target_link_libraries, add_executable(, compile definitions, ...)
          still falls back like any other CMakeLists.txt edit. Doc-only
          changes run the greps only. Prints every decision and why.
          Target: well under 2 min for a single-topic change.
      tools\ci.ps1 -Topic terrain,vehicle     same as -Affected, topics by hand.
      tools\ci.ps1                            DEFAULT FAST run, every leg. At
          batch boundaries and before reporting work as done. Target < 5 min.
      tools\ci.ps1 -Full                      every test on every leg. Only
          before a phase counts done, or after a foundational change
          (core math/jobs/world/backend, CMake, presets, ci.ps1, test support).

.PARAMETER SkipWsl
    Skip the three WSL/Linux legs (linux-release, linux-asan-ubsan, linux-tsan).

.PARAMETER SkipWindows
    Skip all Windows legs (clang-cl debug/release/asan, cl msvc-release).

.PARAMETER Only
    Run only the named leg(s) (e.g. -Only debug,asan). Names match the
    preset names used below (see the $legs definitions).

.PARAMETER Full
    Run every test in every leg (the pre-P1.5b behaviour). Default is FAST:
    release legs (release, msvc-release, linux-release) skip [long]-tagged
    tests; debug/asan/linux-asan-ubsan/linux-tsan skip the whole validation
    suite except the short [sanitizer]-tagged variants. See CLAUDE.md "CI"
    for the tag/label wiring (tests/*/CMakeLists.txt, ADD_TAGS_AS_LABELS).

.PARAMETER NoOverlap
    Run the WSL legs after the Windows legs in this process (the old,
    strictly sequential layout) instead of overlapping the two lanes.

.PARAMETER WslLaneOnly
    INTERNAL. Used by this script to run the WSL lane as a child process:
    phase A of the WSL legs only, results written to -LaneResultFile.

.PARAMETER Affected
    Per-change run: topics derived from the changed paths (see DESCRIPTION).

.PARAMETER Base
    With -Affected: the git ref to diff against (default HEAD = uncommitted
    changes only; e.g. HEAD~3 or a commit id to cover recent commits too).

.PARAMETER Topic
    Per-change run with hand-picked topics, e.g. -Topic terrain,vehicle.
    Known topics: math jobs world backend articulation contact terrain
    vehicle drivetrain io determinism bench.

.PARAMETER Linux
    With -Affected/-Topic: also run the linux-release leg (topic tests).

.PARAMETER PlanOnly
    With -Affected/-Topic: print the decision (topics, reasons, legs,
    scenarios) and exit without building or testing anything.

.NOTES
    Layout (default):
      Phase A  Windows lane (release, debug, asan, msvc-release, greps) in
               this process, overlapped with the WSL lane (linux-release,
               linux-asan-ubsan, linux-tsan) in a child process. Every
               ctest here excludes the "walltime" label.
      Phase B  Serial wall-clock phase, started only after BOTH lanes have
               finished: each leg's "walltime"-labelled tests (benches that
               assert timing on that build, tests/bench/CMakeLists.txt), one
               leg at a time, with nothing else running on the machine.
      Then     the cross-platform state-hash compare.
    The two WSL sanitizer legs (linux-asan-ubsan, linux-tsan) configure and
    build one after the other, then run their ctest side by side with half
    the parallelism each (measured: fast 225 s -> 159 s, -Full ~463 s ->
    397 s, peak ~4.7 GB of WSL's 15 GB).
#>
[CmdletBinding()]
param(
    [switch]$SkipWsl,
    [switch]$SkipWindows,
    [string[]]$Only,
    [switch]$Full,
    [switch]$NoOverlap,
    [switch]$WslLaneOnly,
    [string]$LaneResultFile,
    [switch]$Affected,
    [string]$Base = 'HEAD',
    [string[]]$Topic,
    [switch]$Linux,
    [switch]$PlanOnly
)

$ErrorActionPreference = 'Continue'
$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$script:Results = New-Object System.Collections.Generic.List[object]
$script:LegTimings = New-Object System.Collections.Generic.List[object]
# Legs whose build succeeded this run, in run order: {Preset, Kind}. Phase B
# (the serial wall-clock phase) runs exactly these.
$script:BuiltLegs = New-Object System.Collections.Generic.List[object]

# -Only a,b arrives as ONE string when this script is started with -File
# (as the WSL-lane child process is), so accept comma-separated values too.
if ($Only) { $Only = @($Only | ForEach-Object { $_ -split ',' } | ForEach-Object { $_.Trim() } | Where-Object { $_ }) }
if ($Topic) { $Topic = @($Topic | ForEach-Object { $_ -split ',' } | ForEach-Object { $_.Trim().ToLower() } | Where-Object { $_ }) }

# Set by an -Affected/-Topic run: a ctest -L regex selecting the chosen
# topics (null = every test), and the hash-compare scenario subset (null =
# all of $script:HashCheckScenarios).
$script:TopicRegex = $null
$script:HashScenarioSelection = $null
$script:RunMode = if ($Full) { 'FULL' } else { 'FAST' }

function Add-Result {
    param([string]$Leg, [string]$Stage, [bool]$Pass, [string]$Detail = '', [double]$Seconds = 0)
    $script:Results.Add([pscustomobject]@{
        Leg = $Leg; Stage = $Stage; Pass = $Pass; Detail = $Detail; Seconds = $Seconds
    })
}

function Should-Run {
    param([string]$Name)
    if (-not $Only -or $Only.Count -eq 0) { return $true }
    return $Only -contains $Name
}

# Windows physical-core count (avoid oversubscribing when tests themselves
# spin up worker pools - see tests/determinism/CMakeLists.txt's PROCESSORS
# note). WSL sees 24 logical threads (PLAN.md section 2).
$script:WindowsParallel = 12
$script:WslParallel = 24

# CTest label selection for phase A. "walltime" is ALWAYS excluded here:
# those tests run in phase B, alone on the machine (see .NOTES above and
# tests/bench/CMakeLists.txt). FAST mode additionally excludes [long] (and,
# on sanitizer legs, the whole [validation] suite).
function Get-CtestLabelArgs {
    param([string]$LegKind) # 'release' or 'sanitizer'
    $excluded = @()
    if (-not $Full) {
        if ($LegKind -eq 'sanitizer') { $excluded += @('validation', 'long') } else { $excluded += 'long' }
    }
    $excluded += 'walltime'
    # ($args is a PowerShell automatic variable - never assign to it.)
    $labelArgs = @('-LE', ($excluded -join '|'))
    if ($script:TopicRegex) { $labelArgs += @('-L', $script:TopicRegex) }
    return $labelArgs
}

# ---------------------------------------------------------------------------
# Enter the VS 2022 Build Tools developer environment (clang-cl and cl are
# not on PATH otherwise - see tools/setup_dev_env.ps1 / CLAUDE.md).
#
# Uses vcvars64.bat (via cmd.exe) rather than Launch-VsDevShell.ps1: the
# latter turned out unreliable when run non-interactively from a script
# (observed failing here with "Could not start Developer PowerShell..." /
# vswhere-not-found even after refreshing PATH, despite working fine when
# typed interactively) - vcvars64.bat + parsing `set` output is the
# well-established robust alternative and does not depend on vswhere being
# resolvable on PATH at all.
# ---------------------------------------------------------------------------
function Enter-VsDevShell {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) {
        throw "vswhere.exe not found; VS 2022 Build Tools not installed (run tools/setup_dev_env.ps1)"
    }
    $vsPath = & $vswhere -products * -latest -property installationPath
    if (-not $vsPath) { throw "vswhere found no VS installation" }
    $vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
    if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found under $vsPath" }

    $envLines = cmd.exe /c "`"$vcvars`" && set"
    foreach ($line in $envLines) {
        if ($line -match '^([^=]+)=(.*)$') {
            [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process')
        }
    }
    if (-not (Get-Command clang-cl -ErrorAction SilentlyContinue)) {
        throw "vcvars64.bat ran but clang-cl is still not on PATH"
    }
}

# ---------------------------------------------------------------------------
# One Windows leg: cmake --preset / --build / ctest, each stage timed.
# ---------------------------------------------------------------------------
function Invoke-WindowsLeg {
    param([string]$Preset, [string]$LegKind = 'release')

    Write-Host ""
    Write-Host "=== Windows leg: $Preset ===" -ForegroundColor Cyan
    $legStart = Get-Date

    $t = Measure-Command { & cmake --preset $Preset }
    $configOk = ($LASTEXITCODE -eq 0)
    Add-Result -Leg $Preset -Stage 'configure' -Pass $configOk -Seconds $t.TotalSeconds
    if (-not $configOk) { return }

    $t = Measure-Command { & cmake --build --preset $Preset }
    $buildOk = ($LASTEXITCODE -eq 0)
    Add-Result -Leg $Preset -Stage 'build' -Pass $buildOk -Seconds $t.TotalSeconds
    if (-not $buildOk) { return }
    $script:BuiltLegs.Add([pscustomobject]@{ Preset = $Preset; Kind = 'windows' })

    $labelArgs = Get-CtestLabelArgs -LegKind $LegKind
    $ctestOut = $null
    $t = Measure-Command {
        $script:ctestOut = & ctest --preset $Preset --output-on-failure --parallel $script:WindowsParallel @labelArgs 2>&1
    }
    $ctestOk = ($LASTEXITCODE -eq 0)
    $summaryLine = ($script:ctestOut | Select-String 'tests passed' | Select-Object -Last 1).ToString()
    Add-Result -Leg $Preset -Stage 'ctest' -Pass $ctestOk -Detail $summaryLine -Seconds $t.TotalSeconds
    if (-not $ctestOk) { Write-Host ($script:ctestOut -join "`n") }

    $script:LegTimings.Add([pscustomobject]@{ Leg = $Preset; TotalSeconds = ((Get-Date) - $legStart).TotalSeconds })
}

# ---------------------------------------------------------------------------
# One WSL/Linux leg: same three steps, run inside WSL, building under the
# Linux filesystem (~/build/physics_sim/<preset>, set via CMakePresets.json),
# never under /mnt/s.
# ---------------------------------------------------------------------------
function Invoke-WslLeg {
    param([string]$Preset, [string]$LegKind = 'release')

    Write-Host ""
    Write-Host "=== WSL leg: $Preset ===" -ForegroundColor Cyan
    $legStart = Get-Date

    $cmd = "cd /mnt/s/claude_code/physics_sim && cmake --preset $Preset"
    $t = Measure-Command { wsl --cd ~ -- bash -lc $cmd }
    $configOk = ($LASTEXITCODE -eq 0)
    Add-Result -Leg $Preset -Stage 'configure' -Pass $configOk -Seconds $t.TotalSeconds
    if (-not $configOk) { return }

    $cmd = "cd /mnt/s/claude_code/physics_sim && cmake --build --preset $Preset"
    $t = Measure-Command { wsl --cd ~ -- bash -lc $cmd }
    $buildOk = ($LASTEXITCODE -eq 0)
    Add-Result -Leg $Preset -Stage 'build' -Pass $buildOk -Seconds $t.TotalSeconds
    if (-not $buildOk) { return }
    $script:BuiltLegs.Add([pscustomobject]@{ Preset = $Preset; Kind = 'wsl' })

    # Quote each arg for the bash -lc string below - the label regex itself
    # ("validation|long") contains a "|" that bash would otherwise parse as
    # a pipe.
    $labelArgsRaw = Get-CtestLabelArgs -LegKind $LegKind
    $labelArgs = ($labelArgsRaw | ForEach-Object { "'$_'" }) -join ' '
    $cmd = "cd /mnt/s/claude_code/physics_sim && ctest --preset $Preset --output-on-failure --parallel $script:WslParallel $labelArgs"
    $ctestOut = $null
    $t = Measure-Command { $script:ctestOut = wsl --cd ~ -- bash -lc $cmd }
    $ctestOk = ($LASTEXITCODE -eq 0)
    $summaryLine = (($script:ctestOut | Select-String 'tests passed') | Select-Object -Last 1).ToString()
    Add-Result -Leg $Preset -Stage 'ctest' -Pass $ctestOk -Detail $summaryLine -Seconds $t.TotalSeconds
    if (-not $ctestOk) { Write-Host ($script:ctestOut -join "`n") }

    $script:LegTimings.Add([pscustomobject]@{ Leg = $Preset; TotalSeconds = ((Get-Date) - $legStart).TotalSeconds })
}

# ---------------------------------------------------------------------------
# Godot-include grep: core/ must have zero Godot/GDExtension includes (D1).
# ---------------------------------------------------------------------------
function Invoke-GodotIncludeGrep {
    Write-Host ""
    Write-Host "=== Godot-include grep (core/ must stay Godot-free) ===" -ForegroundColor Cyan

    $coreDir = Join-Path $repoRoot 'core'
    $pattern = '#include\s*[<"](godot|gdextension)'
    $hits = Get-ChildItem -Path $coreDir -Recurse -File -Include *.h,*.hpp,*.hh,*.hxx,*.cpp,*.cc,*.cxx,*.inl |
        Select-String -Pattern $pattern

    if ($hits) {
        Add-Result -Leg 'godot-grep' -Stage 'grep' -Pass $false -Detail "$($hits.Count) match(es)"
        $hits | ForEach-Object { Write-Host "  $($_.Path):$($_.LineNumber): $($_.Line.Trim())" -ForegroundColor Red }
    } else {
        Add-Result -Leg 'godot-grep' -Stage 'grep' -Pass $true -Detail 'no Godot/GDExtension includes under core/'
    }
}

# ---------------------------------------------------------------------------
# Deterministic-math grep (PLAN.md D7 / vault PHYS-009 Step 1 rule 3): core/
# must never call the platform libm's transcendentals directly - ucrtbase
# (Windows) and glibc (Linux) round them differently, which is the proven
# root cause of PHYS-009's Windows/Linux articulation hash mismatch. Only
# core/src/math/detmath (the musl-derived port itself) and the two files
# that declare/forward the public ps::math wrapper are exempt.
#
# The regex requires the character immediately before the name to be
# neither a word character nor ':' - this is what stops "asin"/"atan2"/
# "sincos" etc. from being torn apart (e.g. matching "sin(" inside
# "asin(", or flagging "ps::math::sin(" because of the "::" before it) and
# what stops a longer identifier like "my_exp(" or "expect(" from matching
# "exp(" - see the accompanying Pester-style self-test invoked right after
# this function is defined, which is how this was verified rather than
# merely asserted.
# ---------------------------------------------------------------------------
$script:BannedTranscendentalNames = @('sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'exp', 'log', 'pow', 'sincos')

function Get-TranscendentalGrepPattern {
    $altNames = ($script:BannedTranscendentalNames | ForEach-Object { [regex]::Escape($_) }) -join '|'
    # (?:std::|::)? also catches an explicit ::sin( global-qualified call -
    # the lookbehind still stops this from matching "math::sin(" (the char
    # right before that "::" is 'h', a word character).
    return "(?<![\w:])(?:std::|::)?(?:$altNames)\s*\("
}

# Single source of truth for "does this one line contain a banned call",
# shared by the self-test and the real grep so they cannot drift apart.
# Strips // line comments first (this codebase writes prose like
# "exp(h * w_body_new)" in comments to describe the maths -
# core/src/articulation/instance.h - which must not trip this grep; no
# string literal in core/ contains a banned "name(" pattern, so this naive
# strip is safe here).
function Test-BannedTranscendentalLine {
    param([string]$Line, [string]$Pattern)
    $codeOnly = $Line -replace '//.*$', ''
    return [bool]($codeOnly -match $Pattern)
}

function Invoke-TranscendentalGrep {
    Write-Host ""
    Write-Host "=== Deterministic-math grep (core/ must not call platform-libm transcendentals - PLAN.md D7) ===" -ForegroundColor Cyan

    $coreDir = Join-Path $repoRoot 'core'
    $exemptPrefixes = @(
        (Join-Path $coreDir 'src\math\detmath'),
        (Join-Path $coreDir 'include\ps\math\transcendental.h'),
        (Join-Path $coreDir 'src\math\transcendental.cpp')
    )
    $pattern = Get-TranscendentalGrepPattern

    $files = Get-ChildItem -Path $coreDir -Recurse -File -Include *.h,*.hpp,*.hh,*.hxx,*.cpp,*.cc,*.cxx,*.inl |
        Where-Object {
            $full = $_.FullName
            -not ($exemptPrefixes | Where-Object { $full.StartsWith($_) })
        }

    $hits = New-Object System.Collections.Generic.List[object]
    foreach ($f in $files) {
        $lines = Get-Content -LiteralPath $f.FullName
        for ($i = 0; $i -lt $lines.Count; $i++) {
            if (Test-BannedTranscendentalLine -Line $lines[$i] -Pattern $pattern) {
                $hits.Add([pscustomobject]@{ Path = $f.FullName; Line = ($i + 1); Text = $lines[$i].Trim() })
            }
        }
    }

    if ($hits.Count -gt 0) {
        Add-Result -Leg 'detmath-grep' -Stage 'grep' -Pass $false -Detail "$($hits.Count) match(es)"
        $hits | ForEach-Object { Write-Host "  $($_.Path):$($_.Line): $($_.Text)" -ForegroundColor Red }
    } else {
        Add-Result -Leg 'detmath-grep' -Stage 'grep' -Pass $true -Detail 'no platform-libm transcendental calls under core/ (excluding core/src/math/detmath)'
    }
}

# Self-test for the grep above (false-positive/false-negative regression
# test for the pattern+strip pipeline, not for the codebase) - runs on
# every invocation since it's essentially free, and is what actually
# verified the "be careful with false positives such as asin in
# identifiers" requirement rather than merely asserting it.
function Test-TranscendentalGrepPattern {
    $pattern = Get-TranscendentalGrepPattern
    $shouldMatch = @(
        'return std::sin(x);',
        'z = ::sin(x) + 1.0;',
        'auto y = cos(x);',
        'r = atan2(y, x);',
        '  pow(x, y);',
        'real asin(real x);' # the bare name IS banned; transcendental.h only escapes the
                              # codebase scan by being exempted as a whole file, not by regex
    )
    $shouldNotMatch = @(
        'return ps::math::sin(x);',
        'return ps::math::detmath::sin(x);',
        'double my_expensive(int x) { return 0; }',
        'if (x < 0) return expect(x);',
        '// q_new = q_old * exp(h * w_body_new)',
        'int exp_data_index = 0;',
        'real x = 1.0; // atan2 of the wheel contact point'
    )

    $failures = 0
    foreach ($s in $shouldMatch) {
        if (-not (Test-BannedTranscendentalLine -Line $s -Pattern $pattern)) { Write-Host "  [grep self-test] expected MATCH, got none: '$s'" -ForegroundColor Red; $failures++ }
    }
    foreach ($s in $shouldNotMatch) {
        if (Test-BannedTranscendentalLine -Line $s -Pattern $pattern) { Write-Host "  [grep self-test] expected NO match, got one: '$s'" -ForegroundColor Red; $failures++ }
    }
    Add-Result -Leg 'detmath-grep' -Stage 'self-test' -Pass ($failures -eq 0) -Detail "$($shouldMatch.Count + $shouldNotMatch.Count) case(s), $failures failure(s)"
}

# ---------------------------------------------------------------------------
# Phase B: the serial wall-clock phase. Runs only after both lanes have been
# joined, so the timing-asserting benches get the machine to themselves: one
# leg at a time, one ctest invocation at a time, nothing else of this CI run
# alive (the WSL-lane child process has exited before this is called).
# ---------------------------------------------------------------------------
function Invoke-WalltimePhase {
    Write-Host ""
    Write-Host "=== Phase B: serial wall-clock phase (walltime-labelled tests, one leg at a time, machine otherwise idle) ===" -ForegroundColor Cyan
    Write-Host ("  started {0:HH:mm:ss} - both lanes finished, no other CI work running" -f (Get-Date))
    foreach ($leg in $script:BuiltLegs) {
        $preset = $leg.Preset
        # Repeated -L options are ANDed by ctest (>= 3.x; verified on 4.2/4.4).
        $topicArgs = @()
        if ($script:TopicRegex) { $topicArgs = @('-L', $script:TopicRegex) }
        if ($leg.Kind -eq 'windows') {
            $t = Measure-Command { $script:wtOut = & ctest --preset $preset --output-on-failure -L walltime @topicArgs 2>&1 }
        } else {
            $topicStr = ($topicArgs | ForEach-Object { "'$_'" }) -join ' '
            $cmd = "cd /mnt/s/claude_code/physics_sim && ctest --preset $preset --output-on-failure -L walltime $topicStr 2>&1"
            $t = Measure-Command { $script:wtOut = wsl --cd ~ -- bash -lc $cmd }
        }
        $ok = ($LASTEXITCODE -eq 0)
        $joined = ($script:wtOut -join "`n")
        if ($joined -match 'No tests were found') {
            Add-Result -Leg $preset -Stage 'walltime' -Pass $true -Detail 'no timing-asserting bench on this build' -Seconds $t.TotalSeconds
            continue
        }
        $line = $script:wtOut | Select-String 'tests passed' | Select-Object -Last 1
        $detail = if ($line) { $line.ToString() } else { 'no ctest summary line' }
        Add-Result -Leg $preset -Stage 'walltime' -Pass $ok -Detail $detail -Seconds $t.TotalSeconds
        if (-not $ok) { Write-Host $joined }
    }
    Write-Host ("  finished {0:HH:mm:ss}" -f (Get-Date))
}

# ---------------------------------------------------------------------------
# Cross-platform state-hash comparison (PLAN.md D7 / vault PHYS-009 Step 1
# rule 4): run ps_run on a fixed scenario set on both a Windows build and
# the WSL/Linux build and compare state_hash. Cheap (a few seconds total),
# so it runs in fast CI too, whenever both sides were actually built this
# invocation (skipped with a note otherwise, e.g. -SkipWindows/-SkipWsl).
# ---------------------------------------------------------------------------
$script:HashCheckScenarios = @('free_fall.json', 'pendulum.json', 'joint_chain.json', 'articulation_chain.json', 'multi_articulation.json', 'contact_articulation.json', 'terrain_tiles.json', 'terrain_multi_interest.json', 'vehicle_step_steer.json', 'vehicle_powertrain.json')

function Get-PsRunStateHash {
    param([string]$PsRunPath, [string]$ScenarioPath, [string]$CsvOutPath, [switch]$Wsl)

    if ($Wsl) {
        $cmd = "`"$PsRunPath`" `"$ScenarioPath`" `"$CsvOutPath`""
        $out = wsl --cd ~ -- bash -lc $cmd
    } else {
        $out = & $PsRunPath $ScenarioPath $CsvOutPath 2>&1
    }
    if ($LASTEXITCODE -ne 0) { return $null }
    $joined = ($out -join "`n")
    if ($joined -match 'state_hash=0x([0-9a-fA-F]+)') { return $Matches[1].ToLower() }
    return $null
}

function Invoke-CrossPlatformHashCheck {
    Write-Host ""
    Write-Host "=== Cross-platform state-hash comparison (Windows vs Linux, PLAN.md D7) ===" -ForegroundColor Cyan

    $winPsRun = Join-Path $repoRoot 'out\build\release\tools\ps_run\ps_run.exe'
    if (-not (Test-Path $winPsRun)) {
        Add-Result -Leg 'hash-compare' -Stage 'compare' -Pass $true -Detail 'skipped: no Windows release ps_run.exe (leg not built this run)'
        return
    }
    $linuxProbe = wsl --cd ~ -- bash -lc "test -x ~/build/physics_sim/linux-release/tools/ps_run/ps_run && echo OK"
    if (($linuxProbe -join '') -notmatch 'OK') {
        Add-Result -Leg 'hash-compare' -Stage 'compare' -Pass $true -Detail 'skipped: no Linux linux-release ps_run (leg not built this run)'
        return
    }

    $tmpWin = Join-Path $env:TEMP 'ps_hashcheck'
    New-Item -ItemType Directory -Force -Path $tmpWin | Out-Null

    $scenarios = if ($null -ne $script:HashScenarioSelection) { @($script:HashScenarioSelection) } else { $script:HashCheckScenarios }
    if ($scenarios.Count -eq 0) {
        Add-Result -Leg 'hash-compare' -Stage 'compare' -Pass $true -Detail 'skipped: no scenario belongs to the selected topics'
        return
    }
    $mismatches = 0
    foreach ($scenario in $scenarios) {
        $scenarioPath = Join-Path $repoRoot "data\scenarios\$scenario"
        $winCsv = Join-Path $tmpWin "$scenario.win.csv"
        $winHash = Get-PsRunStateHash -PsRunPath $winPsRun -ScenarioPath $scenarioPath -CsvOutPath $winCsv

        $linuxHash = Get-PsRunStateHash -PsRunPath '~/build/physics_sim/linux-release/tools/ps_run/ps_run' `
            -ScenarioPath "/mnt/s/claude_code/physics_sim/data/scenarios/$scenario" `
            -CsvOutPath "/home/sebas/ps_hashcheck_$scenario.csv" -Wsl

        if (-not $winHash -or -not $linuxHash) {
            Add-Result -Leg 'hash-compare' -Stage $scenario -Pass $false -Detail "ps_run failed to report a hash (win='$winHash' linux='$linuxHash')"
            $mismatches++
        } elseif ($winHash -ne $linuxHash) {
            Add-Result -Leg 'hash-compare' -Stage $scenario -Pass $false -Detail "MISMATCH win=0x$winHash linux=0x$linuxHash"
            Write-Host "  $scenario`: win=0x$winHash linux=0x$linuxHash" -ForegroundColor Red
            $mismatches++
        } else {
            Add-Result -Leg 'hash-compare' -Stage $scenario -Pass $true -Detail "0x$winHash"
        }
    }
    if ($mismatches -eq 0) {
        Write-Host "  All $($scenarios.Count) scenario hashes match Windows vs Linux." -ForegroundColor Green
    }
}

# ===========================================================================
# Run legs
# ===========================================================================
$script:WallStart = Get-Date

# release-kind legs run everything except [long]; sanitizer-kind legs run
# only unit+determinism+[sanitizer] (see Get-CtestLabelArgs).
$releaseLegs = @('release')
$sanitizerLegs = @('debug', 'asan')

# Runs a bash script inside WSL (written under out/ci_tmp with LF endings:
# anything with &, (), $? or quotes does not survive `bash -lc <string>`
# from PowerShell intact).
function Invoke-WslScript {
    param([string]$Name, [string[]]$Lines)
    $dir = Join-Path $repoRoot 'out\ci_tmp'
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    $path = Join-Path $dir $Name
    [System.IO.File]::WriteAllText($path, (($Lines -join "`n") + "`n"))
    return (wsl --cd ~ -- bash "/mnt/s/claude_code/physics_sim/out/ci_tmp/$Name")
}

# The two WSL sanitizer legs: configure + build one after the other, then
# both ctests side by side, --parallel split between them. Nothing
# timing-sensitive runs here (walltime is excluded from phase A), and the
# [heavy] determinism tests only get slower, not wrong, when not alone.
function Invoke-WslSanitizerPair {
    param([string[]]$Presets)
    $ready = @()
    $prep = @{}
    foreach ($preset in $Presets) {
        Write-Host ""
        Write-Host "=== WSL leg: $preset (ctest runs side by side with the other sanitizer leg) ===" -ForegroundColor Cyan
        $legStart = Get-Date
        $cmd = "cd /mnt/s/claude_code/physics_sim && cmake --preset $preset"
        $t = Measure-Command { wsl --cd ~ -- bash -lc $cmd }
        $ok = ($LASTEXITCODE -eq 0)
        Add-Result -Leg $preset -Stage 'configure' -Pass $ok -Seconds $t.TotalSeconds
        if (-not $ok) { continue }
        $cmd = "cd /mnt/s/claude_code/physics_sim && cmake --build --preset $preset"
        $t = Measure-Command { wsl --cd ~ -- bash -lc $cmd }
        $ok = ($LASTEXITCODE -eq 0)
        Add-Result -Leg $preset -Stage 'build' -Pass $ok -Seconds $t.TotalSeconds
        if (-not $ok) { continue }
        $script:BuiltLegs.Add([pscustomobject]@{ Preset = $preset; Kind = 'wsl' })
        $ready += $preset
        $prep[$preset] = ((Get-Date) - $legStart).TotalSeconds
    }
    if ($ready.Count -eq 0) { return }

    $labelArgs = ((Get-CtestLabelArgs -LegKind 'sanitizer') | ForEach-Object { "'$_'" }) -join ' '
    $par = [math]::Max(1, [int][math]::Floor($script:WslParallel / $ready.Count))
    $lines = @('cd /mnt/s/claude_code/physics_sim')
    foreach ($preset in $ready) {
        $lines += "( s=`$(date +%s.%N); ctest --preset $preset --output-on-failure --parallel $par $labelArgs > `$HOME/ps_ci_$preset.log 2>&1; echo `$? > `$HOME/ps_ci_$preset.exit; echo `"`$s `$(date +%s.%N)`" > `$HOME/ps_ci_$preset.time ) &"
    }
    $lines += 'wait'
    Write-Host ("  running ctest for {0} side by side (--parallel {1} each)" -f ($ready -join ' + '), $par)
    $null = Invoke-WslScript -Name 'sanitizer_pair.sh' -Lines $lines

    foreach ($preset in $ready) {
        $log = wsl --cd ~ -- bash -lc "cat ~/ps_ci_$preset.log"
        $exit = (wsl --cd ~ -- bash -lc "cat ~/ps_ci_$preset.exit") -join ''
        $times = ((wsl --cd ~ -- bash -lc "cat ~/ps_ci_$preset.time") -join '').Trim() -split '\s+'
        $secs = 0.0
        if ($times.Count -eq 2) { $secs = [double]::Parse($times[1], [Globalization.CultureInfo]::InvariantCulture) - [double]::Parse($times[0], [Globalization.CultureInfo]::InvariantCulture) }
        $ok = ($exit.Trim() -eq '0')
        $line = $log | Select-String 'tests passed' | Select-Object -Last 1
        $detail = if ($line) { $line.ToString() } else { 'no ctest summary line' }
        Add-Result -Leg $preset -Stage 'ctest' -Pass $ok -Detail $detail -Seconds $secs
        if (-not $ok) { Write-Host ($log -join "`n") }
        $script:LegTimings.Add([pscustomobject]@{ Leg = $preset; TotalSeconds = $prep[$preset] + $secs })
    }
}

function Invoke-WslLane {
    if (Should-Run 'linux-release') { Invoke-WslLeg -Preset 'linux-release' -LegKind 'release' }
    $pair = @('linux-asan-ubsan', 'linux-tsan') | Where-Object { Should-Run $_ }
    if (@($pair).Count -eq 2) {
        Invoke-WslSanitizerPair -Presets $pair
    } else {
        foreach ($preset in $pair) { Invoke-WslLeg -Preset $preset -LegKind 'sanitizer' }
    }
}

# ===========================================================================
# -Affected / -Topic: per-change runs
# ===========================================================================
$script:KnownTopics = @('math', 'jobs', 'world', 'backend', 'articulation', 'contact', 'terrain', 'vehicle', 'drivetrain', 'io', 'determinism', 'bench')

# Path -> topics. First matching rule wins; paths are repo-relative with '/'.
# Kind: 'topics' (run Topics), 'everything' (foundational: default fast run),
# 'doc' (greps only), 'none' (outside core CI: build + greps only, with Note),
# 'testfile' (topics read from the file's own Catch2 tags), 'scenario',
# 'cmake-test-registration' (a tests/{unit,validation,determinism}/CMakeLists.txt
# edit that ONLY adds/removes a bare test_*.cpp/.h line in the executable's
# source list - see Get-AffectedPlan's handler below - contributes no topics
# of its own and does NOT escalate to 'everything'; any other change to that
# file, e.g. target_link_libraries/add_executable(/compile definitions/
# catch_discover_tests, still does).
# A path matching NO rule makes the whole run fall back to the default fast
# suite (never to nothing).
$script:PathRules = @(
    @{ Re = '\.md$'; Kind = 'doc' },
    @{ Re = '^(\.gitignore|\.gitattributes|\.claude/)'; Kind = 'doc' },
    @{ Re = '^core/(include/ps|src)/(math|jobs|world|backend)/'; Kind = 'everything'; Why = 'foundational core (math/jobs/world/backend)' },
    @{ Re = '^core/include/ps/(types|version)\.h$|^core/src/(version|jolt_proof)\.cpp$'; Kind = 'everything'; Why = 'foundational core types' },
    @{ Re = '^tests/(unit|validation|determinism)/CMakeLists\.txt$'; Kind = 'cmake-test-registration' },
    @{ Re = '(^|/)CMakeLists\.txt$|^CMakePresets\.json$'; Kind = 'everything'; Why = 'build system' },
    @{ Re = '^tools/ci\.ps1$|^tools/tsan_suppressions\.txt$'; Kind = 'everything'; Why = 'CI itself' },
    @{ Re = '^tools/common/|^tests/support/'; Kind = 'everything'; Why = 'linked into every test' },
    @{ Re = '^core/(include/ps|src)/articulation/'; Kind = 'topics'; Topics = @('articulation', 'contact') },
    @{ Re = '^core/(include/ps|src)/terrain/'; Kind = 'topics'; Topics = @('terrain') },
    @{ Re = '^core/(include/ps|src)/vehicle/'; Kind = 'topics'; Topics = @('vehicle') },
    @{ Re = '^core/(include/ps|src)/drivetrain/'; Kind = 'topics'; Topics = @('drivetrain') },
    @{ Re = '^core/(include/ps|src)/io/'; Kind = 'topics'; Topics = @('io'); AllScenarios = $true },
    @{ Re = '^data/(vehicles|tyres|engines|gearboxes)/'; Kind = 'topics'; Topics = @('vehicle') },
    @{ Re = '^data/surfaces/'; Kind = 'topics'; Topics = @('vehicle', 'terrain', 'io') },
    @{ Re = '^data/tracks/'; Kind = 'topics'; Topics = @('terrain', 'io') },
    @{ Re = '^data/scenarios/'; Kind = 'scenario' },
    @{ Re = '^tools/ps_run/'; Kind = 'topics'; Topics = @('io'); AllScenarios = $true },
    @{ Re = '^tests/bench/bench_force_elements\.cpp$'; Kind = 'topics'; Topics = @('jobs') },
    @{ Re = '^tests/bench/bench_contact_coupling\.cpp$'; Kind = 'topics'; Topics = @('contact') },
    @{ Re = '^tests/bench/bench_terrain_tiles\.cpp$'; Kind = 'topics'; Topics = @('terrain') },
    @{ Re = '^tests/(unit|validation|determinism)/[^/]+\.(cpp|h)$'; Kind = 'testfile' },
    @{ Re = '^(tools/sweep/|tests/bench/energy_study\.cpp$)'; Kind = 'none'; Note = 'no ctest coverage; the release build still compiles it' },
    @{ Re = '^adapters/'; Kind = 'none'; Note = 'Godot adapter: not built by ci.ps1 - run adapters\godot\smoke_test.ps1' },
    @{ Re = '^(data/models/|tools/(assetgen|trackgen|plots)/|tools/setup_dev_env\.ps1$)'; Kind = 'none'; Note = 'not used by core builds or tests' }
)

# Topic -> hash-compare scenarios (a scenario runs if any of its topics is selected).
$script:ScenarioTopics = @{
    'free_fall.json' = @('backend'); 'pendulum.json' = @('backend'); 'joint_chain.json' = @('backend')
    'articulation_chain.json' = @('articulation'); 'multi_articulation.json' = @('articulation')
    'contact_articulation.json' = @('articulation', 'contact')
    'terrain_tiles.json' = @('terrain'); 'terrain_multi_interest.json' = @('terrain')
    'vehicle_step_steer.json' = @('vehicle'); 'vehicle_powertrain.json' = @('vehicle')
}
# Scenario inputs that are not themselves hash-compare scenarios.
$script:ScenarioInputTopics = @{ 'terrain_spec_small.json' = @('terrain') }

# 'cmake-test-registration' allow-list (see $script:PathRules above): a
# changed line - diff-prefixed ('+'/'-', the normal tracked-file case) or
# unprefixed (Get-ChangedText's Untracked/whole-file case) - passes only if
# it is NOTHING BUT a bare test_*.cpp/.h source-list entry. Deliberately
# narrow: it does not try to recognise a comment, a blank line already
# filtered by Get-ChangedText, or anything else - one non-matching line
# (target_link_libraries, add_executable(, a CMake comment, ...) sends the
# whole file back to the ordinary 'everything' (build-system) treatment.
$script:CmakeTestLineAllow = '^[+-]?\s*test_[A-Za-z0-9_]+\.(cpp|h)\s*$'

# Diff-content patterns that make a change determinism- or platform-
# sensitive (Linux legs added). Threading ones also add linux-tsan.
$script:SensitivePatterns = @(
    @{ Re = 'unordered_(map|set|multimap|multiset)'; Why = 'unordered container (iteration order differs MSVC vs libstdc++)' },
    @{ Re = 'std::(sort|stable_sort|partial_sort|nth_element)\b'; Why = 'ordering' },
    @{ Re = '\b(std::)?(sin|cos|tan|asin|acos|atan|atan2|exp|log|pow|sqrt|cbrt|hypot|fma)\s*\('; Why = 'libm / FP-contract-sensitive math' },
    @{ Re = 'std::(thread|atomic|mutex|condition_variable|shared_mutex)|parallel_for|parallel_batches|JobSystem'; Why = 'threading'; Tsan = $true }
)

function Get-ChangedPaths {
    param([string]$BaseRef)
    $tracked = @(git diff --name-only $BaseRef -- 2>$null)
    if ($LASTEXITCODE -ne 0) { throw "git diff against '$BaseRef' failed - is it a valid ref?" }
    $untracked = @(git ls-files --others --exclude-standard 2>$null)
    return @(($tracked + $untracked) | Where-Object { $_ } | Sort-Object -Unique)
}

function Get-TestFileTopics {
    param([string]$RelPath)
    $full = Join-Path $repoRoot $RelPath
    if (-not (Test-Path -LiteralPath $full)) { return @() }   # deleted file
    $text = Get-Content -LiteralPath $full -Raw
    $tags = [regex]::Matches($text, '"(\[[^"]*\])"') | ForEach-Object { [regex]::Matches($_.Groups[1].Value, '\[([^\]]*)\]') | ForEach-Object { $_.Groups[1].Value } }
    return @($tags | Where-Object { $script:KnownTopics -contains $_ } | Sort-Object -Unique)
}

function Get-ChangedText {
    param([string]$RelPath, [string]$BaseRef, [bool]$Untracked)
    if ($Untracked) {
        $full = Join-Path $repoRoot $RelPath
        if (Test-Path -LiteralPath $full) { return (Get-Content -LiteralPath $full -Raw) }
        return ''
    }
    $diff = git diff $BaseRef -- $RelPath 2>$null
    return (($diff | Where-Object { $_ -match '^[+-]' -and $_ -notmatch '^(\+\+\+|---) ' -and $_ -notmatch '^[+-]\s*//' }) -join "`n")
}

# Returns the plan for an -Affected/-Topic run.
function Get-AffectedPlan {
    $plan = [pscustomobject]@{
        Mode = 'topics'; Topics = New-Object System.Collections.Generic.List[string]
        Scenarios = New-Object System.Collections.Generic.List[string]; AllScenarios = $false
        Linux = [bool]$Linux; Tsan = $false; Reasons = New-Object System.Collections.Generic.List[string]
        Notes = New-Object System.Collections.Generic.List[string]
    }
    function Add-Topics($list, $why) {
        foreach ($t in $list) { if (-not $plan.Topics.Contains($t)) { $plan.Topics.Add($t) } }
        $plan.Reasons.Add(("  {0,-40} <- {1}" -f (($list | Sort-Object) -join ','), $why))
    }

    if ($Topic) {
        $bad = @($Topic | Where-Object { $script:KnownTopics -notcontains $_ })
        if ($bad.Count -gt 0) { throw "unknown topic(s): $($bad -join ', ') (known: $($script:KnownTopics -join ' '))" }
        Add-Topics $Topic '-Topic (chosen by hand)'
    } else {
        $paths = Get-ChangedPaths -BaseRef $Base
        $untracked = @(git ls-files --others --exclude-standard 2>$null)
        $plan.Reasons.Add("changed paths vs '$Base' (working tree + untracked): $($paths.Count)")
        if ($paths.Count -eq 0) {
            $plan.Mode = 'greps'
            $plan.Reasons.Add("  nothing changed vs '$Base' - greps only (use -Base HEAD~N to cover recent commits)")
            return $plan
        }
        $codeSeen = $false
        foreach ($path in $paths) {
            $rule = $script:PathRules | Where-Object { $path -match $_.Re } | Select-Object -First 1
            if (-not $rule) {
                $plan.Mode = 'fallback'
                $plan.Reasons.Add("  UNMAPPED: $path - no rule in `$script:PathRules, falling back to the default fast run")
                continue
            }
            switch ($rule.Kind) {
                'doc' { $plan.Reasons.Add("  (doc) $path") }
                'none' { $codeSeen = $true; $plan.Reasons.Add("  (no tests) $path"); if (-not $plan.Notes.Contains($rule.Note)) { $plan.Notes.Add($rule.Note) } }
                'everything' { if ($plan.Mode -ne 'fallback') { $plan.Mode = 'everything' }; $plan.Reasons.Add("  FOUNDATIONAL: $path ($($rule.Why)) - default fast run; run -Full before calling it done") }
                'topics' { $codeSeen = $true; Add-Topics $rule.Topics $path; if ($rule.AllScenarios) { $plan.AllScenarios = $true } }
                'testfile' {
                    $codeSeen = $true
                    $tt = Get-TestFileTopics -RelPath $path
                    if ($tt.Count -eq 0) { $plan.Mode = 'fallback'; $plan.Reasons.Add("  UNMAPPED: $path - test file with no topic tag (or deleted), falling back to the default fast run") }
                    else { Add-Topics $tt "$path (its own Catch2 tags)" }
                }
                'cmake-test-registration' {
                    $codeSeen = $true
                    $text = Get-ChangedText -RelPath $path -BaseRef $Base -Untracked ($untracked -contains $path)
                    $lines = @($text -split "`n" | Where-Object { $_.Trim().Length -gt 0 })
                    $bad = @($lines | Where-Object { $_ -notmatch $script:CmakeTestLineAllow })
                    if ($lines.Count -eq 0 -or $bad.Count -gt 0) {
                        if ($plan.Mode -ne 'fallback') { $plan.Mode = 'everything' }
                        $plan.Reasons.Add("  FOUNDATIONAL: $path (test registration changed more than its test_*.cpp/.h source list, e.g. target_link_libraries/add_executable(/compile definitions) - default fast run; run -Full before calling it done")
                    } else {
                        $plan.Reasons.Add("  (cmake) $path - only test_*.cpp/.h source-list line(s) added/removed; topics come from that test file's own Catch2 tags")
                    }
                }
                'scenario' {
                    $codeSeen = $true
                    $name = Split-Path $path -Leaf
                    if ($script:ScenarioTopics.ContainsKey($name)) {
                        Add-Topics (@('io') + $script:ScenarioTopics[$name]) $path
                        if (-not $plan.Scenarios.Contains($name)) { $plan.Scenarios.Add($name) }
                    } elseif ($script:ScenarioInputTopics.ContainsKey($name)) {
                        Add-Topics (@('io') + $script:ScenarioInputTopics[$name]) $path
                    } else {
                        Add-Topics @('io') "$path (unknown scenario file: every hash scenario)"; $plan.AllScenarios = $true
                    }
                }
            }
            # Determinism/platform sensitivity of the changed PRODUCTION code
            # (core/, tools/): the diff's own added/removed lines, comments
            # excluded. Test sources are not scanned - a test name that
            # mentions parallel_for does not make a change platform-sensitive.
            if ($rule.Kind -eq 'topics' -and $path -match '^(core|tools)/.*\.(cpp|h|hpp)$') {
                $text = Get-ChangedText -RelPath $path -BaseRef $Base -Untracked ($untracked -contains $path)
                foreach ($sp in $script:SensitivePatterns) {
                    if ($text -match $sp.Re) {
                        $plan.Linux = $true
                        if ($sp.Tsan) { $plan.Tsan = $true }
                        $plan.Reasons.Add("  SENSITIVE: $path touches '$($Matches[0])' ($($sp.Why)) - adds linux-release$(if ($sp.Tsan) { ' + linux-tsan' })")
                    }
                }
            }
        }
        if ($plan.Mode -eq 'topics' -and $plan.Topics.Count -eq 0) {
            $plan.Mode = if ($codeSeen) { 'build' } else { 'greps' }
        }
    }

    foreach ($sc in $script:HashCheckScenarios) {
        $hit = $plan.AllScenarios -or ($script:ScenarioTopics[$sc] | Where-Object { $plan.Topics.Contains($_) })
        if ($hit -and -not $plan.Scenarios.Contains($sc)) { $plan.Scenarios.Add($sc) }
    }
    return $plan
}

function Invoke-AffectedRun {
    param($Plan)
    Write-Host ""
    Write-Host "=== -Affected/-Topic decision ===" -ForegroundColor Cyan
    $Plan.Reasons | ForEach-Object { Write-Host $_ }
    $Plan.Notes | ForEach-Object { Write-Host "  NOTE: $_" -ForegroundColor Yellow }

    if ($Plan.Mode -in @('fallback', 'everything')) {
        Write-Host "  => default fast run (all legs)" -ForegroundColor Yellow
        if ($PlanOnly) { exit 0 }
        return $false   # caller continues with the normal full-leg flow
    }
    if ($PlanOnly) {
        Write-Host ("  => mode={0} topics=[{1}] scenarios=[{2}] linux={3} tsan={4}" -f $Plan.Mode, ($Plan.Topics -join ','), ($Plan.Scenarios -join ','), $Plan.Linux, $Plan.Tsan) -ForegroundColor Yellow
        exit 0
    }
    $script:RunMode = "AFFECTED: $(if ($Plan.Topics.Count) { $Plan.Topics -join ',' } else { $Plan.Mode })"

    Enter-VsDevShell
    if ($Plan.Mode -eq 'greps') {
        Write-Host "  => greps only" -ForegroundColor Yellow
    } else {
        if ($Plan.Topics.Count -gt 0) {
            $script:TopicRegex = '^(' + (($Plan.Topics | Sort-Object) -join '|') + ')$'
            Write-Host "  => topics: $($Plan.Topics -join ', ')  (ctest -L '$script:TopicRegex')" -ForegroundColor Yellow
        } else {
            Write-Host "  => build only (changed code has no ctest topic)" -ForegroundColor Yellow
        }
        $script:HashScenarioSelection = @($Plan.Scenarios)
        Write-Host "  => hash compare: $(if ($Plan.Scenarios.Count) { $Plan.Scenarios -join ', ' } else { 'none' })" -ForegroundColor Yellow
        Write-Host "  => Linux legs: $(if ($Plan.Linux) { 'linux-release' + $(if ($Plan.Tsan) { ' + linux-tsan' } else { '' }) } else { 'no (not determinism/platform-sensitive; -Linux forces linux-release)' })" -ForegroundColor Yellow

        if ($Plan.Topics.Count -gt 0) {
            Invoke-WindowsLeg -Preset 'release' -LegKind 'release'
        } else {
            $t = Measure-Command { & cmake --preset release; if ($LASTEXITCODE -eq 0) { & cmake --build --preset release } }
            Add-Result -Leg 'release' -Stage 'build' -Pass ($LASTEXITCODE -eq 0) -Seconds $t.TotalSeconds
        }
        if ($Plan.Linux -and $Plan.Topics.Count -gt 0) {
            Invoke-WslLeg -Preset 'linux-release' -LegKind 'release'
            if ($Plan.Tsan) { Invoke-WslLeg -Preset 'linux-tsan' -LegKind 'sanitizer' }
        } elseif ($Plan.Scenarios.Count -gt 0) {
            # The hash compare needs a CURRENT Linux ps_run; build just that target.
            Write-Host ""
            Write-Host "=== WSL: building linux-release ps_run only (for the hash compare) ===" -ForegroundColor Cyan
            $cmd = "cd /mnt/s/claude_code/physics_sim && cmake --preset linux-release > /dev/null && cmake --build --preset linux-release --target ps_run"
            $t = Measure-Command { wsl --cd ~ -- bash -lc $cmd }
            Add-Result -Leg 'linux-release' -Stage 'ps_run' -Pass ($LASTEXITCODE -eq 0) -Seconds $t.TotalSeconds
        }
    }
    Invoke-GodotIncludeGrep
    Test-TranscendentalGrepPattern
    Invoke-TranscendentalGrep
    if ($Plan.Mode -ne 'greps') {
        $phaseBStart = Get-Date
        if ($Plan.Topics.Count -gt 0) { Invoke-WalltimePhase }
        $script:PhaseTimings.Add([pscustomobject]@{ Phase = 'walltime (topic benches)'; TotalSeconds = ((Get-Date) - $phaseBStart).TotalSeconds })
        $hashStart = Get-Date
        Invoke-CrossPlatformHashCheck
        $script:PhaseTimings.Add([pscustomobject]@{ Phase = 'hash compare'; TotalSeconds = ((Get-Date) - $hashStart).TotalSeconds })
    }
    return $true
}

# --- Child mode: the WSL lane only, results handed back through a file. ---
if ($WslLaneOnly) {
    Invoke-WslLane
    [pscustomobject]@{
        # .ToArray(): PowerShell 5.1 throws "Argument types do not match"
        # for @() around a generic List inside a hashtable literal.
        Results = $script:Results.ToArray()
        LegTimings = $script:LegTimings.ToArray()
        BuiltLegs = $script:BuiltLegs.ToArray()
    } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $LaneResultFile -Encoding UTF8
    exit 0
}

$script:PhaseTimings = New-Object System.Collections.Generic.List[object]
$overlap = (-not $SkipWindows) -and (-not $SkipWsl) -and (-not $NoOverlap)

$affectedDone = $false
if ($Affected -or $Topic) {
    $affectedDone = Invoke-AffectedRun -Plan (Get-AffectedPlan)
}
if (-not $affectedDone) {

# --- Phase A, WSL lane: started first so it overlaps the Windows lane. ---
$wslProc = $null
if ($overlap) {
    $laneTag = "ps_ci_wsl_lane_$PID"
    $laneResultPath = Join-Path $env:TEMP "$laneTag.json"
    $laneLogPath = Join-Path $env:TEMP "$laneTag.log"
    $laneErrPath = Join-Path $env:TEMP "$laneTag.err.log"
    Remove-Item -LiteralPath $laneResultPath, $laneLogPath, $laneErrPath -ErrorAction SilentlyContinue
    $childArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', "`"$PSCommandPath`"", '-WslLaneOnly', '-LaneResultFile', "`"$laneResultPath`"")
    if ($Full) { $childArgs += '-Full' }
    if ($Only) { $childArgs += @('-Only', ($Only -join ',')) }
    $wslLaneStart = Get-Date
    $wslProc = Start-Process -FilePath 'powershell.exe' -ArgumentList $childArgs -NoNewWindow -PassThru `
        -RedirectStandardOutput $laneLogPath -RedirectStandardError $laneErrPath
    # Touch the handle now: a Start-Process -PassThru object that never had
    # its handle opened reports an empty ExitCode and no ExitTime after exit.
    $null = $wslProc.Handle
    Write-Host ("=== Phase A: WSL lane started in a child process (pid {0}) at {1:HH:mm:ss}, overlapping the Windows lane ===" -f $wslProc.Id, $wslLaneStart) -ForegroundColor Cyan
}

# --- Phase A, Windows lane (this process). ---
if (-not $SkipWindows) {
    $winLaneStart = Get-Date
    Enter-VsDevShell

    foreach ($preset in $releaseLegs) {
        if (Should-Run $preset) { Invoke-WindowsLeg -Preset $preset -LegKind 'release' }
    }
    foreach ($preset in $sanitizerLegs) {
        if (Should-Run $preset) { Invoke-WindowsLeg -Preset $preset -LegKind 'sanitizer' }
    }
    if (Should-Run 'msvc-release') { Invoke-WindowsLeg -Preset 'msvc-release' -LegKind 'release' }

    Invoke-GodotIncludeGrep
    Test-TranscendentalGrepPattern
    Invoke-TranscendentalGrep
    $script:PhaseTimings.Add([pscustomobject]@{ Phase = 'A: Windows lane'; TotalSeconds = ((Get-Date) - $winLaneStart).TotalSeconds })
}

# --- Phase A, WSL lane: join the child, or run it here without overlap. ---
if ($overlap) {
    $wslProc.WaitForExit()
    $script:PhaseTimings.Add([pscustomobject]@{ Phase = 'A: WSL lane (overlapped)'; TotalSeconds = ($wslProc.ExitTime - $wslLaneStart).TotalSeconds })
    Write-Host ""
    Write-Host "=== WSL lane output (child process) ===" -ForegroundColor Cyan
    if (Test-Path -LiteralPath $laneLogPath) { Get-Content -LiteralPath $laneLogPath | ForEach-Object { Write-Host $_ } }
    if ((Test-Path -LiteralPath $laneErrPath) -and (Get-Item -LiteralPath $laneErrPath).Length -gt 0) {
        Get-Content -LiteralPath $laneErrPath | ForEach-Object { Write-Host $_ }
    }
    if (Test-Path -LiteralPath $laneResultPath) {
        $lane = Get-Content -LiteralPath $laneResultPath -Raw | ConvertFrom-Json
        foreach ($r in @($lane.Results)) { if ($r) { Add-Result -Leg $r.Leg -Stage $r.Stage -Pass ([bool]$r.Pass) -Detail $r.Detail -Seconds ([double]$r.Seconds) } }
        foreach ($l in @($lane.LegTimings)) { if ($l) { $script:LegTimings.Add([pscustomobject]@{ Leg = $l.Leg; TotalSeconds = [double]$l.TotalSeconds }) } }
        foreach ($b in @($lane.BuiltLegs)) { if ($b) { $script:BuiltLegs.Add([pscustomobject]@{ Preset = $b.Preset; Kind = $b.Kind }) } }
    } else {
        Add-Result -Leg 'wsl-lane' -Stage 'child' -Pass $false -Detail "WSL-lane child process (exit code $($wslProc.ExitCode)) wrote no result file"
    }
} elseif (-not $SkipWsl) {
    $wslLaneStart = Get-Date
    Invoke-WslLane
    $script:PhaseTimings.Add([pscustomobject]@{ Phase = 'A: WSL lane (sequential)'; TotalSeconds = ((Get-Date) - $wslLaneStart).TotalSeconds })
}

# --- Phase B: serial wall-clock phase (machine otherwise idle). ---
$phaseBStart = Get-Date
Invoke-WalltimePhase
$script:PhaseTimings.Add([pscustomobject]@{ Phase = 'B: serial walltime'; TotalSeconds = ((Get-Date) - $phaseBStart).TotalSeconds })

# Cheap, and meaningful only once both sides have actually been built this
# run - PLAN.md D7 / vault PHYS-009 Step 1 rule 4 ("runs in the fast CI too").
$hashStart = Get-Date
Invoke-CrossPlatformHashCheck
$script:PhaseTimings.Add([pscustomobject]@{ Phase = 'hash compare'; TotalSeconds = ((Get-Date) - $hashStart).TotalSeconds })
}  # end of the default (all-leg) flow

$wallSeconds = ((Get-Date) - $script:WallStart).TotalSeconds

# ===========================================================================
# Summary
# ===========================================================================
Write-Host ""
Write-Host "=== CI summary ($script:RunMode) ===" -ForegroundColor Cyan
$script:Results | ForEach-Object {
    $status = if ($_.Pass) { 'PASS' } else { 'FAIL' }
    $color = if ($_.Pass) { 'Green' } else { 'Red' }
    $secs = if ($_.Seconds -gt 0) { "{0,7:N1}s" -f $_.Seconds } else { '       ' }
    Write-Host ("  [{0}] {1,-20} {2,-10} {3}  {4}" -f $status, $_.Leg, $_.Stage, $secs, $_.Detail) -ForegroundColor $color
}

Write-Host ""
Write-Host "=== Per-leg wall clock ===" -ForegroundColor Cyan
$script:LegTimings | ForEach-Object {
    Write-Host ("  {0,-20} {1,7:N1}s" -f $_.Leg, $_.TotalSeconds)
}
Write-Host ("  {0,-20} {1,7:N1}s" -f 'TOTAL', $wallSeconds) -ForegroundColor Cyan

Write-Host ""
Write-Host "=== Phase wall clock ===" -ForegroundColor Cyan
$script:PhaseTimings | ForEach-Object {
    Write-Host ("  {0,-26} {1,7:N1}s" -f $_.Phase, $_.TotalSeconds)
}

$failCount = ($script:Results | Where-Object { -not $_.Pass }).Count
Write-Host ""
if ($failCount -eq 0) {
    Write-Host "All $($script:Results.Count) steps passed. Wall clock: $([math]::Round($wallSeconds, 1))s" -ForegroundColor Green
    exit 0
} else {
    Write-Host "$failCount step(s) FAILED out of $($script:Results.Count). Wall clock: $([math]::Round($wallSeconds, 1))s" -ForegroundColor Red
    exit 1
}
