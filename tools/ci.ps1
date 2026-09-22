#Requires -Version 5.1
<#
.SYNOPSIS
    physics_sim CI script. Configures, builds and tests every required leg,
    greps for Godot includes leaking into core/, and prints a clear per-leg
    PASS/FAIL summary with per-stage timing. Exits non-zero if anything
    failed.

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
#>
[CmdletBinding()]
param(
    [switch]$SkipWsl,
    [switch]$SkipWindows,
    [string[]]$Only,
    [switch]$Full
)

$ErrorActionPreference = 'Continue'
$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$script:Results = New-Object System.Collections.Generic.List[object]
$script:LegTimings = New-Object System.Collections.Generic.List[object]

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

# CTest label selection for the FAST (default) mode. Empty string == no
# filter (used for -Full and is also passed through unchanged).
function Get-CtestLabelArgs {
    param([string]$LegKind) # 'release' or 'sanitizer'
    if ($Full) { return @() }
    if ($LegKind -eq 'sanitizer') { return @('-LE', 'validation|long') }
    return @('-LE', 'long')
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
# Cross-platform state-hash comparison (PLAN.md D7 / vault PHYS-009 Step 1
# rule 4): run ps_run on a fixed scenario set on both a Windows build and
# the WSL/Linux build and compare state_hash. Cheap (a few seconds total),
# so it runs in fast CI too, whenever both sides were actually built this
# invocation (skipped with a note otherwise, e.g. -SkipWindows/-SkipWsl).
# ---------------------------------------------------------------------------
$script:HashCheckScenarios = @('free_fall.json', 'pendulum.json', 'joint_chain.json', 'articulation_chain.json', 'multi_articulation.json', 'contact_articulation.json')

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

    $mismatches = 0
    foreach ($scenario in $script:HashCheckScenarios) {
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
        Write-Host "  All $($script:HashCheckScenarios.Count) scenario hashes match Windows vs Linux." -ForegroundColor Green
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

if (-not $SkipWindows) {
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
}

if (-not $SkipWsl) {
    if (Should-Run 'linux-release') { Invoke-WslLeg -Preset 'linux-release' -LegKind 'release' }
    if (Should-Run 'linux-asan-ubsan') { Invoke-WslLeg -Preset 'linux-asan-ubsan' -LegKind 'sanitizer' }
    if (Should-Run 'linux-tsan') { Invoke-WslLeg -Preset 'linux-tsan' -LegKind 'sanitizer' }
}

# Cheap, and meaningful only once both sides have actually been built this
# run - PLAN.md D7 / vault PHYS-009 Step 1 rule 4 ("runs in the fast CI too").
Invoke-CrossPlatformHashCheck

$wallSeconds = ((Get-Date) - $script:WallStart).TotalSeconds

# ===========================================================================
# Summary
# ===========================================================================
Write-Host ""
Write-Host "=== CI summary ($(if ($Full) { 'FULL' } else { 'FAST' })) ===" -ForegroundColor Cyan
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

$failCount = ($script:Results | Where-Object { -not $_.Pass }).Count
Write-Host ""
if ($failCount -eq 0) {
    Write-Host "All $($script:Results.Count) steps passed. Wall clock: $([math]::Round($wallSeconds, 1))s" -ForegroundColor Green
    exit 0
} else {
    Write-Host "$failCount step(s) FAILED out of $($script:Results.Count). Wall clock: $([math]::Round($wallSeconds, 1))s" -ForegroundColor Red
    exit 1
}
