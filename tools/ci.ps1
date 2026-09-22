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
}

if (-not $SkipWsl) {
    if (Should-Run 'linux-release') { Invoke-WslLeg -Preset 'linux-release' -LegKind 'release' }
    if (Should-Run 'linux-asan-ubsan') { Invoke-WslLeg -Preset 'linux-asan-ubsan' -LegKind 'sanitizer' }
    if (Should-Run 'linux-tsan') { Invoke-WslLeg -Preset 'linux-tsan' -LegKind 'sanitizer' }
}

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
