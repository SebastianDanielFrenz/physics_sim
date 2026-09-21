#Requires -Version 5.1
<#
.SYNOPSIS
    physics_sim CI script (Phase 0). Configures, builds and tests every
    required leg, greps for Godot includes leaking into core/, and prints a
    clear per-leg PASS/FAIL summary. Exits non-zero if anything failed.

.PARAMETER SkipWsl
    Skip the three WSL/Linux legs (linux-release, linux-asan-ubsan, linux-tsan).

.PARAMETER SkipWindows
    Skip all Windows legs (clang-cl debug/release/asan, cl msvc-release).

.PARAMETER Only
    Run only the named leg(s) (e.g. -Only debug,asan). Names match the
    preset names used below (see the $legs definitions).
#>
[CmdletBinding()]
param(
    [switch]$SkipWsl,
    [switch]$SkipWindows,
    [string[]]$Only
)

$ErrorActionPreference = 'Continue'
$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$script:Results = New-Object System.Collections.Generic.List[object]

function Add-Result {
    param([string]$Leg, [string]$Stage, [bool]$Pass, [string]$Detail = '')
    $script:Results.Add([pscustomobject]@{
        Leg = $Leg; Stage = $Stage; Pass = $Pass; Detail = $Detail
    })
}

function Should-Run {
    param([string]$Name)
    if (-not $Only -or $Only.Count -eq 0) { return $true }
    return $Only -contains $Name
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
# One Windows leg: cmake --preset / --build / ctest.
# ---------------------------------------------------------------------------
function Invoke-WindowsLeg {
    param([string]$Preset)

    Write-Host ""
    Write-Host "=== Windows leg: $Preset ===" -ForegroundColor Cyan

    & cmake --preset $Preset
    $configOk = ($LASTEXITCODE -eq 0)
    Add-Result -Leg $Preset -Stage 'configure' -Pass $configOk
    if (-not $configOk) { return }

    & cmake --build --preset $Preset
    $buildOk = ($LASTEXITCODE -eq 0)
    Add-Result -Leg $Preset -Stage 'build' -Pass $buildOk
    if (-not $buildOk) { return }

    $ctestOut = & ctest --preset $Preset --output-on-failure 2>&1
    $ctestOk = ($LASTEXITCODE -eq 0)
    $summaryLine = ($ctestOut | Select-String 'tests passed' | Select-Object -Last 1).ToString()
    Add-Result -Leg $Preset -Stage 'ctest' -Pass $ctestOk -Detail $summaryLine
    if (-not $ctestOk) { Write-Host ($ctestOut -join "`n") }
}

# ---------------------------------------------------------------------------
# One WSL/Linux leg: same three steps, run inside WSL, building under the
# Linux filesystem (~/build/physics_sim/<preset>, set via CMakePresets.json),
# never under /mnt/s.
# ---------------------------------------------------------------------------
function Invoke-WslLeg {
    param([string]$Preset)

    Write-Host ""
    Write-Host "=== WSL leg: $Preset ===" -ForegroundColor Cyan

    $cmd = "cd /mnt/s/claude_code/physics_sim && cmake --preset $Preset"
    wsl --cd ~ -- bash -lc $cmd
    $configOk = ($LASTEXITCODE -eq 0)
    Add-Result -Leg $Preset -Stage 'configure' -Pass $configOk
    if (-not $configOk) { return }

    $cmd = "cd /mnt/s/claude_code/physics_sim && cmake --build --preset $Preset"
    wsl --cd ~ -- bash -lc $cmd
    $buildOk = ($LASTEXITCODE -eq 0)
    Add-Result -Leg $Preset -Stage 'build' -Pass $buildOk
    if (-not $buildOk) { return }

    $cmd = "cd /mnt/s/claude_code/physics_sim && ctest --preset $Preset --output-on-failure"
    $ctestOut = wsl --cd ~ -- bash -lc $cmd
    $ctestOk = ($LASTEXITCODE -eq 0)
    $summaryLine = (($ctestOut | Select-String 'tests passed') | Select-Object -Last 1).ToString()
    Add-Result -Leg $Preset -Stage 'ctest' -Pass $ctestOk -Detail $summaryLine
    if (-not $ctestOk) { Write-Host ($ctestOut -join "`n") }
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
if (-not $SkipWindows) {
    Enter-VsDevShell

    foreach ($preset in @('debug', 'release', 'asan')) {
        if (Should-Run $preset) { Invoke-WindowsLeg -Preset $preset }
    }
    if (Should-Run 'msvc-release') { Invoke-WindowsLeg -Preset 'msvc-release' }

    Invoke-GodotIncludeGrep
}

if (-not $SkipWsl) {
    foreach ($preset in @('linux-release', 'linux-asan-ubsan', 'linux-tsan')) {
        if (Should-Run $preset) { Invoke-WslLeg -Preset $preset }
    }
}

# ===========================================================================
# Summary
# ===========================================================================
Write-Host ""
Write-Host "=== CI summary ===" -ForegroundColor Cyan
$script:Results | ForEach-Object {
    $status = if ($_.Pass) { 'PASS' } else { 'FAIL' }
    $color = if ($_.Pass) { 'Green' } else { 'Red' }
    Write-Host ("  [{0}] {1,-20} {2,-10} {3}" -f $status, $_.Leg, $_.Stage, $_.Detail) -ForegroundColor $color
}

$failCount = ($script:Results | Where-Object { -not $_.Pass }).Count
Write-Host ""
if ($failCount -eq 0) {
    Write-Host "All $($script:Results.Count) steps passed." -ForegroundColor Green
    exit 0
} else {
    Write-Host "$failCount step(s) FAILED out of $($script:Results.Count)." -ForegroundColor Red
    exit 1
}
