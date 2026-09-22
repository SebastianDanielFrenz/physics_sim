#Requires -Version 5.1
<#
.SYNOPSIS
    PLAN.md P2 headless smoke test: builds the ps_godot_adapter GDExtension
    and its two plain-C++ tests, runs them, then runs the demo Godot project
    headless and fails if Godot prints any error.

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
#>
[CmdletBinding()]
param(
    [int]$QuitAfterFrames = 300,
    [switch]$SkipBuild
)

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

# ---------------------------------------------------------------------------
# VS 2022 Build Tools developer environment (same approach as tools/ci.ps1's
# Enter-VsDevShell - duplicated here rather than dot-sourcing that file,
# since ci.ps1 is the other agent's lane right now).
# ---------------------------------------------------------------------------
function Enter-VsDevShell {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found; VS 2022 Build Tools not installed" }
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

function Find-GodotConsoleExe {
    $candidates = Get-ChildItem -Path "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter "Godot_v*_console.exe" -ErrorAction SilentlyContinue
    if (-not $candidates -or $candidates.Count -eq 0) {
        throw "Godot console executable not found under WinGet packages (expected Godot_v4.7.2-stable_win64_console.exe)"
    }
    return $candidates[0].FullName
}

Write-Host "=== PLAN.md P2 headless smoke test ===" -ForegroundColor Cyan
Write-Host "repo root: $repoRoot"
Write-Host "build dir: $buildDir"

if (-not $SkipBuild) {
    Enter-VsDevShell

    Write-Host "`n-- configure --" -ForegroundColor Cyan
    & cmake -S $repoRoot -B $buildDir -G Ninja `
        -DCMAKE_CXX_COMPILER=clang-cl `
        -DCMAKE_BUILD_TYPE=RelWithDebInfo `
        -DPS_BUILD_TESTS=OFF `
        -DPS_BUILD_GODOT_ADAPTER=ON
    if ($LASTEXITCODE -ne 0) { Report-Fail "cmake configure failed (exit $LASTEXITCODE)"; }

    if ($script:Failures.Count -eq 0) {
        Write-Host "`n-- build --" -ForegroundColor Cyan
        & cmake --build $buildDir --target ps_godot_adapter ps_godot_test_input_map ps_godot_test_origin_rebase
        if ($LASTEXITCODE -ne 0) { Report-Fail "cmake build failed (exit $LASTEXITCODE)" }
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
    foreach ($testName in @('ps_godot_test_input_map', 'ps_godot_test_origin_rebase')) {
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
    & $godotExe --headless --path $demoDir --quit-after $QuitAfterFrames 1>$stdoutFile 2>$stderrFile
    $godotExit = $LASTEXITCODE
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

Write-Host "`n=== summary ===" -ForegroundColor Cyan
if ($script:Failures.Count -eq 0) {
    Write-Host "PASS: headless smoke test" -ForegroundColor Green
    exit 0
} else {
    Write-Host "FAIL: $($script:Failures.Count) failure(s):" -ForegroundColor Red
    $script:Failures | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 1
}
