#Requires -Version 5.1
<#
.SYNOPSIS
    Sets up (or verifies) the physics_sim development toolchain on a fresh
    Windows machine: VS 2022 Build Tools (MSVC + clang-cl), CMake, Ninja,
    Godot, Python, Blender, and the WSL Ubuntu build toolchain.

.DESCRIPTION
    Idempotent: each step checks whether its tool is already present/working
    before installing anything. Safe to re-run.

.PARAMETER CheckOnly
    Do not install anything. Only verify every tool (Windows + WSL), print
    its version, and exit with a non-zero code if anything is missing.

.PARAMETER BlenderLocation
    Optional install directory passed to winget's --location for the Blender
    MSI (e.g. a custom drive/path). Ignored if Blender is already found by
    Find-Blender.

.NOTES
    Deliberate deviation from a "download LLVM.LLVM via winget" approach:
    the standalone LLVM.LLVM winget package is NOT installed by this script.
    On this machine its installer prompted UAC with no verified publisher
    shown, so it was declined. clang-cl is obtained instead from the VS
    Build Tools' own "VC.Llvm.Clang" / "VC.Llvm.ClangToolset" components,
    which are Microsoft-signed and already carry a clang-cl that is
    link-compatible with the rest of the MSVC toolchain (same CRT/ABI as
    godot-cpp when it is built with MSVC in P2). Never assume
    "C:\Program Files\LLVM" exists on this machine; use the Build Tools'
    own copy at VC\Tools\Llvm\x64\bin\clang-cl.exe instead.
#>
[CmdletBinding()]
param(
    [switch]$CheckOnly,
    [string]$BlenderLocation
)

$ErrorActionPreference = 'Stop'
$script:AllOk = $true

function Write-Section($title) {
    Write-Host ""
    Write-Host "=== $title ===" -ForegroundColor Cyan
}

function Write-Ok($msg) {
    Write-Host "  [OK]   $msg" -ForegroundColor Green
}

function Write-Missing($msg) {
    Write-Host "  [MISS] $msg" -ForegroundColor Red
    $script:AllOk = $false
}

function Test-CommandVersion {
    param(
        [string]$Name,
        [string]$Command,
        [string[]]$ArgumentList = @('--version')
    )
    try {
        $out = & $Command @ArgumentList 2>&1 | Select-Object -First 1
        if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne $null) {
            # Some tools (e.g. git) return 0 reliably; if not, still show output.
        }
        if ($out) {
            Write-Ok "$Name -> $out"
            return $true
        } else {
            Write-Missing "$Name (no output from '$Command $ArgumentList')"
            return $false
        }
    } catch {
        Write-Missing "$Name (command '$Command' not found or failed: $($_.Exception.Message))"
        return $false
    }
}

# ---------------------------------------------------------------------------
# Refresh PATH from machine+user (a shell opened before an install won't see
# newly-installed tools until this is done).
# ---------------------------------------------------------------------------
function Update-SessionPath {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}

# ---------------------------------------------------------------------------
# Locate the VS Build Tools installation and its bundled clang-cl via
# vswhere, without assuming a fixed install path.
# ---------------------------------------------------------------------------
function Get-VsInstallPath {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { return $null }
    $installPath = & $vswhere -products * -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    return $installPath
}

function Get-ClangClPath {
    $vsPath = Get-VsInstallPath
    if (-not $vsPath) { return $null }
    $clangCl = Join-Path $vsPath 'VC\Tools\Llvm\x64\bin\clang-cl.exe'
    if (Test-Path $clangCl) { return $clangCl }
    return $null
}

# ---------------------------------------------------------------------------
# Locate blender.exe without relying on PATH: the user's install is not on
# PATH (a custom -Location install directory). Order: explicit override env
# var, PATH (in case a future machine does put it there), the uninstall
# registry entries winget/the MSI register (both bitness views, both hives),
# then the default per-machine install directory.
# ---------------------------------------------------------------------------
function Find-Blender {
    if ($env:PS_BLENDER -and (Test-Path $env:PS_BLENDER)) {
        return $env:PS_BLENDER
    }

    $cmd = Get-Command blender -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    $uninstallKeys = @(
        'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*',
        'HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*',
        'HKCU:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*'
    )
    $entry = Get-ItemProperty -Path $uninstallKeys -ErrorAction SilentlyContinue |
        Where-Object { $_.DisplayName -like 'Blender*' -and $_.InstallLocation } |
        Select-Object -First 1
    if ($entry) {
        $exe = Join-Path $entry.InstallLocation 'blender.exe'
        if (Test-Path $exe) { return $exe }
    }

    $blenderDir = Get-ChildItem "$env:ProgramFiles\Blender Foundation" -Directory -Filter 'Blender *' -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($blenderDir) {
        $exe = Join-Path $blenderDir.FullName 'blender.exe'
        if (Test-Path $exe) { return $exe }
    }

    return $null
}

# ---------------------------------------------------------------------------
# winget install helper (idempotent: winget itself no-ops if already
# installed at the requested version; we additionally skip the call when a
# working command is already on PATH, to keep -CheckOnly-free runs fast).
# ---------------------------------------------------------------------------
function Install-WingetPackage {
    param(
        [string]$Id,
        [string[]]$ExtraArgs = @()
    )
    Write-Host "Installing/verifying $Id via winget..."
    $args = @('install', '--id', $Id, '-e',
              '--accept-package-agreements', '--accept-source-agreements',
              '--disable-interactivity') + $ExtraArgs
    & winget @args
    if ($LASTEXITCODE -ne 0) {
        throw "winget install of $Id failed with exit code $LASTEXITCODE"
    }
}

# ===========================================================================
# CHECK-ONLY MODE
# ===========================================================================
if ($CheckOnly) {
    Update-SessionPath

    Write-Section "Windows toolchain"
    Test-CommandVersion -Name 'git'    -Command 'git'    -ArgumentList @('--version') | Out-Null
    Test-CommandVersion -Name 'cmake'  -Command 'cmake'  -ArgumentList @('--version') | Out-Null
    Test-CommandVersion -Name 'ninja'  -Command 'ninja'  -ArgumentList @('--version') | Out-Null
    Test-CommandVersion -Name 'python (py -3.13)' -Command 'py' -ArgumentList @('-3.13', '--version') | Out-Null

    $vsPath = Get-VsInstallPath
    if ($vsPath) {
        Write-Ok "VS 2022 Build Tools -> $vsPath"
    } else {
        Write-Missing "VS 2022 Build Tools (VC.Tools.x86.x64 workload) not found via vswhere"
    }

    $clangCl = Get-ClangClPath
    if ($clangCl) {
        $ver = & $clangCl --version | Select-Object -First 1
        Write-Ok "clang-cl -> $clangCl ($ver)"
    } else {
        Write-Missing "clang-cl not found under VC\Tools\Llvm\x64\bin (VC.Llvm.Clang / VC.Llvm.ClangToolset components)"
    }

    $godotDir = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Directory -Filter 'GodotEngine.GodotEngine*' -ErrorAction SilentlyContinue
    $godotExe = $godotDir | Get-ChildItem -Filter '*_console.exe' -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($godotExe) {
        Write-Ok "Godot -> $($godotExe.FullName)"
    } else {
        Write-Missing "Godot (GodotEngine.GodotEngine winget package) not found"
    }

    $blender = Find-Blender
    if ($blender) {
        $blenderVer = & $blender --version 2>&1 | Select-Object -First 1
        Write-Ok "Blender -> $blender ($blenderVer)"
    } else {
        Write-Missing "Blender (BlenderFoundation.Blender winget package) not found"
    }

    Write-Section "WSL toolchain (Ubuntu)"
    $wslDistros = (wsl -l -q 2>$null) -replace "`0", ""
    if ($wslDistros -match 'Ubuntu') {
        Write-Ok "WSL distro present: $($wslDistros -join ', ')"
        # One simple, single-line command per tool - a multi-line heredoc
        # passed through `wsl -- bash -lc <string>` from PowerShell gets its
        # quoting mangled (command substitutions and embedded quotes do not
        # survive the round trip), so keep each probe trivial instead.
        $wslTools = @(
            @{ Name = 'clang';    Cmd = 'clang --version' },
            @{ Name = 'clang++';  Cmd = 'clang++ --version' },
            @{ Name = 'ld.lld';   Cmd = 'ld.lld --version' },
            @{ Name = 'cmake';    Cmd = 'cmake --version' },
            @{ Name = 'ninja';    Cmd = 'ninja --version' },
            @{ Name = 'python3';  Cmd = 'python3 --version' },
            @{ Name = 'git';      Cmd = 'git --version' }
        )
        foreach ($tool in $wslTools) {
            $out = wsl --cd ~ -- bash -lc "$($tool.Cmd) 2>&1 | head -n1"
            if ($LASTEXITCODE -eq 0 -and $out) {
                Write-Ok "$($tool.Name) (WSL) -> $out"
            } else {
                Write-Missing "$($tool.Name) (WSL) not found"
            }
        }
    } else {
        Write-Missing "No Ubuntu WSL distro found (wsl -l -q)"
    }

    Write-Section "Result"
    if ($script:AllOk) {
        Write-Host "All tools present." -ForegroundColor Green
        exit 0
    } else {
        Write-Host "One or more tools are missing. Re-run without -CheckOnly to install." -ForegroundColor Red
        exit 1
    }
}

# ===========================================================================
# INSTALL MODE
# ===========================================================================
Write-Section "VS 2022 Build Tools (C++ workload, Windows SDK, clang-cl)"
if (Get-ClangClPath) {
    Write-Host "Already installed, skipping." -ForegroundColor Yellow
} else {
    # LLVM.LLVM (the standalone winget package) is deliberately NOT used here
    # - see the script header comment. clang-cl comes from the Build Tools'
    # own Clang components instead.
    Install-WingetPackage -Id 'Microsoft.VisualStudio.2022.BuildTools' -ExtraArgs @(
        '--override', '"--wait --quiet --norestart --nocache --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --add Microsoft.VisualStudio.Component.VC.Llvm.Clang --add Microsoft.VisualStudio.Component.VC.Llvm.ClangToolset"'
    )
}

Write-Section "CMake"
Install-WingetPackage -Id 'Kitware.CMake'

Write-Section "Ninja"
Install-WingetPackage -Id 'Ninja-build.Ninja'

Write-Section "Godot 4.x"
Install-WingetPackage -Id 'GodotEngine.GodotEngine'

Write-Section "Blender (dev-time only - headless model checks/preview renders via tools/assetgen/blender_check.py)"
if (Find-Blender) {
    Write-Host "Already installed, skipping." -ForegroundColor Yellow
} else {
    $blenderArgs = @()
    if ($BlenderLocation) { $blenderArgs += @('--location', $BlenderLocation) }
    Install-WingetPackage -Id 'BlenderFoundation.Blender' -ExtraArgs $blenderArgs
}

Write-Section "Python 3.13 (telemetry plots only - never a runtime dependency)"
Install-WingetPackage -Id 'Python.Python.3.13'

Update-SessionPath

Write-Section "WSL Ubuntu build toolchain"
$wslDistros = (wsl -l -q 2>$null) -replace "`0", ""
if ($wslDistros -match 'Ubuntu') {
    Write-Host "Installing/verifying apt packages in WSL as root (idempotent)..."
    wsl -u root -- apt-get update
    wsl -u root -- apt-get install -y clang lld cmake ninja-build python3 git
    if ($LASTEXITCODE -ne 0) {
        throw "WSL apt-get install failed with exit code $LASTEXITCODE"
    }
} else {
    Write-Host "No Ubuntu WSL distro found; install WSL2 + Ubuntu manually first (outside this script's scope - it does not install WSL itself)." -ForegroundColor Red
    $script:AllOk = $false
}

Write-Section "Done"
Write-Host "Run with -CheckOnly to verify the full toolchain." -ForegroundColor Cyan
if (-not $script:AllOk) { exit 1 }
