#Requires -Version 5.1
<#
.SYNOPSIS
    Shared helpers for adapters/godot/smoke_test.ps1 and the repo-root
    drive.ps1: VS 2022 dev-env entry, Godot executable lookup, and thin
    cmake configure/build wrappers. Dot-sourced, not run directly - has no
    side effects of its own until one of its functions is called.

.DESCRIPTION
    Factored out so drive.ps1 (root of the repo - "build then launch
    Godot on the demo, so the owner can just run it to drive") does not
    duplicate smoke_test.ps1's own copies of this logic. smoke_test.ps1's
    behaviour/output is unchanged by this refactor - it still calls
    Enter-VsDevShell and Find-GodotConsoleExe itself, and still runs its
    own cmake configure/build commands (now via the wrappers below,
    same flags, same $LASTEXITCODE-based error handling as before).
#>

# ---------------------------------------------------------------------------
# VS 2022 Build Tools developer environment (same approach as
# tools/ci.ps1's Enter-VsDevShell - duplicated rather than dot-sourcing
# that file, since ci.ps1 is another agent's lane; see smoke_test.ps1's
# original comment on this, now equally true for drive.ps1).
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
    # vcvars64.bat internally makes its OWN vswhere.exe call inside a
    # narrower cmd.exe PATH and prints a benign "vswhere.exe is not
    # recognized" line to stderr when that inner call fails - cosmetic,
    # vcvars64.bat still resolves the toolchain correctly afterward (see
    # out/handover_p3.md's identical note for tools/ci.ps1's own copy of
    # this same pattern). With a caller's $ErrorActionPreference = 'Stop',
    # that stderr line would otherwise be promoted to a terminating
    # NativeCommandError and abort the whole build - redirect cmd.exe's
    # stderr to $null so only the "set" output (stdout) is parsed below.
    $previousEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $envLines = cmd.exe /c "`"$vcvars`" && set"
    $ErrorActionPreference = $previousEap
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

# Non-console counterpart (drive.ps1's -NoConsole - no extra terminal
# window pops up alongside Godot's own). Same WinGet search, filtered to
# exclude the "_console.exe" variant Find-GodotConsoleExe returns.
function Find-GodotExe {
    $candidates = Get-ChildItem -Path "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter "Godot_v*.exe" -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -notlike '*_console.exe' }
    if (-not $candidates -or $candidates.Count -eq 0) {
        throw "Godot executable not found under WinGet packages (expected Godot_v4.7.2-stable_win64.exe)"
    }
    return $candidates[0].FullName
}

# ---------------------------------------------------------------------------
# cmake configure/build - thin wrappers only. They do NOT throw or use
# Report-Fail/Report-Ok (smoke_test.ps1's own reporting mechanism); each
# returns cmake's own $LASTEXITCODE and lets the caller decide how to
# react (Report-Fail + continue in smoke_test.ps1, throw + stop in
# drive.ps1), same division of responsibility as before this refactor.
# ---------------------------------------------------------------------------
function Invoke-AdapterCMakeConfigure {
    param(
        [Parameter(Mandatory)][string]$RepoRoot,
        [Parameter(Mandatory)][string]$BuildDir
    )
    # Piped through Out-Host (prints immediately, same as before this was
    # a function) rather than left unredirected: cmake's own stdout is a
    # stream of plain strings, and without Out-Host those lines flow into
    # THIS function's own output right alongside the `return` value below
    # - the caller's `$exitCode = Invoke-AdapterCMakeConfigure ...` would
    # then capture cmake's entire printed output concatenated with the
    # real exit code instead of just the integer (found by actually
    # running this: $exitCode became a multi-hundred-character string
    # ending in "... 0", not the int 0, and `-ne 0` was always true).
    & cmake -S $RepoRoot -B $BuildDir -G Ninja `
        -DCMAKE_CXX_COMPILER=clang-cl `
        -DCMAKE_BUILD_TYPE=RelWithDebInfo `
        -DPS_BUILD_TESTS=OFF `
        -DPS_BUILD_GODOT_ADAPTER=ON | Out-Host
    return $LASTEXITCODE
}

function Invoke-AdapterCMakeBuild {
    param(
        [Parameter(Mandatory)][string]$BuildDir,
        [Parameter(Mandatory)][string[]]$Targets
    )
    # See Invoke-AdapterCMakeConfigure's comment on Out-Host above - same
    # reason.
    & cmake --build $BuildDir --target @Targets | Out-Host
    return $LASTEXITCODE
}

# Whether $BuildDir already looks configured (a Ninja build.ninja exists)
# - drive.ps1's "configure only if needed" incremental path. Once
# configured, plain `cmake --build` is sufficient even after a
# CMakeLists.txt edit: Ninja's own RERUN_CMAKE rule re-invokes cmake
# automatically, so skipping the explicit configure step here does not
# risk building against a stale cache.
function Test-AdapterConfigured {
    param([Parameter(Mandatory)][string]$BuildDir)
    return Test-Path (Join-Path $BuildDir 'build.ninja')
}

# True if $Path exists and is currently open/mapped by another process
# (e.g. a running Godot instance with the GDExtension DLL loaded) such
# that it cannot be opened for exclusive read-write access right now -
# drive.ps1's pre-build check ("tell the owner to close Godot" instead of
# failing with a cryptic linker/link.exe error partway through the
# build). A plain file-lock probe rather than process/module enumeration:
# works without elevated permissions and needs no assumption about which
# executable name is holding it.
function Test-DllLocked {
    param([Parameter(Mandatory)][string]$Path)
    if (-not (Test-Path $Path)) { return $false }
    try {
        $stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::ReadWrite, [System.IO.FileShare]::None)
        $stream.Close()
        return $false
    } catch [System.IO.IOException] {
        return $true
    }
}
