$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'adapters\godot\godot_common.ps1')
$probeGodot = Find-GodotConsoleExe
& $probeGodot --path (Join-Path $PSScriptRoot 'adapters\godot\demo') --xr-mode off --audio-driver Dummy --rendering-method mobile --script res://scripts/launch_audio_probe.gd
exit $LASTEXITCODE
