$ErrorActionPreference = 'Continue'
$lock = 'S:\claude_code\physics_sim\out\build.lock'
$wt = 'S:\claude_code\physics_sim'
$own = 'coordinator p1b ci'
$log = 'C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\p1b_ci.log'
while ($true) { try { New-Item -ItemType Directory -Path $lock -ErrorAction Stop | Out-Null; break } catch { Start-Sleep -Seconds 30 } }
Set-Content -Path "$lock\owner.txt" -Value $own -Encoding ascii
"lock acquired $(Get-Date -Format HH:mm:ss)" | Out-File $log -Encoding utf8
try {
    Set-Location $wt
    & powershell -ExecutionPolicy Bypass -File tools\ci.ps1 -Affected -Base 96d34da *>&1 | Out-File $log -Append -Encoding utf8
    "ci exit $LASTEXITCODE" | Out-File $log -Append -Encoding utf8
    & 'S:/claude_code/physics_sim/out/build/release/tests/validation/ps_validation_tests.exe' '[n2o]' *>&1 | Out-File $log -Append -Encoding utf8
    "validation long exit $LASTEXITCODE" | Out-File $log -Append -Encoding utf8
    & 'S:/claude_code/physics_sim/out/build/release/tests/unit/ps_unit_tests.exe' '[tyre]' *>&1 | Out-File $log -Append -Encoding utf8
    "unit long exit $LASTEXITCODE" | Out-File $log -Append -Encoding utf8
} finally {
    $o = Get-Content "$lock\owner.txt" -ErrorAction SilentlyContinue
    if ($o -eq $own) { Remove-Item -Recurse -Force $lock }
    "done $(Get-Date -Format HH:mm:ss)" | Out-File $log -Append -Encoding utf8
}
