param([string]$Targets = "ps_unit_tests", [string]$Exe = "", [string]$Filter = "", [string]$Log = "burst.log")
$lock = "S:\claude_code\physics_sim\out\build.lock"
$me = "sonnet garage P3b1"
$wt = "S:\claude_code\physics_sim\.claude\worktrees\agent-a9b42dc2791ec55d1"
$sp = "C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad"
$got = $false
for ($i = 0; $i -lt 200; $i++) {
    try { New-Item -ItemType Directory -Path $lock -ErrorAction Stop | Out-Null; $got = $true; break } catch { Start-Sleep -Seconds 30 }
}
if (-not $got) { "could not get lock" | Out-File "$sp\$Log"; exit 2 }
Set-Content "$lock\owner.txt" $me
try {
    Set-Location $wt
    if ($Targets -ne "") {
        & cmd.exe /c "$sp\build.cmd $Targets" > "$sp\$Log.build" 2>&1
        $code = $LASTEXITCODE
        "build exit $code" | Out-File "$sp\$Log"
        Select-String -Path "$sp\$Log.build" -Pattern "error|FAILED" | Select-Object -First 30 | ForEach-Object { $_.Line.Substring(0, [Math]::Min(300, $_.Line.Length)) } | Out-File "$sp\$Log" -Append
        if ($code -ne 0) { exit $code }
    }
    if ($Exe -ne "") {
        & "$wt\$Exe" $Filter.Split("|") *> "$sp\$Log.run"
        "run exit $LASTEXITCODE" | Out-File "$sp\$Log" -Append
    }
} finally {
    if ((Test-Path "$lock\owner.txt") -and ((Get-Content "$lock\owner.txt") -eq $me)) { Remove-Item -Recurse -Force $lock }
}
