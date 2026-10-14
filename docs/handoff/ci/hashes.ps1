$wt = "S:\claude_code\physics_sim\.claude\worktrees\agent-a9b42dc2791ec55d1"
$sp = "C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad"
$exe = "$wt\out\build\release\tools\ps_run\ps_run.exe"
$cache = "$env:TEMP\ps_garage_p3b1_enginecache"
New-Item -ItemType Directory -Force $cache | Out-Null
$list = 'free_fall','pendulum','joint_chain','articulation_chain','multi_articulation','contact_articulation','terrain_tiles','terrain_multi_interest','terrain_prefetch_drive','terrain_prefetch_multi_interest','vehicle_step_steer','vehicle_powertrain','vehicle_tractor_i4','vehicle_fuel_tank','vehicle_sim_engine','vehicle_gen_engine','environment_day','environment_weather','environment_surface','vehicle_thermal'
$out = @()
foreach ($s in $list) {
    $h = @{}
    foreach ($w in 1, 10) {
        $o = & $exe "$wt\data\scenarios\$s.json" "$sp\h_$s.csv" --workers $w --engine-cache $cache 2>&1
        $m = ($o -join "`n") | Select-String -Pattern 'state_hash=0x([0-9a-fA-F]+)'
        $h[$w] = if ($m) { $m.Matches[0].Groups[1].Value } else { "NONE" }
    }
    $out += ("{0,-34} w1={1} w10={2} {3}" -f $s, $h[1], $h[10], $(if ($h[1] -eq $h[10]) { "equal" } else { "DIFFER" }))
}
$out | Out-File "$sp\hashes.txt"
