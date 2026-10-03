# Diagnostic demo

This is a separate diagnostic Godot project. Its generated adapter binary is
not versioned. Build the main adapter, then copy its binary before launching:

```powershell
Copy-Item ../demo/bin/libps_godot.dll ./bin/libps_godot.dll -Force
```

Run this command from this directory after creating `bin` if needed. The main
project at `../demo` contains the maintained live engine, cockpit and audio
routing; this diagnostic snapshot is retained for reproducing older cases.