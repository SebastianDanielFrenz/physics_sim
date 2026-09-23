@echo off
rem drive.cmd - thin wrapper so "drive" works from cmd.exe (and by
rem double-click) the same way ".\drive.ps1" works from PowerShell.
rem See drive.ps1's own header comment for switches and the extra-args
rem pass-through convention (e.g. "drive.cmd -- --m1-autopilot").
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0drive.ps1" %*
