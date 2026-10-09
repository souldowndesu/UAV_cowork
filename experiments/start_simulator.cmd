@echo off
setlocal
set "NAV_SCENE_EXE=%~1"
if not defined NAV_SCENE_EXE set "NAV_SCENE_EXE=%~dp0..\..\CityEnviron\WindowsNoEditor\CityEnviron.exe"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_simulator.ps1" -Executable "%NAV_SCENE_EXE%" -Settings "%~dp0..\environment\settings_replay.json"
exit /b %errorlevel%
