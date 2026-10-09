@echo off
setlocal
set "NAV_SCENE_EXE=%~1"
if not defined NAV_SCENE_EXE set "NAV_SCENE_EXE=%~dp0..\..\CityEnviron\WindowsNoEditor\CityEnviron.exe"
if not exist "%NAV_SCENE_EXE%" (
  echo Simulator not found. Usage: replay_scene.cmd "C:\path\to\CityEnviron.exe"
  pause
  exit /b 1
)
powershell -NoProfile -Command "if ((Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue) -or (Get-Process -Name ([IO.Path]::GetFileNameWithoutExtension($env:NAV_SCENE_EXE)) -ErrorAction SilentlyContinue)) { exit 1 }; exit 0"
if errorlevel 1 (
  echo The simulator is already running or starting. Close it before starting the replay configuration.
  pause
  exit /b 1
)
start "" "%NAV_SCENE_EXE%" -settings="%~dp0..\environment\settings_replay.json" -windowed -ResX=1280 -ResY=720
