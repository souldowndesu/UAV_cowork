@echo off
setlocal
cd /d "%~dp0.."
set "PYTHONUTF8=1"
if not exist "environment\runtime\pip" mkdir "environment\runtime\pip"
set "TEMP=%~dp0runtime\pip"
set "TMP=%TEMP%"
if not exist ".venv\Scripts\python.exe" (
  where py >nul 2>nul
  if not errorlevel 1 (
    py -3.10 -m venv ".venv"
  ) else (
    python -m venv ".venv"
  )
  if errorlevel 1 goto :failed
)
".venv\Scripts\python.exe" -c "import sys,struct; assert sys.version_info[:2]==(3,10) and struct.calcsize('P')==8, 'Use CPython 3.10 x64 for this package'"
if errorlevel 1 goto :failed
if exist "environment\wheelhouse\" (
  ".venv\Scripts\python.exe" -m pip install --no-index --find-links environment\wheelhouse -r environment\requirements.txt
) else (
  echo Offline wheels are missing. Install the complete distribution package.
  goto :failed
)
if errorlevel 1 goto :failed
".venv\Scripts\python.exe" -m pip check
if errorlevel 1 goto :failed
".venv\Scripts\python.exe" -c "import airsim,numpy,scipy,cv2; from navigation.backend import native; print('Environment ready. Native kernels:', native is not None)"
if errorlevel 1 goto :failed
echo Ready. Configure AirSim settings, start the simulator, then use fly.cmd or scene_test.py.
exit /b 0
:failed
echo Setup failed. This package requires Windows x64 and CPython 3.10 x64.
echo See README.md for installation and simulator configuration.
pause
exit /b 1
