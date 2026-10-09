@echo off
setlocal
set "PYTHONUTF8=1"
set "NAV_EXAMPLE_PYTHON=%~dp0.venv\Scripts\python.exe"
if not exist "%NAV_EXAMPLE_PYTHON%" set "NAV_EXAMPLE_PYTHON=%~dp0..\.venv\Scripts\python.exe"
if not exist "%NAV_EXAMPLE_PYTHON%" (
  echo Python environment not found. Run setup_env.cmd first.
  pause
  exit /b 1
)
"%NAV_EXAMPLE_PYTHON%" -u "%~dp0experiments\fly_examples.py" %*
set "NAV_FLY_EXIT=%errorlevel%"
if not "%NAV_FLY_EXIT%"=="0" pause
exit /b %NAV_FLY_EXIT%
