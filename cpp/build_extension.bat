@echo off
setlocal
set "NAV_PYTHON=%~1"
if not defined NAV_PYTHON set "NAV_PYTHON=python"
set "NAV_VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%NAV_VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "NAV_VS=%%i"
if not defined NAV_VS exit /b 1
call "%NAV_VS%\VC\Auxiliary\Build\vcvars64.bat"
if errorlevel 1 exit /b 1
cd /d "%~dp0"
"%NAV_PYTHON%" setup.py build_ext --inplace
if errorlevel 1 exit /b 1
copy /Y "_fast*.pyd" "..\nav\"
exit /b %errorlevel%
