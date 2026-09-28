@echo off
REM Build the _fast C++ extension (pybind11 + MSVC BuildTools).
REM Prereqs: 1) VS BuildTools with C++ workload  2) pybind11 installed in venv

setlocal

REM venv Python lives at CompetitionEnv\.venv (two levels up from cpp\)
set "PYTHON=%~dp0..\..\.venv\Scripts\python.exe"
if not exist "%PYTHON%" (
    echo [ERROR] venv Python not found: "%PYTHON%"
    exit /b 1
)

set "VCVARS="
if exist "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not defined VCVARS if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not defined VCVARS if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
if not defined VCVARS (
    echo [ERROR] vcvars64.bat not found. Install VS BuildTools with C++ workload.
    exit /b 1
)

echo [1/3] checking pybind11 ...
"%PYTHON%" -c "import pybind11" 2>NUL
if errorlevel 1 (
    echo installing pybind11 ...
    "%PYTHON%" -m pip install pybind11
    if errorlevel 1 (
        echo [ERROR] failed to install pybind11. Check network.
        exit /b 1
    )
)

echo [2/3] loading MSVC environment ...
call "%VCVARS%" >NUL

echo [3/3] building extension ...
cd /d "%~dp0"
"%PYTHON%" setup.py build_ext --inplace
if errorlevel 1 (
    echo [ERROR] build failed. The numpy fallback still works without it.
    exit /b 1
)

copy /Y "_fast*.pyd" "%~dp0..\nav\" >NUL
if errorlevel 1 (
    echo [ERROR] failed to copy _fast.pyd to nav\.
    exit /b 1
)

echo DONE: _fast extension built and copied to nav\.
endlocal
