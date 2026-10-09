@echo off
call "%~dp0experiments\start_simulator.cmd" %*
exit /b %errorlevel%
