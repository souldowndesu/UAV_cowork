@echo off
call "%~dp0environment\setup_env.cmd" %*
exit /b %errorlevel%
