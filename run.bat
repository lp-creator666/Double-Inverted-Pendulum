@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts/run.ps1" %*
set "RUN_EXIT_CODE=%ERRORLEVEL%"
if not "%RUN_EXIT_CODE%"=="0" pause
exit /b %RUN_EXIT_CODE%
