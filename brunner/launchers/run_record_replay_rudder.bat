@echo off
setlocal
cd /d "%~dp0.."
title Brunner Rudder Record ^& Replay (50 Hz @ Port 15090)

echo ========================================================
echo   Brunner Rudder Record ^& Replay (50 Hz / 20 ms)
echo   Direct CLS2Sim UDP (Port 15090, Axis 0x04)
echo ========================================================
echo.

if not exist "%~dp0..\src\single_axis\record_replay_rudder.py" (
    echo [ERROR] Could not find record_replay_rudder.py in src\single_axis directory.
    echo.
    pause
    exit /b 1
)

where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    set "PYCMD=py"
) else (
    where python >nul 2>nul
    if %ERRORLEVEL% equ 0 (
        set "PYCMD=python"
    ) else (
        echo [ERROR] Neither 'py' nor 'python' was found in PATH.
        echo.
        pause
        exit /b 1
    )
)

%PYCMD% "%~dp0..\src\single_axis\record_replay_rudder.py"
set EXITCODE=%ERRORLEVEL%

if %EXITCODE% neq 0 (
    echo.
    echo [ERROR] Script exited with code %EXITCODE%
)

echo.
pause
exit /b %EXITCODE%
