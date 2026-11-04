@echo off
setlocal
cd /d "%~dp0.."
title Brunner 3-Axis Record ^& Replay (Pitch, Roll ^& Rudder @ 50 Hz)

echo ========================================================
echo   Brunner 3-Axis Record ^& Replay (Pitch, Roll, Rudder)
echo   Direct CLS2Sim UDP Protocol v2 (Port 15090)
echo   Rate: 50 Hz (20 ms) ^| Lead Lookahead: 80 ms
echo ========================================================
echo.

if not exist "%~dp0..\src\record_replay_pitch_roll_rudder.py" (
    echo [ERROR] Could not find record_replay_pitch_roll_rudder.py in src directory.
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
        echo Please ensure Python is installed and in your PATH.
        echo.
        pause
        exit /b 1
    )
)

%PYCMD% "%~dp0..\src\record_replay_pitch_roll_rudder.py"
set EXITCODE=%ERRORLEVEL%

if %EXITCODE% neq 0 (
    echo.
    echo [ERROR] Script exited with code %EXITCODE%
)

echo.
pause
exit /b %EXITCODE%
