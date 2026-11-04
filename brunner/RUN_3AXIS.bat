@echo off
setlocal
cd /d "%~dp0"
title Brunner 3-Axis Record ^& Replay (Pitch, Roll ^& Rudder)

echo ========================================================
echo   Brunner 3-Axis Record ^& Replay (Pitch, Roll, Rudder)
echo   Direct CLS2Sim UDP Protocol v2 (Port 15090)
echo ========================================================
echo.

:: 1. Check if Python is installed
where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    echo [INFO] Running via Python launcher (py)...
    py "%~dp0src\record_replay_pitch_roll_rudder.py"
    goto DONE
)

where python >nul 2>nul
if %ERRORLEVEL% equ 0 (
    echo [INFO] Running via Python...
    python "%~dp0src\record_replay_pitch_roll_rudder.py"
    goto DONE
)

:: 2. Fallback to standalone compiled executable (for Sim PC without Python)
if exist "%~dp0dist\standalone\Brunner_Record_Replay.exe" (
    echo [INFO] Python not found in PATH.
    echo [INFO] Launching standalone executable (dist\standalone\Brunner_Record_Replay.exe)...
    echo.
    "%~dp0dist\standalone\Brunner_Record_Replay.exe"
    goto DONE
)

echo [ERROR] Neither Python nor dist\standalone\Brunner_Record_Replay.exe was found.
echo.
pause
exit /b 1

:DONE
echo.
pause
exit /b %ERRORLEVEL%

