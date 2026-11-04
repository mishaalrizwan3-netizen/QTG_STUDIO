@echo off
setlocal
cd /d "%~dp0"
title Super Mushshak QTG Flight Test Logger - PySide6 GUI

echo ========================================================
echo   Super Mushshak Automatic Testing Logger - PySide6 GUI
echo   100%% Brunner Controls Exclusive (UDP 15090)
echo   Prepar3D Read-Only Telemetry & Deterministic t=0 Setup
echo ========================================================
echo.

if exist "%~dp0.venv\Scripts\python.exe" (
    "%~dp0.venv\Scripts\python.exe" "%~dp0Super_Mushak_Logger_GUI.py"
    goto DONE
)

where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    echo [INFO] Launching GUI via Python Launcher (py)...
    py "%~dp0Super_Mushak_Logger_GUI.py"
    goto DONE
)

where python >nul 2>nul
if %ERRORLEVEL% equ 0 (
    echo [INFO] Launching GUI via Python...
    python "%~dp0Super_Mushak_Logger_GUI.py"
    goto DONE
)

echo [ERROR] Python 3 was not found in PATH.
echo Please ensure Python with PySide6 is installed.
pause
exit /b 1

:DONE
exit /b %ERRORLEVEL%

