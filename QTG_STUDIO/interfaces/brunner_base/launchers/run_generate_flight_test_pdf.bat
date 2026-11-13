@echo off
setlocal
cd /d "%~dp0.."
title Super Mushshak Flight Test PDF Generator

echo =====================================================================
echo  Super Mushshak Automatic Flight Test - PDF Report Generator
echo  Brunner CLS-P Rig: Pitch, Roll ^& Rudder Control Loading
echo =====================================================================
echo.

where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    set "PY_CMD=py"
    goto RUN
)
where python >nul 2>nul
if %ERRORLEVEL% equ 0 (
    set "PY_CMD=python"
    goto RUN
)
echo [ERROR] Neither 'py' nor 'python' was found in PATH.
pause
exit /b 1

:RUN
echo Using Python launcher: %PY_CMD%
"%PY_CMD%" "%~dp0..\src\generate_flight_test_pdf.py" %*
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] PDF generation failed with code %ERRORLEVEL%.
    pause
    exit /b %ERRORLEVEL%
)
echo.
echo [DONE] Report generated successfully.
pause
