@echo off
cd /d "%~dp0"
title QTG Studio FNPT II - Diagnostic Launcher
echo =========================================================
echo Aeromix Multi-Crew FNPT II - Diagnostic Launcher
echo =========================================================
echo Current Folder: %CD%
echo.

:: 1. Terminate any hung background instance from previous run
echo [1/3] Closing any orphaned background processes...
taskkill /F /IM QTG_Studio_FNPTII.exe /T >nul 2>&1

:: 2. Check files
echo [2/3] Checking executable and runtime libraries...
set "EXE_PATH=QTG_Studio_FNPTII.exe"
if not exist "%EXE_PATH%" (
    if exist "dist\QTG_Studio_FNPTII\QTG_Studio_FNPTII.exe" (
        set "EXE_PATH=dist\QTG_Studio_FNPTII\QTG_Studio_FNPTII.exe"
    ) else (
        echo [ERROR] QTG_Studio_FNPTII.exe was not found!
        echo Please ensure you copied the entire QTG_Studio_FNPTII folder.
        pause
        exit /b 1
    )
)

echo [OK] Executable found: %EXE_PATH%

:: 3. Launch with start so window opens immediately
echo.
echo [3/3] Launching QTG Studio...
echo =========================================================
echo.

start "" "%EXE_PATH%"

echo [OK] Application launched! The QTG Studio window is now opening.
echo If it doesn't appear, check if startup_error.log was generated.
echo.
timeout /t 5
