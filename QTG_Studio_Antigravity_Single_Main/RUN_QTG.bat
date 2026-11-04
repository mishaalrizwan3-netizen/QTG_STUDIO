@echo off
cd /d "%~dp0"
if exist "dist\QTG_Studio_FNPTII\QTG_Studio_FNPTII.exe" (
    start "" "dist\QTG_Studio_FNPTII\QTG_Studio_FNPTII.exe"
    exit /b 0
)
if exist "QTG_Studio_FNPTII.exe" (
    start "" "QTG_Studio_FNPTII.exe"
    exit /b 0
)
echo Opening QTG Studio from main.py...
where py >nul 2>&1
if not errorlevel 1 (
  py -3 main.py
) else (
  python main.py
)
if errorlevel 1 (
  echo.
  echo GUI did not start. Install Python 3.10+ with Tkinter.
  pause
)
