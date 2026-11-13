@echo off
setlocal
cd /d "%~dp0"

echo =======================================================
echo Building SMK Brunner-Only V4.1 Executable
echo =======================================================

set PYTHON=python

if not exist %PYTHON% (
    set PYTHON=python
)

echo.
echo [1/2] Building Single-File Standalone Executable...
%PYTHON% -m PyInstaller --clean --noconfirm --onefile --windowed ^
    --add-data "smk_simbridge_v4.dll;." ^
    --add-data "smk_simbridge.dll;." ^
    --name "SMK_IAS_PID_BRUNNER_ONLY_V4_1_SingleFile" ^
    SMK_IAS_PID_BRUNNER_ONLY_V4_1.py

echo.
echo [2/2] Building Directory Executable...
%PYTHON% -m PyInstaller --noconfirm --onedir --windowed ^
    --add-data "smk_simbridge_v4.dll;." ^
    --add-data "smk_simbridge.dll;." ^
    --name "SMK_IAS_PID_BRUNNER_ONLY_V4_1" ^
    SMK_IAS_PID_BRUNNER_ONLY_V4_1.py

copy /y smk_simbridge*.dll dist\SMK_IAS_PID_BRUNNER_ONLY_V4_1\ >nul 2>&1

echo.
echo =======================================================
echo BUILD FINISHED!
echo Output files available in dist\:
echo   - dist\SMK_IAS_PID_BRUNNER_ONLY_V4_1_SingleFile.exe
echo   - dist\SMK_IAS_PID_BRUNNER_ONLY_V4_1\SMK_IAS_PID_BRUNNER_ONLY_V4_1.exe
echo =======================================================
pause
