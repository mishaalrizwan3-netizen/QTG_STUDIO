@echo off
setlocal
cd /d "%~dp0.."
".venv\Scripts\python.exe" -m PyInstaller --noconfirm --clean --distpath dist --workpath build\pyinstaller tools\Super_Mushak_Logger.spec
if errorlevel 1 exit /b 1
copy /y README.md dist\Super_Mushak_Logger\README.md >nul
exit /b 0
