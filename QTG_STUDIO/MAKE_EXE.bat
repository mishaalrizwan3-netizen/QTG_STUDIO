@echo off
setlocal
cd /d "%~dp0"
py -3 tools\build_portable.py
if errorlevel 1 (
 echo Build failed. The previous dist release has been preserved.
 pause
 exit /b 1
)
echo Standalone release: dist\QTG_Studio_FNPTII
