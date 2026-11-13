@echo off
setlocal
cd /d "%~dp0.."
if not exist build\native mkdir build\native
set "QTG_VCVARS="
for %%I in ("C:/Program Files/Microsoft Visual Studio/2022/Community" "C:/Program Files/Microsoft Visual Studio/2022/Preview" "C:/Program Files/Microsoft Visual Studio/18/Community") do if exist "%%~I/VC/Auxiliary/Build/vcvars64.bat" set "QTG_VCVARS=%%~I/VC/Auxiliary/Build/vcvars64.bat"
if not defined QTG_VCVARS exit /b 1
call "%QTG_VCVARS%"
if errorlevel 1 exit /b 1
rem Installed P3D 5.2 requires the compatible v4.5 SimConnect client library.
cl /nologo /MD /LD /EHsc /O2 /std:c++17 /I"C:\Program Files\Lockheed Martin\Prepar3D v4 SDK 4.5.14.34698\inc\SimConnect" native\src\p3d_bridge_v15_full_causal_state.cpp /Fo"build\native\p3d_bridge.obj" /Fe"build\native\p3d_bridge.dll" /link /LIBPATH:"C:\Program Files\Lockheed Martin\Prepar3D v4 SDK 4.5.14.34698\lib\SimConnect" SimConnect.lib user32.lib advapi32.lib ole32.lib shell32.lib ws2_32.lib
if errorlevel 1 exit /b 1
copy /y build\native\p3d_bridge.dll native\bin\p3d_bridge_engine_init_v1517.dll >nul
exit /b %ERRORLEVEL%
