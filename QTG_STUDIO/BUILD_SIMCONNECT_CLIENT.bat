@echo off
setlocal
cd /d "%~dp0"

REM Discover MSVC outside parenthesized blocks so environment expansion works.
if /I "%VSCMD_ARG_TGT_ARCH%"=="x64" goto compiler_ready
for /f "usebackq delims=" %%I in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do call "%%I\VC\Auxiliary\Build\vcvars64.bat"
:compiler_ready
where cl >nul 2>&1
if errorlevel 1 (
    echo.
    echo ERROR: Microsoft C++ Build Tools x64 not found.
    echo Install Visual Studio Build Tools with "Desktop development with C++".
    echo You can still run the 26-tab GUI with RUN_QTG.bat without compiling.
    exit /b 1
)
if not exist "p3d_sdk\SimConnect.h" (
    echo ERROR: Missing p3d_sdk\SimConnect.h
    exit /b 1
)
if not exist "p3d_sdk\SimConnect.lib" (
    echo ERROR: Missing p3d_sdk\SimConnect.lib
    exit /b 1
)
if not exist "bridge\bin" mkdir "bridge\bin"
echo Building x64 SimConnect client library ...
cl /nologo /LD /MD /EHsc /std:c++17 /O2 /W3 /I"p3d_sdk" "bridge\qtg_simconnect.cpp" /link /MACHINE:X64 /LIBPATH:"p3d_sdk" SimConnect.lib Ws2_32.lib Shlwapi.lib User32.lib Advapi32.lib Ole32.lib Shell32.lib /DEF:"bridge\qtg_simconnect.def" /OUT:"bridge\bin\qtg_simconnect.dll"
if errorlevel 1 (
    echo.
    echo ERROR: Build failed. Check compiler output and SDK version.
    exit /b 1
)
echo.
echo SUCCESS: bridge\bin\qtg_simconnect.dll
exit /b 0
