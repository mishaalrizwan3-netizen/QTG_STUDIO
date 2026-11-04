@echo off
setlocal
cd /d "%~dp0"

REM Auto-find the MSVC x64 compiler if available. This does NOT require opening Visual Studio IDE.
if /I not "%VSCMD_ARG_TGT_ARCH%"=="x64" (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    if exist "%VSWHERE%" (
        for /f "usebackq delims=" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
            if exist "%%I\VC\Auxiliary\Build\vcvars64.bat" (
                set "PATH=%SystemRoot%\system32;%SystemRoot%;%SystemRoot%\System32\Wbem;%SystemRoot%\System32\WindowsPowerShell\v1.0\;%SystemRoot%\System32\OpenSSH\"
                call "%%I\VC\Auxiliary\Build\vcvars64.bat" >nul
            )
        )
    )
)
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
echo Building read-only x64 SimConnect bridge ...
cl /nologo /LD /MD /EHsc /std:c++17 /O2 /W3 /I"p3d_sdk" "bridge\qtg_p3d_bridge.cpp" /link /MACHINE:X64 /LIBPATH:"p3d_sdk" SimConnect.lib Ws2_32.lib Shlwapi.lib User32.lib Advapi32.lib Ole32.lib Shell32.lib /OUT:"bridge\bin\qtg_p3d_bridge.dll"
if errorlevel 1 (
    echo.
    echo ERROR: Build failed. Check compiler output and SDK version.
    exit /b 1
)
echo.
echo SUCCESS: bridge\bin\qtg_p3d_bridge.dll
exit /b 0
