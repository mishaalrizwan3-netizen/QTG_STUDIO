@echo off
setlocal

echo =======================================================
echo Building smk_simbridge.dll (V4.1) for Prepar3D (64-bit)
echo =======================================================

call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

set SDK_INC=C:\Program Files\Lockheed Martin\Prepar3D v4 SDK 4.5.14.34698\inc\SimConnect
set SDK_LIB=C:\Program Files\Lockheed Martin\Prepar3D v4 SDK 4.5.14.34698\lib\SimConnect

cd /d "%~dp0"

cl.exe /LD /O2 /W3 /EHsc /MD /std:c++17 /I"%SDK_INC%" SimBridge_BrunnerOnly_V4_1.cpp /Fe:smk_simbridge.dll /link /LIBPATH:"%SDK_LIB%" SimConnect.lib User32.lib Advapi32.lib Ole32.lib Shell32.lib

if %ERRORLEVEL% equ 0 (
    echo.
    echo Copying to smk_simbridge_v4.dll ...
    copy /y smk_simbridge.dll smk_simbridge_v4.dll
    echo.
    echo =======================================================
    echo BUILD SUCCESSFUL: smk_simbridge.dll V4.1 created!
    echo =======================================================
    del /q SimBridge_BrunnerOnly_V4_1.obj smk_simbridge.exp smk_simbridge.lib 2>nul
) else (
    echo.
    echo =======================================================
    echo BUILD FAILED with error code %ERRORLEVEL%
    echo =======================================================
)
