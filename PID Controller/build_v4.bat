@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
cd /d "%~dp0"
cl.exe /LD /O2 /W3 /EHsc /MD /std:c++17 /I"C:\Program Files\Lockheed Martin\Prepar3D v4 SDK 4.5.14.34698\inc\SimConnect" SimBridge_BrunnerOnly_V4.cpp /Fe:smk_simbridge_v4.dll /link /LIBPATH:"C:\Program Files\Lockheed Martin\Prepar3D v4 SDK 4.5.14.34698\lib\SimConnect" SimConnect.lib User32.lib Advapi32.lib Ole32.lib Shell32.lib
