@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
set "CM=C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NJ=C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
"%CM%" -G Ninja -S C:/dev/flycast -B C:/dev/flycast/build-devkit -DCMAKE_BUILD_TYPE=Release -DUSE_DX9=OFF -DENABLE_GDB_SERVER=ON "-DCMAKE_MAKE_PROGRAM=%NJ%" "-DCMAKE_RC_COMPILER=C:/Program Files (x86)/Windows Kits/10/bin/10.0.26100.0/x64/rc.exe"
