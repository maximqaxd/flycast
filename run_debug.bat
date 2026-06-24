@echo off
rem ============================================================
rem  Launch Flycast with the SH-4 GDB remote stub enabled, so a
rem  debugger (Ghidra / gdb) can attach on TCP 3263 and debug a
rem  running WinCE app (e.g. halflife_dc.exe) at the SH-4 level.
rem
rem  Usage:   run_debug.bat  <path-to-image.gdi|.cdi|.chd>
rem           run_debug.bat            (no arg -> boot BIOS only)
rem
rem  The debug_agent is MMU-aware, so breakpoints/reads resolve
rem  paged WinCE user-space virtual addresses, not just the
rem  direct-mapped kernel window.
rem
rem  GDBWaitForConnection is OFF here so the game boots normally
rem  and you attach whenever you like. Set it to "yes" below to
rem  make Flycast halt at startup until the debugger connects.
rem ============================================================
set "FLYCAST_EXE=C:\dev\flycast\build-devkit\flycast.exe"
set "BIOSDIR=C:\dev\firmares"

if not exist "%FLYCAST_EXE%" (
    echo ERROR: %FLYCAST_EXE% not found - build Flycast first.
    pause
    exit /b 1
)

rem  Optional 2nd arg: host COM port to bridge the SH-4 SCIF to (for windbg
rem  serial KD). Use one end of a com0com virtual null-modem pair; point
rem  windbg's serial transport at the other end.
rem      run_debug.bat Half-Life.gdi COM5
set "IMG=%~1"
if not "%~2"=="" (
    set "FLYCAST_SERIAL_COM=%~2"
    echo Bridging SH-4 SCIF to %~2  ^(windbg connects to its com0com peer^)
)

if "%IMG%"=="" (
    echo No image given - booting BIOS with GDB stub on TCP 3263...
    start "" "%FLYCAST_EXE%" ^
        -config config:ContentPath=%BIOSDIR% ^
        -config Debug:GDBEnabled=yes ^
        -config Debug:GDBPort=3263 ^
        -config Debug:GDBWaitForConnection=no
) else (
    echo Booting "%IMG%" with GDB stub on TCP 3263...
    start "" "%FLYCAST_EXE%" ^
        -config Debug:GDBEnabled=yes ^
        -config Debug:GDBPort=3263 ^
        -config Debug:GDBWaitForConnection=no ^
        "%IMG%"
)
