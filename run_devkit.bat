@echo off
rem ============================================================
rem  Launch the dev-kit build of Flycast as a virtual KATANA_DA.
rem  - FLYCAST_DEVKIT=1 starts the SCSI-over-TCP server on boot.
rem  - ContentPath points at the KABUTO dev BIOS folder so the
rem    "Boot BIOS" button finds dc_boot.bin + dc_flash.bin.
rem  After the window opens: Main menu -> Boot BIOS.
rem  Then run DAcheck.exe / dctool.exe through iMekugi.
rem ============================================================
set "FLYCAST_DEVKIT=1"
set "FLYCAST_EXE=C:\dev\flycast\build-devkit\flycast.exe"
set "BIOSDIR=C:\dev\firmares"

if not exist "%FLYCAST_EXE%" (
    echo ERROR: %FLYCAST_EXE% not found - build Flycast first.
    pause
    exit /b 1
)

echo Starting Flycast in dev-kit mode (KATANA_DA server on TCP 7032)...
start "" "%FLYCAST_EXE%" -config config:ContentPath=%BIOSDIR%
