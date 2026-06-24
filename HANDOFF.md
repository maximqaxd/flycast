# Flycast "Dev-Kit" mode — KATANA_DA emulation — HANDOFF

Goal: make Flycast emulate a **Sega Katana Set5 dev-kit Debug Adapter (DA)** so the
authentic toolchain works on the cancelled **Half-Life Dreamcast** port:
`dctool` boots the WinCE (DragonOS) kernel, then **VS6 / eVC uploads & debugs
`halflife_dc.exe`** over the DA — no gdb, exactly like the real dev box.

Branch: **`katana-devkit`** (this repo, fork `maximqaxd/flycast`).

---

## Current status (2026-06-25)

**WORKS end-to-end:**
- Flycast boots as a virtual KATANA_DA (SCSI-over-TCP server on **:7032**).
- `dctool` connects, loads the kernel image, GO. The HL-DC **debug kernel boots
  to a live Windows CE shell prompt** (`Windows CE>` in dctool's Command Shell
  pane) and **serves files over PPFS** (loads `\Windows\arial.ttf`, etc.).
- Console / Output pane shows clean kernel debug output.
- **VS6 connects** (exec the exe → it drives the launch): sends FastFileSys
  traffic and the `s CEMGRC /T:vchanc.dll` launch command over VChannel-1.

**WIP — the one remaining wall: host→target command READ.**
The kernel's only host→target reader is `OEMParallelPortGetByte` (ASE op
`0x0a09`), gated by `NoPPFS`. At boot it does a PPFS RPC and, if dctool's DCFS
doesn't answer in time, times out → `CESH receiving timeout. CESH Disabled.` →
`NoPPFS=1` → the kernel stops reading **all** host commands (incl. VS's
`s CEMGRC`). So CEMGRC never launches and the upload can't complete.

**ROOT CAUSE (found) + FIX JUST APPLIED (UNTESTED):**
The DA multiplexes channels by **TYPE**: `0`=debug console, `1`=PPFS/DCFS,
`3`=VChannel transport. I had been merging the kernel's PPFS (type 1) into the
console (type 0), so dctool's DCFS (which reads the type-1 channel) never saw
the boot PPFS request → CESH timed out.
Refactor to per-type queues `g_chTx[]/g_chRx[]` is done. The host side selects
the channel via the **0x28/0x29 `cmddata` AREA byte (== type)** — *not* the SCSI
CDB mode (verified: mode is always 0). The kernel selects via the ASE op low
byte (`0x0b08`→0, `0x0b09`→1, `0x0b0b`→3; read `0x0a08/09/0b`).
Diagnostics confirmed dctool polls area 0, **1** (DCFS!), and 3.

### >>> IMMEDIATE NEXT STEP <<<
Re-test the area-routed per-type build:
1. Boot the kernel with `dctool` (debug kernel, `Release\debug`).
2. **Exec `halflife_dc.exe` from VS6** (it prompts to connect; dctool must have
   booted the DC first — that's expected).
3. In `build-devkit/flycast.log` check:
   - `CESH ... Disabled` should be **GONE**, `ASE READ type N` count **> 0**.
   - Expect the kernel to now read **type 1** (PPFS) *and* **type 3** (the
     `s CEMGRC` VChannel command). If type-3 reads appear and CESH stays alive,
     CEMGRC should launch.
4. Watch `0x28 POLL cdbMode-type=.. area=..` and `0x29 ... -> RX type N` lines to
   confirm the area→type routing matches the kernel's read ops.

If CESH still disables: the DCFS (type 1) response timing may still lose the boot
race (emulation latency) — next lever is the **DA→SH-4 channel-data interrupt**
(RE which IRQ the kernel installs and assert it from `katana_da` on `0x29`).

---

## Architecture / key facts

- iMekugi `wnaspi32.dll` replaces the real ASPI driver with a TCP tunnel to
  Flycast's DA server (port 7032). `dctool.exe`, and the VS/eVC `vchans.dll`
  COM server, both load it. cfg `1:127.0.0.1:7032:<listener>:<lvl>:<MB>`.
  **Keep iMekugi log level at 1** — level 3 filled the disk (986 MB).
- DAPIPE over SCSI `WRITE_BUFFER(0x3B)` / `READ_BUFFER(0x3C)`:
  `0x15` write-mem, `0x16/0x17` reg context, `0x1C` GO, `0x28/0x29` channel
  read/write (the **`cmddata[0]` byte = the channel/type**).
- DA detect: board reg `0xA05F68A0==0x80000000` → Set5; `g_DAPresent` iff
  `*(0xAC004000)==0x000B003B` (we plant this magic).
- ASE BIOS (KABUTO) is ABSENT in Flycast → intercepted at the vector
  `0xAC004000` (phys `0x0C004000`) from `rdv_FailedToFindBlock`.
  Ops: read `0x0a0x`, write `0x0b0x`; r5=len, r6=buf, r7=&count; ret
  `r0=status<<16` (0 ok, 6/7 would-block).
- On-wire framings per channel: console=ASCII; PPFS/DCFS type1=**0xAA5555AA**;
  VChannel type3=**0x89ABCDEF**(data)/`0xACACACAC`(ctrl), `0xEDEDEDED`/`0x00BADBAD`
  magic2, VChannel# at packet byte+5 (1=shell, 13=FastFileSys, 3=heartbeat).
- MMU: CESH/PPFS BIOS calls run on a paged **U0 user stack** in WinCE slots.
  `aseXlat` translates via the UTLB; on a genuine TLB miss `aseFaultIn` raises
  the SH-4 TLB-miss exception so WinCE's handler loads the page and the BIOS
  call re-executes (this fixed the original bad-pcnt runaway).

## Key code
- `core/hw/devkit/katana_da.cpp` — everything: TCP/ASPI server, DAPIPE,
  `handleAseKfunc` (ASE intercept), `aseXlat`/`aseFaultIn` (MMU), per-type
  `g_chTx[]/g_chRx[]`, status-poll availability bitmask.
- `core/hw/sh4/dyna/driver.cpp` — the `rdv_FailedToFindBlock` hook.
- `core/emulator.cpp` — starts the server when `FLYCAST_DEVKIT=1`.
- `core/hw/sh4/modules/serial.cpp` — SCIF `DaSerialPipe` bridge (windbg/KD path).

## Build & run (Windows, VS 2022 / "18")
```
_devkit_configure.bat   # one-time: CMake -> build-devkit/ (Ninja, GDB server on)
_devkit_build.bat       # vcvars64 + ninja  (the vswhere.exe warning is harmless)
run_devkit.bat          # launches with FLYCAST_DEVKIT=1 -> DA server on :7032
```
`build-devkit/flycast.log` is the main diagnostic. The kernel's full console
output is logged there too (`ASE WRITE ... (console) ..: "<text>"`).

## External dependencies to bring to the other PC (NOT in this repo)
- `C:\WCEDreamcast\` — the SDK/toolchain: `tools\dctool.exe`, `tools\vcce\`
  (cemgr/vchans/vchanc + client tools), iMekugi `wnaspi32.*`, the kernel images
  in `Release\debug\0winceos.bin` + `Release\debug\Applications\CEMGRC.EXE`,
  served files. iMekugi `wnaspi32` must be where each loader's PROCESS searches
  (dctool dir; for VS, where the `vchans` COM host searches).
- `C:\dev\halflife_dc\` — game source; built exe at
  `obj\WCESH4Dbg\halflife_dc.exe` → `deploy\halflife_dc.exe` (the `.dsp` per-
  project Intermediate_Dir fix is in there; rebuild order client→halflife→
  halflife_dc).
- Flycast firmware folder (`run_devkit.bat` BIOSDIR, e.g. `C:\dev\firmares`):
  `dc_boot.bin` / `dc_flash.bin`.
- VS6 / eVC with the **Platform Manager "Dreamcast" device** registered
  (HKLM\SOFTWARE\WOW6432Node\Microsoft\Windows CE Tools\Platform Manager;
  transport = `vcce\client\vchanc.dll`, CPU SH4).

## RE setup (Ghidra) — for continuing the protocol work
- Ghidra MCP bridge on TCP 8089, project `wce`, programs **`nknodbg.exe`**
  (the kernel, PE base 0x8c000000, nk.pdb/nk.map applied — typed decompiles)
  and **`dctool.exe`** (host DA server, base 0x01000000).
- On Windows the AF_UNIX socket is unavailable: `rm` the stale
  `C:\tmp\ghidra-mcp-*\*.sock` then `connect_instance("wce")` to use TCP.
- Hot dctool functions: console read `FUN_0100b406` (type0); DCFS/ppRead
  `FUN_01013b60`→`FUN_01017470`→`FUN_01017a60(.., type=1)`; VChannel demux
  `FUN_01015009`; shell OutputThread `FUN_01013ef2` (VChannel 1); FastFileSys
  `FUN_01012968` (VChannel 13); status decode `FUN_01009efe`; SALSA error map
  `FUN_01016ec0` (0x20100 = "command pending completion").
- Hot kernel functions: `OEMParallelPortGetByte@0x8c03d53c` (read, NoPPFS gate,
  CESH timeout), `OEMParallelPortSendByte@0x8c03d678` (write), `SchedInit`/
  `OpenExe`/`SafeOpenExe`/`ropen` (boot PPFS load path).

## Full project log
Detailed iteration-by-iteration notes (every fix, why, and the test result) are
in the persistent memory file:
`C:\Users\<you>\.claude\projects\C--Dev\memory\flycast-devkit-project.md`
(entries up to **#24**). Read it for the complete reasoning trail.
