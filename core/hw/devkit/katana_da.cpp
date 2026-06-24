/*
	Flycast "Dev Kit" mode — virtual Sega Katana KATANA_DA debug adapter.

	Phase 1: iMekugi ASPI-over-TCP server + virtual SCSI target. Decodes the
	"SRBT" envelope and the ASPI SRB, answers the host-adapter / device-type /
	INQUIRY queries so the SDK tools detect the adapter, and runs a DAPIPE
	request/response engine on top of SCSI WRITE BUFFER (0x3B) / READ BUFFER
	(0x3C). Memory/CPU side effects are stubbed (Phase 3) but every packet is
	logged so the protocol can be validated against real dctool/DAcheck traffic.

	Protocol spec: c:/dev/hldc-notes/katana_da_protocol.md

	This file is part of Flycast and is distributed under the GPL v2+ (see LICENSE).
 */
#include "katana_da.h"
#include "network/net_platform.h"
#include "log/Log.h"
#include "emulator.h"
#include "hw/sh4/sh4_if.h"
#include "hw/sh4/sh4_mem.h"
#include "hw/sh4/modules/mmu.h"
#include "hw/sh4/modules/modules.h"
#include "hw/sh4/sh4_mmr.h"
#include "hw/hwreg.h"
#include <deque>
#include <mutex>

#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace katana_da
{

// ---------------------------------------------------------------------------
// Tunable knobs (refine empirically from logs).
// ---------------------------------------------------------------------------
// SCSI INQUIRY peripheral device type the virtual DA reports. The SDK's SALSA
// scan classifies the DA from its INQUIRY response; 0x03 (processor device) is
// the working guess until confirmed from a real DAcheck capture.
static constexpr u8  kInquiryDeviceType = 0x03;
static constexpr char kInquiryVendor[8]   = { 'S','E','G','A',' ',' ',' ',' ' };
static constexpr char kInquiryProduct[16] = { 'K','A','T','A','N','A','_','D','A',' ',' ',' ',' ',' ',' ',' ' };
static constexpr char kInquiryRevision[4] = { '1','.','0','0' };

// Reported DA firmware version (DAPIPE 0x04).
static constexpr u32 kFwVersionMajor = 1;
static constexpr u32 kFwVersionMinor = 11;

// ---------------------------------------------------------------------------
// Endian helpers (DAPIPE multi-byte fields are big-endian on the wire).
// ---------------------------------------------------------------------------
static inline u16 rdBE16(const u8 *p) { return (u16)((p[0] << 8) | p[1]); }
static inline u32 rdBE32(const u8 *p) { return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3]; }
static inline void wrBE16(u8 *p, u16 v) { p[0] = v >> 8; p[1] = (u8)v; }
static inline void wrBE32(u8 *p, u32 v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = (u8)v; }

// ---------------------------------------------------------------------------
// Logging helper: hex dump at DEBUG level.
// ---------------------------------------------------------------------------
static void hexDump(const char *tag, const u8 *data, u32 len)
{
	char line[16 * 3 + 8];
	NOTICE_LOG(NETWORK, "[KATANA_DA] %s (%u bytes):", tag, len);
	for (u32 off = 0; off < len; off += 16)
	{
		int n = 0;
		for (u32 i = 0; i < 16 && off + i < len; i++)
			n += snprintf(line + n, sizeof(line) - n, "%02x ", data[off + i]);
		NOTICE_LOG(NETWORK, "[KATANA_DA]   %04x: %s", off, line);
	}
}

// ---------------------------------------------------------------------------
// SH-4 run control. The DA model: suspend/reset halts the CPU, the host loads
// memory + sets the entry PC, then GO resumes. emu.stop()/start() pause/resume
// the emulation thread (same primitives Flycast's GDB stub uses cross-thread).
// ---------------------------------------------------------------------------
static bool g_cpuHalted = false;

static void haltCpu()
{
	if (!g_cpuHalted)
	{
		emu.stop();
		g_cpuHalted = true;
		NOTICE_LOG(NETWORK, "[KATANA_DA] SH-4 halted");
	}
}

static void resumeCpu()
{
	if (g_cpuHalted)
	{
		emu.start();
		g_cpuHalted = false;
		NOTICE_LOG(NETWORK, "[KATANA_DA] SH-4 resumed");
	}
}

// ---------------------------------------------------------------------------
// Serial bridge. The KATANA_DA is fundamentally a SH-4 SCIF-serial <-> host
// SCSI bridge: the kernel's asedbg/EDBG transport (PPFS file server, CESH
// launch, windbg/KD) all ride the serial port. We plug a Pipe into Flycast's
// SCIF: DAPIPE channel-write (0x29) bytes -> SH-4 serial RX; SH-4 serial TX
// bytes -> DAPIPE channel-read (0x28). Byte-level, so SCIF baud/timing is moot.
// ---------------------------------------------------------------------------
static std::mutex g_serialMutex;
static std::deque<u8> g_hostToTarget;   // dctool -> SH-4 serial RX (0x29)
static std::deque<u8> g_targetToHost;   // SH-4 serial TX -> dctool (0x28)

static std::atomic<u32> g_txBytes{ 0 };   // SH-4 serial TX (kernel -> host)
static std::atomic<u32> g_rxBytes{ 0 };   // SH-4 serial RX consumed (host -> kernel)
static std::atomic<u32> g_availCalls{ 0 };

// ---------------------------------------------------------------------------
// ASE-BIOS channel. The CESH/PPFS transport (the VS/eVC debug session) does
// NOT ride the SH-4 SCIF serial — it rides the Set5 "ASE BIOS", reached via
// kernel kfunc vectors 0xFFFFEFxx (ReadASERxBuf->0xFFFFEFE7, WriteASETxBuf->
// 0xFFFFEFFB). On real hardware KABUTO/the DA service those over H-UDI; in
// Flycast that address is unmapped, so we intercept the kfunc (see
// handleAseKfunc) and move bytes through these queues, which the DAPIPE
// channel (0x29 in / 0x28 out) fills/drains.
static std::mutex g_aseMutex;
// Per-DA-channel-TYPE byte queues. The DA multiplexes several logical channels,
// each a distinct TYPE that BOTH sides agree on (RE'd from dctool: console reader
// type 0, ppRead/DCFS type 1, VChannel transport type 3). The kernel selects the
// type via the ASE-BIOS op low byte (0x08+type: 0x0b08 console, 0x0b09 PPFS,
// 0x0b0b VChannel; read 0x0a08/09/0b); dctool selects it via the SCSI
// READ/WRITE BUFFER CDB mode = type<<5. Keeping them separate is essential: e.g.
// the kernel's boot PPFS request (type 1) must reach dctool's DCFS (type 1) or it
// times out -> "CESH Disabled" -> the kernel stops reading ALL host commands.
static constexpr int kNumChan = 8;
static std::deque<u8> g_chTx[kNumChan];   // target -> host, per type (kernel write op 0x0b0(8+t) -> host READ_BUFFER mode t<<5)
static std::deque<u8> g_chRx[kNumChan];   // host -> target, per type (host WRITE_BUFFER mode t<<5 -> kernel read op 0x0a0(8+t))
// type from the kernel ASE op low byte (0x08->0, 0x09->1, 0x0b->3); -1 if not a channel op.
static inline int aseChanType(u32 op) { const int t = (int)(op & 0xFF) - 8; return (t >= 0 && t < kNumChan) ? t : -1; }
static const char *chanName(int t) { return t == 0 ? "console" : t == 1 ? "ppfs/dcfs" : t == 3 ? "vchannel" : "chan"; }
static const u32 kMagicDAPresent = 0x000B003B; // *(ASE vector) the DA-detect checks for

// Compact hexdump (caps at 64 bytes) for tracing the host<->target byte exchange.
static void hexdumpLog(const char *tag, const u8 *p, u32 n)
{
	char line[3 * 64 + 4];
	const u32 show = n > 64 ? 64 : n;
	int o = 0;
	for (u32 i = 0; i < show; i++)
		o += snprintf(line + o, sizeof(line) - o, "%02x ", p[i]);
	NOTICE_LOG(NETWORK, "[KATANA_DA] %s (%u bytes): %s%s", tag, n, line, n > show ? "..." : "");
}

class DaSerialPipe final : public SerialPort::Pipe
{
public:
	void write(u8 data) override {            // SH-4 transmitted a byte
		std::lock_guard<std::mutex> l(g_serialMutex);
		g_targetToHost.push_back(data);
		if (g_txBytes == 0)
			NOTICE_LOG(NETWORK, "[KATANA_DA] *** SH-4 serial TX started, first byte=0x%02x ***", data);
		g_txBytes++;
	}
	int available() override {                 // bytes ready for SH-4 RX
		g_availCalls++;
		std::lock_guard<std::mutex> l(g_serialMutex);
		return (int)g_hostToTarget.size();
	}
	u8 read() override {                       // SH-4 reads a byte
		std::lock_guard<std::mutex> l(g_serialMutex);
		if (g_hostToTarget.empty())
			return 0;
		u8 v = g_hostToTarget.front();
		g_hostToTarget.pop_front();
		g_rxBytes++;
		return v;
	}
};
static DaSerialPipe g_serialPipe;
static bool g_pipeAttached = false;

static void attachSerialPipe()
{
	if (!g_pipeAttached)
	{
		SCIFSerialPort::Instance().setPipe(&g_serialPipe);
		g_pipeAttached = true;
		NOTICE_LOG(NETWORK, "[KATANA_DA] serial pipe attached to SH-4 SCIF");
		// Plant the ASE-BIOS "DA present" magic at both candidate vectors so the
		// kernel's OEMGetPlatformVersion detect sets g_DAPresent=1 (it checks
		// *(vector) == 0x000B003B). 0x0C004000/0x0C008000 are below the kernel
		// load; we also intercept execution there (handleAseKfunc).
		WriteMem32_nommu(0x8C004000, kMagicDAPresent);
		WriteMem32_nommu(0x8C008000, kMagicDAPresent);
		NOTICE_LOG(NETWORK, "[KATANA_DA] planted ASE-BIOS DA-present magic @0x0C004000/0x0C008000");
	}
}

// ---------------------------------------------------------------------------
// ASE-BIOS emulation. The Set5 PPFS/CESH transport is a shared-memory mailbox
// (a parallel-port FIFO). The kernel calls KABUTO's ASE BIOS, a single fn at a
// fixed P2 vector (Set5: 0xAC004000, board-v4: 0xAC008000 -> phys 0x0C004000/
// 0x0C008000), to refill/flush the channel buffers. KABUTO maps that code; here
// it's absent, so the SH-4 dynarec block-miss path lands on it and we emulate
// it. Authoritative contract (nk.pdb on nknodbg.exe, OEMParallelPortGet/SendByte):
//   r4 = op id : 0x0A09 READ, 0x0B09 WRITE   (high byte selects)
//   r5 = max/len bytes,  r6 = buffer ptr (ASERxBuf/ASETxBuf),  r7 = &count out
//   ret r0 = status<<16 : 0 = ok, 7 = read would-block, 6 = write would-block
// The kernel polls (SC_Sleep(1) loop) so no interrupt is needed; r6/r7 are the
// runtime pointers, so this is independent of the kernel's load base.
// Translate a possibly-paged SH-4 VA to physical. P1/P2 (>=0x80000000) and
// MMU-off pass through; P0/U0 go through the UTLB. The CESH/PPFS BIOS calls run
// on the *calling thread's* stack, which for a user process (CEMGRC/the app) is
// a paged U0 address (e.g. 0x0205f7c0) — so &sent and sometimes the data buffer
// must be translated, not treated as physical. Returns false if unmapped.
static bool aseXlat(u32 va, u32 &pa)
{
	if (!mmu_enabled() || va >= 0x80000000) { pa = va; return true; }
	const TLB_Entry *e;
	if (mmu_full_lookup(va, &e, pa) == MmuError::NONE)
		return true;
	// The strict lookup matches against the *current* context's ASID. CESH/PPFS
	// BIOS calls arrive with ASID=0 (kernel context, SV=0) while the count pointer
	// / data buffer lives in a user process's slot (e.g. slot-1 VA 0x0204xxxx)
	// tagged with that process's non-zero ASID — so the strict lookup misses.
	// A WinCE per-process slot address belongs to exactly ONE process, so there is
	// normally a single UTLB entry covering it: resolve via that UNIQUE entry
	// (ignoring ASID). If 2+ entries cover the VA it is genuinely ambiguous — bail
	// rather than risk writing through the wrong process's page (that corrupts).
	const TLB_Entry *match = nullptr;
	u32 matchPa = 0;
	int hits = 0;
	for (const TLB_Entry &t : UTLB)
	{
		if (t.Data.V == 0)
			continue;
		const u32 sz   = t.Data.SZ1 * 2 + t.Data.SZ0;
		const u32 mask = mmu_mask[sz];
		if (((t.Address.VPN << 10) & mask) == (va & mask))
		{
			if (++hits > 1) { match = nullptr; break; }
			match = &t;
			matchPa = ((t.Data.PPN << 10) & mask) | (va & ~mask);
		}
	}
	if (match != nullptr)
	{
		pa = matchPa;
		return true;
	}
	return false;
}

// When an ASE-BIOS operand (count pointer or data buffer) lives in a user
// process's space but is not currently in the hardware UTLB, the real KABUTO BIOS
// would touch it, take a TLB miss, and WinCE's miss handler would load the page.
// Our C++ intercept does direct physical access and never triggers that, so the
// access fails permanently (e.g. the count write never lands -> the kernel's CESH
// flush loop spins until timeout -> PPFS disabled). Mirror the hardware: raise the
// SH-4 TLB-miss exception for the unmapped operand and leave pc at the BIOS vector
// so that, after WinCE's handler LDTLBs the page and RTEs, the BIOS call
// re-executes with the page resident. Returns true if an exception was raised (the
// caller must then return immediately so the handler runs). Guarded so a genuinely
// unmappable address can't loop forever.
static bool aseFaultIn(u32 va, bool write, u32 vectorPc)
{
	static u32 s_va = 0;
	static u32 s_tries = 0;
	if (va == 0 || !mmu_enabled() || va >= 0x80000000)
		return false;                       // physical / P1-P2: no translation needed
	u32 pa;
	if (aseXlat(va, pa))
	{
		if (s_va == va) { s_va = 0; s_tries = 0; }   // resolved (likely after a fault+reload)
		return false;
	}
	if (s_va == va) { if (++s_tries > 6) { s_tries = 0; s_va = 0; return false; } }
	else            { s_va = va; s_tries = 1; }
	Sh4cntx.pc = vectorPc;                  // SPC for the handler's RTE = the BIOS vector
	DoMMUException(va, MmuError::TLB_MISS, write ? MMU_TT_DWRITE : MMU_TT_DREAD);
	return true;
}

static void dumpMmuTlb();   // fwd decl (defined below) for the bad-pcnt diagnostic

bool handleAseKfunc(u32 pc)
{
	const u32 phys = pc & 0x1FFFFFFF;
	if ((phys != 0x0C004000 && phys != 0x0C008000) || !g_pipeAttached)
		return false;

	const u32 id  = Sh4cntx.r[4];
	const u32 len = Sh4cntx.r[5];
	const u32 buf = Sh4cntx.r[6];
	const u32 pcnt = Sh4cntx.r[7];
	u32 status = 7; // default: read would-block

	// Fault in any operand page that the BIOS will touch but that isn't currently
	// resident, then let the BIOS call re-execute (see aseFaultIn). The count
	// pointer is always written; the data buffer is read on a WRITE (0x0Bxx) and
	// written on a READ (0x0Axx). Do this BEFORE any side effects so the retry is
	// clean.
	const bool isReadOp  = (id & 0xFF00) == 0x0A00;
	const bool isWriteOp = (id & 0xFF00) == 0x0B00;
	if (isReadOp || isWriteOp)
	{
		if (aseFaultIn(pcnt, true, pc))
			return true;
		if (aseFaultIn(buf, isReadOp, pc))
			return true;
	}

	// Real transfers are tiny (read max 0x20; write = TxBufBound, <= the ~64-byte
	// ASETxBuf) and the buffer is in the 16 MB system RAM (phys 0x0C..0x0D). Cap
	// the actual byte move + validate the pointer so a garbage call (wild jump
	// into the vector, or unsynced regs) can't OOM or wild-read. CRITICAL: the
	// WRITE caller (OEMParallelPortSendByte) only resets TxBufBound when we report
	// sent == requested (local_20 == local_1c). So we ALWAYS report *pcnt = len,
	// even when we copied fewer/zero bytes — otherwise its flush loop never
	// completes, TxBufBound runs away, and it spins forever.
	const u32 kMaxAseXfer = 0x2000;
	const u32 cap = (len > kMaxAseXfer) ? kMaxAseXfer : len;
	u32 pcntPa;
	const bool pcntOk = (pcnt != 0) && aseXlat(pcnt, pcntPa);

	// Diagnostic: the runaway begins at the first WRITE whose count pointer (r7)
	// won't translate — we then can't report bytes-sent, the kernel's flush loop
	// keeps a stale count and walks its buffer pointer into garbage. Dump the full
	// register/context state for the first handful so we can see what r7/r15 are.
	if (!pcntOk && (id & 0xFF00) == 0x0B00)
	{
		static std::atomic<u32> s_badPcnt{ 0 };
		const u32 bc = s_badPcnt++;
		if (bc < 32)
			NOTICE_LOG(NETWORK, "[KATANA_DA] BAD-PCNT #%u: id=0x%x len=%u buf=0x%08x pcnt(r7)=0x%08x "
					"r15=0x%08x pr=0x%08x pc=0x%08x mmu=%d asid=%u sv=%u md=%u",
					bc, id, len, buf, pcnt, Sh4cntx.r[15], Sh4cntx.pr, pc, (int)mmu_enabled(),
					(u32)CCN_PTEH.ASID, (u32)CCN_MMUCR.SV, (u32)Sh4cntx.sr.MD);
		if (bc == 0)
			dumpMmuTlb();   // one-shot: is the count-ptr page (VPN of r7) resident? how many entries?
	}

	static std::atomic<u32> s_logCount{ 0 };
	const u32 lc = s_logCount++;
	const bool doLog = (lc < 64) || (lc % 8192 == 0);   // rate-limit (don't fill disk)

	if ((id & 0xFF00) == 0x0A00)        // READ: drain host channel -> buffer
	{
		const int ty = aseChanType(id);
		std::deque<u8> *rxq = (ty >= 0) ? &g_chRx[ty] : nullptr;
		// Diagnostic: log each distinct READ op id once (which channel/type the kernel
		// reads host->target data on — e.g. 0x0a09 PPFS(t1) vs 0x0a0b VChannel(t3)).
		{
			static u32 s_rdSeen = 0;
			const u32 lb = id & 0x1F;
			if (!(s_rdSeen & (1u << lb))) { s_rdSeen |= (1u << lb);
				NOTICE_LOG(NETWORK, "[KATANA_DA] ASE READ op id=0x%x -> type %d (%s) pendingRx=%d",
						id, ty, chanName(ty), rxq ? (int)rxq->size() : -1); }
		}
		u32 n = 0;
		if (rxq != nullptr)
		{
			std::lock_guard<std::mutex> l(g_aseMutex);
			while (n < cap && !rxq->empty())
			{
				u32 pa;
				if (!aseXlat(buf + n, pa))
					break;
				WriteMem8_nommu(pa, rxq->front());
				rxq->pop_front();
				n++;
			}
		}
		if (pcntOk)
			WriteMem32_nommu(pcntPa, n);
		status = (n != 0) ? 0 : 7;       // 7 = would-block -> kernel polls again
		if (n != 0)
			NOTICE_LOG(NETWORK, "[KATANA_DA] ASE READ  type %d (%s) %u byte(s) -> 0x%08x", ty, chanName(ty), n, buf);
	}
	else if ((id & 0xFF00) == 0x0B00)   // WRITE: buffer -> host channel
	{
		// Route to the per-TYPE TX queue (RE'd: 0x0b08 console=type0, 0x0b09
		// PPFS/DCFS=type1, 0x0b0b VChannel=type3). dctool reads each type via its own
		// SCSI READ_BUFFER mode (type<<5), so the three streams MUST stay separate:
		// the PPFS (type1, 0xAA5555AA) must reach dctool's DCFS, not the console.
		const int ty = aseChanType(id);
		std::deque<u8> &txq = g_chTx[ty >= 0 ? ty : 0];
		const bool isVchan = (ty == 3);
		// CONTAINMENT: a corrupted/runaway kernel send loop calls us with garbage
		// len (billions) and wild buf pointers. Legit console/CESH writes are tiny
		// (<=64 KB). For an insane length, copy NOTHING but still echo *pcnt=len so
		// the kernel's flush loop completes and resets TxBufBound (stopping the
		// runaway) — and never let the host queue grow without bound.
		const bool saneLen = (len <= 0x10000);
		const u32 kMaxQueue = 0x80000;                // 512 KB backstop per channel
		u32 n = 0;
		char txt[129];
		u32 txtN = 0;
		char hx[49];
		u32 hxN = 0;
		if (saneLen)
		{
			std::lock_guard<std::mutex> l(g_aseMutex);
			if (txq.size() < kMaxQueue)
				for (; n < cap; n++)
				{
					u32 pa;
					if (!aseXlat(buf + n, pa))
						break;
					const u8 b = ReadMem8_nommu(pa);
					txq.push_back(b);
					if (txtN < sizeof(txt) - 1)
						txt[txtN++] = (b >= 0x20 && b < 0x7f) ? (char)b : (b == '\n' || b == '\r') ? ' ' : '.';
					if (n < 16)
						hxN += snprintf(hx + hxN, sizeof(hx) - hxN, "%02x ", b);
				}
		}
		txt[txtN] = '\0';
		hx[hxN] = '\0';
		// Report the *requested* len as sent so SendByte's flush completes and
		// resets TxBufBound (it loops until reported-sent == requested). Even if
		// we copied fewer (bad page / insane len) we echo len to avoid the runaway.
		if (pcntOk)
			WriteMem32_nommu(pcntPa, len);
		status = 0;
		// Log console (area0) write TEXT so the kernel's full debug output is visible
		// in flycast.log regardless of how/whether dctool drains it. CESH (area3) is
		// binary VChannel framing, so only byte-count it.
		if (ty == 0)
			NOTICE_LOG(NETWORK, "[KATANA_DA] ASE WRITE id=0x%x (console) %u B: \"%s\"", id, len, txt);
		else
			NOTICE_LOG(NETWORK, "[KATANA_DA] ASE WRITE id=0x%x (type%d %s) %u B hex[%s] \"%s\"%s%s",
					id, ty, chanName(ty), len, hx, txt,
					pcntOk ? "" : " [bad pcnt]", saneLen ? "" : " [INSANE]");
	}
	else                                 // unknown op: report ok, move nothing
	{
		if (pcntOk)
			WriteMem32_nommu(pcntPa, 0);
		status = 0;
		if (doLog)
			NOTICE_LOG(NETWORK, "[KATANA_DA] ASE BIOS op id=0x%x len=%u buf=0x%08x", id, len, buf);
	}

	Sh4cntx.r[0] = status << 16;
	Sh4cntx.pc = Sh4cntx.pr;   // return to the BIOS caller
	return true;
}

// ---------------------------------------------------------------------------
// Diagnostic: after GO, periodically sample the SH-4 PC so we can see whether
// the uploaded code is hung at one address, looping in a small range (TLB-miss
// / poll loop), or advancing (booting, possibly slowly under full MMU).
// ---------------------------------------------------------------------------
static std::atomic<bool> g_monitorRun{ false };
static std::atomic<bool> g_launched{ false };
static std::thread g_monitorThread;

// ---------------------------------------------------------------------------
// Empirical channel discovery: WinCE's KITL/debug transport writes VChannel
// packets framed with these magics into its TX buffer in RAM. Sweep the 16 MB
// system RAM for them to locate the channel buffers + read their headers.
// ---------------------------------------------------------------------------
static void scanRamForChannels()
{
	static const struct { u32 magic; const char *name; } kMagics[] = {
		{ 0x89abcdef, "VCHAN-DATA" },
		{ 0xacacacac, "VCHAN-CTRL" },
		{ 0xededeede, "VCHAN-ALT"  }, // 0xEDEDEDED neighbourhood
		{ 0xededeeed, "VCHAN-ALT2" },
	};
	NOTICE_LOG(NETWORK, "[KATANA_DA] === RAM channel scan (0x8c000000..0x8d000000) ===");
	int hits = 0;
	for (u32 addr = 0x8c000000; addr < 0x8d000000 && hits < 64; addr += 4)
	{
		const u32 v = ReadMem32_nommu(addr);
		bool match = false;
		const char *name = nullptr;
		for (const auto &m : kMagics)
			if (v == m.magic) { match = true; name = m.name; break; }
		// also catch the generic 0xED..ED / 0xAC.. patterns
		if (!match && (v == 0xededeed0u || v == 0xbadbad00u))
		{ match = true; name = "VCHAN-MISC"; }
		if (match)
		{
			hits++;
			NOTICE_LOG(NETWORK, "[KATANA_DA] channel magic %s 0x%08x @ 0x%08x : %08x %08x %08x %08x %08x %08x",
					name, v, addr,
					ReadMem32_nommu(addr +  4), ReadMem32_nommu(addr +  8),
					ReadMem32_nommu(addr + 12), ReadMem32_nommu(addr + 16),
					ReadMem32_nommu(addr + 20), ReadMem32_nommu(addr + 24));
		}
	}
	NOTICE_LOG(NETWORK, "[KATANA_DA] === RAM channel scan done: %d hits ===", hits);

	// Dump the descriptor cluster + follow the recurring pointers, to map the
	// channel-table layout (per-channel {magic,state,bufptr,head,tail,size}).
	auto dumpDwords = [](const char *tag, u32 base, u32 count) {
		NOTICE_LOG(NETWORK, "[KATANA_DA] --- dump %s @ 0x%08x ---", tag, base);
		for (u32 i = 0; i < count; i += 8)
			NOTICE_LOG(NETWORK, "[KATANA_DA]   %08x: %08x %08x %08x %08x %08x %08x %08x %08x",
					base + i * 4,
					ReadMem32_nommu(base + (i + 0) * 4), ReadMem32_nommu(base + (i + 1) * 4),
					ReadMem32_nommu(base + (i + 2) * 4), ReadMem32_nommu(base + (i + 3) * 4),
					ReadMem32_nommu(base + (i + 4) * 4), ReadMem32_nommu(base + (i + 5) * 4),
					ReadMem32_nommu(base + (i + 6) * 4), ReadMem32_nommu(base + (i + 7) * 4));
	};
	dumpDwords("descriptor cluster", 0x8c2b7700, 0x100); // 0x8c2b7700..0x8c2b7b00
	dumpDwords("ptr 0x8c021178",     0x8c021178, 0x20);
	dumpDwords("ptr 0x8c0200f0",     0x8c0200f0, 0x20);
	dumpDwords("ptr 0x8c015438",     0x8c015438, 0x20);
}

// Dump the loaded WinCE kernel RAM to a file so it can be loaded into Ghidra at
// its real base (0x8c010000, contiguous) to RE the KITL channel code.
static void dumpKernelRam(const char *path = "C:/dev/hldc-notes/nk_ram.bin")
{
	const u32 base = 0x8c010000;
	const u32 size = 0x00330000; // ~3.3 MB, covers NK image + ASE vectors/buffers (0x8c03c..0x8c05b)
	u8 *p = GetMemPtr(base, size);
	if (p == nullptr)
	{
		ERROR_LOG(NETWORK, "[KATANA_DA] dumpKernelRam: GetMemPtr failed");
		return;
	}
	FILE *f = fopen(path, "wb");
	if (f == nullptr)
	{
		ERROR_LOG(NETWORK, "[KATANA_DA] dumpKernelRam: fopen failed (%s)", path);
		return;
	}
	fwrite(p, 1, size, f);
	fclose(f);
	NOTICE_LOG(NETWORK, "[KATANA_DA] dumped kernel RAM 0x%08x..0x%08x -> %s (%u bytes)",
			base, base + size, path, size);
}

// Dump the live SH-4 UTLB (WinCE runs kernel data in translated P0 space) so we
// can map the low-virtual KITL globals/structs to physical/P1 addresses present
// in nk_ram.bin. Also translates the specific KITL addresses of interest.
static void dumpMmuTlb()
{
	static const u32 szbytes[4] = { 1024, 4096, 65536, 1024 * 1024 };
	NOTICE_LOG(NETWORK, "[KATANA_DA] === UTLB dump (mmuOn=%d) ===", (int)mmuOn);
	for (int i = 0; i < 64; i++)
	{
		if (UTLB[i].Data.V == 0)
			continue;
		const u32 vpn = UTLB[i].Address.VPN << 10;
		const u32 ppn = UTLB[i].Data.PPN << 10;
		const u32 sz  = szbytes[(UTLB[i].Data.SZ1 << 1) | UTLB[i].Data.SZ0];
		NOTICE_LOG(NETWORK, "[KATANA_DA] UTLB[%2d] V=0x%08x..0x%08x -> P=0x%08x sz=%-7u asid=%d",
				i, vpn, vpn + sz, ppn, sz, UTLB[i].Address.ASID);
	}
	const u32 wanted[] = { 0x0204a6c4, 0x0204a6b0, 0x0204a000, 0x000215d0, 0x00021178 };
	for (u32 va : wanted)
	{
		bool found = false;
		for (int i = 0; i < 64; i++)
		{
			if (UTLB[i].Data.V == 0)
				continue;
			const u32 vpn = UTLB[i].Address.VPN << 10;
			const u32 sz  = szbytes[(UTLB[i].Data.SZ1 << 1) | UTLB[i].Data.SZ0];
			if (va >= vpn && va < vpn + sz)
			{
				const u32 pa = (UTLB[i].Data.PPN << 10) + (va - vpn);
				const u32 p1 = 0x80000000u | pa;
				NOTICE_LOG(NETWORK, "[KATANA_DA] xlat V=0x%08x -> phys=0x%08x P1=0x%08x  *=0x%08x",
						va, pa, p1, ReadMem32_nommu(p1));
				found = true;
				break;
			}
		}
		if (!found)
			NOTICE_LOG(NETWORK, "[KATANA_DA] xlat V=0x%08x -> (no UTLB entry)", va);
	}
}

static void monitorLoop()
{
	u32 lastPc = 0;
	u32 lastPend = 0;
	int sample = 0;
	bool scanned = false;
	while (g_monitorRun)
	{
		for (int i = 0; i < 50 && g_monitorRun; i++)
			std::this_thread::sleep_for(std::chrono::milliseconds(15)); // ~750ms
		if (!g_launched || g_cpuHalted)
			continue;
		const u32 pc  = Sh4cntx.pc;
		const u32 pr  = Sh4cntx.pr;
		const u32 r15 = Sh4cntx.r[15];
		u32 tx, rx, av, pend;
		{
			std::lock_guard<std::mutex> l(g_serialMutex);
			tx = g_txBytes; rx = g_rxBytes; av = g_availCalls;
			pend = (u32)g_targetToHost.size();
		}
		NOTICE_LOG(NETWORK, "[KATANA_DA] SH4 sample #%d: pc=0x%08x pr=0x%08x r15=0x%08x%s"
				" | serial TX=%u RX=%u availPolls=%u txPending=%u",
				sample, pc, pr, r15, pc == lastPc ? "  (PC UNCHANGED)" : "",
				tx, rx, av, pend);
		// Dump the kernel's serial output (its KD/KITL announce) once it produces any.
		if (pend > 0 && pend != lastPend)
		{
			std::lock_guard<std::mutex> l(g_serialMutex);
			std::string hex;
			char b[4];
			int n = (int)g_targetToHost.size();
			if (n > 64) n = 64;
			for (int i = 0; i < n; i++) { snprintf(b, sizeof b, "%02x ", g_targetToHost[i]); hex += b; }
			NOTICE_LOG(NETWORK, "[KATANA_DA] SH4 serial TX bytes (first %d of %u): %s",
					n, pend, hex.c_str());
			lastPend = pend;
		}
		lastPc = pc;
		(void)scanned;
		sample++;
	}
}

// ---------------------------------------------------------------------------
// Per-connection DAPIPE state.
// ---------------------------------------------------------------------------
struct DapipeState
{
	bool  haveResponse = false;
	u8    command = 0;
	u8    seq[2] = { 0, 0 };
	u16   status = 0;       // 0 = OK
	// Last write-memory request parameters (Phase 3 will act on these).
	u32   writeAddr = 0;
	u8    writeElemSize = 0;
	u32   writeCount = 0;   // element count or byte count, command dependent
	u8    readArea = 0;     // DA area/channel of the last 0x28 BULK_READ (cmddata[0])
};

// ---------------------------------------------------------------------------
// DAPIPE: process a request packet (arrived via WRITE BUFFER).
// dataLen = total bytes of the WRITE BUFFER transfer (header + cmd-data + payload).
// ---------------------------------------------------------------------------
static void dapipeProcessRequest(DapipeState &st, const u8 *data, u32 dataLen, u8 cdbMode)
{
	const int hostType = cdbMode >> 5;   // DA channel type from SCSI CDB mode (type<<5)
	if (dataLen < sizeof(DapipeHeader))
	{
		WARN_LOG(NETWORK, "[KATANA_DA] DAPIPE request too short: %u bytes", dataLen);
		st.haveResponse = false;
		return;
	}
	const DapipeHeader *h = (const DapipeHeader *)data;
	st.command = h->command;
	st.seq[0] = h->seq[0];
	st.seq[1] = h->seq[1];
	st.status = 0;
	st.haveResponse = true;

	const u32 count = rdBE32(h->count);
	DEBUG_LOG(NETWORK, "[KATANA_DA] DAPIPE req cmd=0x%02x seq=%u count=%u",
			h->command, rdBE16(h->seq), count);

	switch (h->command)
	{
	case DAPIPE_RESET_NODA:
	case DAPIPE_RESET_DA:
		NOTICE_LOG(NETWORK, "[KATANA_DA] >> RESET target (cmd 0x%02x)", h->command);
		haltCpu();
		break;
	case DAPIPE_SUSPEND:
		NOTICE_LOG(NETWORK, "[KATANA_DA] >> SUSPEND/halt target");
		haltCpu();
		break;
	case DAPIPE_GO:
		NOTICE_LOG(NETWORK, "[KATANA_DA] >> GO from PC 0x%08x", Sh4cntx.pc);
		// The code we injected via WriteMem*_nommu bypassed the dynarec's block
		// invalidation, so stale/garbage blocks would run. Flush the block cache
		// so the recompiler picks up the freshly-uploaded code.
		emu.getSh4Executor()->ResetCache();
		attachSerialPipe();
		g_launched = true;
		resumeCpu();
		break;
	case DAPIPE_WRITE_MEM:
	{
		// cmd-data (10 bytes after the 10-byte header): [0, elemSize, addr(4 BE), elemCount(4 BE)]
		// Each element is byte-reversed on the wire; restore original LE bytes in RAM.
		if (dataLen >= sizeof(DapipeHeader) + 10)
		{
			const u8 *cd = data + sizeof(DapipeHeader);
			const u8 elemSize  = cd[1];
			const u32 addr     = rdBE32(cd + 2);
			const u32 elemCount= rdBE32(cd + 6);
			const u8 *payload  = data + sizeof(DapipeHeader) + 10;
			const u32 avail    = dataLen - (sizeof(DapipeHeader) + 10);
			u32 bytes = elemCount * elemSize;
			if (bytes > avail)
				bytes = avail;
			st.writeAddr = addr;
			st.writeElemSize = elemSize;
			st.writeCount = elemCount;
			NOTICE_LOG(NETWORK, "[KATANA_DA] >> WRITE MEM addr=0x%08x elem=%u count=%u (%u bytes)",
					addr, elemSize, elemCount, bytes);
			haltCpu();
			u32 a = addr;
			for (u32 i = 0; i + elemSize <= bytes; i += elemSize)
			{
				switch (elemSize)
				{
				case 4: WriteMem32_nommu(a, rdBE32(payload + i)); break;
				case 2: WriteMem16_nommu(a, rdBE16(payload + i)); break;
				default:
					for (u8 j = 0; j < elemSize; j++)
						WriteMem8_nommu(a + j, payload[i + j]);
					break;
				}
				a += elemSize;
			}
		}
		break;
	}
	case DAPIPE_BULK_WRITE:
	{
		// Channel WRITE (host -> target). RE of nknodbg (2026-06-24 #8) proved the
		// kernel's ONLY host->target DA reader for the dctool/VS path is the ASE BIOS
		// channel: OEMParallelPortGetByte (ReadASERxBuf + ASEBIOS vector) — called by
		// the PPFS RPC primitives (ropen/rread/...) which the boot loader drives
		// (SchedInit->OpenExe->SafeOpenExe->ropen). The kernel is the PPFS *client*:
		// at boot it issues an RPC request over ASE and blocks reading the response
		// over ASE. dctool runs the PPFS server; we are a transparent DA pipe.
		// cmddata = the 6 bytes after the 10-byte header; data[10] = DA channel/area.
		// Area 3 = the PPFS/CESH FileServer channel -> ASE RX (g_aseRx). Other areas
		// (KD/windbg debug monitor) -> SCIF RX (g_hostToTarget).
		const u32 hdrCmd = sizeof(DapipeHeader) + 6;
		const u32 plen = dataLen > hdrCmd ? dataLen - hdrCmd : 0;
		const u8 *pl = data + hdrCmd;
		const u8 *cmddata = data + sizeof(DapipeHeader);  // 6 bytes
		// Deliver to the RX queue of the channel the host addressed. The selector is
		// the cmddata AREA byte (data[10]) — which equals the type (0=console,
		// 1=PPFS/DCFS, 3=VChannel); the kernel drains it via op 0x0a0(8+type). (The
		// SCSI CDB mode is always 0 here, NOT type<<5 — verified from the 0x28/0x29
		// diagnostics — so route by area, not mode.)
		(void)hostType;
		const int t = (cmddata[0] < kNumChan) ? cmddata[0] : 3;
		{
			std::lock_guard<std::mutex> l(g_aseMutex);
			for (u32 i = 0; i < plen; i++)
				g_chRx[t].push_back(pl[i]);
		}
		st.writeCount = plen;
		NOTICE_LOG(NETWORK, "[KATANA_DA] >> CHANNEL WRITE (0x29) %u bytes -> RX type %d (%s)  cdbMode=0x%02x cmddata=%02x %02x %02x %02x %02x %02x",
				plen, t, chanName(t), cdbMode,
				cmddata[0], cmddata[1], cmddata[2], cmddata[3], cmddata[4], cmddata[5]);
		if (plen)
			hexdumpLog("0x29 host->target", pl, plen);
		// One-shot: when the first VChannel packet (magic 0x89ABCDEF) arrives
		// (the CESH 's CEMGRC' session start), the kernel's ASE-BIOS vtable +
		// buffers are already initialized and the CESH thread is idle-blocked
		// waiting on the ASE wake-interrupt. Snapshot live RAM so we can follow
		// the real _RxBufPtr/_ASEBIOS_VECTOR funcptrs + find the wake IRQ.
		{
			static bool g_aseDumped = false;
			if (!g_aseDumped && plen >= 4
					&& pl[0] == 0xef && pl[1] == 0xcd && pl[2] == 0xab && pl[3] == 0x89)
			{
				g_aseDumped = true;
				NOTICE_LOG(NETWORK, "[KATANA_DA] first VChannel cmd -> snapshotting live kernel RAM to nk_ase.bin");
				dumpKernelRam("C:/dev/hldc-notes/nk_ase.bin");
			}
		}
		break;
	}
	case DAPIPE_BULK_READ:
	{
		// Channel READ request. The actual data is fetched by the following
		// READ_BUFFER, whose CDB mode>>5 is the channel TYPE — so the per-read type
		// is captured there, not here. Stash this command's cmddata[0] for logging.
		const u32 cmddataOff = sizeof(DapipeHeader);
		st.readArea = (dataLen > cmddataOff) ? data[cmddataOff] : 0;
		// Diagnostic: log each distinct (cdbMode-type, cmddata area) the host polls.
		{
			static u32 s_pollSeen = 0;
			const u32 key = ((u32)(hostType & 7) << 5) | (st.readArea & 0x1F);
			if (!(s_pollSeen & (1u << (key & 0x1F))) || hostType > 0) { s_pollSeen |= (1u << (st.readArea & 0x1F));
				NOTICE_LOG(NETWORK, "[KATANA_DA] 0x28 POLL cdbMode-type=%d area=%u", hostType, st.readArea); }
		}
		break;
	}
	case DAPIPE_READ_BLOCK:
		NOTICE_LOG(NETWORK, "[KATANA_DA] >> READ BLOCK / register context");
		haltCpu();
		// Response is zero-filled; dctool patches the entry PC and writes it back via 0x17.
		break;
	case DAPIPE_WRITE_BLOCK:
	{
		// hdr(10) + cmd-data(6) + 140-byte register block; entry PC = LE u32 at block offset 0x40.
		const u32 pcOffset = sizeof(DapipeHeader) + 6 + 0x40; // = 80
		if (dataLen >= pcOffset + 4)
		{
			const u8 *p = data + pcOffset;
			const u32 pc = (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
			haltCpu();
			Sh4cntx.pc = pc;
			NOTICE_LOG(NETWORK, "[KATANA_DA] >> WRITE CONTEXT: set entry PC = 0x%08x", pc);
		}
		break;
	}
	case DAPIPE_INQUIRY:
		NOTICE_LOG(NETWORK, "[KATANA_DA] >> INQUIRY / version (0x0A)");
		break;
	case DAPIPE_GET_FWVER:
		NOTICE_LOG(NETWORK, "[KATANA_DA] >> GET FW VERSION (0x04)");
		break;
	default:
		WARN_LOG(NETWORK, "[KATANA_DA] >> UNKNOWN DAPIPE cmd 0x%02x", h->command);
		break;
	}
}

// ---------------------------------------------------------------------------
// DAPIPE: build a response packet (requested via READ BUFFER, respLen bytes).
// Returns the number of bytes written into out (== respLen).
// ---------------------------------------------------------------------------
static u32 dapipeBuildResponse(DapipeState &st, u8 *out, u32 respLen, u8 cdbMode)
{
	const int respType = cdbMode >> 5;   // DA channel type of THIS read (mode = type<<5)
	memset(out, 0, respLen);
	if (respLen < sizeof(DapipeHeader))
		return respLen;

	// ---- DA CONSOLE STATUS POLL ------------------------------------------
	// RE of dctool.exe (FUN_010181c0 / FUN_01017940 / FUN_01009efe): after GO,
	// dctool polls the DA console via a *bare 10-byte* READ_BUFFER (mode 0,
	// 24000x in the tunnel log). The 10-byte response IS a DapipeHeader whose:
	//   byte+1 (reserved0) = per-channel-type DATA-AVAILABLE bitmask
	//                        (bit0=0x01 = type0 = debug console)
	//   byte+4 (status[0]) = console run-state (0 = RUNNING)
	// dctool only issues a channel data-read when the matching bit is set, so a
	// zero byte+1 means "no console data" and the kernel banner is never read.
	// Set bit0 whenever the target has produced console/debug bytes (g_aseTx).
	// Command responses are always >10 bytes (54/22/152/14 in the log), so a
	// respLen of exactly 10 unambiguously identifies the status poll.
	if (respLen == sizeof(DapipeHeader))
	{
		DapipeHeader *sh = (DapipeHeader *)out;
		// The 10-byte read ALSO doubles as the SALSA command-completion poll:
		// dctool validates the echoed command byte to know its posted command
		// finished. So preserve the normal header echo (command/seq/status,
		// count=0) — overriding command to 0 makes the SALSA layer report
		// "Command pending completion" and hang. Then OR in the availability bit.
		if (st.haveResponse)
		{
			sh->command  = st.command;
			sh->seq[0]   = st.seq[0];
			sh->seq[1]   = st.seq[1];
			wrBE16(sh->status, st.status);  // run-state byte+4 = 0 = RUNNING
			wrBE32(sh->count, 0);
		}
		{
			std::lock_guard<std::mutex> l(g_aseMutex);
			// byte+1 bit T = channel type T has target->host data pending. dctool then
			// issues a READ_BUFFER with mode T<<5 for that type. (0=console, 1=PPFS/DCFS,
			// 3=VChannel — but set every non-empty type so nothing is missed.)
			for (int t = 0; t < kNumChan; t++)
				if (!g_chTx[t].empty())
					sh->reserved0 |= (u8)(1u << t);
		}
		return respLen;
	}

	if (!st.haveResponse)
	{
		// No request processed yet: resp[0]=0 makes dctool poll again (0x20100).
		return respLen;
	}

	DapipeHeader *h = (DapipeHeader *)out;
	h->command = st.command;        // echo command (validated by host)
	h->seq[0]  = st.seq[0];
	h->seq[1]  = st.seq[1];
	wrBE16(h->status, st.status);   // 0 = OK
	const u32 payloadLen = respLen - sizeof(DapipeHeader);
	wrBE32(h->count, payloadLen);

	u8 *payload = out + sizeof(DapipeHeader);

	switch (st.command)
	{
	case DAPIPE_GET_FWVER:
		// [u32 major BE][u32 minor BE][strings...]
		if (payloadLen >= 8)
		{
			wrBE32(payload + 0, kFwVersionMajor);
			wrBE32(payload + 4, kFwVersionMinor);
			// remaining bytes: suffix / build date / build time strings (NUL-terminated)
		}
		break;
	case DAPIPE_INQUIRY:
		// 12-byte id; dctool byte-swaps u32@0 and u16@6. Left zero for now.
		break;
	case DAPIPE_BULK_WRITE:
		// Serial channel WRITE result: 4-byte extra-response = bytes accepted (BE).
		if (payloadLen >= 4)
			wrBE32(payload, st.writeCount);
		break;
	case DAPIPE_BULK_READ:
	{
		// Channel READ (target -> host): [u32 count BE][data]. The channel TYPE is
		// this READ_BUFFER's CDB mode>>5 (console=0, PPFS/DCFS=1, VChannel=3); drain
		// that type's TX queue. Each type stays a clean separate stream so the PPFS
		// (0xAA5555AA) reaches dctool's DCFS and the VChannel (0x89ABCDEF) reaches the
		// demux without one corrupting the other.
		// Channel selector is the AREA from the preceding 0x28 request (st.readArea ==
		// type), not the READ_BUFFER CDB mode (always 0). See BULK_WRITE note.
		(void)respType;
		const u32 maxData = payloadLen >= 4 ? payloadLen - 4 : 0;
		const int t = (st.readArea < kNumChan) ? st.readArea : 0;
		u32 k = 0;
		{
			std::lock_guard<std::mutex> l(g_aseMutex);
			std::deque<u8> &txq = g_chTx[t];
			while (k < maxData && !txq.empty())
			{
				payload[4 + k] = txq.front();
				txq.pop_front();
				k++;
			}
		}
		wrBE32(payload, k); // bytes actually read
		if (k)
		{
			char tag[40];
			snprintf(tag, sizeof tag, "0x28 t%d(%s)->host", t, chanName(t));
			hexdumpLog(tag, payload + 4, k);
		}
		break;
	}
	default:
		// control commands / reads: zero-filled payload for now (Phase 3).
		break;
	}

	return respLen;
}

// ---------------------------------------------------------------------------
// SCSI target: execute one CDB (SC_EXEC_SCSI_CMD).
//   buf points at the SC_EXEC data buffer (length bufLen) inside the packet.
// Returns the SRB status.
// ---------------------------------------------------------------------------
static u8 scsiExec(DapipeState &st, const u8 *cdb, u8 cdbLen, u8 *buf, u32 bufLen, u8 flags)
{
	DEBUG_LOG(NETWORK, "[KATANA_DA] SCSI CDB op=0x%02x len=%u bufLen=%u flags=0x%02x",
			cdb[0], cdbLen, bufLen, flags);

	switch (cdb[0])
	{
	case SCSI_TEST_UNIT_READY:
		return SS_COMP;

	case SCSI_INQUIRY:
	{
		// Standard INQUIRY data.
		memset(buf, 0, bufLen);
		if (bufLen >= 36)
		{
			buf[0] = kInquiryDeviceType;     // peripheral qualifier 0 | device type
			buf[1] = 0x00;                   // not removable
			buf[2] = 0x02;                   // SCSI-2
			buf[3] = 0x02;                   // response data format
			buf[4] = 32;                     // additional length (n-4)
			memcpy(buf + 8,  kInquiryVendor,   8);
			memcpy(buf + 16, kInquiryProduct,  16);
			memcpy(buf + 32, kInquiryRevision, 4);
		}
		return SS_COMP;
	}

	case SCSI_REQUEST_SENSE:
		memset(buf, 0, bufLen);
		if (bufLen >= 1)
			buf[0] = 0x70;                   // valid, no sense
		return SS_COMP;

	case SCSI_WRITE_BUFFER:               // host -> DA : DAPIPE request
		dapipeProcessRequest(st, buf, bufLen, cdb[1]);   // cdb[1] = mode = DA channel type<<5
		return SS_COMP;

	case SCSI_READ_BUFFER:                // DA -> host : DAPIPE response
		dapipeBuildResponse(st, buf, bufLen, cdb[1]);
		return SS_COMP;

	default:
		WARN_LOG(NETWORK, "[KATANA_DA] unhandled SCSI op 0x%02x", cdb[0]);
		return SS_COMP;                    // pretend success; log for analysis
	}
}

// ---------------------------------------------------------------------------
// Dispatch one fully-received TCPSRB packet, editing it in place. Returns the
// number of bytes to send back (== packet size), or 0 on a fatal parse error.
// ---------------------------------------------------------------------------
static u32 dispatchPacket(DapipeState &st, u8 *pkt, u32 pktLen)
{
	if (pktLen < sizeof(TcpSrbHeader))
		return 0;
	TcpSrbHeader *hdr = (TcpSrbHeader *)pkt;
	const bool isWin32 = hdr->pointerSize >= 4;
	u8 *srbBytes = pkt + sizeof(TcpSrbHeader);
	SrbHeader *srb = (SrbHeader *)srbBytes;

	u32 ret = SS_INVALID_CMD;
	switch (srb->Cmd)
	{
	case SC_HA_INQUIRY:
	{
		SrbHAInquiry *q = (SrbHAInquiry *)srbBytes;
		q->HA_Count   = 1;
		q->HA_SCSI_ID = 7;
		memcpy(q->HA_ManagerId, "ASPI for Win32  ", 16);
		memset(q->HA_Identifier, 0, 16);
		snprintf((char *)q->HA_Identifier, 16, "Host Adapter %d", q->HaId);
		memset(q->HA_Unique, 0, 16);
		q->HA_Unique[3] = 8;              // max targets
		q->HA_Sup_Ext = 0;
		q->Status = SS_COMP;
		ret = SS_COMP;
		NOTICE_LOG(NETWORK, "[KATANA_DA] SC_HA_INQUIRY ha=%d", q->HaId);
		break;
	}
	case SC_GET_DEV_TYPE:
	{
		SrbGDevBlock *d = (SrbGDevBlock *)srbBytes;
		// Present the DA on every target/LUN for now (refine once detection
		// behaviour is confirmed from logs).
		d->DeviceType = kInquiryDeviceType;
		d->Status = SS_COMP;
		ret = SS_COMP;
		NOTICE_LOG(NETWORK, "[KATANA_DA] SC_GET_DEV_TYPE (ha=%d tgt=%d lun=%d) -> type 0x%02x",
				d->HaId, d->Target, d->Lun, d->DeviceType);
		break;
	}
	case SC_EXEC_SCSI_CMD:
	{
		SrbExecScsiCmd *e = (SrbExecScsiCmd *)srbBytes;
		// Data buffer follows the 80-byte SRB struct within the packet.
		u8 *dataBuf = srbBytes + sizeof(SrbExecScsiCmd);
		const u32 maxBuf = (pktLen > sizeof(TcpSrbHeader) + sizeof(SrbExecScsiCmd))
				? pktLen - sizeof(TcpSrbHeader) - (u32)sizeof(SrbExecScsiCmd) : 0;
		const u32 bufLen = e->BufLen <= maxBuf ? e->BufLen : maxBuf;
		e->Status   = scsiExec(st, e->CDBByte, e->CDBLen, dataBuf, bufLen, e->Flags);
		e->HaStat   = 0;
		e->TargStat = 0;
		ret = e->Status;
		break;
	}
	case SC_RESET_DEV:
		srb->Status = SS_COMP;
		ret = SS_COMP;
		NOTICE_LOG(NETWORK, "[KATANA_DA] SC_RESET_DEV");
		break;
	case SC_ABORT_SRB:
		srb->Status = SS_COMP;
		ret = SS_COMP;
		break;
	case SC_GETSET_TIMEOUTS:
		srb->Status = SS_COMP;
		ret = SS_COMP;
		break;
	default:
		WARN_LOG(NETWORK, "[KATANA_DA] unhandled SRB cmd 0x%02x (win32=%d)", srb->Cmd, isWin32);
		srb->Status = SS_COMP;
		ret = SS_COMP;
		break;
	}

	hdr->ret = ret;
	return hdr->size <= pktLen ? hdr->size : pktLen;
}

// ---------------------------------------------------------------------------
// Serializes whole-command dispatch across concurrent host clients. The real
// dev kit has one SCSI adapter shared by dctool + cemgr/vchans, arbitrated by
// the host-side "Cross Products SALSA DA Channel Semaphore". iMekugi turns each
// host process into its own TCP connection, so we accept several and serialize
// their SH-4 / serial-bridge access here.
static std::mutex g_dispatchMutex;

// Connection handler: assemble TCPSRB packets and reply.
// ---------------------------------------------------------------------------
static void handleClient(sock_t client)
{
	NOTICE_LOG(NETWORK, "[KATANA_DA] client connected");
	DapipeState st;
	std::vector<u8> rx;
	rx.reserve(128 * 1024);
	u8 chunk[16 * 1024];

	for (;;)
	{
		int n = (int)recv(client, (char *)chunk, sizeof(chunk), 0);
		if (n <= 0)
			break;
		rx.insert(rx.end(), chunk, chunk + n);

		// Process all complete packets currently buffered.
		for (;;)
		{
			if (rx.size() < sizeof(TcpSrbHeader))
				break;
			TcpSrbHeader *hdr = (TcpSrbHeader *)rx.data();
			if (hdr->magic != kTcpSrbMagic)
			{
				ERROR_LOG(NETWORK, "[KATANA_DA] bad TCPSRB magic 0x%08x; dropping connection", hdr->magic);
				rx.clear();
				closesocket(client);
				return;
			}
			if (hdr->size < sizeof(TcpSrbHeader) || hdr->size > 16 * 1024 * 1024)
			{
				ERROR_LOG(NETWORK, "[KATANA_DA] bogus TCPSRB size %u", hdr->size);
				rx.clear();
				closesocket(client);
				return;
			}
			if (rx.size() < hdr->size)
				break;                    // need more data

			const u32 pktLen = hdr->size;
			u32 sendLen;
			{
				std::lock_guard<std::mutex> dl(g_dispatchMutex);
				sendLen = dispatchPacket(st, rx.data(), pktLen);
			}
			if (sendLen == 0)
				sendLen = pktLen;

			// Reply.
			u32 sent = 0;
			while (sent < sendLen)
			{
				int w = (int)send(client, (const char *)rx.data() + sent, sendLen - sent, 0);
				if (w <= 0)
				{
					ERROR_LOG(NETWORK, "[KATANA_DA] send failed");
					closesocket(client);
					return;
				}
				sent += w;
			}
			rx.erase(rx.begin(), rx.begin() + pktLen);
		}
	}
	closesocket(client);
	NOTICE_LOG(NETWORK, "[KATANA_DA] client disconnected");
}

// ---------------------------------------------------------------------------
// Listener thread + lifecycle.
// ---------------------------------------------------------------------------
static std::atomic<bool> running{ false };
static std::thread serverThread;
static sock_t listenSock = INVALID_SOCKET;

static void serverLoop(u16 port)
{
	listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (!VALID(listenSock))
	{
		ERROR_LOG(NETWORK, "[KATANA_DA] socket() failed");
		running = false;
		return;
	}
	int opt = 1;
	setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);
	if (bind(listenSock, (sockaddr *)&addr, sizeof(addr)) != 0)
	{
		ERROR_LOG(NETWORK, "[KATANA_DA] bind() to port %d failed", port);
		closesocket(listenSock);
		listenSock = INVALID_SOCKET;
		running = false;
		return;
	}
	if (listen(listenSock, 4) != 0)
	{
		ERROR_LOG(NETWORK, "[KATANA_DA] listen() failed");
		closesocket(listenSock);
		listenSock = INVALID_SOCKET;
		running = false;
		return;
	}

	NOTICE_LOG(NETWORK, "[KATANA_DA] dev-kit SCSI server listening on TCP port %d", port);
	while (running)
	{
		sock_t client = accept(listenSock, nullptr, nullptr);
		if (!VALID(client))
		{
			if (running)
				continue;
			break;
		}
		set_tcp_nodelay(client);
		// Service each client on its own thread so dctool and cemgr/vchans
		// (a second iMekugi TCP connection from VS) can be active at once.
		std::thread(handleClient, client).detach();
	}
	if (VALID(listenSock))
		closesocket(listenSock);
	listenSock = INVALID_SOCKET;
}

bool start(u16 port)
{
	if (running)
		return true;
#ifdef _WIN32
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
	running = true;
	g_monitorRun = true;
	g_monitorThread = std::thread(monitorLoop);
	serverThread = std::thread(serverLoop, port);
	return true;
}

void stop()
{
	if (!running)
		return;
	running = false;
	if (VALID(listenSock))
	{
		// Break accept() by closing the listening socket.
		closesocket(listenSock);
		listenSock = INVALID_SOCKET;
	}
	if (serverThread.joinable())
		serverThread.join();
	g_monitorRun = false;
	if (g_monitorThread.joinable())
		g_monitorThread.join();
	g_launched = false;
	NOTICE_LOG(NETWORK, "[KATANA_DA] dev-kit SCSI server stopped");
}

bool isRunning()
{
	return running;
}

} // namespace katana_da
