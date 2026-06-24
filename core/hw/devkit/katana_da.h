/*
	Flycast "Dev Kit" mode — virtual Sega Katana KATANA_DA debug adapter.

	Emulates the SCSI target that the Sega WinCE Dreamcast SDK host tools
	(dctool.exe / DAcheck.exe / dumpreg.exe / flash.exe) talk to through
	wnaspi32.dll. Reached over the iMekugi ASPI-over-TCP tunnel:

	    dctool.exe -> SALSA -> wnaspi32 (iMekugi) --TCP("SRBT")--> Flycast

	Protocol spec: c:/dev/hldc-notes/katana_da_protocol.md

	This file is part of Flycast and is distributed under the GPL v2+ (see LICENSE).
 */
#pragma once
#include "types.h"

namespace katana_da
{

// Default iMekugi sync TCP port (scsiserv DEFAULTSYNCPORTNR).
constexpr u16 kDefaultPort = 7032;

// ---- iMekugi TCP envelope -------------------------------------------------
// "SRBT" magic, little-endian on the wire.
constexpr u32 kTcpSrbMagic = 0x54425253; // 'S','R','B','T' as read from a LE u32

#pragma pack(push, 1)
struct TcpSrbHeader        // 16 bytes; data[] follows
{
	u32 magic;             // kTcpSrbMagic
	u32 size;              // total packet length incl. this 16-byte header
	u32 ret;               // SRB dispatcher return (filled by server)
	u32 pointerSize;       // client pointer size (4 = win32 dctool)
};

// ASPI SRB header (common top of every SRB).
struct SrbHeader
{
	u8  Cmd;
	u8  Status;
	u8  HaId;
	u8  Flags;
	u32 HdrRsvd;
};

// SC_EXEC_SCSI_CMD, win32 layout. Pointer slots are explicit u32 so the
// struct is exactly 80 bytes on any host (matches the 32-bit client wire form).
struct SrbExecScsiCmd
{
	u8  Cmd;               // 0x02 SC_EXEC_SCSI_CMD
	u8  Status;
	u8  HaId;
	u8  Flags;             // 0x50 = DIR_OUT|EVENT_NOTIFY, 0x48 = DIR_IN|EVENT_NOTIFY
	u32 HdrRsvd;
	u8  Target;            // SCSI target id
	u8  Lun;
	u16 Rsvd1;
	u32 BufLen;            // data buffer length
	u32 BufPointer;        // 32-bit wire pointer (ignored by us)
	u8  SenseLen;          // sense allocation length (14)
	u8  CDBLen;            // CDB length (10 for the DA)
	u8  HaStat;            // host adapter status (out)
	u8  TargStat;          // target status (out)
	u32 PostProc;          // 32-bit wire pointer (ignored)
	u8  Rsvd2[20];
	u8  CDBByte[16];       // SCSI CDB
	u8  SenseArea[16];     // SENSE_LEN(14)+2
};

// SC_HA_INQUIRY (60 bytes).
struct SrbHAInquiry
{
	u8  Cmd;
	u8  Status;
	u8  HaId;
	u8  Flags;
	u16 ExtReqSig;
	u16 ExtBufLen;
	u8  HA_Count;
	u8  HA_SCSI_ID;
	u8  HA_ManagerId[16];
	u8  HA_Identifier[16];
	u8  HA_Unique[16];
	u16 HA_Sup_Ext;
};

// SC_GET_DEV_TYPE (win32, SRB_GDEVBlock).
struct SrbGDevBlock
{
	u8  Cmd;
	u8  Status;
	u8  HaId;
	u8  Flags;
	u32 HdrRsvd;
	u8  Target;
	u8  Lun;
	u8  DeviceType;        // SCSI peripheral device type (out)
	u8  Rsvd1;
};
#pragma pack(pop)

static_assert(sizeof(TcpSrbHeader) == 16, "TcpSrbHeader must be 16 bytes");
static_assert(sizeof(SrbExecScsiCmd) == 80, "SrbExecScsiCmd must be 80 bytes (win32 wire layout)");
static_assert(sizeof(SrbHAInquiry) == 60, "SrbHAInquiry must be 60 bytes");

// ---- ASPI SRB command codes ----------------------------------------------
enum SrbCmd : u8 {
	SC_HA_INQUIRY      = 0x00,
	SC_GET_DEV_TYPE    = 0x01,
	SC_EXEC_SCSI_CMD   = 0x02,
	SC_ABORT_SRB       = 0x03,
	SC_RESET_DEV       = 0x04,
	SC_GET_DISK_INFO   = 0x06,
	SC_RESCAN_SCSI_BUS = 0x07,
	SC_GETSET_TIMEOUTS = 0x08,
};

// ---- ASPI SRB status codes ------------------------------------------------
enum SrbStatus : u8 {
	SS_PENDING    = 0x00,
	SS_COMP       = 0x01,
	SS_ERR        = 0x04,
	SS_INVALID_CMD= 0x80,
	SS_INVALID_HA = 0x81,
	SS_NO_DEVICE  = 0x82,
};

// ---- SCSI CDB opcodes the DA uses -----------------------------------------
enum ScsiOp : u8 {
	SCSI_TEST_UNIT_READY = 0x00,
	SCSI_REQUEST_SENSE   = 0x03,
	SCSI_INQUIRY         = 0x12,
	SCSI_WRITE_BUFFER    = 0x3B, // host -> DA : carries a DAPIPE request packet
	SCSI_READ_BUFFER     = 0x3C, // DA  -> host: carries a DAPIPE response packet
};

// ---- DAPIPE command codes (descriptor byte +0x04) -------------------------
enum DapipeCmd : u8 {
	DAPIPE_RESET_NODA   = 0x01, // reset target (no DA stub), long timeout
	DAPIPE_GET_FWVER    = 0x04, // get DA firmware version (44 bytes)
	DAPIPE_RESET_DA     = 0x05, // reset target (with DA stub), long timeout
	DAPIPE_SUSPEND      = 0x06, // suspend/halt target
	DAPIPE_INQUIRY      = 0x0A, // version/inquiry (12 bytes)
	DAPIPE_WRITE_MEM    = 0x15, // write memory (element-sized)
	DAPIPE_READ_BLOCK   = 0x16, // read block / register context
	DAPIPE_WRITE_BLOCK  = 0x17, // write block / register context
	DAPIPE_GO           = 0x1C, // resume execution
	DAPIPE_BULK_READ    = 0x28, // bulk read memory
	DAPIPE_BULK_WRITE   = 0x29, // bulk write memory
};

// DAPIPE 10-byte packet header (multi-byte fields BIG-ENDIAN on the wire).
#pragma pack(push, 1)
struct DapipeHeader
{
	u8  command;     // +0
	u8  reserved0;   // +1
	u8  seq[2];      // +2 sequence (BE)
	u8  status[2];   // +4 reserved on request / status word on response (BE)
	u8  count[4];    // +6 byte count (BE)
};
#pragma pack(pop)
static_assert(sizeof(DapipeHeader) == 10, "DapipeHeader must be 10 bytes");

// ---- public API -----------------------------------------------------------
// Start/stop the dev-kit SCSI server. Safe to call repeatedly; idempotent.
bool start(u16 port = kDefaultPort);
void stop();
bool isRunning();

// SH-4 ASE-BIOS kfunc intercept. Called from the dynarec block-miss path
// (rdv_FailedToFindBlock): if pc is a 0xFFFFEFxx ASE vector and the dev-kit
// session is live, service the DA channel read/write, set Sh4cntx.pc to the
// return address, and return true. Otherwise returns false immediately.
bool handleAseKfunc(u32 pc);

} // namespace katana_da
