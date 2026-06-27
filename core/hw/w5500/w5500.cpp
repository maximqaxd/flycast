/*
	w5500.cpp - WIZnet W5500 (MACRAW, socket 0) emulation. See w5500.h.

	SPI frame: [addr_hi][addr_lo][control][data...]; control = (BSB<<3)|(RW<<2)|OM.
	BSB 0=common, 1=socket0 reg, 2=socket0 TX buffer, 3=socket0 RX buffer. VDM (OM=0):
	CS frames the transaction; the 16-bit address auto-increments per data byte.
	MACRAW RX: each packet in the RX buffer is prefixed with a 2-byte length (incl header).
*/
#include "w5500.h"
#include "network/netservice.h"
#include <cstdlib>
#include <cstring>
#include <deque>
#include <vector>
#include <mutex>

// ---- common + socket-0 register offsets ----------------------------------------
#define MR        0x0000
#define SHAR      0x0009
#define VERSIONR  0x0039
#define Sn_MR     0x0000
#define Sn_CR     0x0001
#define Sn_SR     0x0003
#define Sn_RXBUF  0x001E
#define Sn_TXBUF  0x001F
#define Sn_TX_FSR 0x0020
#define Sn_TX_RD  0x0022
#define Sn_TX_WR  0x0024
#define Sn_RX_RSR 0x0026
#define Sn_RX_RD  0x0028
#define Sn_RX_WR  0x002A
#define CMD_OPEN  0x01
#define CMD_CLOSE 0x10
#define CMD_SEND  0x20
#define CMD_RECV  0x40
#define SOCK_MACRAW 0x42
#define BUFSZ     0x4000          // 16 KiB per socket (we give socket 0 all of it)
#define BUFMASK   (BUFSZ - 1)

namespace {

bool  csLow;                      // CS asserted
int   phase;                      // 0..2 header bytes, 3 = data
u8    ctrl, bsb;
bool  rwWrite;
u16   curAddr;

u8    mac[6];
u8    sn_mr, sn_sr;
u16   tx_wr, tx_rd, rx_wr, rx_rd;
u8    txBuf[BUFSZ];
u8    rxBuf[BUFSZ];

std::deque<std::vector<u8>> rxQueue;   // host->DC frames (filled on the net thread)
std::mutex rxMutex;

void doReset()
{
	csLow = false; phase = 0; ctrl = 0; bsb = 0; rwWrite = false; curAddr = 0;
	sn_mr = 0; sn_sr = 0;
	tx_wr = tx_rd = rx_wr = rx_rd = 0;
	std::lock_guard<std::mutex> lk(rxMutex);
	rxQueue.clear();
}

// Move queued host frames into the RX ring (SH-4 thread only touches the ring).
void pumpRx()
{
	std::lock_guard<std::mutex> lk(rxMutex);
	while (!rxQueue.empty())
	{
		const std::vector<u8>& f = rxQueue.front();
		u32 need = (u32)f.size() + 2;
		u16 free = (u16)(BUFSZ - (u16)(rx_wr - rx_rd));
		if (need > free)
			break;                                  // ring full; leave queued
		u16 p = rx_wr;
		rxBuf[p & BUFMASK] = (u8)(need >> 8); p++;   // MACRAW 2-byte length (incl header)
		rxBuf[p & BUFMASK] = (u8)need;        p++;
		for (size_t i = 0; i < f.size(); i++) { rxBuf[p & BUFMASK] = f[i]; p++; }
		rx_wr = p;
		rxQueue.pop_front();
	}
}

void doSend()
{
	u16 len = (u16)(tx_wr - tx_rd);
	if (len == 0 || len > 1600) { tx_rd = tx_wr; return; }
	u8 frame[1600];
	for (u16 i = 0; i < len; i++)
		frame[i] = txBuf[(tx_rd + i) & BUFMASK];
	tx_rd = tx_wr;
	net::modbba::receiveEthFrame(frame, len);
}

void writeByte(u8 in)
{
	if (bsb == 2) { txBuf[curAddr & BUFMASK] = in; return; }   // TX buffer
	if (bsb == 3) return;                                       // RX buffer (driver never writes)
	if (bsb == 0)                                               // common regs
	{
		if (curAddr == MR && (in & 0x80)) doReset();
		else if (curAddr >= SHAR && curAddr < SHAR + 6) mac[curAddr - SHAR] = in;
		return;
	}
	// bsb == 1: socket 0 registers
	switch (curAddr)
	{
	case Sn_MR: sn_mr = in; break;
	case Sn_CR:
		if (in == CMD_OPEN)  sn_sr = ((sn_mr & 0x0f) == 0x04) ? SOCK_MACRAW : 0;
		else if (in == CMD_CLOSE) sn_sr = 0;
		else if (in == CMD_SEND)  doSend();
		// CMD_RECV: driver already advanced Sn_RX_RD; RSR recomputes on read
		break;
	case Sn_TX_WR:     tx_wr = (u16)((tx_wr & 0x00ff) | (in << 8)); break;
	case Sn_TX_WR + 1: tx_wr = (u16)((tx_wr & 0xff00) | in);        break;
	case Sn_RX_RD:     rx_rd = (u16)((rx_rd & 0x00ff) | (in << 8)); break;
	case Sn_RX_RD + 1: rx_rd = (u16)((rx_rd & 0xff00) | in);        break;
	default: break;                                            // RXBUF/TXBUF size etc. - fixed 16K
	}
}

u8 readByte()
{
	if (bsb == 3) return rxBuf[curAddr & BUFMASK];             // RX buffer
	if (bsb == 2) return 0;                                     // TX buffer (not read)
	if (bsb == 0)                                               // common regs
	{
		if (curAddr == VERSIONR) return 0x04;                  // chip signature
		if (curAddr >= SHAR && curAddr < SHAR + 6) return mac[curAddr - SHAR];
		return 0;
	}
	// bsb == 1: socket 0 registers
	switch (curAddr)
	{
	case Sn_CR:        return 0;                               // command auto-cleared
	case Sn_SR:        return sn_sr;
	case Sn_TX_FSR:    return (u8)((BUFSZ - (u16)(tx_wr - tx_rd)) >> 8);
	case Sn_TX_FSR + 1:return (u8) (BUFSZ - (u16)(tx_wr - tx_rd));
	case Sn_TX_RD:     return (u8)(tx_rd >> 8);
	case Sn_TX_RD + 1: return (u8) tx_rd;
	case Sn_TX_WR:     return (u8)(tx_wr >> 8);
	case Sn_TX_WR + 1: return (u8) tx_wr;
	case Sn_RX_RSR:    return (u8)((u16)(rx_wr - rx_rd) >> 8);
	case Sn_RX_RSR + 1:return (u8) (u16)(rx_wr - rx_rd);
	case Sn_RX_RD:     return (u8)(rx_rd >> 8);
	case Sn_RX_RD + 1: return (u8) rx_rd;
	case Sn_RX_WR:     return (u8)(rx_wr >> 8);
	case Sn_RX_WR + 1: return (u8) rx_wr;
	default:           return 0;
	}
}

} // namespace

bool w5500_active()
{
	static int cached = -1;
	if (cached < 0)
	{
		const char *e = getenv("FLYCAST_W5500");
		cached = (e != nullptr && e[0] != '\0' && e[0] != '0') ? 1 : 0;
	}
	return cached != 0;
}

void w5500_reset() { doReset(); }

void w5500_spi_cs(bool assert)
{
	if (assert && !csLow)
	{
		pumpRx();                  // bring queued host frames into the ring
		phase = 0;                 // start a fresh transaction
	}
	csLow = assert;
}

u8 w5500_spi_byte(u8 in)
{
	if (!csLow) return 0xff;
	switch (phase)
	{
	case 0: curAddr = (u16)(in << 8); phase = 1; return 0xff;
	case 1: curAddr = (u16)(curAddr | in); phase = 2; return 0xff;
	case 2:
		ctrl = in; bsb = (u8)(ctrl >> 3); rwWrite = (ctrl & 0x04) != 0; phase = 3;
		return 0xff;
	default:
		if (rwWrite) { writeByte(in); curAddr++; return 0xff; }
		else         { u8 v = readByte(); curAddr++; return v; }
	}
}

int w5500_rx_frame(const u8 *data, int len)
{
	if (len <= 0 || len > 1600) return 1;
	std::lock_guard<std::mutex> lk(rxMutex);
	if (rxQueue.size() < 64)
		rxQueue.emplace_back(data, data + len);
	return 1;
}
