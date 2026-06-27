/*
	w5500.h - WIZnet W5500 (MACRAW socket 0) emulation for the Dreamcast WinCE port's
	SPI ethernet backend. The DC drives it over the SH-4 SCI (clocked-synchronous SPI);
	this device reconstructs the SPI register/buffer protocol and bridges MACRAW frames
	through the existing host network (net::modbba, same as the BBA).

	Enabled by the env var FLYCAST_W5500=1. CS comes from PA7 (BSC_PDTRA bit 7), SPI bytes
	from the SCI (serial.cpp), host->DC frames from bba_recv_frame (bba.cpp).
*/
#pragma once
#include "types.h"

// Is the W5500 SPI device active (env FLYCAST_W5500)? Cached on first call.
bool w5500_active();

// Hard reset of the virtual chip state.
void w5500_reset();

// CS line from PA7: assert=true => CS low (transaction begins).
void w5500_spi_cs(bool assert);

// Exchange one SPI byte (MSB-first, already un-bit-reversed by the caller).
u8 w5500_spi_byte(u8 in);

// Deliver a host->DC ethernet frame into socket 0's MACRAW RX buffer. Returns 1 if consumed.
int w5500_rx_frame(const u8 *data, int len);
