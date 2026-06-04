#ifndef __ETH_RX_H__
#define __ETH_RX_H__

#include <stdint.h>
#include "hardware/pio.h"

#define ETH_RX_SAMPLE_HZ      (60u * 1000u * 1000u)   // 60 MHz, 3 samples per Manchester half-bit
#define ETH_RX_BUF_LOG2_BYTES 15                       // 2^15 = 32 KB
#define ETH_RX_BUF_BYTES      (1u << ETH_RX_BUF_LOG2_BYTES)
#define ETH_RX_BUF_WORDS      (ETH_RX_BUF_BYTES / 4)

void eth_rx_init(PIO pio, uint sm, uint pin_rx);

// Returns the byte offset in eth_rx_buf where DMA will write next.
// (Useful for finding the most-recent samples.)
uint32_t eth_rx_dma_write_offset(void);

// Smoke test: print n_bytes from the ring buffer ending at the current
// DMA write address (with wrap).
void eth_rx_dump_recent(uint32_t n_bytes);

// Scan the whole buffer for non-idle bytes and dump 256 bytes around the first hit.
void eth_rx_find_and_dump_activity(void);

// Scan the buffer for the longest stretch of "active" bytes (not 0x00/0xFF),
// which corresponds to a frame's worth of Manchester audio. Report it and
// dump the first ~384 bytes around it.
void eth_rx_find_frame_candidates(void);

// Find the longest active run, phase-lock to the preamble's first falling
// edge, decode Manchester, find SFD, and print the first N decoded bytes.
void eth_rx_decode_frame(void);

extern uint32_t eth_rx_buf[ETH_RX_BUF_WORDS];

#endif // __ETH_RX_H__
