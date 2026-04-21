#ifndef __ETH_RX_H__
#define __ETH_RX_H__

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

#define ETH_RX_FRAME_BUF_SIZE   1536

void eth_rx_init(uint pin_rx);
bool eth_rx_poll(uint8_t *frame, uint16_t *len);

// Debug: cumulative signal stats accumulated by eth_rx_poll
typedef struct {
    uint32_t dma_write_idx;     // current DMA write position (words)
    uint32_t words_scanned;     // cumulative words scanned by poll
    uint32_t ones;              // cumulative '1' samples
    uint32_t transitions;       // cumulative bit-to-bit transitions
    uint32_t max_run;           // longest run of consecutive same-bit samples
} eth_rx_debug_t;

void eth_rx_debug_get(eth_rx_debug_t *out);
void eth_rx_debug_reset(void);
// Scan ring buffer continuously and update debug counters (call from main loop)
void eth_rx_debug_scan(void);

#endif // __ETH_RX_H__
