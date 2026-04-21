/********************************************************
 * Title    : 10BASE-T Manchester Receiver
 * Date     : 2026/04/08
 * Note     : 4x oversampled Manchester decoder with
 *            DMA ring buffer from PIO1.
 *            Input: comparator output on single GPIO.
 ********************************************************/
#include "eth_rx.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "rx_10base_t.pio.h"
#include <string.h>

// Ring buffer: must be power-of-2 sized and aligned
#define RX_RING_SIZE_WORDS  2048
#define RX_RING_SIZE_BYTES  (RX_RING_SIZE_WORDS * 4)

static uint32_t rx_ring_buf[RX_RING_SIZE_WORDS] __attribute__((aligned(RX_RING_SIZE_BYTES)));
static uint32_t rx_read_idx;

// PIO / DMA state
static PIO      pio_rx = pio1;
static uint     sm_rx  = 0;
static uint     dma_ch_rx;

// Decoder state
typedef enum {
    RX_STATE_IDLE,
    RX_STATE_PREAMBLE,
    RX_STATE_DATA,
} rx_state_t;

static rx_state_t  rx_state;
static uint8_t     rx_frame[ETH_RX_FRAME_BUF_SIZE];
static uint16_t    rx_frame_len;
static uint8_t     rx_bit_count;
static uint8_t     rx_cur_byte;
static int         rx_phase;           // Sample phase within 4-sample bit window
static uint32_t    rx_idle_count;      // Count of consecutive idle samples
static uint8_t     rx_preamble_count;  // Count of valid preamble bytes seen
static bool        rx_frame_ready;
static uint8_t     rx_ready_frame[ETH_RX_FRAME_BUF_SIZE];
static uint16_t    rx_ready_len;


void eth_rx_init(uint pin_rx)
{
    // Init PIO1 for RX oversampling
    uint offset = pio_add_program(pio_rx, &rx_10base_t_program);
    rx_10base_t_program_init(pio_rx, sm_rx, offset, pin_rx);

    // Configure DMA: PIO1 RX FIFO -> ring buffer, continuous
    dma_ch_rx = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_ch_rx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio_rx, sm_rx, false));
    // Ring wrap on write address: log2(RX_RING_SIZE_BYTES)
    channel_config_set_ring(&c, true, 13);  // 2^13 = 8192 bytes

    dma_channel_configure(
        dma_ch_rx,
        &c,
        rx_ring_buf,                    // Write to ring buffer
        &pio_rx->rxf[sm_rx],            // Read from PIO RX FIFO
        0xFFFFFFFF,                     // Transfer "forever" (ring wraps)
        true                            // Start immediately
    );

    rx_read_idx = 0;
    rx_state = RX_STATE_IDLE;
    rx_frame_len = 0;
    rx_bit_count = 0;
    rx_cur_byte = 0;
    rx_phase = -1;
    rx_idle_count = 0;
    rx_preamble_count = 0;
    rx_frame_ready = false;
}


// Get current DMA write position in ring buffer (in words)
static inline uint32_t rx_dma_write_idx(void)
{
    uint32_t write_addr = (uint32_t)dma_channel_hw_addr(dma_ch_rx)->write_addr;
    uint32_t base_addr  = (uint32_t)rx_ring_buf;
    return (write_addr - base_addr) / 4;
}


// Get number of available words in ring buffer
static inline uint32_t rx_available(void)
{
    uint32_t write_idx = rx_dma_write_idx();
    return (write_idx - rx_read_idx) & (RX_RING_SIZE_WORDS - 1);
}


// Extract a single bit from the ring buffer at a given bit position
static inline uint8_t rx_get_sample(uint32_t bit_pos)
{
    uint32_t word_idx = (bit_pos / 32) & (RX_RING_SIZE_WORDS - 1);
    uint32_t bit_idx  = 31 - (bit_pos % 32);  // MSB first (shift left in PIO)
    return (rx_ring_buf[word_idx] >> bit_idx) & 1;
}


// Decode one Manchester bit from 4 samples starting at the given phase
// Returns: 0 or 1 for valid data bit, -1 for invalid/idle
static int decode_manchester_bit(uint32_t base_bit_pos)
{
    // At 4x oversample, a bit period has 4 samples.
    // Manchester '0' (low-to-high): first half low, second half high -> ~0011
    // Manchester '1' (high-to-low): first half high, second half low -> ~1100
    // We look at the two middle samples (indices 1 and 2) for the transition.
    uint8_t s1 = rx_get_sample(base_bit_pos + 1);
    uint8_t s2 = rx_get_sample(base_bit_pos + 2);

    if (s1 == 1 && s2 == 0) {
        return 1;   // High-to-low transition = '1'
    } else if (s1 == 0 && s2 == 1) {
        return 0;   // Low-to-high transition = '0'
    }
    return -1;  // No valid transition (idle or error)
}


// Push a decoded bit into the frame buffer
static void rx_push_bit(int bit)
{
    rx_cur_byte |= (bit << rx_bit_count);  // LSB first
    rx_bit_count++;

    if (rx_bit_count == 8) {
        if (rx_frame_len < ETH_RX_FRAME_BUF_SIZE) {
            rx_frame[rx_frame_len++] = rx_cur_byte;
        }
        rx_bit_count = 0;
        rx_cur_byte = 0;
    }
}


// Complete frame reception - copy to ready buffer
static void rx_complete_frame(void)
{
    // Minimum valid Ethernet frame: 14 (header) + 4 (FCS) = 18 bytes
    // Discard runt frames
    if (rx_frame_len >= 18 && !rx_frame_ready) {
        memcpy(rx_ready_frame, rx_frame, rx_frame_len);
        rx_ready_len = rx_frame_len;
        rx_frame_ready = true;
    }
}


// Try to find preamble phase by looking for alternating transitions
// Returns phase offset (0-3) or -1 if not found
static int find_preamble_phase(uint32_t start_bit_pos)
{
    // Try each possible phase alignment
    for (int phase = 0; phase < 4; phase++) {
        int good = 0;
        for (int i = 0; i < 4; i++) {
            int bit = decode_manchester_bit(start_bit_pos + phase + i * 4);
            if (bit == 0 || bit == 1) good++;
        }
        if (good >= 3) return phase;
    }
    return -1;
}


bool eth_rx_poll(uint8_t *frame, uint16_t *len)
{
    // If a complete frame is ready, return it
    if (rx_frame_ready) {
        memcpy(frame, rx_ready_frame, rx_ready_len);
        *len = rx_ready_len;
        rx_frame_ready = false;
        return true;
    }

    uint32_t avail = rx_available();
    if (avail == 0) return false;

    // Process available samples (each word = 32 samples = 8 bit periods at 4x)
    // We process sample-by-sample through the ring buffer
    uint32_t total_bits = avail * 32;

    // Don't process too much at once to keep main loop responsive
    if (total_bits > 8192) total_bits = 8192;

    uint32_t base_bit = rx_read_idx * 32;
    uint32_t bit_pos = 0;

    while (bit_pos + 4 <= total_bits) {
        switch (rx_state) {

        case RX_STATE_IDLE: {
            // Look for signal activity: transitions in the data
            // Scan for first transition (any edge)
            uint8_t s0 = rx_get_sample(base_bit + bit_pos);
            uint8_t s1 = rx_get_sample(base_bit + bit_pos + 1);

            if (s0 != s1) {
                // Found a transition - potential start of preamble
                // Try to find phase alignment
                int phase = find_preamble_phase(base_bit + bit_pos);
                if (phase >= 0) {
                    rx_phase = phase;
                    rx_state = RX_STATE_PREAMBLE;
                    rx_preamble_count = 0;
                    rx_frame_len = 0;
                    rx_bit_count = 0;
                    rx_cur_byte = 0;
                    rx_idle_count = 0;
                    // Align bit_pos to the found phase
                    bit_pos += phase;
                    continue;
                }
            }
            bit_pos++;
            break;
        }

        case RX_STATE_PREAMBLE: {
            // Decode bits looking for preamble (0x55 = 10101010) then SFD (0xD5 = 10101011)
            int bit = decode_manchester_bit(base_bit + bit_pos);
            bit_pos += 4;  // Advance by one bit period

            if (bit < 0) {
                // Invalid transition - back to idle
                rx_state = RX_STATE_IDLE;
                break;
            }

            rx_push_bit(bit);

            // Check if we've accumulated a full byte
            if (rx_bit_count == 0 && rx_frame_len > 0) {
                uint8_t last_byte = rx_frame[rx_frame_len - 1];

                if (last_byte == 0x55) {
                    // Preamble byte
                    rx_preamble_count++;
                } else if (last_byte == 0xD5 && rx_preamble_count >= 1) {
                    // SFD found! Start receiving data.
                    // Reset frame buffer - preamble and SFD are not part of the frame
                    rx_frame_len = 0;
                    rx_bit_count = 0;
                    rx_cur_byte = 0;
                    rx_state = RX_STATE_DATA;
                } else {
                    // Unexpected byte during preamble
                    rx_state = RX_STATE_IDLE;
                }
            }
            break;
        }

        case RX_STATE_DATA: {
            int bit = decode_manchester_bit(base_bit + bit_pos);
            bit_pos += 4;

            if (bit < 0) {
                rx_idle_count++;
                if (rx_idle_count >= 2) {
                    // End of frame (sustained idle)
                    // Flush any partial byte (shouldn't happen for valid frames)
                    rx_complete_frame();
                    rx_state = RX_STATE_IDLE;
                }
                break;
            }

            rx_idle_count = 0;
            rx_push_bit(bit);

            // Guard against buffer overflow
            if (rx_frame_len >= ETH_RX_FRAME_BUF_SIZE) {
                rx_state = RX_STATE_IDLE;
            }
            break;
        }

        }  // switch
    }

    // Update read index: advance by the number of complete words we consumed
    uint32_t words_consumed = bit_pos / 32;
    rx_read_idx = (rx_read_idx + words_consumed) & (RX_RING_SIZE_WORDS - 1);

    // Check again if a frame became ready during processing
    if (rx_frame_ready) {
        memcpy(frame, rx_ready_frame, rx_ready_len);
        *len = rx_ready_len;
        rx_frame_ready = false;
        return true;
    }

    return false;
}


// Cumulative debug counters (separate read pointer so we don't fight eth_rx_poll)
static uint32_t dbg_scan_idx;
static uint32_t dbg_words_scanned;
static uint32_t dbg_ones;
static uint32_t dbg_transitions;
static uint32_t dbg_max_run;
static uint32_t dbg_cur_run;
static uint8_t  dbg_prev_bit;
static bool     dbg_first = true;

void eth_rx_debug_reset(void)
{
    dbg_scan_idx = rx_dma_write_idx();
    dbg_words_scanned = 0;
    dbg_ones = 0;
    dbg_transitions = 0;
    dbg_max_run = 0;
    dbg_cur_run = 0;
    dbg_first = true;
}

void eth_rx_debug_scan(void)
{
    uint32_t write_idx = rx_dma_write_idx();
    uint32_t avail = (write_idx - dbg_scan_idx) & (RX_RING_SIZE_WORDS - 1);
    if (avail == 0) return;

    // Cap so we don't hog the loop
    if (avail > 512) avail = 512;

    for (uint32_t i = 0; i < avail; i++) {
        uint32_t w = rx_ring_buf[(dbg_scan_idx + i) & (RX_RING_SIZE_WORDS - 1)];
        for (int b = 31; b >= 0; b--) {
            uint8_t bit = (w >> b) & 1;
            if (bit) dbg_ones++;
            if (!dbg_first) {
                if (bit != dbg_prev_bit) {
                    dbg_transitions++;
                    if (dbg_cur_run > dbg_max_run) dbg_max_run = dbg_cur_run;
                    dbg_cur_run = 1;
                } else {
                    dbg_cur_run++;
                }
            } else {
                dbg_cur_run = 1;
            }
            dbg_prev_bit = bit;
            dbg_first = false;
        }
    }

    dbg_words_scanned += avail;
    dbg_scan_idx = (dbg_scan_idx + avail) & (RX_RING_SIZE_WORDS - 1);
}

void eth_rx_debug_get(eth_rx_debug_t *out)
{
    out->dma_write_idx = rx_dma_write_idx();
    out->words_scanned = dbg_words_scanned;
    out->ones          = dbg_ones;
    out->transitions   = dbg_transitions;
    out->max_run       = dbg_max_run;
}
