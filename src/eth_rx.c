#include "eth_rx.h"
#include "hardware/dma.h"
#include "rx_10base_t.pio.h"
#include <stdio.h>

__attribute__((aligned(ETH_RX_BUF_BYTES)))
uint32_t eth_rx_buf[ETH_RX_BUF_WORDS];

static uint eth_rx_dma_ch;

static uint32_t eth_rx_crc_table[256];
static bool eth_rx_crc_ready = false;

static void eth_rx_make_crc_table(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (uint32_t j = 0; j < 8; j++) {
            c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
        }
        eth_rx_crc_table[i] = c;
    }
    eth_rx_crc_ready = true;
}

static uint32_t eth_rx_crc32(const uint8_t *data, uint32_t n) {
    if (!eth_rx_crc_ready) eth_rx_make_crc_table();
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        crc = (crc >> 8) ^ eth_rx_crc_table[(crc ^ data[i]) & 0xFFu];
    }
    return crc ^ 0xFFFFFFFFu;
}

void eth_rx_init(PIO pio, uint sm, uint pin_rx) {
    uint offset = pio_add_program(pio, &rx_10base_t_program);
    rx_10base_t_program_init(pio, sm, offset, pin_rx, ETH_RX_SAMPLE_HZ);

    eth_rx_dma_ch = dma_claim_unused_channel(true);
    dma_channel_config conf = dma_channel_get_default_config(eth_rx_dma_ch);
    channel_config_set_transfer_data_size(&conf, DMA_SIZE_32);
    channel_config_set_read_increment(&conf, false);
    channel_config_set_write_increment(&conf, true);
    channel_config_set_dreq(&conf, pio_get_dreq(pio, sm, false));
    // Wrap the write address every 2^15 bytes (= 32 KB) so the buffer is a ring.
    channel_config_set_ring(&conf, true, ETH_RX_BUF_LOG2_BYTES);

    dma_channel_configure(eth_rx_dma_ch, &conf,
        eth_rx_buf,                            // initial write address
        &pio->rxf[sm],                         // PIO RX FIFO
        dma_encode_endless_transfer_count(),   // RP2350 endless mode
        true                                   // start now
    );

    printf("[eth_rx] init: buf=%p sm=%u dma=%u sample_hz=%u\n",
           (void*)eth_rx_buf, sm, eth_rx_dma_ch, ETH_RX_SAMPLE_HZ);
}

uint32_t eth_rx_dma_write_offset(void) {
    uint32_t base = (uint32_t)eth_rx_buf;
    uint32_t wa   = dma_hw->ch[eth_rx_dma_ch].write_addr;
    return (wa - base) & (ETH_RX_BUF_BYTES - 1u);
}

void eth_rx_dump_recent(uint32_t n_bytes) {
    if (n_bytes > ETH_RX_BUF_BYTES) n_bytes = ETH_RX_BUF_BYTES;

    uint32_t end_off = eth_rx_dma_write_offset();
    // Start n_bytes back from where DMA is currently writing.
    uint32_t start_off = (end_off + ETH_RX_BUF_BYTES - n_bytes) & (ETH_RX_BUF_BYTES - 1u);

    const uint8_t *bytes = (const uint8_t *)eth_rx_buf;
    printf("[eth_rx] dump %lu bytes ending at offset 0x%04lx (start 0x%04lx):\n",
           n_bytes, end_off, start_off);
    for (uint32_t i = 0; i < n_bytes; i++) {
        if ((i & 0x1f) == 0) printf("\n  %04lx:", i);
        uint32_t off = (start_off + i) & (ETH_RX_BUF_BYTES - 1u);
        printf(" %02x", bytes[off]);
    }
    printf("\n");
}

// Find the longest stretch of "active" bytes (not 0x00 and not 0xFF) in the
// buffer — that's frame-shaped audio. NLPs are 1-2 bytes wide; frames are 50+.
void eth_rx_find_frame_candidates(void) {
    const uint8_t *bytes = (const uint8_t *)eth_rx_buf;

    uint32_t run_start = UINT32_MAX;
    uint32_t run_len = 0;
    uint32_t best_start = UINT32_MAX;
    uint32_t best_len = 0;

    for (uint32_t i = 0; i < ETH_RX_BUF_BYTES; i++) {
        uint8_t b = bytes[i];
        bool active = (b != 0xFF && b != 0x00);
        if (active) {
            if (run_start == UINT32_MAX) run_start = i;
            run_len++;
            if (run_len > best_len) {
                best_start = run_start;
                best_len = run_len;
            }
        } else {
            run_start = UINT32_MAX;
            run_len = 0;
        }
    }

    if (best_len < 16) {
        printf("[eth_rx] no frame candidates (longest active run = %lu bytes)\n",
               best_len);
        return;
    }

    // Each byte = 8 samples at 60 MHz = 133 ns of line audio.
    // 64-byte Ethernet frame is ~51 us audio = ~384 bytes of buffer.
    uint32_t us = (best_len * 8u * 1000u) / (ETH_RX_SAMPLE_HZ / 1000u);
    printf("[eth_rx] frame candidate: offset 0x%04lx, %lu active bytes (~%lu us audio)\n",
           best_start, best_len, us);

    // Dump 384 bytes starting 32 bytes before the candidate (catch preamble entry).
    uint32_t dump_start = (best_start >= 32) ? (best_start - 32) : 0;
    if (dump_start + 384 > ETH_RX_BUF_BYTES) dump_start = ETH_RX_BUF_BYTES - 384;

    printf("[eth_rx]   dump 384 bytes from 0x%04lx:", dump_start);
    for (uint32_t i = 0; i < 384; i++) {
        if ((i & 0x1f) == 0) printf("\n  %04lx:", dump_start + i);
        printf(" %02x", bytes[dump_start + i]);
    }
    printf("\n");
}

// Read one sample bit from the ring buffer.
// bit_offset is measured from the start of `byte_base`, in samples (=bits in the
// stored data, since each byte = 8 samples LSB-first).
static inline uint8_t rx_sample(uint32_t byte_base, uint32_t bit_offset) {
    const uint8_t *buf = (const uint8_t *)eth_rx_buf;
    uint32_t boff = (byte_base + (bit_offset >> 3)) & (ETH_RX_BUF_BYTES - 1u);
    return (buf[boff] >> (bit_offset & 7u)) & 1u;
}

// Locate the longest active run (frame audio); returns its byte offset & length.
// Returns true if a credible run (>= 100 bytes) was found.
static bool rx_find_longest_active(uint32_t *out_start, uint32_t *out_len) {
    const uint8_t *bytes = (const uint8_t *)eth_rx_buf;
    uint32_t run_start = UINT32_MAX, run_len = 0;
    uint32_t best_start = UINT32_MAX, best_len = 0;
    for (uint32_t i = 0; i < ETH_RX_BUF_BYTES; i++) {
        uint8_t b = bytes[i];
        if (b != 0xFF && b != 0x00) {
            if (run_start == UINT32_MAX) run_start = i;
            run_len++;
            if (run_len > best_len) { best_start = run_start; best_len = run_len; }
        } else {
            run_start = UINT32_MAX; run_len = 0;
        }
    }
    *out_start = best_start;
    *out_len = best_len;
    return best_len >= 100u;
}

void eth_rx_decode_frame(void) {
    uint32_t base, nbytes;
    if (!rx_find_longest_active(&base, &nbytes)) {
        printf("[eth_rx] decode: no frame-sized run (longest=%lu)\n", nbytes);
        return;
    }

    uint32_t nsamples = nbytes * 8;

    // Find the first HIGH->LOW falling edge in the active run. In a preamble,
    // every other half-bit transition is falling; the first one within the
    // active region marks a mid-bit boundary (the start of HB[3] = the second
    // half of "bit 1" in our local indexing).
    uint32_t F = UINT32_MAX;
    uint8_t prev = rx_sample(base, 0);
    for (uint32_t i = 1; i < nsamples; i++) {
        uint8_t s = rx_sample(base, i);
        if (prev == 1 && s == 0) { F = i; break; }
        prev = s;
    }
    if (F == UINT32_MAX) {
        printf("[eth_rx] decode: no falling edge in run\n");
        return;
    }

    // Phase-lock: F is the first LOW sample after a HIGH run = the start of
    // HB[0] (first half of preamble bit 0) when entering from idle.
    // (Preamble bit 0 = data 1 = LH; first half = L.)
    // Data bit k value = midpoint of HB[2k+1] (second half) = sample F + 4 + 6k.
    static uint8_t bits[2048];
    uint32_t nbits = 0;
    for (uint32_t j = 0; j < 1600u; j++) {
        uint32_t idx = F + 4u + 6u * j;
        if (idx >= nsamples) break;
        bits[nbits++] = rx_sample(base, idx);
        if (nbits >= sizeof(bits)) break;
    }

    // Locate SFD: in the decoded stream, preamble bits alternate 1,0,1,0,...
    // SFD byte 0xD5 transmitted LSB-first ends in two 1s (bits 6 and 7 of SFD).
    // First "1,1" pair in the stream = end of SFD.
    uint32_t sfd_end = UINT32_MAX;
    for (uint32_t i = 1; i < nbits; i++) {
        if (bits[i] == 1 && bits[i - 1] == 1) { sfd_end = i; break; }
    }
    if (sfd_end == UINT32_MAX) {
        printf("[eth_rx] decode: SFD not found in %lu bits (F=%lu, %lu bytes audio)\n",
               nbits, F, nbytes);
        return;
    }

    // Pack frame bytes (LSB-first per IEEE 802.3) from the bit after SFD.
    uint32_t start_bit = sfd_end + 1;
    uint32_t avail_bits = (nbits > start_bit) ? (nbits - start_bit) : 0;
    static uint8_t frame[1600];
    uint32_t nframe = avail_bits / 8;
    if (nframe > sizeof(frame)) nframe = sizeof(frame);
    for (uint32_t i = 0; i < nframe; i++) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; j++) byte |= bits[start_bit + i * 8u + j] << j;
        frame[i] = byte;
    }

    // Determine the real frame length. For IPv4 (EtherType 0x0800), the IP
    // total-length field at offset 16..17 (big-endian) tells us the IP packet
    // size. Frame = 14 (Ethernet header) + ip_total_len + 4 (FCS).
    uint32_t frame_len = nframe; // fallback
    if (nframe >= 18) {
        uint16_t etype = ((uint16_t)frame[12] << 8) | frame[13];
        if (etype == 0x0800u) {
            uint16_t ip_total_len = ((uint16_t)frame[16] << 8) | frame[17];
            uint32_t computed = 14u + (uint32_t)ip_total_len + 4u;
            if (computed <= nframe) frame_len = computed;
        } else if (etype == 0x0806u && nframe >= 64u) {
            frame_len = 64u; // ARP: 60-byte min payload + 4-byte FCS
        }
    }

    // Verify the FCS: CRC-32/IEEE-802.3 over bytes [0..frame_len-5],
    // expected to match the trailing 4 bytes (little-endian on the wire).
    bool fcs_ok = false;
    uint32_t crc_computed = 0, fcs_in_frame = 0;
    if (frame_len >= 14u + 4u) {
        crc_computed = eth_rx_crc32(frame, frame_len - 4u);
        fcs_in_frame =  (uint32_t)frame[frame_len - 4u]
                     | ((uint32_t)frame[frame_len - 3u] << 8)
                     | ((uint32_t)frame[frame_len - 2u] << 16)
                     | ((uint32_t)frame[frame_len - 1u] << 24);
        fcs_ok = (crc_computed == fcs_in_frame);
    }

    printf("[eth_rx] frame %lu bytes, FCS %s — dst %02x:%02x:%02x:%02x:%02x:%02x  src %02x:%02x:%02x:%02x:%02x:%02x  type=%04x\n",
           frame_len, fcs_ok ? "OK" : "FAIL",
           frame[0], frame[1], frame[2], frame[3], frame[4], frame[5],
           frame[6], frame[7], frame[8], frame[9], frame[10], frame[11],
           (uint16_t)((frame[12] << 8) | frame[13]));
    uint32_t to_print = frame_len < 64u ? frame_len : 64u;
    for (uint32_t i = 0; i < to_print; i++) {
        if ((i & 0xf) == 0) printf("\n  %04lx:", i);
        printf(" %02x", frame[i]);
    }
    printf("\n");
}

// Scan the entire buffer for the first stretch of "activity" — a region
// where multiple bytes differ from 0xFF (idle high) or 0x00. Print a
// summary of where we found activity and 256 bytes centered on it.
void eth_rx_find_and_dump_activity(void) {
    const uint8_t *bytes = (const uint8_t *)eth_rx_buf;

    uint32_t activity_offset = UINT32_MAX;
    uint32_t total_active = 0;

    for (uint32_t i = 0; i < ETH_RX_BUF_BYTES; i++) {
        if (bytes[i] != 0xFF && bytes[i] != 0x00) {
            total_active++;
            if (activity_offset == UINT32_MAX) activity_offset = i;
        }
    }

    printf("[eth_rx] activity scan: %lu non-trivial bytes in 32 KB buffer\n",
           total_active);

    if (activity_offset == UINT32_MAX) {
        printf("[eth_rx]   (no activity found — buffer is pure idle)\n");
        return;
    }

    // Center 256 bytes on the activity region.
    uint32_t start_off = (activity_offset >= 64) ? (activity_offset - 64) : 0;
    if (start_off + 256 > ETH_RX_BUF_BYTES) start_off = ETH_RX_BUF_BYTES - 256;

    printf("[eth_rx]   first activity at offset 0x%04lx, dumping 256 bytes from 0x%04lx:\n",
           activity_offset, start_off);
    for (uint32_t i = 0; i < 256; i++) {
        if ((i & 0x1f) == 0) printf("\n  %04lx:", start_off + i);
        printf(" %02x", bytes[start_off + i]);
    }
    printf("\n");
}
