/********************************************************
 * Title    : 10BASE-T lwIP Network Interface Driver
 * Date     : 2026/04/08
 * Note     : TX logic migrated from udp.c
 *            RX via eth_rx.c Manchester decoder
 *            Integrates with lwIP as a netif driver
 ********************************************************/
#include "eth_netif.h"
#include "eth_rx.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "ser_10base_t.pio.h"

#include "lwip/etharp.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "netif/ethernet.h"

#include <string.h>

// Manchester encoding table (from original udp.c)
// input 8bit, output 32bit, LSB first
// b00 -> IDLE, b01 -> LOW, b10 -> HIGH, b11 -> not use
static const uint32_t tbl_manchester[256] = {
    0x66666666, 0x66666669, 0x66666696, 0x66666699, 0x66666966, 0x66666969, 0x66666996, 0x66666999, 0x66669666, 0x66669669, 0x66669696, 0x66669699, 0x66669966, 0x66669969, 0x66669996, 0x66669999,
    0x66696666, 0x66696669, 0x66696696, 0x66696699, 0x66696966, 0x66696969, 0x66696996, 0x66696999, 0x66699666, 0x66699669, 0x66699696, 0x66699699, 0x66699966, 0x66699969, 0x66699996, 0x66699999,
    0x66966666, 0x66966669, 0x66966696, 0x66966699, 0x66966966, 0x66966969, 0x66966996, 0x66966999, 0x66969666, 0x66969669, 0x66969696, 0x66969699, 0x66969966, 0x66969969, 0x66969996, 0x66969999,
    0x66996666, 0x66996669, 0x66996696, 0x66996699, 0x66996966, 0x66996969, 0x66996996, 0x66996999, 0x66999666, 0x66999669, 0x66999696, 0x66999699, 0x66999966, 0x66999969, 0x66999996, 0x66999999,
    0x69666666, 0x69666669, 0x69666696, 0x69666699, 0x69666966, 0x69666969, 0x69666996, 0x69666999, 0x69669666, 0x69669669, 0x69669696, 0x69669699, 0x69669966, 0x69669969, 0x69669996, 0x69669999,
    0x69696666, 0x69696669, 0x69696696, 0x69696699, 0x69696966, 0x69696969, 0x69696996, 0x69696999, 0x69699666, 0x69699669, 0x69699696, 0x69699699, 0x69699966, 0x69699969, 0x69699996, 0x69699999,
    0x69966666, 0x69966669, 0x69966696, 0x69966699, 0x69966966, 0x69966969, 0x69966996, 0x69966999, 0x69969666, 0x69969669, 0x69969696, 0x69969699, 0x69969966, 0x69969969, 0x69969996, 0x69969999,
    0x69996666, 0x69996669, 0x69996696, 0x69996699, 0x69996966, 0x69996969, 0x69996996, 0x69996999, 0x69999666, 0x69999669, 0x69999696, 0x69999699, 0x69999966, 0x69999969, 0x69999996, 0x69999999,
    0x96666666, 0x96666669, 0x96666696, 0x96666699, 0x96666966, 0x96666969, 0x96666996, 0x96666999, 0x96669666, 0x96669669, 0x96669696, 0x96669699, 0x96669966, 0x96669969, 0x96669996, 0x96669999,
    0x96696666, 0x96696669, 0x96696696, 0x96696699, 0x96696966, 0x96696969, 0x96696996, 0x96696999, 0x96699666, 0x96699669, 0x96699696, 0x96699699, 0x96699966, 0x96699969, 0x96699996, 0x96699999,
    0x96966666, 0x96966669, 0x96966696, 0x96966699, 0x96966966, 0x96966969, 0x96966996, 0x96966999, 0x96969666, 0x96969669, 0x96969696, 0x96969699, 0x96969966, 0x96969969, 0x96969996, 0x96969999,
    0x96996666, 0x96996669, 0x96996696, 0x96996699, 0x96996966, 0x96996969, 0x96996996, 0x96996999, 0x96999666, 0x96999669, 0x96999696, 0x96999699, 0x96999966, 0x96999969, 0x96999996, 0x96999999,
    0x99666666, 0x99666669, 0x99666696, 0x99666699, 0x99666966, 0x99666969, 0x99666996, 0x99666999, 0x99669666, 0x99669669, 0x99669696, 0x99669699, 0x99669966, 0x99669969, 0x99669996, 0x99669999,
    0x99696666, 0x99696669, 0x99696696, 0x99696699, 0x99696966, 0x99696969, 0x99696996, 0x99696999, 0x99699666, 0x99699669, 0x99699696, 0x99699699, 0x99699966, 0x99699969, 0x99699996, 0x99699999,
    0x99966666, 0x99966669, 0x99966696, 0x99966699, 0x99966966, 0x99966969, 0x99966996, 0x99966999, 0x99969666, 0x99969669, 0x99969696, 0x99969699, 0x99969966, 0x99969969, 0x99969996, 0x99969999,
    0x99996666, 0x99996669, 0x99996696, 0x99996699, 0x99996966, 0x99996969, 0x99996996, 0x99996999, 0x99999666, 0x99999669, 0x99999696, 0x99999699, 0x99999966, 0x99999969, 0x99999996, 0x99999999,
};

// TX state
static PIO      pio_tx = pio0;
static uint     sm_tx  = 0;
static uint     dma_ch_tx;
static dma_channel_config dma_conf_tx;
static uint32_t time_nlp = 0;

// TX buffer: preamble(7) + SFD(1) + frame(up to 1518) + FCS(4) = 1530 max raw bytes
// Manchester encoded: each byte -> 1 uint32_t, plus TP_IDL
#define TX_RAW_BUF_SIZE     1530
#define TX_MANCH_BUF_SIZE   (TX_RAW_BUF_SIZE + 1)

static uint8_t  tx_raw_buf[TX_RAW_BUF_SIZE];
static uint32_t tx_manch_buf[TX_MANCH_BUF_SIZE];

// RX frame buffer for polling
static uint8_t  rx_poll_frame[ETH_RX_FRAME_BUF_SIZE];

// MAC address
static const uint8_t eth_mac[6] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};


static void tx_init(uint pin_tx)
{
    uint offset = pio_add_program(pio_tx, &ser_10base_t_program);
    ser_10base_t_program_init(pio_tx, sm_tx, offset, pin_tx);

    dma_ch_tx = dma_claim_unused_channel(true);
}


static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    (void)netif;
    uint32_t idx = 0;
    uint32_t i;

    // Preamble (7 bytes of 0x55)
    for (i = 0; i < 7; i++) {
        tx_raw_buf[idx++] = 0x55;
    }
    // SFD
    tx_raw_buf[idx++] = 0xD5;

    // Copy pbuf chain (Ethernet frame: dst MAC + src MAC + ethertype + payload)
    uint32_t frame_start = idx;
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        if (idx + q->len > TX_RAW_BUF_SIZE - 4) break;  // Leave room for FCS
        memcpy(&tx_raw_buf[idx], q->payload, q->len);
        idx += q->len;
    }
    uint32_t frame_len = idx - frame_start;

    // Pad to minimum Ethernet frame size (60 bytes, excluding FCS)
    while (frame_len < 60) {
        tx_raw_buf[idx++] = 0x00;
        frame_len++;
    }

    // Calculate FCS (CRC-32) using DMA sniffer
    dma_conf_tx = dma_channel_get_default_config(dma_ch_tx);
    channel_config_set_transfer_data_size(&dma_conf_tx, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_conf_tx, true);
    channel_config_set_write_increment(&dma_conf_tx, false);
    dma_channel_configure(
        dma_ch_tx, &dma_conf_tx,
        NULL,                       // Dummy destination
        &tx_raw_buf[frame_start],   // Source: frame data (after preamble+SFD)
        frame_len,                  // Number of bytes
        false                       // Don't start yet
    );
    dma_sniffer_enable(dma_ch_tx, 1, true);
    hw_set_bits(&dma_hw->sniff_ctrl,
               (DMA_SNIFF_CTRL_OUT_INV_BITS | DMA_SNIFF_CTRL_OUT_REV_BITS));
    dma_hw->sniff_data = 0xFFFFFFFF;
    dma_channel_set_read_addr(dma_ch_tx, &tx_raw_buf[frame_start], true);
    dma_channel_wait_for_finish_blocking(dma_ch_tx);
    uint32_t crc = dma_hw->sniff_data;

    // Append FCS
    tx_raw_buf[idx++] = (crc >>  0) & 0xFF;
    tx_raw_buf[idx++] = (crc >>  8) & 0xFF;
    tx_raw_buf[idx++] = (crc >> 16) & 0xFF;
    tx_raw_buf[idx++] = (crc >> 24) & 0xFF;

    // Manchester encode
    for (i = 0; i < idx; i++) {
        tx_manch_buf[i] = tbl_manchester[tx_raw_buf[i]];
    }
    // TP_IDL
    tx_manch_buf[idx] = 0x00000AAA;
    uint32_t manch_len = idx + 1;

    // DMA to PIO TX FIFO
    dma_conf_tx = dma_channel_get_default_config(dma_ch_tx);
    channel_config_set_dreq(&dma_conf_tx, pio_get_dreq(pio_tx, sm_tx, true));
    channel_config_set_transfer_data_size(&dma_conf_tx, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_conf_tx, true);
    channel_config_set_write_increment(&dma_conf_tx, false);
    dma_channel_configure(
        dma_ch_tx, &dma_conf_tx,
        &pio_tx->txf[sm_tx],   // Write to PIO TX FIFO
        tx_manch_buf,           // Read from Manchester buffer
        manch_len,              // Number of 32-bit words
        true                    // Start
    );
    dma_channel_wait_for_finish_blocking(dma_ch_tx);

    time_nlp = time_us_32();  // Reset NLP timer after any TX

    return ERR_OK;
}


bool eth_send_nlp(void)
{
    uint32_t time_now = time_us_32();
    if ((time_now - time_nlp) > ETH_NLP_INTERVAL_US) {
        time_nlp = time_now;
        // NLP pulse (100ns)
        ser_10base_t_tx_10b(pio_tx, sm_tx, 0x0000000A);
        return true;
    }
    return false;
}


err_t eth_netif_init(struct netif *netif)
{
    netif->name[0] = 'e';
    netif->name[1] = 't';
    netif->mtu = 1500;
    netif->hwaddr_len = 6;
    memcpy(netif->hwaddr, eth_mac, 6);
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    netif->output     = etharp_output;
    netif->linkoutput  = low_level_output;

    // Initialize TX (PIO0)
    tx_init(ETH_PIN_TXD);

    // Initialize RX (PIO1 + DMA)
    eth_rx_init(ETH_PIN_RXD);

    return ERR_OK;
}


void eth_netif_poll(struct netif *netif)
{
    uint16_t len = 0;

    if (!eth_rx_poll(rx_poll_frame, &len)) {
        return;
    }

    // Allocate pbuf and copy received frame
    struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (p == NULL) {
        return;
    }

    // Copy frame data into pbuf chain
    pbuf_take(p, rx_poll_frame, len);

    // Feed to lwIP - this handles ARP, ICMP, UDP routing
    if (netif->input(p, netif) != ERR_OK) {
        pbuf_free(p);
    }
}
