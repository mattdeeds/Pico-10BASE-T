/********************************************************
 * Title    : Pico-10BASE-T Bidirectional
 * Date     : 2026/04/08
 * Note     : Full bidirectional 10BASE-T with lwIP stack
 *            TX: GP10 (TX-), GP11 (TX+)
 *            RX: GP14 (comparator output)
 *            Supports: ARP, ICMP (ping), UDP
 * Design   : kingyo (original TX), extended for RX + lwIP
 ********************************************************/
#include "pico/stdlib.h"
#include "pico/async_context_poll.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/udp.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"

#include "eth_netif.h"
#include "eth_rx.h"

#include <stdio.h>
#include <string.h>

#define HW_PINNUM_LED0      (25)
#define DEF_TX_INTERVAL_US  (200000)

static struct netif netif;
static async_context_poll_t poll_context;
static struct repeating_timer led_timer;

static bool led_timer_callback(struct repeating_timer *t) {
    static bool led_state = true;
    gpio_put(HW_PINNUM_LED0, led_state);
    led_state = !led_state;
    return true;
}

// lwIP requires this for NO_SYS mode
static bool lwip_nosys_init_simple(async_context_t *context) {
    lwip_init();
    return true;
}

int main() {
    uint32_t lp_cnt = 0;
    uint32_t time_tx = 0;
    uint32_t time_dbg = 0;

    // 120 MHz: clean 20 MHz PIO division for TX, clean 40 MHz for RX
    set_sys_clock_khz(120000, true);
    stdio_init_all();

    // LED blink
    gpio_init(HW_PINNUM_LED0);
    gpio_set_dir(HW_PINNUM_LED0, GPIO_OUT);
    add_repeating_timer_ms(-500, led_timer_callback, NULL, &led_timer);

    // Initialize lwIP
    lwip_init();

    // Set up network interface with static IP
    ip4_addr_t ipaddr, netmask, gateway;
    IP4_ADDR(&ipaddr,  192, 168, 37, 24);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gateway, 192, 168, 37, 1);

    netif_add(&netif, &ipaddr, &netmask, &gateway,
              NULL, eth_netif_init, ethernet_input);
    netif_set_default(&netif);
    netif_set_up(&netif);

    // GP15: test signal for comparator chain debug
    // Wire GP15 (physical pin 20 - bottom-left) to comparator +IN (pin 3)
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);

    // Wait for link up - send NLPs
    for (uint32_t i = 0; i < 200;) {
        if (eth_send_nlp()) i++;
    }

    // Set up UDP for "Hello World" demo (broadcast so it works without ARP/RX)
    struct udp_pcb *upcb = udp_new();
    udp_bind(upcb, &ipaddr, 1234);

    // Main loop
    while (1) {
        uint32_t time_now = time_us_32();

        // Poll for received frames and feed to lwIP
        eth_netif_poll(&netif);

        // Process lwIP timers (ARP aging, etc.)
        sys_check_timeouts();

        // Send NLP pulses to maintain link
        eth_send_nlp();

        // GP15 slow toggle: 1 Hz (500ms HIGH, 500ms LOW)
        gpio_put(15, (time_now / 500000) & 1);

        // Continuous RX scan + periodic dump
        eth_rx_debug_scan();
        if ((time_now - time_dbg) > 100000) {
            time_dbg = time_now;
            eth_rx_debug_t dbg;
            eth_rx_debug_get(&dbg);
            uint32_t samples = dbg.words_scanned * 32;
            uint32_t ones_pct = samples ? (dbg.ones * 100 / samples) : 0;
            printf("RX dma_w=%u scanned_w=%u ones=%u(%u%%) trans=%u max_run=%u gp15=%d\n",
                   (unsigned)dbg.dma_write_idx,
                   (unsigned)dbg.words_scanned,
                   (unsigned)dbg.ones,
                   (unsigned)ones_pct,
                   (unsigned)dbg.transitions,
                   (unsigned)dbg.max_run,
                   gpio_get(15));
            eth_rx_debug_reset();
        }

        // Periodic UDP send (demo)
        if ((time_now - time_tx) > DEF_TX_INTERVAL_US) {
            time_tx = time_now;

            char payload[128];
            int payload_len = snprintf(payload, sizeof(payload),
                "Hello World!! Raspico 10BASE-T !! lp_cnt:%d", lp_cnt++);

            struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, payload_len, PBUF_RAM);
            if (p != NULL) {
                memcpy(p->payload, payload, payload_len);
                ip4_addr_t bcast;
                IP4_ADDR(&bcast, 255, 255, 255, 255);
                udp_sendto(upcb, p, &bcast, 1234);
                pbuf_free(p);
            }
        }
    }
}
