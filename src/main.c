/********************************************************
* Title    : Pico-10BASE-T Sample
* Date     : 2022/08/22
* Note     : GP14 TX -
             GP15 TX +
             GP13 RX  (comparator output)
* Design   : kingyo
********************************************************/
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "udp.h"
#include "eth_rx.h"
#include <stdio.h>

#define HW_PINNUM_TXD       (14)        // 10BASE-T TX (ISL3177E DI) Pin.
#define HW_PINNUM_RXD       (13)        // 10BASE-T RX (ISL3177E RO) Pin.
#define HW_PINNUM_LED0      (25)        // Pico onboard LED
#define DEF_TX_INTERVAL_US  (200000)    // Dummy Data TX interval


static struct repeating_timer timer;

static volatile uint32_t rx_edge_count = 0;

static void rx_gpio_callback(uint gpio, uint32_t events) {
    if (gpio == HW_PINNUM_RXD) rx_edge_count++;
}

// Timer interrupt (L-tika)
static bool repeating_timer_callback(struct repeating_timer *t) {
    static bool led0_state = true;

    gpio_put(HW_PINNUM_LED0, led0_state);
    led0_state = !led0_state;

    return true;
}


int main() {
    uint32_t tx_buf_udp[DEF_UDP_BUF_SIZE+1] = {0};
    uint8_t udp_payload[DEF_UDP_PAYLOAD_SIZE] = {0};

    uint32_t lp_cnt = 0;
    uint32_t time_now = 0;
    uint32_t time_tx = 0;

    // Setting the Clock frequency divider to a multiple of 20 MHz,
    // allows the PIO divider to operate at integer multiples (Reduce jitter).
    set_sys_clock_khz(120000, true);

    stdio_init_all();
    udp_init(HW_PINNUM_TXD);

    // Onboard LED tikatika~
    gpio_init(HW_PINNUM_LED0);
    gpio_set_dir(HW_PINNUM_LED0, GPIO_OUT);
    add_repeating_timer_ms(-500, repeating_timer_callback, NULL, &timer);

    // RX pin: count edges from comparator output (kept for sanity check during Phase 2a)
    gpio_init(HW_PINNUM_RXD);
    gpio_set_dir(HW_PINNUM_RXD, GPIO_IN);
    gpio_set_irq_enabled_with_callback(HW_PINNUM_RXD,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &rx_gpio_callback);

    // RX sampler: PIO0 SM1 (SM0 is used by TX) drives DMA into eth_rx_buf.
    eth_rx_init(pio0, 1, HW_PINNUM_RXD);


    // Wait for Link up....
    for (uint32_t i = 0; i < 200;) {
        if (udp_send_nlp()) i++;
    }

    uint32_t time_dump = time_us_32();

    // Main loop
    // Send packets every about 200ms.
    while (1) {
        time_now = time_us_32();

        // Sending NLP Puls
        udp_send_nlp();

        // Sending UDP packets
        if ((time_now - time_tx) > DEF_TX_INTERVAL_US) {
            time_tx = time_now;
            sprintf(udp_payload, "Hello World!! Raspico 10BASE-T !! lp_cnt:%d", lp_cnt++);
            udp_packet_gen_10base(tx_buf_udp, udp_payload);
            udp_send_packet(tx_buf_udp);
        }

        // Phase 2: every 1 s, find a frame, decode it, verify CRC.
        if ((time_now - time_dump) > 1000000) {
            time_dump = time_now;
            eth_rx_decode_frame();
        }
    }

}
