#ifndef __ETH_NETIF_H__
#define __ETH_NETIF_H__

#include "lwip/netif.h"

// Pin assignments
#define ETH_PIN_TXD     10      // TX- (TX+ is pin+1 = 11)
#define ETH_PIN_RXD     14      // Comparator output (single-ended RX)

// NLP: Normal Link Pulse interval (16ms +/- 8ms)
#define ETH_NLP_INTERVAL_US     16000

err_t eth_netif_init(struct netif *netif);
void  eth_netif_poll(struct netif *netif);
bool  eth_send_nlp(void);

#endif // __ETH_NETIF_H__
