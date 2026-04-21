#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

// Bare-metal (no RTOS)
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define SYS_LIGHTWEIGHT_PROT        0

// Memory
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4096
#define MEMP_NUM_PBUF               16
#define MEMP_NUM_UDP_PCB            4
#define MEMP_NUM_TCP_PCB            0
#define MEMP_NUM_TCP_PCB_LISTEN     0
#define MEMP_NUM_ARP_QUEUE          8
#define PBUF_POOL_SIZE              16
#define PBUF_POOL_BUFSIZE           1536

// Protocols
#define LWIP_ARP                    1
#define LWIP_IPV4                   1
#define LWIP_ICMP                   1
#define LWIP_UDP                    1
#define LWIP_TCP                    0
#define LWIP_RAW                    0
#define LWIP_DHCP                   0
#define LWIP_AUTOIP                 0
#define LWIP_IPV6                   0

// Misc
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define ETHARP_TRUST_IP_SRC         1
#define ETH_PAD_SIZE                0
#define LWIP_STATS                  0
#define LWIP_PROVIDE_ERRNO          1
#define LWIP_CHKSUM_ALGORITHM       3

#endif // __LWIPOPTS_H__
