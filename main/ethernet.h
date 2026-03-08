#pragma once

/**
 * Initialise the SPI Ethernet interface, attach it to the lwIP TCP/IP
 * stack, and wait up to 15 s for a DHCP lease.
 *
 * Requires esp_netif_init() and esp_event_loop_create_default() to
 * have been called first (done unconditionally in app_main).
 *
 * The chip variant and all GPIO/SPI parameters are taken from
 * Kconfig (CONFIG_ETH_SPI_*).
 */
void ethernet_init(void);
