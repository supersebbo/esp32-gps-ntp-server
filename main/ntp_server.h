#pragma once

#include <stdint.h>

/**
 * Start the NTP server task.
 *
 * Listens on UDP port 123 using POSIX sockets (lwIP).
 * Responds to NTP v3/v4 client requests with GPS-derived timestamps.
 * Operates as stratum 1 when GPS is synced; stratum 16 otherwise.
 */
void ntp_server_start(void);

/**
 * Return the total number of NTP requests served since boot.
 */
uint32_t ntp_get_request_count(void);
