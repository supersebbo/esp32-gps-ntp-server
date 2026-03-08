#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Start the SNTP client pointing at CONFIG_NTP_FALLBACK_SERVER.
 *
 * The client polls continuously.  When it first syncs it calls
 * settimeofday() so that gettimeofday() returns correct UTC.
 * GPS time is served in preference whenever gps_is_synced() is true;
 * this module is only consulted when GPS is unavailable.
 */
void sntp_fallback_init(void);

/**
 * Returns true once the SNTP client has completed at least one sync.
 */
bool sntp_fallback_is_synced(void);

/**
 * Returns the 32-bit IPv4 address of the upstream NTP server in
 * network byte order, suitable for direct use as the NTP ref_id
 * field in a stratum-2 response.
 * Returns 0 if SNTP has not yet resolved / synced.
 */
uint32_t sntp_fallback_ref_id(void);
