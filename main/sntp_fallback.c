#include "sntp_fallback.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_sntp.h"
#include "lwip/ip_addr.h"
#include "sdkconfig.h"

static const char *TAG = "SNTP";

/*
 * Both fields are written once from the SNTP sync callback (lwIP task)
 * and read from the NTP server task.  On ESP32 (32-bit Xtensa), aligned
 * 32-bit writes/reads are atomic, so volatile without a mutex is safe
 * for these narrow types.
 */
static volatile bool     s_synced = false;
static volatile uint32_t s_ref_id = 0;   /* upstream IPv4, network byte order */

/* ------------------------------------------------------------------ */
/*  SNTP sync callback                                                  */
/* ------------------------------------------------------------------ */

static void on_sntp_sync(struct timeval *tv)
{
    /*
     * Capture the upstream server's IPv4 address for use as the NTP
     * reference identifier in stratum-2 responses.
     * lwIP stores ip4_addr_t.addr in network byte order; using it
     * directly as the NTP ref_id produces the correct on-wire bytes
     * without an extra htonl().
     */
    /* sntp_getserver() returns const ip_addr_t * (pointer, not value) */
    const ip_addr_t *srv = esp_sntp_getserver(0);
    if (srv != NULL && IP_IS_V4(srv)) {
        s_ref_id = ip_2_ip4(srv)->addr;
    }

    s_synced = true;

    struct tm t;
    gmtime_r(&tv->tv_sec, &t);
    ESP_LOGI(TAG, "Synced from %s → %04d-%02d-%02d %02d:%02d:%02d UTC",
             esp_sntp_getservername(0),
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void sntp_fallback_init(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, CONFIG_NTP_FALLBACK_SERVER);
    esp_sntp_set_time_sync_notification_cb(on_sntp_sync);
    esp_sntp_init();

    ESP_LOGI(TAG, "SNTP client started → %s", CONFIG_NTP_FALLBACK_SERVER);
}

bool sntp_fallback_is_synced(void)
{
    return s_synced;
}

uint32_t sntp_fallback_ref_id(void)
{
    return s_ref_id;
}
