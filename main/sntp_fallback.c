#include "sntp_fallback.h"
#include "sdkconfig.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "lwip/ip_addr.h"

static const char *TAG = "SNTP";

#ifndef CONFIG_SNTP_UPDATE_INTERVAL
#define CONFIG_SNTP_UPDATE_INTERVAL 300
#endif
/* Stale after 3× the poll interval — tolerates two consecutive missed polls */
#define SNTP_STALE_US ((int64_t)CONFIG_SNTP_UPDATE_INTERVAL * 3 * 1000000LL)

/*
 * s_last_sync_us: esp_timer_get_time() at last successful sync, 0 = never.
 * s_ref_id:       upstream IPv4 in network byte order for NTP ref_id field.
 * Written from the SNTP sync callback (lwIP task), read from NTP/OLED tasks.
 * int64_t writes are not atomic on 32-bit Xtensa but staleness is
 * non-critical (a torn read produces a slightly wrong timeout, not a crash).
 */
static volatile int64_t  s_last_sync_us = 0;
static volatile uint32_t s_ref_id = 0;   /* upstream IPv4, network byte order */

/* ------------------------------------------------------------------ */
/*  Periodic status log (every 60 s)                                    */
/* ------------------------------------------------------------------ */

static void sntp_status_log_cb(void *arg)
{
    int64_t last = s_last_sync_us;
    if (last == 0) {
        ESP_LOGI(TAG, "status: no sync yet  (poll=%ds)",
                 CONFIG_SNTP_UPDATE_INTERVAL);
        return;
    }
    int64_t age_s   = (esp_timer_get_time() - last) / 1000000LL;
    int64_t stale_s = (int64_t)CONFIG_SNTP_UPDATE_INTERVAL * 3;
    ESP_LOGI(TAG, "status: last sync %llds ago  %s  (poll=%ds stale=%llds)",
             (long long)age_s,
             age_s <= stale_s ? "OK" : "STALE",
             CONFIG_SNTP_UPDATE_INTERVAL,
             (long long)stale_s);
}

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

    s_last_sync_us = esp_timer_get_time();

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
    /* Override the lwIP default poll interval at runtime so the setting
     * is always applied regardless of what sdkconfig holds. */
    esp_sntp_set_sync_interval((uint32_t)CONFIG_SNTP_UPDATE_INTERVAL * 1000U);

    esp_timer_handle_t status_timer;
    const esp_timer_create_args_t timer_args = {
        .callback = sntp_status_log_cb,
        .name     = "sntp_log",
    };
    if (esp_timer_create(&timer_args, &status_timer) == ESP_OK)
        esp_timer_start_periodic(status_timer, 60ULL * 1000000ULL);  /* 60 s */

    ESP_LOGI(TAG, "SNTP client started → %s  poll=%ds  stale=%ds",
             CONFIG_NTP_FALLBACK_SERVER,
             CONFIG_SNTP_UPDATE_INTERVAL,
             CONFIG_SNTP_UPDATE_INTERVAL * 3);
}

bool sntp_fallback_is_synced(void)
{
    int64_t last = s_last_sync_us;
    if (last == 0) return false;
    return (esp_timer_get_time() - last) <= SNTP_STALE_US;
}

uint32_t sntp_fallback_ref_id(void)
{
    return s_ref_id;
}

int64_t sntp_fallback_last_sync_us(void)
{
    return s_last_sync_us;
}
