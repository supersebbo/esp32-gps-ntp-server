#include "ntp_server.h"
#include "gps.h"
#include "sntp_fallback.h"
#include "rtc.h"

#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "NTP";

static volatile uint32_t s_req_count = 0;

/* ------------------------------------------------------------------ */
/*  NTP constants                                                       */
/* ------------------------------------------------------------------ */

#define NTP_PORT          123
#define NTP_PACKET_SIZE   48

/*
 * Seconds between the NTP epoch (1 Jan 1900) and
 * the Unix epoch (1 Jan 1970) = 70 years.
 */
#define NTP_UNIX_OFFSET   2208988800UL

/* Reference ID for a GPS-disciplined stratum-1 clock */
#define NTP_REFID_GPS     0x47505300UL   /* "GPS\0" */
/* Reference ID for a battery-backed RTC (stratum 12) */
#define NTP_REFID_RTC     0x52544300UL   /* "RTC\0" */

/* NTP mode field values */
#define NTP_MODE_CLIENT   3
#define NTP_MODE_SERVER   4

/* LI (leap-second indicator) values */
#define NTP_LI_NONE       0   /* no warning                */
#define NTP_LI_UNSYNC     3   /* clock not synchronised    */

/* ------------------------------------------------------------------ */
/*  NTP packet layout (RFC 5905, §7.3)                                  */
/* ------------------------------------------------------------------ */

typedef struct __attribute__((packed)) {
    uint8_t  li_vn_mode;        /* [7:6]=LI, [5:3]=VN, [2:0]=mode      */
    uint8_t  stratum;
    uint8_t  poll;              /* log2 of max poll interval, seconds   */
    int8_t   precision;         /* log2 of clock precision, seconds     */
    uint32_t root_delay;        /* NTP short: 16-bit seconds + fraction */
    uint32_t root_dispersion;   /* NTP short: 16-bit seconds + fraction */
    uint32_t ref_id;
    uint32_t ref_ts_secs;       /* reference timestamp                  */
    uint32_t ref_ts_frac;
    uint32_t orig_ts_secs;      /* originate timestamp (echo of client) */
    uint32_t orig_ts_frac;
    uint32_t rx_ts_secs;        /* receive timestamp                    */
    uint32_t rx_ts_frac;
    uint32_t tx_ts_secs;        /* transmit timestamp (set last)        */
    uint32_t tx_ts_frac;
} ntp_packet_t;

_Static_assert(sizeof(ntp_packet_t) == NTP_PACKET_SIZE,
               "NTP packet struct size mismatch");

/* ------------------------------------------------------------------ */
/*  Timestamp conversion helpers                                        */
/* ------------------------------------------------------------------ */

/* Convert Unix time_t to 32-bit NTP seconds field */
static inline uint32_t unix_to_ntp_sec(time_t t)
{
    return (uint32_t)((uint32_t)t + NTP_UNIX_OFFSET);
}

/*
 * Convert microseconds (0–999999) to the 32-bit NTP fraction field.
 * NTP fraction = usec * 2^32 / 1e6
 * Use 64-bit arithmetic to avoid overflow.
 */
static inline uint32_t usec_to_ntp_frac(suseconds_t usec)
{
    return (uint32_t)(((uint64_t)usec * 4294967296ULL) / 1000000ULL);
}

/* Fill an NTP timestamp pair from a struct timeval.
 * Uses memcpy to write into packed struct members, avoiding the
 * -Waddress-of-packed-member unaligned pointer warning on Xtensa. */
static inline void fill_ntp_ts(void *secs, void *frac,
                                const struct timeval *tv)
{
    uint32_t s = htonl(unix_to_ntp_sec(tv->tv_sec));
    uint32_t f = htonl(usec_to_ntp_frac(tv->tv_usec));
    memcpy(secs, &s, sizeof(s));
    memcpy(frac, &f, sizeof(f));
}

/* ------------------------------------------------------------------ */
/*  Server task                                                         */
/* ------------------------------------------------------------------ */

static void ntp_server_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in srv = {
        .sin_family      = AF_INET,
        .sin_port        = htons(NTP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };

    if (bind(sock, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "NTP server listening on UDP port %d", NTP_PORT);

    ntp_packet_t req;
    ntp_packet_t resp;
    struct sockaddr_in client;
    socklen_t          client_len = sizeof(client);

    while (1) {
        int len = recvfrom(sock, &req, sizeof(req), 0,
                           (struct sockaddr *)&client, &client_len);

        if (len < 0) {
            ESP_LOGW(TAG, "recvfrom error: %d", errno);
            continue;
        }
        if (len < NTP_PACKET_SIZE) {
            ESP_LOGD(TAG, "Short packet (%d bytes) from %s, ignored",
                     len, inet_ntoa(client.sin_addr));
            continue;
        }

        /* ---------------------------------------------------------- */
        /*  Capture receive timestamp immediately after recvfrom       */
        /* ---------------------------------------------------------- */
        struct timeval rx_tv;
        gps_get_time(&rx_tv);   /* GPS-interpolated, or system clock */

        /* ---------------------------------------------------------- */
        /*  Determine sync source (GPS beats SNTP)                     */
        /* ---------------------------------------------------------- */
        bool gps_ok  = gps_is_synced();
        bool sntp_ok = !gps_ok && sntp_fallback_is_synced();
        bool rtc_ok  = !gps_ok && !sntp_ok && ds3231_is_valid();

        /* ---------------------------------------------------------- */
        /*  Validate client mode (must be 1=symmetric-active or        */
        /*  3=client; reject everything else including broadcasts)     */
        /* ---------------------------------------------------------- */
        uint8_t client_mode = req.li_vn_mode & 0x07;
        if (client_mode != NTP_MODE_CLIENT && client_mode != 1) {
            ESP_LOGD(TAG, "Ignoring packet with mode %d", client_mode);
            continue;
        }

        uint8_t client_vn = (req.li_vn_mode >> 3) & 0x07;
        if (client_vn < 1 || client_vn > 4) {
            ESP_LOGD(TAG, "Ignoring packet with unsupported NTP version %d",
                     client_vn);
            continue;
        }

        /* ---------------------------------------------------------- */
        /*  Build response                                             */
        /* ---------------------------------------------------------- */
        memset(&resp, 0, sizeof(resp));

        bool any_sync = gps_ok || sntp_ok || rtc_ok;
        uint8_t li    = any_sync ? NTP_LI_NONE : NTP_LI_UNSYNC;
        resp.li_vn_mode = (uint8_t)((li << 6) | (client_vn << 3) | NTP_MODE_SERVER);

        resp.poll = req.poll;

        if (gps_ok) {
            /*
             * Stratum 1 — GPS primary reference.
             * NTP short format: 1 unit = 1/65536 s ≈ 15.26 µs.
             *
             * With PPS + crystal drift correction:
             *   precision    = -20  (2^-20 s ≈ 1 µs, matches esp_timer res.)
             *   root_disp    =   5  (5/65536 s ≈ 76 µs, covers ISR jitter)
             *
             * NMEA-only (no PPS):
             *   precision    = -18  (2^-18 s ≈ 4 µs)
             *   root_disp    =  16  (16/65536 s ≈ 244 µs)
             */
            resp.stratum    = 1;
            resp.root_delay = 0;
            resp.ref_id     = htonl(NTP_REFID_GPS);
#ifdef CONFIG_GPS_PPS_ENABLED
            resp.precision       = -20;
            resp.root_dispersion = htonl(5);
#else
            resp.precision       = -18;
            resp.root_dispersion = htonl(16);
#endif

        } else if (sntp_ok) {
            /*
             * Stratum 2 — disciplined by an upstream NTP server.
             * Precision -7 ≈ 8 ms (internet SNTP typical accuracy).
             * Root delay and dispersion are set conservatively at 100 ms
             * (NTP short format: 0x00001999 ≈ 100 ms).
             * ref_id is the upstream server's IPv4 address (network byte
             * order from lwIP — no htonl needed).
             */
            resp.stratum         = 2;
            resp.precision       = -7;
            resp.root_delay      = htonl(0x00001999);   /* ~100 ms */
            resp.root_dispersion = htonl(0x00001999);   /* ~100 ms */
            resp.ref_id          = sntp_fallback_ref_id();

        } else if (rtc_ok) {
            /*
             * Stratum 12 — battery-backed DS3231 RTC.
             * DS3231 is ±2 ppm (≈ 175 ms/day drift).  Conservative values:
             *   precision    = -10  (2^-10 s ≈ 1 ms)
             *   root_disp    = 656  (656/65536 ≈ 10 ms)
             */
            resp.stratum         = 12;
            resp.precision       = -10;
            resp.root_delay      = 0;
            resp.root_dispersion = htonl(656);
            resp.ref_id          = htonl(NTP_REFID_RTC);

        } else {
            /* Stratum 16 = unsynchronised; clients should discard */
            resp.stratum         = 16;
            resp.precision       = -7;
            resp.root_delay      = 0;
            resp.root_dispersion = 0;
            resp.ref_id          = 0;
        }

        /*
         * Reference timestamp: time when the clock was last corrected
         * (RFC 5905 §7.3).  Use the exact PPS/NMEA fix timestamp when
         * GPS is the source; fall back to current time otherwise.
         */
        {
            struct timeval ref_tv = rx_tv;
            if (gps_ok) gps_get_last_fix_time(&ref_tv);
            fill_ntp_ts(&resp.ref_ts_secs, &resp.ref_ts_frac, &ref_tv);
        }

        /* Originate timestamp: echo the client's transmit timestamp */
        resp.orig_ts_secs = req.tx_ts_secs;
        resp.orig_ts_frac = req.tx_ts_frac;

        /* Receive timestamp */
        fill_ntp_ts(&resp.rx_ts_secs, &resp.rx_ts_frac, &rx_tv);

        /*
         * Transmit timestamp: captured as late as possible to minimise
         * asymmetric delay.  gps_get_time() is cheap (mutex + arithmetic).
         */
        struct timeval tx_tv;
        gps_get_time(&tx_tv);
        fill_ntp_ts(&resp.tx_ts_secs, &resp.tx_ts_frac, &tx_tv);

        sendto(sock, &resp, sizeof(resp), 0,
               (struct sockaddr *)&client, client_len);
        s_req_count++;

        ESP_LOGD(TAG, "Served %s  stratum=%d  src=%s",
                 inet_ntoa(client.sin_addr), resp.stratum,
                 gps_ok ? "GPS" : sntp_ok ? "SNTP" : rtc_ok ? "RTC" : "UNSYNC");
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void ntp_server_start(void)
{
    xTaskCreate(ntp_server_task, "ntp_server", 4096, NULL, 5, NULL);
}

uint32_t ntp_get_request_count(void)
{
    return s_req_count;
}
