#include "gps.h"

#include <sys/time.h>    /* struct timeval, gettimeofday — used by stubs */
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "GPS";

#ifdef CONFIG_GPS_ENABLED

#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* ------------------------------------------------------------------ */
/*  timegm() replacement                                                */
/*  timegm is a non-standard POSIX extension not exposed by newlib.    */
/*  This implements the same conversion: UTC struct tm → time_t.       */
/* ------------------------------------------------------------------ */

static time_t utc_mktime(const struct tm *t)
{
    /*
     * Days from the Unix epoch (1970-01-01) to the start of year Y,
     * counting every 4th year as leap, removing century years, then
     * adding back 400-year cycles (proleptic Gregorian calendar).
     *
     * Let y = year - 1970.  Then:
     *   leap days added   = (y+1)/4
     *   century exclusions= (y+69)/100   (removes 2000 exclusion...)
     *   400-year restores = (y+369)/400  (...then adds it back)
     */
    int year = t->tm_year + 1900;
    int y    = year - 1970;

    time_t days = (time_t)y * 365
                + (time_t)(y + 1)   / 4
                - (time_t)(y + 69)  / 100
                + (time_t)(y + 369) / 400;

    /* Cumulative days at the start of each month (non-leap year) */
    static const int mdays[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
    days += mdays[t->tm_mon];

    /* Add leap-day for this year if we are past February */
    bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    if (t->tm_mon > 1 && leap) days++;

    days += t->tm_mday - 1;

    return days * 86400
         + (time_t)t->tm_hour * 3600
         + (time_t)t->tm_min  * 60
         + (time_t)t->tm_sec;
}

/* ------------------------------------------------------------------ */
/*  Configuration                                                       */
/* ------------------------------------------------------------------ */

#define GPS_UART_NUM    ((uart_port_t)CONFIG_GPS_UART_NUM)
#define GPS_RX_PIN      CONFIG_GPS_RX_PIN
#define GPS_TX_PIN      CONFIG_GPS_TX_PIN
#define GPS_BAUD_RATE   CONFIG_GPS_BAUD_RATE

#define UART_BUF_SIZE   2048
#define NMEA_MAX_LEN    128

/*
 * Kconfig bool options emit '#define CONFIG_FOO 1' when enabled and
 * nothing when disabled.  Guard them so they can be used as C integers.
 */
#ifndef CONFIG_GPS_GNSS_GLONASS
#  define CONFIG_GPS_GNSS_GLONASS 0
#endif
#ifndef CONFIG_GPS_GNSS_GALILEO
#  define CONFIG_GPS_GNSS_GALILEO 0
#endif
#ifndef CONFIG_GPS_GNSS_BEIDOU
#  define CONFIG_GPS_GNSS_BEIDOU  0
#endif
#ifndef CONFIG_GPS_GNSS_SBAS
#  define CONFIG_GPS_GNSS_SBAS    0
#endif

/* ------------------------------------------------------------------ */
/*  UBX binary protocol helpers                                         */
/* ------------------------------------------------------------------ */

/*
 * Fletcher-8 checksum over bytes [class..last_payload_byte].
 * (Sync chars 0xB5 0x62 are excluded from the checksum.)
 */
static void ubx_cksum(const uint8_t *data, int len,
                      uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (int i = 0; i < len; i++) { a += data[i]; b += a; }
    *ck_a = a;
    *ck_b = b;
}

/*
 * Wait for a UBX-ACK-ACK (0x05 0x01) or UBX-ACK-NAK (0x05 0x00) in
 * response to a CFG-GNSS (class=0x06, id=0x3E) message.
 *
 * Returns:  1 = ACK received
 *           0 = NAK received
 *          -1 = timeout
 *
 * The byte-stream state machine advances through the fixed 10-byte ACK
 * frame layout.  Any unexpected byte resets to state 0 (or to state 1
 * if the byte is the UBX sync-1 char 0xB5), so interleaved NMEA text
 * is harmlessly skipped.
 */
static int ubx_wait_ack(int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;
    int state = 0;
    int ack_id = -1;   /* filled once we see id byte (0x01 or 0x00) */

    while (esp_timer_get_time() < deadline) {
        uint8_t b;
        int n = uart_read_bytes(GPS_UART_NUM, &b, 1, pdMS_TO_TICKS(10));
        if (n <= 0) continue;

        switch (state) {
        case 0: state = (b == 0xB5) ? 1 : 0;               break;
        case 1: state = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0); break;
        case 2: state = (b == 0x05) ? 3 : (b == 0xB5 ? 1 : 0); break;
        case 3:
            if (b == 0x01 || b == 0x00) { ack_id = b; state = 4; }
            else state = (b == 0xB5) ? 1 : 0;
            break;
        case 4: state = (b == 0x02) ? 5 : (b == 0xB5 ? 1 : 0); break;
        case 5: state = (b == 0x00) ? 6 : (b == 0xB5 ? 1 : 0); break;
        case 6: state = (b == 0x06) ? 7 : (b == 0xB5 ? 1 : 0); break;
        case 7:
            if (b == 0x3E) return (ack_id == 0x01) ? 1 : 0;
            state = (b == 0xB5) ? 1 : 0;
            break;
        }
    }
    return -1;   /* timeout */
}

/*
 * Send UBX-CFG-GNSS (class 0x06, id 0x3E) to configure which GNSS
 * constellations the NEO-M8 should track.
 *
 * GPS is always enabled (mandatory).  The other constellations are
 * controlled by Kconfig booleans GPS_GNSS_GLONASS / GALILEO / BEIDOU /
 * SBAS.  The NEO-M8 enforces a maximum of 3 simultaneous constellations
 * (GPS + up to 2 others); SBAS is exempt from that limit.
 *
 * Frame layout: sync(2) + class+id+len(4) + payload(44) + cksum(2) = 52 B
 *
 * sigCfgMask 0x01 = L1C/A (GPS/SBAS/GLONASS L1OF / Galileo E1 / BeiDou B1I).
 * Flags (32-bit LE): bits[0]=enable, bits[16..20]=sigCfgMask.
 */
static void ubx_configure_gnss(void)
{
#define UBX_GNSS_FRAME_LEN  52   /* 2 sync + 4 hdr + 44 payload + 2 cksum */
#define UBX_GNSS_N_BLOCKS    5

    /* Helper macro: 8-byte constellation block */
#define GNSS_BLOCK(id, res, max, en) \
        (id), (res), (max), 0x00,    \
        (en) ? 0x01 : 0x00, 0x00, 0x01, 0x00

    uint8_t frame[UBX_GNSS_FRAME_LEN] = {
        0xB5, 0x62,           /* sync */
        0x06, 0x3E,           /* class, id */
        0x2C, 0x00,           /* payload length = 44 (LE) */
        /* payload header */
        0x00,                 /* msgVer */
        0x00,                 /* numTrkChHw (0 = query not needed) */
        0xFF,                 /* numTrkChUse (0xFF = use all) */
        UBX_GNSS_N_BLOCKS,   /* numConfigBlocks */
        /* constellation blocks (gnssId, resTrkCh, maxTrkCh, reserved,
           flags[0..3] LE: bit0=enable, bits[16:20]=sigCfgMask) */
        GNSS_BLOCK(0, 8, 16, 1),                        /* GPS    — always on */
        GNSS_BLOCK(1, 1,  3, CONFIG_GPS_GNSS_SBAS),     /* SBAS */
        GNSS_BLOCK(2, 4,  8, CONFIG_GPS_GNSS_GALILEO),  /* Galileo */
        GNSS_BLOCK(3, 8, 16, CONFIG_GPS_GNSS_BEIDOU),   /* BeiDou */
        GNSS_BLOCK(6, 8, 14, CONFIG_GPS_GNSS_GLONASS),  /* GLONASS */
        /* checksum placeholder */
        0x00, 0x00,
    };

#undef GNSS_BLOCK

    /* Checksum covers bytes[2..49] (class through last payload byte) */
    ubx_cksum(&frame[2], UBX_GNSS_FRAME_LEN - 4,
              &frame[UBX_GNSS_FRAME_LEN - 2],
              &frame[UBX_GNSS_FRAME_LEN - 1]);

    uart_write_bytes(GPS_UART_NUM, frame, UBX_GNSS_FRAME_LEN);

    int result = ubx_wait_ack(1500);
    if (result == 1) {
        ESP_LOGI(TAG, "UBX-CFG-GNSS ACK — GPS%s%s%s%s",
                 CONFIG_GPS_GNSS_GLONASS  ? "+GLONASS"  : "",
                 CONFIG_GPS_GNSS_GALILEO  ? "+Galileo"  : "",
                 CONFIG_GPS_GNSS_BEIDOU   ? "+BeiDou"   : "",
                 CONFIG_GPS_GNSS_SBAS     ? "+SBAS"     : "");
        vTaskDelay(pdMS_TO_TICKS(500));   /* GNSS engine restart */
    } else if (result == 0) {
        ESP_LOGW(TAG, "UBX-CFG-GNSS NAK — module rejected constellation config "
                      "(too many simultaneous constellations?)");
    } else {
        ESP_LOGW(TAG, "UBX-CFG-GNSS timeout — no ACK received");
    }

#undef UBX_GNSS_FRAME_LEN
#undef UBX_GNSS_N_BLOCKS
}

/* ------------------------------------------------------------------ */
/*  State                                                               */
/* ------------------------------------------------------------------ */

typedef struct {
    struct timeval    tv;         /* GPS time at reference point            */
    int64_t           esp_us;     /* esp_timer_get_time() at ref point       */
    bool              valid;      /* Has at least one valid fix been seen?   */
    int32_t           drift_ppb;  /* esp_timer freq error; >0 = runs fast   */
    int32_t           sat_count;  /* satellites in use from $xxGGA; -1=unknown */
    char              fix_talker[3]; /* 2-char NMEA talker of last RMC fix, e.g. "GN" */
    SemaphoreHandle_t mutex;
} gps_state_t;

static gps_state_t s_gps;

#ifdef CONFIG_GPS_PPS_ENABLED
/* Written from ISR, read from GPS task — declared volatile */
static volatile int64_t s_pps_esp_us      = 0;
static volatile int64_t s_pps_esp_us_prev = 0;

static void IRAM_ATTR pps_isr_handler(void *arg)
{
    s_pps_esp_us_prev = s_pps_esp_us;
    s_pps_esp_us      = esp_timer_get_time();
}
#endif

/* ------------------------------------------------------------------ */
/*  NMEA helpers                                                        */
/* ------------------------------------------------------------------ */

/* Parse "HHMMSS.sss" → h/m/s/ms.  Returns false on bad input. */
static bool parse_time_field(const char *f,
                              int *h, int *m, int *s, int *ms)
{
    if (!f || strlen(f) < 6) return false;
    *h  = (f[0] - '0') * 10 + (f[1] - '0');
    *m  = (f[2] - '0') * 10 + (f[3] - '0');
    *s  = (f[4] - '0') * 10 + (f[5] - '0');
    const char *dot = strchr(f, '.');
    *ms = dot ? (int)(atof(dot) * 1000.0) : 0;
    return true;
}

/* Parse "DDMMYY" → d/mo/y.  Returns false on bad input. */
static bool parse_date_field(const char *f, int *d, int *mo, int *y)
{
    if (!f || strlen(f) < 6) return false;
    *d  = (f[0] - '0') * 10 + (f[1] - '0');
    *mo = (f[2] - '0') * 10 + (f[3] - '0');
    *y  = (f[4] - '0') * 10 + (f[5] - '0');
    return true;
}

/* Validate NMEA checksum: XOR of all bytes between '$' and '*'. */
static bool nmea_checksum_ok(const char *sentence)
{
    const char *star = strchr(sentence, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t calc = 0;
    for (const char *p = sentence + 1; p < star; p++) calc ^= (uint8_t)*p;
    uint8_t given = (uint8_t)strtol(star + 1, NULL, 16);
    return calc == given;
}

/* ------------------------------------------------------------------ */
/*  NMEA sentence processor                                             */
/* ------------------------------------------------------------------ */

static void process_nmea(const char *sentence)
{
    if (sentence[0] != '$') return;

    if (!nmea_checksum_ok(sentence)) {
        ESP_LOGD(TAG, "Checksum fail: %.20s...", sentence);
        return;
    }

    /* $xxGGA — extract satellite count in use (field 7) */
    if (strncmp(sentence + 3, "GGA,", 4) == 0) {
        char buf[NMEA_MAX_LEN];
        strncpy(buf, sentence, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        char *star = strchr(buf, '*');
        if (star) *star = '\0';

        char *fields[12];
        int   nf = 0;
        for (char *tok = strtok(buf, ","); tok && nf < 12; tok = strtok(NULL, ","))
            fields[nf++] = tok;

        /* field[6]=fix quality (0=invalid), field[7]=sats in use */
        if (nf >= 8 && atoi(fields[6]) > 0) {
            int sats = atoi(fields[7]);
            xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
            s_gps.sat_count = sats;
            xSemaphoreGive(s_gps.mutex);
        }
        return;
    }

    /* Accept $GPRMC and $GNRMC only from here on */
    if (strncmp(sentence + 3, "RMC,", 4) != 0) return;

    /* Work on a mutable copy without the checksum suffix */
    char buf[NMEA_MAX_LEN];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *star = strchr(buf, '*');
    if (star) *star = '\0';

    /* Split on commas: $xxRMC,time,status,lat,NS,lon,EW,spd,cog,date,... */
    char *fields[12];
    int   nf = 0;
    for (char *tok = strtok(buf, ","); tok && nf < 12; tok = strtok(NULL, ","))
        fields[nf++] = tok;

    if (nf < 10) return;
    /* fields[1]=time, fields[2]=status (A/V), fields[9]=date */

    if (fields[2][0] != 'A') {
        if (s_gps.valid) {
            xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
            s_gps.valid = false;
            xSemaphoreGive(s_gps.mutex);
            ESP_LOGW(TAG, "GPS fix lost");
        }
        return;
    }

    int h, m, s, ms, d, mo, y;
    if (!parse_time_field(fields[1], &h, &m, &s, &ms)) return;
    if (!parse_date_field(fields[9], &d, &mo, &y))     return;

    struct tm t = {
        .tm_hour  = h,
        .tm_min   = m,
        .tm_sec   = s,
        .tm_mday  = d,
        .tm_mon   = mo - 1,
        .tm_year  = 100 + y,   /* years since 1900; NEO-M8 gives 2-digit year */
        .tm_isdst = 0,
    };
    time_t unix_sec = utc_mktime(&t);

    /* ------------------------------------------------------------ */
    /*  Choose time reference                                        */
    /* ------------------------------------------------------------ */
    int64_t ref_esp_us;
    suseconds_t ref_usec;

#ifdef CONFIG_GPS_PPS_ENABLED
    /*
     * Snapshot both PPS timestamps atomically before use to avoid
     * an inconsistent read if the ISR fires mid-computation.
     * The PPS pulse fires at the second boundary of the second whose
     * NMEA sentence arrives shortly afterward.  Accept a PPS within 1.1 s.
     */
    int64_t pps_cur  = s_pps_esp_us;
    int64_t pps_prev = s_pps_esp_us_prev;
    int64_t now_us   = esp_timer_get_time();
    int64_t pps_age  = now_us - pps_cur;
    if (pps_cur > 0 && pps_age >= 0 && pps_age < 1100000LL) {
        ref_esp_us = pps_cur;
        ref_usec   = 0;    /* PPS is aligned to the second boundary */
    } else {
        ESP_LOGD(TAG, "PPS not recent (age=%lld µs), using NMEA arrival time",
                 (long long)pps_age);
        ref_esp_us = now_us;
        ref_usec   = (suseconds_t)(ms * 1000);
    }

    /*
     * Estimate crystal frequency error from the interval between
     * the two most recent PPS pulses.  The GPS PPS is accurate to
     * ±30 ns; any deviation from exactly 1,000,000 µs is due to the
     * ESP32's XTAL running slightly fast or slow (typically ±50 ppm).
     *
     * An 8-sample exponential moving average (τ ≈ 8 s) filters noise.
     * Outliers beyond ±200 ppm are rejected (spurious ISR or race).
     *
     * drift_ppb > 0  →  esp_timer counts faster than GPS time
     *                →  interpolated deltas need to be reduced
     */
    int32_t new_drift = s_gps.drift_ppb;   /* sole writer — safe without mutex */
    if (ref_usec == 0 && pps_prev > 0) {
        int64_t interval = pps_cur - pps_prev;
        if (interval > 900000LL && interval < 1100000LL) {
            int64_t raw_ppb = (interval - 1000000LL) * 1000LL;
            new_drift = (int32_t)(((int64_t)new_drift * 7 + raw_ppb) / 8);
            ESP_LOGD(TAG, "PPS interval %lld µs  drift %+ld ppb",
                     (long long)interval, (long)new_drift);
        }
    }
#else
    ref_esp_us = esp_timer_get_time();
    ref_usec   = (suseconds_t)(ms * 1000);
#endif

    /* ------------------------------------------------------------ */
    /*  Update shared state and system clock                         */
    /* ------------------------------------------------------------ */
    xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
    s_gps.tv.tv_sec      = unix_sec;
    s_gps.tv.tv_usec     = ref_usec;
    s_gps.esp_us         = ref_esp_us;
    s_gps.valid          = true;
    s_gps.fix_talker[0]  = sentence[1];   /* e.g. 'G' */
    s_gps.fix_talker[1]  = sentence[2];   /* e.g. 'P','N','L','A','B' */
    s_gps.fix_talker[2]  = '\0';
#ifdef CONFIG_GPS_PPS_ENABLED
    s_gps.drift_ppb  = new_drift;
#endif
    xSemaphoreGive(s_gps.mutex);

    struct timeval sys_tv = { .tv_sec = unix_sec, .tv_usec = ref_usec };
    settimeofday(&sys_tv, NULL);

    ESP_LOGI(TAG, "Fix: 20%02d-%02d-%02d %02d:%02d:%02d.%03d UTC%s",
             y, mo, d, h, m, s, ms,
#ifdef CONFIG_GPS_PPS_ENABLED
             (ref_usec == 0) ? " [PPS]" : " [NMEA]"
#else
             ""
#endif
    );
}

/* ------------------------------------------------------------------ */
/*  UART reader task                                                    */
/* ------------------------------------------------------------------ */

static void gps_task(void *arg)
{
    uint8_t *raw   = malloc(UART_BUF_SIZE);
    char     line[NMEA_MAX_LEN];
    int      lpos  = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, raw, UART_BUF_SIZE - 1,
                                  pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            char c = (char)raw[i];

            if (c == '$') lpos = 0;   /* start of new sentence */

            if (lpos < NMEA_MAX_LEN - 1)
                line[lpos++] = c;

            if (c == '\n' || c == '\r') {
                if (lpos > 6) {
                    line[lpos] = '\0';
                    process_nmea(line);
                }
                lpos = 0;
            }
        }
    }
}

#endif /* CONFIG_GPS_ENABLED */

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/*                                                                      */
/*  All functions are always defined.  When CONFIG_GPS_ENABLED is not  */
/*  set they compile to stubs: gps_get_time falls back to the system   */
/*  clock (updated by SNTP), all sync indicators return false/-1.      */
/* ------------------------------------------------------------------ */

#ifdef CONFIG_GPS_ENABLED
static bool s_gps_hw_ok = false;
#endif

void gps_init(void)
{
#ifdef CONFIG_GPS_ENABLED
    s_gps.mutex     = xSemaphoreCreateMutex();
    s_gps.sat_count = -1;

    uart_config_t uart_cfg = {
        .baud_rate  = GPS_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return;
    }
    err = uart_param_config(GPS_UART_NUM, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return;
    }
    err = uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return;
    }

    ubx_configure_gnss();

#ifdef CONFIG_GPS_PPS_ENABLED
    gpio_config_t pps_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_GPS_PPS_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    err = gpio_config(&pps_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPS gpio_config failed: %s", esp_err_to_name(err));
        return;
    }
    /* gpio_install_isr_service returns ESP_ERR_INVALID_STATE if already
     * installed by another driver — that is fine, treat it as success. */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return;
    }
    err = gpio_isr_handler_add(CONFIG_GPS_PPS_PIN, pps_isr_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPS isr_handler_add failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "PPS enabled on GPIO %d", CONFIG_GPS_PPS_PIN);
#endif

    s_gps_hw_ok = true;
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 6, NULL);
    ESP_LOGI(TAG, "Initialized: UART%d  RX=GPIO%d  TX=GPIO%d  %d baud",
             GPS_UART_NUM, GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);
#else
    ESP_LOGI(TAG, "GPS disabled — stratum-2 via SNTP fallback only");
#endif
}

bool gps_get_time(struct timeval *tv_out)
{
#ifdef CONFIG_GPS_ENABLED
    xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
    bool valid = s_gps.valid;

    if (valid) {
        int64_t delta_us = esp_timer_get_time() - s_gps.esp_us;
#ifdef CONFIG_GPS_PPS_ENABLED
        delta_us -= (delta_us * (int64_t)s_gps.drift_ppb) / 1000000000LL;
#endif
        int64_t base_us  = (int64_t)s_gps.tv.tv_sec * 1000000LL
                         + (int64_t)s_gps.tv.tv_usec;
        int64_t now_us   = base_us + delta_us;
        tv_out->tv_sec   = (time_t)(now_us / 1000000LL);
        tv_out->tv_usec  = (suseconds_t)(now_us % 1000000LL);
    } else {
        gettimeofday(tv_out, NULL);
    }

    xSemaphoreGive(s_gps.mutex);
    return valid;
#else
    /* No GPS — return system clock updated by SNTP, not GPS-synced */
    gettimeofday(tv_out, NULL);
    return false;
#endif
}

bool gps_is_synced(void)
{
#ifdef CONFIG_GPS_ENABLED
    xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
    bool v = s_gps.valid;
    xSemaphoreGive(s_gps.mutex);
    return v;
#else
    return false;
#endif
}

bool gps_get_last_fix_time(struct timeval *tv_out)
{
#ifdef CONFIG_GPS_ENABLED
    xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
    bool valid = s_gps.valid;
    if (valid) *tv_out = s_gps.tv;
    xSemaphoreGive(s_gps.mutex);
    return valid;
#else
    (void)tv_out;
    return false;
#endif
}

int gps_get_sat_count(void)
{
#ifdef CONFIG_GPS_ENABLED
    xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
    int n = s_gps.sat_count;
    xSemaphoreGive(s_gps.mutex);
    return n;
#else
    return -1;
#endif
}

void gps_get_fix_talker(char out[3])
{
#ifdef CONFIG_GPS_ENABLED
    xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
    out[0] = s_gps.fix_talker[0];
    out[1] = s_gps.fix_talker[1];
    out[2] = '\0';
    xSemaphoreGive(s_gps.mutex);
#else
    out[0] = out[1] = out[2] = '\0';
#endif
}

bool gps_hw_ok(void)
{
#ifdef CONFIG_GPS_ENABLED
    return s_gps_hw_ok;
#else
    return false;
#endif
}

bool gps_is_pps_locked(void)
{
#if defined(CONFIG_GPS_ENABLED) && defined(CONFIG_GPS_PPS_ENABLED)
    int64_t pps_cur = s_pps_esp_us;
    int64_t age     = esp_timer_get_time() - pps_cur;
    return pps_cur > 0 && age >= 0 && age < 1100000LL;
#else
    return false;
#endif
}
