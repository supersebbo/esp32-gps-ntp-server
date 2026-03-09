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

    ESP_LOGI(TAG, "UBX-CFG-GNSS no ACK — module likely pre-configured (multi-constellation active)");
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
static int ubx_configure_gnss(void)  /* returns 1=ACK, 0=NAK, -1=timeout */
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
        /* detailed diagnosis already logged inside ubx_wait_ack() */
    }
    return result;

#undef UBX_GNSS_FRAME_LEN
#undef UBX_GNSS_N_BLOCKS
}

/* ------------------------------------------------------------------ */
/*  UBX-CFG-PRT — set UART1 baud rate (8N1, UBX+NMEA in and out)      */
/* ------------------------------------------------------------------ */

static void ubx_set_uart_baud(uint32_t baud)
{
    uint8_t frame[28];
    frame[0]  = 0xB5; frame[1]  = 0x62;  /* UBX sync chars           */
    frame[2]  = 0x06; frame[3]  = 0x00;  /* class=CFG, id=PRT        */
    frame[4]  = 0x14; frame[5]  = 0x00;  /* payload length = 20      */
    frame[6]  = 0x01;                     /* portID = UART1           */
    frame[7]  = 0x00;                     /* reserved                 */
    frame[8]  = 0x00; frame[9]  = 0x00;  /* txReady (disabled)       */
    frame[10] = 0xD0; frame[11] = 0x08;  /* mode = 0x000008D0 (8N1)  */
    frame[12] = 0x00; frame[13] = 0x00;
    frame[14] = (uint8_t)(baud);          /* baudRate, little-endian  */
    frame[15] = (uint8_t)(baud >>  8);
    frame[16] = (uint8_t)(baud >> 16);
    frame[17] = (uint8_t)(baud >> 24);
    frame[18] = 0x07; frame[19] = 0x00;  /* inProtoMask: UBX+NMEA+RTCM */
    frame[20] = 0x03; frame[21] = 0x00;  /* outProtoMask: UBX+NMEA   */
    frame[22] = 0x00; frame[23] = 0x00;  /* flags                    */
    frame[24] = 0x00; frame[25] = 0x00;  /* reserved                 */
    ubx_cksum(&frame[2], 24, &frame[26], &frame[27]);
    uart_write_bytes(GPS_UART_NUM, frame, sizeof(frame));
}

/* ------------------------------------------------------------------ */
/*  UBX-CFG-CFG — save current configuration to BBR and flash         */
/* ------------------------------------------------------------------ */

static void ubx_save_config(void)
{
    uint8_t frame[21];
    frame[0]  = 0xB5; frame[1]  = 0x62;  /* sync                     */
    frame[2]  = 0x06; frame[3]  = 0x09;  /* class=CFG, id=CFG        */
    frame[4]  = 0x0D; frame[5]  = 0x00;  /* payload length = 13      */
    /* clearMask = 0 (clear nothing) */
    frame[6]  = 0x00; frame[7]  = 0x00; frame[8]  = 0x00; frame[9]  = 0x00;
    /* saveMask = 0x0000001F: ioPort | msgConf | infMsg | navConf | rxmConf */
    frame[10] = 0x1F; frame[11] = 0x00; frame[12] = 0x00; frame[13] = 0x00;
    /* loadMask = 0 (load nothing) */
    frame[14] = 0x00; frame[15] = 0x00; frame[16] = 0x00; frame[17] = 0x00;
    /* deviceMask = 0x03: devBBR (bit0) | devFlash (bit1) */
    frame[18] = 0x03;
    ubx_cksum(&frame[2], 17, &frame[19], &frame[20]);
    uart_write_bytes(GPS_UART_NUM, frame, sizeof(frame));
}

/* ------------------------------------------------------------------ */
/*  UBX-NAV-TIMEGPS poll — read leap-second status from module         */
/*                                                                      */
/*  Sends a poll request and parses the 16-byte response payload:      */
/*    offset 10: leapS  (int8)  — GPS-UTC leap seconds                 */
/*    offset 11: valid  (uint8) — bit 2 = leapSValid                   */
/*                                                                      */
/*  Returns true on success.  The caller should not rely on the values  */
/*  when returning false (timeout — no TX line, or module not ready).   */
/*                                                                      */
/*  NMEA bytes that arrive during the 500 ms window pass through the    */
/*  state machine without matching and are discarded; at most one NMEA  */
/*  sentence per query is lost, which is acceptable for periodic use.   */
/* ------------------------------------------------------------------ */

static bool ubx_query_leapsec(int8_t *leapS_out, bool *valid_out)
{
    /* Poll request: zero-payload NAV-TIMEGPS */
    uint8_t poll[8] = { 0xB5, 0x62, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00 };
    ubx_cksum(&poll[2], 4, &poll[6], &poll[7]);
    uart_write_bytes(GPS_UART_NUM, poll, sizeof(poll));

    /*
     * Response frame layout (24 bytes total):
     *   [0-1]   0xB5 0x62          sync
     *   [2]     0x01               class = NAV
     *   [3]     0x20               id    = TIMEGPS
     *   [4-5]   0x10 0x00          payload length = 16
     *   [6-21]  payload (16 bytes)
     *   [22-23] ck_a ck_b
     *
     * Payload offsets (relative to payload start):
     *   0-3   iTOW  (uint32, ms)
     *   4-7   fTOW  (int32,  ns)
     *   8-9   week  (int16)
     *   10    leapS (int8)        ← what we want
     *   11    valid (uint8, flags) bit2 = leapSValid
     *   12-15 tAcc  (uint32, ns)
     *
     * State machine: 0=sync1, 1=sync2, 2=class, 3=id, 4=len_lo,
     *                5=len_hi, 6..21=payload[0..15]
     * Any unexpected byte resets to 0 (or 1 if 0xB5).
     */
    int64_t deadline = esp_timer_get_time() + 500000LL;  /* 500 ms */
    int     state    = 0;
    uint8_t payload[16];

    while (esp_timer_get_time() < deadline) {
        uint8_t b;
        if (uart_read_bytes(GPS_UART_NUM, &b, 1, pdMS_TO_TICKS(10)) <= 0)
            continue;

        switch (state) {
        case 0: state = (b == 0xB5) ? 1 : 0;                         break;
        case 1: state = (b == 0x62) ? 2 : (b == 0xB5 ? 1 : 0);      break;
        case 2: state = (b == 0x01) ? 3 : (b == 0xB5 ? 1 : 0);      break;
        case 3: state = (b == 0x20) ? 4 : (b == 0xB5 ? 1 : 0);      break;
        case 4: state = (b == 0x10) ? 5 : (b == 0xB5 ? 1 : 0);      break;
        case 5: state = (b == 0x00) ? 6 : (b == 0xB5 ? 1 : 0);      break;
        default:
            /* States 6–21: collect 16 payload bytes */
            payload[state - 6] = b;
            state++;
            if (state == 22) {
                *leapS_out = (int8_t)payload[10];
                *valid_out = (payload[11] & 0x04) != 0;
                return true;
            }
            break;
        }
    }
    return false;   /* timeout */
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
static bool s_gps_hw_ok = false;          /* true once any NMEA sentence is received */

/* Per-constellation SNR lists.  Each has its own work buffer so that
 * interleaved $xxGSV groups from different constellations don't clobber
 * each other.  SNR=0 is kept (satellite in view but no signal yet). */
#define SNRLIST_MAX 16
static int s_gp_work[SNRLIST_MAX]; static int s_gp_work_n = 0;
static int s_gl_work[SNRLIST_MAX]; static int s_gl_work_n = 0;
static int s_ga_work[SNRLIST_MAX]; static int s_ga_work_n = 0;
static int s_gn_work[SNRLIST_MAX]; static int s_gn_work_n = 0;
static int s_gp_snr[SNRLIST_MAX];  static int s_gp_snr_n  = 0;  static int s_gp_in_view = 0;
static int s_gl_snr[SNRLIST_MAX];  static int s_gl_snr_n  = 0;  static int s_gl_in_view = 0;
static int s_ga_snr[SNRLIST_MAX];  static int s_ga_snr_n  = 0;  static int s_ga_in_view = 0;
static int s_gn_snr[SNRLIST_MAX];  static int s_gn_snr_n  = 0;  static int s_gn_in_view = 0;
/* Best SNR across all constellations — derived after each group completes */
static volatile int32_t s_best_snr = -1;

/* First-lock sanity check state (GPS-UTC leap-second validation) */
static int     s_fix_sanity_rejects = 0;
static int64_t s_leapsec_correction = 0;  /* auto-detected correction to add to NMEA unix_sec */
static bool    s_ls_utc_valid       = false; /* UBX-NAV-TIMEGPS confirmed leapSValid */

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

/*
 * Split a mutable NMEA field string on commas, storing pointers to each
 * field in fields[].  Unlike strtok, consecutive commas produce empty-string
 * entries so field indices always match the NMEA spec even when optional
 * fields (e.g. course-over-ground) are absent.  Returns the field count.
 */
static int nmea_split(char *buf, char **fields, int max_fields)
{
    int n = 0;
    if (n < max_fields) fields[n++] = buf;
    for (char *p = buf; *p && n < max_fields; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    return n;
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

    /* $xxGGA — sat_count no longer sourced from GGA (driven by GSV in_view
     * totals instead, which reflect actual current tracking vs. cached fix). */
    if (strncmp(sentence + 3, "GGA,", 4) == 0) return;

    /* $xxGSV — collect per-satellite SNR grouped by constellation.
     * Talker: GP=GPS  GL=GLONASS  GA=Galileo
     * Sentence: $xxGSV,total,msgnum,sats_in_view,prn,elev,az,snr[,...]*hh
     * SNR fields at indices 7, 11, 15, 19 (up to 4 sats per sentence). */
    if (strncmp(sentence + 3, "GSV,", 4) == 0) {
        char buf[NMEA_MAX_LEN];
        strncpy(buf, sentence, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        char *star = strchr(buf, '*');
        if (star) *star = '\0';

        char *fields[20];
        int   nf = nmea_split(buf, fields, 20);

        if (nf < 4) return;
        int  total  = atoi(fields[1]);
        int  msgnum = atoi(fields[2]);
        char talker = sentence[2];   /* 'P'=GPS  'L'=GLONASS  'A'=Galileo  'N'=combined */

        /* Select this constellation's own work buffer and published array */
        int *work; int *work_n; int *dst; int *dst_n; int *in_view;
        if      (talker == 'P') { work=s_gp_work; work_n=&s_gp_work_n; dst=s_gp_snr; dst_n=&s_gp_snr_n; in_view=&s_gp_in_view; }
        else if (talker == 'L') { work=s_gl_work; work_n=&s_gl_work_n; dst=s_gl_snr; dst_n=&s_gl_snr_n; in_view=&s_gl_in_view; }
        else if (talker == 'A') { work=s_ga_work; work_n=&s_ga_work_n; dst=s_ga_snr; dst_n=&s_ga_snr_n; in_view=&s_ga_in_view; }
        else if (talker == 'N') { work=s_gn_work; work_n=&s_gn_work_n; dst=s_gn_snr; dst_n=&s_gn_snr_n; in_view=&s_gn_in_view; }
        else { return; }

        if (msgnum == 1) {
            *work_n  = 0;
            /* field[3] = total sats in view for this constellation */
            *in_view = (nf > 3) ? atoi(fields[3]) : 0;
        }

        /* Collect SNR values.  Cap at 60 dBHz — values above this indicate
         * a field-misparse (e.g. PRN or azimuth read instead of CNR). */
        for (int i = 7; i < nf; i += 4) {
            if (fields[i][0] != '\0' && *work_n < SNRLIST_MAX) {
                int snr = atoi(fields[i]);
                work[(*work_n)++] = (snr > 60) ? 0 : snr;
            }
        }

        if (msgnum == total) {
            *dst_n = *work_n;
            memcpy(dst, work, *work_n * sizeof(int));

            /* Recompute best SNR across all constellations */
            int best = -1;
            for (int i = 0; i < s_gp_snr_n; i++) if (s_gp_snr[i] > best) best = s_gp_snr[i];
            for (int i = 0; i < s_gl_snr_n; i++) if (s_gl_snr[i] > best) best = s_gl_snr[i];
            for (int i = 0; i < s_ga_snr_n; i++) if (s_ga_snr[i] > best) best = s_ga_snr[i];
            for (int i = 0; i < s_gn_snr_n; i++) if (s_gn_snr[i] > best) best = s_gn_snr[i];
            s_best_snr = best;

            /* Update sat count from GSV in_view totals — more accurate than
             * GGA sats-in-solution during acquisition (module can report a stale
             * high count from a cached position while only a few are tracked). */
            xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
            s_gps.sat_count = s_gp_in_view + s_gl_in_view + s_ga_in_view + s_gn_in_view;
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

    /*
     * Split on commas.
     * NMEA 0183 v4.1 $xxRMC has 14 fields (0–13):
     *   [0]  sentence ID     [1]  time      [2]  status (A/V)
     *   [3]  lat             [4]  N/S        [5]  lon     [6]  E/W
     *   [7]  speed           [8]  course     [9]  date
     *   [10] mag-var         [11] mag-var-dir
     *   [12] mode indicator  [13] nav-status (A=valid, V=warning, N=invalid)
     * Older firmware omits fields [12] and [13].
     */
    char *fields[15];
    int   nf = nmea_split(buf, fields, 15);

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

    /*
     * NMEA 4.1 navigation status field (fields[13]).
     * 'V' = "Navigation receiver Warning": UTC time model not yet validated
     * (leap-second data not yet confirmed from navigation message).
     * s_ls_utc_valid (from UBX-NAV-TIMEGPS leapSValid) is the authoritative
     * check and owns user-visible messaging; nav-status here is informational.
     */
    bool nav_status_warn = (nf >= 14 && fields[13][0] == 'V');
    ESP_LOGD(TAG, "RMC nav-status=%c", nav_status_warn ? 'V' : 'A');

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
    /*  Sanity-check GPS time against system clock on first lock     */
    /*                                                               */
    /*  The NEO-M8 can assert RMC status='A' before (or without)    */
    /*  downloading the current GPS-UTC leap-second correction.      */
    /*  If the module's stored model is stale (e.g. GPS-UTC=11 from  */
    /*  ~1999 vs the current value of 18), its NMEA UTC output is    */
    /*  persistently fast by exactly (18 - stored) seconds.          */
    /*                                                               */
    /*  Strategy:                                                     */
    /*   1. Compare GPS time to system clock (set by SNTP) on every  */
    /*      first-lock attempt.                                       */
    /*   2. If they differ by >2 s, reject and count.                */
    /*   3. After 30 consistent rejects (~30 s), the offset is       */
    /*      clearly systematic.  Auto-apply a correction so the      */
    /*      module's time becomes usable even with a stale UTC model. */
    /*   PPS timing is unaffected — the PPS edge is always aligned   */
    /*   to the true GPS second boundary regardless of the UTC model. */
    /* ------------------------------------------------------------ */
    if (!s_gps.valid) {
        /* Apply any previously auto-detected leap-second correction */
        unix_sec += s_leapsec_correction;

        /*
         * Sanity-check GPS time against system clock (set by SNTP/RTC).
         *
         * Skipped when the module has already confirmed its UTC model via
         * UBX-NAV-TIMEGPS leapSValid=YES — in that case the NMEA UTC is
         * authoritative and we accept it immediately, just like u-center.
         *
         * When the UTC model is not yet validated (dead coin cell / cold
         * start), the module may output NMEA time that is off by exactly
         * (18 - stored_leapS) seconds.  We compare against the system
         * clock (SNTP) and auto-correct after 5 consistent rejects.
         * Re-correction is allowed on every subsequent multiple of 5 so
         * a wrong first correction doesn't permanently block lock.
         */
        if (!s_ls_utc_valid) {
            struct timeval sys_tv;
            gettimeofday(&sys_tv, NULL);
            /* 1577836800 = 2020-01-01 00:00:00 UTC */
            if (sys_tv.tv_sec > 1577836800LL) {
                int64_t diff     = (int64_t)unix_sec - (int64_t)sys_tv.tv_sec;
                int64_t abs_diff = (diff < 0) ? -diff : diff;

                if (abs_diff > 2) {
                    if (++s_fix_sanity_rejects == 1 || (s_fix_sanity_rejects % 30) == 0) {
                        ESP_LOGW(TAG,
                                 "GPS time sanity fail: GPS=%lld sys=%lld diff=%llds "
                                 "(reject #%d; nav-status=%c stale GPS-UTC offset)",
                                 (long long)unix_sec, (long long)sys_tv.tv_sec,
                                 (long long)diff, s_fix_sanity_rejects,
                                 nav_status_warn ? 'V' : 'A');
                    }
                    /* After 5 consistent rejects the error is systematic.
                     * Re-apply correction every 5 rejects so a wrong first
                     * estimate gets corrected rather than blocking lock forever. */
                    if (s_fix_sanity_rejects % 5 == 0) {
                        s_leapsec_correction = -diff;
                        ESP_LOGW(TAG,
                                 "GPS UTC model stale: auto-applying %+lld s correction "
                                 "(reject #%d; stored GPS-UTC offset ≈ %lld s, "
                                 "current correct value is 18 s since Jan 2017)",
                                 (long long)s_leapsec_correction, s_fix_sanity_rejects,
                                 (long long)(18LL + s_leapsec_correction));
                    }
                    return;
                }
            }
        }
        s_fix_sanity_rejects = 0;
        if (s_leapsec_correction != 0) {
            ESP_LOGI(TAG, "GPS time accepted with %+lld s leap-sec correction applied",
                     (long long)s_leapsec_correction);
        }
    }

    /* ------------------------------------------------------------ */
    /*  Update shared state and system clock                         */
    /* ------------------------------------------------------------ */
    xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
    bool first_fix       = !s_gps.valid;
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

    if (first_fix) {
        ESP_LOGI(TAG, "GPS locked: 20%02d-%02d-%02d %02d:%02d:%02d UTC  "
                      "talker=%c%c  sats=%d%s",
                 y, mo, d, h, m, s,
                 sentence[1], sentence[2],
                 s_gps.sat_count,
#ifdef CONFIG_GPS_PPS_ENABLED
                 (ref_usec == 0) ? "  [PPS]" : "  [NMEA]"
#else
                 ""
#endif
        );
    } else {
        ESP_LOGD(TAG, "Fix: 20%02d-%02d-%02d %02d:%02d:%02d.%03d UTC%s",
                 y, mo, d, h, m, s, ms,
#ifdef CONFIG_GPS_PPS_ENABLED
                 (ref_usec == 0) ? " [PPS]" : " [NMEA]"
#else
                 ""
#endif
        );
    }
}

/* ------------------------------------------------------------------ */
/*  UART reader task                                                    */
/* ------------------------------------------------------------------ */

static void gps_task(void *arg)
{
    uint8_t *raw      = malloc(UART_BUF_SIZE);
    char     line[NMEA_MAX_LEN];
    int      lpos     = 0;
    int      search_ticks = 0;   /* counts 100 ms UART reads while no fix */

    ESP_LOGI(TAG, "Searching for satellites...");

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
                    if (!s_gps_hw_ok) {
                        s_gps_hw_ok = true;
                        ESP_LOGI(TAG, "GPS module detected (NMEA data received)");
                    }
                    process_nmea(line);
                }
                lpos = 0;
            }
        }

        /* Log "searching" every 30 s while no fix */
        if (!s_gps.valid) {
            if (++search_ticks >= 300) {   /* 300 × 100 ms = 30 s */
                search_ticks = 0;
                int sats = s_gps.sat_count;
                int snr  = s_best_snr;
                if (snr >= 0) {
                    /* Build per-constellation SNR list string */
                    char snr_str[256] = "";
                    int  pos = 0;
                    struct { const char *lbl; const int *arr; int n; int iv; } cs[4] = {
                        { "GPS", s_gp_snr, s_gp_snr_n, s_gp_in_view },
                        { "GLO", s_gl_snr, s_gl_snr_n, s_gl_in_view },
                        { "GAL", s_ga_snr, s_ga_snr_n, s_ga_in_view },
                        { "GN",  s_gn_snr, s_gn_snr_n, s_gn_in_view },
                    };
                    for (int c = 0; c < 4; c++) {
                        if (cs[c].iv == 0 && cs[c].n == 0) continue;
                        /* label(in_view/snr_count): show even if no SNR data */
                        pos += snprintf(snr_str + pos, sizeof(snr_str) - pos,
                                        " %s(%d/%d)[", cs[c].lbl, cs[c].iv, cs[c].n);
                        for (int i = 0; i < cs[c].n; i++)
                            pos += snprintf(snr_str + pos, sizeof(snr_str) - pos,
                                            "%s%d", i ? " " : "", cs[c].arr[i]);
                        pos += snprintf(snr_str + pos, sizeof(snr_str) - pos, "]");
                    }
                    ESP_LOGI(TAG, "Searching... %d sat%s  dBHz:%s  best=%d%s",
                             sats, sats == 1 ? "" : "s", snr_str, snr,
                             snr >= 30 ? " (ephemeris downloading)" : " (weak — check antenna)");
                } else
                    ESP_LOGI(TAG, "Searching... no satellites yet");
            }
        } else {
            search_ticks = 0;   /* reset so we log promptly after next fix loss */
        }

        /* -------------------------------------------------------- */
        /*  Periodic UBX-NAV-TIMEGPS query — monitor leap-second    */
        /*  validity so the operator can see when the UTC model is   */
        /*  confirmed after a cold start (e.g. dead backup battery). */
        /*  Queries every 30 s until leapSValid, then every 5 min.  */
        /* -------------------------------------------------------- */
        if (s_gps_hw_ok) {
            static int  s_ls_ticks    = 150;   /* first query at ~15 s */
            static bool s_ls_valid    = false;
            static int8_t s_ls_value  = 0;

            int interval = s_ls_valid ? 3000 : 300;   /* 5 min or 30 s */
            if (++s_ls_ticks >= interval) {
                s_ls_ticks = 0;
                int8_t ls; bool lv;
                if (ubx_query_leapsec(&ls, &lv)) {
                    if (!lv) {
                        ESP_LOGW(TAG, "leapS=%d leapSValid=NO "
                                 "(UTC model not yet confirmed — cold start / flat battery?)",
                                 ls);
                    } else if (!s_ls_valid) {
                        /* Just became valid */
                        ESP_LOGI(TAG, "leapS=%d leapSValid=YES — UTC model confirmed by satellites",
                                 ls);
                        if (ls != 18)
                            ESP_LOGW(TAG, "leapS=%d unexpected (current correct value is 18 "
                                     "since Jan 2017) — check module firmware", ls);
                        if (s_leapsec_correction != 0)
                            ESP_LOGI(TAG, "Note: auto-correction of %+lld s was applied during "
                                     "the unvalidated window", (long long)s_leapsec_correction);
                    } else if (ls != s_ls_value) {
                        ESP_LOGW(TAG, "leapS changed: %d → %d", s_ls_value, ls);
                    } else {
                        ESP_LOGD(TAG, "leapS=%d leapSValid=YES", ls);
                    }
                    s_ls_valid     = lv;
                    s_ls_utc_valid = lv;   /* allow sanity-check bypass once confirmed */
                    s_ls_value     = ls;
                } else {
                    ESP_LOGD(TAG, "UBX-NAV-TIMEGPS poll: no response (TX line connected?)");
                }
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

void gps_init(void)
{
#ifdef CONFIG_GPS_ENABLED
    s_gps.mutex     = xSemaphoreCreateMutex();
    s_gps.sat_count = -1;

    /*
     * UART configuration.
     * If the target baud rate differs from 9600 (the module's factory
     * default), we start at 9600 so we can reliably send the baud-rate
     * change command regardless of what the module currently has stored.
     * After negotiation the driver is switched to the target rate.
     */
    uart_config_t uart_cfg = {
#if CONFIG_GPS_BAUD_RATE != 9600
        .baud_rate  = 9600,           /* initial rate; negotiated below */
#else
        .baud_rate  = GPS_BAUD_RATE,
#endif
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

    vTaskDelay(pdMS_TO_TICKS(500));  /* allow GPS module to boot before UBX */

#if CONFIG_GPS_BAUD_RATE != 9600
    /*
     * Baud-rate negotiation.
     *
     * The NEO-M8 reverts to 9600 after power loss unless the config is
     * saved to flash.  Strategy:
     *   1. Send CFG-PRT at 9600.
     *      - If module is at 9600: it receives the command and switches.
     *      - If module is already at GPS_BAUD_RATE: the bytes arrive as
     *        framing noise and are silently discarded; the module stays
     *        at GPS_BAUD_RATE unchanged.
     *   2. Switch the ESP32 UART to GPS_BAUD_RATE — both sides now agree.
     *   3. Send CFG-CFG to persist the baud rate to BBR + flash so it
     *      survives future power cycles.
     */
    ESP_LOGI(TAG, "Negotiating baud rate: 9600 → %d", GPS_BAUD_RATE);
    ubx_set_uart_baud(GPS_BAUD_RATE);
    uart_wait_tx_done(GPS_UART_NUM, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(100));        /* module applies new baud  */
    uart_set_baudrate(GPS_UART_NUM, GPS_BAUD_RATE);
    uart_flush_input(GPS_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(100));        /* UART settle              */
    ubx_save_config();
    vTaskDelay(pdMS_TO_TICKS(500));        /* flash write completes    */
    ESP_LOGI(TAG, "Baud rate %d set and saved to module flash", GPS_BAUD_RATE);
#endif

    int ubx_result = ubx_configure_gnss();

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

    (void)ubx_result;  /* UBX ACK not used to gate hw_ok — NMEA presence does */
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

int64_t gps_last_fix_esp_us(void)
{
#ifdef CONFIG_GPS_ENABLED
    xSemaphoreTake(s_gps.mutex, portMAX_DELAY);
    int64_t us = s_gps.esp_us;   /* 0 until first fix; retained after fix loss */
    xSemaphoreGive(s_gps.mutex);
    return us;
#else
    return 0;
#endif
}
