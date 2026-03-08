#include "oled.h"
#include "sdkconfig.h"

#ifdef CONFIG_OLED_ENABLED

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "i2c_bus.h"

#include "esp_netif.h"
#include "esp_timer.h"

#if !CONFIG_IDF_TARGET_ESP32
#  include "driver/temperature_sensor.h"
static temperature_sensor_handle_t s_temp_sensor = NULL;
#endif

#include "gps.h"
#include "sntp_fallback.h"
#include "ntp_server.h"
#include "rtc.h"

static const char *TAG = "OLED";

/* ------------------------------------------------------------------ */
/*  6×8 pixel font  (95 printable ASCII chars, 0x20–0x7E)              */
/*                                                                      */
/*  Encoding: 5 bytes per character, one byte per column (left→right). */
/*  Each byte represents 8 vertical pixels; bit 0 = topmost pixel.     */
/*  A 6th zero-column is added automatically at render time (spacing). */
/* ------------------------------------------------------------------ */

static const uint8_t s_font[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 0x20  ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* 0x21  '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* 0x22  '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* 0x23  '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* 0x24  '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* 0x25  '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* 0x26  '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* 0x27  '\'' */
    {0x00,0x1C,0x22,0x41,0x00}, /* 0x28  '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* 0x29  ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* 0x2A  '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* 0x2B  '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* 0x2C  ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* 0x2D  '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* 0x2E  '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* 0x2F  '/' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 0x30  '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* 0x31  '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* 0x32  '2' */
    {0x21,0x41,0x49,0x4D,0x33}, /* 0x33  '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* 0x34  '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* 0x35  '5' */
    {0x3C,0x4A,0x49,0x49,0x31}, /* 0x36  '6' */
    {0x41,0x21,0x11,0x09,0x07}, /* 0x37  '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* 0x38  '8' */
    {0x46,0x49,0x49,0x29,0x1E}, /* 0x39  '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* 0x3A  ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* 0x3B  ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* 0x3C  '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* 0x3D  '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* 0x3E  '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* 0x3F  '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* 0x40  '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 0x41  'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 0x42  'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 0x43  'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 0x44  'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 0x45  'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 0x46  'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 0x47  'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 0x48  'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 0x49  'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 0x4A  'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 0x4B  'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 0x4C  'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 0x4D  'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 0x4E  'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 0x4F  'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 0x50  'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 0x51  'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 0x52  'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 0x53  'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 0x54  'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 0x55  'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 0x56  'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 0x57  'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 0x58  'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 0x59  'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 0x5A  'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* 0x5B  '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* 0x5C  '\\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* 0x5D  ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* 0x5E  '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* 0x5F  '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* 0x60  '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 0x61  'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 0x62  'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 0x63  'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 0x64  'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 0x65  'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 0x66  'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 0x67  'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 0x68  'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 0x69  'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 0x6A  'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 0x6B  'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 0x6C  'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 0x6D  'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 0x6E  'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 0x6F  'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 0x70  'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 0x71  'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 0x72  'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 0x73  's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 0x74  't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 0x75  'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 0x76  'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 0x77  'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 0x78  'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 0x79  'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 0x7A  'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* 0x7B  '{' */
    {0x00,0x00,0x7F,0x00,0x00}, /* 0x7C  '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* 0x7D  '}' */
    {0x10,0x08,0x08,0x10,0x08}, /* 0x7E  '~' */
};

/* ------------------------------------------------------------------ */
/*  Framebuffer                                                         */
/*                                                                      */
/*  SSD1306 horizontal addressing mode:                                 */
/*    fb[page * 128 + col]  — page = y/8, bit = y%8 (bit 0 = topmost) */
/*                                                                      */
/*  s_tx[0] is always 0x40 (I2C data-stream control byte); the display */
/*  framebuffer occupies s_tx[1..512].  This lets ssd1306_flush() send */
/*  the entire buffer in a single i2c_master_write_to_device() call.   */
/* ------------------------------------------------------------------ */

#define FB_W  128
#ifdef CONFIG_OLED_HEIGHT_64
#  define FB_H   64
#else
#  define FB_H   32
#endif
#define FB_PAGES (FB_H / 8)   /* 4 pages (32-row) or 8 pages (64-row) */

static uint8_t s_tx[1 + FB_W * FB_PAGES];   /* [0]=0x40 control, [1..512]=pixels */
#define s_fb (s_tx + 1)                      /* alias: s_fb[0..511] = pixels */

static inline void fb_clear(void)
{
    memset(s_fb, 0, FB_W * FB_PAGES);
}

static inline void fb_set_pixel(int x, int y)
{
    if ((unsigned)x >= FB_W || (unsigned)y >= FB_H) return;
    s_fb[(y >> 3) * FB_W + x] |= (uint8_t)(1u << (y & 7));
}

static void fb_draw_char_1x(int x0, int y0, char c)
{
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *g = s_font[(uint8_t)c - 0x20];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 8; row++)
            if ((bits >> row) & 1)
                fb_set_pixel(x0 + col, y0 + row);
    }
    /* col 5 is implicit zero (spacing) */
}

/*
 * 2× scale: each font pixel → 2×2 block.
 * Character cell: 12 px wide (5 cols × 2 + 2 spacing), 16 px tall.
 */
static void fb_draw_char_2x(int x0, int y0, char c)
{
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *g = s_font[(uint8_t)c - 0x20];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 8; row++) {
            if ((bits >> row) & 1) {
                int px = x0 + col * 2;
                int py = y0 + row * 2;
                fb_set_pixel(px,     py);
                fb_set_pixel(px + 1, py);
                fb_set_pixel(px,     py + 1);
                fb_set_pixel(px + 1, py + 1);
            }
        }
    }
}

static void fb_draw_str_1x(int x, int y, const char *str)
{
    while (*str) {
        fb_draw_char_1x(x, y, *str++);
        x += 6;
    }
}

static void fb_draw_str_2x(int x, int y, const char *str)
{
    while (*str) {
        fb_draw_char_2x(x, y, *str++);
        x += 12;
    }
}

/* ------------------------------------------------------------------ */
/*  SSD1306 I2C driver (legacy ESP-IDF i2c_master_write_to_device API) */
/* ------------------------------------------------------------------ */

#define I2C_PORT  ((i2c_port_t)CONFIG_OLED_I2C_PORT)
#define I2C_ADDR  ((uint8_t)CONFIG_OLED_I2C_ADDR)
#define I2C_TIMEOUT_MS  50

static void ssd1306_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};   /* 0x00 = Co=0, D/C#=0 → command */
    i2c_bus_lock(I2C_PORT);
    i2c_master_write_to_device(I2C_PORT, I2C_ADDR,
                               buf, sizeof(buf),
                               pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_bus_unlock(I2C_PORT);
}

/*
 * Initialisation sequence for a 128×32 SSD1306 panel.
 * Key differences from 128×64: MUX ratio = 31, COM pins config = 0x02.
 */
static void ssd1306_init(void)
{
    ssd1306_cmd(0xAE);        /* display off */
    ssd1306_cmd(0xD5);        /* set display clock divide / osc freq */
    ssd1306_cmd(0x80);        /*   divide=1, freq=8 */
    ssd1306_cmd(0xA8);        /* set multiplex ratio */
#ifdef CONFIG_OLED_HEIGHT_64
    ssd1306_cmd(0x3F);        /*   63  (for 64-row panel) */
#else
    ssd1306_cmd(0x1F);        /*   31  (for 32-row panel) */
#endif
    ssd1306_cmd(0xD3);        /* set display offset */
    ssd1306_cmd(0x00);        /*   0 */
    ssd1306_cmd(0x40);        /* set start line = 0  (0x40 | 0) */
    ssd1306_cmd(0x8D);        /* charge pump */
    ssd1306_cmd(0x14);        /*   enable */
    ssd1306_cmd(0x20);        /* set memory addressing mode */
    ssd1306_cmd(0x00);        /*   horizontal */
    ssd1306_cmd(0xA1);        /* segment re-map (col 127 → SEG0) */
    ssd1306_cmd(0xC8);        /* COM output scan direction: remapped */
    ssd1306_cmd(0xDA);        /* set COM pins hardware config */
#ifdef CONFIG_OLED_HEIGHT_64
    ssd1306_cmd(0x12);        /*   alternative COM config for 64-row panel */
#else
    ssd1306_cmd(0x02);        /*   sequential, no left-right remap (32-row) */
#endif
    ssd1306_cmd(0x81);        /* set contrast */
    ssd1306_cmd(0xCF);
    ssd1306_cmd(0xD9);        /* set pre-charge period */
    ssd1306_cmd(0xF1);
    ssd1306_cmd(0xDB);        /* set VCOMH deselect level */
    ssd1306_cmd(0x40);
    ssd1306_cmd(0xA4);        /* entire display on: resume from RAM */
    ssd1306_cmd(0xA6);        /* normal (non-inverted) display */
    ssd1306_cmd(0xAF);        /* display on */
}

/*
 * Set column/page address window to the full display, then transfer
 * the 512-byte framebuffer in one I2C transaction.
 *
 * At 400 kHz this takes ~13 ms.  Because oled_task runs at priority 2,
 * all GPS/NTP tasks (priority 5-6) preempt freely during the transfer.
 */
static void ssd1306_flush(void)
{
    /* Set addressing window: columns 0–127, pages 0–(FB_PAGES-1) */
    static const uint8_t addr[] = {
        0x00,                    /* control: commands */
        0x21, 0, 127,            /* set column address 0-127 */
        0x22, 0, FB_PAGES - 1,   /* set page address (3 for 32-row, 7 for 64-row) */
    };

    /* Hold the bus mutex across both transactions so no other driver
     * can interleave between the address-window command and pixel data.
     * An interleaved write would leave the SSD1306's internal column/page
     * pointer in an unknown position, causing display corruption. */
    i2c_bus_lock(I2C_PORT);

    i2c_master_write_to_device(I2C_PORT, I2C_ADDR,
                               addr, sizeof(addr),
                               pdMS_TO_TICKS(I2C_TIMEOUT_MS));

    /* Stream pixel data: s_tx[0]=0x40 (data control), s_tx[1..N]=fb */
    s_tx[0] = 0x40;
    i2c_master_write_to_device(I2C_PORT, I2C_ADDR,
                               s_tx, sizeof(s_tx),
                               pdMS_TO_TICKS(200));

    i2c_bus_unlock(I2C_PORT);
}

/* ------------------------------------------------------------------ */
/*  Display task                                                        */
/*                                                                      */
/*  128×32 layout:                                                      */
/*   x=0              x=96 x=128                                       */
/*  y=0  ┌────────────────┬─────┐                                      */
/*       │                │ SRC │  "GPS" / "NTP" / "---"  1× (6×8)    */
/*  y=8  │  HH:MM:SS      ├─────┤                                      */
/*       │  (2× font,     │.mmm │  milliseconds            1× (6×8)   */
/*  y=16 │   12×16 px)    └─────┤                                      */
/*       ├──────────────────────┤                                       */
/*  y=16 │ Lock GNSS Sat: 14    │  1× row  (talker from last RMC fix)  */
/*  y=24 ├──────────────────────┤                                       */
/*       │ rotating row         │  date / IP / NTP count / uptime      */
/*  y=32 └──────────────────────┘                                      */
/*                                                                      */
/*  128×64 layout (all rows static, no rotation):                      */
/*   x=0              x=96 x=128                                       */
/*  y=0  ┌────────────────┬─────┐                                      */
/*       │                │ GPS │  source label           1× (6×8)     */
/*  y=8  │  HH:MM:SS      ├─────┤                                      */
/*       │  (2× font)     │.mmm │  milliseconds           1× (6×8)     */
/*  y=16 ├────────────────┴─────┤                                      */
/*       │ S:1  YYYY-MM-DD UTC  │  stratum + date         1× (6×8)     */
/*  y=24 ├──────────────────────┤                                       */
/*       │ Lock GNSS Sat:14 PPS │  GNSS status            1× (6×8)     */
/*  y=32 ├──────────────────────┤                                       */
/*       │ IP: 192.168.1.100    │  IP address             1× (6×8)     */
/*  y=40 ├──────────────────────┤                                       */
/*       │ NTP:    12345 req    │  NTP request count      1× (6×8)     */
/*  y=48 ├──────────────────────┤                                       */
/*       │ Up:  2d 03:14:15     │  uptime                 1× (6×8)     */
/*  y=56 ├──────────────────────┤                                       */
/*       │ CPU: 43.2C           │  chip temperature       1× (6×8)     */
/*  y=64 └──────────────────────┘                                      */
/* ------------------------------------------------------------------ */

/*
 * Fill buf with the IP address of the first active network interface.
 * Tries WiFi STA then Ethernet; falls back to "---" if neither is up.
 */
static void get_ip_str(char *buf, size_t len)
{
    static const char *const keys[] = {"WIFI_STA_DEF", "ETH_DEF", NULL};
    for (int i = 0; keys[i]; i++) {
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey(keys[i]);
        if (!netif) continue;
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
            ip_info.ip.addr != 0) {
            snprintf(buf, len, IPSTR, IP2STR(&ip_info.ip));
            return;
        }
    }
    snprintf(buf, len, "---");
}

/*
 * Map a 2-char NMEA talker ID to a short display label (≤4 chars).
 *   GP → "GPS"    GN → "GNSS"   GL → "GLO"
 *   GA → "GAL"    GB → "BDS"    anything else → "???"
 */
#ifdef CONFIG_GPS_ENABLED
/* All labels are padded to 4 chars so "Sat:" stays at a fixed column. */
static const char *talker_label(const char t[3])
{
    if (t[0] == 'G') {
        switch (t[1]) {
        case 'P': return "GPS ";
        case 'N': return "GNSS";
        case 'L': return "GLO ";
        case 'A': return "GAL ";
        case 'B': return "BDS ";
        }
    }
    return t[0] ? "??? " : "--- ";   /* "--- " = no fix received yet */
}
#endif /* CONFIG_GPS_ENABLED */

/* Bottom row (y=24) rotates through 4 pages, each held for 5 s (50 × 100 ms).
 * Only used in the 128×32 layout. */
#define PAGE_HOLD_CYCLES  50
#define NUM_PAGES          4

/* Top-right source label alternates with stratum every 2 s (20 × 100 ms).
 * Only used in the 128×32 layout. */
#define SRC_HOLD_CYCLES  20

/* ------------------------------------------------------------------ */
/*  128×64 layout — all rows drawn at once, no rotation               */
/* ------------------------------------------------------------------ */

#ifdef CONFIG_OLED_HEIGHT_64
static void draw_layout_64(void)
{
    struct timeval tv;
    bool gps_ok  = gps_get_time(&tv);
    bool sntp_ok = !gps_ok && sntp_fallback_is_synced();
    bool rtc_ok  = !gps_ok && !sntp_ok && ds3231_is_valid();

    struct tm t;
    gmtime_r(&tv.tv_sec, &t);
    int ms = (int)(tv.tv_usec / 1000) % 1000;

    /* y=0..15: HH:MM:SS at 2× (x=0..95) */
    char timebuf[9];
    snprintf(timebuf, sizeof(timebuf), "%02d:%02d:%02d",
             t.tm_hour, t.tm_min, t.tm_sec);
    fb_draw_str_2x(0, 0, timebuf);

    /* y=0: source label (x=96) */
    fb_draw_str_1x(96, 0,
                   gps_ok ? "GPS" : sntp_ok ? "NTP" : rtc_ok ? "RTC" : "---");

    /* y=8: milliseconds (x=96) — sits directly alongside the 2× time */
    char msbuf[5];
    snprintf(msbuf, sizeof(msbuf), ".%03d", ms);
    fb_draw_str_1x(96, 8, msbuf);

    /* y=16: stratum + date + UTC */
    int stratum = gps_ok ? 1 : sntp_ok ? 2 : rtc_ok ? 12 : 16;
    char row16[22];
    snprintf(row16, sizeof(row16), "S:%-2d %04d-%02d-%02d UTC",
             stratum, t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
    fb_draw_str_1x(0, 16, row16);

    /* y=24: GNSS status */
#ifndef CONFIG_GPS_ENABLED
    fb_draw_str_1x(0, 24, "NO GPS MODULE");
#else
    if (!gps_hw_ok()) {
        fb_draw_str_1x(0, 24, "GPS MODULE ERROR");
    } else {
        int  sats = gps_get_sat_count();
        char talker[3];
        gps_get_fix_talker(talker);
        const char *constel = talker_label(talker);
        bool pps = gps_is_pps_locked();
        char gnss_row[22];
        if (sats >= 0)
            snprintf(gnss_row, sizeof(gnss_row), "%s %s Sat:%2d%s",
                     gps_ok ? "Lock " : "NoFix", constel, sats,
                     pps ? " PPS" : "");
        else
            snprintf(gnss_row, sizeof(gnss_row), "%s %s Sat:--%s",
                     gps_ok ? "Lock " : "NoFix", constel,
                     pps ? " PPS" : "");
        fb_draw_str_1x(0, 24, gnss_row);
    }
#endif

    /* y=32: IP address */
    char ipbuf[16];
    get_ip_str(ipbuf, sizeof(ipbuf));
    char row32[22];
    snprintf(row32, sizeof(row32), "IP:%-15s", ipbuf);
    fb_draw_str_1x(0, 32, row32);

    /* y=40: NTP request count */
    char row40[22];
    snprintf(row40, sizeof(row40), "NTP:%9lu req",
             (unsigned long)ntp_get_request_count());
    fb_draw_str_1x(0, 40, row40);

    /* y=48: uptime */
    int64_t up_us = esp_timer_get_time();
    int up_s = (int)(up_us / 1000000LL);
    char row48[22];
    snprintf(row48, sizeof(row48), "Up:%3dd %02d:%02d:%02d",
             up_s / 86400,
             (up_s % 86400) / 3600,
             (up_s % 3600) / 60,
             up_s % 60);
    fb_draw_str_1x(0, 48, row48);

    /* y=56: CPU chip temperature (ESP32-S3 only; blank on original ESP32) */
#if !CONFIG_IDF_TARGET_ESP32
    if (s_temp_sensor) {
        float temp_c = 0.0f;
        if (temperature_sensor_get_celsius(s_temp_sensor, &temp_c) == ESP_OK) {
            char row56[18];
            snprintf(row56, sizeof(row56), "CPU: %.1fC", (double)temp_c);
            fb_draw_str_1x(0, 56, row56);
        }
    }
#endif
}
#endif /* CONFIG_OLED_HEIGHT_64 */

static void oled_task(void *arg)
{
#ifdef CONFIG_OLED_HEIGHT_64

    /* 128×64: draw all rows every frame — no rotating pages needed */
    while (1) {
        fb_clear();
        draw_layout_64();
        ssd1306_flush();
        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz refresh */
    }

#else /* 128×32 */

    int  page             = 0;
    int  page_timer       = 0;
    int  src_timer        = 0;
    bool src_show_stratum = false;

    while (1) {
        fb_clear();

        /* ---------------------------------------------------------- */
        /*  Top half — always visible                                  */
        /* ---------------------------------------------------------- */
        struct timeval tv;
        bool gps_ok  = gps_get_time(&tv);
        bool sntp_ok = !gps_ok && sntp_fallback_is_synced();
        bool rtc_ok  = !gps_ok && !sntp_ok && ds3231_is_valid();

        struct tm t;
        gmtime_r(&tv.tv_sec, &t);
        int ms = (int)(tv.tv_usec / 1000) % 1000;  /* clamp 0-999 */

        /* Top-left: HH:MM:SS at 2× scale */
        char timebuf[9];
        snprintf(timebuf, sizeof(timebuf), "%02d:%02d:%02d",
                 t.tm_hour, t.tm_min, t.tm_sec);
        fb_draw_str_2x(0, 0, timebuf);

        /* Top-right: source label / stratum (y=0, alternating) + ms (y=8).
         * Both fields are 4 chars wide, drawn at x=102, to fit "S:16". */
        int stratum = gps_ok ? 1 : sntp_ok ? 2 : rtc_ok ? 12 : 16;
        char src_label[5];
        if (src_show_stratum)
            snprintf(src_label, sizeof(src_label), "S:%d", stratum);
        else
            snprintf(src_label, sizeof(src_label), "%s",
                     gps_ok ? "GPS" : sntp_ok ? "NTP" : rtc_ok ? "RTC" : "---");
        fb_draw_str_1x(102, 0, src_label);

        if (++src_timer >= SRC_HOLD_CYCLES) {
            src_timer = 0;
            src_show_stratum = !src_show_stratum;
        }

        char msbuf[8];
        snprintf(msbuf, sizeof(msbuf), ".%03d", ms);
        fb_draw_str_1x(96, 8, msbuf);

        /* ---------------------------------------------------------- */
        /*  y=16 — GNSS status, always visible                        */
        /* ---------------------------------------------------------- */
#ifndef CONFIG_GPS_ENABLED
        fb_draw_str_1x(0, 16, "NO GPS MODULE");
#else
        if (!gps_hw_ok()) {
            fb_draw_str_1x(0, 16, "GPS MODULE ERROR");
        } else {
            int  sats = gps_get_sat_count();
            char talker[3];
            gps_get_fix_talker(talker);
            const char *constel = talker_label(talker);
            bool pps = gps_is_pps_locked();
            char gnss_row[32];
            if (sats >= 0)
                snprintf(gnss_row, sizeof(gnss_row), "%s %s Sat:%2d%s",
                         gps_ok ? "Lock " : "NoFix", constel, sats,
                         pps ? " PPS" : "");
            else
                snprintf(gnss_row, sizeof(gnss_row), "%s %s Sat:--%s",
                         gps_ok ? "Lock " : "NoFix", constel,
                         pps ? " PPS" : "");
            fb_draw_str_1x(0, 16, gnss_row);
        }
#endif

        /* ---------------------------------------------------------- */
        /*  y=24 — single rotating row                                */
        /* ---------------------------------------------------------- */
        char row[32];

        switch (page) {
        case 0:
            snprintf(row, sizeof(row), "%04d-%02d-%02d  UTC",
                     t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
            break;
        case 1: {
            char ipbuf[16];
            get_ip_str(ipbuf, sizeof(ipbuf));
            snprintf(row, sizeof(row), "IP:%-15s", ipbuf);
            break;
        }
        case 2:
            snprintf(row, sizeof(row), "NTP:%9lu req",
                     (unsigned long)ntp_get_request_count());
            break;
        case 3: {
            int64_t up_us = esp_timer_get_time();
            int up_s = (int)(up_us / 1000000LL);
            snprintf(row, sizeof(row), "Up:%3dd %02d:%02d:%02d",
                     up_s / 86400,
                     (up_s % 86400) / 3600,
                     (up_s % 3600) / 60,
                     up_s % 60);
            break;
        }
        default:
            row[0] = '\0';
            break;
        }

        fb_draw_str_1x(0, 24, row);

        ssd1306_flush();

        if (++page_timer >= PAGE_HOLD_CYCLES) {
            page_timer = 0;
            page = (page + 1) % NUM_PAGES;
        }

        vTaskDelay(pdMS_TO_TICKS(100));   /* 10 Hz refresh */
    }

#endif /* CONFIG_OLED_HEIGHT_64 / 128×32 */
}

#endif /* CONFIG_OLED_ENABLED */

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

/*
 * Always defined (regardless of CONFIG_OLED_ENABLED) so the linker
 * can always resolve calls from main.c.  The body is a no-op when
 * the OLED feature is disabled.
 */
void oled_init(void)
{
#ifdef CONFIG_OLED_ENABLED
    /* I2C master — 400 kHz fast mode */
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = CONFIG_OLED_SDA_PIN,
        .scl_io_num       = CONFIG_OLED_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    /* Create the per-port mutex before installing the driver so it is
     * ready before any task can use the bus.  Idempotent if rtc.c
     * already called i2c_bus_mutex_init for the same port. */
    i2c_bus_mutex_init(I2C_PORT);

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
    /* Tolerate "already installed" — ESP_ERR_INVALID_STATE in newer IDF,
     * ESP_FAIL in some IDF 5.x builds — so the DS3231 and OLED can share. */
    esp_err_t i2c_err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (i2c_err == ESP_ERR_INVALID_STATE || i2c_err == ESP_FAIL) {
        ESP_LOGI(TAG, "I2C%d already installed — sharing bus", CONFIG_OLED_I2C_PORT);
    } else if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(i2c_err));
        return;
    }

    ssd1306_init();
    ESP_LOGI(TAG, "SSD1306 128x%d ready  I2C%d  SDA=%d  SCL=%d  addr=0x%02X",
             FB_H,
             CONFIG_OLED_I2C_PORT, CONFIG_OLED_SDA_PIN,
             CONFIG_OLED_SCL_PIN,  CONFIG_OLED_I2C_ADDR);

    /* Temperature sensor — not available on original ESP32 */
#if !CONFIG_IDF_TARGET_ESP32
    temperature_sensor_config_t ts_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&ts_cfg, &s_temp_sensor) == ESP_OK &&
        temperature_sensor_enable(s_temp_sensor) == ESP_OK) {
        ESP_LOGI(TAG, "CPU temperature sensor enabled");
    } else {
        ESP_LOGW(TAG, "CPU temperature sensor unavailable");
        s_temp_sensor = NULL;
    }
#endif

    xTaskCreate(oled_task, "oled", 3072, NULL, 2, NULL);
#endif /* CONFIG_OLED_ENABLED */
}
