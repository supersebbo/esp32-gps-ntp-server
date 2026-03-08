#include "rtc.h"
#include "sdkconfig.h"

#ifdef CONFIG_RTC_ENABLED

#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "i2c_bus.h"

static const char *TAG = "RTC";

/* DS3231 I2C address and key register addresses */
#define DS3231_ADDR        ((uint8_t)0x68)
#define DS3231_REG_TIME    0x00   /* seconds — burst-read 7 bytes for full time */
#define DS3231_REG_CTRL    0x0E   /* control register */
#define DS3231_REG_STATUS  0x0F   /* status register */
#define DS3231_OSF_BIT     0x80   /* Oscillator Stop Flag (bit 7 of status) */
#define DS3231_EOSC_BIT    0x80   /* Enable Oscillator (active-low, bit 7 of ctrl) */

#define I2C_PORT           ((i2c_port_t)CONFIG_RTC_I2C_PORT)
#define I2C_TIMEOUT_MS     50

static bool s_rtc_hw_ok = false;
static bool s_rtc_valid = false;   /* true when DS3231 has trustworthy time */

/* ------------------------------------------------------------------ */
/*  BCD helpers                                                         */
/* ------------------------------------------------------------------ */

static inline int     bcd_to_dec(uint8_t b) { return (b >> 4) * 10 + (b & 0x0F); }
static inline uint8_t dec_to_bcd(int    v)  { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

/* ------------------------------------------------------------------ */
/*  Low-level I2C helpers                                              */
/* ------------------------------------------------------------------ */

static esp_err_t ds3231_read(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_bus_lock(I2C_PORT);
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, DS3231_ADDR,
                                                 &reg, 1, buf, len,
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_bus_unlock(I2C_PORT);
    return err;
}

static esp_err_t ds3231_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t frame[2] = {reg, val};
    i2c_bus_lock(I2C_PORT);
    esp_err_t err = i2c_master_write_to_device(I2C_PORT, DS3231_ADDR,
                                               frame, sizeof(frame),
                                               pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_bus_unlock(I2C_PORT);
    return err;
}

static esp_err_t ds3231_write_time(const uint8_t regs[7])
{
    uint8_t frame[8];
    frame[0] = DS3231_REG_TIME;
    memcpy(frame + 1, regs, 7);
    i2c_bus_lock(I2C_PORT);
    esp_err_t err = i2c_master_write_to_device(I2C_PORT, DS3231_ADDR,
                                               frame, sizeof(frame),
                                               pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_bus_unlock(I2C_PORT);
    return err;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void ds3231_init(void)
{
    /* Create the per-port bus mutex before any I2C transactions.
     * This runs before oled_init() in app_main, so the mutex exists
     * before the oled_task is created. */
    i2c_bus_mutex_init(I2C_PORT);

    /* Configure I2C master at 400 kHz.  i2c_param_config is safe to call
     * even if the driver is already running (idempotent when pins match). */
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = CONFIG_RTC_SDA_PIN,
        .scl_io_num       = CONFIG_RTC_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return;
    }

    /* Install driver — tolerate "already installed": ESP_ERR_INVALID_STATE in
     * newer IDF, ESP_FAIL in some IDF 5.x builds — so RTC can share with OLED. */
    err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err == ESP_ERR_INVALID_STATE || err == ESP_FAIL) {
        ESP_LOGI(TAG, "I2C%d already installed — sharing bus with OLED",
                 CONFIG_RTC_I2C_PORT);
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return;
    }
    s_rtc_hw_ok = true;

    /* Ensure oscillator runs on battery (EOSC# = 0).
     * Factory default is already 0; this guards against a mis-configured module. */
    uint8_t ctrl = 0;
    if (ds3231_read(DS3231_REG_CTRL, &ctrl, 1) == ESP_OK && (ctrl & DS3231_EOSC_BIT)) {
        ds3231_write_reg(DS3231_REG_CTRL, ctrl & ~DS3231_EOSC_BIT);
        ESP_LOGW(TAG, "DS3231 EOSC was set — enabled battery oscillator");
    }

    /* Read status register: check Oscillator Stop Flag */
    uint8_t status = 0;
    err = ds3231_read(DS3231_REG_STATUS, &status, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DS3231 not found at 0x%02X on I2C%d: %s",
                 DS3231_ADDR, CONFIG_RTC_I2C_PORT, esp_err_to_name(err));
        return;
    }

    if (status & DS3231_OSF_BIT) {
        ESP_LOGW(TAG, "DS3231 OSF set — oscillator was stopped, time unreliable."
                      " Waiting for GPS/SNTP sync to calibrate.");
        return;   /* s_rtc_valid stays false */
    }

    /* Read 7 time registers: sec, min, hr, dow, date, month, year */
    uint8_t regs[7];
    err = ds3231_read(DS3231_REG_TIME, regs, 7);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DS3231 time read failed: %s", esp_err_to_name(err));
        return;
    }

    /* Decode BCD — always use 24-hour mode (bit 6 of hours reg = 0) */
    int sec  = bcd_to_dec(regs[0] & 0x7F);
    int min  = bcd_to_dec(regs[1] & 0x7F);
    int hr;
    if (regs[2] & 0x40) {
        /* Module was set to 12-hour mode — convert to 24-hour */
        hr = bcd_to_dec(regs[2] & 0x1F);
        if (regs[2] & 0x20) hr += 12;
        if (hr == 24) hr = 0;
    } else {
        hr = bcd_to_dec(regs[2] & 0x3F);
    }
    /* regs[3] = day of week (1–7): not needed for epoch calculation */
    int mday    = bcd_to_dec(regs[4] & 0x3F);
    int month   = bcd_to_dec(regs[5] & 0x1F);
    int century = (regs[5] & 0x80) ? 2000 : 1900;
    int year    = bcd_to_dec(regs[6]) + century;

    /* Basic sanity check before trusting the values */
    if (month < 1 || month > 12 || mday < 1 || mday > 31 ||
        hr > 23 || min > 59 || sec > 59 || year < 2000 || year > 2099) {
        ESP_LOGW(TAG, "DS3231 implausible time: %04d-%02d-%02d %02d:%02d:%02d",
                 year, month, mday, hr, min, sec);
        return;
    }

    /* Convert to Unix epoch and set system clock.
     * mktime() treats tm as local time.  ESP-IDF defaults TZ to UTC so
     * mktime() == timegm() in this project. */
    struct tm t = {
        .tm_sec  = sec,
        .tm_min  = min,
        .tm_hour = hr,
        .tm_mday = mday,
        .tm_mon  = month - 1,
        .tm_year = year - 1900,
        .tm_isdst = 0,
    };
    time_t epoch = mktime(&t);
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    s_rtc_valid = true;
    ESP_LOGI(TAG, "DS3231 loaded: %04d-%02d-%02d %02d:%02d:%02d UTC  I2C%d  SDA=%d  SCL=%d",
             year, month, mday, hr, min, sec,
             CONFIG_RTC_I2C_PORT, CONFIG_RTC_SDA_PIN, CONFIG_RTC_SCL_PIN);
}

bool ds3231_hw_ok(void)
{
    return s_rtc_hw_ok;
}

bool ds3231_is_valid(void)
{
    return s_rtc_valid;
}

void ds3231_sync_from(const struct timeval *tv)
{
    if (!s_rtc_hw_ok) return;

    struct tm t;
    gmtime_r(&tv->tv_sec, &t);

    int year  = t.tm_year + 1900;
    int month = t.tm_mon  + 1;

    /* Build 7 time register bytes (24-hour mode, century bit set for 2000+) */
    uint8_t regs[7];
    regs[0] = dec_to_bcd(t.tm_sec);
    regs[1] = dec_to_bcd(t.tm_min);
    regs[2] = dec_to_bcd(t.tm_hour);        /* bit6=0 → 24-hour mode */
    regs[3] = 1;                              /* day of week unused — set to 1 */
    regs[4] = dec_to_bcd(t.tm_mday);
    regs[5] = dec_to_bcd(month);
    if (year >= 2000) regs[5] |= 0x80;       /* century bit */
    regs[6] = dec_to_bcd(year % 100);

    esp_err_t err = ds3231_write_time(regs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DS3231 write failed: %s", esp_err_to_name(err));
        return;
    }

    /* Clear the Oscillator Stop Flag now that time is valid */
    uint8_t status = 0;
    if (ds3231_read(DS3231_REG_STATUS, &status, 1) == ESP_OK) {
        ds3231_write_reg(DS3231_REG_STATUS, status & ~DS3231_OSF_BIT);
    }

    static bool s_first = true;
    if (s_first) {
        ESP_LOGI(TAG, "DS3231 first sync: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 year, month, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
        s_first = false;
    } else {
        ESP_LOGD(TAG, "DS3231 updated: %04d-%02d-%02d %02d:%02d:%02d UTC",
                 year, month, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    }

    s_rtc_valid = true;
}

#else /* CONFIG_RTC_ENABLED not set */

void ds3231_init(void)                          {}
bool ds3231_hw_ok(void)                         { return false; }
bool ds3231_is_valid(void)                      { return false; }
void ds3231_sync_from(const struct timeval *tv) { (void)tv; }

#endif /* CONFIG_RTC_ENABLED */
