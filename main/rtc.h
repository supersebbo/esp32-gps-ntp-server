#pragma once

#include <stdbool.h>
#include <sys/time.h>

/**
 * Initialise the DS3231 I2C RTC driver.
 *
 * If CONFIG_RTC_ENABLED is set:
 *   - Installs the I2C master driver (tolerates "already installed" so the
 *     DS3231 can share the OLED bus without conflict).
 *   - Checks the Oscillator Stop Flag (OSF).  If OSF is clear, reads the
 *     current time and calls settimeofday() to prime the system clock.
 *   - Logs a warning if OSF was set (oscillator was interrupted; wait for
 *     GPS/SNTP to sync and call rtc_sync_from() to calibrate).
 *
 * No-op when CONFIG_RTC_ENABLED is not set.
 */
void ds3231_init(void);

/**
 * Returns true if the I2C hardware initialised successfully and the DS3231
 * responded on the bus.  Always false when CONFIG_RTC_ENABLED is not set.
 */
bool ds3231_hw_ok(void);

/**
 * Returns true when the DS3231 holds a trustworthy time:
 *   - Hardware init succeeded, AND
 *   - Oscillator Stop Flag was clear at init (clock was never interrupted), OR
 *   - ds3231_sync_from() has been called at least once (which clears OSF).
 *
 * Always false when CONFIG_RTC_ENABLED is not set.
 */
bool ds3231_is_valid(void);

/**
 * Write a known-good time to the DS3231 and clear the OSF flag.
 * Call this whenever GPS or SNTP achieves a sync so the RTC stays accurate
 * across power cycles.
 *
 * No-op (safe to call unconditionally) when CONFIG_RTC_ENABLED is not set.
 */
void ds3231_sync_from(const struct timeval *tv);
