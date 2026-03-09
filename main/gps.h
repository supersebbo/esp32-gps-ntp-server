#pragma once

#include <stdbool.h>
#include <sys/time.h>

/**
 * Initialize the GPS UART driver and start the NMEA parsing task.
 * Parses $GPRMC / $GNRMC sentences for date/time.
 * If CONFIG_GPS_PPS_ENABLED, also installs a GPIO ISR on the PPS pin
 * to align timestamps precisely to the UTC second boundary.
 */
void gps_init(void);

/**
 * Returns true if the GPS hardware initialised successfully.
 * False if CONFIG_GPS_ENABLED is not set, or if UART/GPIO setup failed.
 */
bool gps_hw_ok(void);

/**
 * Get the current GPS-derived time with microsecond-level interpolation.
 *
 * When GPS has a valid fix the returned time is:
 *   last_fix_time + (esp_timer_get_time() - esp_timer_at_last_fix)
 *
 * With PPS enabled the reference is the PPS rising edge (±1 µs),
 * giving sub-millisecond NTP accuracy.  Without PPS the reference is
 * the NMEA sentence arrival time (~5–50 ms accuracy).
 *
 * Also falls back to gettimeofday() (system clock) when GPS is not
 * synced, so the caller always receives a plausible struct timeval.
 *
 * @param[out] tv_out  Filled with the current UTC time.
 * @return             true if GPS has a valid fix, false otherwise.
 */
bool gps_get_time(struct timeval *tv_out);

/**
 * Returns true when the GPS has reported at least one valid fix.
 */
bool gps_is_synced(void);

/**
 * Return the number of GNSS satellites currently in use, as reported
 * by the most recent valid $xxGGA sentence.
 * Returns -1 if no GGA sentence has been received yet.
 */
int gps_get_sat_count(void);

/**
 * Copy the 2-character NMEA talker ID of the sentence that produced
 * the most recent time fix into out[0..1], with a null terminator at
 * out[2].  Common values:
 *   "GP" — GPS-only solution
 *   "GN" — combined multi-GNSS solution (NEO-M8 default with multi-GNSS)
 *   "GL" — GLONASS
 *   "GA" — Galileo
 *   "GB" — BeiDou
 * Returns "\0\0" (empty string) if no fix has been received yet.
 *
 * @param[out] out  Caller-allocated 3-byte buffer.
 */
void gps_get_fix_talker(char out[3]);

/**
 * Get the GPS time at the most recent fix/PPS reference point.
 * Unlike gps_get_time(), this does NOT interpolate forward — it returns
 * the exact timestamp that was recorded at the last PPS edge or NMEA fix.
 * Used by the NTP server for the reference timestamp field (RFC 5905:
 * "time when the system clock was last set or corrected").
 *
 * @param[out] tv_out  Filled with the reference time.
 * @return             true if GPS has a valid fix, false otherwise.
 */
bool gps_get_last_fix_time(struct timeval *tv_out);

/**
 * Returns true when the PPS signal has fired within the last 1.1 seconds,
 * indicating the clock is disciplined to sub-millisecond accuracy.
 * Always returns false when CONFIG_GPS_PPS_ENABLED is not set.
 */
bool gps_is_pps_locked(void);

/**
 * Returns the esp_timer_get_time() value recorded at the most recent GPS
 * fix or PPS reference point, or 0 if no fix has ever been received.
 * Retains the last value even after GPS lock is lost, so it can be used
 * as a reference point for RTC holdover drift estimation.
 */
int64_t gps_last_fix_esp_us(void);
