#pragma once

/**
 * Initialise the I2C bus, bring up the SSD1306, and start the
 * low-priority display task (priority 2).
 *
 * Must be called after gps_init() so that gps_get_time() and
 * gps_get_sat_count() are available.  Calling before the network
 * stack is up is fine — the display shows "---" until GPS/SNTP sync.
 *
 * Has no effect when CONFIG_OLED_ENABLED is not set.
 */
void oled_init(void);
