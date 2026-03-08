#pragma once

/**
 * Install the vprintf hook as early as possible (first line of app_main).
 * Log output is buffered in a ring until a remote client connects, so no
 * boot messages are lost.  No-op when CONFIG_DEV_MODE_ENABLED is not set.
 */
void log_server_early_init(void);

/**
 * Start the TCP listener task.  Call after at least one network interface
 * is up.  No-op when CONFIG_DEV_MODE_ENABLED is not set.
 *
 * Connect with:
 *   nc <device-ip> <LOG_SERVER_PORT>
 */
void log_server_init(void);
