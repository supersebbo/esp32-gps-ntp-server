#pragma once

/**
 * Start the OTA HTTP server and mark the current firmware slot as valid.
 * Call once after at least one network interface (WiFi or Ethernet) is up.
 * No-op when CONFIG_DEV_MODE_ENABLED is not set.
 *
 * Usage (when dev mode is enabled):
 *   curl -X POST http://<device-ip>:<OTA_PORT>/ota \
 *        --data-binary @build/esp32s3_ntp_server.bin
 *
 * If OTA_TOKEN is set, add: -H "X-OTA-Token: <token>"
 */
void ota_server_init(void);
