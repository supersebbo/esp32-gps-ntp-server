# ESP32 GPS NTP Server

A GPS-disciplined stratum-1 NTP server for ESP32 and ESP32-S3, built with ESP-IDF 5.x.

Achieves sub-millisecond accuracy using a NEO-M8 GPS module with PPS (pulse-per-second) output. Falls back to a configurable upstream SNTP server (stratum 2) when GPS is unavailable. Supports wired Ethernet (W5500/DM9051/KSZ8851SNL via SPI) and/or WiFi.

---

## Features

- **Stratum 1** NTP when GPS lock + PPS is active (<1 ms accuracy)
- **Stratum 2** SNTP fallback when GPS is unavailable (~10 ms accuracy)
- **Stratum 12** RTC holdover via optional DS3231 (survives power cycles)
- SPI Ethernet support: W5500, DM9051, KSZ8851SNL
- WiFi STA support
- Optional SSD1306 OLED status display (128×32 or 128×64)
- Optional DS3231 RTC for pre-GPS time at boot
- Multi-constellation GNSS: GPS + GLONASS + Galileo + SBAS (NEO-M8)
- Developer mode: OTA firmware update + remote log server (disabled by default)
- Crystal drift correction via 8-sample EMA of PPS interval
- All options configurable via `idf.py menuconfig` — no code changes required

---

## Hardware

### Required

| Component | Notes |
|-----------|-------|
| ESP32 or ESP32-S3 | Any module with at least 2 MB flash |
| u-blox NEO-M8 GPS module | NEO-M8N or NEO-M8M recommended |
| Active GPS antenna | Patch antenna or external with clear sky view |

### Optional

| Component | Notes |
|-----------|-------|
| W5500 / DM9051 / KSZ8851SNL | SPI Ethernet for wired NTP serving |
| SSD1306 OLED | 128×32 (0.91") or 128×64 (0.96"), I2C |
| DS3231 RTC module | Battery-backed I2C RTC, shares bus with OLED |

---

## Wiring

All pins are configurable in `menuconfig`. The defaults are:

### GPS (UART)

| GPS Pin | ESP32 GPIO | ESP32-S3 GPIO |
|---------|-----------|---------------|
| TX (→ ESP RX) | GPIO 16 | GPIO 18 |
| RX (← ESP TX) | GPIO 17 | GPIO 17 |
| PPS | GPIO 4 | GPIO 4 |
| VCC | 3.3 V | 3.3 V |
| GND | GND | GND |

### SPI Ethernet (W5500 shown)

| W5500 Pin | ESP32 GPIO | ESP32-S3 GPIO |
|-----------|-----------|---------------|
| MOSI | GPIO 23 | GPIO 11 |
| MISO | GPIO 19 | GPIO 12 |
| SCLK | GPIO 18 | GPIO 13 |
| CS | GPIO 5 | GPIO 14 |
| INT | GPIO 34 | GPIO 10 |
| RST | GPIO 27 | GPIO 9 |
| VCC | 3.3 V | 3.3 V |
| GND | GND | GND |

### OLED + DS3231 (shared I2C bus)

| Module Pin | ESP32 GPIO | ESP32-S3 GPIO |
|------------|-----------|---------------|
| SDA | GPIO 21 | GPIO 5 |
| SCL | GPIO 22 | GPIO 6 |
| VCC | 3.3 V | 3.3 V |
| GND | GND | GND |

The OLED (address `0x3C`) and DS3231 (address `0x68`) can share the same I2C bus — just wire SDA/SCL of both modules to the same GPIOs.

---

## Getting Started

### 1. Install ESP-IDF

If you haven't used ESP-IDF before, follow Espressif's official setup guide for your OS:

- [Windows](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html)
- [Linux / macOS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html)

This project requires **ESP-IDF 5.x** (tested on 5.5.x). The VS Code [ESP-IDF Extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) is the easiest way to get started on any platform — it handles toolchain installation and provides build/flash buttons.

Alternatively, a [Dev Container](.devcontainer/) is included — if you have Docker and the VS Code [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers), open the repo and click **"Reopen in Container"** to get a fully configured environment with no local installation required.

### 2. Clone and Configure

```bash
git clone https://github.com/supersebbo/esp32-gps-ntp-server.git
cd esp32-gps-ntp-server

# Open the interactive configuration menu
idf.py menuconfig
```

In menuconfig, go to **"GPS NTP Server Configuration"** and set at minimum:
- Your WiFi credentials (if using WiFi)
- GPIO pins to match your wiring
- Which optional modules to enable (OLED, RTC, Ethernet)

### 3. Build

```bash
idf.py build
```

### 4. Flash

Connect your ESP32 via USB, then:

```bash
# Linux / macOS
idf.py -p /dev/ttyUSB0 flash monitor

# Windows
idf.py -p COM3 flash monitor
```

Replace the port with whatever your device enumerates as. The `monitor` target opens the serial console so you can see boot output immediately.

> **First flash note:** if you enable Developer Mode (OTA), the custom partition table (`partitions.csv`) must be flashed via USB at least once. Subsequent updates can use OTA.

---

## Software Prerequisites

- [ESP-IDF 5.x](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/) installed and activated (`idf.py` on PATH)
- Tested with ESP-IDF **5.5.x**

---

## Configuration

All options are under **"GPS NTP Server Configuration"** in `idf.py menuconfig`.

| Option | Default | Description |
|--------|---------|-------------|
| `USE_WIFI` | n | Enable WiFi STA |
| `USE_ETHERNET` | y | Enable SPI Ethernet |
| `GPS_ENABLED` | y | Enable GPS UART + NMEA parser |
| `GPS_PPS_ENABLED` | y | Enable PPS for sub-ms accuracy |
| `GPS_GNSS_GLONASS` | y | Enable GLONASS |
| `GPS_GNSS_GALILEO` | y | Enable Galileo |
| `GPS_GNSS_BEIDOU` | n | Enable BeiDou (note: NEO-M8 max 3 constellations) |
| `GPS_GNSS_SBAS` | y | Enable SBAS augmentation |
| `OLED_ENABLED` | y | Enable SSD1306 OLED |
| `OLED_HEIGHT_32/64` | 32 | Display height |
| `RTC_ENABLED` | n | Enable DS3231 RTC |
| `USE_STATIC_IP` | n | Static IP instead of DHCP |
| `NTP_FALLBACK_SERVER` | pool.ntp.org | Upstream SNTP server |
| `DEV_MODE_ENABLED` | n | OTA + remote log server |

---

## Pointing NTP Clients at the Server

### Linux / macOS (`/etc/ntp.conf` or `chrony.conf`)

```
server 192.168.1.x iburst prefer
```

### Windows

```
w32tm /config /manualpeerlist:"192.168.1.x" /syncfromflags:manual /reliable:YES /update
```

### Router (e.g. OpenWrt / pfSense)

Set the LAN NTP server to the device's IP in DHCP options. All clients on the network will then use it automatically.

---

## OLED Display

The 128×32 layout shows:
- Large time (top left), sync source + accuracy (top right)
- GNSS satellite count and constellation status
- Rotating bottom row: date / IP address / NTP request count / uptime

The 128×64 layout shows all fields simultaneously without rotation.

---

## Developer Mode (disabled by default)

Enable `DEV_MODE_ENABLED` in menuconfig to activate:

### OTA Firmware Update

```bash
curl -X POST http://<device-ip>:3232/ota \
     -H "X-OTA-Token: <your-token>" \
     --data-binary @build/esp32-gps-ntp-server.bin
```

The device validates the new image on boot. If it crashes before completing startup, the bootloader automatically rolls back to the previous firmware.

### Remote Log Server

```bash
nc <device-ip> 4444
```

Streams all ESP log output over TCP. Boot messages from before the network came up are buffered and replayed on connection.

### VS Code Tasks

With the workspace open, use **Terminal → Run Task** to access:
- `OTA: Upload firmware`
- `OTA: Build then upload firmware`
- `Monitor: Remote serial (nc)`

---

## NTP Stratum Behaviour

| Condition | Stratum | Reference |
|-----------|---------|-----------|
| GPS lock + PPS | 1 | `GPS\0` |
| GPS lock, no PPS | 1 | `GPS\0` |
| SNTP fallback only | 2 | upstream server |
| DS3231 RTC only | 12 | `RTC\0` |
| No sync | 16 | (unsynchronised) |

---

## Accuracy

| Mode | Typical accuracy |
|------|-----------------|
| GPS + PPS | < 1 ms |
| GPS only (NMEA) | ~10 ms |
| SNTP fallback | ~10–50 ms (network dependent) |
| DS3231 holdover | drifts ~2 ppm (~170 ms/day) |

---

## Licence

MIT
