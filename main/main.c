#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "gps.h"
#include "ntp_server.h"
#include "sntp_fallback.h"
#include "rtc.h"
#ifdef CONFIG_USE_ETHERNET
#include "ethernet.h"
#endif
#include "oled.h"
#include "ota_server.h"
#include "log_server.h"
#include "sdkconfig.h"

static const char *TAG = "MAIN";

/* ------------------------------------------------------------------ */
/*  WiFi (optional — disabled when CONFIG_USE_WIFI is not set)         */
/* ------------------------------------------------------------------ */

#ifdef CONFIG_USE_WIFI

#include "freertos/event_groups.h"
#include "esp_wifi.h"

#define WIFI_CONNECTED_BIT  BIT0

static EventGroupHandle_t  s_wifi_events;
static esp_netif_t        *s_sta_netif;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
#ifdef CONFIG_USE_STATIC_IP
        {
            esp_netif_ip_info_t ip = {};
            ip4addr_aton(CONFIG_STATIC_IP_ADDR, (ip4_addr_t *)&ip.ip);
            ip4addr_aton(CONFIG_STATIC_NETMASK, (ip4_addr_t *)&ip.netmask);
            ip4addr_aton(CONFIG_STATIC_GATEWAY, (ip4_addr_t *)&ip.gw);
            esp_err_t e = esp_netif_dhcpc_stop(s_sta_netif);
            if (e != ESP_OK && e != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
                ESP_ERROR_CHECK(e);
            ESP_ERROR_CHECK(esp_netif_set_ip_info(s_sta_netif, &ip));
            esp_netif_dns_info_t dns = {};
            dns.ip.type = IPADDR_TYPE_V4;
            ip4addr_aton(CONFIG_STATIC_DNS1, (ip4_addr_t *)&dns.ip.u_addr.ip4);
            ESP_ERROR_CHECK(esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns));
            if (strlen(CONFIG_STATIC_DNS2) > 0) {
                ip4addr_aton(CONFIG_STATIC_DNS2, (ip4_addr_t *)&dns.ip.u_addr.ip4);
                ESP_ERROR_CHECK(esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_BACKUP, &dns));
            }
            ESP_LOGI(TAG, "Static IP: %s  mask %s  gw %s",
                     CONFIG_STATIC_IP_ADDR, CONFIG_STATIC_NETMASK, CONFIG_STATIC_GATEWAY);
        }
#endif

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi disconnected — reconnecting…");
        esp_wifi_connect();

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi connected. IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    /* esp_netif_init() and esp_event_loop_create_default() are called
     * unconditionally in app_main() before this function. */
    s_wifi_events = xEventGroupCreate();

    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid               = CONFIG_WIFI_SSID,
            .password           = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg.capable    = true,
        },
    };
    /* Allow open networks when password is empty */
    if (strlen(CONFIG_WIFI_PASSWORD) == 0)
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   /* disable power-save for low-latency NTP */
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID: %s", CONFIG_WIFI_SSID);

    /*
     * Wait up to 15 s for the initial connection.
     * The handler retries indefinitely so the NTP server remains
     * usable even if WiFi drops and later recovers.
     */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED_BIT))
        ESP_LOGW(TAG, "WiFi not yet connected — NTP server will start anyway");
}

#endif /* CONFIG_USE_WIFI */

/* ------------------------------------------------------------------ */
/*  app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    /* Hold the onboard WS2812B data line low so it doesn't latch a
     * random colour from boot noise (GPIO21 on WaveShare ESP32-S3-ETH). */
    gpio_set_direction(21, GPIO_MODE_OUTPUT);
    gpio_set_level(21, 0);

    /* Install log hook first — buffers all output until a remote client
     * connects, so boot messages are not lost.  No-op if dev mode off. */
    log_server_early_init();

    /* NVS is required by the WiFi driver and used by esp_netif */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /*
     * These two must be called before any networking component
     * (WiFi, Ethernet, SNTP, sockets) — do it unconditionally here
     * so the path is the same whether WiFi is enabled or not.
     */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "ESP32-S3 GPS NTP Server  (stratum 1 / GPS, stratum 2 / %s fallback)",
             CONFIG_NTP_FALLBACK_SERVER);
#ifdef CONFIG_GPS_ENABLED
    ESP_LOGI(TAG, "GPS : UART%d  RX=GPIO%d  TX=GPIO%d  %d baud",
             CONFIG_GPS_UART_NUM, CONFIG_GPS_RX_PIN,
             CONFIG_GPS_TX_PIN,   CONFIG_GPS_BAUD_RATE);
#  ifdef CONFIG_GPS_PPS_ENABLED
    ESP_LOGI(TAG, "PPS : GPIO%d", CONFIG_GPS_PPS_PIN);
#  endif
#else
    ESP_LOGI(TAG, "GPS : disabled — SNTP fallback only (stratum 2)");
#endif
#ifdef CONFIG_USE_WIFI
    ESP_LOGI(TAG, "Net : WiFi STA (%s)", CONFIG_WIFI_SSID);
#endif
#ifdef CONFIG_USE_ETHERNET
    ESP_LOGI(TAG, "Net : SPI Ethernet enabled");
#endif
#if !defined(CONFIG_USE_WIFI) && !defined(CONFIG_USE_ETHERNET)
    ESP_LOGW(TAG, "Net : no network interface enabled — NTP will not be reachable");
#endif

    /* Read DS3231 RTC first so the system clock is valid before GPS/SNTP sync.
     * rtc_init() is a no-op when CONFIG_RTC_ENABLED is not set. */
    ds3231_init();

    /* Start GPS UART reader immediately — acquisition begins now */
    gps_init();

    /* Bring up I2C OLED — low-priority task, non-blocking for NTP/GPS */
    oled_init();

#ifdef CONFIG_USE_WIFI
    /* Connect to WiFi (blocks up to 15 s for first connection) */
    wifi_init_sta();
#endif

#ifdef CONFIG_USE_ETHERNET
    /* Bring up SPI Ethernet (blocks up to 15 s for DHCP) */
    ethernet_init();
#endif

    /* Start SNTP fallback client — syncs from upstream when GPS unavailable */
    sntp_fallback_init();

    /* Start NTP server — serves requests once a network interface is up */
    ntp_server_start();

    /* OTA push server + remote log mirror (no-ops when dev mode is disabled) */
    ota_server_init();
    log_server_init();

    /* Periodic status log + RTC sync-back (once per hour when synced) */
    static int rtc_sync_countdown = 0;
    while (1) {
        struct timeval tv;
        bool gps_ok  = gps_get_time(&tv);
        bool sntp_ok = sntp_fallback_is_synced();
        bool rtc_ok  = ds3231_is_valid();

        /* Sync DS3231 on first lock, then every hour (360 × 10 s loops) */
        if ((gps_ok || sntp_ok) && rtc_sync_countdown-- <= 0) {
            ds3231_sync_from(&tv);
            rtc_sync_countdown = 360;
        }

        struct tm t;
        gmtime_r(&tv.tv_sec, &t);

        const char *src = gps_ok  ? "GPS   " :
                          sntp_ok ? "SNTP  " :
                          rtc_ok  ? "RTC   " : "UNSYNC";

        ESP_LOGI(TAG, "[%s str=%d] %04d-%02d-%02d %02d:%02d:%02d.%06ld UTC",
                 src,
                 gps_ok ? 1 : sntp_ok ? 2 : rtc_ok ? 12 : 16,
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec,
                 (long)tv.tv_usec);

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
