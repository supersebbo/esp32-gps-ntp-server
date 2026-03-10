#include "ethernet.h"
#include "sdkconfig.h"

#ifdef CONFIG_USE_ETHERNET

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"

/* SPI Ethernet MAC/PHY headers (IDF v5.x unified) */
#include "esp_eth_mac_spi.h"
#include "esp_eth_phy.h"
#include "esp_mac.h"

static const char *TAG = "ETH";

#define ETH_CONNECTED_BIT  BIT0

/* Map Kconfig SPI host choice to the ESP-IDF enum value */
#if CONFIG_ETH_USE_SPI2
#  define ETH_SPI_HOST  SPI2_HOST
#else
#  define ETH_SPI_HOST  SPI3_HOST
#endif

static EventGroupHandle_t  s_eth_events;
static esp_netif_t        *s_eth_netif;

/* ------------------------------------------------------------------ */
/*  Event handler                                                       */
/* ------------------------------------------------------------------ */

static void eth_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == ETH_EVENT && id == ETHERNET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Link up");
#ifdef CONFIG_USE_STATIC_IP
        {
            esp_netif_ip_info_t ip = {};
            ip4addr_aton(CONFIG_STATIC_IP_ADDR, (ip4_addr_t *)&ip.ip);
            ip4addr_aton(CONFIG_STATIC_NETMASK, (ip4_addr_t *)&ip.netmask);
            ip4addr_aton(CONFIG_STATIC_GATEWAY, (ip4_addr_t *)&ip.gw);
            esp_err_t e = esp_netif_dhcpc_stop(s_eth_netif);
            if (e != ESP_OK && e != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
                ESP_ERROR_CHECK(e);
            ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &ip));
            esp_netif_dns_info_t dns = {};
            dns.ip.type = IPADDR_TYPE_V4;
            ip4addr_aton(CONFIG_STATIC_DNS1, (ip4_addr_t *)&dns.ip.u_addr.ip4);
            ESP_ERROR_CHECK(esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_MAIN, &dns));
            if (strlen(CONFIG_STATIC_DNS2) > 0) {
                ip4addr_aton(CONFIG_STATIC_DNS2, (ip4_addr_t *)&dns.ip.u_addr.ip4);
                ESP_ERROR_CHECK(esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_BACKUP, &dns));
            }
            ESP_LOGI(TAG, "Static IP: %s  mask %s  gw %s",
                     CONFIG_STATIC_IP_ADDR, CONFIG_STATIC_NETMASK, CONFIG_STATIC_GATEWAY);
        }
#else
        ESP_LOGI(TAG, "Waiting for DHCP…");
#endif

    } else if (base == ETH_EVENT && id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "Link down");
        xEventGroupClearBits(s_eth_events, ETH_CONNECTED_BIT);

    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_eth_events, ETH_CONNECTED_BIT);
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void ethernet_init(void)
{
    s_eth_events = xEventGroupCreate();

    /* ---------------------------------------------------------------- */
    /*  SPI bus                                                          */
    /* ---------------------------------------------------------------- */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = CONFIG_ETH_SPI_MOSI_PIN,
        .miso_io_num   = CONFIG_ETH_SPI_MISO_PIN,
        .sclk_io_num   = CONFIG_ETH_SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* ---------------------------------------------------------------- */
    /*  SPI device — command/address bit widths differ per chip          */
    /* ---------------------------------------------------------------- */
    spi_device_interface_config_t devcfg = {
        .mode           = 0,
        .clock_speed_hz = CONFIG_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num   = CONFIG_ETH_SPI_CS_PIN,
        .queue_size     = 20,
        /* command_bits / address_bits omitted: IDF v5.x MAC drivers
         * handle SPI framing internally for W5500/DM9051/KSZ8851SNL */
    };

    /* ---------------------------------------------------------------- */
    /*  MAC and PHY                                                      */
    /* ---------------------------------------------------------------- */
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    esp_eth_mac_t *mac;
    esp_eth_phy_t *phy;

#if CONFIG_ETH_SPI_ETHERNET_W5500
    eth_w5500_config_t chip_cfg = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    chip_cfg.int_gpio_num     = CONFIG_ETH_SPI_INT_PIN;
    chip_cfg.poll_period_ms   = (CONFIG_ETH_SPI_INT_PIN < 0) ? 10 : 0;
    phy_config.reset_gpio_num = CONFIG_ETH_SPI_RST_PIN;
    mac = esp_eth_mac_new_w5500(&chip_cfg, &mac_config);
    phy = esp_eth_phy_new_w5500(&phy_config);

#elif CONFIG_ETH_SPI_ETHERNET_DM9051
    eth_dm9051_config_t chip_cfg = ETH_DM9051_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    chip_cfg.int_gpio_num     = CONFIG_ETH_SPI_INT_PIN;
    phy_config.reset_gpio_num = CONFIG_ETH_SPI_RST_PIN;
    mac = esp_eth_mac_new_dm9051(&chip_cfg, &mac_config);
    phy = esp_eth_phy_new_dm9051(&phy_config);

#elif CONFIG_ETH_SPI_ETHERNET_KSZ8851SNL
    eth_ksz8851snl_config_t chip_cfg = ETH_KSZ8851SNL_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    chip_cfg.int_gpio_num     = CONFIG_ETH_SPI_INT_PIN;
    phy_config.reset_gpio_num = CONFIG_ETH_SPI_RST_PIN;
    mac = esp_eth_mac_new_ksz8851snl(&chip_cfg, &mac_config);
    phy = esp_eth_phy_new_ksz8851snl(&phy_config);
#endif

    /* ---------------------------------------------------------------- */
    /*  Install Ethernet driver                                          */
    /* ---------------------------------------------------------------- */
    if (mac == NULL || phy == NULL) {
        ESP_LOGE(TAG, "MAC/PHY init failed — chip not responding or wrong SPI pins");
        if (mac) mac->del(mac);
        if (phy) phy->del(phy);
        return;
    }

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle;
    if (esp_eth_driver_install(&eth_cfg, &eth_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet driver install failed — NTP will continue without Ethernet");
        mac->del(mac);
        phy->del(phy);
        return;
    }

    /* Set MAC address from the ESP32 chip's built-in unique ID.
     * The W5500 has no OTP MAC of its own; without this it defaults to
     * 00:00:00:00:00:00 which most DHCP servers silently reject. */
    uint8_t mac_addr[6];
    esp_efuse_mac_get_default(mac_addr);
    mac_addr[0] |= 0x02;   /* set locally-administered bit */
    mac_addr[0] &= 0xFE;   /* clear multicast bit */
    esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr);
    ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    /* ---------------------------------------------------------------- */
    /*  Attach to lwIP / esp_netif                                       */
    /* ---------------------------------------------------------------- */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif,
                                     esp_eth_new_netif_glue(eth_handle)));

    /* ---------------------------------------------------------------- */
    /*  Register events and start                                        */
    /* ---------------------------------------------------------------- */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_ETH_GOT_IP, eth_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

#if CONFIG_ETH_SPI_ETHERNET_W5500
    const char *chip_name = "W5500";
#elif CONFIG_ETH_SPI_ETHERNET_DM9051
    const char *chip_name = "DM9051";
#else
    const char *chip_name = "KSZ8851SNL";
#endif
    ESP_LOGI(TAG, "Started: %s  SPI%d  %d MHz  "
                  "MOSI=%d MISO=%d SCLK=%d CS=%d INT=%d RST=%d",
             chip_name,
             CONFIG_ETH_USE_SPI2 ? 2 : 3,
             CONFIG_ETH_SPI_CLOCK_MHZ,
             CONFIG_ETH_SPI_MOSI_PIN, CONFIG_ETH_SPI_MISO_PIN,
             CONFIG_ETH_SPI_SCLK_PIN, CONFIG_ETH_SPI_CS_PIN,
             CONFIG_ETH_SPI_INT_PIN,  CONFIG_ETH_SPI_RST_PIN);

    /* Wait up to 15 s for a DHCP address */
    EventBits_t bits = xEventGroupWaitBits(s_eth_events, ETH_CONNECTED_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(15000));
    if (!(bits & ETH_CONNECTED_BIT))
        ESP_LOGW(TAG, "No IP yet — NTP server will start anyway");
}

#endif /* CONFIG_USE_ETHERNET */
