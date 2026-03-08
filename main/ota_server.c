#include "ota_server.h"
#include "sdkconfig.h"

#ifdef CONFIG_DEV_MODE_ENABLED

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_system.h"

static const char *TAG = "OTA";

#define OTA_BUF_SIZE 1024

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    /* Optional token authentication */
    if (strlen(CONFIG_OTA_TOKEN) > 0) {
        char token[64] = {0};
        if (httpd_req_get_hdr_value_str(req, "X-OTA-Token",
                                        token, sizeof(token)) != ESP_OK ||
            strcmp(token, CONFIG_OTA_TOKEN) != 0) {
            ESP_LOGW(TAG, "OTA rejected — bad or missing token");
            httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, "Invalid token\n");
            return ESP_FAIL;
        }
    }

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        ESP_LOGE(TAG, "No OTA partition found — is the partition table correct?");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "No OTA partition\n");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Writing to partition '%s' at 0x%08" PRIx32,
             update_part->label, update_part->address);

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES,
                                  &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "OTA begin failed\n");
        return ESP_FAIL;
    }

    char buf[OTA_BUF_SIZE];
    int  total = 0;

    while (1) {
        int ret = httpd_req_recv(req, buf, sizeof(buf));
        if (ret < 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "recv error %d — aborting", ret);
            esp_ota_abort(ota_handle);
            return ESP_FAIL;
        }
        if (ret == 0) break;    /* all data received */

        err = esp_ota_write(ota_handle, buf, ret);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "Flash write failed\n");
            return ESP_FAIL;
        }
        total += ret;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Image validation failed\n");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Set boot partition failed\n");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA complete (%d bytes) — rebooting", total);
    httpd_resp_sendstr(req, "OTA OK — rebooting\n");

    vTaskDelay(pdMS_TO_TICKS(1500));    /* let response flush before reset */
    esp_restart();
    return ESP_OK;
}

#endif /* CONFIG_DEV_MODE_ENABLED */

void ota_server_init(void)
{
#ifdef CONFIG_DEV_MODE_ENABLED
    /* Confirm this slot is good — prevents bootloader rollback on next reset */
    esp_ota_mark_app_valid_cancel_rollback();

    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = CONFIG_OTA_PORT;
    cfg.recv_wait_timeout = 30;         /* seconds — generous for large binaries */

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start OTA HTTP server");
        return;
    }

    static const httpd_uri_t ota_uri = {
        .uri     = "/ota",
        .method  = HTTP_POST,
        .handler = ota_post_handler,
    };
    httpd_register_uri_handler(server, &ota_uri);

    ESP_LOGI(TAG, "OTA server ready on port %d", CONFIG_OTA_PORT);
    ESP_LOGI(TAG, "  curl -X POST http://<ip>:%d/ota --data-binary @firmware.bin",
             CONFIG_OTA_PORT);
#endif
}
