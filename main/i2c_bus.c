#include "i2c_bus.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "I2C_BUS";

/* One mutex per I2C port (I2C_NUM_MAX is 2 on ESP32/ESP32-S3). */
static SemaphoreHandle_t s_mutex[I2C_NUM_MAX];

void i2c_bus_mutex_init(i2c_port_t port)
{
    if ((unsigned)port >= I2C_NUM_MAX) return;
    if (s_mutex[port] == NULL) {
        s_mutex[port] = xSemaphoreCreateMutex();
        if (s_mutex[port] == NULL)
            ESP_LOGE(TAG, "Failed to create mutex for I2C%d", (int)port);
        else
            ESP_LOGD(TAG, "Bus mutex created for I2C%d", (int)port);
    }
}

void i2c_bus_lock(i2c_port_t port)
{
    if ((unsigned)port < I2C_NUM_MAX && s_mutex[port])
        xSemaphoreTake(s_mutex[port], portMAX_DELAY);
}

void i2c_bus_unlock(i2c_port_t port)
{
    if ((unsigned)port < I2C_NUM_MAX && s_mutex[port])
        xSemaphoreGive(s_mutex[port]);
}
