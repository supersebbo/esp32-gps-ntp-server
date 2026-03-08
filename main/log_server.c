#include "log_server.h"
#include "sdkconfig.h"

#ifdef CONFIG_DEV_MODE_ENABLED

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "LOGSVR";

/* fd of the currently connected client, or -1 */
static volatile int s_client = -1;

/* ---------------------------------------------------------------------------
 * Boot log ring buffer.
 * Stores output from boot until a remote client first connects, so early
 * messages are not lost.  Fixed static allocation — size set in Kconfig.
 * When full, oldest bytes are silently dropped (ring overwrites itself).
 * Access is protected by a portMUX spinlock (safe from any task context).
 * --------------------------------------------------------------------------*/
#ifndef CONFIG_LOG_RING_BUFFER_SIZE
#define CONFIG_LOG_RING_BUFFER_SIZE 4096
#endif
#define RING_SIZE CONFIG_LOG_RING_BUFFER_SIZE

static char         s_ring[RING_SIZE];
static size_t       s_ring_head = 0;    /* oldest byte */
static size_t       s_ring_tail = 0;    /* next write position */
static bool         s_ring_full = false;
static portMUX_TYPE s_ring_mux  = portMUX_INITIALIZER_UNLOCKED;

static void ring_write(const char *buf, size_t len)
{
    portENTER_CRITICAL(&s_ring_mux);
    for (size_t i = 0; i < len; i++) {
        s_ring[s_ring_tail] = buf[i];
        s_ring_tail = (s_ring_tail + 1) % RING_SIZE;
        if (s_ring_full) {
            /* Overwrite: advance head to discard the oldest byte */
            s_ring_head = (s_ring_head + 1) % RING_SIZE;
        }
        s_ring_full = (s_ring_tail == s_ring_head);
    }
    portEXIT_CRITICAL(&s_ring_mux);
}

/* Drain the ring buffer to fd, then clear it.  Called once on client connect
 * before switching to live streaming.  send() calls are outside the spinlock
 * to avoid holding it across a blocking syscall. */
static void ring_drain(int fd)
{
    /* Snapshot and reset under lock */
    portENTER_CRITICAL(&s_ring_mux);
    size_t head = s_ring_head;
    size_t tail = s_ring_tail;
    bool   full = s_ring_full;
    s_ring_head = s_ring_tail = 0;
    s_ring_full = false;
    portEXIT_CRITICAL(&s_ring_mux);

    if (head == tail && !full) return;  /* nothing buffered */

    if (full || tail <= head) {
        /* Wrapped: send head→end, then 0→tail */
        send(fd, s_ring + head, RING_SIZE - head, 0);
        if (tail > 0) send(fd, s_ring, tail, 0);
    } else {
        send(fd, s_ring + head, tail - head, 0);
    }
}

/* ---------------------------------------------------------------------------
 * Custom vprintf: format once, write to UART and to any connected TCP client.
 * When no client is connected, buffer into the ring for later replay.
 * Called from the ESP logging subsystem — may run on any task context.
 * --------------------------------------------------------------------------*/
static int remote_vprintf(const char *fmt, va_list args)
{
    char buf[512];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len <= 0) return len;

    /* Always write to UART (stdout → UART0 via VFS) */
    fwrite(buf, 1, (size_t)len, stdout);

    int fd = s_client;
    if (fd >= 0) {
        /* Live client — send directly */
        if (send(fd, buf, len, MSG_DONTWAIT) < 0) {
            s_client = -1;
        }
    } else {
        /* No client yet — buffer for replay on next connection */
        ring_write(buf, (size_t)len);
    }
    return len;
}

/* ---------------------------------------------------------------------------
 * Server task: listens for TCP connections and updates s_client.
 * Priority kept low (3) — logging must never stall NTP or GPS.
 * --------------------------------------------------------------------------*/
static void log_server_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(CONFIG_LOG_SERVER_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    listen(listen_sock, 1);

    ESP_LOGI(TAG, "Log server ready — nc <ip> %d  (ring buffer: %d bytes)",
             CONFIG_LOG_SERVER_PORT, RING_SIZE);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(listen_sock,
                            (struct sockaddr *)&client_addr, &client_len);
        if (client < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Displace any existing client */
        int old = s_client;
        s_client = -1;
        if (old >= 0) close(old);

        ESP_LOGI(TAG, "Log client connected from %s",
                 inet_ntoa(client_addr.sin_addr));

        /* Replay buffered boot messages before going live */
        ring_drain(client);

        s_client = client;

        /* Block here until the client disconnects */
        char discard[16];
        while (recv(client, discard, sizeof(discard), 0) > 0) {}

        s_client = -1;
        close(client);
        ESP_LOGI(TAG, "Log client disconnected");
    }
}

#endif /* CONFIG_DEV_MODE_ENABLED */

void log_server_early_init(void)
{
#ifdef CONFIG_DEV_MODE_ENABLED
    /* Install hook immediately — all subsequent ESP_LOG* output is buffered
     * in the ring until a client connects, so no boot messages are lost. */
    esp_log_set_vprintf(remote_vprintf);
#endif
}

void log_server_init(void)
{
#ifdef CONFIG_DEV_MODE_ENABLED
    xTaskCreate(log_server_task, "log_server", 3072, NULL, 3, NULL);
    ESP_LOGI(TAG, "Remote log server on port %d  (ring: %d bytes)",
             CONFIG_LOG_SERVER_PORT, RING_SIZE);
#endif
}
