/*
 * c5_bridge.c — UART AT Client for ESP32-C5 ESP-AT firmware.
 *
 * Protocol:
 *   Send: "AT\r\n"           → expect "\r\nOK\r\n"
 *   Send: "AT+CWJAP=...\r\n" → expect "\r\nWIFI CONNECTED\r\n" then "\r\nOK\r\n"
 */

#include "c5_bridge.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

static const char *TAG = "c5_bridge";

/* ---- Internal state ------------------------------------------------- */

static int            s_uart_num   = -1;
static EventGroupHandle_t s_events = NULL;
static TaskHandle_t       s_rx_task = NULL;
static bool               s_running  = false;

/* Response buffer (double-buffered for async reads) */
#define RX_RING_SIZE  4096
static uint8_t  s_rx_ring[RX_RING_SIZE];
static uint32_t s_rx_head = 0;   // write cursor (ISR / rx task)
static uint32_t s_rx_tail = 0;   // read cursor (app)

/* UART RX FIFO watermark */
#define UART_RX_FIFO_SIZE  256

/* ---- Internals: ring buffer ---------------------------------------- */

static inline void ring_push(uint8_t b)
{
    s_rx_ring[s_rx_head % RX_RING_SIZE] = b;
    s_rx_head++;
}

static inline int ring_pop(void)
{
    if (s_rx_head == s_rx_tail) return -1;
    uint8_t b = s_rx_ring[s_rx_tail % RX_RING_SIZE];
    s_rx_tail++;
    return b;
}

static inline void ring_clear(void)
{
    s_rx_head = s_rx_tail = 0;
}

/* ---- Internals: UART RX task --------------------------------------- */

static void uart_rx_task(void *param)
{
    uint8_t buf[128];
    while (s_running) {
        int len = uart_read_bytes(s_uart_num, buf, sizeof(buf),
                                   pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                ring_push(buf[i]);
            }
            /* Check for unsolicited WiFi events */
            /* We scan the ring for known URC patterns */
        }
    }
    vTaskDelete(NULL);
}

/* ---- Internals: UART read string ----------------------------------- */

/* Read from ring until 'end' string found or timeout.
 * Returns number of bytes read (excluding end marker), or -1 on timeout. */
static int uart_read_until(char *buf, int max_len, const char *end,
                            uint32_t timeout_ms)
{
    int pos = 0;
    int end_len = strlen(end);
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        int b = ring_pop();
        if (b < 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (pos < max_len - 1) {
            buf[pos++] = (char)b;
            buf[pos] = '\0';
        }
        /* Check for end marker at tail */
        if (pos >= end_len) {
            if (strncmp(&buf[pos - end_len], end, end_len) == 0) {
                buf[pos - end_len] = '\0'; // strip end marker
                return pos - end_len;
            }
        }
    }
    return -1; // timeout
}

/* Read all available data into buffer (no timeout) */
static int uart_drain(char *buf, int max_len, uint32_t timeout_ms)
{
    int pos = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        int b = ring_pop();
        if (b < 0) {
            if (pos > 0) break; // got something, return it
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (pos < max_len - 1) {
            buf[pos++] = (char)b;
        }
    }
    if (pos > 0) buf[pos] = '\0';
    return pos;
}

/* ---- Internals: AT command send ------------------------------------ */

static esp_err_t at_send_internal(const char *cmd, char *resp, size_t resp_len,
                                   uint32_t timeout_ms, bool expect_ok)
{
    if (!s_running) return ESP_ERR_INVALID_STATE;

    ring_clear();

    /* Send command */
    uart_write_bytes(s_uart_num, cmd, strlen(cmd));
    uart_write_bytes(s_uart_num, "\r\n", 2);

    ESP_LOGD(TAG, "AT TX: %s", cmd);

    if (!resp || resp_len == 0) {
        /* Fire-and-forget mode — just drain and look for OK */
        if (expect_ok) {
            char tmp[128];
            int n = uart_read_until(tmp, sizeof(tmp), "\r\nOK\r\n", timeout_ms);
            if (n < 0) {
                /* Also accept \r\nERROR\r\n */
                ring_clear();
                /* Drain whatever we have */
                vTaskDelay(pdMS_TO_TICKS(timeout_ms));
                return ESP_ERR_TIMEOUT;
            }
            ESP_LOGD(TAG, "AT RX: %s", tmp);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        return ESP_OK;
    }

    /* Read response up to "OK\r\n" or "ERROR\r\n" */
    int n = uart_read_until(resp, (int)resp_len, "\r\nOK\r\n", timeout_ms);
    if (n < 0) {
        /* Check for ERROR */
        ring_clear();
        uart_write_bytes(s_uart_num, cmd, strlen(cmd));
        uart_write_bytes(s_uart_num, "\r\n", 2);
        n = uart_read_until(resp, (int)resp_len, "\r\nERROR\r\n", timeout_ms / 2);
        if (n >= 0) {
            ESP_LOGW(TAG, "AT ERROR response: %s", resp);
            return ESP_FAIL;
        }
        ESP_LOGW(TAG, "AT timeout: %s", cmd);
        return ESP_ERR_TIMEOUT;
    }

    /* Trim trailing \r\n from response */
    while (n > 0 && (resp[n-1] == '\r' || resp[n-1] == '\n')) {
        resp[--n] = '\0';
    }

    ESP_LOGD(TAG, "AT RX (%d bytes): %s", n, resp);
    return ESP_OK;
}

/* ---- Public API ---------------------------------------------------- */

esp_err_t c5_bridge_init(const c5_bridge_config_t *cfg)
{
    if (s_running) return ESP_ERR_INVALID_STATE;

    c5_bridge_config_t def = C5_BRIDGE_CONFIG_DEFAULT();
    if (cfg) def = *cfg;

    /* Install UART driver */
    uart_config_t uart_cfg = {
        .baud_rate = def.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(def.uart_num, def.rx_buf_size * 2,
                                         def.rx_buf_size * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(def.uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(def.uart_num, def.tx_gpio, def.rx_gpio,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_uart_num = def.uart_num;

    /* Create event group */
    s_events = xEventGroupCreate();
    if (!s_events) return ESP_ERR_NO_MEM;

    /* Start RX task */
    s_running = true;
    BaseType_t ret = xTaskCreate(uart_rx_task, "c5_uart_rx", 3072,
                                  NULL, 5, &s_rx_task);
    if (ret != pdPASS) {
        s_running = false;
        uart_driver_delete(def.uart_num);
        vEventGroupDelete(s_events);
        s_events = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "C5 bridge init ok: UART%d TX=%d RX=%d @ %d baud",
             def.uart_num, def.tx_gpio, def.rx_gpio, def.baud_rate);
    return ESP_OK;
}

esp_err_t c5_bridge_test(void)
{
    /* Send AT a few times (C5 may echo during init) */
    for (int i = 0; i < 3; i++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_err_t err = at_send_internal("AT", NULL, 0, 1000, true);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "C5 AT test OK");
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "C5 AT test FAILED");
    return ESP_FAIL;
}

esp_err_t c5_bridge_wifi_connect(const char *ssid, const char *password,
                                  uint32_t timeout_ms)
{
    if (!ssid || !password) return ESP_ERR_INVALID_ARG;
    if (timeout_ms == 0) timeout_ms = 15000;

    /* Set STA mode */
    esp_err_t err = at_send_internal("AT+CWMODE=1", NULL, 0, 2000, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA mode");
        return err;
    }

    /* Connect to AP */
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    err = at_send_internal(cmd, NULL, 0, timeout_ms, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed (check SSID/password)");
        return err;
    }

    /* Wait for IP */
    char ip[32];
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        esp_err_t ip_err = c5_bridge_get_ip(ip, sizeof(ip));
        if (ip_err == ESP_OK && strlen(ip) > 0 && strcmp(ip, "0.0.0.0") != 0) {
            ESP_LOGI(TAG, "WiFi connected, IP: %s", ip);
            if (s_events) {
                xEventGroupSetBits(s_events, C5_WIFI_CONNECTED_BIT | C5_WIFI_GOT_IP_BIT);
            }
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGE(TAG, "WiFi connected but no IP (timeout)");
    return ESP_ERR_TIMEOUT;
}

esp_err_t c5_bridge_get_ip(char *buf, size_t len)
{
    if (!buf || len == 0) return ESP_ERR_INVALID_ARG;

    char resp[128];
    esp_err_t err = at_send_internal("AT+CIFSR", resp, sizeof(resp), 3000, true);
    if (err != ESP_OK) {
        buf[0] = '\0';
        return err;
    }

    /* Parse: +CIFSR:STAIP,"192.168.1.100" */
    char *p = strstr(resp, "+CIFSR:STAIP,\"");
    if (p) {
        p += 14; // skip '+CIFSR:STAIP,"'
        char *q = strchr(p, '"');
        if (q && (size_t)(q - p) < len) {
            memcpy(buf, p, q - p);
            buf[q - p] = '\0';
            return ESP_OK;
        }
    }
    buf[0] = '\0';
    return ESP_FAIL;
}

esp_err_t c5_bridge_at_cmd(const char *cmd, char *resp, size_t resp_len,
                            uint32_t timeout_ms)
{
    return at_send_internal(cmd, resp, resp_len,
                             timeout_ms ? timeout_ms : 3000, true);
}

EventGroupHandle_t c5_bridge_get_event_group(void)
{
    return s_events;
}

void c5_bridge_deinit(void)
{
    s_running = false;
    if (s_rx_task) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }
    if (s_uart_num >= 0) {
        uart_driver_delete(s_uart_num);
        s_uart_num = -1;
    }
    if (s_events) {
        vEventGroupDelete(s_events);
        s_events = NULL;
    }
}
