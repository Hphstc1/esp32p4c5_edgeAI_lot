/*
 * c5_at_sdio.c — AT Command Client over SDIO transport.
 *
 * Implements ESP-AT protocol commands over the SDIO transport layer
 * used for P4 <-> C5 communication.
 *
 * Protocol:
 *   Send: "AT\r\n"           → expect "\r\nOK\r\n"
 *   Send: "AT+CWJAP=...\r\n" → expect "\r\nOK\r\n"
 *   TCP data uses +IPD framing for receives
 */

#include "c5_bridge.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "c5_at_sdio";

/* SDIO block size — try to align reads/writes for efficiency */
#define SDIO_BLOCK_SIZE  512

/* ---- Internal helpers ------------------------------------------------ */

/**
 * @brief Read from SDIO, accumulating into buf, until `terminator` string
 *        is found or timeout expires.
 *
 * @param buf         Output buffer for data before the terminator.
 * @param max_len     Max bytes to write to buf (including NUL).
 * @param terminator  String to search for (e.g. "\r\nOK\r\n").
 * @param timeout_ms  Timeout in milliseconds.
 * @return Number of bytes before terminator, or -1 on timeout/error.
 */
static int sdio_read_until(char *buf, size_t max_len,
                           const char *terminator, uint32_t timeout_ms)
{
    int pos = 0;
    size_t term_len = strlen(terminator);
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    uint8_t chunk[SDIO_BLOCK_SIZE];

    while (esp_timer_get_time() < deadline) {
        int n = c5_sdio_read(1, 0, chunk, sizeof(chunk), 100);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        for (int i = 0; i < n; i++) {
            if (pos < (int)max_len - 1) {
                buf[pos++] = (char)chunk[i];
                buf[pos] = '\0';
            }
            /* Check for terminator at tail */
            if (pos >= (int)term_len) {
                if (strncmp(&buf[pos - term_len], terminator, term_len) == 0) {
                    buf[pos - term_len] = '\0';

                    /* Stash any unconsumed bytes AFTER the terminator
                     * in this chunk — they may contain +IPD TCP data
                     * that arrived during an AT response read. */
                    int remaining = n - (i + 1);
                    if (remaining > 0) {
                        c5_sdio_stash_data(chunk + i + 1, remaining);
                    }
                    return pos - term_len;
                }
            }
        }
    }

    return -1; /* timeout */
}

/**
 * @brief Drain and DISCARD any pending data from SDIO.
 *
 * Use this before sending AT commands to ensure a clean response channel.
 * Data is discarded (not stashed) because:
 *   1. Stashing +IPD data before CIPSEND causes the '>' wait loop to
 *      consume stash data first → not '>' → discard anyway.
 *   2. Stashed data can create an infinite read loop when the same bytes
 *      are returned by c5_sdio_read(), checked for a marker, re-stashed,
 *      and returned again without ever polling the hardware.
 *   3. Lost +IPD packets are harmless — the browser retries GET requests.
 */
static void sdio_drain(uint32_t timeout_ms)
{
    uint8_t chunk[SDIO_BLOCK_SIZE];
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        int n = c5_sdio_read(1, 0, chunk, sizeof(chunk), 50);
        if (n <= 0) {
            break;
        }
        /* discard — see comment above */
    }
}

/**
 * @brief Read from SDIO for AT response, auto-stashing any +IPD data
 *        that arrives before the expected terminator.
 *
 * Same as sdio_read_until() but uses a larger working buffer and
 * automatically detects/stashes +IPD unsolicited codes that arrive
 * during the wait. This prevents the AT response parser from being
 * confused by +IPD data from other TCP links.
 *
 * @param buf         Output buffer for data before the terminator.
 * @param max_len     Max bytes to write to buf (including NUL).
 * @param terminator  String to search for (e.g. "\r\nSEND OK\r\n").
 * @param timeout_ms  Timeout in milliseconds.
 * @return Number of bytes before terminator, or -1 on timeout/error.
 */
static int sdio_read_until_safe(char *buf, size_t max_len,
                                 const char *terminator, uint32_t timeout_ms)
{
    int pos = 0;
    size_t term_len = strlen(terminator);
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    uint8_t chunk[SDIO_BLOCK_SIZE];

    while (esp_timer_get_time() < deadline) {
        int n = c5_sdio_read(1, 0, chunk, sizeof(chunk), 100);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        for (int i = 0; i < n; i++) {
            if (pos < (int)max_len - 1) {
                buf[pos++] = (char)chunk[i];
                buf[pos] = '\0';
            }
            /* Check for terminator at tail */
            if (pos >= (int)term_len) {
                if (strncmp(&buf[pos - term_len], terminator, term_len) == 0) {
                    buf[pos - term_len] = '\0';

                    /* Stash unconsumed bytes AFTER the terminator */
                    int remaining = n - (i + 1);
                    if (remaining > 0) {
                        c5_sdio_stash_data(chunk + i + 1, remaining);
                    }
                    return pos - term_len;
                }
            }
        }

        /* If buffer is full and we haven't found the terminator yet,
         * the accumulated data is likely +IPD or other unsolicited
         * traffic.  Reset the buffer and keep looking — do NOT stash
         * (same infinite-loop risk as the '>' wait in tcp_send). */
        if (pos >= (int)max_len - 1) {
            pos = 0;
            buf[0] = '\0';
        }
    }

    return -1; /* timeout */
}

/* ---- Public API ------------------------------------------------------- */

esp_err_t c5_at_sdio_init(void)
{
    ESP_LOGI(TAG, "Testing SDIO AT transport...");

    /* C5 may still be initializing SDIO slave. Retry with increasing delays. */
    for (int i = 0; i < 5; i++) {
        int delay_ms = 500 + i * 500;  /* 500, 1000, 1500, 2000, 2500 ms */
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        esp_err_t err = c5_sdio_write(1, 0, "AT\r\n", 4);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "write failed on attempt %d", i + 1);
            continue;
        }

        char resp[64];
        int n = sdio_read_until_safe(resp, sizeof(resp), "\r\nOK\r\n", 2000);
        if (n >= 0) {
            ESP_LOGI(TAG, "AT test OK");

            /* Disable AT echo — critical for reliable prompt detection
             * in c5_at_sdio_tcp_send(). With echo on, C5 echoes back
             * every command (e.g. "AT+CIPSEND=0,512") which can fill
             * the small prompt_buf before the '>' prompt arrives. */
            sdio_drain(200);  /* drain any pending data first */
            err = c5_sdio_write(1, 0, "ATE0\r\n", 6);
            if (err == ESP_OK) {
                sdio_read_until_safe(resp, sizeof(resp), "\r\nOK\r\n", 1000);
                ESP_LOGI(TAG, "ATE0 sent (echo disabled)");
            }
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "AT test FAILED — C5 not responding over SDIO");
    return ESP_FAIL;
}

esp_err_t c5_at_sdio_ping(void)
{
    /* Single-shot "AT" with short timeout — no retries, no ATE0.
     * Designed for the watchdog loop that runs every ~5 seconds. */
    esp_err_t err = c5_sdio_write(1, 0, "AT\r\n", 4);
    if (err != ESP_OK) {
        return err;
    }

    char resp[32];
    int n = sdio_read_until_safe(resp, sizeof(resp), "\r\nOK\r\n", 1000);
    return (n >= 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t c5_at_sdio_send_cmd(const char *cmd, char *resp,
                              size_t resp_len, uint32_t timeout_ms)
{
    if (!cmd) {
        return ESP_ERR_INVALID_ARG;
    }
    if (timeout_ms == 0) {
        timeout_ms = 3000;
    }

    ESP_LOGD(TAG, "AT TX: %s", cmd);

    /* Build command with \r\n in a single buffer so C5 receives a
     * complete AT command in one SDIO chunk. Two separate writes
     * could arrive as separate sdio_slave_recv() calls, and the AT
     * parser might not handle a bare command without terminator. */
    char tx_buf[256];
    int cmd_len = snprintf(tx_buf, sizeof(tx_buf), "%s\r\n", cmd);

    /* Send command + \r\n as a single SDIO write */
    esp_err_t err = c5_sdio_write(1, 0, tx_buf, cmd_len);
    if (err != ESP_OK) {
        return err;
    }

    /* Fire-and-forget: drain response and check for OK */
    if (!resp || resp_len == 0) {
        char tmp[128];
        int n = sdio_read_until_safe(tmp, sizeof(tmp), "\r\nOK\r\n", timeout_ms);
        if (n < 0) {
            sdio_drain(100);
            return ESP_ERR_TIMEOUT;
        }
        return ESP_OK;
    }

    /* Read response terminated by OK */
    int n = sdio_read_until_safe(resp, resp_len, "\r\nOK\r\n", timeout_ms);
    if (n >= 0) {
        /* Strip trailing CR/LF from response */
        while (n > 0 && (resp[n - 1] == '\r' || resp[n - 1] == '\n')) {
            resp[--n] = '\0';
        }
        ESP_LOGD(TAG, "AT RX (%d bytes): %s", n, resp);
        return ESP_OK;
    }

    /* Timeout — check if the last response was an ERROR */
    ESP_LOGW(TAG, "AT timeout: %s", cmd);
    return ESP_ERR_TIMEOUT;
}

esp_err_t c5_at_sdio_wifi_connect(const char *ssid, const char *password,
                                  uint32_t timeout_ms)
{
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }
    if (timeout_ms == 0) {
        timeout_ms = 15000;
    }

    /* Set STA mode */
    esp_err_t err = c5_at_sdio_send_cmd("AT+CWMODE=1", NULL, 0, 2000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set STA mode");
        return err;
    }

    /* Connect to AP */
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    err = c5_at_sdio_send_cmd(cmd, NULL, 0, timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed (check SSID/password)");
        return err;
    }

    /* Wait for valid IP */
    char ip[32];
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(500));
        err = c5_at_sdio_get_ip(ip, sizeof(ip));
        if (err == ESP_OK && strlen(ip) > 0 && strcmp(ip, "0.0.0.0") != 0) {
            ESP_LOGI(TAG, "WiFi connected, IP: %s", ip);
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "WiFi connected but no IP (timeout)");
    return ESP_ERR_TIMEOUT;
}

esp_err_t c5_at_sdio_wifi_ap(const char *ssid, const char *password)
{
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = c5_at_sdio_send_cmd("AT+CWMODE=2", NULL, 0, 2000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set AP mode");
        return err;
    }

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CWSAP=\"%s\",\"%s\",5,3", ssid, password);
    return c5_at_sdio_send_cmd(cmd, NULL, 0, 3000);
}

esp_err_t c5_at_sdio_get_ip(char *buf, size_t len)
{
    if (!buf || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    char resp[256];
    esp_err_t err = c5_at_sdio_send_cmd("AT+CIFSR", resp, sizeof(resp), 3000);
    if (err != ESP_OK) {
        buf[0] = '\0';
        return err;
    }

    /* Parse STA IP: +CIFSR:STAIP,"192.168.1.100" */
    char *p = strstr(resp, "+CIFSR:STAIP,\"");
    if (p) {
        p += 14; /* skip '+CIFSR:STAIP,"' (14 chars) */
        char *q = strchr(p, '"');
        if (q && (size_t)(q - p) < len) {
            memcpy(buf, p, q - p);
            buf[q - p] = '\0';
            return ESP_OK;
        }
    }

    /* Fallback to AP IP: +CIFSR:APIP,"192.168.4.1" */
    p = strstr(resp, "+CIFSR:APIP,\"");
    if (p) {
        p += 13; /* skip '+CIFSR:APIP,"' (13 chars) */
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

esp_err_t c5_at_sdio_tcp_server_start(int port)
{
    /* Enable multiple connections (required for CIPSERVER) */
    esp_err_t err = c5_at_sdio_send_cmd("AT+CIPMUX=1", NULL, 0, 2000);
    if (err != ESP_OK) {
        return err;
    }

    /* Allow up to 5 simultaneous clients (default is 1, which blocks
     * the /stream long-lived connection when REST API polls arrive). */
    err = c5_at_sdio_send_cmd("AT+CIPSERVERMAXCONN=5", NULL, 0, 2000);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CIPSERVERMAXCONN failed (err=0x%x) — continuing anyway", err);
    }

    /* Start TCP server */
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%d", port);
    return c5_at_sdio_send_cmd(cmd, NULL, 0, 2000);
}

esp_err_t c5_at_sdio_tcp_close(int link_id)
{
    /* Explicitly close a TCP link so it can be reused immediately.
     * Without this, C5 ESP-AT's default server (CIPSERVERMAXCONN=1)
     * rejects new connections while a previous link is still in
     * the TCP TIME_WAIT / CLOSE_WAIT state. */
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d", link_id);
    return c5_at_sdio_send_cmd(cmd, NULL, 0, 2000);
}

esp_err_t c5_at_sdio_tcp_send(int link_id, const uint8_t *data,
                              size_t len, uint32_t timeout_ms)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (timeout_ms == 0) {
        timeout_ms = 3000;
    }

    /* For large payloads, split into SDIO-block-aligned chunks.
     * C5 block size is 512 bytes. */
    size_t max_chunk = SDIO_BLOCK_SIZE;
    size_t offset = 0;

    while (offset < len) {
        size_t chunk_len = (len - offset > max_chunk) ? max_chunk : (len - offset);

        /*
         * PRE-DRAIN: stash any pending +IPD/unsolicited data before
         * sending the CIPSEND command. This reduces the chance of
         * +IPD data mixing with the AT response ('>' or 'SEND OK').
         * The stashed data is returned by the next c5_sdio_read().
         */
        sdio_drain(20);

        /* Send AT+CIPSEND=<link_id>,<len>\r\n as single write */
        char cmd[64];
        int cmd_len = snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%u\r\n", link_id, (unsigned)chunk_len);

        ESP_LOGD(TAG, "TCP send chunk %u/%u: %.*s",
                 (unsigned)(offset + chunk_len), (unsigned)len, cmd_len - 2, cmd);

        esp_err_t err = c5_sdio_write(1, 0, cmd, cmd_len);
        if (err != ESP_OK) return err;

        /*
         * Wait for '>' prompt. If we get data without '>', discard it —
         * it's likely +IPD from another link.  Do NOT re-stash: the stash
         * is returned by c5_sdio_read() before hardware is polled, so
         * re-stashing creates an infinite loop (read stash → not '>' →
         * stash → read same bytes → ...). */
        char prompt_buf[SDIO_BLOCK_SIZE];
        int64_t prompt_deadline = esp_timer_get_time() + 5000 * 1000;
        int got_prompt = 0;

        while (esp_timer_get_time() < prompt_deadline) {
            int n = c5_sdio_read(1, 0, prompt_buf, sizeof(prompt_buf) - 1, 500);
            if (n > 0) {
                prompt_buf[n] = '\0';
                char *gt = strchr(prompt_buf, '>');
                if (gt) {
                    got_prompt = 1;
                    /* Stash any extra data before/after '>' (may be +IPD) */
                    if (gt > prompt_buf) {
                        c5_sdio_stash_data((uint8_t *)prompt_buf, gt - prompt_buf);
                    }
                    int after_gt = n - (int)(gt + 1 - prompt_buf);
                    if (after_gt > 0) {
                        c5_sdio_stash_data((uint8_t *)(gt + 1), after_gt);
                    }
                    break;
                }
                /* No '>' found — discard the data (likely +IPD).
                 * Do NOT re-stash!  c5_sdio_read() returns stash data
                 * first, so re-stashing creates an infinite loop where
                 * the same +IPD bytes are read → not '>' → stashed →
                 * read again, and the hardware is never polled for the
                 * real '>' prompt.  Dropping a few +IPD packets is safe:
                 * the browser retries idempotent GET requests. */
                /* (data discarded — do not stash) */
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!got_prompt) {
            ESP_LOGE(TAG, "No '>' prompt for CIPSEND on link %d", link_id);
            return ESP_ERR_TIMEOUT;
        }

        /* Send raw data chunk */
        err = c5_sdio_write(1, 0, data + offset, chunk_len);
        if (err != ESP_OK) return err;

        /*
         * Wait for SEND OK using the safe reader: large buffer that
         * auto-stashes +IPD data arriving before the AT response.
         */
        char send_resp[SDIO_BLOCK_SIZE];
        int n = sdio_read_until_safe(send_resp, sizeof(send_resp),
                                     "\r\nSEND OK\r\n", timeout_ms);
        if (n < 0) {
            ESP_LOGE(TAG, "CIPSEND chunk at offset %u failed", (unsigned)offset);
            return ESP_FAIL;
        }

        offset += chunk_len;
    }

    ESP_LOGI(TAG, "TCP send complete: %u bytes over link %d",
             (unsigned)len, link_id);
    return ESP_OK;
}

int c5_at_sdio_tcp_recv(int link_id, uint8_t *buf,
                        size_t max_len, uint32_t timeout_ms)
{
    if (!buf || max_len == 0) {
        return -1;
    }
    if (timeout_ms == 0) {
        timeout_ms = 3000;
    }

    /*
     * Accumulate data from SDIO and look for the +IPD header:
     *   +IPD,<link_id>,<len>:<payload>
     *
     * Reads are done in SDIO block-sized chunks into a scratch buffer
     * so we can scan for the header across chunk boundaries.
     */
    uint8_t scratch[SDIO_BLOCK_SIZE * 2];
    size_t accum = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        uint8_t chunk[SDIO_BLOCK_SIZE];
        int n = c5_sdio_read(1, 0, chunk, sizeof(chunk), 200);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Accumulate into scratch buffer (don't overflow) */
        size_t room = sizeof(scratch) - accum;
        size_t to_copy = ((size_t)n < room) ? (size_t)n : room;
        memcpy(scratch + accum, chunk, to_copy);
        accum += to_copy;
        scratch[accum] = '\0';

        /* Build header pattern for this link_id */
        char pattern[32];
        snprintf(pattern, sizeof(pattern), "+IPD,%d,", link_id);

        char *p = strstr((char *)scratch, pattern);
        if (!p) {
            /* No header found yet — need more data */
            continue;
        }

        /* Parse data length after the pattern */
        p += strlen(pattern);
        int data_len = atoi(p);
        if (data_len <= 0) {
            continue;
        }

        /* Find the ':' that separates length from payload */
        char *colon = strchr(p, ':');
        if (!colon) {
            /* Colon not yet received — need more data */
            continue;
        }

        size_t header_end = (colon + 1) - (char *)scratch;
        size_t payload_avail = accum - header_end;

        if (payload_avail < (size_t)data_len) {
            /* Not all payload received yet — keep reading */
            continue;
        }

        /* Complete payload is available — copy to caller.
         * Stash any trailing data after the payload in case it
         * contains the next +IPD notification or other events. */
        size_t copy_len = ((size_t)data_len < max_len) ? (size_t)data_len : max_len;
        memcpy(buf, colon + 1, copy_len);

        size_t consumed = header_end + data_len;
        if (accum > consumed) {
            c5_sdio_stash_data(scratch + consumed, accum - consumed);
        }
        return (int)copy_len;
    }

    return -1; /* timeout */
}

int c5_at_sdio_tcp_recv_any(int *out_link_id, uint8_t *buf,
                             size_t max_len, uint32_t timeout_ms)
{
    if (!out_link_id || !buf || max_len == 0) {
        return -1;
    }
    if (timeout_ms == 0) {
        timeout_ms = 3000;
    }

    /*
     * Same as c5_at_sdio_tcp_recv() but matches +IPD from ANY link_id.
     * Parses the link_id from the header and returns it via out_link_id.
     */
    uint8_t scratch[SDIO_BLOCK_SIZE * 2];
    size_t accum = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        uint8_t chunk[SDIO_BLOCK_SIZE];
        int n = c5_sdio_read(1, 0, chunk, sizeof(chunk), 200);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Accumulate into scratch buffer (don't overflow) */
        size_t room = sizeof(scratch) - accum;
        size_t to_copy = ((size_t)n < room) ? (size_t)n : room;
        memcpy(scratch + accum, chunk, to_copy);
        accum += to_copy;
        scratch[accum] = '\0';

        /* Search for ANY +IPD header */
        char *p = strstr((char *)scratch, "+IPD,");
        if (!p) {
            continue;
        }

        /* Parse link_id: +IPD,<link_id>,<len>: */
        p += 5; /* skip "+IPD," */
        int link_id = atoi(p);
        if (link_id < 0 || link_id > 9) {
            /* Bogus link_id — skip this occurrence and keep looking */
            size_t skip = (p - (char *)scratch) + 1;
            if (skip < accum) {
                memmove(scratch, scratch + skip, accum - skip);
                accum -= skip;
            } else {
                accum = 0;
            }
            continue;
        }

        /* Find the comma after link_id, then parse data length */
        char *comma = strchr(p, ',');
        if (!comma) {
            continue;
        }
        int data_len = atoi(comma + 1);
        if (data_len <= 0) {
            continue;
        }

        /* Find the ':' that separates length from payload */
        char *colon = strchr(comma + 1, ':');
        if (!colon) {
            /* Colon not yet received — need more data */
            continue;
        }

        size_t header_end = (colon + 1) - (char *)scratch;
        size_t payload_avail = accum - header_end;

        if (payload_avail < (size_t)data_len) {
            /* Not all payload received yet — keep reading */
            continue;
        }

        /* Complete payload is available */
        *out_link_id = link_id;
        size_t copy_len = ((size_t)data_len < max_len) ? (size_t)data_len : max_len;
        memcpy(buf, colon + 1, copy_len);

        size_t consumed = header_end + data_len;
        if (accum > consumed) {
            c5_sdio_stash_data(scratch + consumed, accum - consumed);
        }
        return (int)copy_len;
    }

    return -1; /* timeout */
}
