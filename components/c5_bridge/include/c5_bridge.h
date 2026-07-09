/*
 * c5_bridge.h — P4 ↔ C5 communication (SDIO primary, UART fallback)
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/sdmmc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- SDIO ---------------------------------------------------------- */

/**
 * @brief Probe for C5 ESP-AT SDIO slave.
 *
 * Resets C5 via GPIO13, initializes SDMMC Slot 1, and tries to detect
 * the C5 as an SDIO card.
 *
 * @return ESP_OK if C5 SDIO slave is detected.
 */
esp_err_t c5_sdio_probe(void);

/**
 * @brief Re-initialize the C5 SDIO link from scratch (watchdog recovery).
 *
 * Resets C5 via GPIO13, re-initializes SDMMC host, re-probes the card,
 * and re-initializes the transport layer.  Safe to call after the link
 * has been detected as hung (AT ping timeout, repeated CMD53 failures).
 *
 * @return ESP_OK on successful reinit.
 */
esp_err_t c5_sdio_reinit(void);

/**
 * @brief Get the SDMMC card handle after successful probe.
 */
sdmmc_card_t *c5_sdio_get_card(void);

/**
 * @brief Set SDIO clock frequency after successful AT verification.
 *
 * Call this after c5_at_sdio_init() returns ESP_OK to raise the SDIO
 * clock from probing speed (400kHz) to a higher frequency for better
 * throughput during MJPEG streaming.
 *
 * @param freq_khz  Target frequency in kHz (e.g. SDMMC_FREQ_DEFAULT = 20000).
 * @return ESP_OK on success, ESP_FAIL if card not probed or clock set fails.
 */
esp_err_t c5_sdio_set_clock(uint32_t freq_khz);

/* ---- SDIO Transport ------------------------------------------------ */

/**
 * @brief Initialize the SDIO transport layer.
 * @param card  Previously probed SDMMC card handle.
 * @return ESP_OK on success.
 */
esp_err_t c5_sdio_transport_init(sdmmc_card_t *card);

/**
 * @brief Write bytes to an SDIO function address.
 * @param function  SDIO function number (1-7).
 * @param addr      Register/FIFO address (OR with SDMMC_IO_FIXED_ADDR for FIFO).
 * @param data      Data buffer.
 * @param len       Number of bytes to write.
 * @return ESP_OK on success.
 */
esp_err_t c5_sdio_write(int function, uint32_t addr, const void *data, size_t len);

/**
 * @brief Read up to max_len bytes from an SDIO function address.
 * @param function   SDIO function number (1-7).
 * @param addr       Register/FIFO address.
 * @param buf        Output buffer.
 * @param max_len    Maximum bytes to read.
 * @param timeout_ms Wait for interrupt before retry (0 = no wait).
 * @return Number of bytes read on success, -1 on timeout/error.
 */
int c5_sdio_read(int function, uint32_t addr, void *buf, size_t max_len, uint32_t timeout_ms);

/**
 * @brief Wait for an SDIO interrupt from the slave.
 *
 * Matches official sdio_host_wait_int().
 *
 * @param timeout_ms  Timeout in milliseconds.
 * @return ESP_OK if interrupt received, ESP_ERR_TIMEOUT on timeout.
 */
esp_err_t c5_sdio_wait_int(uint32_t timeout_ms);

/**
 * @brief Read SDIO slave interrupt status registers.
 *
 * Matches official sdio_host_get_intr():
 *   - REG_SDIO_INT_RAW (0x050): raw interrupt bits
 *   - REG_SDIO_INT_ST  (0x058): masked interrupt bits
 *
 * @param intr_raw  [out] Raw interrupt bits (may be NULL).
 * @param intr_st   [out] Masked interrupt bits (may be NULL).
 * @return ESP_OK on success.
 */
esp_err_t c5_sdio_get_intr(uint32_t *intr_raw, uint32_t *intr_st);

/**
 * @brief Clear SDIO slave interrupt bits.
 *
 * Matches official sdio_host_clear_intr():
 *   Writes mask to REG_SDIO_INT_CLR (0x0D4).
 *
 * @param intr_mask  Bitmask of interrupts to clear.
 * @return ESP_OK on success.
 */
esp_err_t c5_sdio_clear_intr(uint32_t intr_mask);

/**
 * @brief Get the current SDIO TX congestion percentage (0 = idle, 100 = full).
 *
 * Reads the TOKEN_RDATA register and computes how full the C5 slave's
 * TX buffer is.  Used to drive adaptive JPEG quality.
 *
 * @return Congestion percentage (0-100), or 0 if no card probed.
 */
int c5_sdio_get_tx_congestion_pct(void);

/**
 * @brief Dump SDIO RX state registers for diagnostics.
 *
 * Prints REG_SDIO_PKT_LEN, REG_SDIO_TOKEN_RDATA, interrupt status,
 * and computed available bytes.  Call periodically from the polling
 * loop to see if the C5 slave is actually sending data.
 */
void c5_sdio_dump_rx_state(void);

/**
 * @brief Stash data for later retrieval by c5_sdio_read().
 *
 * Use this when a reader consumed more SDIO data than it needed
 * (e.g. +IPD notifications arriving during an AT response read).
 * The next c5_sdio_read() will return stashed data first before
 * reading from hardware.
 *
 * @param data  Data to stash.
 * @param len   Number of bytes.
 */
void c5_sdio_stash_data(const uint8_t *data, size_t len);

/* ---- AT Commands over SDIO ---------------------------------------- */

/**
 * @brief Test SDIO AT transport by sending "AT" and checking for OK.
 * @return ESP_OK if C5 responds with OK.
 */
esp_err_t c5_at_sdio_init(void);

/**
 * @brief Quick AT ping (short timeout, single attempt).
 *
 * Unlike c5_at_sdio_init() which retries 5x with escalating delays,
 * this sends one "AT" and waits briefly.  Designed for the watchdog
 * loop that runs every few seconds.
 *
 * @return ESP_OK if C5 responds with OK.
 */
esp_err_t c5_at_sdio_ping(void);

/**
 * @brief Send an AT command and wait for response.
 * @param cmd         AT command string (without \r\n).
 * @param resp        Output buffer for response text (before terminator).
 *                    May be NULL for fire-and-forget.
 * @param resp_len    Size of resp buffer.
 * @param timeout_ms  Timeout in milliseconds (default 3000 if 0).
 * @return ESP_OK on OK response, ESP_ERR_TIMEOUT on timeout/error.
 */
esp_err_t c5_at_sdio_send_cmd(const char *cmd, char *resp, size_t resp_len, uint32_t timeout_ms);

/**
 * @brief Connect to a WiFi access point in STA mode.
 * @param ssid       Access point SSID.
 * @param password   Access point password.
 * @param timeout_ms Total timeout (default 15000 if 0).
 * @return ESP_OK when connected with valid IP.
 */
esp_err_t c5_at_sdio_wifi_connect(const char *ssid, const char *password, uint32_t timeout_ms);

/**
 * @brief Start a WiFi SoftAP.
 * @param ssid      AP SSID.
 * @param password  AP password (min 8 chars for WPA2).
 * @return ESP_OK on success.
 */
esp_err_t c5_at_sdio_wifi_ap(const char *ssid, const char *password);

/**
 * @brief Get the current IP address from AT+CIFSR.
 * @param buf  Output buffer for IP string.
 * @param len  Size of buf.
 * @return ESP_OK if IP parsed, ESP_FAIL otherwise.
 */
esp_err_t c5_at_sdio_get_ip(char *buf, size_t len);

/**
 * @brief Start a TCP server on the C5.
 * @param port  TCP port number.
 * @return ESP_OK on success.
 */
esp_err_t c5_at_sdio_tcp_server_start(int port);

/**
 * @brief Send data over an established TCP connection.
 *
 * For payloads larger than 512 bytes, data is split into
 * SDIO-block-aligned chunks.
 *
 * @param link_id    TCP link ID (0-n).
 * @param data       Data to send.
 * @param len        Data length.
 * @param timeout_ms Per-chunk timeout (default 3000 if 0).
 * @return ESP_OK on complete send.
 */
esp_err_t c5_at_sdio_tcp_send(int link_id, const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Receive TCP data on a link.
 *
 * Reads from SDIO looking for +IPD,<link_id>,<len>: header,
 * then copies the payload into buf.
 *
 * @param link_id    TCP link ID to receive from.
 * @param buf        Output buffer for payload.
 * @param max_len    Maximum bytes to receive.
 * @param timeout_ms Timeout in milliseconds (default 3000 if 0).
 * @return Number of bytes received, or -1 on timeout/error.
 */
int c5_at_sdio_tcp_recv(int link_id, uint8_t *buf, size_t max_len, uint32_t timeout_ms);

/**
 * @brief Receive TCP data from ANY link (wildcard).
 *
 * Same as c5_at_sdio_tcp_recv() but matches +IPD from any link_id.
 * The actual link_id is written to *out_link_id.
 *
 * @param out_link_id  [out] The link_id the data came from.
 * @param buf          Output buffer for payload.
 * @param max_len      Maximum bytes to receive.
 * @param timeout_ms   Timeout in milliseconds (default 3000 if 0).
 * @return Number of bytes received, or -1 on timeout/error.
 */
int c5_at_sdio_tcp_recv_any(int *out_link_id, uint8_t *buf, size_t max_len, uint32_t timeout_ms);

/**
 * @brief Explicitly close a TCP link (AT+CIPCLOSE).
 *
 * Frees the link immediately so C5 can accept new connections.
 * Essential when CIPSERVERMAXCONN is limited and REST API
 * responses use Connection: close.
 *
 * @param link_id  TCP link ID (0-4).
 * @return ESP_OK on success.
 */
esp_err_t c5_at_sdio_tcp_close(int link_id);

#ifdef __cplusplus
}
#endif
