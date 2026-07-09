/*
 * c5_sdio_transport.c — SDIO transport for P4 ↔ C5 ESP-AT communication.
 *
 * Rewritten based on the official ESP-AT at_sdio_host reference:
 *   D:\esp32\esp-at-c5\examples\at_sdio_host\ESP32\components\sdio_host\
 *
 * KEY PROTOCOL (from official reference):
 *   Before sending → read ESP_SDIO_TOKEN_RDATA (Fn1:0x044) to check TX buffer count
 *   After sending  → notify slave via ESP_SDIO_CONF (Fn1:0x08C) interrupt
 *   Before reading → read ESP_SDIO_PKT_LEN  (Fn1:0x060) to check available data size
 *   Address = ESP_SLAVE_CMD53_END_ADDR (0x1F800) - data_length
 *
 * INIT SEQUENCE (from official esp_slave_init_io):
 *   1. Enable BOTH Function 1 and Function 2 (IOE=6)
 *   2. Write to IOR to signal expected functions
 *   3. Set block sizes for Function 0, 1, AND 2
 *   4. Enable master + function interrupts (IE=7)
 */

#include "c5_bridge.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"
#include "sd_protocol_defs.h"
#include "driver/sdmmc_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "c5_sdio_transport";

static sdmmc_card_t *s_card = NULL;

/* ---- Register definitions (from official sdio_host_reg.h) ------------ */
/* These are the ESP SDIO Slave hardware register offsets, accessed via
 * Function 1 CMD52/CMD53. The base address & 0x3FF yields the Fn1 offset.
 * Same offsets across ESP32 / ESP32-C2 / C3 / C5 / C6 / C5 slave IP. */

#define ESP_SLAVE_CMD53_END_ADDR    0x1F800  /* FIFO end address */

/* SLCHOST register offsets (from official sdio_host_reg.h):
 *   ESP32_SLCHOST_BASE = 0x3ff55000, masked with &0x3FF to get Fn1 addr */
#define REG_SDIO_PKT_LEN          0x060    /* RX: packet length available */
#define REG_SDIO_TOKEN_RDATA      0x044    /* TX: buffer credit/token */
#define REG_SDIO_INT_RAW          0x050    /* Raw interrupt status */
#define REG_SDIO_INT_ST           0x058    /* Masked interrupt status */
#define REG_SDIO_INT_CLR          0x0D4    /* Interrupt clear */
#define REG_SDIO_CONF             0x08C    /* Host→Slave interrupt trigger */
#define REG_SDIO_CONF_OFFSET      0
#define REG_SDIO_SEND_OFFSET      16

/* Flow control constants */
#define TX_BUFFER_MAX   0x1000
#define TX_BUFFER_MASK  0xFFF
#define RX_BYTE_MAX     0x100000
#define RX_BYTE_MASK    0xFFFFF

/* Block size used by the C5 slave (CONFIG_AT_SDIO_BLOCK_SIZE = 512) */
#define C5_SDIO_BLOCK_SIZE  512

/* CCCR register offsets */
#define SD_IO_CCCR_FN_ENABLE        0x02
#define SD_IO_CCCR_FN_READY         0x03
#define SD_IO_CCCR_INT_ENABLE       0x04
#define SD_IO_CCCR_BLKSIZEL         0x10
#define SD_IO_CCCR_BLKSIZEH         0x11

/* SD_IO_FBR_START already defined in sd_protocol_defs.h as 0x00100 */

/* ---- Flow-control counters (mirrors official approach) --------------- */
static uint32_t tx_sent_buffers = 0;
static uint32_t rx_got_bytes   = 0;

/* ---- Shared receive stash — prevents data loss when AT response
 *     readers accidentally consume TCP +IPD notifications. ------------ */
#define SDIO_STASH_SIZE  (C5_SDIO_BLOCK_SIZE * 16)
static uint8_t s_stash_buf[SDIO_STASH_SIZE];
static size_t  s_stash_len = 0;

void c5_sdio_stash_data(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    if (len > SDIO_STASH_SIZE - s_stash_len) {
        ESP_LOGW(TAG, "stash overflow: have=%u add=%u — truncating",
                 (unsigned)s_stash_len, (unsigned)len);
        len = SDIO_STASH_SIZE - s_stash_len;
    }
    memcpy(s_stash_buf + s_stash_len, data, len);
    s_stash_len += len;
}

/* ---- Helper: delay in ms (FreeRTOS) ---------------------------------- */
static inline void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ==== INIT SEQUENCE (matches official esp_slave_init_io) ============== */

static esp_err_t c5_sdio_enable_function(sdmmc_card_t *card)
{
    esp_err_t err;
    uint8_t val;

    /* 1. Read current I/O Enable (CCCR 0x02) */
    err = sdmmc_io_read_byte(card, 0, SD_IO_CCCR_FN_ENABLE, &val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read CCCR IOE failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "CCCR IOE: 0x%02x", val);

    /* 2. Read I/O Ready (CCCR 0x03) */
    uint8_t ior = 0;
    err = sdmmc_io_read_byte(card, 0, SD_IO_CCCR_FN_READY, &ior);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read CCCR IOR failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "CCCR IOR: 0x%02x", ior);

    /* 3. Enable Function 1 AND Function 2 (bits 1 and 2 → 0x06).
     *    Official code always enables both functions. */
    val = 0x06;  /* Fn1 + Fn2 */
    err = sdmmc_io_write_byte(card, 0, SD_IO_CCCR_FN_ENABLE, val, &val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write CCCR IOE=0x06 failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "CCCR IOE (after enable): 0x%02x", val);

    /* 4. Write to IOR to signal which functions the host expects ready.
     *    Official code writes IOE value to IOR. */
    ior = 0x06;
    err = sdmmc_io_write_byte(card, 0, SD_IO_CCCR_FN_READY, val, &ior);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write CCCR IOR=0x06 failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "CCCR IOR (after write): 0x%02x", ior);

    /* 5. Enable interrupts: master enable (bit 0) + Fn1 (bit 1) + Fn2 (bit 2) = 0x07 */
    err = sdmmc_io_read_byte(card, 0, SD_IO_CCCR_INT_ENABLE, &val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read CCCR IE failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "CCCR IE: 0x%02x", val);

    val = 0x07;  /* master + Fn1 + Fn2 */
    err = sdmmc_io_write_byte(card, 0, SD_IO_CCCR_INT_ENABLE, val, &val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write CCCR IE=0x07 failed: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "CCCR IE (after enable): 0x%02x", val);

    /* 6. Set Function 0 block size to 512 (CCCR 0x10-0x11) */
    uint8_t bsl, bsh;
    bsl = 0x00;
    err = sdmmc_io_write_byte(card, 0, SD_IO_CCCR_BLKSIZEL, bsl, &bsl);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Function 0 BSL: 0x%02x", bsl);

    bsh = 0x02;  /* 512 = 0x0200 */
    err = sdmmc_io_write_byte(card, 0, SD_IO_CCCR_BLKSIZEH, bsh, &bsh);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Function 0 BSH: 0x%02x", bsh);

    /* 7. Set Function 1 block size to 512 (FBR at 0x100 + 0x10 = 0x110) */
    bsl = 0x00;
    err = sdmmc_io_write_byte(card, 0, 0x110, bsl, &bsl);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Function 1 BSL: 0x%02x", bsl);

    bsh = 0x02;
    err = sdmmc_io_write_byte(card, 0, 0x111, bsh, &bsh);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Function 1 BSH: 0x%02x", bsh);

    /* 8. Set Function 2 block size to 512 (FBR at 0x200 + 0x10 = 0x210) */
    bsl = 0x00;
    err = sdmmc_io_write_byte(card, 0, 0x210, bsl, &bsl);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Function 2 BSL: 0x%02x", bsl);

    bsh = 0x02;
    err = sdmmc_io_write_byte(card, 0, 0x210, bsh, &bsh);  /* note: official writes to 0x210 for BSH too */
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Function 2 BSH: 0x%02x", bsh);

    ESP_LOGI(TAG, "SDIO function enable complete (Fn1 + Fn2, block_size=%u)",
             (unsigned)C5_SDIO_BLOCK_SIZE);
    return ESP_OK;
}

/* ==== PUBLIC API ======================================================= */

esp_err_t c5_sdio_transport_init(sdmmc_card_t *card)
{
    s_card = card;

    ESP_LOGI(TAG, "Transport init: card=%p is_sdio=%u io_functions=%u",
             (void *)card,
             (unsigned)card->is_sdio,
             (unsigned)card->num_io_functions);
    ESP_LOGI(TAG, "  CID: %s  Max freq: %u kHz",
             card->cid.name, (unsigned)card->max_freq_khz);

    /* Full init sequence from official esp_slave_init_io() */
    esp_err_t err = c5_sdio_enable_function(card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "c5_sdio_enable_function failed: 0x%x (%s)", err, esp_err_to_name(err));
        return err;
    }

    /* Enable SDIO interrupts from slave */
    err = sdmmc_io_enable_int(card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sdmmc_io_enable_int failed: 0x%x (%s)", err, esp_err_to_name(err));
        return err;
    }

    /* Reset flow-control counters (mirrors official sdio_init) */
    tx_sent_buffers = 0;
    rx_got_bytes   = 0;

    ESP_LOGI(TAG, "SDIO transport init OK");
    return ESP_OK;
}

/* ==== TX FLOW CONTROL (from official esp_sdio_host_get_buffer_size) ==== */

static uint32_t esp_sdio_host_get_buffer_size(void)
{
    uint32_t len;
    esp_err_t err = sdmmc_io_read_bytes(s_card, 1, REG_SDIO_TOKEN_RDATA,
                                        (uint8_t *)&len, 4);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read token/rdata error: 0x%x", err);
        return 0;
    }

    len = (len >> REG_SDIO_SEND_OFFSET) & TX_BUFFER_MASK;
    len = (len + TX_BUFFER_MAX - tx_sent_buffers) % TX_BUFFER_MAX;
    return len;
}

/* ==== HOST → SLAVE INTERRUPT (from official sdio_host_send_intr) ====== */

static esp_err_t esp_sdio_send_intr(uint8_t intr_no)
{
    if (intr_no >= 8) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t intr_mask = 0x1 << (intr_no + REG_SDIO_CONF_OFFSET);
    esp_err_t err = sdmmc_io_write_byte(s_card, 1, REG_SDIO_CONF,
                                        (uint8_t)intr_mask, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send_intr(%u) failed: 0x%x", intr_no, err);
    }
    return err;
}

/* ==== WRITE (from official sdio_host_send_packet) ====================== */

esp_err_t c5_sdio_write(int function, uint32_t addr, const void *data, size_t len)
{
    if (!s_card) {
        return ESP_ERR_INVALID_STATE;
    }
    (void)addr;    /* unused — protocol uses 0x1F800 - len */
    (void)function; /* always Fn1 */

    if (len == 0) {
        return ESP_OK;
    }

    /* ---- Flow control: wait for slave buffer space ---- */
    int buffer_used = (len + C5_SDIO_BLOCK_SIZE - 1) / C5_SDIO_BLOCK_SIZE;
    uint32_t cnt = 10000;  /* max retries */

    while (1) {
        uint32_t num = esp_sdio_host_get_buffer_size();

        if (num * C5_SDIO_BLOCK_SIZE < len) {
            if (!--cnt) {
                ESP_LOGE(TAG, "TX buffer full: need %d blocks, avail %lu", buffer_used, num);
                return ESP_ERR_TIMEOUT;
            }
            delay_ms(1);
            continue;
        }
        break;
    }

    /* ---- Send data using byte-mode CMD53 only.
     *
     *    On ESP32-P4 + C5 SDIO slave, sdmmc_io_write_blocks() (CMD53 block mode)
     *    returns ESP_ERR_INVALID_RESPONSE (0x109).  Only sdmmc_io_write_bytes()
     *    works correctly.  The 4-byte alignment padding added by the driver is
     *    discarded by the C5 slave hardware (data length is derived from address
     *    offset, not byte count). */
    uint8_t *start_ptr = (uint8_t *)data;
    uint32_t len_remain = len;
    esp_err_t err = ESP_OK;

    do {
        int len_to_send = (len_remain < C5_SDIO_BLOCK_SIZE)
                        ? (int)len_remain : C5_SDIO_BLOCK_SIZE;

        err = sdmmc_io_write_bytes(s_card, 1,
                                   ESP_SLAVE_CMD53_END_ADDR - len_remain,
                                   start_ptr, (len_to_send + 3) & (~3));

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "CMD53 write failed: fn=1 addr=0x%05lx len=%d err=0x%x (%s)",
                     (unsigned long)(ESP_SLAVE_CMD53_END_ADDR - len_remain),
                     len_to_send, err, esp_err_to_name(err));
            return err;
        }

        start_ptr += len_to_send;
        len_remain -= len_to_send;
    } while (len_remain);

    /* ---- Update flow control ---- */
    if (tx_sent_buffers >= TX_BUFFER_MAX) {
        tx_sent_buffers -= TX_BUFFER_MAX;
    }
    tx_sent_buffers += buffer_used;

    /* ---- Notify slave that data is ready (official: send interrupt) ---- */
    esp_sdio_send_intr(0);

    return ESP_OK;
}

/* ==== RX DATA SIZE CHECK (from official esp_sdio_slave_get_rx_data_size) */

static esp_err_t esp_sdio_slave_get_rx_data_size(uint32_t *rx_size)
{
    uint32_t len;
    esp_err_t err = sdmmc_io_read_bytes(s_card, 1, REG_SDIO_PKT_LEN,
                                        (uint8_t *)&len, 4);
    if (err != ESP_OK) {
        return err;
    }
    len &= RX_BYTE_MASK;
    len = (len + RX_BYTE_MAX - rx_got_bytes) % RX_BYTE_MAX;
    *rx_size = len;
    return ESP_OK;
}

/* ==== READ (polling variant — caller handles timeout) ================== */

int c5_sdio_read(int function, uint32_t addr, void *buf, size_t max_len, uint32_t timeout_ms)
{
    if (!s_card) {
        return -1;
    }
    (void)addr;
    (void)function;

    /* 1. Return stashed data first (saved from a previous read that
     *    consumed more data than it needed, e.g. +IPD after SEND OK). */
    if (s_stash_len > 0) {
        size_t copy = (s_stash_len < max_len) ? s_stash_len : max_len;
        memcpy(buf, s_stash_buf, copy);
        if (copy < s_stash_len) {
            memmove(s_stash_buf, s_stash_buf + copy, s_stash_len - copy);
        }
        s_stash_len -= copy;
        return (int)copy;
    }

    uint32_t len = 0;
    esp_err_t err;

    /* 2. Poll for data availability from slave.
     *
     *    OFFICIAL PATTERN (from sdio_host_get_packet in esp-at-c5 reference):
     *    Poll REG_SDIO_PKT_LEN in a tight 1-ms loop until data appears or
     *    timeout expires.  No sdmmc_io_wait_int() inside the read path —
     *    the interrupt is only used as a high-level wake-up signal in a
     *    separate receive task (sdio_recv_task), which THEN calls this
     *    polling function.  Polling is more reliable because:
     *      - No dependency on SDIO interrupt signalling (ESP32-P4 +
     *        C5 slave may differ)
     *      - No need to clear interrupt status registers
     *      - Data is detected as soon as it lands in the slave buffer */
    if (timeout_ms == 0) {
        /* Single-shot check — no polling */
        err = esp_sdio_slave_get_rx_data_size(&len);
        if (err != ESP_OK || len == 0) {
            return -1;
        }
    } else {
        /* Poll with 1-ms grain, matching official sdio_host_get_packet() */
        uint32_t waited = 0;
        int logged_polling = 0;
        for (;;) {
            err = esp_sdio_slave_get_rx_data_size(&len);
            if (err == ESP_OK && len > 0) {
                if (logged_polling) {
                    ESP_LOGI(TAG, "poll: data arrived after %lu ms, len=%lu",
                             (unsigned long)waited, (unsigned long)len);
                }
                break;  /* data available */
            }
            if (err != ESP_OK) {
                return -1;  /* register read error */
            }
            if (waited >= timeout_ms) {
                return -1;  /* timeout — no data within window */
            }
            if (!logged_polling && waited >= 100) {
                logged_polling = 1;
                /* Only at DEBUG — use c5_sdio_dump_rx_state() for INFO */
                ESP_LOGD(TAG, "poll: still waiting (waited=%lu ms, rx_got=%lu)",
                         (unsigned long)waited, (unsigned long)rx_got_bytes);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            waited++;
        }
    }

    if (len > max_len) {
        len = max_len;
    }

    /* 2.5 Sanity check: if the slave reports an amount of data that seems
     *     stale (e.g. after a C5 partial reset), the CMD53 below will fail
     *     with ESP_FAIL.  Re-sync the counter from hardware when len is
     *     suspiciously large relative to what a single AT response can be.
     *     A full 512-byte block read failing repeatedly is the tell-tale. */
    static int s_consecutive_read_fails = 0;

    /* Read data using byte-mode CMD53 only.
     * On ESP32-P4 + C5 SDIO slave, sdmmc_io_read_blocks() fails with
     * ESP_ERR_INVALID_RESPONSE (0x109).  Byte-mode with 4-byte alignment works;
     * padding bytes are discarded by the slave hardware. */
    uint32_t len_remain = len;
    uint8_t *start_ptr = (uint8_t *)buf;

    do {
        int len_to_read = (len_remain < C5_SDIO_BLOCK_SIZE)
                        ? (int)len_remain : C5_SDIO_BLOCK_SIZE;

        err = sdmmc_io_read_bytes(s_card, 1,
                                  ESP_SLAVE_CMD53_END_ADDR - len_remain,
                                  start_ptr, (len_to_read + 3) & (~3));

        if (err != ESP_OK) {
            s_consecutive_read_fails++;
            ESP_LOGE(TAG, "CMD53 read failed: fn=1 addr=0x%05lx len=%d err=0x%x",
                     (unsigned long)(ESP_SLAVE_CMD53_END_ADDR - len_remain),
                     len_to_read, err);

            /* If reads keep failing, re-sync rx_got_bytes from the
             * hardware register to clear any drift between host and
             * slave counters.  Drop the stale read and let the caller
             * retry with the corrected counter. */
            if (s_consecutive_read_fails >= 3) {
                ESP_LOGW(TAG, "re-syncing rx_got_bytes (was %lu) from hardware",
                         (unsigned long)rx_got_bytes);
                uint32_t raw;
                if (sdmmc_io_read_bytes(s_card, 1, REG_SDIO_PKT_LEN,
                                        (uint8_t *)&raw, 4) == ESP_OK) {
                    rx_got_bytes = (raw & RX_BYTE_MASK);
                }
                s_consecutive_read_fails = 0;
            }
            return -1;
        }

        s_consecutive_read_fails = 0;  /* successful read clears the count */

        start_ptr += len_to_read;
        len_remain -= len_to_read;
    } while (len_remain);

    rx_got_bytes += len;
    return (int)len;
}

/* ==== INTERRUPT HANDLING (matches official sdio_host_transport) ========= */

esp_err_t c5_sdio_wait_int(uint32_t timeout_ms)
{
    if (!s_card) {
        return ESP_ERR_INVALID_STATE;
    }
    return sdmmc_io_wait_int(s_card, pdMS_TO_TICKS(timeout_ms));
}

/**
 * @brief Read SDIO slave interrupt status (raw + masked).
 *
 * Matches official sdio_host_get_intr():
 *   Reads ESP_SDIO_INT_RAW (0x050) and ESP_SDIO_INT_ST (0x058).
 *
 * @param intr_raw  [out] Raw interrupt bits (may be NULL).
 * @param intr_st   [out] Masked interrupt bits (may be NULL).
 * @return ESP_OK on success.
 */
esp_err_t c5_sdio_get_intr(uint32_t *intr_raw, uint32_t *intr_st)
{
    if (!s_card) {
        return ESP_ERR_INVALID_STATE;
    }

    if (intr_raw) {
        esp_err_t err = sdmmc_io_read_bytes(s_card, 1, REG_SDIO_INT_RAW,
                                            (uint8_t *)intr_raw, 4);
        if (err != ESP_OK) return err;
    }

    if (intr_st) {
        esp_err_t err = sdmmc_io_read_bytes(s_card, 1, REG_SDIO_INT_ST,
                                            (uint8_t *)intr_st, 4);
        if (err != ESP_OK) return err;
    }

    return ESP_OK;
}

/**
 * @brief Clear SDIO slave interrupt bits.
 *
 * Matches official sdio_host_clear_intr():
 *   Writes mask to ESP_SDIO_INT_CLR (0x0D4).
 *
 * @param intr_mask  Bitmask of interrupts to clear.
 * @return ESP_OK on success.
 */
esp_err_t c5_sdio_clear_intr(uint32_t intr_mask)
{
    if (!s_card) {
        return ESP_ERR_INVALID_STATE;
    }
    return sdmmc_io_write_bytes(s_card, 1, REG_SDIO_INT_CLR,
                                (uint8_t *)&intr_mask, 4);
}

/* Interrupt bit definitions (from official sdio_host_transport.h) */
#define HOST_SLC0_RX_NEW_PACKET_INT_ST   (BIT(23))
#define HOST_SLC0_TOHOST_BIT0_INT_ST     (BIT(0))

/* ==== CONGESTION FEEDBACK ================================================= */

int c5_sdio_get_tx_congestion_pct(void)
{
    if (!s_card) {
        return 0;
    }

    /* Read the TOKEN_RDATA register which reports how many 512-byte blocks
     * the slave has consumed (i.e. how much TX buffer is free).
     *
     *   token = (REG_SDIO_TOKEN_RDATA >> 16) & 0xFFF    <- TX consumed blocks
     *   tx_sent_buffers                                   <- blocks we've sent
     *
     * The difference (sent - consumed) = blocks still in-flight / queued.
     * TX_BUFFER_MAX = 0x1000 blocks (theoretical max tracked). */
    uint32_t token_raw = 0;
    esp_err_t err = sdmmc_io_read_bytes(s_card, 1, REG_SDIO_TOKEN_RDATA,
                                        (uint8_t *)&token_raw, 4);
    if (err != ESP_OK) {
        return 0;
    }

    uint32_t consumed = (token_raw >> REG_SDIO_SEND_OFFSET) & TX_BUFFER_MASK;
    uint32_t in_flight = (tx_sent_buffers + TX_BUFFER_MAX - consumed) % TX_BUFFER_MAX;

    /* Normalize to percentage.  TX_BUFFER_MAX = 4096 blocks.
     * When in_flight > 75% of max, the link is congested. */
    unsigned pct = (unsigned long)in_flight * 100u / TX_BUFFER_MAX;
    if (pct > 100) pct = 100;
    return (int)pct;
}

/* ==== DIAGNOSTICS ======================================================== */

void c5_sdio_dump_rx_state(void)
{
    if (!s_card) {
        ESP_LOGI(TAG, "dump: no card");
        return;
    }

    uint32_t pkt_len_raw = 0, token_rdata = 0, intr_raw = 0, intr_st = 0;
    sdmmc_io_read_bytes(s_card, 1, REG_SDIO_PKT_LEN,
                        (uint8_t *)&pkt_len_raw, 4);
    sdmmc_io_read_bytes(s_card, 1, REG_SDIO_TOKEN_RDATA,
                        (uint8_t *)&token_rdata, 4);
    c5_sdio_get_intr(&intr_raw, &intr_st);

    uint32_t avail = ((pkt_len_raw & RX_BYTE_MASK) + RX_BYTE_MAX - rx_got_bytes) % RX_BYTE_MAX;
    uint32_t tx_avail = ((token_rdata >> REG_SDIO_SEND_OFFSET) & TX_BUFFER_MASK);
    tx_avail = (tx_avail + TX_BUFFER_MAX - tx_sent_buffers) % TX_BUFFER_MAX;

    ESP_LOGI(TAG, "dump: PKT_LEN=0x%08lx avail=%lu | TOKEN=0x%08lx tx_avail=%lu | INT_RAW=0x%08lx INT_ST=0x%08lx | rx_got=%lu",
             (unsigned long)pkt_len_raw, (unsigned long)avail,
             (unsigned long)token_rdata, (unsigned long)tx_avail,
             (unsigned long)intr_raw, (unsigned long)intr_st,
             (unsigned long)rx_got_bytes);
}
