/*
 * c5_sdio_probe.c — P4 SDIO host detects C5 SDIO slave.
 *
 * SDIO pin mapping (WT99P4C5-S1 schematic):
 *   P4 GPIO14 → C5 GPIO8  (D0)
 *   P4 GPIO15 → C5 GPIO7  (D1)
 *   P4 GPIO16 → C5 GPIO14 (D2)
 *   P4 GPIO17 → C5 GPIO13 (D3)
 *   P4 GPIO18 → C5 GPIO9  (CLK)
 *   P4 GPIO19 → C5 GPIO10 (CMD)
 *   P4 GPIO13 → C5 CHIP_PU (reset)
 */
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "c5_bridge.h"

static const char *TAG = "c5_sdio";

static sdmmc_card_t *s_card = NULL;

/* ---- SDIO GPIOs ---------------------------------------------------- */
#define C5_SDIO_CLK  GPIO_NUM_18
#define C5_SDIO_CMD  GPIO_NUM_19
#define C5_SDIO_D0   GPIO_NUM_14
#define C5_SDIO_D1   GPIO_NUM_15
#define C5_SDIO_D2   GPIO_NUM_16
#define C5_SDIO_D3   GPIO_NUM_17
#define C5_RST_GPIO  GPIO_NUM_13  /* → C5 CHIP_PU */

/* ---- Public API ---------------------------------------------------- */

esp_err_t c5_sdio_probe(void)
{
    ESP_LOGI(TAG, "=== C5 SDIO Probe ===");

    /* 1. Reset C5 via GPIO13 → CHIP_PU */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = BIT64(C5_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_cfg);

    ESP_LOGI(TAG, "Resetting C5 via GPIO%d (200ms low pulse)...", C5_RST_GPIO);
    gpio_set_level(C5_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(200));  /* longer pulse for clean reset */
    gpio_set_level(C5_RST_GPIO, 1);
    ESP_LOGI(TAG, "Waiting for C5 to boot ESP-AT + SDIO slave...");
    vTaskDelay(pdMS_TO_TICKS(3500)); /* C5 ESP-AT: ROM→bootloader→WiFi/PHY→SDIO init */

    /* 2. Initialize SDMMC host — 4-bit mode (hardware D0-D3 all connected).
     *
     *    IMPORTANT: Set max_freq_khz = SDMMC_FREQ_DEFAULT (20 MHz) from the
     *    start, matching the official at_sdio_host example.  The actual probing
     *    is always done at 400 kHz (sdmmc_host_init_slot forces it), then
     *    sdmmc_card_init() negotiates up to max_freq_khz internally, applying
     *    the input_delay_phase during the clock raise.  This avoids the P4-only
     *    problem of running CMD53 at 20 MHz without the correct sampling phase.
     *
     *    On ESP32-P4 the SDMMC controller requires input_delay_phase tuning for
     *    reliable multi-byte CMD53 at high speed.  Without it, CMD53 returns R5
     *    response errors (0x109 / ESP_ERR_INVALID_RESPONSE) while CMD52 works.
     *    Phase 2 is the empirically verified value for the WT99P4C5-S1 board.
     *
     *    C5 (ESP-AT SDIO slave) automatically adapts to the host bus width —
     *    no change needed on the C5 side. */
    static sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_4BIT | SDMMC_HOST_FLAG_ALLOC_ALIGNED_BUF;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;  /* 20 MHz — card_init handles the raise */
    host.input_delay_phase = SDMMC_DELAY_PHASE_2;  /* P4 SDMMC sampling phase */
    /* host.slot is set to SDMMC_HOST_SLOT_1 in SDMMC_HOST_DEFAULT */

    ESP_LOGI(TAG, "SDIO pins: CLK=%d CMD=%d D0=%d D1=%d D2=%d D3=%d RST=%d",
             C5_SDIO_CLK, C5_SDIO_CMD,
             C5_SDIO_D0, C5_SDIO_D1, C5_SDIO_D2, C5_SDIO_D3,
             C5_RST_GPIO);

    esp_err_t err = sdmmc_host_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sdmmc_host_init failed: 0x%x (%s)", err, esp_err_to_name(err));
        return err;
    }

    /* 3. Configure Slot 1 with our GPIOs */
    sdmmc_slot_config_t slot_cfg = {
        .clk = C5_SDIO_CLK,
        .cmd = C5_SDIO_CMD,
        .d0  = C5_SDIO_D0,
        .d1  = C5_SDIO_D1,
        .d2  = C5_SDIO_D2,
        .d3  = C5_SDIO_D3,
        .d4  = GPIO_NUM_NC,
        .d5  = GPIO_NUM_NC,
        .d6  = GPIO_NUM_NC,
        .d7  = GPIO_NUM_NC,
        .cd  = SDMMC_SLOT_NO_CD,
        .wp  = SDMMC_SLOT_NO_WP,
        .width = 4,   /* 4-bit mode (hardware D0-D3 all connected) */
        .flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP,
    };
    err = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sdmmc_host_init_slot failed: 0x%x (%s)", err, esp_err_to_name(err));
        return err;
    }

    /* 4. Probe for C5 SDIO card — retry like official at_sdio_host example,
     *    because the C5 SDIO slave may not be ready immediately after reset. */
    ESP_LOGI(TAG, "Probing SDIO on Slot 1...");
    sdmmc_card_t card;
    int retry;
    for (retry = 0; retry < 10; retry++) {
        err = sdmmc_card_init(&host, &card);
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "SDIO card init attempt %d/10 failed: 0x%x, retrying...",
                 retry + 1, err);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SDIO card init FAILED after %d attempts: 0x%x (%s)",
                 retry, err, esp_err_to_name(err));
        return err;
    }

    /* 5. Report */
    ESP_LOGI(TAG, "=== SDIO card DETECTED ===");
    ESP_LOGI(TAG, "  is_sdio: %u  is_mem: %u  is_mmc: %u",
             (unsigned)card.is_sdio, (unsigned)card.is_mem, (unsigned)card.is_mmc);
    ESP_LOGI(TAG, "  Num IO functions: %u", (unsigned)card.num_io_functions);
    ESP_LOGI(TAG, "  Max freq: %u kHz", (unsigned)card.max_freq_khz);
    ESP_LOGI(TAG, "  CID name: %s", card.cid.name);

    sdmmc_card_print_info(stdout, &card);

    /* 6. Save card handle (heap copy since card is stack-local) and init transport */
    sdmmc_card_t *p = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));
    if (p != NULL) {
        memcpy(p, &card, sizeof(card));
        s_card = p;
        c5_sdio_transport_init(s_card);
        /* Clock is already at full speed (20 MHz) — sdmmc_card_init()
         * raised it from probing 400 kHz with correct input_delay_phase.
         * No separate c5_sdio_set_clock() call needed. */
        ESP_LOGI(TAG, "SDIO clock at %u kHz (raised by card_init with delay_phase=%d)",
                 (unsigned)host.max_freq_khz, (int)host.input_delay_phase);
    } else {
        ESP_LOGE(TAG, "Failed to allocate card handle copy");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t c5_sdio_reinit(void)
{
    ESP_LOGW(TAG, "=== C5 SDIO Reinit (watchdog) ===");

    if (!s_card) {
        /* No prior card — just do a full probe */
        ESP_LOGW(TAG, "no prior card, falling back to full probe");
        return c5_sdio_probe();
    }

    /* 1. Toggle C5 reset via GPIO13 */
    ESP_LOGI(TAG, "Toggling C5 reset (200ms low)...");
    gpio_set_level(C5_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(C5_RST_GPIO, 1);
    ESP_LOGI(TAG, "Waiting for C5 to re-boot (3.5s)...");
    vTaskDelay(pdMS_TO_TICKS(3500));

    /* 2. Re-enumerate the card.  The SDMMC host is still running; we
     *    just need to re-do card detection and initialization sequence.
     *    Re-init the slot first to be safe (clears any stale state). */
    sdmmc_slot_config_t slot_cfg = {
        .clk = C5_SDIO_CLK, .cmd = C5_SDIO_CMD,
        .d0  = C5_SDIO_D0,  .d1  = C5_SDIO_D1,
        .d2  = C5_SDIO_D2,  .d3  = C5_SDIO_D3,
        .d4  = GPIO_NUM_NC, .d5  = GPIO_NUM_NC,
        .d6  = GPIO_NUM_NC, .d7  = GPIO_NUM_NC,
        .cd  = SDMMC_SLOT_NO_CD, .wp  = SDMMC_SLOT_NO_WP,
        .width = 4,
        .flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP,
    };
    esp_err_t err = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "reinit: sdmmc_host_init_slot failed: 0x%x", err);
        return err;
    }

    /* 3. Re-probe with retries */
    static sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_4BIT | SDMMC_HOST_FLAG_ALLOC_ALIGNED_BUF;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    host.input_delay_phase = SDMMC_DELAY_PHASE_2;

    sdmmc_card_t card;
    int retry;
    for (retry = 0; retry < 10; retry++) {
        err = sdmmc_card_init(&host, &card);
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "reinit: attempt %d/10 failed: 0x%x", retry + 1, err);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "reinit FAILED after 10 attempts: 0x%x", err);
        return err;
    }

    /* 4. Update stored card handle */
    memcpy(s_card, &card, sizeof(card));
    ESP_LOGI(TAG, "reinit: card re-detected: %s freq=%u kHz",
             card.cid.name, (unsigned)card.max_freq_khz);

    /* 5. Re-init transport layer */
    err = c5_sdio_transport_init(s_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "reinit: transport init failed: 0x%x", err);
        return err;
    }

    ESP_LOGI(TAG, "=== C5 SDIO Reinit OK ===");
    return ESP_OK;
}

sdmmc_card_t *c5_sdio_get_card(void)
{
    return s_card;
}

esp_err_t c5_sdio_set_clock(uint32_t freq_khz)
{
    if (!s_card) {
        ESP_LOGE(TAG, "c5_sdio_set_clock: no card probed yet");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Raising SDIO clock to %u kHz (from %u kHz probing)...",
             (unsigned)freq_khz, (unsigned)SDMMC_FREQ_PROBING);

    esp_err_t err = sdmmc_host_set_card_clk(SDMMC_HOST_SLOT_1, freq_khz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sdmmc_host_set_card_clk(%u kHz) failed: 0x%x — "
                 "keeping current clock", (unsigned)freq_khz, err);
        return err;
    }

    ESP_LOGI(TAG, "SDIO clock raised to %u kHz successfully", (unsigned)freq_khz);
    return ESP_OK;
}
