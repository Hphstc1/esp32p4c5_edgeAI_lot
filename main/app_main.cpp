/*
 * p4_face_stream: face recognition + MJPEG streaming on the WT99P4C5-S1.
 *
 * Data flow:
 *   OV5647 (MIPI-CSI)  ──►  V4L2 RGB565 frames  ──►  FaceAi (every Nth frame)
 *                                              \─►  draw::annotate_faces
 *                                              \─►  jpeg_encoder
 *                                              \─►  HttpServer.push_jpeg
 *                                                   │ lwIP (WiFi AP + Ethernet)
 *                                                   ▼
 *                                              PC Browser
 *
 * WiFi AP: SSID "P4-FaceStream", password "12345678", IP 192.168.4.1
 * Ethernet: disabled for this wireless-only test build
 *
 * Endpoints (port 8080)
 *   GET  /              dashboard HTML
 *   GET  /stream        multipart MJPEG with face boxes drawn
 *   GET  /api/info      device / FPS / enrollment JSON
 *   GET  /api/events    latest recognition event JSON
 *   POST /api/enroll    trigger one-shot enrollment on next frame
 *   POST /api/delete    drop the last enrolled face
 */
/* ---- WiFi / HTTP enable switch ---------------------------------- */
#define WIFI_HTTP_ENABLE  1   // 置 1 开启 WiFi+HTTP, 置 0 关闭仅测推理

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/jpeg_encode.h"
#include "esp_video_init.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"

#include "bsp/esp-bsp.h"
#include "app_video.h"
#include "face_ai.hpp"
#include "jpeg_annotate.hpp"
#include "http_server.hpp"

static const char *TAG = "p4fs";

/* ---- Shared state with the HTTP layer ------------------------------ */
namespace p4fs {
std::atomic<int>     g_enroll_pending{0};     // set by /api/enroll
std::atomic<int>     g_last_enroll_id{-2};    // -2=no attempt, -1=failed, >=1=enrolled id
uint32_t             g_cam_fps_x100 = 0;      // updated every second
}

/* ---- Camera + encoder + face-ai + HTTP handles ---------------------- */
static p4fs::FaceAi     *g_face_ai     = nullptr;
static p4fs::HttpServer *g_http_server = nullptr;
static jpeg_encoder_handle_t  g_jpeg         = nullptr;
static uint8_t               *g_jpeg_out     = nullptr;
static size_t                 g_jpeg_out_sz  = 0;

/* Per-frame scratch: full-resolution RGB565 buffer for JPEG encode + draw. */
static uint8_t *g_scratch_rgb = nullptr;
static size_t   g_scratch_bytes = 0;
static SemaphoreHandle_t g_scratch_lock = nullptr;

/* We process face AI every kProcessEveryN frames to keep CPU headroom for
 * JPEG encoding. The annotator is cheap, so it runs on every frame. */
static constexpr int kProcessEveryN = 3;

/* ---- WiFi SoftAP via esp_wifi_remote / esp_hosted ------------------- */
#if WIFI_HTTP_ENABLE
static void wifi_init_softap(void) {
    ESP_LOGI(TAG, "WiFi: initializing SoftAP ...");

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (!ap_netif) {
        ESP_LOGE(TAG, "Failed to create default WiFi AP netif");
        abort();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap_config = {};
    const char *ssid = "P4-FaceStream";
    const char *pass = "12345678";
    strncpy((char *)ap_config.ap.ssid, ssid, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = (uint8_t)strlen(ssid);
    strncpy((char *)ap_config.ap.password, pass, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "WiFi SoftAP started: SSID=%s, IP=192.168.4.1", ssid);
}
#endif  // WIFI_HTTP_ENABLE

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== p4_face_stream starting ===");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* --- Storage: SPIFFS for the face DB ----------------------- */
    esp_vfs_spiffs_conf_t spiffs_cfg = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t sp = esp_vfs_spiffs_register(&spiffs_cfg);
    if (sp != ESP_OK) {
        ESP_LOGE(TAG, "spiffs mount failed: %s", esp_err_to_name(sp));
    } else {
        size_t total = 0, used = 0;
        esp_spiffs_info("storage", &total, &used);
        ESP_LOGI(TAG, "SPIFFS: total=%u used=%u", (unsigned)total, (unsigned)used);
    }

    /* --- Ethernet BSP init (kept for board-level power/clock setup) --
     * Make failure non-fatal: the wireless-only test path only needs the
     * board-level clocks/power to be up.  If the PHY doesn't respond this
     * boot, keep running on WiFi instead of aborting. */
    esp_err_t eth_err = bsp_eth_init();
    if (eth_err != ESP_OK) {
        ESP_LOGW(TAG, "bsp_eth_init failed: %s (continuing without Ethernet)",
                 esp_err_to_name(eth_err));
    }

    /* --- Static IP disabled: wireless-only test, do not bring up eth IP -- */
#if 0
    {
        esp_netif_t *n = esp_netif_get_handle_from_ifkey("ETH_DEF");
        if (n) {
            esp_netif_ip_info_t ip = {};
            esp_netif_str_to_ip4("192.168.1.200", &ip.ip);
            esp_netif_str_to_ip4("192.168.1.1",   &ip.gw);
            esp_netif_str_to_ip4("255.255.255.0", &ip.netmask);
            esp_netif_dhcpc_stop(n);
            esp_netif_set_ip_info(n, &ip);
        }
    }
    ESP_LOGI(TAG, "Ethernet up, IP 192.168.1.200");
#endif

    /* --- Face AI (loads model from flash) ---------------------- */
    p4fs::FaceAiConfig fcfg;
    fcfg.db_path = "/spiffs/face.db";
    auto *face_ai = new p4fs::FaceAi(fcfg);
    ESP_ERROR_CHECK(face_ai->init() ? ESP_OK : ESP_FAIL);
    g_face_ai = face_ai;

    /* --- WiFi SoftAP via C5 SDIO (esp_wifi_remote + esp_hosted) */
#if WIFI_HTTP_ENABLE
    wifi_init_softap();
#endif

    /* --- Unified HTTP server (port 8080, both WiFi and Ethernet) */
#if WIFI_HTTP_ENABLE
    {
        p4fs::HttpServerConfig hcfg;
        hcfg.port = 8080;
        // esp_http_server reserves 3 sockets internally and caps max_open_sockets
        // at LWIP_MAX_SOCKETS (7 by default).  7 gives room for 2 concurrent
        // /stream viewers plus polling of /api/info and /api/events.
        hcfg.max_clients = 7;
        hcfg.jpeg_quality = 80;
        auto *http = new p4fs::HttpServer(hcfg, face_ai);
        ESP_ERROR_CHECK(http->start() ? ESP_OK : ESP_FAIL);
        g_http_server = http;
        ESP_LOGI(TAG, "HTTP server started on port %d", hcfg.port);
    }
#endif

    /* --- Camera + encoder scratch ------------------------------ */
    static const esp_video_init_csi_config_t csi_cfg[] = {{
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = { .port = 0, .scl_pin = GPIO_NUM_8, .sda_pin = GPIO_NUM_7 },
            .freq = 100000,
        },
        .reset_pin = GPIO_NUM_46,
        .pwdn_pin  = GPIO_NUM_NC,
    }};
    static const esp_video_init_config_t vcfg = { .csi = csi_cfg };
    ESP_ERROR_CHECK(esp_video_init(&vcfg));

    int cam_fd = app_video_open(EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT_RGB565);
    if (cam_fd < 0) { ESP_LOGE(TAG, "camera open failed"); abort(); }
    ESP_ERROR_CHECK(app_video_set_bufs(cam_fd, EXAMPLE_CAM_BUF_NUM, nullptr));

    g_scratch_lock = xSemaphoreCreateMutex();

    /* JPEG encoder (hardware). Allocate worst-case output buffer = input / 2. */
    jpeg_encode_engine_cfg_t jec = { .timeout_ms = 5000 };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&jec, &g_jpeg));
    {
        jpeg_encode_memory_alloc_cfg_t mac = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
        uint32_t alloc = app_video_get_buf_size() / 2;
        g_jpeg_out = (uint8_t *)jpeg_alloc_encoder_mem(alloc, &mac, &g_jpeg_out_sz);
        if (!g_jpeg_out) { ESP_LOGE(TAG, "jpeg out alloc failed"); abort(); }
        ESP_LOGI(TAG, "jpeg: out buf alloc=%u actual_sz=%u (frame raw=%u)",
                 (unsigned)alloc, (unsigned)g_jpeg_out_sz,
                 (unsigned)app_video_get_buf_size());
    }

    /* --- FPS counter task -------------------------------------- */
    static uint32_t s_fc = 0;
    xTaskCreatePinnedToCore([](void *) {
        uint32_t last = 0;
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            uint32_t now = s_fc;
            p4fs::g_cam_fps_x100 = (now - last) * 100;
            last = now;
        }
    }, "fps", 2048, nullptr, 1, nullptr, 1);

    /* --- Per-frame callback (runs on V4L2 capture task) -------- */
    static int s_frame_idx = 0;
    static int64_t s_last_frame_us = 0;
    auto frame_cb = [](uint8_t *buf, uint8_t idx, uint32_t w, uint32_t h, size_t len) {
        (void)idx; (void)len;
        s_fc++;

        /* Diagnostic: log first 3 frames to confirm pipeline health */
        if (s_frame_idx < 3) {
            ESP_LOGI(TAG, "frame[%d]: buf=%p idx=%u w=%u h=%u len=%u (expected=%u)",
                     s_frame_idx, (void*)buf, (unsigned)idx,
                     (unsigned)w, (unsigned)h, (unsigned)len,
                     (unsigned)(w * h * 2));
        }

        /* Use a separate scratch buffer so we can mutate it without racing
         * the camera DMA. The lock makes the scratch buffer exclusive to us. */
        if (xSemaphoreTake(g_scratch_lock, 0) != pdTRUE) {
            // Another frame is still being processed; just drop this one.
            return;
        }
        if (!g_scratch_rgb || g_scratch_bytes < (size_t)(w * h * 2)) {
            g_scratch_bytes = w * h * 2;
            free(g_scratch_rgb);
            /* TEST: force internal RAM to rule out SDIO/PSRAM interference */
            g_scratch_rgb = (uint8_t *)heap_caps_malloc(g_scratch_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!g_scratch_rgb) {
                g_scratch_rgb = (uint8_t *)heap_caps_malloc(g_scratch_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            }
            if (!g_scratch_rgb) {
                g_scratch_rgb = (uint8_t *)malloc(g_scratch_bytes);
            }
            if (g_scratch_rgb) {
                ESP_LOGI(TAG, "scratch alloc ok, %u bytes", (unsigned)g_scratch_bytes);
            }
        }
        if (!g_scratch_rgb) { xSemaphoreGive(g_scratch_lock); return; }
        std::memcpy(g_scratch_rgb, buf, w * h * 2);

        /* Face AI: run every Nth frame after warm-up (first 5 frames are
         * just the JPEG feed so the model can stabilise). */
        std::vector<p4fs::FaceHit> hits;
        if (s_frame_idx >= 5 && (s_frame_idx % kProcessEveryN) == 0 && g_face_ai) {
            hits = g_face_ai->process(g_scratch_rgb, (int)w, (int)h);
        }

        /* Honor pending enroll request on EVERY frame after warm-up. */
        if (s_frame_idx >= 5 && g_face_ai &&
            p4fs::g_enroll_pending.exchange(0) == 1) {
            ESP_LOGI(TAG, "enroll: triggered on frame %d (w=%u h=%u)",
                     s_frame_idx, (unsigned)w, (unsigned)h);
            int id = g_face_ai->enroll_largest(g_scratch_rgb, (int)w, (int)h);
            p4fs::g_last_enroll_id.store(id);
            ESP_LOGI(TAG, "enroll result: id=%d (pending cleared)", id);
        }
        s_frame_idx++;

        /* Draw boxes (cheap) on every frame so the stream stays annotated. */
        if (!hits.empty()) {
            p4fs::draw::annotate_faces(g_scratch_rgb, (int)w, (int)h, hits);
        }

        /* Frame rate limiter: cap push to HTTP at ~5 FPS.
         * Face AI processing above is unaffected by this skip. */
        int64_t now = esp_timer_get_time();
        if (now - s_last_frame_us < 200000) {
            xSemaphoreGive(g_scratch_lock);
            return;
        }
        s_last_frame_us = now;

        /* Lower JPEG quality to reduce SDIO/WiFi backpressure; face boxes still readable. */
        static uint32_t s_jpeg_quality = 60;

#if WIFI_HTTP_ENABLE
        jpeg_encode_cfg_t jc = {
            .height = h, .width = w,
            .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
            .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
            .image_quality = s_jpeg_quality,
        };
        uint32_t out_len = 0;
        if (jpeg_encoder_process(g_jpeg, &jc, g_scratch_rgb, w * h * 2,
                                 g_jpeg_out, g_jpeg_out_sz, &out_len) == ESP_OK) {
            if (g_http_server) g_http_server->push_jpeg(g_jpeg_out, out_len);
        }
#endif
        xSemaphoreGive(g_scratch_lock);
    };
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(frame_cb));
    ESP_ERROR_CHECK(app_video_stream_task_start(cam_fd, 0));

    ESP_LOGI(TAG, "Streaming: connect to AP 'P4-FaceStream' at http://192.168.4.1:8080/");
    ESP_LOGI(TAG, "startup: free_heap=%u min_free=%u psram_free=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "alive frames=%u fps=%.1f enrolled=%d "
                 "free_heap=%u psram_free=%u",
                 (unsigned)s_fc, p4fs::g_cam_fps_x100 / 100.0f,
                 g_face_ai ? g_face_ai->num_enrolled() : -1,
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
}
