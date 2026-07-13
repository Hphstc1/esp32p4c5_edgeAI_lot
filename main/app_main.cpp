/*
 * p4_face_stream: face recognition + MJPEG streaming on the WT99P4C5-S1.
 *
 * Data flow:
 *   OV5647 (MIPI-CSI)  -->  V4L2 RGB565 frames  -->  FaceAi (every Nth frame)
 *                                              \-->  draw::annotate_faces
 *                                              \-->  jpeg_encoder
 *                                              \-->  HttpServer.push_jpeg
 *                                                   | lwIP (WiFi STA)
 *                                                   V
 *                                              PC Browser
 *
 * WiFi STA: credentials + target IP stored in NVS namespace "p4cfg".
 * Serial console (UART0) commands: ssid, pass, target, save, reboot, status, help.
 *
 * Endpoints (port 8080)
 *   GET  /              dashboard HTML
 *   GET  /stream        multipart MJPEG with face boxes drawn
 *   GET  /api/info      device / FPS / enrollment / target_ip JSON
 *   GET  /api/events    latest recognition event JSON
 *   POST /api/enroll    trigger one-shot enrollment on next frame
 *   POST /api/delete    drop the last enrolled face
 */
/* ---- WiFi / HTTP enable switch ---------------------------------- */
#define WIFI_HTTP_ENABLE  1   // 1=enable WiFi+HTTP, 0=disable and test inference only

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
#include "esp_event.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/jpeg_encode.h"
#include "esp_video_init.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "lwip/ip4_addr.h"

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

/* ---- Persistent configuration (NVS namespace "p4cfg") --------------- */
constexpr char NVS_NS[]    = "p4cfg";
constexpr char KEY_SSID[]  = "wifi_ssid";
constexpr char KEY_PASS[]  = "wifi_pass";
constexpr char KEY_TARGET[]= "target_ip";
constexpr size_t SSID_MAX  = 32;
constexpr size_t PASS_MAX  = 64;
constexpr size_t IP_MAX    = 16;

/* Hard-coded demo credentials: hph666 / He4496385 / 192.168.43.242.
 * NVS values will override these only if they are non-empty. */
char g_wifi_ssid[SSID_MAX + 1] = "hph666";
char g_wifi_pass[PASS_MAX + 1] = "He4496385";
char g_target_ip[IP_MAX]       = "192.168.43.242";
char g_wifi_ip[IP_MAX]         = "0.0.0.0";
std::atomic<bool>    g_wifi_connected{false};
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

/* ---- NVS config helpers --------------------------------------------- */
static void cfg_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(p4fs::NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "NVS config namespace not found, using defaults");
        return;
    }

    size_t len = 0;
    if (nvs_get_str(h, p4fs::KEY_SSID, nullptr, &len) == ESP_OK && len > 1 && len <= sizeof(p4fs::g_wifi_ssid)) {
        nvs_get_str(h, p4fs::KEY_SSID, p4fs::g_wifi_ssid, &len);
    }
    if (nvs_get_str(h, p4fs::KEY_PASS, nullptr, &len) == ESP_OK && len > 1 && len <= sizeof(p4fs::g_wifi_pass)) {
        nvs_get_str(h, p4fs::KEY_PASS, p4fs::g_wifi_pass, &len);
    }
    if (nvs_get_str(h, p4fs::KEY_TARGET, nullptr, &len) == ESP_OK && len > 1 && len <= sizeof(p4fs::g_target_ip)) {
        nvs_get_str(h, p4fs::KEY_TARGET, p4fs::g_target_ip, &len);
    }

    nvs_close(h);
    ESP_LOGI(TAG, "NVS config loaded: ssid='%s' target_ip=%s",
             p4fs::g_wifi_ssid, p4fs::g_target_ip);
}

static void cfg_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(p4fs::NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        printf("save: failed to open NVS namespace (%s)\r\n", esp_err_to_name(err));
        return;
    }

    ESP_ERROR_CHECK(nvs_set_str(h, p4fs::KEY_SSID,  p4fs::g_wifi_ssid));
    ESP_ERROR_CHECK(nvs_set_str(h, p4fs::KEY_PASS,  p4fs::g_wifi_pass));
    ESP_ERROR_CHECK(nvs_set_str(h, p4fs::KEY_TARGET, p4fs::g_target_ip));
    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        printf("save: configuration saved to NVS\r\n");
    } else {
        printf("save: commit failed (%s)\r\n", esp_err_to_name(err));
    }
}

/* ---- Serial console task -------------------------------------------- */
static char *cmd_arg(char *line)
{
    char *p = strchr(line, ' ');
    if (!p) return nullptr;
    while (*p == ' ') ++p;
    return (*p == '\0') ? nullptr : p;
}

static void serial_console_task(void *arg)
{
    (void)arg;
    char line[128];

    vTaskDelay(pdMS_TO_TICKS(500));
    printf("\r\n");
    printf("P4 console ready. Commands: ssid <n> | pass <p> | target <ip> | save | reboot | status | help\r\n");
    if (p4fs::g_wifi_ssid[0] == '\0') {
        printf("WARNING: no WiFi SSID configured. Use 'ssid' and 'pass', then 'save' + 'reboot'.\r\n");
    }

    while (true) {
        printf("\r\nP4> ");
        fflush(stdout);

        /* Blocking line read: getchar() may return EOF when no data is ready,
         * so we poll slowly and do NOT re-print the prompt until a line is
         * complete. This avoids the P4> spam seen with fgets() on this console. */
        int idx = 0;
        bool got_any = false;
        while (idx < (int)sizeof(line) - 1) {
            int c = getchar();
            if (c == EOF) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            got_any = true;
            if (c == '\r' || c == '\n') {
                break;
            }
            line[idx++] = (char)c;
        }
        line[idx] = '\0';

        if (!got_any || line[0] == '\0') {
            continue;
        }

        if (strncmp(line, "ssid ", 5) == 0) {
            char *a = cmd_arg(line);
            if (a) {
                size_t len = strnlen(a, p4fs::SSID_MAX);
                std::memcpy(p4fs::g_wifi_ssid, a, len);
                p4fs::g_wifi_ssid[len] = '\0';
                printf("ssid set to '%s' (not saved yet)\r\n", p4fs::g_wifi_ssid);
            } else {
                printf("usage: ssid <name>\r\n");
            }
        } else if (strncmp(line, "pass ", 5) == 0) {
            char *a = cmd_arg(line);
            if (a) {
                size_t len = strnlen(a, p4fs::PASS_MAX);
                std::memcpy(p4fs::g_wifi_pass, a, len);
                p4fs::g_wifi_pass[len] = '\0';
                printf("pass set (len=%zu, not saved yet)\r\n", strlen(p4fs::g_wifi_pass));
            } else {
                printf("usage: pass <password>\r\n");
            }
        } else if (strncmp(line, "target ", 7) == 0) {
            char *a = cmd_arg(line);
            if (a) {
                size_t len = strnlen(a, p4fs::IP_MAX - 1);
                std::memcpy(p4fs::g_target_ip, a, len);
                p4fs::g_target_ip[len] = '\0';
                printf("target_ip set to '%s' (not saved yet)\r\n", p4fs::g_target_ip);
            } else {
                printf("usage: target <ip>\r\n");
            }
        } else if (strcmp(line, "save") == 0) {
            cfg_save();
        } else if (strcmp(line, "reboot") == 0) {
            printf("rebooting now...\r\n");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        } else if (strcmp(line, "status") == 0) {
            printf("ssid:      %s\r\n", p4fs::g_wifi_ssid[0] ? p4fs::g_wifi_ssid : "(not set)");
            printf("pass:      %s\r\n", p4fs::g_wifi_pass[0] ? "(set)" : "(not set)");
            printf("target_ip: %s\r\n", p4fs::g_target_ip);
            printf("wifi:      %s, ip=%s\r\n",
                   p4fs::g_wifi_connected.load() ? "connected" : "disconnected",
                   p4fs::g_wifi_ip);
        } else if (strcmp(line, "help") == 0) {
            printf("Commands:\r\n");
            printf("  ssid <name>   set WiFi SSID (max %zu chars)\r\n", p4fs::SSID_MAX);
            printf("  pass <pwd>    set WiFi password (max %zu chars)\r\n", p4fs::PASS_MAX);
            printf("  target <ip>   set target IP, e.g. 192.168.43.100\r\n");
            printf("  save          write config to NVS\r\n");
            printf("  reboot        restart the device\r\n");
            printf("  status        show current config and WiFi state\r\n");
            printf("  help          show this message\r\n");
        } else {
            printf("unknown command '%s'. Type 'help'.\r\n", line);
        }
    }
}

/* ---- WiFi STA via esp_wifi_remote / esp_hosted ---------------------- */
#if WIFI_HTTP_ENABLE
static constexpr int WIFI_MAX_RETRY = 10;
static int s_wifi_retry_num = 0;
static esp_event_handler_instance_t s_wifi_event_hdl = nullptr;
static esp_event_handler_instance_t s_ip_event_hdl = nullptr;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started, connecting ...");
        esp_wifi_connect();
        s_wifi_retry_num = 0;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        p4fs::g_wifi_connected.store(false);
        if (s_wifi_retry_num < WIFI_MAX_RETRY) {
            ESP_LOGI(TAG, "WiFi disconnected, retry %d/%d", s_wifi_retry_num + 1, WIFI_MAX_RETRY);
            esp_wifi_connect();
            s_wifi_retry_num++;
        } else {
            ESP_LOGE(TAG, "WiFi connection failed after %d retries", WIFI_MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(p4fs::g_wifi_ip, sizeof(p4fs::g_wifi_ip), IPSTR, IP2STR(&event->ip_info.ip));
        p4fs::g_wifi_connected.store(true);
        s_wifi_retry_num = 0;
        ESP_LOGI(TAG, "WiFi connected, IP=%s", p4fs::g_wifi_ip);
    }
}

static void wifi_init_sta(void)
{
    ESP_LOGI(TAG, "WiFi: initializing STA ...");

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        nullptr,
                                                        &s_wifi_event_hdl));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        nullptr,
                                                        &s_ip_event_hdl));

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (!sta_netif) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA netif");
        abort();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    if (p4fs::g_wifi_ssid[0] != '\0') {
        wifi_config_t wifi_config = {};
        size_t ssid_len = strnlen(p4fs::g_wifi_ssid, sizeof(wifi_config.sta.ssid) - 1);
        std::memcpy(wifi_config.sta.ssid, p4fs::g_wifi_ssid, ssid_len);
        wifi_config.sta.ssid[ssid_len] = '\0';
        size_t pass_len = strnlen(p4fs::g_wifi_pass, sizeof(wifi_config.sta.password) - 1);
        std::memcpy(wifi_config.sta.password, p4fs::g_wifi_pass, pass_len);
        wifi_config.sta.password[pass_len] = '\0';
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        ESP_LOGI(TAG, "WiFi STA started, SSID=%s", p4fs::g_wifi_ssid);
    } else {
        ESP_LOGW(TAG, "WiFi STA configured but SSID is empty; use serial console to set credentials, then save+reboot");
        ESP_ERROR_CHECK(esp_wifi_start());
    }
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

    /* --- Load persistent configuration -------------------------- */
    cfg_load();

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

    /* --- Serial configuration console -------------------------- */
    xTaskCreate(serial_console_task, "serial_console", 4096, nullptr, 2, nullptr);

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

    /* --- WiFi STA via C5 SDIO (esp_wifi_remote + esp_hosted) */
#if WIFI_HTTP_ENABLE
    wifi_init_sta();
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

    ESP_LOGI(TAG, "Streaming: device IP will be shown on WiFi connection; target IP=%s", p4fs::g_target_ip);
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
