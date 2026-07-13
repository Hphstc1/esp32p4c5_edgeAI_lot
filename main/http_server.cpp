/*
 * http_server implementation based on esp_http_server.
 *
 * A single server instance serves both the WiFi SoftAP and Ethernet interfaces
 * (it binds to INADDR_ANY via lwIP).  The camera task pushes freshly encoded
 * JPEGs into a shared protected buffer; every /stream client copies from it
 * and sends the frames as multipart/x-mixed-replace.
 */
#include "http_server.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "face_ai.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

namespace p4fs {

namespace {

/* Async stream worker pool: the /stream URI handler cannot block the single
 * httpd thread, otherwise /api/info and /api/events stop being served.
 * We hand the request off to a worker task that runs the (long) MJPEG loop. */
struct AsyncStreamReq {
    httpd_req_t *req;
    HttpServer  *server;
};

constexpr int kStreamWorkers = 2;
QueueHandle_t   s_stream_queue = nullptr;
SemaphoreHandle_t s_stream_worker_ready = nullptr;
TaskHandle_t    s_stream_worker_handles[kStreamWorkers] = { nullptr };

} // namespace

static const char *TAG = "http";
static const char *kBoundary = "p4fsframe";

static const char *kIndexHtml = R"HTML(
<!doctype html>
<html><head><meta charset="utf-8"><title>P4 Face Stream</title>
<style>
body{background:#111;color:#eee;font-family:monospace;margin:0;padding:0}
header{padding:8px 12px;background:#222}
main{display:grid;grid-template-columns:2fr 1fr;gap:12px;padding:12px}
#view{background:#000;display:flex;justify-content:center;align-items:center}
#view img{max-width:100%;max-height:75vh}
#panel{background:#1a1a1a;padding:8px;border:1px solid #333;max-height:75vh;overflow:auto}
.row{margin:4px 0}
.btn{background:#285;color:#fff;border:0;padding:6px 10px;cursor:pointer;margin-right:4px}
.btn:hover{background:#3a6}
.btn:disabled{background:#444;cursor:not-allowed}
pre{white-space:pre-wrap;font-size:12px}
#status{color:#fc0;font-size:12px;min-height:16px}
</style></head>
<body><header><h2>P4 Face Stream</h2></header>
<main>
  <div id="view"><img id="mjpeg" src="/stream" alt="stream"></div>
  <div id="panel">
    <div class="row"><button class="btn" onclick="act('enroll')">Enroll</button>
       <button class="btn" onclick="act('delete')">Delete last</button></div>
    <div class="row">Enrolled: <span id="enr">?</span> | FPS: <span id="fps">?</span></div>
    <div class="row">Last enroll: <span id="laster">-</span></div>
    <div class="row" id="status"></div>
    <div class="row">Latest event:</div>
    <pre id="ev">{}</pre>
  </div>
</main>
<script>
var apiOk = false;
async function poll() {
  try {
    const r = await fetch('/api/info'); const j = await r.json();
    document.getElementById('enr').textContent = j.enrolled;
    document.getElementById('fps').textContent = j.fps.toFixed(1);
    var le = document.getElementById('laster');
    if (j.last_enroll === -1) le.textContent = 'failed (no face?)';
    else if (j.last_enroll >= 1) le.textContent = 'ok, id=' + j.last_enroll;
    else if (j.last_enroll === 0) le.textContent = '-';
    else le.textContent = '-';
    apiOk = true;
    document.getElementById('status').textContent = '';
    const e = await fetch('/api/events'); const ej = await e.json();
    document.getElementById('ev').textContent = JSON.stringify(ej, null, 2);
  } catch (e) {
    if (!apiOk) document.getElementById('status').textContent = 'Connecting...';
  }
  setTimeout(poll, 500);
}
async function act(op) {
  var btn = event.target;
  btn.disabled = true;
  var st = document.getElementById('status');
  st.textContent = op === 'enroll' ? 'Enrolling...' : 'Deleting...';
  try {
    var r = await fetch('/api/' + op, {method:'POST'});
    var j = await r.json();
    if (op === 'enroll') {
      st.textContent = 'Enroll: queued, waiting...';
      for (var i = 0; i < 10; i++) {
        await new Promise(function(resolve) { setTimeout(resolve, 300); });
        var ri = await fetch('/api/info'); var ij = await ri.json();
        if (ij.last_enroll !== -2) {
          if (ij.last_enroll >= 1) st.textContent = 'Enrolled! id=' + ij.last_enroll;
          else if (ij.last_enroll === -1) st.textContent = 'Enroll failed: no face detected';
          else st.textContent = 'Enroll: unexpected result ' + ij.last_enroll;
          setTimeout(function(){ st.textContent = ''; }, 3000);
          break;
        }
      }
      if (i === 10) { st.textContent = 'Enroll: timeout (no result)'; }
    } else {
      st.textContent = j.ok ? 'Deleted' : 'Delete failed';
      setTimeout(function(){ st.textContent = ''; }, 2000);
    }
  } catch (e) {
    st.textContent = 'Error: ' + e.message;
  }
  btn.disabled = false;
}
poll();
</script>
</body></html>
)HTML";

/* Shared state exported by app_main.cpp (same namespace). */
extern uint32_t       g_cam_fps_x100;
extern std::atomic<int> g_last_enroll_id;
extern std::atomic<int> g_enroll_pending;
extern char           g_target_ip[];

// ---- Static C handlers that forward to the instance -------------------
static esp_err_t root_hdl(httpd_req_t *req) {
    return static_cast<HttpServer *>(req->user_ctx)->root_handler(req);
}
static esp_err_t stream_hdl(httpd_req_t *req) {
    return static_cast<HttpServer *>(req->user_ctx)->stream_handler(req);
}
static esp_err_t info_hdl(httpd_req_t *req) {
    return static_cast<HttpServer *>(req->user_ctx)->info_handler(req);
}
static esp_err_t events_hdl(httpd_req_t *req) {
    return static_cast<HttpServer *>(req->user_ctx)->events_handler(req);
}
static esp_err_t enroll_hdl(httpd_req_t *req) {
    return static_cast<HttpServer *>(req->user_ctx)->enroll_handler(req);
}
static esp_err_t delete_hdl(httpd_req_t *req) {
    return static_cast<HttpServer *>(req->user_ctx)->delete_handler(req);
}

HttpServer::HttpServer(const HttpServerConfig &cfg, FaceAi *face_ai)
    : cfg_(cfg), face_ai_(face_ai) {
    lock_ = xSemaphoreCreateMutex();
}

HttpServer::~HttpServer() {
    stop();
    if (lock_) {
        vSemaphoreDelete(lock_);
        lock_ = nullptr;
    }
    free(jpeg_buf_);
}

bool HttpServer::start() {
    if (running_.load() || server_) return true;

    start_stream_workers();

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port        = cfg_.port;
    config.max_open_sockets   = cfg_.max_clients;
    config.lru_purge_enable   = true;
    config.stack_size         = 20480;

    if (httpd_start(&server_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return false;
    }

    httpd_uri_t root_uri   = { "/",       HTTP_GET,  root_hdl,   this };
    httpd_uri_t stream_uri = { "/stream", HTTP_GET,  stream_hdl, this };
    httpd_uri_t info_uri   = { "/api/info",   HTTP_GET,  info_hdl,   this };
    httpd_uri_t events_uri = { "/api/events", HTTP_GET,  events_hdl, this };
    httpd_uri_t enroll_uri = { "/api/enroll", HTTP_POST, enroll_hdl, this };
    httpd_uri_t delete_uri = { "/api/delete", HTTP_POST, delete_hdl, this };

    httpd_register_uri_handler(server_, &root_uri);
    httpd_register_uri_handler(server_, &stream_uri);
    httpd_register_uri_handler(server_, &info_uri);
    httpd_register_uri_handler(server_, &events_uri);
    httpd_register_uri_handler(server_, &enroll_uri);
    httpd_register_uri_handler(server_, &delete_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", cfg_.port);
    running_.store(true);
    return true;
}

void HttpServer::stop() {
    if (server_) {
        httpd_stop(server_);
        server_ = nullptr;
    }
    running_.store(false);
}

esp_err_t HttpServer::root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t HttpServer::stream_handler(httpd_req_t *req) {
    /* /stream cannot block the single httpd thread; hand it to a worker task. */
    return queue_stream_request(req, this);
}

esp_err_t HttpServer::stream_async(httpd_req_t *req) {
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=p4fsframe");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    char hdr[256];
    std::vector<uint8_t> local;
    uint32_t last_seq = 0;

    while (running_.load()) {
        uint32_t cur_seq = seq_.load();
        if (cur_seq == last_seq) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        last_seq = cur_seq;

        if (xSemaphoreTake(lock_, pdMS_TO_TICKS(100)) != pdTRUE) continue;
        if (jpeg_len_) {
            local.assign(jpeg_buf_, jpeg_buf_ + jpeg_len_);
        } else {
            local.clear();
        }
        xSemaphoreGive(lock_);

        if (local.empty()) continue;

        int n = snprintf(hdr, sizeof(hdr),
            "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            kBoundary, (unsigned)local.size());
        if (httpd_resp_send_chunk(req, hdr, n) != ESP_OK) break;
        if (httpd_resp_send_chunk(req, (const char *)local.data(), local.size()) != ESP_OK) break;
        if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) break;

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    httpd_resp_sendstr_chunk(req, nullptr);
    return ESP_OK;
}

esp_err_t HttpServer::info_handler(httpd_req_t *req) {
    int last_id = g_last_enroll_id.load();
    char body[512];
    int n = snprintf(body, sizeof(body),
        "{\"device\":\"p4_face_stream\",\"chip\":\"ESP32-P4\","
        "\"enrolled\":%d,\"frames\":%u,\"frames_with_face\":%u,"
        "\"fps\":%.2f,\"uptime_s\":%llu,\"heap_free\":%u,"
        "\"target_ip\":\"%s\",\"last_enroll\":%d}",
        face_ai_ ? (int)face_ai_->num_enrolled() : 0,
        face_ai_ ? (unsigned)face_ai_->frames_processed() : 0u,
        face_ai_ ? (unsigned)face_ai_->frames_with_face() : 0u,
        g_cam_fps_x100 / 100.0f,
        (unsigned long long)(esp_timer_get_time() / 1000000ULL),
        (unsigned)esp_get_free_heap_size(),
        g_target_ip,
        last_id);
    if (n < 0 || (size_t)n >= sizeof(body)) {
        ESP_LOGW(TAG, "info response truncated (need %d, have %u)", n, (unsigned)sizeof(body));
        n = (int)sizeof(body) - 1;
        body[n] = '\0';
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, body, n);
    return ESP_OK;
}

esp_err_t HttpServer::events_handler(httpd_req_t *req) {
    const std::string ev = face_ai_ ? face_ai_->last_event_json() : std::string("{}");
    const char *body = ev.empty() ? "{}" : ev.c_str();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t HttpServer::enroll_handler(httpd_req_t *req) {
    (void)req;
    g_enroll_pending.store(1);
    const char *resp = "{\"status\":\"queued\"}";
    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t HttpServer::delete_handler(httpd_req_t *req) {
    int rc = face_ai_ ? face_ai_->delete_last() : -1;
    char body[64];
    int n = snprintf(body, sizeof(body), "{\"ok\":%s}", rc == 0 ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, body, n);
    return ESP_OK;
}

/* ---- Async MJPEG stream worker pool -------------------------------- */

void HttpServer::stream_worker_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "stream worker started");
    while (true) {
        xSemaphoreGive(s_stream_worker_ready);
        AsyncStreamReq async_req;
        if (xQueueReceive(s_stream_queue, &async_req, portMAX_DELAY)) {
            ESP_LOGI(TAG, "stream worker serving client");
            async_req.server->stream_async(async_req.req);
            if (httpd_req_async_handler_complete(async_req.req) != ESP_OK) {
                ESP_LOGE(TAG, "stream async complete failed");
            }
            ESP_LOGI(TAG, "stream worker client done");
        }
    }
}

void HttpServer::start_stream_workers() {
    if (s_stream_queue) return;   // already started

    s_stream_worker_ready = xSemaphoreCreateCounting(kStreamWorkers, 0);
    s_stream_queue = xQueueCreate(kStreamWorkers, sizeof(AsyncStreamReq));
    if (!s_stream_worker_ready || !s_stream_queue) {
        ESP_LOGE(TAG, "failed to create stream worker queue/semaphore");
        return;
    }

    for (int i = 0; i < kStreamWorkers; i++) {
        if (!xTaskCreate(stream_worker_task, "stream_worker", 16384, nullptr, 5,
                         &s_stream_worker_handles[i])) {
            ESP_LOGE(TAG, "failed to create stream worker %d", i);
        }
    }
}

esp_err_t HttpServer::queue_stream_request(httpd_req_t *req, HttpServer *server) {
    httpd_req_t *copy = nullptr;
    if (httpd_req_async_handler_begin(req, &copy) != ESP_OK) {
        ESP_LOGE(TAG, "stream async begin failed");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "stream async begin failed");
        return ESP_OK;
    }

    AsyncStreamReq async_req = { copy, server };

    if (xSemaphoreTake(s_stream_worker_ready, 0) != pdTRUE) {
        ESP_LOGW(TAG, "no stream workers available");
        httpd_req_async_handler_complete(copy);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "no stream workers available");
        return ESP_OK;
    }

    if (xQueueSend(s_stream_queue, &async_req, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "stream worker queue full");
        httpd_req_async_handler_complete(copy);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "stream worker queue full");
        return ESP_OK;
    }

    return ESP_OK;
}

void HttpServer::push_jpeg(const uint8_t *data, size_t len) {
    if (!data || len == 0 || !lock_) return;

    if (xSemaphoreTake(lock_, pdMS_TO_TICKS(10)) != pdTRUE) return;

    if (len > jpeg_cap_) {
        free(jpeg_buf_);
        jpeg_cap_ = (len + 1023) & ~((size_t)1023);
        jpeg_buf_ = (uint8_t *)heap_caps_malloc(jpeg_cap_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!jpeg_buf_) {
            jpeg_buf_ = (uint8_t *)malloc(jpeg_cap_);
            if (!jpeg_buf_) {
                jpeg_cap_ = 0;
                jpeg_len_ = 0;
                xSemaphoreGive(lock_);
                ESP_LOGE(TAG, "jpeg buf alloc failed for %u bytes", (unsigned)len);
                return;
            }
            ESP_LOGW(TAG, "jpeg buf fallback to internal RAM (%u bytes)", (unsigned)jpeg_cap_);
        }
    }

    std::memcpy(jpeg_buf_, data, len);
    jpeg_len_ = len;
    xSemaphoreGive(lock_);
    seq_.fetch_add(1);
}

} // namespace p4fs
