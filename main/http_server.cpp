/*
 * http_server implementation. See http_server.hpp for the endpoint list.
 *
 * Architecture: the main accept loop runs in one FreeRTOS task ("http_srv").
 * Short-lived requests (GET /, API calls, POST actions) are served inline.
 * Long-lived MJPEG stream clients ("/stream") are handed off to dedicated
 * "http_str" tasks so the accept loop stays free for API requests.
 */
#include "http_server.hpp"

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "face_ai.hpp"

namespace p4fs {

const char *HttpServer::kBoundary = "p4fsframe";

const char *HttpServer::kIndexHtml = R"HTML(
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
    // Display last enrollment result if present
    var le = document.getElementById('laster');
    if (j.last_enroll === -1) le.textContent = 'failed (no face?)';
    else if (j.last_enroll >= 1) le.textContent = 'ok, id=' + j.last_enroll;
    else if (j.last_enroll === 0) le.textContent = '-';
    else le.textContent = '-';  // -2 = no attempt yet
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
      // Poll /api/info until we get a result or timeout (3 s).
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

static const char *TAG = "http";

// -------- socket helpers -----------------------------------------------
static int set_nonblock(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

static int wall_write(int fd, const void *buf, int n) {
    const uint8_t *p = (const uint8_t *)buf;
    int sent = 0;
    while (sent < n) {
        int r = send(fd, p + sent, n - sent, MSG_NOSIGNAL);
        if (r <= 0) {
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            return -1;
        }
        sent += r;
    }
    return sent;
}

static bool write_str(int fd, const char *s) {
    return wall_write(fd, s, (int)strlen(s)) >= 0;
}

// Snapshot of the latest JPEG for a new client. Returns false if no frame yet.
bool HttpServer::serve_stream(int fd) {
    char hdr[256];
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=%s\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n", kBoundary);
    if (wall_write(fd, hdr, n) < 0) return false;

    uint32_t last_seen = 0xFFFFFFFFu;
    while (running_) {
        // Wait for a new frame.
        uint32_t cur = push_seq_.load();
        if (cur == last_seen) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_seen = cur;
        // Snapshot the most recent frame (head-1).
        int idx;
        if (ring_fill_ == 0) continue;
        if (ring_fill_ < ring_cap_) idx = (ring_head_ - 1 + ring_fill_) % ring_cap_;
        else                         idx = (ring_head_ - 1 + ring_cap_) % ring_cap_;
        const JpegFrame &f = ring_[idx];
        if (f.data.empty()) continue;

        int n2 = snprintf(hdr, sizeof(hdr),
            "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            kBoundary, (unsigned)f.data.size());
        if (wall_write(fd, hdr, n2) < 0) return false;
        if (wall_write(fd, f.data.data(), (int)f.data.size()) < 0) return false;
        if (wall_write(fd, "\r\n", 2) < 0) return false;
    }
    return true;
}

bool HttpServer::serve_root(int fd) {
    char hdr[128];
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n",
        (unsigned)strlen(kIndexHtml));
    return wall_write(fd, hdr, n) >= 0 &&
           wall_write(fd, kIndexHtml, (int)strlen(kIndexHtml)) >= 0;
}

bool HttpServer::serve_info(int fd) {
    extern uint32_t g_cam_fps_x100;       // defined in app_main.cpp
    extern std::atomic<int> g_last_enroll_id;  // defined in app_main.cpp
    int last_id = g_last_enroll_id.load();
    char body[384];
    int n = snprintf(body, sizeof(body),
        "{\"device\":\"p4_face_stream\",\"chip\":\"ESP32-P4\","
        "\"enrolled\":%d,\"frames\":%u,\"frames_with_face\":%u,"
        "\"fps\":%.2f,\"uptime_s\":%llu,\"heap_free\":%u,"
        "\"last_enroll\":%d}",
        face_ai_ ? (int)face_ai_->num_enrolled() : 0,
        face_ai_ ? (unsigned)face_ai_->frames_processed() : 0u,
        face_ai_ ? (unsigned)face_ai_->frames_with_face() : 0u,
        g_cam_fps_x100 / 100.0f,
        (unsigned long long)(esp_timer_get_time() / 1000000ULL),
        (unsigned)esp_get_free_heap_size(),
        last_id);
    char hdr[128];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n",
        (unsigned)n);
    return wall_write(fd, hdr, hn) >= 0 && wall_write(fd, body, n) >= 0;
}

bool HttpServer::serve_events(int fd) {
    const std::string ev = face_ai_ ? face_ai_->last_event_json() : std::string("{}");
    // Ensure we always emit valid JSON even if no face has been seen yet.
    const char *body = ev.empty() ? "{}" : ev.c_str();
    int n = (int)strlen(body);
    char hdr[128];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)n);
    return wall_write(fd, hdr, hn) >= 0 && wall_write(fd, body, n) >= 0;
}

bool HttpServer::serve_enroll(int fd, const std::string &body) {
    (void)body;
    // The actual enroll happens in app_main.cpp via a flag: the camera task
    // sees the flag on the next frame, runs enroll_largest, clears it. Here
    // we just acknowledge.
    extern std::atomic<int> g_enroll_pending;
    g_enroll_pending.store(1);
    const char *resp = "{\"status\":\"queued\"}";
    char hdr[256];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 202 Accepted\r\nContent-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n",
        (unsigned)strlen(resp));
    return wall_write(fd, hdr, hn) >= 0 && write_str(fd, resp);
}

bool HttpServer::serve_delete(int fd) {
    int rc = face_ai_ ? face_ai_->delete_last() : -1;
    char body[64];
    int n = snprintf(body, sizeof(body), "{\"ok\":%s}", rc == 0 ? "true" : "false");
    char hdr[128];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n", (unsigned)n);
    return wall_write(fd, hdr, hn) >= 0 && wall_write(fd, body, n) >= 0;
}

// SPR: true if this request should be served in a background task (long-lived).
bool HttpServer::spr_path(const char *method, const char *path) {
    return (!strcmp(method, "GET") && !strcmp(path, "/stream"));
}

bool HttpServer::serve_client(int fd, char *req, int req_len) {
    // Trim CRLF and isolate the request line.
    char *line_end = (char *)memchr(req, '\r', req_len);
    if (!line_end) line_end = (char *)memchr(req, '\n', req_len);
    if (!line_end) return false;
    *line_end = 0;

    char method[8] = {0}, path[64] = {0};
    if (sscanf(req, "%7s %63s", method, path) != 2) return false;
    ESP_LOGI(TAG, "%s %s", method, path);

    if (!strcmp(method, "GET") && !strcmp(path, "/"))         return serve_root(fd);
    if (!strcmp(method, "GET") && !strcmp(path, "/stream"))  return serve_stream(fd);
    if (!strcmp(method, "GET") && !strcmp(path, "/api/info"))    return serve_info(fd);
    if (!strcmp(method, "GET") && !strcmp(path, "/api/events"))  return serve_events(fd);
    if (!strcmp(method, "POST") && !strcmp(path, "/api/enroll")) return serve_enroll(fd, "");
    if (!strcmp(method, "POST") && !strcmp(path, "/api/delete")) return serve_delete(fd);

    const char *resp = "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
    return write_str(fd, resp);
}

// Per-stream-client task data. Freed by the task when the stream ends.
struct StreamTaskCtx {
    HttpServer *self;
    int fd;
};

static void stream_task(void *arg) {
    auto *ctx = static_cast<StreamTaskCtx *>(arg);
    ctx->self->serve_stream(ctx->fd);
    close(ctx->fd);
    delete ctx;
    vTaskDelete(nullptr);
}

void HttpServer::run_loop() {
    ring_cap_ = cfg_.jpeg_buffer_max;
    if (ring_cap_ < 1) ring_cap_ = 1;
    if (ring_cap_ > (int)(sizeof(ring_)/sizeof(ring_[0]))) ring_cap_ = sizeof(ring_)/sizeof(ring_[0]);

    char req[1024];
    while (running_) {
        struct sockaddr_in cli;
        socklen_t cl = sizeof(cli);
        int c = accept(listen_fd_, (struct sockaddr *)&cli, &cl);
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        ESP_LOGI(TAG, "client %s:%d fd=%d",
                 inet_ntoa(cli.sin_addr), ntohs(cli.sin_port), c);
        int n = recv(c, req, sizeof(req) - 1, 0);
        if (n > 0) {
            req[n] = 0;

            // Check whether this is a long-lived stream request BEFORE serving.
            // We peek at the request line so we can decide routing strategy.
            char req_copy[1024];
            memcpy(req_copy, req, n + 1);
            char *le = (char *)memchr(req_copy, '\r', n);
            if (!le) le = (char *)memchr(req_copy, '\n', n);
            if (le) *le = 0;
            char m[8] = {0}, p[64] = {0};
            if (sscanf(req_copy, "%7s %63s", m, p) == 2 && spr_path(m, p)) {
                // Long-lived stream: hand off to a dedicated task so the
                // main accept loop stays free for API requests.
                auto *ctx = new StreamTaskCtx{this, c};
                if (xTaskCreatePinnedToCore(stream_task, "http_str",
                                            4096, ctx, 2, nullptr, 1) != pdPASS) {
                    close(c);
                    delete ctx;
                }
            } else {
                serve_client(c, req, n);
                close(c);
            }
        } else {
            close(c);
        }
    }
    running_ = false;
    vTaskDelete(nullptr);
}

HttpServer::HttpServer(const HttpServerConfig &cfg, FaceAi *face_ai)
    : cfg_(cfg), face_ai_(face_ai) {}

HttpServer::~HttpServer() { stop(); }

bool HttpServer::start() {
    if (running_) return true;
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) { ESP_LOGE(TAG, "socket: %d", errno); return false; }
    int yes = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in a = {};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(cfg_.port);
    if (bind(listen_fd_, (struct sockaddr *)&a, sizeof(a)) < 0) {
        ESP_LOGE(TAG, "bind: %d", errno); close(listen_fd_); listen_fd_ = -1; return false;
    }
    if (listen(listen_fd_, cfg_.max_clients) < 0) {
        ESP_LOGE(TAG, "listen: %d", errno); close(listen_fd_); listen_fd_ = -1; return false;
    }
    set_nonblock(listen_fd_);
    running_ = true;
    if (xTaskCreatePinnedToCore(
            [](void *self) { static_cast<HttpServer *>(self)->run_loop(); },
            "http_srv", 6144, this, 4, &task_, 1) != pdPASS) {
        running_ = false;
        close(listen_fd_); listen_fd_ = -1;
        return false;
    }
    ESP_LOGI(TAG, "listening on :%d (max=%d)", cfg_.port, cfg_.max_clients);
    return true;
}

void HttpServer::stop() {
    if (!running_) return;
    running_ = false;
    if (listen_fd_ >= 0) { close(listen_fd_); listen_fd_ = -1; }
    // task self-deletes once it observes running_==false
}

void HttpServer::push_jpeg(const uint8_t *data, size_t len) {
    if (!running_ || !data || len == 0 || ring_cap_ == 0) return;
    JpegFrame &slot = ring_[ring_head_];
    slot.data.assign(data, data + len);
    ring_head_ = (ring_head_ + 1) % ring_cap_;
    if (ring_fill_ < ring_cap_) ring_fill_++;
    push_seq_.fetch_add(1);
}

} // namespace p4fs
