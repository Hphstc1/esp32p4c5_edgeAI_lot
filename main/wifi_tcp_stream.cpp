/*
 * wifi_tcp_stream implementation. See wifi_tcp_stream.hpp for the API.
 *
 * Architecture: a single FreeRTOS task time-slices between streaming MJPEG
 * frames and serving REST API requests.  Between every frame push, the task
 * polls for incoming HTTP requests on other TCP links so that API calls
 * (info, enroll, delete, events) are served promptly even when a /stream
 * client is active.
 *
 * The C5 runs the WiFi SoftAP + TCP server. This file sends AT commands over
 * SDIO to control WiFi and tunnel TCP data.
 */
#include "wifi_tcp_stream.hpp"

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "c5_bridge.h"
#include "face_ai.hpp"

namespace p4fs {

// ---- Stream switch (declared in stream_switch.h) --------------------------
std::atomic<bool> g_ethernet_stream_enabled{false};

// ---- Extern globals from app_main.cpp --------------------------------------
extern uint32_t g_cam_fps_x100;
extern std::atomic<int> g_enroll_pending;
extern std::atomic<int> g_last_enroll_id;

static const char *TAG = "wifi";

const char *WiFiTcpStreamer::kBoundary = "p4fsframe";

const char *WiFiTcpStreamer::kIndexHtml = R"HTML(
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
var failCount = 0;
async function poll() {
  var img = document.getElementById('mjpeg');
  if (img.complete) {
    if (img.naturalWidth > 0) {
      // /stream is connected
    } else {
      document.getElementById('status').textContent = 'Stream unavailable';
    }
  }
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
    failCount = 0;
    document.getElementById('status').textContent = '';
    const e = await fetch('/api/events'); const ej = await e.json();
    document.getElementById('ev').textContent = JSON.stringify(ej, null, 2);
  } catch (e) {
    failCount++;
    if (!apiOk) {
      if (failCount >= 3) {
        document.getElementById('status').textContent = 'Reconnecting...';
      } else {
        document.getElementById('status').textContent = 'Connecting...';
      }
    }
  }
  setTimeout(poll, 2000);
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
// DEBUG: disable API polling to test /stream connection
// poll();
</script>
</body></html>
)HTML";

// ======== Constructor / Destructor ==========================================

WiFiTcpStreamer::WiFiTcpStreamer(const char *ssid, const char *ap_password, int tcp_port)
    : ssid_(ssid), ap_password_(ap_password), tcp_port_(tcp_port) {}

WiFiTcpStreamer::~WiFiTcpStreamer() { stop(); }

// ======== Start / Stop =====================================================

bool WiFiTcpStreamer::start(FaceAi *face_ai) {
    if (running_) return true;
    face_ai_ = face_ai;

    // Test AT transport
    if (c5_at_sdio_init() != ESP_OK) {
        ESP_LOGE(TAG, "AT SDIO init failed");
        return false;
    }

    /* SDIO clock is already at full speed (20 MHz) — sdmmc_card_init()
     * raised it during probe with correct P4 input_delay_phase.
     * No separate c5_sdio_set_clock() call needed. */

    // Start WiFi SoftAP
    if (c5_at_sdio_wifi_ap(ssid_.c_str(), ap_password_.c_str()) != ESP_OK) {
        ESP_LOGE(TAG, "WiFi AP start failed");
        return false;
    }

    // Get the AP IP address
    char ip[32] = {0};
    if (c5_at_sdio_get_ip(ip, sizeof(ip)) != ESP_OK) {
        ESP_LOGW(TAG, "Could not get IP (AP may still be starting)");
        strcpy(ip, "192.168.4.1");
    }

    // Start TCP server on the C5
    if (c5_at_sdio_tcp_server_start(tcp_port_) != ESP_OK) {
        ESP_LOGE(TAG, "TCP server start failed");
        return false;
    }

    running_ = true;
    stream_link_id_ = -1;  /* no active stream client yet */
    if (xTaskCreatePinnedToCore(
            [](void *self) { static_cast<WiFiTcpStreamer *>(self)->run_loop(); },
            "wifi_srv", 8192, this, 4, &task_, 1) != pdPASS) {
        running_ = false;
        ESP_LOGE(TAG, "Failed to create wifi_srv task");
        return false;
    }

    ESP_LOGI(TAG, "WiFi AP: %s IP: %s TCP server on port %d",
             ssid_.c_str(), ip, tcp_port_);
    return true;
}

void WiFiTcpStreamer::stop() {
    if (!running_) return;
    running_ = false;
    // The task self-deletes once it observes running_ == false
}

// ======== Push JPEG ========================================================

void WiFiTcpStreamer::push_jpeg(const uint8_t *data, size_t len) {
    if (!running_ || !data || len == 0 || ring_cap_ == 0) return;
    JpegFrame &slot = ring_[ring_head_];
    slot.data.assign(data, data + len);
    ring_head_ = (ring_head_ + 1) % ring_cap_;
    if (ring_fill_ < ring_cap_) ring_fill_++;
    push_seq_.fetch_add(1);
}

// ======== TCP send helpers =================================================

bool WiFiTcpStreamer::tcp_send(int link_id, const void *data, size_t len) {
    return c5_at_sdio_tcp_send(link_id, (const uint8_t *)data, len, 5000) == ESP_OK;
}

bool WiFiTcpStreamer::tcp_send_str(int link_id, const char *s) {
    return tcp_send(link_id, s, strlen(s));
}

// ======== HTTP serving =====================================================

bool WiFiTcpStreamer::serve_client(int link_id, const char *req, int req_len) {
    // Trim CRLF and isolate the request line
    const char *line_end = (const char *)memchr(req, '\r', req_len);
    if (!line_end) line_end = (const char *)memchr(req, '\n', req_len);
    if (!line_end) return false;

    // Copy just the first line into a temporary buffer
    int line_len = (int)(line_end - req);
    if (line_len > 127) line_len = 127;
    char line[128];
    memcpy(line, req, line_len);
    line[line_len] = 0;

    char method[8] = {0}, path[64] = {0};
    if (sscanf(line, "%7s %63s", method, path) != 2) return false;

    ESP_LOGI(TAG, "%s %s (link=%d)", method, path, link_id);

    if (!strcmp(method, "GET") && !strcmp(path, "/"))         return serve_root(link_id);
    if (!strcmp(method, "GET") && !strcmp(path, "/snapshot"))  return serve_snapshot(link_id);
    if (!strcmp(method, "GET") && !strcmp(path, "/api/info"))    return serve_info(link_id);
    if (!strcmp(method, "GET") && !strcmp(path, "/api/events"))  return serve_events(link_id);
    if (!strcmp(method, "POST") && !strcmp(path, "/api/enroll")) return serve_enroll(link_id);
    if (!strcmp(method, "POST") && !strcmp(path, "/api/delete")) return serve_delete(link_id);

    const char *resp = "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
    return tcp_send_str(link_id, resp);
}

// ======== /stream endpoint (non-blocking) ==================================

bool WiFiTcpStreamer::send_stream_headers(int link_id) {
    char hdr[256];
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=%s\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n", kBoundary);
    return tcp_send(link_id, hdr, (size_t)n);
}

bool WiFiTcpStreamer::send_stream_frame(int link_id) {
    if (ring_fill_ == 0) return true;  /* no frames yet, keep waiting */
    int idx = (ring_head_ - 1 + ring_cap_) % ring_cap_;
    const JpegFrame &f = ring_[idx];
    if (f.data.empty()) return true;

    /* Send multipart boundary + frame.
     * If any send fails, the client disconnected — caller will clear
     * stream_link_id_. */
    char hdr[256];
    int n = snprintf(hdr, sizeof(hdr),
        "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
        kBoundary, (unsigned)f.data.size());
    return tcp_send(link_id, hdr, (size_t)n) &&
           tcp_send(link_id, f.data.data(), f.data.size()) &&
           tcp_send_str(link_id, "\r\n");
}

// ======== Other endpoint handlers ==========================================

bool WiFiTcpStreamer::serve_snapshot(int link_id) {
    // Grab the latest JPEG from the ring buffer.
    if (ring_fill_ == 0) {
        // No frame available yet — return 204 No Content.
        const char *resp = "HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n";
        return tcp_send_str(link_id, resp);
    }

    int idx = (ring_head_ - 1 + ring_cap_) % ring_cap_;
    const JpegFrame &f = ring_[idx];
    if (f.data.empty()) {
        const char *resp = "HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n";
        return tcp_send_str(link_id, resp);
    }

    char hdr[128];
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\n"
        "Content-Length: %u\r\nCache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n",
        (unsigned)f.data.size());
    if (!tcp_send(link_id, hdr, n)) return false;
    if (!tcp_send(link_id, f.data.data(), f.data.size())) return false;

    ESP_LOGI(TAG, "snapshot: sent %u bytes on link %d",
             (unsigned)f.data.size(), link_id);
    return true;
}

bool WiFiTcpStreamer::serve_root(int link_id) {
    ESP_LOGI(TAG, "serve_root: sending HTML (%u bytes)", (unsigned)strlen(kIndexHtml));
    char hdr[128];
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n",
        (unsigned)strlen(kIndexHtml));
    return tcp_send(link_id, hdr, n) &&
           tcp_send(link_id, kIndexHtml, strlen(kIndexHtml));
}

bool WiFiTcpStreamer::serve_info(int link_id) {
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
    return tcp_send(link_id, hdr, hn) && tcp_send(link_id, body, n);
}

bool WiFiTcpStreamer::serve_events(int link_id) {
    const std::string ev = face_ai_ ? face_ai_->last_event_json() : std::string("{}");
    const char *body = ev.empty() ? "{}" : ev.c_str();
    int n = (int)strlen(body);
    char hdr[128];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n",
        (unsigned)n);
    return tcp_send(link_id, hdr, hn) && tcp_send(link_id, body, n);
}

bool WiFiTcpStreamer::serve_enroll(int link_id) {
    g_enroll_pending.store(1);
    const char *resp = "{\"status\":\"queued\"}";
    char hdr[256];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 202 Accepted\r\nContent-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n",
        (unsigned)strlen(resp));
    return tcp_send(link_id, hdr, hn) && tcp_send_str(link_id, resp);
}

bool WiFiTcpStreamer::serve_delete(int link_id) {
    int rc = face_ai_ ? face_ai_->delete_last() : -1;
    char body[64];
    int n = snprintf(body, sizeof(body), "{\"ok\":%s}", rc == 0 ? "true" : "false");
    char hdr[128];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %u\r\nConnection: close\r\n\r\n",
        (unsigned)n);
    return tcp_send(link_id, hdr, hn) && tcp_send(link_id, body, n);
}

// ======== Run loop (time-sliced) ===========================================

void WiFiTcpStreamer::run_loop() {
    // Clamp ring_cap_ to fit the fixed array.
    if (ring_cap_ < 1) ring_cap_ = 1;
    int max_ring = (int)(sizeof(ring_) / sizeof(ring_[0]));
    if (ring_cap_ > max_ring) ring_cap_ = max_ring;

    uint8_t buf[2048];
    int poll_count = 0;
    int watchdog_fails = 0;      /* consecutive AT ping failures */
    const int kWdThreshold = 3;  /* trigger reinit after this many failures */
    while (running_) {
        /* ---- Phase 1: Poll for incoming data (any TCP link) ---- */
        int link_id = 0;
        int n = c5_at_sdio_tcp_recv_any(&link_id, buf, sizeof(buf) - 1, 80);

        if (n > 0) {
            buf[n] = 0;
            ESP_LOGI(TAG, "TCP recv %d bytes on link %d", n, link_id);
            poll_count = 0;
            watchdog_fails = 0;   /* any TCP exchange = C5 alive */

            /* Parse the HTTP request line */
            char req_copy[1024];
            memcpy(req_copy, buf, n + 1 < (int)sizeof(req_copy) ? n + 1 : (int)sizeof(req_copy));
            req_copy[sizeof(req_copy) - 1] = 0;
            char *le = (char *)memchr(req_copy, '\r', n);
            if (!le) le = (char *)memchr(req_copy, '\n', n);
            if (le) *le = 0;
            char method[8] = {0}, path[64] = {0};
            bool parsed = (sscanf(req_copy, "%7s %63s", method, path) == 2);

            if (parsed && !strcmp(method, "GET") && !strcmp(path, "/stream")) {
                /* ---- /stream request ---- */
                if (stream_link_id_ >= 0) {
                    ESP_LOGW(TAG, "stream already active on link %d, rejecting %d",
                             stream_link_id_, link_id);
                    tcp_send_str(link_id,
                        "HTTP/1.1 503 Service Unavailable\r\n"
                        "Connection: close\r\n\r\n");
                    c5_at_sdio_tcp_close(link_id);  /* free the slot immediately */
                } else {
                    if (send_stream_headers(link_id)) {
                        stream_link_id_ = link_id;
                        stream_last_seq_ = push_seq_.load();
                        ESP_LOGI(TAG, "stream started on link %d", link_id);
                    } else {
                        ESP_LOGW(TAG, "stream: failed to send headers on link %d", link_id);
                        c5_at_sdio_tcp_close(link_id);
                    }
                }
            } else if (parsed) {
                /* ---- REST API request ---- */
                serve_client(link_id, (const char *)buf, n);
                /* Explicit CIPCLOSE so the link is freed immediately.
                 * Without this, C5's CIPSERVERMAXCONN limit (default 1)
                 * blocks the /stream img connection from being accepted. */
                c5_at_sdio_tcp_close(link_id);
            } else {
                ESP_LOGW(TAG, "unparseable request on link %d: [%.60s]", link_id, buf);
                c5_at_sdio_tcp_close(link_id);
            }
        } else {
            /* ---- No incoming data: heartbeat + watchdog ---- */
            if (++poll_count % 25 == 0) {
                ESP_LOGI(TAG, "heartbeat: polled=%d seq=%lu stream_link=%d fails=%d/%d",
                         poll_count, (unsigned long)push_seq_.load(),
                         stream_link_id_, watchdog_fails, kWdThreshold);

                /* AT ping to verify C5 is responsive */
                if (c5_at_sdio_ping() == ESP_OK) {
                    watchdog_fails = 0;  /* C5 is healthy */
                } else {
                    watchdog_fails++;
                    ESP_LOGW(TAG, "C5 AT ping FAILED (%d/%d)",
                             watchdog_fails, kWdThreshold);

                    if (watchdog_fails >= kWdThreshold) {
                        /* C5 is unresponsive — full recovery */
                        ESP_LOGE(TAG, "C5 watchdog triggered! Re-initializing SDIO...");
                        c5_sdio_dump_rx_state();
                        stream_link_id_ = -1;  /* reset stream state */

                        esp_err_t re_err = c5_sdio_reinit();
                        if (re_err != ESP_OK) {
                            ESP_LOGE(TAG, "C5 reinit failed: 0x%x — will retry", re_err);
                            watchdog_fails = kWdThreshold - 1;
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            continue;
                        }

                        /* Re-establish AT transport and WiFi */
                        ESP_LOGI(TAG, "C5 reinit OK — re-establishing WiFi AP...");
                        if (c5_at_sdio_init() != ESP_OK ||
                            c5_at_sdio_wifi_ap(ssid_.c_str(), ap_password_.c_str()) != ESP_OK ||
                            c5_at_sdio_tcp_server_start(tcp_port_) != ESP_OK) {
                            ESP_LOGE(TAG, "C5 service reinit failed");
                            watchdog_fails = kWdThreshold - 1;
                            continue;
                        }

                        watchdog_fails = 0;
                        ESP_LOGI(TAG, "C5 watchdog recovery complete — service restored");
                    }
                }
            }
        }

        /* ---- Phase 2: Push latest frame to active stream (non-blocking) ---- */
        if (stream_link_id_ >= 0) {
            uint32_t cur = push_seq_.load();
            if (cur != stream_last_seq_) {
                stream_last_seq_ = cur;
                if (!send_stream_frame(stream_link_id_)) {
                    ESP_LOGI(TAG, "stream: client on link %d disconnected", stream_link_id_);
                    stream_link_id_ = -1;
                }
            }
        }

        /* Yield to avoid starving the camera/encoder task.
         * Longer delay when no stream is active to reduce CPU usage. */
        vTaskDelay(pdMS_TO_TICKS(stream_link_id_ >= 0 ? 5 : 20));
    }
    running_ = false;
    stream_link_id_ = -1;
    vTaskDelete(nullptr);
}

} // namespace p4fs
