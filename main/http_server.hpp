/*
 * http_server: esp_http_server-based dashboard + MJPEG stream.
 *
 * Endpoints
 *   GET  /            – dashboard HTML
 *   GET  /stream      – multipart/x-mixed-replace MJPEG
 *   GET  /api/info    – JSON: device info, FPS, enrollment count
 *   GET  /api/events  – JSON: latest recognition event (poll)
 *   POST /api/enroll  – trigger one-shot enrollment
 *   POST /api/delete  – drop the last enrolled face
 *
 * The MJPEG feed is shared: the camera task pushes JPEG frames into a single
 * protected buffer and every /stream client copies from it independently.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_http_server.h"

namespace p4fs {

class FaceAi;  // fwd

struct HttpServerConfig {
    int  port            = 8080;
    int  max_clients     = 4;
    int  jpeg_quality    = 80;   // kept for app_main compatibility; not used here
    int  jpeg_buffer_max = 3;    // kept for API compatibility; not used here
};

class HttpServer {
public:
    HttpServer(const HttpServerConfig &cfg, FaceAi *face_ai);
    ~HttpServer();

    // Start the esp_http_server. Returns true on success.
    bool start();

    // Stop the server. Safe to call multiple times.
    void stop();

    // Push a freshly-encoded JPEG. Called from the camera task.
    void push_jpeg(const uint8_t *data, size_t len);

    // URI handlers (invoked by the internal static C wrappers).
    esp_err_t root_handler(httpd_req_t *req);
    esp_err_t stream_handler(httpd_req_t *req);
    esp_err_t stream_async(httpd_req_t *req);
    esp_err_t info_handler(httpd_req_t *req);
    esp_err_t events_handler(httpd_req_t *req);
    esp_err_t enroll_handler(httpd_req_t *req);
    esp_err_t delete_handler(httpd_req_t *req);

private:
    HttpServerConfig cfg_;
    FaceAi          *face_ai_ = nullptr;

    httpd_handle_t   server_ = nullptr;
    SemaphoreHandle_t lock_  = nullptr;

    // Latest JPEG buffer, protected by lock_.
    uint8_t *jpeg_buf_ = nullptr;
    size_t   jpeg_len_ = 0;
    size_t   jpeg_cap_ = 0;

    std::atomic<uint32_t> seq_{0};   // increments on every push
    std::atomic<bool>     running_{false};

    // Async MJPEG stream worker pool.
    static void start_stream_workers();
    static void stream_worker_task(void *arg);
    static esp_err_t queue_stream_request(httpd_req_t *req, HttpServer *server);
};

} // namespace p4fs
