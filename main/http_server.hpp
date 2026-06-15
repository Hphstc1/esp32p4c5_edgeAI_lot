/*
 * http_server: tiny blocking-friendly HTTP/1.1 server.
 *
 * Endpoints
 *   GET  /            – dashboard HTML
 *   GET  /stream      – multipart/x-mixed-replace MJPEG
 *   GET  /api/info    – JSON: device info, FPS, enrollment count
 *   GET  /api/events  – JSON: latest recognition event (poll)
 *   POST /api/enroll  – trigger one-shot enrollment; returns new id or -1
 *   POST /api/delete  – drop the last enrolled face
 *
 * The MJPEG broadcast is pull-based: the camera task pushes JPEG frames
 * into a small ring buffer and the server task wakes the waiting
 * subscribers on every push.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace p4fs {

class FaceAi;  // fwd

struct HttpServerConfig {
    int  port            = 8080;
    int  max_clients     = 4;
    int  jpeg_quality    = 80;
    int  jpeg_buffer_max = 3;   // ring buffer depth
};

struct JpegFrame {
    std::vector<uint8_t> data;
};

class HttpServer {
public:
    HttpServer(const HttpServerConfig &cfg, FaceAi *face_ai);
    ~HttpServer();

    // Start the listener + worker task. Returns ESP_OK on success.
    bool start();

    // Stop and join the task. Safe to call once.
    void stop();

    // Push a freshly-encoded JPEG. The oldest frame in the ring is dropped
    // when the ring is full. Called from the camera task.
    void push_jpeg(const uint8_t *data, size_t len);

    // Connection handling helpers, return false if the connection died.
    bool serve_client(int fd, char *req, int req_len);
    bool spr_path(const char *method, const char *path);
    bool serve_stream(int fd);
    bool serve_root(int fd);
    bool serve_info(int fd);
    bool serve_events(int fd);
    bool serve_enroll(int fd, const std::string &body);
    bool serve_delete(int fd);

private:
    void run_loop();

    static const char *kBoundary;
    static const char *kIndexHtml;

    HttpServerConfig cfg_;
    FaceAi          *face_ai_ = nullptr;

    int   listen_fd_ = -1;
    bool  running_    = false;
    TaskHandle_t task_ = nullptr;

    // Ring buffer of recent JPEGs.
    JpegFrame   ring_[8];
    int         ring_cap_ = 0;
    int         ring_head_ = 0;   // next slot to overwrite
    int         ring_fill_ = 0;   // current number of valid frames
    std::atomic<uint32_t> push_seq_{0};   // monotonic, used to wake waiters
};

} // namespace p4fs
