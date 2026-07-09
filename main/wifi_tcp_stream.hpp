/*
 * wifi_tcp_stream: WiFi TCP streamer over C5 AT SDIO transport.
 *
 * Provides the same HTTP MJPEG + JSON endpoints as HttpServer, but
 * tunnels through the C5's AT TCP server instead of a direct socket.
 *
 * Architecture: a single FreeRTOS task time-slices between streaming
 * MJPEG frames and serving REST API requests.  Unlike a blocking
 * serve_stream() which locks out all other clients, this design polls
 * for new connections between every frame push.
 *
 * Endpoints (same as HttpServer)
 *   GET  /            – dashboard HTML
 *   GET  /stream      – multipart/x-mixed-replace MJPEG stream
 *   GET  /snapshot    – single JPEG snapshot (image/jpeg)
 *   GET  /api/info    – JSON: device info, FPS, enrollment count
 *   GET  /api/events  – JSON: latest recognition event (poll)
 *   POST /api/enroll  – trigger one-shot enrollment; returns 202
 *   POST /api/delete  – drop the last enrolled face
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

class WiFiTcpStreamer {
public:
    // ssid/ap_password: WiFi AP credentials for C5
    // tcp_port: TCP server port (default 8080)
    WiFiTcpStreamer(const char *ssid, const char *ap_password, int tcp_port = 8080);
    ~WiFiTcpStreamer();

    // Start WiFi AP + TCP server. Returns true on success.
    bool start(FaceAi *face_ai);

    // Stop the streamer.
    void stop();

    // Push a JPEG frame into the ring buffer (same interface as HttpServer).
    void push_jpeg(const uint8_t *data, size_t len);

    // Check if the streamer is running.
    bool is_running() const { return running_; }

private:
    void run_loop();

    // HTTP serving (similar to HttpServer)
    bool serve_client(int link_id, const char *req, int req_len);
    bool send_stream_headers(int link_id);
    bool send_stream_frame(int link_id);
    bool serve_snapshot(int link_id);
    bool serve_root(int link_id);
    bool serve_info(int link_id);
    bool serve_events(int link_id);
    bool serve_enroll(int link_id);
    bool serve_delete(int link_id);

    // TCP send helper over C5 AT
    bool tcp_send(int link_id, const void *data, size_t len);
    bool tcp_send_str(int link_id, const char *s);

    static const char *kBoundary;
    static const char *kIndexHtml;

    std::string ssid_;
    std::string ap_password_;
    int tcp_port_;
    FaceAi *face_ai_ = nullptr;
    bool running_ = false;
    TaskHandle_t task_ = nullptr;

    // Ring buffer of recent JPEGs (same pattern as HttpServer).
    // ring_cap_ controls how many frames are kept.  With 2 slots the stream
    // always shows the latest frame (lowest latency) but drops everything
    // under backpressure.  With 4+ slots the client can fall behind a few
    // frames without tearing, at the cost of ~100ms extra latency per slot
    // at 5 FPS.  4 is a good balance for WiFi streaming where transient
    // pauses are common.
    struct JpegFrame { std::vector<uint8_t> data; };
    JpegFrame ring_[8];
    int ring_cap_ = 4;
    int ring_head_ = 0;
    int ring_fill_ = 0;
    std::atomic<uint32_t> push_seq_{0};

    // Active stream client (time-sliced, non-blocking streaming).
    // stream_link_id_ >= 0 means a client has connected to /stream and
    // the main loop will push frames to that link between other work.
    int stream_link_id_ = -1;
    uint32_t stream_last_seq_ = 0;
};

} // namespace p4fs
