/*
 * stream_switch.h — Control whether the Ethernet HTTP server is enabled.
 *
 * When g_ethernet_stream_enabled is false (default), the Ethernet HTTP
 * server is disabled and only the WiFi TCP streamer serves clients.
 * Set to true to re-enable the Ethernet server at runtime.
 */
#pragma once

#include <atomic>

namespace p4fs {
// When false (default): Ethernet HTTP server is disabled.
// When true: Ethernet HTTP server is enabled (set via API or serial).
extern std::atomic<bool> g_ethernet_stream_enabled;
}
