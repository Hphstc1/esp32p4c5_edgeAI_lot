#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
p4c5 PC bridge
==============

Runs on the Windows PC. Responsibilities:
  1. Pull the MJPEG stream from the p4c5 HTTP server (/stream).
  2. Run YOLOv8 pest detection every N frames on the PC.
  3. Poll /api/events and /api/info from the p4c5.
  4. Serve a local dashboard HTML and proxy the video stream.
  5. Push face / pest / info messages to the dashboard via WebSocket.

Network topology for the demo:
  phone hotspot
    ├── PC  (runs this bridge)
    └── p4c5 (provides /stream, /api/events, /api/info)

The dashboard is then opened on the PC browser at http://localhost:8080/.
It still connects to the cloud MQTT broker for the existing sensor data.
"""

import argparse
import concurrent.futures
import json
import logging
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import requests
from flask import Flask, Response, send_from_directory
from flask_sock import Sock

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("p4c5_bridge")

# ---------------------------------------------------------------------------
# Flask / WebSocket setup
# ---------------------------------------------------------------------------
app = Flask(__name__, static_folder="static")
sock = Sock(app)

# Shared configuration (set in main)
stream_fps: float = 20.0

# ---------------------------------------------------------------------------
# Shared state (protected by locks where noted)
# ---------------------------------------------------------------------------
class BridgeState:
    def __init__(self, history_limit: int = 100):
        self.lock = threading.Lock()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_pest: Optional[dict] = None
        self.latest_face: Optional[dict] = None
        self.latest_info: Optional[dict] = None
        self.pest_history: list[dict] = []
        self.face_history: list[dict] = []
        self.frame_seq = 0
        self.ws_clients: set = set()
        self.history_limit = history_limit

state = BridgeState()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _probe_host(ip: str, port: int, timeout: float) -> Optional[str]:
    """Try to connect and HTTP-probe a single host. Returns IP if it looks like p4c5."""
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            r = requests.get(f"http://{ip}:{port}/api/info", timeout=1)
            if r.status_code == 200:
                data = r.json()
                if data and (data.get("device") == "p4_face_stream" or "chip" in data):
                    return ip
    except Exception:
        pass
    return None


def discover_p4c5_ip(port: int = 8080, timeout: float = 0.3, max_workers: int = 64) -> Optional[str]:
    """
    Scan common phone-hotspot subnets concurrently for the p4c5 HTTP server.
    Returns the first responsive p4c5 IP.
    """
    subnets = ["192.168.43", "192.168.137", "192.168.173", "172.20.10"]
    candidates = [f"{prefix}.{last}" for prefix in subnets for last in range(2, 255)]

    logger.info("Scanning %d candidate IPs for p4c5 (port %d)...", len(candidates), port)
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {executor.submit(_probe_host, ip, port, timeout): ip for ip in candidates}
        for future in concurrent.futures.as_completed(futures):
            result = future.result()
            if result:
                logger.info("Discovered p4c5 at %s:%d", result, port)
                # cancel remaining futures best-effort
                for f in futures:
                    f.cancel()
                return result
    return None


def _add_history(history_list: list, item: dict, limit: int) -> None:
    """Append an item to a history list and trim to limit."""
    history_list.append(item)
    while len(history_list) > limit:
        history_list.pop(0)


def broadcast(msg: dict) -> None:
    """Send a JSON message to every connected WebSocket client."""
    payload = json.dumps(msg, ensure_ascii=False, default=str)
    dead = set()
    with state.lock:
        clients = list(state.ws_clients)
    for ws in clients:
        try:
            ws.send(payload)
        except Exception as exc:
            logger.debug("WebSocket send failed: %s", exc)
            dead.add(ws)
    if dead:
        with state.lock:
            state.ws_clients -= dead


# ---------------------------------------------------------------------------
# MJPEG stream reader
# ---------------------------------------------------------------------------
def stream_reader_loop(base_url: str, reconnect_delay: float = 1.0) -> None:
    """
    Continuously read the MJPEG stream from p4c5 and update latest_jpeg.
    The p4c5 uses boundary 'p4fsframe'.
    """
    stream_url = f"{base_url}/stream"
    boundary = b"p4fsframe"
    while True:
        try:
            logger.info("Connecting to p4c5 stream: %s", stream_url)
            with requests.get(stream_url, stream=True, timeout=10) as resp:
                resp.raise_for_status()
                buffer = b""
                for chunk in resp.iter_content(chunk_size=4096):
                    if not chunk:
                        continue
                    buffer += chunk
                    # Find boundary markers and extract JPEG frames
                    while True:
                        start = buffer.find(b"--" + boundary)
                        if start == -1:
                            break
                        # find next boundary
                        end = buffer.find(b"--" + boundary, start + len(boundary) + 2)
                        if end == -1:
                            # need more data
                            # keep the last boundary onwards
                            buffer = buffer[start:]
                            break
                        part = buffer[start:end]
                        # JPEG starts at \xff\xd8 and ends at \xff\xd9
                        jpg_start = part.find(b"\xff\xd8")
                        jpg_end = part.rfind(b"\xff\xd9")
                        if jpg_start != -1 and jpg_end != -1 and jpg_end > jpg_start:
                            jpeg = part[jpg_start:jpg_end + 2]
                            with state.lock:
                                state.latest_jpeg = jpeg
                                state.frame_seq += 1
                        buffer = buffer[end:]
        except Exception as exc:
            logger.warning("Stream error: %s. Reconnecting in %.1fs...", exc, reconnect_delay)
            time.sleep(reconnect_delay)


# ---------------------------------------------------------------------------
# Pest detection inference
# ---------------------------------------------------------------------------
def inference_loop(model_path: str, interval: int, conf: float) -> None:
    """
    Run YOLOv8 pest detection every `interval` frames.
    """
    from ultralytics import YOLO

    logger.info("Loading YOLOv8 model: %s", model_path)
    model = YOLO(model_path)
    logger.info("Model loaded: %d classes", len(model.names))

    processed_seq = -1
    while True:
        time.sleep(0.02)
        with state.lock:
            jpeg = state.latest_jpeg
            seq = state.frame_seq
        if jpeg is None or seq == processed_seq:
            continue
        if seq % interval != 0:
            processed_seq = seq
            continue

        try:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                continue

            results = model.predict(source=img, conf=conf, save=False, verbose=False)
            boxes = []
            if results and results[0].boxes is not None:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())
                    conf_val = float(box.conf[0].cpu().numpy())
                    boxes.append({
                        "x1": int(x1), "y1": int(y1),
                        "x2": int(x2), "y2": int(y2),
                        "class": model.names[cls],
                        "conf": round(conf_val, 3),
                    })

            msg = {
                "type": "pest_detection",
                "frame_seq": seq,
                "boxes": boxes,
                "count": len(boxes),
                "timestamp": time.time(),
            }
            with state.lock:
                state.latest_pest = msg
                _add_history(state.pest_history, msg, state.history_limit)
            broadcast(msg)
            logger.debug("Pest detection: %d objects", len(boxes))
        except Exception as exc:
            logger.error("Inference error: %s", exc)
        finally:
            processed_seq = seq


# ---------------------------------------------------------------------------
# p4c5 API pollers
# ---------------------------------------------------------------------------
def face_poller_loop(base_url: str, interval: float = 0.5) -> None:
    """Poll /api/events from p4c5 and push face recognition events."""
    url = f"{base_url}/api/events"
    last_payload = None
    while True:
        try:
            resp = requests.get(url, timeout=5)
            resp.raise_for_status()
            data = resp.json()
            payload = json.dumps(data, sort_keys=True)
            if payload != last_payload and data:
                msg = {"type": "face_event", "data": data, "timestamp": time.time()}
                with state.lock:
                    state.latest_face = msg
                    _add_history(state.face_history, msg, state.history_limit)
                broadcast(msg)
                last_payload = payload
        except Exception as exc:
            logger.debug("Face poller error: %s", exc)
        time.sleep(interval)


def info_poller_loop(base_url: str, interval: float = 1.0) -> None:
    """Poll /api/info from p4c5 and push device info."""
    url = f"{base_url}/api/info"
    while True:
        try:
            resp = requests.get(url, timeout=5)
            resp.raise_for_status()
            data = resp.json()
            msg = {"type": "info", "data": data, "timestamp": time.time()}
            with state.lock:
                state.latest_info = msg
            broadcast(msg)
        except Exception as exc:
            logger.debug("Info poller error: %s", exc)
        time.sleep(interval)


# ---------------------------------------------------------------------------
# HTTP endpoints
# ---------------------------------------------------------------------------
@app.after_request
def add_cors_headers(response):
    """Allow the frontend to call these endpoints from any origin."""
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response


@app.route("/")
def index():
    """Serve the dashboard HTML (placeholder for frontend teammate)."""
    return send_from_directory(app.static_folder, "dashboard_p4c5.html")


@app.route("/stream")
def video_stream():
    """Proxy the latest JPEG frame as an MJPEG stream."""
    def generate():
        while True:
            with state.lock:
                jpeg = state.latest_jpeg
            if jpeg:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    + jpeg + b"\r\n"
                )
            time.sleep(1.0 / stream_fps)
    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-cache"},
    )


@app.route("/api/pest/latest")
def api_pest_latest():
    """Return the latest pest detection result."""
    with state.lock:
        data = state.latest_pest
    return jsonify_or_null(data)


@app.route("/api/face/latest")
def api_face_latest():
    """Return the latest face recognition event."""
    with state.lock:
        data = state.latest_face
    return jsonify_or_null(data)


@app.route("/api/info/latest")
def api_info_latest():
    """Return the latest p4c5 device info."""
    with state.lock:
        data = state.latest_info
    return jsonify_or_null(data)


@app.route("/api/pest/history")
def api_pest_history():
    """Return recent pest detection history (newest last)."""
    with state.lock:
        history = list(state.pest_history)
    return {"type": "pest_history", "count": len(history), "data": history}


@app.route("/api/face/history")
def api_face_history():
    """Return recent face event history (newest last)."""
    with state.lock:
        history = list(state.face_history)
    return {"type": "face_history", "count": len(history), "data": history}


def jsonify_or_null(data: Optional[dict]) -> tuple:
    """Return JSON data or a null placeholder with 503 if not available yet."""
    if data is None:
        return ({"type": "none", "data": None}, 503)
    return (data, 200)


# ---------------------------------------------------------------------------
# WebSocket endpoint
# ---------------------------------------------------------------------------
@sock.route("/ws")
def websocket(ws):
    with state.lock:
        state.ws_clients.add(ws)
    logger.info("WebSocket client connected. Total: %d", len(state.ws_clients))
    try:
        # Send current state immediately
        with state.lock:
            for msg in (state.latest_pest, state.latest_face, state.latest_info):
                if msg:
                    ws.send(json.dumps(msg, ensure_ascii=False, default=str))
        while True:
            data = ws.receive()
            if data is None:
                break
    finally:
        with state.lock:
            state.ws_clients.discard(ws)
        logger.info("WebSocket client disconnected. Total: %d", len(state.ws_clients))


# ---------------------------------------------------------------------------
# Simulation mode (for frontend development without real hardware)
# ---------------------------------------------------------------------------
SIM_PEST_CLASSES = [
    "rice leaf roller", "rice leaf caterpillar", "Potosiabre vitarsis",
    "oides decempunctata", "aphid", "whitefly", "thrips"
]


def _encode_sim_frame(frame: np.ndarray) -> bytes:
    """Encode an OpenCV BGR image to JPEG bytes."""
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    if not ok:
        return b""
    return buf.tobytes()


def simulation_stream_loop(fps: float = 5.0) -> None:
    """Generate fake MJPEG frames in place of a real p4c5 stream."""
    width, height = 640, 480
    t0 = time.time()
    logger.info("Simulation stream loop started (%dx%d @ %.1f FPS)", width, height, fps)

    while True:
        # Create a simple animated scene
        t = time.time() - t0
        frame = np.full((height, width, 3), (220, 230, 245), dtype=np.uint8)

        # Moving background rectangle
        x = int((width - 120) * (0.5 + 0.5 * np.sin(t * 0.7)))
        y = int((height - 120) * (0.5 + 0.5 * np.cos(t * 0.5)))
        cv2.rectangle(frame, (x, y), (x + 120, y + 120), (80, 140, 220), -1)

        # Rotating circle
        cx = int(width / 2 + 120 * np.cos(t))
        cy = int(height / 2 + 80 * np.sin(t * 1.3))
        cv2.circle(frame, (cx, cy), 40, (60, 180, 120), -1)

        # Timestamp text
        ts = time.strftime("%H:%M:%S", time.localtime())
        cv2.putText(frame, f"SIMULATION {ts}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (30, 30, 30), 2)

        jpeg = _encode_sim_frame(frame)
        if jpeg:
            with state.lock:
                state.latest_jpeg = jpeg
                state.frame_seq += 1

        time.sleep(1.0 / fps)


def simulation_inference_loop(interval: int, conf: float = 0.25) -> None:
    """Generate fake pest detection results."""
    import random
    processed_seq = -1
    rng = random.Random(42)

    while True:
        time.sleep(0.02)
        with state.lock:
            seq = state.frame_seq
        if seq == processed_seq or seq == 0:
            continue
        if seq % interval != 0:
            processed_seq = seq
            continue

        # Generate 0~3 random boxes
        num = rng.randint(0, 3)
        boxes = []
        for _ in range(num):
            x1 = rng.randint(50, 540)
            y1 = rng.randint(50, 380)
            x2 = min(x1 + rng.randint(60, 160), 630)
            y2 = min(y1 + rng.randint(60, 160), 470)
            cls = rng.choice(SIM_PEST_CLASSES)
            c = round(rng.uniform(conf, 1.0), 3)
            boxes.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "class": cls, "conf": c})

        msg = {
            "type": "pest_detection",
            "frame_seq": seq,
            "boxes": boxes,
            "count": len(boxes),
            "timestamp": time.time(),
        }
        with state.lock:
            state.latest_pest = msg
            _add_history(state.pest_history, msg, state.history_limit)
        broadcast(msg)
        logger.debug("Simulated pest detection: %d objects", len(boxes))
        processed_seq = seq


def simulation_face_loop(interval: float = 2.0) -> None:
    """Generate fake face recognition events periodically."""
    import random
    rng = random.Random(7)
    demo_names = ["Alice", "Bob", "Charlie", "David", "Eve"]
    last = 0

    while True:
        time.sleep(interval)
        if rng.random() < 0.6:
            data = {
                "id": rng.randint(1, 5),
                "name": rng.choice(demo_names),
                "score": round(rng.uniform(0.75, 0.98), 2),
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            }
            msg = {"type": "face_event", "data": data, "timestamp": time.time()}
            with state.lock:
                state.latest_face = msg
                _add_history(state.face_history, msg, state.history_limit)
            broadcast(msg)
            last = msg["timestamp"]


def simulation_info_loop(interval: float = 1.0) -> None:
    """Generate fake p4c5 device info."""
    import random
    rng = random.Random(99)
    while True:
        time.sleep(interval)
        data = {
            "device": "p4_face_stream",
            "chip": "ESP32-P4",
            "enrolled": rng.randint(0, 8),
            "frames": state.frame_seq,
            "frames_with_face": int(state.frame_seq * 0.3),
            "fps": round(4.8 + rng.uniform(-0.3, 0.3), 1),
            "uptime_s": int(time.time() % 10000),
            "heap_free": rng.randint(80000, 150000),
            "last_enroll": rng.randint(-2, 5),
        }
        msg = {"type": "info", "data": data, "timestamp": time.time()}
        with state.lock:
            state.latest_info = msg
        broadcast(msg)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="p4c5 PC bridge")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP server bind host")
    parser.add_argument("--port", type=int, default=8080, help="HTTP server port")
    parser.add_argument("--p4c5-ip", default=None, help="p4c5 IP address (auto-discover if omitted)")
    parser.add_argument("--p4c5-port", type=int, default=8080, help="p4c5 HTTP port")
    parser.add_argument("--model", default="../inference_pest/best.pt", help="YOLOv8 model path")
    parser.add_argument("--interval", type=int, default=5, help="Run pest detection every N frames")
    parser.add_argument("--conf", type=float, default=0.25, help="YOLOv8 confidence threshold")
    parser.add_argument("--stream-fps", type=float, default=5.0, help="Public MJPEG stream FPS (lower = less bandwidth)")
    parser.add_argument("--simulate", action="store_true", help="Run in simulation mode without real p4c5")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    args = parser.parse_args()

    if args.debug:
        logger.setLevel(logging.DEBUG)

    global stream_fps
    stream_fps = args.stream_fps
    logger.info("Public MJPEG stream FPS set to %.1f", stream_fps)

    if args.simulate:
        logger.info("=== SIMULATION MODE ===")
        logger.info("No real p4c5 or YOLO model required.")
        threading.Thread(target=simulation_stream_loop, daemon=True).start()
        threading.Thread(target=simulation_inference_loop, args=(args.interval, args.conf), daemon=True).start()
        threading.Thread(target=simulation_face_loop, daemon=True).start()
        threading.Thread(target=simulation_info_loop, daemon=True).start()
    else:
        # Resolve p4c5 IP
        p4c5_ip = args.p4c5_ip
        if not p4c5_ip:
            logger.info("No --p4c5-ip provided, attempting auto-discovery...")
            p4c5_ip = discover_p4c5_ip(args.p4c5_port)
            if not p4c5_ip:
                logger.error("Could not discover p4c5. Please provide --p4c5-ip manually.")
                sys.exit(1)

        base_url = f"http://{p4c5_ip}:{args.p4c5_port}"
        logger.info("Target p4c5 base URL: %s", base_url)

        # Validate model path
        model_path = Path(args.model).resolve()
        if not model_path.exists():
            logger.error("Model not found: %s", model_path)
            logger.error("Please run from p4c5_bridge directory or specify --model correctly.")
            sys.exit(1)

        # Start background threads
        threading.Thread(target=stream_reader_loop, args=(base_url,), daemon=True).start()
        threading.Thread(
            target=inference_loop,
            args=(str(model_path), args.interval, args.conf),
            daemon=True,
        ).start()
        threading.Thread(target=face_poller_loop, args=(base_url,), daemon=True).start()
        threading.Thread(target=info_poller_loop, args=(base_url,), daemon=True).start()

    logger.info("Starting HTTP server on http://%s:%d", args.host, args.port)
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
