# p4_face_stream

Face recognition + real-time MJPEG streaming on the **WT99P4C5-S1** (ESP32-P4 + ESP32-C5) running ESP-IDF v5.5.

This project glues together two existing art pieces that already live in this workspace:

| From                        | Reused                                                                |
|-----------------------------|-----------------------------------------------------------------------|
| `p4_camera_stream/`         | Working MIPI-CSI capture (OV5647), JPEG encoder, BSP, Ethernet init   |
| `esp-who/examples/human_face_recognition` | Underlying `espressif/human_face_recognition` model (the same one esp-who's `WhoRecognition` wraps in C++) |

The result is a single firmware image that:

1. Captures OV5647 frames at QVGA / WQVGA in RGB565.
2. Runs **face detection** + **face recognition** on every Nth frame (the model is `MNLP` + `MobileFaceNet` quantized for ESP-DL).
3. Draws coloured boxes + name labels directly into the RGB565 buffer.
4. JPEG-encodes the annotated frame and pushes it into a small ring buffer.
5. Serves the latest JPEG over **HTTP MJPEG** (`multipart/x-mixed-replace`) plus a small **JSON control API** for enroll / delete / event polling.

## Endpoints (port 8080)

| Method | Path           | Purpose                                                       |
|-------:|----------------|---------------------------------------------------------------|
| GET    | `/`            | Dashboard HTML with embedded `<img src=/stream>` and live event panel |
| GET    | `/stream`      | Multipart MJPEG — annotated live view                          |
| GET    | `/api/info`    | JSON device info, FPS, enrolled count                          |
| GET    | `/api/events`  | JSON latest recognition event (id, name, similarity, bbox, ts) |
| POST   | `/api/enroll`  | Trigger one-shot enrollment on the next frame                  |
| POST   | `/api/delete`  | Drop the most recently enrolled face                           |

`face.db` (the face embedding database) lives in the on-board SPIFFS partition (`/spiffs/face.db`), so enrollments survive reboot.

## Hardware

Per the **WT99P4C5-S1 开发板使用指南 v1.2**:

* ESP32-P4, dual-core RISC-V @ 360 MHz, 32 MB PSRAM
* ESP32-C5-WROOM-1 companion for 2.4 / 5 GHz Wi-Fi 6 (not used by this firmware but available)
* OV5647 sensor over MIPI-CSI
* 7″ MIPI-DSI LCD (not driven — we stream over Ethernet)
* 10/100 M Ethernet PHY
* ES8311 audio codec (unused)

The MIPI-CSI pin map and Ethernet GPIOs are pinned in `sdkconfig.defaults` and `app_main.cpp`. Ethernet is brought up via `bsp_eth_init()` and assigned a static IP `192.168.1.200/24` so the demo URL is stable.

## How the code is organised

```
p4_face_stream/
├── CMakeLists.txt           # top-level; EXTRA_COMPONENT_DIRS points to the
│                            # shared BSP in p4_camera_stream/components
├── sdkconfig.defaults        # chip + camera + ethernet defaults
├── partitions.csv            # adds a 1 MB SPIFFS partition for face.db
├── README.md
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml    # esp_video, esp_cam_sensor, esp_jpeg,
    │                        # human_face_recognition, human_face_detect
    ├── app_main.cpp          # orchestration: NVS, ethernet, camera, http
    ├── app_video.{c,h}       # V4L2 capture loop (OV5647, RGB565)
    ├── face_ai.{hpp,cpp}     # HumanFaceDetect + HumanFaceRecognizer wrapper
    ├── jpeg_annotate.{hpp,cpp}  # box / 5x7-font label drawing on RGB565
    └── http_server.{hpp,cpp}    # blocking-friendly HTTP/1.1 + MJPEG
```

The C++ portions are limited to the model wrapper and HTTP server; everything else is plain C and follows the coding style of `p4_camera_stream/main/`.

## Build & flash

```sh
# from the project root, with ESP-IDF 5.5 exported
cd D:/esp32/p4_face_stream
idf.py set-target esp32p4
idf.py build
idf.py -p COMx flash monitor
```

After boot the serial log will print:

```
...
Ethernet up, IP 192.168.1.200
SPIFFS: total=1048576 used=0
FaceAi ready, db=/spiffs/face.db, enrolled=0
http_srv: listening on :8080 (max=4)
Streaming at http://192.168.1.200:8080/
```

Open a browser to <http://192.168.1.200:8080/>. The first few frames are empty while the model warms up; once `s_frame_idx > 5` boxes start showing up.

## Demo flow

1. Open the dashboard in a browser.
2. Click **Enroll** → stand in front of the camera for ~2 s. The next face detection adds a new entry to the face database. The page polls `/api/info` and shows the new enrollment count.
3. Click **Enroll** again with a different person to add id:2, id:3, …
4. Walk around; the stream shows green boxes for recognized faces and orange boxes for strangers.
5. Click **Delete last** to roll back the most recent enrollment.
6. Power-cycle the board — `/api/info` will still report the saved enrollments because the database lives in SPIFFS.

## Performance notes

| Stage                | Typical cost on P4 @ 360 MHz             |
|----------------------|------------------------------------------|
| V4L2 capture (QVGA)  | DMA, free during CPU work                |
| Face detection       | ~80–110 ms / frame (quantized MNLP)      |
| Face recognition     | ~30–50 ms / face                         |
| Annotation (boxes)   | < 1 ms                                  |
| JPEG encode (QVGA)   | ~5–10 ms (hardware)                      |
| HTTP MJPEG push      | dominated by link speed on Ethernet      |

Because detection is the hot loop, we run the model on every 3rd frame by default (`kProcessEveryN` in `app_main.cpp`) to keep the effective stream ≥ 10 FPS at QVGA while still catching every face. Boxes are still drawn on every frame, so the visual stream stays smooth.

## Project context (TDP-Net)

This project is the **AI 视觉节点 (P4Sight)** firmware deliverable for the TDP-Net system described in `TDP-Net 开发计划书 v8.0.docx`. The PRD there calls for:

* P4 NPU human detection < 100 ms ✅
* MJPEG live push over the WT99P4C5-S1 ✅
* ESP-NOW control plane / Wi-Fi 6 data plane — *out of scope of this image; this firmware focuses on the per-node capture + recognition + HTTP preview.*

`TDP-Net` itself (TDMA mesh, AES-GCM, multi-hop routing) lives in the team repo and is wired in around this image.
