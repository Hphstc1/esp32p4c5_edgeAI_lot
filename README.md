# esp32p4c5_edgeAI_lot

ESP32-P4 人脸识别 + MJPEG 实时推流，基于 **WT99P4C5-S1** 开发板，ESP-IDF **v5.5.4**。

## 功能

1. OV5647 摄像头 RGB565 采集（VGA，经 V4L2 缩放后供模型使用）
2. 人脸检测（ESPDet-pico 224×224）+ 人脸识别（MobileFaceNet，ESP-DL 量化）
3. 检测框/ID 标注叠加
4. 硬件 JPEG 编码 → HTTP MJPEG 推流（port 8080，~5 FPS）
5. JSON API：录入 / 删除 / 设备信息 / 识别事件
6. SPIFFS 人脸库（重启不丢失）
7. 仪表盘实时刷新 Enrolled 数、FPS、Last enroll、最新事件

## 硬件

- **芯片：** ESP32-P4，双核 RISC-V @ 360MHz，32 MB PSRAM
- **开发板：** WT99P4C5-S1
- **摄像头：** OV5647 MIPI-CSI
- **网络：** ESP32-C5 SDIO 协处理器提供 WiFi SoftAP
  - SSID：`P4-FaceStream`
  - 密码：`12345678`
  - 网关 IP：`192.168.4.1`

## 目录结构

```
esp32p4c5_edgeAI_lot/
├── CMakeLists.txt              # 顶层 CMake
├── sdkconfig                   # Kconfig 配置
├── sdkconfig.defaults          # 默认配置覆盖
├── partitions.csv              # Flash 分区表
├── .gitignore                  # 不上传的文件列表
├── components/                 # 自带的 BSP 组件
│   ├── wt99p4c5_s1_board/      # 板卡 BSP
│   └── bsp_extra/              # BSP 扩展
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml       # 组件依赖声明
    ├── app_main.cpp            # 主入口：初始化 + 主循环
    ├── app_video.c / .h        # V4L2 摄像头
    ├── face_ai.cpp / .hpp      # 人脸检测+识别
    ├── http_server.cpp / .hpp  # HTTP + MJPEG 推流（async /stream worker）
    └── jpeg_annotate.cpp / .hpp # 画框标注
```

## 编译与烧录

使用项目根目录外保留的脚本（已针对 IDF 5.5.4 配置好环境变量）：

```cmd
D:\esp32\tmp\build_p4fs_5.5.4.bat
D:\esp32\tmp\flash_p4fs_5.5.4.bat
```

> 脚本内已固定 `ESP_IDF_VERSION=5.5`，避免 `esp_wifi_remote` 的 Kconfig 版本选择错误导致 Unknown Slave Target。

## HTTP API（port 8080）

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/` | 仪表盘（Enrolled / FPS / Last enroll / 最新事件） |
| GET | `/stream` | MJPEG 视频流（带人脸框） |
| GET | `/api/info` | 设备信息 / FPS / 录入数 |
| GET | `/api/events` | 最新识别事件 |
| POST | `/api/enroll` | 录入人脸 |
| POST | `/api/delete` | 删除最近录入的人脸 |

## 启动与使用

1. 电脑连接 WiFi：`P4-FaceStream` / `12345678`
2. 浏览器打开 `http://192.168.4.1:8080/`
3. 仪表盘会每秒轮询 `/api/info` 和 `/api/events`，不再被 `/stream` 阻塞

示例启动日志：

```
I (2220) p4fs: === p4_face_stream starting ===
I (6340) p4fs: WiFi SoftAP started: SSID=P4-FaceStream, IP=192.168.4.1
I (6340) http: stream worker started
I (6340) http: stream worker started
I (6340) http: HTTP server started on port 8080
```

## 关键修复记录

- **模型：** 使用 `espdet_pico_224_224_face.espdl` 人脸检测模型（`CONFIG_DEFAULT_HUMAN_FACE_DETECT_MODEL=1`）。
- **C5 SDIO：** 通过 `ESP_IDF_VERSION=5.5` 让 `esp_wifi_remote` 正确选择 `SLAVE_IDF_TARGET_ESP32C5`。
- **HTTP 单线程阻塞：** `/stream` 改为 async handler + worker task pool，`/api/info` 与 `/api/events` 可并发响应。
- **max_clients：** 受 LWIP_MAX_SOCKETS 限制，设置为 7（httpd 内部占用 3 个）。
- **ETH 初始化：** 改为非致命失败，避免 PHY 偶发未响应时直接 abort。
