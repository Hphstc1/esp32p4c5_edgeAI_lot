# esp32p4c5_edgeAI_lot

ESP32-P4 人脸识别 + MJPEG 实时推流，基于 **WT99P4C5-S1** 开发板，ESP-IDF v5.4.4。

> **架构 v2.0**: 本固件仅部署人脸识别模型。害虫检测已迁移至 PC 端 [PestYOLO](../PestYOLO/)，通过 MQTT 桥接视频帧流。

## 功能

1. OV5647 摄像头 1280×960 RGB565 采集
2. 人脸检测 + 人脸识别（MSR + MNP + MobileFaceNet，ESP-DL 量化）
3. 检测框/姓名标注叠加 (5×7 像素字体, 零依赖)
4. 硬件 JPEG 编码 → HTTP MJPEG 推流（port 8080, ~9 FPS）
5. JSON API：录入 / 删除 / 设备信息 / 识别事件
6. SPIFFS 人脸库（重启不丢失）

## 硬件

- **芯片：** ESP32-P4，双核 RISC-V @ 400MHz，32 MB PSRAM
- **开发板：** WT99P4C5-S1
- **摄像头：** OV5647 MIPI-CSI
- **网络：** 10/100M 以太网，静态 IP `192.168.1.200`

## 目录结构

```
esp32p4c5_edgeAI_lot/
├── CMakeLists.txt              # 顶层 CMake
├── sdkconfig                   # Kconfig 配置
├── sdkconfig.defaults           # 默认配置覆盖
├── partitions.csv               # Flash 分区表
├── .gitignore                   # 不上传的文件列表
├── scripts/
│   ├── build_now.bat            # CMD 编译脚本
│   ├── build_p4.bat             # 编译脚本（旧）
│   └── flash_p4.bat             # 烧录脚本（仅固件）
├── components/                  # 自带的 BSP 组件
│   ├── wt99p4c5_s1_board/       # 板卡 BSP
│   └── bsp_extra/               # BSP 扩展
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml        # 组件依赖声明
    ├── app_main.cpp             # 主入口：初始化 + 主循环
    ├── app_video.c / .h         # V4L2 摄像头
    ├── face_ai.cpp / .hpp       # 人脸检测+识别
    ├── http_server.cpp / .hpp   # HTTP + MJPEG 推流
    ├── jpeg_annotate.cpp / .hpp # 画框标注
    └── pest_ai.cpp / .hpp       # ⚠️ 已废弃 — 害虫检测迁移至 PC 端
```

## 编译

打开 **Windows CMD**（不是 Git Bash）：

```cmd
D:\esp32\tmp\build_now.bat
```

## 烧录

```cmd
D:\esp32\tmp\flash_p4.bat COM7
```

## HTTP API（port 8080）

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/` | 仪表盘 |
| GET | `/stream` | MJPEG 视频流（带人脸框） |
| GET | `/api/info` | 设备信息 / FPS / 录入数 |
| GET | `/api/events` | 最新识别事件 |
| POST | `/api/enroll` | 录入人脸 |
| POST | `/api/delete` | 删除最近录入的人脸 |

## 启动日志

```
ESP-ROM:esp32p4-20240710
I (6721) app_video: allocated 4 buffers
I (6743) http: listening on :8080
I (6748) p4fs: Streaming at http://192.168.1.200:8080/
I (6748) p4fs: startup: free_heap=20057980 psram_free=19617548
I (11788) p4fs: alive frames=45 fps=9.0 enrolled=0
```

浏览器打开 `http://192.168.1.200:8080/`（PC 以太网口需设为同一网段，如 `192.168.1.100`）。

## 与 PC 端害虫检测配合

本固件的 MJPEG 帧流可供 PC 端 [PestYOLO](../PestYOLO/) 消费：

```
ESP32-P4 (:8080 MJPEG) ──► MQTT publish JPEG ──► PC PestYOLO (:7860)
                                                       │
                               YOLOv8 102类害虫检测 + Web UI
```

详见 [PestYOLO README](../PestYOLO/README.MD)。
