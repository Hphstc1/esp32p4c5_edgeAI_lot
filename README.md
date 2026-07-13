# esp32p4c5_edgeAI_lot

ESP32-P4 人脸识别 + MJPEG 实时推流，基于 **WT99P4C5-S1** 开发板，ESP-IDF **v5.5.4**。

---

## 功能

1. OV5647 摄像头 RGB565 采集，1280×960，硬件 JPEG 编码。
2. 人脸检测（ESPDet-pico 224×224）+ 人脸识别（MobileFaceNet，ESP-DL 量化）。
3. 检测框/ID 标注叠加。
4. HTTP MJPEG 推流（port 8080，~10 FPS）。
5. JSON API：录入 / 删除 / 设备信息 / 识别事件。
6. SPIFFS 人脸库（重启不丢失）。
7. WiFi **STA 模式**，连接手机热点；支持串口配置 NVS 保存。

---

## 硬件

- **芯片：** ESP32-P4，双核 RISC-V @ 360MHz，32 MB PSRAM
- **开发板：** WT99P4C5-S1
- **摄像头：** OV5647 MIPI-CSI
- **网络：** ESP32-C5 SDIO 协处理器提供 WiFi（`esp_wifi_remote` + `esp_hosted`）

---

## 当前硬编码配置

为 Demo 方便，热点信息已硬编码在 `main/app_main.cpp`：

```cpp
char g_wifi_ssid[SSID_MAX + 1] = "hph666";
char g_wifi_pass[PASS_MAX + 1] = "He4496385";
char g_target_ip[IP_MAX]       = "192.168.43.242";
```

如需修改，直接改 `main/app_main.cpp` 后重新编译烧录。

> 上电后也会尝试从 NVS 命名空间 `p4cfg` 读取配置；若 NVS 中保存的值非空，会覆盖硬编码默认值。

---

## 串口命令行

上电后通过 UART0/USB-Serial-JTAG（115200）进入命令行。

支持命令：

```text
ssid <name>      设置 WiFi 热点名称
pass <pwd>       设置 WiFi 密码
target <ip>      设置 PC 桥接端 IP
save             保存到 NVS
reboot           重启设备
status           查看当前配置和连接状态
help             显示帮助
```

示例：

```text
ssid hph666
pass He4496385
target 192.168.43.242
save
reboot
```

---

## 目录结构

```text
esp32p4c5_edgeAI_lot/
├── CMakeLists.txt              # 顶层 CMake
├── sdkconfig                   # Kconfig 配置
├── sdkconfig.defaults          # 默认配置覆盖
├── partitions.csv              # Flash 分区表
├── .gitignore
├── components/                 # 自带的 BSP 组件
│   ├── wt99p4c5_s1_board/      # 板卡 BSP
│   └── bsp_extra/              # BSP 扩展
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml       # 组件依赖声明
│   ├── app_main.cpp            # 主入口：WiFi STA、NVS、串口配置、视频/人脸/HTTP 启动
│   ├── app_video.c / .h        # V4L2 摄像头
│   ├── face_ai.cpp / .hpp      # 人脸检测+识别
│   ├── http_server.cpp / .hpp  # HTTP + MJPEG 推流（async /stream worker）
│   └── jpeg_annotate.cpp / .hpp # 画框标注
└── README.md                   # 本文件
```

---

## 编译与烧录

### 环境要求

- ESP-IDF v5.5.4（已安装于 `D:\esp32\Espressif\frameworks\esp-idf-v5.5.4`）
- Python 虚拟环境：`D:\esp32\Espressif\python_env\idf5.5_py3.12_env`
- 串口：`COM7`

### 一键脚本

```cmd
D:\esp32\tmp\build_p4fs_5.5.4.bat    # 编译
D:\esp32\tmp\flash_p4fs_5.5.4.bat    # 烧录
D:\esp32\tmp\monitor_p4fs_5.5.4.bat  # 串口监视器
```

### 手动编译（Git Bash）

```bash
cd D:/esp32/esp32p4c5_edgeAI_lot
MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1 cmd /c 'D:\esp32\tmp\build_p4fs_5.5.4.bat'
```

### 手动烧录

```bash
cd D:/esp32/esp32p4c5_edgeAI_lot
MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1 cmd /c 'D:\esp32\tmp\flash_p4fs_5.5.4.bat'
```

### 串口监视

```bash
cd D:/esp32/esp32p4c5_edgeAI_lot
MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1 cmd /c 'D:\esp32\tmp\monitor_p4fs_5.5.4.bat'
```

---

## 关键日志

启动成功后，串口会输出：

```text
P4 console ready. Commands: ssid <n> | pass <p> | target <ip> | save | reboot | status | help
...
I (xxxx) p4fs: WiFi STA started, SSID=hph666
...
I (xxxx) p4fs: WiFi connected, IP=192.168.43.50
I (xxxx) p4fs: HTTP server started on port 8080
```

记下 `IP=` 后面的地址，供 PC 桥接使用。

---

## HTTP API（port 8080）

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/` | 示例仪表盘 |
| GET | `/stream` | MJPEG 视频流（带人脸框） |
| GET | `/api/info` | 设备信息 / FPS / 录入数 / `target_ip` |
| GET | `/api/events` | 最新识别事件 |
| POST | `/api/enroll` | 录入人脸 |
| POST | `/api/delete` | 删除最近录入的人脸 |

---

## Demo 当天流程

1. 手机开启热点 `hph666`，密码 `He4496385`。
2. PC 连接该热点。
3. 给 p4c5 上电，固件自动连接热点。
4. 查看串口日志获取 p4c5 IP（如 `192.168.43.50`）。
5. PC 运行桥接：`python bridge.py --p4c5-ip 192.168.43.50 --port 8081`。
6. 浏览器打开 `http://localhost:8081/`。

---

## 关键修复记录

- **WiFi 模式：** 从 SoftAP 改为 STA，支持手机热点 Demo 场景。
- **NVS 配置：** 新增 `p4cfg` 命名空间保存 SSID / 密码 / target_ip。
- **串口配置：** 新增命令行任务，修复 `P4>` 刷屏问题。
- **模型：** 使用 `espdet_pico_224_224_face.espdl` 人脸检测模型（`CONFIG_DEFAULT_HUMAN_FACE_DETECT_MODEL=1`）。
- **C5 SDIO：** 通过 `ESP_IDF_VERSION=5.5` 让 `esp_wifi_remote` 正确选择 `SLAVE_IDF_TARGET_ESP32C5`。
- **HTTP 单线程阻塞：** `/stream` 改为 async handler + worker task pool，`/api/info` 与 `/api/events` 可并发响应。
- **max_clients：** 受 LWIP_MAX_SOCKETS 限制，设置为 7（httpd 内部占用 3 个）。
- **ETH 初始化：** 改为非致命失败，避免 PHY 偶发未响应时直接 abort。

---

## 常见问题

| 现象 | 处理 |
|---|---|
| 串口 `P4>` 刷屏 | 已修复，使用 `getchar()` 轮询 |
| WiFi 连不上 | 检查热点名/密码；用串口 `status` 查看 |
| 拿不到 IP | 确认手机热点开启且 PC 也连在同一热点；部分热点需关闭 AP 隔离 |
| 编译报错 `stringop-truncation` | 已修复，使用 `strnlen` + `memcpy` |
