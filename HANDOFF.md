# WT99P4C5-S1 开发交接文档

> 最后更新: 2026-07-05

## 硬件概览

WT99P4C5-S1 是双芯片开发板：

| 芯片 | 功能 | COM 口 | USB-UART 芯片 |
|------|------|--------|---------------|
| **ESP32-P4** | 主控：人脸识别 + MJPEG 推流 + 以太网 | COM7 | CP210x (Silicon Labs) |
| **ESP32-C5** | WiFi 6 / BLE 5.0 协处理器（通过 SDIO 连接 P4） | COM9 | FTDI FT232R |

### SDIO 引脚映射

| 信号 | P4 GPIO | C5 GPIO |
|------|---------|---------|
| CLK  | 18      | 9       |
| CMD  | 19      | 10      |
| D0   | 14      | 8       |
| D1   | 15      | 7       |
| D2   | 16      | 14      |
| D3   | 17      | 13      |
| RST  | 13      | CHIP_PU |

C5 的 SDIO 引脚是硬件固定的（无法软件修改）。两个芯片各有一个 USB-C 口，对应不同的 COM 口。

---

## 当前状态一览

### ✅ ESP32-P4 — 已完成

- **固件**：`p4_face_stream`（人脸识别 + MJPEG HTTP 推流）
- **项目路径**：`D:\esp32\esp32p4c5_edgeAI_lot\`
- **GitHub**：https://github.com/Hphstc1/esp32p4c5_edgeAI_lot
- **ESP-IDF**：v5.5.4，路径 `D:\esp32\Espressif\frameworks\esp-idf-v5.5.4`
- **编译**：`D:\esp32\tmp\build_p4_now.bat`（Windows CMD 运行）
- **烧录**：`D:\esp32\esp32p4c5_edgeAI_lot\scripts\flash_p4.bat COM7`
- **烧录参数**：DIO, 80MHz, 16MB flash, bootloader@0x2000, 分区表@0x8000, 固件@0x10000
- **功能**：摄像头 OV5647 1280×960 → 人脸检测+识别 → 硬件 JPEG → HTTP MJPEG 推流
- **网络**：
  - 以太网静态 IP `192.168.1.200`（eth0）
  - C5 WiFi AP `P4-FaceStream` @ `192.168.4.1:8080`（通过 SDIO AT 命令控制）

### ✅ ESP32-C5 — 已完成（SDIO AT 模式）

- **固件**：ESP-AT（SDIO 模式），模块 `ESP32C5-SDIO`
- **ESP-AT 源码**：`D:\esp32\esp-at-c5\`
- **ESP-IDF**：v5.5.2，路径 `D:\esp32\esp-at-c5\esp-idf`
- **编译**：`D:\esp32\tmp\build_c5_now.bat`（Windows CMD 运行）
- **烧录**：见下文烧录命令章节
- **Console UART**：GPIO11(TX) / GPIO12(RX) @ 115200 → COM9
- **SDIO 模式**：当前 1-bit @ 400kHz（已验证稳定，可后续提速）

---

## P4-C5 SDIO 通信架构

### 协议栈

```
P4 (Host)                              C5 (Slave)
┌─────────────────┐                 ┌──────────────────────┐
│ c5_at_sdio.c    │  AT commands    │ at_sdio_task.c       │
│ (AT over SDIO)  │ ◄─────────────► │ (sdio_slave_recv/    │
├─────────────────┤   over SDIO     │  sdio_slave_transmit)│
│ c5_sdio_transport│                 │ driver/sdio_slave.h  │
│ (flow control,  │  CMD52/CMD53    │ (ESP-IDF SDIO slave) │
│  FIFO protocol) │ ◄─────────────► │                      │
├─────────────────┤                 └──────────────────────┘
│ c5_sdio_probe.c │
│ (card detect,   │
│  GPIO init)     │
├─────────────────┤
│ driver/sdmmc    │
│ (ESP-IDF host)  │
└─────────────────┘
```

### 关键文件

| 文件 | 用途 |
|------|------|
| `components/c5_bridge/c5_sdio_probe.c` | P4 端：SDIO 卡检测、GPIO 初始化、时钟控制 |
| `components/c5_bridge/c5_sdio_transport.c` | P4 端：SDIO 传输层（基于官方 at_sdio_host 重写） |
| `components/c5_bridge/c5_at_sdio.c` | P4 端：AT 命令封装（WiFi 连接、TCP 收发等） |
| `D:\esp32\esp-at-c5\main\interface\sdio\at_sdio_task.c` | C5 端：SDIO 从机接收/发送 |
| `D:\esp32\esp-at-c5\examples\at_sdio_host\` | **官方参考实现**（这次修复的依据） |

### SDIO 通信协议（基于官方 at_sdio_host）

```
发送流程:
  1. 读 REG_SDIO_TOKEN_RDATA (Fn1:0x044) — 检查从机可用 buffer 数量
  2. 等 buffer 足够 → CMD53 写入地址 0x1F800 - data_len
  3. 写 REG_SDIO_CONF (Fn1:0x08C) — 通知从机数据就绪

接收流程:
  1. 等 DAT1 中断（sdmmc_io_wait_int）
  2. 读 REG_SDIO_PKT_LEN (Fn1:0x060) — 检查从机待发数据量
  3. CMD53 从地址 0x1F800 - data_len 读取数据

初始化序列（esp_slave_init_io）:
  1. 读 IOE/IOR
  2. 写 IOE=0x06 (Fn1 + Fn2)
  3. 写 IOR=0x06 (通知从机)
  4. 写 IE=0x07 (master + Fn1 + Fn2 中断使能)
  5. 设 Fn0/Fn1/Fn2 block size = 512
```

---

## 编译环境详情

### ESP-IDF v5.5.4（用于 P4）

| 项目 | 路径/版本 |
|------|-----------|
| IDF 路径 | `D:\esp32\Espressif\frameworks\esp-idf-v5.5.4` |
| Python venv | `D:\esp32\Espressif\python_env\idf5.5_py3.12_env` |
| 工具链 | `D:\esp32\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121` |
| CMake | `D:\esp32\Espressif\tools\cmake\3.30.2` |
| Ninja | `D:\esp32\Espressif\tools\ninja\1.12.1` |
| ROM ELFs | `D:\esp32\Espressif\tools\esp-rom-elfs\20241011` |

### ESP-IDF v5.5.2（用于 C5 ESP-AT）

| 项目 | 路径/版本 |
|------|-----------|
| IDF 路径 | `D:\esp32\esp-at-c5\esp-idf` |
| Python venv | `D:\esp32\Espressif\python_env\idf5.5_py3.12_env` |
| 工具链 | `D:\esp32\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20251107` |

---

## 烧录命令

### P4 烧录（COM7）

```cmd
D:\esp32\esp32p4c5_edgeAI_lot\scripts\flash_p4.bat COM7
```

或手动：
```cmd
D:\esp32\Espressif\python_env\idf5.5_py3.12_env\Scripts\python.exe -m esptool --chip esp32p4 -p COM7 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 16MB --flash_freq 80m ^
  0x2000 D:\esp32\esp32p4c5_edgeAI_lot\build\bootloader\bootloader.bin ^
  0x8000 D:\esp32\esp32p4c5_edgeAI_lot\build\partition_table\partition-table.bin ^
  0x10000 D:\esp32\esp32p4c5_edgeAI_lot\build\p4_face_stream.bin
```

### C5 烧录（COM9，BOOT 需先接地！）

**完整工厂镜像（推荐）**：
```cmd
D:\esp32\Espressif\python_env\idf5.5_py3.12_env\Scripts\python.exe -m esptool --chip esp32c5 -p COM9 -b 115200 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m 0x0 D:\esp32\esp-at-c5\build\factory\factory_ESP32C5-SDIO.bin
```

**C5 烧录关键参数**：
- Flash: DIO, 80MHz, **4MB**
- 镜像偏移: **0x0**（工厂镜像已包含 bootloader + 分区表 + 固件）
- BOOT 引脚必须**接地**才能进下载模式
- C5 的 Console UART 输出在 GPIO11/12（COM9 上可见，但 SDIO 模式下 AT 命令走 SDIO 不走 UART）

---

## 问题排查记录

### 已解决：SDIO CMD53 超时 (0x107 = ESP_ERR_TIMEOUT)

**现象**：P4 检测到 C5 SDIO 卡（CMD52 到 CCCR 正常），但 CMD53 数据传输全部超时。

**根因**：P4 端 SDIO 传输层实现与 ESP-AT C5 从机协议不匹配：

1. **初始化序列不完整** — 只使能了 Fn1，官方需要 Fn1 + Fn2 同时使能
2. **缺少流控** — 没有在发送前检查从机 buffer 可用量
3. **缺少中断通知** — 发送后没有通知从机数据就绪
4. **时钟过快** — 直接上 20MHz 4-bit，应从 400kHz 1-bit 起步

**解决**：基于官方 `examples/at_sdio_host` 参考实现重写 `c5_sdio_transport.c`：
- 1-bit 模式 + 400kHz 时钟（README 建议："适配一开始建议使用 1-bit 模式，并将 SDIO clock 尽量调低"）
- 完整 CCCR 初始化（IOE=6, IOR write, Fn0/Fn1/Fn2 block size, IE=7）
- 发送前读 TOKEN_RDATA 检查 buffer，发送后写 CONF 触发中断
- 接收前读 PKT_LEN 检查数据量

**验证日志**：
```
I (6254) c5_sdio_transport: CCCR IOE: 0x00
I (6262) c5_sdio_transport: CCCR IOE (after enable): 0x06
I (6275) c5_sdio_transport: CCCR IE (after enable): 0x07
I (6306) c5_sdio_transport: SDIO function enable complete (Fn1 + Fn2, block_size=512)
I (8829) c5_at_sdio: AT test OK                          ← 问题解决
I (8872) wifi: WiFi AP: P4-FaceStream IP: 192.168.4.1    ← 全链路通
```

### 已解决：C5 ESP-AT COM9 无输出

**原因**：ESP-AT 在 SDIO 模式下，AT 命令走 SDIO 通道，UART 只用作 console。最初 COM9 无输出是因为 console UART 引脚配置为默认值（非 GPIO11/12）。

**解决**：在 `sdkconfig` 和 `sdkconfig.defaults` 中配置 `CONFIG_ESP_CONSOLE_UART_CUSTOM=y`，TX=11, RX=12。验证后 C5 console 输出正常。

---

## 下一步建议

### 性能优化（按优先级排列）

1. **提升 SDIO 时钟**（当前 400kHz → 目标 20MHz）
   - 在 `c5_sdio_probe.c` 中取消 `sdmmc_host_set_card_clk()` 的注释
   - 逐步测试：1MHz → 5MHz → 10MHz → 20MHz
   - 每次升频后验证 AT 命令响应正常

2. **切换到 4-bit 模式**
   - 修改 `host.flags` 加入 `SDMMC_HOST_FLAG_4BIT`
   - 修改 `slot_cfg.width` 从 1 改为 4
   - 需要板上 SDIO 数据线有上拉电阻（DAT2/DAT3）

3. **参考测试数据**（官方 at_sdio_host README）：
   - ESP32 host + 4-bit @ 20MHz: TX 13.3Mbps, RX 12.4Mbps
   - 当前 1-bit @ 400kHz 吞吐量约 50KB/s，仅够基本 AT 命令

### 功能扩展

- 将 C5 WiFi 从 AP 模式切换到 STA 模式（连接外部路由器）
- 利用 C5 的 BLE 5.0 功能
- 测试 TCP 大数据透传（`c5_at_sdio_tcp_send/recv`）

---

## 关键文件索引

| 文件 | 用途 |
|------|------|
| `D:\esp32\tmp\build_p4_now.bat` | P4 编译脚本 |
| `D:\esp32\esp32p4c5_edgeAI_lot\scripts\flash_p4.bat` | P4 烧录脚本 |
| `D:\esp32\tmp\build_c5_now.bat` | C5 ESP-AT 编译脚本 |
| `D:\esp32\esp32p4c5_edgeAI_lot\components\c5_bridge\c5_sdio_transport.c` | **P4 SDIO 传输层**（基于官方例程重写） |
| `D:\esp32\esp32p4c5_edgeAI_lot\components\c5_bridge\c5_sdio_probe.c` | P4 SDIO 卡检测与初始化 |
| `D:\esp32\esp32p4c5_edgeAI_lot\components\c5_bridge\c5_at_sdio.c` | P4 AT 命令封装 |
| `D:\esp32\esp-at-c5\examples\at_sdio_host\` | **官方 SDIO host 参考实现** |
| `D:\esp32\esp-at-c5\main\interface\sdio\at_sdio_task.c` | C5 SDIO 从机实现 |
| `D:\esp32\esp-at-c5\sdkconfig.defaults` | C5 Console UART 引脚配置 |
| `D:\esp32\esp-at-c5\build\factory\factory_ESP32C5-SDIO.bin` | C5 工厂镜像 |
| `D:\esp32\esp32p4c5_edgeAI_lot\build\p4_face_stream.bin` | P4 固件镜像 |
