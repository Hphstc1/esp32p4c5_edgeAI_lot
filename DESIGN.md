# P4-C5 SDIO Bridge Design

## Overview

The `c5_bridge` component enables communication between an ESP32-P4 (host) and an ESP32-C5 (ESP-AT SDIO slave) over the SDIO bus. The P4 acts as the SDMMC host controller (Slot 1), and the C5 acts as an SDIO slave device providing AT command processing and WiFi connectivity.

## Architecture

```
+-------------------+       SDIO Bus (4-bit)       +-------------------+
|  ESP32-P4 (Host)  |<---------------------------->|  ESP32-C5 (Slave) |
|                   |  CLK=GPIO18  CMD=GPIO19      |                   |
|  SDMMC Slot 1     |  D0=GPIO14  D1=GPIO15        |  ESP-AT Firmware  |
|                   |  D2=GPIO16  D3=GPIO17        |                   |
|  RST=GPIO13       |------------------------------>|  CHIP_PU          |
+-------------------+                               +-------------------+
```

## Software Layers

### 1. Probe Layer (`c5_sdio_probe.c`)
- Resets C5 via GPIO13 (CHIP_PU)
- Initializes SDMMC host driver for Slot 1
- Detects C5 as SDIO card via `sdmmc_card_init()`
- Validates detection (`is_sdio=1`, `num_io_functions>=1`)

### 2. Transport Layer (`c5_sdio_transport.c`)
- Wraps `sdmmc_io_read_bytes()` / `sdmmc_io_write_bytes()` for CMD53 byte-mode transfers
- Handles SDIO interrupt wait with timeout/retry
- Manages SDIO CCCR initialization:
  - Enables Function 1 via CCCR I/O Enable (0x02)
  - Polls Function 1 ready via CCCR I/O Ready (0x03)
  - Sets Function 1 block size to 512 via FBR registers (0x110-0x111)
  - Configures function interrupts via CCCR Int Enable (0x04)

### 3. AT Command Layer (`c5_at_sdio.c`)
- Sends AT commands over SDIO to C5
- Parses responses looking for OK/ERROR terminators
- Implements WiFi STA connection, SoftAP, TCP send/recv
- Handles +IPD data framing for TCP receive

## File Ownership

| File | Owner | Purpose |
|------|-------|---------|
| `components/c5_bridge/c5_sdio_probe.c` | c5_bridge component | SDIO card detection and host init |
| `components/c5_bridge/c5_sdio_transport.c` | c5_bridge component | SDIO read/write transport + CCCR init |
| `components/c5_bridge/c5_at_sdio.c` | c5_bridge component | AT command protocol over SDIO |
| `components/c5_bridge/include/c5_bridge.h` | c5_bridge component | Public API declarations |
| `components/c5_bridge/CMakeLists.txt` | c5_bridge component | Build configuration |
| `main/app_main.c` | Application | Startup and orchestration |
| `main/sdio_demo.c` | Application | SDIO demo/example code |

## SDIO Protocol Details

### CCCR Initialization (Required)
Per SDIO spec, after power-up all functions beyond Function 0 are disabled. The host must:

1. **Read** CCCR I/O Enable (0x02), set BIT(1), **write back**
2. **Poll** CCCR I/O Ready (0x03) until BIT(1) is set
3. **Set** Function 1 block size to 512 via `FBR[1] + BLKSIZEL/H` (0x110-0x111)
4. **Enable** function interrupts in CCCR Int Enable (0x04) with `BIT(0) | BIT(1)`

### Data Transfer
- CMD53 in byte mode (OP Code set, incrementing address)
- Function 1 for data, Function 0 for CCCR/control
- Host interrupt via D1 line

## Pin Mapping

| P4 GPIO | C5 GPIO | Signal | Notes           |
|---------|---------|--------|-----------------|
| 18      | 9       | CLK    |                 |
| 19      | 10      | CMD    |                 |
| 14      | 8       | D0     |                 |
| 15      | 7       | D1     | Interrupt line  |
| 16      | 14      | D2     |                 |
| 17      | 13      | D3     |                 |
| 13      | CHIP_PU | RESET  | Active-low      |

## Key Files

| File | Purpose |
|------|---------|
| `components/c5_bridge/c5_sdio_probe.c` | SDIO card detection and host init |
| `components/c5_bridge/c5_sdio_transport.c` | SDIO read/write transport + CCCR init |
| `components/c5_bridge/c5_at_sdio.c` | AT command protocol over SDIO |
| `components/c5_bridge/include/c5_bridge.h` | Public API declarations |
| `components/c5_bridge/CMakeLists.txt` | Build config (REQUIRES: driver, sdmmc, freertos) |

## Dependencies

- `sdmmc` component: card init, CMD52/CMD53 I/O
- `driver` component: SDMMC host controller HAL, GPIO
- `freertos`: task delays, tick conversion

## References

- ESP-IDF `sdmmc_io.c` -- CMD53 byte-mode implementation
- ESP-IDF `essl_sdio.c` -- Reference for CCCR function enable sequence
- SDIO Simplified Specification v3.00, Section 3.2: I/O Function Enable
