# V0 桥接服务公网访问操作手册

本文档记录如何让 `p4c5_bridge/V0` 在本地接上 p4c5 后，通过免费公网隧道暴露到互联网。

---

## 当前环境（示例）

| 设备 | 地址 / 说明 |
|---|---|
| 手机热点 | `hph666 / He4496385`（提供局域网 + 外网） |
| PC | `192.168.43.242`，运行 bridge + cloudflared 隧道 |
| p4c5 | `192.168.43.50:8080`，提供 MJPEG 视频流和 API |
| 本地 bridge | `http://localhost:8081` |
| 公网隧道（本次） | `https://packages-structured-opera-islands.trycloudflare.com` |

> 注意：`trycloudflare.com` 地址**每次启动都会变**，下方的 `xxx.trycloudflare.com` 都要替换成你实际拿到的地址。

---

## 目录说明

```text
p4c5_bridge/V0/
├── bridge.py                 # 桥接后端（拉 p4c5 流、YOLO 检测、WebSocket/HTTP）
├── static/dashboard_p4c5.html # 前端 dashboard
├── best.pt                    # YOLOv8 害虫检测模型
├── cloudflared.exe            # Cloudflare TryCloudflare 隧道工具（已下载）
├── venv/                      # V0 自带的虚拟环境（已损坏，不要用）
└── README.md                  # 本文件
```

---

## 前置条件

1. 手机开启热点 `hph666`，密码 `He4496385`。
2. PC 和 p4c5 都连上该热点。
3. p4c5 已上电，串口日志显示连上 WiFi 并拿到 IP。

---

## 启动步骤

### 1. 启动 bridge

V0 自带的 `venv` 指向不存在的 `E:\python.exe`，**改用主项目的 venv**：

```cmd
cd D:\esp32\p4c5_bridge\V0
..\venv\Scripts\python.exe bridge.py --port 8081 --model best.pt
```

默认公网视频流帧率已降到 **5 FPS**，如果想改：

```cmd
..\venv\Scripts\python.exe bridge.py --port 8081 --model best.pt --stream-fps 3
```

成功后会显示：

```text
Discovered p4c5 at 192.168.43.50:8080
Loading YOLOv8 model: D:\esp32\p4c5_bridge\V0\best.pt
Model loaded: 102 classes
Starting HTTP server on http://0.0.0.0:8081
```

### 2. 启动 Cloudflare 公网隧道

在另一个终端窗口执行：

```cmd
cd D:\esp32\p4c5_bridge\V0
cloudflared.exe tunnel --url http://localhost:8081
```

稍等几秒，终端会输出类似：

```text
+--------------------------------------------------------------------------------------------+
|  Your quick Tunnel has been created! Visit it at (it may take some time to be reachable):  |
|  https://packages-structured-opera-islands.trycloudflare.com                               |
+--------------------------------------------------------------------------------------------+
```

复制这个 `https://...` 地址，就是公网访问地址。

---

## 访问地址

把 `xxx.trycloudflare.com` 换成你实际拿到的地址：

| 功能 | 公网地址 |
|---|---|
| Dashboard | `https://xxx.trycloudflare.com/` |
| 视频流 | `https://xxx.trycloudflare.com/stream` |
| WebSocket | `wss://xxx.trycloudflare.com/ws` |
| 设备信息 | `https://xxx.trycloudflare.com/api/info/latest` |
| 最新害虫 | `https://xxx.trycloudflare.com/api/pest/latest` |
| 最新人脸 | `https://xxx.trycloudflare.com/api/face/latest` |

---

## 给 Netlify 前端用

如果你要把 dashboard 部署到 Netlify（`https://adorable-griffin-4f60a0.netlify.app/`），需要让页面知道后端地址。

当前 `dashboard_p4c5.html` 用的是相对路径（`/stream`、`/ws`），从 Netlify 打开会连到 Netlify 自己的服务器，**找不到本地 bridge**。

下一步需要改前端，支持 URL 参数，例如：

```text
https://adorable-griffin-4f60a0.netlify.app/?api=https://packages-structured-opera-islands.trycloudflare.com
```

改完后每次启动隧道得到新地址，只需要改 URL 参数即可，不需要重新部署前端。

---

## 快速验证

在浏览器或手机里打开：

```text
https://packages-structured-opera-islands.trycloudflare.com/
```

如果能看到登录页面，说明公网访问已经通了。

用 curl 测试 API：

```bash
curl https://packages-structured-opera-islands.trycloudflare.com/api/info/latest
```

---

## 常见问题

| 问题 | 原因 / 处理 |
|---|---|
| `did not find executable at 'E:\python.exe'` | V0 自带的 venv 已损坏，用 `..\venv\Scripts\python.exe` |
| 8080 端口被占用 | 本方案已改用 `--port 8081` |
| 视频卡顿 | 手机热点上行带宽不够；可降低 p4c5 分辨率/帧率，或改用模拟模式测试 |
| 隧道地址每次都变 | TryCloudflare 免费特性；演示时把新地址发给观众或写进 Netlify URL 参数。想要固定地址，需要注册 Cloudflare 账号 + 自己的域名，或使用付费 ngrok。 |
| 云端 MQTT 数据不显示 | 确认 PC 能上网；检查 `dashboard_p4c5.html` 里的 MQTT 配置 |

---

## 公网地址是否每次都不同？

**是的。**

`cloudflared tunnel --url http://localhost:8081` 这种免费试用模式，每次启动都会分配一个随机的 `xxx.trycloudflare.com` 地址。

- 本次地址：`https://packages-structured-opera-islands.trycloudflare.com`
- 下次重启隧道后，地址会变。

想要**固定公网地址**，需要：

1. 注册 Cloudflare 账号
2. 添加自己的域名（如 `yourdomain.com`）
3. 创建一个命名隧道（named tunnel）
4. 配置 `https://p4c5.yourdomain.com` 固定指向本地 8081

或者使用付费 ngrok 的保留域名/子域名。

---

## 停止服务

在两个终端分别按 `Ctrl+C`：

1. 先停 `cloudflared.exe`（公网地址失效）
2. 再停 `bridge.py`

---

## 模拟模式（不接 p4c5 时调试用）

如果只想测试前端页面，不接真实 p4c5：

```cmd
..\venv\Scripts\python.exe bridge.py --simulate --port 8081 --model best.pt
```

然后再启动隧道即可。
