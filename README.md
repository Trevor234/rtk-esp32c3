# ESP32-C3 GPS / RTK 模拟器

基于 ESP32-C3 的通用 GPS 与双天线 RTK 模拟器，专为开源飞控（如 ArduPilot、PX4）及地面站 / ROS / MAVROS 室内台架联调设计。

在无需外接真实 GPS 模块、无开阔天空搜星环境的条件下，该模拟器可通过串口输出高精度定位、真实双天线航向（`HDT`）及 RTCM 差分链路交互，使飞控 EKF 状态机在台架上即可完成对齐与全链路验证。

---

## 1. 功能特性

- **标准 NMEA 0183 输出**：支持 5 Hz / 10 Hz 周期输出 `$GNGGA`、`$GNRMC` 与 `$GNHDT`（真航向）。
- **双天线航向模拟**：专为依赖 GPS 测向（如纯 GPS 航向对齐）的飞控算法设计，解决室内调试时 EKF 拒绝对齐的问题。
- **RTCM 3.x 差分链路闭环**：
  - 串口双向通信，可接收飞控或地面站注入的 RTCM 差分数据包；
  - 自动完成 RTCM 3.x 分帧、CRC-24Q 校验与帧类型统计；
  - 支持差分超时自动降级（Fixed / Float → 单点定位），真实模拟差分链路通断行为。
- **动态运动模型**：内置静止、直线定速运动、圆周回转等运动学轨迹推算。
- **噪声注入（Gauss-Markov 模型）**：支持水平位置游走（米级/厘米级）与航向角漂移仿真，模拟真实卫星接收机的统计特性。
- **Web UI 与 REST API**：
  - 上电自动建立 Wi-Fi 热点并启动 HTTP 服务器；
  - 网页端实时监控定位状态、RTCM 统计、动态调整经纬度原点、航向、速度、精度因子等；
  - 提供 RESTful 接口，支持自动化脚本控制。
- **独立解耦的核心库**：NMEA 生成、RTCM 解码及运动推算为纯 C 实现，不依赖 ESP-IDF，支持在 PC 宿主机上独立编译运行自动化单元测试。

---

## 2. 硬件接线

推荐使用 ESP32-C3 开发板（3.3V TTL 电平），与飞控串口连接时**必须共地**：

```
       ESP32-C3                          飞控 GPS 串口
   ┌──────────────┐                    ┌──────────────────┐
   │  GPIO21 TX   ├──── NMEA 数据 ─────►│  RX              │
   │  GPIO20 RX   ◄──── RTCM 差分 ─────┤  TX              │
   │  GND         ├────────────────────┤  GND             │
   │  5V / 3V3    ├────────────────────┤  VCC             │
   └──────────────┘                    └──────────────────┘

   状态指示（可选）：
   - 板载 LED（默认 GPIO8）：每次发送 NMEA 数据闪烁一次。
   - 调试监视口（默认 GPIO7）：发送镜像 NMEA 流，便于用 USB-TTL 串口助手监听。
```

> **注意**：ESP32-C3 的 UART0 默认分配给 GPS 链路输出；日志输出已配置重定向至芯片内置的 **USB-Serial-JTAG**，因此通过 USB 调试日志不会与飞控串口发生冲突。

---

## 3. 快速烧录与构建

### 方式 A：直接烧录合并固件（无需编译环境）

项目 `dist/` 目录下提供了预编译的 4MB 完整固件映像：

```bash
# 使用 esptool 烧录（请将 PORT 替换为实际串口号，如 /dev/ttyUSB0 或 COMx）
python -m esptool --chip esp32c3 -p PORT -b 460800 write_flash 0x0 dist/usv_gps_sim_esp32c3_merged.bin
```

### 方式 B：源码编译与烧录

基于标准 ESP-IDF v5.x / v6.x：

```bash
cd firmware
idf.py set-target esp32c3
idf.py build
idf.py -p PORT flash monitor
```

---

## 4. 飞控参数配置参考

以 ArduPilot Rover / Copter 为例，GPS 串口主要配置建议如下：

| 参数 | 推荐值 | 说明 |
|---|---|---|
| `GPS1_TYPE` | `5` (NMEA) | 采用标准 NMEA 协议解析 |
| `GPS1_RATE_MS` | `200` (5 Hz) | 匹配模拟器输出频率 |
| `SERIALx_PROTOCOL` | `5` (GPS) | 对应连接 GPS 的飞控串口 |
| `SERIALx_BAUD` | `115` 或 `230` | 串口波特率（须与模拟器配置一致，默认 115200） |
| `EK3_SRC1_YAW` | `2` (GPS) | 启用双天线 GPS 航向对齐 |
| `EK3_SRC1_POSXY` | `3` (GPS) | 水平位置源使用 GPS |
| `GPS_AUTO_CONFIG` | `0` | 台架测试建议关闭飞控自动探测，保持参数确定性 |

---

## 5. Web 控制台与 API

### Wi-Fi 访问
- 设备上电后自动广播 Wi-Fi 热点：**`USV-GPS-SIM`**
- 默认密码：`usvgps123`
- 连接后在浏览器中打开：`http://192.168.4.1/`

### 控制台主要功能
1. **状态监测**：实时查看当前经纬度、高程、航向角、定位状态（Fix Quality）、卫星数、HDOP、基线长度，以及 RTCM 数据帧统计与校验状态。
2. **位置与运动**：实时修改坐标原点（支持常见 WGS84 坐标输入）、设定运动速度、航向角及运行模式（静止 / 圆周 / 直线）。
3. **定位解算质量**：一键切换单点定位、DGPS、RTK Float 与 RTK Fixed 状态，便于测试上层软件的故障容错。
4. **噪声注入**：支持调节水平位置噪声 $\sigma$（米）与航向漂移 $\sigma$（度），模拟真实复杂环境。

### REST API 控制接口
支持通过 HTTP 接口由自动化测试脚本直接控制模拟器：

- **获取当前状态**：
  ```bash
  curl http://192.168.4.1/api/status
  ```

- **修改模拟参数**（例如设置为 RTK Fixed、航向 90°、速度 2 m/s）：
  ```bash
  curl -X POST http://192.168.4.1/api/set \
    -H "Content-Type: application/json" \
    -d '{"fix_quality":4,"heading_deg":90,"speed_mps":2,"mode":2}'
  ```

- **软重启模拟器**：
  ```bash
  curl -X POST http://192.168.4.1/api/restart
  ```

---

## 6. 宿主机自动化验证

核心算法组件（`nmea.c` / `rtcm.c` / `sim.c`）具有高度跨平台可移植性，项目附带针对 NMEA 帧时序、度分转换、RTCM 校验及飞控解析特性的宿主机测试套件。

```bash
# Linux / macOS
./tests/run_tests.sh

# Windows (PowerShell)
.\tests\run_tests.ps1
```

---

## 7. 目录结构

```text
├── dist/                     # 预编译固件 (4MB 合并镜像，开箱即烧录)
├── firmware/                 # ESP-IDF 固件工程源码
│   ├── main/
│   │   ├── main.c           # 系统初始化、UART/WiFi 任务调度
│   │   ├── nmea.c / .h      # NMEA 语句组帧与校验和计算
│   │   ├── rtcm.c / .h      # RTCM 3.x 分帧与 CRC-24Q 校验
│   │   ├── sim.c / .h       # 运动推算、精度模型与噪声发生器
│   │   ├── web.c / .h       # HTTP Web 服务器与交互控制面板
│   │   └── Kconfig.projbuild# 模块与管脚配置项
│   ├── CMakeLists.txt
│   └── sdkconfig.defaults   # 默认工程配置
├── tests/                    # 宿主机单元测试套件
│   ├── test_nmea.c
│   ├── run_tests.sh
│   └── run_tests.ps1
├── tools/                    # 辅助脚本（如离线瓦片下载与打包工具）
├── .gitignore
└── README.md
```

---

## 8. 开源许可

本项目遵循 [MIT License](LICENSE) 开源协议。
