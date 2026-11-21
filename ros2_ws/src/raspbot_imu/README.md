# raspbot_imu — IMU 驱动与姿态解算（C++17）

给 RASPBOT-V2（树莓派 5 / Ubuntu 26.04 / ROS 2 Lyrical）用的 IMU 包，**C++17 重写**：支持 **维特智能 HWT101（串口，termios）**、**ICM-42688-P（SPI，spidev ioctl）** 与 **mock** 三个数据源，统一发布 `sensor_msgs/Imu` 到 `/imu/data`（SensorData QoS、depth 10、100 Hz、`frame_id=imu_link`）。

原 Python 版源码已归档到 `E:\code\raspberrypi\legacy_python\raspbot_imu\`（行为参考），其 43 项 pytest 用例原样保留在 `test/*.py` 并已 1:1 重写为 gtest。

## 目录结构

```
raspbot_imu/
├── CMakeLists.txt            # ament_cmake；核心库 + 设备层 + imu_node + 5 个 gtest
├── package.xml
├── include/raspbot_imu/      # 头文件
│   ├── types.hpp             # ImuSample / Vec3 / Quat / 常量
│   ├── quaternion.hpp        # 四元数工具（REP-103）
│   ├── attitude.hpp          # MahonyAHRS / YawEstimator / 零偏估计
│   ├── witmotion.hpp         # 维特 0x55 帧解析（校验和/重同步/分帧）
│   ├── icm42688.hpp          # parse_burst / 量程码表 / 无效标记
│   ├── msgs.hpp              # build_imu_data（不依赖 ROS 的消息组装）
│   ├── config.hpp            # validate_config（参数校验，信息含可选值）
│   ├── sources.hpp           # ImuSource 接口 / SourceOptions / 工厂
│   └── mock_source.hpp       # 合成数据源
├── src/
│   ├── quaternion.cpp attitude.cpp witmotion.cpp icm42688.cpp
│   ├── msgs.cpp config.cpp mock_source.cpp    # 核心库（无 ROS 依赖）
│   ├── sources.cpp           # 设备层：termios 串口 + spidev + 工厂（无 ROS 依赖）
│   └── node.cpp              # rclcpp 节点 + main
├── config/imu.yaml           # 参数（与 Python 版同名同默认值，launch 时加载）
├── launch/imu.launch.py
├── tools/hwt101_static_check.py   # 台架静态检查（Python 工具，保留）
└── test/
    ├── *.py                  # 原 Python 43 用例（字节不变，回归参照）
    └── *.cpp                 # gtest 43 用例（1:1 映射，ament_add_gtest 注册）
```

## 构建与运行（树莓派 5）

```bash
# 串口方案：config.txt 加 enable_uart=1；cmdline.txt 去掉 console=serial0,115200
# udev 规则把 HWT101(CH340) 固定为 /dev/raspbot_imu（见下文接线节）
cd ~/ros2_ws && colcon build --packages-select raspbot_imu
source install/setup.bash
ros2 launch raspbot_imu imu.launch.py
```

验证：

```bash
ros2 topic hz /imu/data        # 期望 ~100 Hz
ros2 topic echo /imu/data --once
```

离线（无硬件）：`ros2 run raspbot_imu imu_node --ros-args -p source:=mock -p calibrate_seconds:=0`

## 单元测试（43 用例，gtest）

colcon 方式（与其它包一起）：

```bash
cd ~/ros2_ws && colcon test --packages-select raspbot_imu && colcon test-result --verbose
```

脱离 colcon 直接编译运行（不碰 `build/`/`install/`，纯逻辑无硬件）：

```bash
cd ~/ros2_ws/src/raspbot_imu
# Ubuntu 上 googletest 头文件与库（libgtest-dev / libgtest 包）
g++ -std=c++17 -I include src/quaternion.cpp src/attitude.cpp src/witmotion.cpp \
    src/icm42688.cpp src/msgs.cpp src/config.cpp src/mock_source.cpp src/sources.cpp \
    test/test_attitude.cpp test/test_icm42688.cpp test/test_sources.cpp \
    test/test_msgs.cpp test/test_config.cpp \
    -lgtest -lgtest_main -pthread -o /tmp/raspbot_imu_gtests
/tmp/raspbot_imu_gtests        # 期望 43 tests PASSED
```

Python 版回归（Windows 开发机，源码在 legacy_python 归档）：

```bash
cd ros2_ws/src/raspbot_imu && python3 -m pytest test -q   # 43 passed（pytest.ini 已指向归档）
```

## 参数（`config/imu.yaml`）

参数名与默认值均与 Python 版一致；C++ 侧直接使用 rclcpp 参数（无需 yaml-cpp，yaml 由 launch 的 `--params-file` 加载）。

| 参数 | 默认 | 说明 |
|---|---|---|
| `source` | `icm42688`（yaml 覆盖为 `witmotion`） | `mock` / `witmotion` / `icm42688` |
| `mode` | `yaw` | `yaw` 只算航向；`mahony` 全姿态（会忽略器件角度） |
| `angle_source` | `gyro`（yaml 覆盖为 `device`） | `device` 用模块自带角度；`gyro` 自己积分 |
| `rate_hz` | 100.0 | 发布频率，与契约一致 |
| `frame_id` | `imu_link` | 契约冻结 |
| `calibrate_seconds` | 5.0 | 上电静止标定；0 表示跳过 |
| `yaw_offset_deg` | 0.0 | IMU 安装方向相对车头的偏角 |
| `serial_port` / `baud` | `/dev/raspbot_imu` / 115200 | HWT101 用（udev 固定名） |
| `spi_bus` / `spi_device` / `spi_speed_hz` | 0 / 0 / 2000000 | ICM-42688 用 |
| `gyro_fs_dps` / `accel_fs_g` / `odr_hz` | 500 / 4 / 1000 | ICM-42688 量程与输出速率 |
| `mahony_kp` / `mahony_ki` | 0.8 / 0.02 | Mahony 增益 |
| `orientation_yaw_variance` | 0.0025（yaml 0.0003） | 航向方差，供 EKF 调参 |
| `orientation_tilt_variance` | 1.0 | yaw 模式下 roll/pitch 未知，给大值 |
| `angular_velocity_variance` / `linear_acceleration_variance` | 0.0004 / 0.04 | 协方差对角元 |

## 接线

### HWT101（串口 / USB-TTL，当前硬件）

```text
HWT101 VCC → 3.3V（pin 1 或 17）   ← 模块 3.3–5.5V，用 3.3V 电平最干净
HWT101 GND → GND（pin 6）
HWT101 TX  → pin 10（GPIO15 / RXD） ← TX/RX 必须交叉
HWT101 RX  → pin 8 （GPIO14 / TXD）
```

- 默认 **115200 8N1**；**出厂输出速率是 10Hz，要用维特上位机改到 100Hz**（不改的话 `/imu/data` 只能到 10Hz，EKF 会抖）。
- 供电 3.3–5.5V（9.5mA）。若买的是 **HWT101CT**（IP67 工业版），供电 5–36V，接 5V。
- 用 5V 供电时要先量 TX 空闲电平；3.3V 供电则逻辑电平就是 3.3V，直接接。
- USB-TTL（CH340）时加 udev 固定名：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="raspbot_imu"
# 直连 ttyAMA0 时改用：SUBSYSTEM=="tty", KERNEL=="ttyAMA0", SYMLINK+="raspbot_imu"
```

- **航向直接用模块的解算结果**：`angle_source: device`（yaml 默认已是）。HWT101 内部做了卡尔曼融合，输出 Z 轴角度（0.1°、几乎不漂），比我们自己积分陀螺好一个量级。
- 模块必须**水平**安装：歪 1° → 航向系统性偏 1°，`yaw_offset_deg` 只能补绕 Z 的偏角，补不了倾斜。

### ICM-42688-P（SPI，备选）

| 模块引脚 | 树莓派 5 40-pin | 说明 |
|---|---|---|
| VCC | pin 1 / 17（3.3V） | **必须 3.3V** |
| GND | pin 6 | 共地 |
| SCLK | pin 23（GPIO11） | SPI0 SCLK |
| MOSI | pin 19（GPIO10） | SPI0 MOSI |
| MISO | pin 21（GPIO9） | SPI0 MISO |
| CS | pin 24（GPIO8） | SPI0 CE0 |

- 需 `dtparam=spi=on`；驱动用 `/dev/spidev{bus}.{device}` ioctl，SPI 模式 3、默认 2 MHz。
- WHO_AM_I 读不到会直接报错退出，不会“静默成功”。

### 第一次上电：台架验证（10 分钟）

```bash
python3 tools/hwt101_static_check.py --seconds 60
# 默认端口 /dev/raspbot_imu；直连时 --port /dev/ttyUSB0 覆盖
```

脚本会打印帧率、航向均值、标准差（噪声）、线性漂移（°/min），并给一句判定：

| 结果 | 含义 | 下一步 |
|---|---|---|
| 帧率 ≈10 Hz | 还是出厂速率 | 用维特上位机改到 100 Hz |
| 标准差 < 0.05°、漂移 < 0.1°/min | 模块状态很好 | 直接进装配 |
| 标准差 0.05–0.2° | 桌面上的正常水平 | 可用，SLAM 前加 10–30Hz 低通 |
| 噪声大 / 漂移大 | 别急着怪模块 | 依次查：是否水平、线束是否在拽、附近电机线/电源 |

## 算法与标定

- **angle_source=device**：`orientation` 直接用 HWT101 的 Z 角度，`angular_velocity` 仍是模块原始角速度；`YawEstimator` / 零偏标定不参与。
- **YawEstimator**：`yaw += (gyro_z - bias_z) * dt`；静止（角速度小、加速度模长≈g）时用 EMA 慢速跟踪 `bias_z`。
- **MahonyAHRS**：加速度计只在模长接近 g（±25% 门限）时参与修正 roll/pitch；yaw 零偏靠启动标定。
- **零偏标定**：静止采集，三轴均值与标准差；标准差超过 0.05 rad/s 判定“车在动”，放弃本次标定并告警（不退出）。

## 已知限制

- **航向是相对的**：没有磁力计/绝对参考，上电位置即 0°；长期漂移不可避免，2D 导航需用 `/scan` 匹配兜底。
- **无温度补偿**：陀螺零偏随温度变化，IMU 远离发热源。
- **振动**：mecanum 轮振动较大，务必机械减震；必要时在驱动后加 30–50 Hz 低通（当前未实现）。
- **单帧无效数据**：ICM-42688 返回 -32768 时该帧被跳过（`SpiSource` 吞掉异常计为空读）。
- **热拔/掉线期间停止发布**：数据源读或处理异常时节点不退出，限频 WARN（≤1 次/5 s）并在周期 INFO 报告里累计错误次数；期间 `/imu/data` **停止发布**（不发假值），源恢复后自动继续。消费方（EKF）需按 FR-E-02 做掉线降级（限速 ≤0.5 m/s）。
- **SIGINT/SIGTERM 优雅退出**：rclcpp 信号处理接管，`spin` 正常返回、退出码 0（对应 Python 版捕获 `ExternalShutdownException`）。

## 与 Python 版的语义差异（有意为之）

1. **QoS**：Python 版用默认 QoS（Reliable、depth 10）；C++ 按契约 §2 用 `SensorDataQoS().keep_last(10)`（best-effort、depth 10）。
2. **串口断线恢复**：Python 版靠 pyserial 行为；C++ 读失败即关 fd，之后按 1 s 节流重开 `/dev/raspbot_imu`，USB 重插后**自动恢复发布**；错误计数语义不变（掉线期间每次失败都计入累计计数、WARN ≤1 次/5 s）。
3. **mock 噪声序列**：Python 用 numpy `default_rng(seed)`（PCG64），C++ 用 `std::mt19937(seed)`；确定性部分（时间、运动轨迹、零偏）完全一致，噪声取值不同（测试不依赖噪声值）。
4. **SPI 访问**：Python 用 `spidev` 库，C++ 用 `/dev/spidev*` ioctl（SPI_IOC_MESSAGE），寄存器/时序一致。
5. **波特率**：termios 支持标准速率 9600–921600；其它值启动报错（HWT101 用 115200，无影响）。
6. **诊断话题**：与 Python 版一致，imu_node 不发布 `/diagnostics`（唯一发布者仍是 H 的聚合节点），异常计数体现在日志与周期报告中。

## 与 CONTRACTS 的对接

- 发布 `/imu/data`，`frame_id=imu_link`，100 Hz，SensorData QoS、depth 10，满足 `CONTRACTS.md` §2 与 WP-B 的接口约定。
- 协方差由 `msgs.hpp` 的 `build_imu_data` 统一组装：`orientation_covariance` 对角为 `[tilt, tilt, yaw]`（yaw 模式下 roll/pitch 方差给 1.0 表示未知），`imu.yaml` 中 yaw 方差 0.0003（HWT101 0.1° 航向）。
- 参数名未变、未新增冻结参数；包类型由 ament_python 变更为 ament_cmake（编译期依赖 rclcpp/sensor_msgs/std_msgs/diagnostic_msgs）。
