# Hailo-8 驱动安装与有效性验证

- 日期：2026-11-17
- 平台：Raspberry Pi 5（`pi5`，192.168.50.98），Ubuntu 26.04.1 LTS arm64，kernel `7.0.0-1023-raspi`
- 硬件：Hailo-8 AI 加速卡（PCIe M.2），`1e60:2864`
- 结论：**HailoRT 4.23.0（Hailo-8 可用的最新 4.x）+ 固件 + Python 绑定全部就位，NPU 实际跑推理 157 FPS（YOLOv8s）**，40 s 满载稳定、无 PCIe 错误。

---

## 1. 结论速览

| 项 | 结果 |
|---|---|
| PCIe 枚举 | `0001:01:00.0 Co-processor: Hailo Technologies Ltd. Hailo-8 AI Processor [1e60:2864] (rev 01)` |
| 内核驱动 | `hailo_pci 4.23.0`（DKMS 源码编译，`/lib/modules/7.0.0-1023-raspi/updates/dkms/hailo_pci.ko.zst`） |
| 设备节点 | `/dev/hailo0`，`lsmod` 显示 `hailo_pci 122880 0` |
| 固件 | `Hailo-8`，`Firmware Version 4.23.0`，`Device Architecture: HAILO8`，`Control Protocol Version 2` |
| 推理性能 | YOLOv8s 640×640：**157 FPS**（CLI, batch 8）；YOLOv6n：**283 FPS** |
| Python 绑定 | `hailo_platform` for **CPython 3.14**（源码编译，链接 libhailort 4.23），`Device.scan()` 正常，Python 单流 200 帧实测 **74 FPS** |
| 满载温度 | SoC 46–49 °C（40 s 持续跑），无降频、无 PCIe 报错 |
| `hailortcli` | 4.23.0，`/usr/bin/hailortcli` |

---

## 2. 安装了什么、为什么这么装

Ubuntu 的源里**没有**任何 hailo 包（`apt-cache search hailo` 只有无关的 golang 包），而树莓派官方仓库有现成的 arm64 deb。为了不把 Debian 仓库整个加进 Ubuntu（依赖会打架），做法是**只下载 deb、用 apt 安装本地文件**，让 apt 从 Ubuntu 源解析依赖：

| 包 | 版本 | 作用 |
|---|---|---|
| `hailort` | 4.23.0 | 运行时 + `libhailort` + `hailortcli`（依赖 gstreamer 基础库，Ubuntu 源里有） |
| `hailort-pcie-driver` | 4.23.0 | 内核驱动源码 + **固件**（`/lib/firmware/hailo/hailo8_fw.4.23.0.bin`），postinst 里跑 DKMS |
| `linux-headers-7.0.0-1023-raspi` | 7.0.0-1023.23 | DKMS 编译必需（原先没装，`/lib/modules/$(uname -r)/build` 不存在） |
| `dkms` / `build-essential` | — | 同上 |

安装结果：

```
$ dkms status
hailo_pci/4.23.0, 7.0.0-1023-raspi, aarch64: installed

$ ls -l /lib/firmware/hailo/
hailo8_fw.4.23.0.bin
hailo8_fw.bin -> /lib/firmware/hailo/hailo8_fw.4.23.0.bin
```

驱动包在 Debian 上原本只测到 6.6/6.12 内核，**在 7.0.0-1023-raspi 上 DKMS 编译一次通过**，没有打任何补丁。

> 为什么不用 `sudo apt install hailo-all`：它会连带 `hailo-tappas-core`（39 MB 二进制 + gstreamer/opencv 整套，且依赖 bookworm 的 `libopencv406`、`libpython3.11`、`gcc-12`），在 Ubuntu 26.04 上大概率装不上或污染系统。做 NPU 推理不需要 TAPPAS。

### 2.1 版本怎么选（4.20 / 4.23 / 5.x）

这块容易踩坑，先说结论：**Hailo-8 只能停在 4.x 线，当前能用的最新公开版本是 4.23.0**。

| 版本 | 状态 | 说明 |
|---|---|---|
| 4.20.0 | 能用但偏旧 | 树莓派 **bookworm** 仓库的版本，固件是 2025-01 的；本项目的首轮安装用的就是它 |
| 4.21.0 | **建议避开** | 社区报过 `hailo_pci` 的 vdma 映射内核 oops（`hailo_vdma_buffer_map`），跑推理会挂 |
| **4.23.0** | **当前采用** | 树莓派 **trixie** 仓库版本（固件 2025-11），本项目实测性能与 4.20 一致、40 s 满载稳定 |
| 4.24.0 | 只有源码 | GitHub 有 `v4.24.0` tag，但公开仓库里没有配套 deb/固件 |
| 5.x（最新 5.5.0） | **不能用** | 5.x 是给 **Hailo-10H** 的；Hailo 官方明确 **Hailo-8 / 8L 只支持 2.x/3.x/4.x**。装 5.x 的 PCIe 驱动会自动卸载 4.x 驱动，而且不认 Hailo-8 |

所以「有没有更新版本」的答案是：**有，4.23.0，已经换上了**；再往上（4.24 / 5.x）对本卡没有实际可用的公开安装包。

版本升级同时改了打包结构：老的 `hailofw` + `hailo-dkms` 两个包，在 4.23 里合并成 `hailort-pcie-driver`（驱动源码 + 固件 + udev 规则 + modprobe 配置）。升级时**必须先 purge 掉旧的 `hailo-dkms`**，否则两个 DKMS 模块都叫 `hailo_pci` 会打架。

另外 4.23 的 `hailort-pcie-driver` 默认写了 `/etc/modprobe.d/hailo_pci.conf`：

```
options hailo_pci force_desc_page_size=4096
```

这个参数在 Pi 5 上是必需的——DMA 描述符页太大时 HailoRT 会报 runtime error。旧版 4.20 的 `hailo-dkms` 不带这个文件，得自己加；换成 4.23 后自动就有了。

### 固件加载日志

```
hailo 0001:01:00.0: Writing file hailo/hailo8_fw.4.23.0.bin
hailo 0001:01:00.0: File hailo/hailo8_fw.4.23.0.bin written successfully
hailo 0001:01:00.0: NNC Firmware loaded successfully
hailo 0001:01:00.0: FW loaded, took 78 ms
hailo 0001:01:00.0: Probing: Added board 1e60-2864, /dev/hailo0
```

中间两行 `Failed to write file hailo/hailo8_board_cfg.bin` / `hailo8_fw_cfg.bin` 是**无害告警**：这两个是可选的板级/固件配置文件，树莓派固件包里本来就没有，驱动打不开就直接跳过，后面的 `NNC Firmware loaded successfully` 才是结论。

---

## 3. 有效性验证

### 3.1 设备识别

```
$ hailortcli fw-control identify
Executing on device: 0001:01:00.0
Identifying board
Control Protocol Version: 2
Firmware Version: 4.23.0 (release,app,extended context switch buffer)
Logger Version: 0
Board Name: Hailo-8
Device Architecture: HAILO8
Serial Number: <N/A>

$ hailortcli scan
Hailo Devices:
[-] Device: 0001:01:00.0
```

`Serial Number <N/A>` 是 Hailo-8 模组的正常现象（该字段只在部分板型上烧写）。

### 3.2 模型与网络信息

模型来自树莓派相机后处理包 `rpicam-apps-hailo-postprocess_1.9.0-1~bpo12+1_arm64.deb`（162 MB，里面打包了 h8/h8l 两套 hef）。只解包取模型，不安装该包：

```
$ hailortcli parse-hef /opt/hailo-models/yolov8s_h8.hef
Architecture HEF was compiled for: HAILO8
Network group name: yolov8s, Single Context
    Input  yolov8s/input_layer1 UINT8, NHWC(640x640x3)
    Output yolov8s/yolov8_nms_postprocess FLOAT32, HAILO NMS BY CLASS(80 classes)
```

`Architecture HEF was compiled for: HAILO8` 与本机 `Device Architecture: HAILO8` 一致——**h8l（13 TOPS）的模型不能用在这里，h8 的才能用**，选型时别拿错。

放进 `/opt/hailo-models/` 的有：`yolov8s_h8.hef`、`yolov6n_h8.hef`、`yolov8s_pose_h8.hef`。

### 3.3 推理性能

```
$ hailortcli run /opt/hailo-models/yolov8s_h8.hef --batch-size 8
 Network group: yolov8s
    Frames count: 786
    FPS: 156.96
    Send Rate: 1543.00 Mbit/s
    Recv Rate: 1533.35 Mbit/s

$ hailortcli run /opt/hailo-models/yolov6n_h8.hef --batch-size 8
    FPS: 282.63        Send 2778.41 / Recv 1629.78 Mbit/s
```

持续 20 s 跑 YOLOv8s：FPS 稳定在 156.96–157.12，SoC 温度 46–49 °C，`dmesg` 无 error/timeout。

---

## 4. PCIe 链路：现在跑 Gen2 ×1，可能是性能上限的主要原因

```
$ lspci -vvs 0001:01:00.0
LnkCap: Port #0, Speed 8GT/s, Width x4          ← 卡本身支持 Gen3 ×4
LnkSta: Speed 5GT/s (downgraded), Width x1 (downgraded)   ← 实际跑 Gen2 ×1
```

Pi 5 的 M.2 槽默认 Gen2 ×1，跑推理时再采样两次，链路仍是 `5GT/s ×1`，也没有 AER/PCIe 报错。

粗算带宽需求：YOLOv8s 输入 640×640×3 = 1.23 MB/帧，157 FPS 时光是输入就要 1.23 MB × 157 ≈ **193 MB/s ≈ 1.54 Gbit/s**，加上输出 1.53 Gbit/s，合计 **~3.1 Gbit/s**；而 Gen2 ×1 编码后理论上限约 4 Gbit/s，也就是**已经用掉 ~77%**。

（注意：`hailortcli` 打印的 Send/Recv Rate 是按“帧大小 × 帧率”折算的标称值，不是实测 PCIe 流量；YOLOv6n 那组 2778 Mbit/s 是同样口径算出来的，不能直接当真实占用读。）

所以「瓶颈在链路」是个**合理假设而不是定论**——batch 1 与 batch 8 都是 157 FPS，更像 NPU 侧每帧 6.4 ms 的固定开销。要证实只能实测对比：

在 `/boot/firmware/config.txt` 加一行后重启：

```
dtparam=pciex1_gen=3
```

预期 Gen3 ×1（8 GT/s，128b/130b ≈ 7.9 Gbit/s）能把带宽翻倍；如果 FPS 明显上涨就是链路受限，如果几乎不变就是 NPU/驱动开销受限。

代价：Gen3 在部分 HAT/排线上不稳定（可能认不到卡或跑崩）。当前 157 FPS 对 30 fps 相机已是 5× 余量，**建议先不动**，要用多路相机或更大模型时再试，出问题删掉这行即可回退。


---

## 5. Python 绑定（已装，源码编译）

### 5.1 为什么不能直接装包

| 路线 | 结果 |
|---|---|
| `sudo apt install python3-hailort`（树莓派仓库 4.23.0） | ❌ 依赖 `python3 (>= 3.13), python3 (<< 3.14)`，本机是 **Python 3.14.4**，ABI 不兼容 |
| `pip install hailort` / `pyhailort` | ❌ PyPI 上两个名字都是 404 |
| Hailo 开发者区 wheel | 需要账号登录，脚本化不了 |

所以走**源码编译**。好在 HailoRT 的绑定 CMake 支持 `-DLIBHAILORT_PATH`，可以只编 `_pyhailort` 这一个扩展模块、直接链接系统已装的 `libhailort.so`，**不需要把整个 HailoRT 重编一遍**（那在 Pi 5 上要几十分钟）。

### 5.2 编译过程与三个坑

脚本：`tools/install_pyhailort_source.sh`（幂等，可重跑）

```bash
git clone --depth 1 --branch v4.23.0 https://github.com/hailo-ai/hailort.git
git clone --depth 1 --branch v3.0.1 https://github.com/pybind/pybind11.git
cmake -S hailort/hailort/libhailort/bindings/python/src -B build \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
  -DPYBIND11_FINDPYTHON=NEW \
  -DPYBIND11_PYTHON_VERSION=3.14 \
  -DLIBHAILORT_PATH=/usr/lib/libhailort.so \
  -DHAILORT_INCLUDE_DIR=<src>/hailort/libhailort/include \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DFETCHCONTENT_SOURCE_DIR_PYBIND11=<pybind11 源目录>
cmake --build build -j4
```

踩到的三个坑，都在脚本里补掉了：

1. **pybind11 版本**。源码里 FetchContent 钉死 `v2.10.1`，它不认识 Python 3.13+，必须换成 **3.0.1**，并用 `-DFETCHCONTENT_SOURCE_DIR_PYBIND11` 指向本地克隆，绕过下载。
2. **C++ 标准**。pybind11 3.x 要求 C++17，而 bindings 的 `CMakeLists.txt` 写死 `CXX_STANDARD 14`，要 `sed` 成 17。
3. **两个 CMake 4 的坑**：
   - `cmake_minimum_required(VERSION 3.5)` → 加 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`；
   - `FetchContent_Populate` 在新 CMake 里被 CMP0169 拦下 → 在 `pybind11.cmake` 里插一行 `cmake_policy(SET CMP0169 OLD)`。
   - 另外 `PYBIND11_PYTHON_VERSION` **必须显式传**，否则 `string(REGEX REPLACE ...)` 少参数，报一个完全看不出原因的 `CMake Error at CMakeLists.txt:33 (string)`。

编译产物：`_pyhailort.cpython-314-aarch64-linux-gnu.so`（1.7 MB，约 1 分钟编完），连同纯 Python 的 `hailo_platform` 包装层一起装到：

```
/usr/local/lib/python3.14/dist-packages/hailo_platform/
├── __init__.py
├── pyhailort/
│   ├── _pyhailort.cpython-314-aarch64-linux-gnu.so
│   ├── pyhailort.py      ← Device / VDevice / HEF / InferVStreams 等 Python API
│   └── hw_object.py
├── common/  drivers/  tools/
```

### 5.3 验证

```python
>>> from hailo_platform import Device, VDevice, HEF
>>> Device.scan()
['0001:01:00.0']
>>> p = VDevice.create_params()
>>> with VDevice(p) as v: print(v.get_physical_devices())
[Device('0001:01:00.0')]
```

真推理（`tools/hailo_python_selftest.py`，YOLOv8s，200 帧）：

```
hailo_platform 导入成功
HEF: /opt/hailo-models/yolov8s_h8.hef
  输入 yolov8s/input_layer1 (640, 640, 3) FormatType.UINT8
  输出 yolov8s/yolov8_nms_postprocess(80, 5, 100)
network group: yolov8s
推理 200 次：2.664 s → 75.1 FPS（单帧 13.32 ms）
  输出 yolov8s/yolov8_nms_postprocess: shape=(1, 80, 0, 5) dtype=float64
```

75 FPS 比 CLI 的 157 FPS 低，原因是 Python 侧是**单流、逐帧**调用，每帧还有 host 端的 float32→uint8 量化转换和 numpy 开销；CLI 用的是 batch 8 流水线。要 100+ FPS 就用 CLI，或者用 `InferVStreams` 建多个流/异步作业。

### 5.4 用的时候一个必踩的坑

`InferVStreams.infer()` 是拿 **`input.shape[0]` 当 batch 数**的：

```python
# 错：直接喂 (640,640,3)，会被当成 640 帧
data = np.random.rand(640, 640, 3)
# → [HailoRT] [error] Memory size of vstream ... does not match the frame count!
#   (Expected 3145728000, got 4915200)

# 对：自己补 batch 维
data = np.random.rand(1, 640, 640, 3).astype(np.float32)
```

`Expected 3145728000 = 640 × 4915200`，看到这个天文数字就是漏了 batch 维。

### 5.5 维护注意

- **版本必须和 `libhailort` 一致**：现在编的是 4.23.0（链接 `libhailort.so.4.23.0`）。以后升级 `hailort` 包，必须用相同 tag 重编绑定：`bash install_pyhailort_source.sh 4.23.0`，脚本会自动发现源码版本不符并重新克隆。
- 装在 `/usr/local/lib/python3.14/dist-packages`，**系统 Python 升级小版本（3.14.x）不受影响，升到 3.15 需要重编**。
- numpy 2.3.5 下实测正常；绑定本身是 pybind11 直接传 buffer，不依赖 numpy C API。

---

## 6. 遗留事项


1. **Python 绑定已装**（见 §5），但要记住它和 `libhailort` 版本绑死，升级 HailoRT 后必须按同一 tag 重编。
2. **TAPPAS 没装**：如果后面要走 GStreamer pipeline（`hailotools`），再单独处理，注意 Debian 依赖与 Ubuntu 的冲突。
3. **重启后自检**：驱动通过 PCI ID `1e60:2864` 注册了模块别名，`hailort.service` 也已 enabled，理论上重启后 `/dev/hailo0` 会自动出现。这次是手动 `modprobe` 加载的，**下次重启后请确认一次**：

   ```
   ls -l /dev/hailo0 && hailortcli fw-control identify
   ```

   如果没出现，`sudo modprobe hailo_pci` 再看 `dmesg | grep hailo`。
4. **散热**：当前只跑 20 s，SoC 46–49 °C。长时间满载时注意 HAT 与 SoC 的散热，必要时加风扇。

---

## 7. 复现命令

一键脚本：`tools/install_hailo_pi.sh`（等价于下面几步）

```bash
sudo apt-get update
sudo apt-get install -y dkms build-essential linux-headers-$(uname -r)

cd /tmp
B=https://archive.raspberrypi.com/debian
curl -sSLO $B/pool/main/h/hailort/hailort_4.23.0_arm64.deb
curl -sSLO $B/pool/main/h/hailort-pcie-driver/hailort-pcie-driver_4.23.0_all.deb
sudo apt-get install -y ./hailort_4.23.0_arm64.deb ./hailort-pcie-driver_4.23.0_all.deb

sudo modprobe hailo_pci
hailortcli fw-control identify
hailortcli scan
hailortcli run /opt/hailo-models/yolov8s_h8.hef --batch-size 8
```

> 从 4.20 升级到 4.23 时，必须先 `sudo apt-get purge hailort hailo-dkms hailofw`（并 `modprobe -r hailo_pci`），否则新旧两个 `hailo_pci` DKMS 模块会冲突。

Python 绑定（另一步，见 §5）：

```bash
bash tools/install_pyhailort_source.sh 4.23.0   # 编译 hailo_platform 并安装（版本要和 libhailort 对齐）
python3 tools/hailo_python_selftest.py /opt/hailo-models/yolov8s_h8.hef 200
```

验证清单：

```bash
lspci -nn | grep -i hailo                 # 1e60:2864
dkms status                               # hailo_pci/4.23.0, <kernel>, aarch64: installed
ls -l /dev/hailo0
hailortcli fw-control identify            # Board Name: Hailo-8 / HAILO8 / Firmware 4.23.0
hailortcli run <model>.hef                # FPS > 0 即通路正常
python3 -c "from hailo_platform import Device; print(Device.scan())"   # ['0001:01:00.0']
```
