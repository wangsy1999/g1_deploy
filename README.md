# g1_deploy — G1 DreamWaQ 机载部署

新增独立的 **RealSense RGB/深度网页测试**与 **Parkour 双网络推理**（MuJoCo 相机 / 实机 RealSense 切换）。
入口为 `apps/realsense/realsense.sh`、`apps/parkour/parkour.sh`，安装后也可用 `./g1 realsense`、`./g1 parkour`。
完整接线、依赖、启动、空跑和标定要求见 [Parkour 与 RealSense 说明](docs/PARKOUR.md)。原 DreamWaQ 程序独立保留。

独立的G1 29DOF DreamWaQ C++推理项目，纯粹的vibe coding by codex。直接读取Unitree DDS低层状态并下发PR关节目标。
包含模型、部署配置、原配遥控器/F710输入、离线检查、网页监控和机载安装入口。
C++控制程序不依赖MuJoCo、ROS或Python；可选网页监控需要Python与官方SDK2 Python依赖，源码已包含在本仓库。

机载部署、输入及空跑验证记录见 [改动总结](CHANGELOG.md)。操作者已确认现场调试通过；
自动化测试、现场调试与具体行走工况的验收范围需分别确认。

## 可运行程序与使用方法

运行环境为 Linux；`./g1 ...` 在源码仓库根目录和完成 CMake 安装后的 `install/` 目录均可执行，使用相同命令。
先按下文“在G1开发计算机上编译”准备 C++ 依赖并构建。示例中的 `eth0`、机器人 IP、模型路径和外部仿真仓库路径均须按实际环境替换。
实机 DDS 使用域 `0` 和连接控制板的网卡，仿真使用正数域和 `lo`；两端域号必须相同。

| 程序 / 源码入口 | 功能 | 常用启动方式 |
| --- | --- | --- |
| `g1_dwaq` / `apps/dwaq/src/main.cpp` | DreamWaQ 行走策略、姿态复位、硬件只读检查和手柄诊断 | `./g1 dwaq check`、`./g1 dwaq sim`、`./g1 dwaq real eth0` |
| `g1_parkour` / `apps/parkour/src/main.cpp` | 深度编码器与动作网络双 ONNX 推理，支持 MuJoCo 或实机深度输入 | `./g1 parkour check`、`./g1 parkour sim`、`./g1 parkour real eth0` |
| `g1_hybrid` / `apps/hybrid/src/main.cpp` | 站立、后退、原地转向使用 DWaQ，持续前进时切换到 Parkour | `./g1 hybrid check`、`./g1 hybrid sim`、`./g1 hybrid real eth0` |
| `apps/monitor/joint_monitor.py` | 只读关节、IMU、实际/目标曲线及空跑目标网页 | `bash apps/monitor/monitor.sh eth0 0.0.0.0 8765` |
| `apps/realsense/realsense_web.py` | RealSense 原始/策略深度预览、可选 RGB、独立相机预览 | `bash apps/realsense/realsense.sh --host 0.0.0.0 --port 8766` |
| `apps/realsense/src/g1_depth_camera.cpp` | C++ 原生 USB 深度采集，进程内缓存；由 Parkour/Hybrid 实机模式自动启动 | `./build/g1_camera_check` 可单独只读验证 |
| `apps/simulation/sim.py` | 启动外部 MuJoCo 查看器，并加载本项目 Parkour 配置与相机桥接 | `python3 apps/simulation/sim.py --mujoco-root /path/to/unitree_mujoco` |

根目录 `g1` 是控制器的统一入口，安装后仍为 `g1`；`apps/dwaq/g1.sh` 作为旧入口转发到它。`monitor.sh`、`realsense.sh`、`parkour.sh` 同时支持源码和安装目录，安装后分别命名为 `monitor`、`realsense`、`parkour`。仿真启动器 `apps/simulation/sim.py` 不随安装包安装，需从源码运行；旧入口 `apps/parkour/parkour_sim.py` 继续兼容。

控制器统一采用 `./g1 <程序> <模式> [参数]`：

| 模式 | DWaQ | Parkour | Hybrid |
| --- | --- | --- | --- |
| 离线检查 | `./g1 dwaq check` | `./g1 parkour check` | `./g1 hybrid check` |
| 仿真控制 | `./g1 dwaq sim` | `./g1 parkour sim` | `./g1 hybrid sim` |
| 实机控制 | `./g1 dwaq real eth0` | `./g1 parkour real eth0` | `./g1 hybrid real eth0` |

公共参数放在模式后，例如 `./g1 dwaq sim --domain 2`。默认读取各自 JSON 中的模型路径；Parkour 的 `--port` 和自动相机管理按其深度输入需求保留。`./g1 run eth0`、`./g1 check` 等旧命令仍兼容。

### DreamWaQ：检查、空跑与行走

```bash
# 以下在源码根目录或 install/ 目录执行
./g1 interfaces                     # 列出网卡和到控制板的路由
./g1 dwaq check                     # 离线加载模型并推理一次，不连接 DDS
./g1 joystick /dev/input/js0         # 只检查 F710 输入，不连接 DDS
./g1 read-only eth0                 # 只读 LowState、CRC、机型、电机和 IMU 检查后退出
./g1 dwaq real eth0                 # 默认原配遥控器，启动时为空跑
```

键盘空跑：`p → 等待 Target interpolation complete → r`；真实输出：`9 → p → 等待 Holding default pose → r`。
仿真键盘使用 `w/s` 调整前后速度、`a/d` 调整横向速度、`q/e` 调整转向、`x` 清零；实机使用遥控器摇杆控制速度，空格或 Ctrl+C 停止。
两套程序实机均默认启用原配遥控器，对应 `Start` 接管、`X` 复位、`A` 策略、`Select` 停止，摇杆直接控制速度；F710 对应 `Start` 接管、`A` 复位、`Y` 策略、仅仿真使用，直接用摇杆运动、`X` 清零、`B` 停止。
接管会清空空跑目标，必须重新复位再进入策略。具体状态要求和保护见下文与 [硬件说明](docs/HARDWARE.md)。

也可直接运行构建产物，两套程序均使用命名参数：

```bash
./build/g1_dwaq --check-policy --config apps/dwaq/config/g1_locomotion.json
./build/g1_dwaq --hardware-read-only --domain 0 --interface eth0 --config apps/dwaq/config/g1_locomotion.json
./build/g1_dwaq --hardware --domain 0 --interface eth0 --config apps/dwaq/config/g1_locomotion.json
# 外部 DreamWaQ 仿真器已启动时：
./build/g1_dwaq --sim --domain 1 --interface lo --config apps/dwaq/config/g1_locomotion.json
```

### DWaQ 与 Parkour 共用 MuJoCo 仿真

两套控制器使用同一套 Python 环境、G1 29DOF 场景及 SDK2 DDS 接口。先从仓库根目录启动一次：

```bash
python3 apps/simulation/sim.py --mujoco-root /path/to/unitree_mujoco --domain 1
```

另开终端，从以下控制器中选择一个：

```bash
# DWaQ 只读取 DDS，忽略相机观测
./g1 dwaq sim --domain 1
# Parkour 同时读取 DDS 和深度观测
./g1 parkour sim --domain 1
```

一次只运行一个控制器；切换策略时先停止当前控制器。入口默认发布 Parkour 深度观测，两套程序都可使用；仅运行 DWaQ 时可加 `--no-depth`，避免启动相机采样。仿真入口只修改当前 Python 进程配置，不改外部仓库文件。两套控制器均使用 `9 → p → 等待复位完成 → r` 控制仿真机器人。

启动仿真时加 `--depth-preview`，可在 MuJoCo 窗口右上角显示深度预览：左侧为原始 36×64 深度图，右侧为裁剪、模糊、归一化后的 18×32 模型输入，白框标记原始图的裁剪区域。预览会随窗口缩放，最多以 15 Hz 更新，复用已采样深度，不额外采样。需要支持 `Handle.set_images` 的 MuJoCo（当前环境 3.15.0 支持）及 OpenCV 图像处理功能，无需 OpenCV GUI；不能与 `--no-depth` 同用。省略该开关可关闭预览。

加 `--follow` 可让主视角跟随机器人躯干位置，初始距离为 3 米；仍可用鼠标旋转和缩放。可与 `--depth-preview` 或 `--no-depth` 配合使用，不影响策略使用的深度相机。

使用 `--terrain mixed` 启动新增的综合地形，或单独选择一类：

| 参数 | 地形 |
| --- | --- |
| `--terrain platforms` | 10、20、30、40、50、60 cm 高台 |
| `--terrain stones` | 高 10–35 cm、直径 42–52 cm 的交错梅花桩 |
| `--terrain gaps` | 宽 20、35、50、65、80、100 cm 的沟槽，落地区长 1.8 m |
| `--terrain stairs` | 台阶高度 8、12、16 cm 的三组上下楼梯 |
| `--terrain hurdles` | 高 10、20、30、40 cm 的窄横栏 |
| `--terrain mixed` | 五类并排独立通道，均沿 +X 前进 |
| `--terrain original` | 外部仓库的原场景（默认） |

```bash
python apps/simulation/sim.py --mujoco-root /path/to/unitree_mujoco \
  --terrain mixed --follow --depth-preview
```

综合场景的五类地形并排摆放，均从 X=3 m 开始：高台中心 Y=0、梅花桩 Y=5、gap Y=-5、楼梯 Y=10、横栏 Y=-10（单位米）。前方是连通各通道的起点广场，可先转向走到目标通道，再沿 +X 测试。通道宽 4 m，中心间距 5 m（`lane_spacing`）；沟槽与桩间底面低于行走面 1 m，未保留隐藏的上层平面。尺寸可在 [terrain.json](apps/simulation/terrain.json) 中调整。启动时打印各通道的 X 范围和 Y 中心坐标；地形临时生成，退出后清理，不修改外部机器人模型和场景文件。两套控制器共用这些地形，能否通过需按模型实际表现验证。

窗口后端默认使用 `x11`（Wayland 会话下通过 XWayland），启动时无需额外指定，避免 libdecor 插件加载失败造成标题栏缺失。可用 `--window-backend wayland` 显式切换，或用 `--window-backend auto` 根据环境选择；auto 会保留已有 `PYGLFW_LIBRARY_VARIANT` 设置，否则在有 `DISPLAY` 时优先使用 X11。需要重启仿真才能生效。

### Parkour：离线推理、仿真和实机

```bash
# 源码根目录；仅执行双网络推理检查，无需相机或 DDS
./g1 parkour check

# 仿真终端一：外部仓库需含 simulate_python/parkour_observation.py
python3 apps/simulation/sim.py --mujoco-root /path/to/unitree_mujoco --domain 1
# 仿真终端二：域号与终端一一致；观测共享内存自动连接
./g1 parkour sim --domain 1

# 实机：自动启动/复用相机，等新帧就绪后启动运控，默认空跑
./g1 parkour real eth0
```

安装后分别使用 `./g1 parkour check`、`./g1 parkour sim ...`、`./g1 parkour real eth0 ...`。
仿真 Python 环境需外部仓库及其 MuJoCo、NumPy、pygame、`unitree_sdk2py` 依赖；本仓库不包含仿真器、XML 和网格。
仿真键盘使用 `p` 复位、`r` 策略、`9` 使能输出、`w/s` 前向速度、`q/e` 转向、`x` 清零、空格/Ctrl+C 停止；可加 `--joystick /dev/input/js0` 使用 F710。
实机使用原配遥控器 `X → 等待 → A` 空跑；完成相机安装、torso IMU 轴向、关节方向和硬件参数核对后，`Start → X → 等待复位 → A` 进入真实输出，`Select` 停止。实机还需要 `rt/secondary_imu` 的 torso IMU。

常用选项：`--config` 选择策略 JSON。实机直接在 C++ 进程内采集深度，仿真观测自动通过本机共享内存连接，无需端口或 channel 参数。模型路径、`reset_seconds`（默认 3 秒）和 `hardware` 参数统一从各程序自己的 JSON 中读取。
直接运行示例为 `./build/g1_parkour --sim --domain 1 --interface lo`；直接实机入口为 `./build/g1_parkour --hardware --domain 0 --interface eth0`，相机由 C++ 自动启动。`./build/g1_parkour --help` 查看参数。

实机入口直接启动 C++ 控制器，不启动 Python 相机进程。先退出独立相机预览，避免重复占用 USB 设备。相机深度为 480×270 Z16、60 Hz；预处理仍在 50 Hz 推理线程完成。

只读验证相机，不初始化 DDS 或启动运控：

```bash
./build/g1_camera_check
```

构建时自动检测 librealsense2 C++ SDK；机器人构建应加 `-DG1_REQUIRE_REALSENSE=ON`，SDK 在私有目录时通过 `CMAKE_PREFIX_PATH` 或 `realsense2_DIR` 指定。仅安装 `pyrealsense2` 不提供 C++ 开发依赖。没有 SDK 的工作站仍可构建仿真与离线检查，实机启动会明确报错。

### 混合策略：DWaQ 站立/后退，Parkour 前进

```bash
./g1 hybrid check                       # 离线验证两种模型及双向切换
./g1 hybrid sim --joystick /dev/input/js0 # 复用已启动的仿真，需保留深度观测
./g1 hybrid real eth0                   # 真机默认原配遥控器，自动管理相机
```

启动后沿用空跑与接管流程；复位目标采用 DWaQ 默认姿态。F710 使用 `Start → X → 等待复位 → A`，`B` 停止；真机原配遥控器使用相同接管/复位/策略键，`Select` 停止。键盘为 `9 → p → 等待 → r`，`w/s` 增减前后速度，`x` 清零，空格停止。

策略根据期望前进速度选择：默认使用 DWaQ；持续达到 0.15 m/s 满 0.2 秒后进入 Parkour；速度降到 0.05 m/s 或后退时选择 DWaQ。两阈值间保留当前模式，避免摇杆抖动反复切换。切换后的第一个推理结果直接使用新策略的目标角度、Kp 和 Kd，不做平滑混合。每个 20 ms 推理周期依次运行两套策略，持续更新双方的观测和动作历史，只发布当前选中策略的输出；模式切换不清空历史。

配置在 [g1_hybrid.json](apps/hybrid/config/g1_hybrid.json)：`policies` 指向两份策略 JSON（相对混合配置目录解析），`switching` 设置切换阈值和前推确认时长，`reset_seconds` 与 `hardware` 是混合程序自己的运行参数。两份模型仍使用各自关节映射、归一化和 PD 增益；DWaQ 使用 LowState 的 pelvis IMU，Parkour 使用原来的 torso/深度观测。

启动时，在初始化 DDS 和创建控制输出端之前，用虚拟站姿和深度图分别运行三个 ONNX 网络 `warmup_steps` 次（默认 10，必须为正整数）。预热结果全部丢弃，完成后清空两个策略的观测/动作历史和模式选择状态，正式从 DWaQ 开始。策略启用后，两套网络使用各自所需的最新实测状态持续推理，保留各自预测的上一次动作；未选中策略的预测不会发送给机器人。后退时 DWaQ 使用负前进速度，Parkour 使用零前进速度。复位和重新启动策略仍会清空双方历史。三个网络的总推理耗时继续受现有 20 ms 周期和结果超时检查约束。

混合程序共用 Parkour 的 DDS、输入、监控、复位和停止循环，同一时刻只有一个控制输出端。站立/后退时仍要求相机/观测链路在线，数据过期会停止，不自动降级。已完成一次无窗口平地物理仿真：站立 5 秒、后退 3 秒、站立 3 秒、前进 3 秒、回到站立 3 秒，双策略持续推理、直接切换版本的最低躯干高度约 0.803 m，最大倾角约 0.097 rad，未检测到跌倒。该结果只覆盖这一段平地、给定初始姿态和指令；障碍物、交互操作及真机切换尚未完成行走验收。

### RealSense：查看相机与转发策略深度

独立网页预览的 Python 依赖为 NumPy、OpenCV 和与目标设备匹配的 `pyrealsense2`，版本范围见 `apps/realsense/requirements.txt`；需要已可工作的 RealSense SDK/USB 相机。
启动脚本支持 `PYTHON_BIN=/path/to/python3` 指定解释器。

```bash
# 独立深度网页预览；启动实机控制前须退出；默认 480×270、60 Hz、Instinct 预处理，RGB 默认关闭
bash apps/realsense/realsense.sh --host 0.0.0.0 --port 8766
# 开启可选 RGB 预览（独立查看用途）
bash apps/realsense/realsense.sh --host 0.0.0.0 --port 8766 --color
# 无相机时用合成画面检查网页；合成图像不会共享给策略
bash apps/realsense/realsense.sh --mock --host 127.0.0.1 --port 8766
```

浏览器打开 `http://机器人IP:8766/`，查看原始深度、32×18 策略深度、帧龄和相机状态；`--color` 额外启用 RGB。

真实相机按 Instinct onboard 的方式由 `wait_for_frames()` 到帧驱动，不再额外 sleep 限频；仅 mock 模式使用软件限频。每 500 个帧间隔统计一次实际采集频率（排除启动等待），输出到日志和状态字段 `capture_fps`，网页分别显示目标频率与实测频率，首次统计前显示“统计中”。

C++ 实机采集线程只负责 USB 取帧、将 Z16 按设备 depth scale 转为米制 float32 图并更新进程内最新帧缓存；不运行相机滤波或图像预处理。缩放、裁剪、补洞及归一化仍在 50 Hz 推理线程完成，高斯模糊默认关闭。策略从自身配置文件同目录的 `realsense_parkour.json` 读取预处理参数。相机设备时钟映射到主机单调时钟，延迟帧不会被重打时间戳续期；断流、设备重启或采集异常会经缓存传递给控制器。控制循环不等待 USB，退出时采集线程停止并释放相机。

实机链路为 RealSense USB → C++ 采集线程 → 进程内最新帧缓存 → 推理线程，不使用 Python、UDP、HTTP 或跨进程共享内存传递策略深度。MuJoCo 同步状态与深度仍通过 `/dev/shm/g1-observation-用户UID` 共享内存自动传输，仿真与控制器须同机同用户，每个用户仅运行一个仿真发布器。独立网页继续使用 HTTP/MJPEG，仅供控制器退出后的相机预览，不能同时打开同一台设备。
策略后处理和双网络推理在独立线程中按 20 ms 固定周期调度，控制线程按 2 ms 周期持续发送最新目标；输入与输出通过短时互斥锁交换快照，推理期间不持锁。策略线程独占历史缓存和复位操作；复位、接管和重新进入策略时清空目标并使上一轮在途结果失效。首个结果到达前保持站姿目标，首帧等待、结果年龄或结果所用输入快照年龄达到 60 ms 即停止；线程异常同样触发现有停止流程。真实输出时仍要求单次后处理与推理不超过 20 ms。仿真只对新的同步观测更新历史。两个循环错过周期时跳过该周期，避免集中补发；实际频率仍受系统调度和 DDS 发送耗时影响。

`--camera-mode legacy` 保留旧处理方式；仿真仍使用原观测协议与预处理，详细差异见 [Parkour 说明](docs/PARKOUR.md) 开头的相机链路对齐章节。
多相机用 `--serial 序列号` 选择；`--profile` 指定预处理配置，`--width/--height/--fps` 选择设备支持的深度模式。
默认仅监听 `127.0.0.1`；`--token 密码` 可开启 HTTP Basic 认证，用户名为 `camera`。
按 Ctrl+C 退出。此服务不连接 DDS、不下发电机命令。
安装后使用 `./g1 realsense ...`；直接 Python 入口为 `python3 apps/realsense/realsense_web.py ...`，参数相同，`--help` 查看完整选项。

### 关节监控：实机与仿真网页

按 [网页部署说明](docs/WEB.md) 准备 `apps/monitor/requirements.txt` 和官方 `unitree_sdk2py`。

```bash
# 源码入口，参数依次为网卡、监听地址、网页端口；固定为实机 DDS 域 0
bash apps/monitor/monitor.sh eth0 0.0.0.0 8765
# 仿真监控：直接 Python 入口可指定仿真域
python3 apps/monitor/joint_monitor.py --domain 1 --interface lo --host 127.0.0.1 --port 8765 --rate 50
# 实机直接 Python 入口
python3 apps/monitor/joint_monitor.py --domain 0 --interface eth0 --host 0.0.0.0 --port 8765 --rate 50
```

浏览器打开 `http://机器人IP:8765/`（本机仿真用 `http://127.0.0.1:8765/`）。网页显示 29 关节状态、IMU、真实命令及空跑目标，过期数据显示离线；网页不能发送运动命令，Ctrl+C 结束服务。
安装后使用 `./g1 monitor eth0 0.0.0.0 8765`。包装脚本省略监听地址和端口时，默认 `127.0.0.1:8765`。

## 快速使用：原配遥控器与网页

以下命令在机器人开发计算机上执行。本次设备地址为`192.168.123.164`，
连接控制板的网卡为`enP8p1s0`；更换设备时先用`ip route get 192.168.123.161`确认实际网卡。
首次部署先完成下文的编译安装，并按 [网页说明](docs/WEB.md)准备可选Python依赖。

电脑终端连接机器人：

```bash
ssh unitree@192.168.123.164
cd /home/unitree/g1_deploy/install
./g1 read-only enP8p1s0
```

只读检查通过后启动原配遥控器入口：

```bash
./g1 run enP8p1s0
```

| 场景 | 操作顺序 |
| --- | --- |
| 空跑预览 | **不按Start/9**，X → 等待`Target interpolation complete` → 摇杆回中 → A |
| 真实控制 | Start → X → 等待`Holding default pose` → 摇杆回中 → A |
| 行走速度 | 左摇杆前后/左右控制前后/横移，右摇杆左右控制转向；不需要按住R1 |
| 停止退出 | Select，或键盘空格/Ctrl+C |

程序启动时输出关闭；空跑目标不会发给电机。Start是软件接管，不是给机器人接通电源。
空跑后按Start会清除预览与策略历史，必须重新按X复位。
Select在已接管时先发送约0.5秒阻尼再退出，不是硬件断电；退出不会自动恢复官方运动模式。

另开终端，在同一个安装目录启动只读网页：

```bash
cd /home/unitree/g1_deploy/install
./g1 monitor enP8p1s0 192.168.123.164 8765
```

电脑浏览器打开 [关节监控](http://192.168.123.164:8765/)。
网页显示29个关节、IMU和实际/目标曲线；空跑目标标为`OFF · 空跑`，网页不能发送运动控制指令。
若端口已被正在运行的监控占用，直接打开现有页面即可。

## 目录

```text
apps/
  dwaq/          DreamWaQ：src/、include/、config/、models/、g1.sh 统一入口
  parkour/       Parkour：src/、include/、config/、models/、启动与仿真脚本
  realsense/     相机服务、静态网页、Python 依赖和相机验证工具
  monitor/       关节监控、静态网页、Python 依赖和启动脚本
  simulation/    DWaQ 与 Parkour 共用的 MuJoCo 启动入口
common/
  include/       共用键盘/信号、服务释放、复位/阻尼、硬件检查、策略线程和 DDS 通道
scripts/         整个项目的机器人环境配置工具
docs/            硬件、Parkour、网页与部署说明
cmake/           构建信息模板
```

两个 C++ 程序使用相同的入口分工：`src/main.cpp` 负责参数解析、帮助和异常出口，`src/options.cpp` 负责命令行选项及校验，`src/controller.cpp` 负责离线检查、初始化和控制循环。对应 `include/g1_*_app.h` 声明选项与运行接口。

运行控制由 `DwaqController` 和 `ParkourController` 管理：`Run()` 只组织读取状态/观测、处理输入、更新策略、发布目标和等待下一周期。接管、复位、策略进入、阻尼退出分别在独立方法中实现。会话数据归控制器所有，策略线程在目标与观测缓冲区销毁前结束。姿态接近检查、输入回中检查、复位目标生成及位置/PD 指令填充统一使用 `g1_control_targets.h`，输入和增益在填充边界均使用 DDS 关节顺序。

两套程序统一等待有效启动数据最多 10 秒，且优先处理停止，其次处理遥控器事件，最后处理键盘事件。输出已经使能时忽略重复接管，不会重新释放运动服务或清空正在运行的策略。

推理工具集中在 `common/include/`：`g1_config_utils.h` 读取配置数组与解析相对路径，`g1_onnx_utils.h` 校验和执行 ONNX 网络，`g1_observation_history.h` 更新历史缓存并按观测项排列输入。两套策略仅保留各自的观测、网络组合和动作转换；策略线程统一使用 `g1_inference::PolicyWorker`。

控制器还共用 `g1_loop_scheduler.h`（周期调度与跳过错过周期）、`g1_dds_state_cache.h`（DDS 订阅、带时间戳快照及故障锁存）、`g1_monitor_publisher.h`（使能状态、实际目标和空跑预览）、`g1_joint_mapping.h`（关节名称映射、编号校验与顺序转换）。状态验证规则和超时参数仍由各自程序提供。

程序专属文件放在对应的 `apps/<程序>/` 内；跨程序共用文件放在 `common/`，避免复制多份。
DWaQ 的策略与硬件保护参数位于 `apps/dwaq/config/g1_locomotion.json`；Parkour 的策略与硬件保护参数位于 `apps/parkour/config/g1_parkour.json`。两套程序分别维护自己的配置。
DWaQ 的 `policy_path`、Parkour 的 `models.depth_encoder` 与 `models.actor` 在各自 JSON 中指定模型文件。相对路径以 JSON 所在目录为基准，默认指向 `../models/`；绝对路径也可使用。需要更换模型时修改 JSON，或通过 `--config` 选择另一份配置。DWaQ 离线检查格式为 `--check-policy [--config 配置.json]`。两份策略 JSON 的顶层 `reset_seconds` 指定复位插值时长，必须为有限正数；旧配置未提供时默认使用 3 秒。
安装包内的资源也按 `apps/` 归类，根目录继续提供 `g1`、`parkour`、`realsense` 和 `monitor` 启动入口。
旧安装目录可能保留旧版文件；建议安装到新目录后再切换使用。

模型和配置来源及SHA256见 [ORIGIN.md](ORIGIN.md)。
网络接口为float32 `observation [1,96]`、`observation_history [1,480]` → `actions [1,29]`。当前 DWaQ 模型来自训练运行 `2026-11-13_10-08-23`，配置 `history_order: "time"`，五帧历史按完整帧从旧到新排列；兼容旧配置的 `"term"` 排列（未设置时默认使用）。Hybrid 自动使用同一份 DWaQ 模型和参数。
DWaQ 在独立策略线程中按配置的 `step_dt` 推理（默认20ms/50Hz），控制线程按2ms/500Hz调度并持续下发同步缓存中的最新关节目标，推理期间不持有缓存锁。策略使用LowState主IMU的pelvis机体系角速度和重力方向。首个结果到达前保持默认站姿；首帧等待、目标年龄或目标使用的输入快照年龄达到三个策略周期（默认60ms）即走停止流程。真实输出时保留单次推理不得超过策略周期的检查。复位、接管和重新进入策略时清空旧目标、丢弃在途结果；停止时先执行阻尼下发，再等待策略线程退出。实际下发频率仍受系统调度及DDS发送耗时影响。 DWaQ 和 Parkour 的运行日志通过有界后台队列输出，队列满或锁忙时丢弃日志；后台使用独立的非阻塞输出描述符，慢终端不阻塞控制线程。实机接管在确认官方服务退出后立即发送阻尼命令，移除原有200/250ms固定静默等待；LowCmd监测按DDS发布者身份排除自身，并在后续控制循环持续检查竞争发布者。

## 网页监控

```bash
# 仓库根目录运行；先按 docs/WEB.md 准备可选Python依赖
bash apps/monitor/monitor.sh enP8p1s0 192.168.123.164 8765
# cmake安装后也可在install目录执行：./g1 monitor enP8p1s0 192.168.123.164 8765
```

浏览器访问`http://192.168.123.164:8765/`。网页只读，不会启动控制程序。
完整说明见 [网页监控部署](docs/WEB.md)。

## 一键配置当前 ARM64 机器人

先**手动复制**本项目与 `.deploy-cache/robot-arm64/` 到机器人，再在项目根目录运行：

```bash
bash scripts/setup_robot.sh
```

该入口离线配置项目独立的 Python、SDK、ONNX Runtime 与相机专用 glibc 2.35，然后编译和安装。
不执行文件同步，不替换系统 glibc，不启动控制程序。支持 `--verify-camera` 只读相机验证。
详细用法、缺失系统包的处理和离线缓存说明见 [机器人一键配置](docs/ROBOT_SETUP.md)。

## 在G1开发计算机上编译

使用开发计算单元PC2，保留运动控制计算机PC1的厂商系统。
Unitree官方网络说明中PC1为`192.168.123.161`、PC2为`192.168.123.164`；实际地址和网卡以交付设备为准，
参考 [官方网络说明](https://github.com/unitreerobotics/xr_teleoperate/wiki/CycloneDDS-and-UnitreeSDK-%28en%29)。

依赖：Linux、CMake>=3.16、C++17、Unitree SDK2、Boost开发头文件、yaml-cpp、CPU版ONNX Runtime C/C++发行包。
SDK2及其CycloneDDS运行库按 [官方SDK安装说明](https://github.com/unitreerobotics/unitree_sdk2) 安装到机载系统。
SDK和ORT必须与机载 `uname -m` 对应：`aarch64` 使用ARM64版，`x86_64` 使用x64版。

将本仓库复制到机载开发计算机，在仓库根目录执行，替换ORT安装路径：

```bash
uname -m
export ONNXRUNTIME_ROOT=/path/to/native/onnxruntime
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j4
cmake --install build
```

安装目录包含`g1`和`monitor`启动脚本、`bin/g1_dwaq`、`apps/`和`lib/`。
ORT随包安装，SDK2/CycloneDDS、yaml-cpp及系统运行库仍由机载环境提供。
安装时固定SDK对应的运行库目录，避免已加载ROS环境时混用DDS库。

```bash
ldd install/bin/g1_dwaq  # 不应有not found
cd install
./g1 check             # 离线网络检查，不初始化DDS
```

整个安装目录可在同一台机器上移动，模型与配置按启动脚本所在位置加载。
SDK仍需位于构建时选定的安装路径。不应将本机x86二进制直接复制到ARM或不兼容的系统。
需要生成当前架构的安装压缩包时：

```bash
cd build
cpack -G TGZ
```

## 网络和只读状态检查

在安装目录执行：

```bash
./g1 interfaces
```

根据`ip route get 192.168.123.161`输出确认连接运动控制板的网卡。
例如`dev eth0 src 192.168.123.164`对应网卡`eth0`；后续示例需替换成实际网卡，不使用`lo`。

```bash
./g1 read-only eth0
```

只读模式不初始化命令发布器、不加载策略、不释放原运动服务。
检查LowState CRC、29DOF机型约束、电机及主IMU，成功后退出。
当前要求`mode_machine=5`，不匹配时先核对硬件型号、腰部开放状态和固件，不修改常量绕过检查。

## 控制运行

### 不按 Start / 9 即为空跑

```bash
cd install
./g1 run enP8p1s0
```

程序默认空跑，读取真实 LowState；不按 `9`，直接按 `p` 预览复位，等待 `Target interpolation complete` 后按 `r` 执行策略推理。
实机速度由原配遥控器摇杆控制；仿真使用 `w/s`、`a/d`、`q/e` 和 `x` 调整或清零速度；空格或 Ctrl+C 退出。
按下 `9` 才申请接管并打开真实输出。在此之前不创建 `rt/lowcmd` 发布器、
不调用 MotionSwitcher、退出时也不发阻尼指令。预览目标使用独立的 `rt/g1/dry_run_target` 遥测话题，
网页标为“空跑”，本程序 Enable 显示 OFF。空跑不改变机器人电源、官方服务或其他程序的输出状态。
按 `9` 时先严格检查真实状态；通过后释放原运动服务、清除空跑目标和策略历史并进入阻尼等待。
必须重新按 `p` 从实测角度复位，再按 `r`；绝不会直接发送先前计算的空跑目标。

空跑使用实测状态进行推理，不模拟机器人随目标运动。复位完成表示目标插值完成，
不要求真实关节到达默认姿态。温度、倾角等硬件检查异常仅报告预览警告；
CRC、机型、有限关节/IMU数据和状态新鲜度仍必须满足读取要求。
仿真和实机的策略后处理统一将29个目标角限制为±5 rad。
实机的`hardware.limit_target_position`当前为`true`，按`joint_lower/joint_upper`执行同样的±5 rad边界；缺省键为`false`。
目标变化率始终不限制。
真实输出保留硬件状态及指令有限值检查，不计算估算PD力矩或据此拦截。
实测关节位置、速度超出配置范围不自动退出，非有限数值仍会停止。
配置的关节范围用于复位姿态校验，并在启用目标角限位时作为下发边界；旧变化率和关节速度限制字段不再读取。

先核对模型、PD、关节范围与实机修订版；按该机器人官方低层开发流程准备调试状态。
首次测试使用吊架和独立停止手段，具体见 [硬件控制说明](docs/HARDWARE.md)。
在安装目录运行：

```bash
./g1 run eth0
```

启动时输出关闭，不自动接管或启动策略：

| 输入 | 行为 |
| --- | --- |
| `9` | 确认释放原运动服务后接管，进入阻尼等待 |
| `p` | 未按9：预览复位；按9后：从实测角度平滑复位 |
| `r` | 复位完成后进入策略；真实输出还要求实测角接近默认姿态，初始速度0 |
| `w/s` | 仿真键盘前后速度±0.1m/s；实机使用遥控器摇杆 |
| `a/d` | 仿真键盘横向速度±0.1m/s |
| `q/e` | 仿真键盘转向速度±0.1rad/s |
| `x` | 仿真键盘三维速度清零 |
| 空格 / Ctrl+C | 停止退出；仅在已接管后发送约0.5秒阻尼命令 |

按`9 → p → 等待Holding default pose → r`操作。推理中按`p`返回复位。
“使能”是软件接管，不操作机器人电源。退出不自动恢复原官方运动模式，不设置开机自动运行。

## 宇树原配遥控器

```bash
cd /home/unitree/g1_deploy/install
./g1 run enP8p1s0
```

原配遥控器使用 LowState 内的 `wireless_remote`，不需要 USB 接收器或 `/dev/input/js0`。
DWaQ 和 Parkour 实机控制均默认使用原配遥控器。键盘保留接管、复位、策略启停和空格/Ctrl+C 停止操作；实机速度由遥控器摇杆控制，F710 仅用于仿真。

| 输入 | 行为 |
| --- | --- |
| Start | 与键盘9相同，检查通过后释放原运动服务并接管，先进入阻尼等待 |
| X | 从实测角度复位；未按Start/9时仅预览目标 |
| A | 复位完成后进入策略；初始速度为零，进入时摇杆须回中 |
| 左摇杆前后 | 前后速度 |
| 左摇杆左右 | 横向速度 |
| 右摇杆左右 | 转向速度 |
| Select | 软件停止/退出；已接管时先发送约0.5秒阻尼；不是硬件断电 |

进入策略后直接使用摇杆，不需要按住R1；松手回中后速度归零，死区12%。
前后范围`[-0.6,1]m/s`、横向`[-0.5,0.5]m/s`、转向`[-1.57,1.57]rad/s`，由JSON配置。
输入归一化为`[ly,-lx,-rx]`。启动时已按住的Start/X/A不触发动作，须松开后重新按；
Select即使启动时按住也停止，且优先于其他按钮。B和Y不映射功能。
真实控制顺序为`Start → X → 等待Holding default pose → A`；
空跑顺序为`X → 等待Target interpolation complete → A`，全程不按Start或键盘9。
空跑后按Start会清除预览和策略状态，必须重新X复位。键盘空格/Ctrl+C仍可退出。

实机控制默认将遥控停止键设为 Select，A 用于进入策略。只读检查不启用遥控控制。
LowState超时、无效轴或软件故障会停止，但LowState持续更新不等于遥控无线链路有心跳；
当前无法保证识别固件重复旧遥控值时的无线失联。

## Logitech F710 (仿真时可用）

接收器插在运行程序的计算机上。切到XInput，MODE灯熄灭，
避免左杆与十字键互换，见 [Logitech手册](https://www.logitech.com/assets/34885/3/f710620-002923003403gswamr.pdf)。
先检查输入，再连接控制：

```bash
./g1 joystick /dev/input/js0       # 仅手柄输入，不初始化DDS
./bin/g1_dwaq 3 lo 1 apps/dwaq/config/g1_locomotion.json --joystick /dev/input/js0
```

Start使能、X复位、A进入策略；仿真进入前摇杆居中，进入后直接用摇杆运动，无需按住RB。
DreamWaQ左杆前后控制前后速度、左右控制横移，右杆左右控制转向；Parkour仅支持前向和转向。
摇杆回中归零，Y清零并锁定运动，摇杆回中后可重新运动；B停止并退出。
真机默认使用原配遥控器，不支持 F710。
速度按JSON范围缩放：默认前后`[-0.6,1]m/s`、横向`[-0.5,0.5]m/s`、转向`[-1.57,1.57]rad/s`。
输入诊断显示每个原始轴值，左杆前后为`ABS_Y (0x1)`；若只变化`0x11`，检查MODE状态。
手柄USB读取失败触发停止，但没有无线链路心跳，不能保证识别接收器仍在线时的无线失联。

不安装时可直接使用构建产物：

```bash
./build/g1_dwaq --check-policy
./build/g1_dwaq --check-joystick --joystick /dev/input/js0
./build/g1_dwaq --hardware-read-only --domain 0 --interface eth0 --config apps/dwaq/config/g1_locomotion.json
./build/g1_dwaq --sim --domain 1 --interface lo --config apps/dwaq/config/g1_locomotion.json --joystick /dev/input/js0
```

## 验证范围

仓库不构建自动化测试程序。日常手动检查可使用 `./g1 check`、`./g1 parkour check`、`./g1 hybrid check` 验证离线推理，使用 `./bin/g1_camera_check` 验证真实相机采集。

历史测试和现场调试结果见 [改动总结](CHANGELOG.md)。离线推理和空跑不能替代实机运动验收，具体运行工况以现场记录为准；硬件接管要求见 [硬件控制说明](docs/HARDWARE.md)。
