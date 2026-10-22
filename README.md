# g1_deploy — G1 DreamWaQ 机载部署

新增独立的 **RealSense RGB/深度网页测试**与 **Parkour 双网络推理**（MuJoCo 相机 / 实机 RealSense 切换）。
入口为 `scripts/realsense.sh`、`scripts/parkour.sh`，安装后也可用 `./g1 realsense`、`./g1 parkour`。
完整接线、依赖、启动、空跑和标定要求见 [Parkour 与 RealSense 说明](docs/PARKOUR.md)。原 DreamWaQ 程序独立保留。

独立的G1 29DOF DreamWaQ C++推理项目，纯粹的vibe coding by codex。直接读取Unitree DDS低层状态并下发PR关节目标。
包含模型、部署配置、原配遥控器/F710输入、离线检查、网页监控和机载安装入口。
C++控制程序不依赖MuJoCo、ROS或Python；可选网页监控需要Python与官方SDK2 Python依赖，源码已包含在本仓库。

机载部署、输入及空跑验证记录见 [改动总结](CHANGELOG.md)。操作者已确认现场调试通过；
自动化测试、现场调试与具体行走工况的验收范围需分别确认。

## 可运行程序与使用方法

运行环境为 Linux；下文源码命令在仓库根目录执行，`./g1 ...` 命令在完成 CMake 安装后的 `install/` 目录执行。
先按下文“在G1开发计算机上编译”准备 C++ 依赖并构建。示例中的 `eth0`、机器人 IP、模型路径和外部仿真仓库路径均须按实际环境替换。
实机 DDS 使用域 `0` 和连接控制板的网卡，仿真使用正数域和 `lo`；两端域号必须相同。

| 程序 / 源码入口 | 功能 | 常用启动方式 |
| --- | --- | --- |
| `g1_dwaq` / `src/main.cpp` | DreamWaQ 行走策略、姿态复位、硬件只读检查和手柄诊断 | 安装后 `./g1 check`、`./g1 read-only eth0`、`./g1 run eth0 remote` |
| `g1_parkour` / `src/parkour.cpp` | 深度编码器与动作网络双 ONNX 推理，支持 MuJoCo 或实机深度输入 | `bash scripts/parkour.sh check\|sim\|real ...`（按需选择一个子命令） |
| `web/joint_monitor.py` | 只读关节、IMU、实际/目标曲线及空跑目标网页 | `bash scripts/monitor.sh eth0 0.0.0.0 8765` |
| `camera/realsense_web.py` | RealSense 原始/策略深度预览、可选 RGB、本机深度 UDP 转发 | `bash scripts/realsense.sh --host 0.0.0.0 --port 8766` |
| `camera/parkour_camera.py` | 管理相机与控制器生命周期，等待新帧、复用兼容服务、保存相机日志 | 由 `bash scripts/parkour.sh real eth0` 自动调用 |
| `scripts/parkour_sim.py` | 启动外部 MuJoCo 查看器，并加载本项目 Parkour 配置与相机桥接 | `python3 scripts/parkour_sim.py --mujoco-root /path/to/unitree_mujoco` |

`scripts/g1.sh` 是安装包统一入口，安装为 `g1`；`monitor.sh`、`realsense.sh`、`parkour.sh` 同时支持源码和安装目录，安装后分别命名为 `monitor`、`realsense`、`parkour`。`parkour_sim.py` 不随安装包安装，需从源码运行。

### DreamWaQ：检查、空跑与行走

```bash
# 以下在 install/ 目录执行
./g1 interfaces                     # 列出网卡和到控制板的路由
./g1 check                          # 离线加载模型并推理一次，不连接 DDS
./g1 joystick /dev/input/js0         # 只检查 F710 输入，不连接 DDS
./g1 read-only eth0                 # 只读 LowState、CRC、机型、电机和 IMU 检查后退出
./g1 run eth0                       # 键盘控制入口，启动时为空跑
./g1 run eth0 remote                # 原配遥控器控制入口
./g1 run eth0 /dev/input/js0         # F710 控制入口
```

键盘空跑：`p → 等待 Target interpolation complete → r`；真实输出：`9 → p → 等待 Holding default pose → r`。
`w/s` 调整前后速度、`a/d` 调整横向速度、`q/e` 调整转向、`x` 清零，空格或 Ctrl+C 停止。
原配遥控器对应 `Start` 接管、`X` 复位、`A` 策略、`Select` 停止，摇杆直接控制速度；F710 对应 `Start` 接管、`A` 复位、`Y` 策略、按住 `RB` 用摇杆运动、`X` 清零、`B` 停止。
接管会清空空跑目标，必须重新复位再进入策略。具体状态要求和保护见下文与 [硬件说明](docs/HARDWARE.md)。

不安装时可直接运行构建产物；位置参数依次为复位秒数、网卡、DDS 域、配置和模型：

```bash
./build/g1_dwaq --check-policy models/policy.onnx config/g1_locomotion.yaml
./build/g1_dwaq --hardware-read-only 3 eth0 0 config/g1_locomotion.yaml
./build/g1_dwaq --hardware 3 eth0 0 config/g1_locomotion.yaml models/policy.onnx --unitree-remote
# 外部 DreamWaQ 仿真器已启动时：
./build/g1_dwaq 3 lo 1 config/g1_locomotion.yaml models/policy.onnx
```

### Parkour：离线推理、仿真和实机

```bash
# 源码根目录；仅执行双网络推理检查，无需相机或 DDS
bash scripts/parkour.sh check

# 仿真终端一：外部仓库需含 simulate_python/parkour_observation.py
python3 scripts/parkour_sim.py --mujoco-root /path/to/unitree_mujoco --domain 1 --port 17200
# 仿真终端二：域号和 UDP 端口与终端一一致
bash scripts/parkour.sh sim --domain 1 --port 17200

# 实机：自动启动/复用相机，等新帧就绪后启动运控，默认空跑
bash scripts/parkour.sh real eth0
# 完成相机安装、torso IMU 轴向、关节方向和硬件参数核对后，允许手动接管
bash scripts/parkour.sh real eth0 --hardware-calibrated
```

安装后分别使用 `./g1 parkour check`、`./g1 parkour sim ...`、`./g1 parkour real eth0 ...`。
仿真 Python 环境需外部仓库及其 MuJoCo、NumPy、pygame、`unitree_sdk2py` 依赖；本仓库不包含仿真器、XML 和网格。
仿真键盘使用 `p` 复位、`r` 策略、`9` 使能输出、`w/s` 前向速度、`q/e` 转向、`x` 清零、空格/Ctrl+C 停止；可加 `--joystick /dev/input/js0` 使用 F710。
实机使用原配遥控器 `X → 等待 → A` 空跑；显式指定 `--hardware-calibrated` 后才可 `Start → X → 等待复位 → A` 进入真实输出，`Select` 停止。实机还需要 `rt/secondary_imu` 的 torso IMU。

常用选项：`--port` 修改深度 UDP 端口（仿真默认 `17200`，实机 `17201`），`--reset-seconds` 修改复位时长（默认 3 秒），`--model-dir`、`--config`、`--limits` 指定模型、策略配置和硬件限制。
实机包装脚本还支持 `--camera-web-port 8767` 修改网页端口、`--no-auto-camera` 改为手动管理相机。
直接运行示例为 `./build/g1_parkour --sim --domain 1 --interface lo --port 17200`；直接实机入口为 `./build/g1_parkour --hardware --domain 0 --interface eth0 --port 17201`，需另行启动相机。`./build/g1_parkour --help` 查看参数。

自动相机管理器 `camera/parkour_camera.py` 会把日志写到源码或安装根目录下的 `logs/parkour-camera.log`，控制器退出时结束本次启动的相机，保留复用的相机服务。单独调用方式：

```bash
python3 camera/parkour_camera.py --root "$PWD" --depth-port 17201 --web-port 8766 -- \
  ./build/g1_parkour --hardware --domain 0 --interface eth0 --port 17201
```

它仅复用深度端口匹配、持续有新帧的 Instinct 480×270/60 Hz 深度服务；旧配置或启用 RGB 的服务需先退出再重启。

### RealSense：查看相机与转发策略深度

Python 依赖为 NumPy、OpenCV 和与目标设备匹配的 `pyrealsense2`，版本范围见 `camera/requirements.txt`；需要已可工作的 RealSense SDK/USB 相机。
启动脚本支持 `PYTHON_BIN=/path/to/python3` 指定解释器。

```bash
# 仅查看深度网页；默认 480×270、60 Hz、Instinct 预处理，RGB 默认关闭
bash scripts/realsense.sh --host 0.0.0.0 --port 8766
# 开启可选 RGB 预览（独立查看用途）
bash scripts/realsense.sh --host 0.0.0.0 --port 8766 --color
# 手动为实机 Parkour 提供本机深度，同时开放网页
bash scripts/realsense.sh --host 0.0.0.0 --port 8766 --depth-port 17201
# 无相机时用合成画面检查网页；不能与 --depth-port 同用
bash scripts/realsense.sh --mock --host 127.0.0.1 --port 8766
```

浏览器打开 `http://机器人IP:8766/`，查看原始深度、32×18 策略深度、帧龄和相机状态；`--color` 额外启用 RGB。
Instinct 模式将深度缩放、裁剪、补洞、模糊并归一化后，用 `G1DEP002` 发至本机控制器；控制器使用 60 项深度缓存取 8 帧历史。
`--camera-mode legacy` 保留旧处理方式；仿真仍使用原观测协议与预处理，详细差异见 [Parkour 说明](docs/PARKOUR.md) 开头的相机链路对齐章节。
多相机用 `--serial 序列号` 选择；`--profile` 指定预处理配置，`--width/--height/--fps` 选择设备支持的深度模式。
默认仅监听 `127.0.0.1`；`--token 密码` 可开启 HTTP Basic 认证，用户名为 `camera`。
按 Ctrl+C 退出。此服务不连接 DDS、不下发电机命令。
安装后使用 `./g1 realsense ...`；直接 Python 入口为 `python3 camera/realsense_web.py ...`，参数相同，`--help` 查看完整选项。

### 关节监控：实机与仿真网页

按 [网页部署说明](docs/WEB.md) 准备 `web/requirements.txt` 和官方 `unitree_sdk2py`。

```bash
# 源码入口，参数依次为网卡、监听地址、网页端口；固定为实机 DDS 域 0
bash scripts/monitor.sh eth0 0.0.0.0 8765
# 仿真监控：直接 Python 入口可指定仿真域
python3 web/joint_monitor.py --domain 1 --interface lo --host 127.0.0.1 --port 8765 --rate 50
# 实机直接 Python 入口
python3 web/joint_monitor.py --domain 0 --interface eth0 --host 0.0.0.0 --port 8765 --rate 50
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
./g1 run enP8p1s0 remote
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
src/main.cpp                 DDS、复位、策略切换与硬件控制
include/                     ONNX策略、硬件检查与F710输入
config/g1_locomotion.yaml    关节映射、默认角、PD、观测和实机保护
models/policy.onnx           DreamWaQ网络
scripts/g1.sh                安装包启动入口
cmake/BUILD_INFO.in          构建信息模板
docs/HARDWARE.md             接管、传感器及停止路径说明
web/joint_monitor.py         只读DDS监控后端
web/static/joint_monitor.html 网页界面
scripts/monitor.sh           网页启动入口
docs/WEB.md                  网页依赖、运行与验证说明
CHANGELOG.md                 本次改动与验证记录
```

模型和配置来源及SHA256见 [ORIGIN.md](ORIGIN.md)。
网络接口为float32 `observation [1,96]`、`observation_history [1,480]` → `actions [1,29]`。
默认20ms推理、2ms发布关节目标，策略使用LowState主IMU的pelvis机体系角速度和重力方向。

## 网页监控

```bash
# 仓库根目录运行；先按 docs/WEB.md 准备可选Python依赖
bash scripts/monitor.sh enP8p1s0 192.168.123.164 8765
# cmake安装后也可在install目录执行：./g1 monitor enP8p1s0 192.168.123.164 8765
```

浏览器访问`http://192.168.123.164:8765/`。网页只读，不会启动控制程序。
完整说明见 [网页监控部署](docs/WEB.md)。

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

安装目录包含`g1`和`monitor`启动脚本、`bin/g1_dwaq`、`models/`、`config/`、`lib/`和`web/`。
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

### 不按 9 即为空跑

```bash
cd install
./g1 run enP8p1s0
# 可选 F710：./g1 run enP8p1s0 /dev/input/js0
```

程序默认空跑，读取真实 LowState；不按 `9`，直接按 `p` 预览复位，等待 `Target interpolation complete` 后按 `r` 执行策略推理。
`w/s`、`a/d`、`q/e` 和 `x` 调整或清零预览速度；空格、Ctrl+C 或F710的B退出。
按下 `9` 或 F710 Start 才申请接管并打开真实输出。在此之前不创建 `rt/lowcmd` 发布器、
不调用 MotionSwitcher、退出时也不发阻尼指令。预览目标使用独立的 `rt/g1/dry_run_target` 遥测话题，
网页标为“空跑”，本程序 Enable 显示 OFF。空跑不改变机器人电源、官方服务或其他程序的输出状态。
按 `9` 时先严格检查真实状态；通过后释放原运动服务、清除空跑目标和策略历史并进入阻尼等待。
必须重新按 `p` 从实测角度复位，再按 `r`；绝不会直接发送先前计算的空跑目标。

空跑使用实测状态进行推理，不模拟机器人随目标运动。复位完成表示目标插值完成，
不要求真实关节到达默认姿态。温度、倾角等硬件检查异常仅报告预览警告；
CRC、机型、有限关节/IMU数据和状态新鲜度仍必须满足读取要求。
预览目标不经过真实输出路径的关节限位与变化率保护，不能作为实机可执行性验证。
真实输出保留完整硬件状态、有限指令、目标限位及变化率检查，不计算估算PD力矩或据此拦截。
实测关节位置、速度超出配置范围也不自动退出，非有限数值仍会停止。
配置的关节范围和速度上限仅用于限制下发目标；若初始实测角已在范围外，
首次修正优先保证目标角在配置范围内，无法同时保证该次修正的变化率。

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
| `w/s` | 前后速度±0.1m/s |
| `a/d` | 横向速度±0.1m/s |
| `q/e` | 转向速度±0.1rad/s |
| `x` | 三维速度清零 |
| 空格 / Ctrl+C | 停止退出；仅在已接管后发送约0.5秒阻尼命令 |

按`9 → p → 等待Holding default pose → r`操作。推理中按`p`返回复位。
“使能”是软件接管，不操作机器人电源。退出不自动恢复原官方运动模式，不设置开机自动运行。

## 宇树原配遥控器

```bash
cd /home/unitree/g1_deploy/install
./g1 run enP8p1s0 remote
```

原配遥控器使用 LowState 内的 `wireless_remote`，不需要 USB 接收器或 `/dev/input/js0`。
此入口与 F710 输入互斥；不加 `remote` 的键盘入口保持原行为。

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
前后范围`[-0.6,1]m/s`、横向`[-0.5,0.5]m/s`、转向`[-1.57,1.57]rad/s`，由YAML配置。
输入归一化为`[ly,-lx,-rx]`。启动时已按住的Start/X/A不触发动作，须松开后重新按；
Select即使启动时按住也停止，且优先于其他按钮。B和Y不映射功能。
真实控制顺序为`Start → X → 等待Holding default pose → A`；
空跑顺序为`X → 等待Target interpolation complete → A`，全程不按Start或键盘9。
空跑后按Start会清除预览和策略状态，必须重新X复位。键盘空格/Ctrl+C仍可退出。

原配遥控器模式将硬件检查中的遥控停止键从A改为Select，A才能进入策略；
未选择此模式时仍保留原配遥控器A停止的旧行为。
LowState超时、无效轴或软件故障会停止，但LowState持续更新不等于遥控无线链路有心跳；
当前无法保证识别固件重复旧遥控值时的无线失联。

## Logitech F710 (仿真时可用）

接收器插在运行程序的计算机上。切到XInput，MODE灯熄灭，
避免左杆与十字键互换，见 [Logitech手册](https://www.logitech.com/assets/34885/3/f710620-002923003403gswamr.pdf)。
先检查输入，再连接控制：

```bash
./g1 joystick /dev/input/js0       # 仅手柄输入，不初始化DDS
./g1 run eth0 /dev/input/js0
```

Start使能、A复位、Y进入策略；进入前摇杆居中并松开RB。
按住RB：左杆前后控制前后速度，左杆左右控制横移，右杆左右控制转向。
松开RB归零，X清零并锁定运动，松开RB且摇杆居中后可重新运动；B停止并退出。
速度按YAML范围缩放：默认前后`[-0.6,1]m/s`、横向`[-0.5,0.5]m/s`、转向`[-1.57,1.57]rad/s`。
输入诊断显示每个原始轴值，左杆前后为`ABS_Y (0x1)`；若只变化`0x11`，检查MODE状态。
手柄USB读取失败触发停止，但没有无线链路心跳，不能保证识别接收器仍在线时的无线失联。

不安装时可直接使用构建产物：

```bash
./build/g1_dwaq --check-policy
./build/g1_dwaq --check-joystick --joystick /dev/input/js0
./build/g1_dwaq --hardware-read-only 3 eth0 0 config/g1_locomotion.yaml
./build/g1_dwaq --hardware 3 eth0 0 config/g1_locomotion.yaml models/policy.onnx --joystick /dev/input/js0
```

## 验证范围

当前仓库已移除自动化测试脚本与 C++ 测试目标。日常检查可运行安装目录下的 `./g1 check`、`./g1 parkour check`，分别验证 DreamWaQ 和 Parkour 模型加载及离线推理；硬件只读检查使用 `./g1 read-only 实际网卡`。

历史测试和现场调试结果见 [改动总结](CHANGELOG.md)。离线推理和空跑不能替代实机运动验收，具体运行工况以现场记录为准；硬件接管要求见 [硬件控制说明](docs/HARDWARE.md)。