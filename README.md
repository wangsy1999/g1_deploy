# g1_deploy — G1 DreamWaQ 机载部署

独立的G1 29DOF DreamWaQ C++推理项目，纯粹的vibe coding by codex。直接读取Unitree DDS低层状态并下发PR关节目标。
包含模型、部署配置、原配遥控器/F710输入、离线检查、网页监控和机载安装入口。
C++控制程序不依赖MuJoCo、ROS或Python；可选网页监控需要Python与官方SDK2 Python依赖，源码已包含在本仓库。

机载部署、输入及空跑验证记录见 [改动总结](CHANGELOG.md)。操作者已确认现场调试通过；
自动化测试、现场调试与具体行走工况的验收范围需分别确认。

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
tests/                      硬件检查、手柄映射、观测及推理测试
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
ctest --test-dir build --output-on-failure
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

```bash
# 仓库根目录执行
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s web/tests -v
```

已记录4项C++测试和3项网页测试通过。离线检查覆盖硬件CRC/状态/目标限制、
F710及原配遥控器映射、初始化按键抑制、停止优先级、速度输入、观测排列、历史、
关节映射和实际ONNX推理；网页检查覆盖资源加载、HTTP/SSE、空跑目标及过期处理。
原配遥控器4轴实测、机载空跑和源码/安装目录的网页实时状态也已验证。
从原仿真项目抽取的控制器曾通过MuJoCo复位/策略重入检查；本独立仓库不带仿真器。
自动化和空跑检查未执行真实接管或行走；操作者确认的现场调试与具体运行工况仍以现场记录为准。
详细结果及边界见 [改动总结](CHANGELOG.md) 和 [硬件控制说明](docs/HARDWARE.md)。
