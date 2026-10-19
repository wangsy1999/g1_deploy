# g1_deploy — G1 DreamWaQ 机载部署

独立的G1 29DOF DreamWaQ C++推理项目，直接读取Unitree DDS低层状态并下发PR关节目标。
包含模型、部署配置、F710输入、离线检查和机载安装入口，不依赖MuJoCo、ROS、Python运行时或其他源码仓库。

程序尚未在真机验收。本仓库提供部署代码和离线检查，不能据此认定实机已经能稳定行走。

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
```

模型和配置来源及SHA256见 [ORIGIN.md](ORIGIN.md)。
网络接口为float32 `observation [1,96]`、`observation_history [1,480]` → `actions [1,29]`。
默认20ms推理、2ms发布关节目标，策略使用LowState主IMU的pelvis机体系角速度和重力方向。

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

安装目录包含`g1`启动脚本、`bin/g1_dwaq`、`models/`、`config/`和`lib/`。
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
| `p` | 从实测角度平滑复位 |
| `r` | 复位完成并接近默认姿态后进入策略，初始速度0 |
| `w/s` | 前后速度±0.1m/s |
| `a/d` | 横向速度±0.1m/s |
| `q/e` | 转向速度±0.1rad/s |
| `x` | 三维速度清零 |
| 空格 / Ctrl+C | 停止并发送约0.5秒阻尼命令 |

按`9 → p → 等待Holding default pose → r`操作。推理中按`p`返回复位。
“使能”是软件接管，不操作机器人电源。退出不自动恢复原官方运动模式，不设置开机自动运行。

## Logitech F710

接收器插在运行程序的机载开发计算机上。切到XInput，MODE灯熄灭，
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

离线检查覆盖硬件CRC/状态/目标限制、F710轴和按钮映射、初始化按键抑制、RB门控、
观测排列、历史、关节映射与实际ONNX推理。
从原仿真项目抽取的控制器曾通过MuJoCo复位/策略重入检查；本独立仓库不带仿真器。
实际机载SDK、网络、MotionSwitcher响应、F710、停止行为和行走稳定性仍需现场验收。
