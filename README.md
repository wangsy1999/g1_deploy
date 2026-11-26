# g1_deploy

G1 29DOF 的 C++ 策略控制、MuJoCo 仿真入口和网页监控。当前 `main` 使用 CPU ONNX Runtime。

| 控制器 | 用途 |
| --- | --- |
| `dwaq` | DreamWaQ 行走策略 |
| `parkour` | 深度视觉越障策略 |
| `hybrid` | DWaQ 站立/后退，持续前进时切换到 Parkour |

以下命令默认在项目根目录执行。一次只运行一个控制器；仿真与控制器须同机、同用户，DDS 域号一致。

## 1. 安装依赖与编译

### ARM64 机器人：自动安装、编译

```bash
bash scripts/setup_robot.sh
```

缺少依赖时自动下载并安装，随后编译到 `build/`、安装到 `install/`，不启动控制程序。
GitHub 依赖下载失败时会自动切换镜像，可用 `G1_GITHUB_MIRROR` 指定镜像站。
pip 联网安装默认使用清华 PyPI 镜像，可用 `G1_PIP_INDEX_URL` 指定其他源。
依赖保存在 `~/.local/share/g1-deploy/aarch64/`，下载缓存在 `~/.cache/g1-deploy/robot-arm64/`；删除整个项目后也能复用，无需保留项目 `.deps/`。

```bash
bash scripts/setup_robot.sh --deps-only      # 只安装依赖
bash scripts/setup_robot.sh --jobs 4         # 调整编译并行数
bash scripts/setup_robot.sh --offline        # 已有依赖/离线缓存时使用
bash scripts/setup_robot.sh --verify-camera  # 编译后额外检查真实相机
```

缺少系统包时需要可用软件源和 sudo 权限；`--no-system-deps` 禁止调用 apt。详细说明见 [机器人环境配置](docs/ROBOT_SETUP.md)。

### Linux 开发机：手动编译

准备 CMake ≥ 3.16、C++17 编译器、Unitree SDK2/CycloneDDS、Boost、yaml-cpp、OpenCV（core/imgproc/photo）和与本机架构匹配的 ONNX Runtime C/C++ SDK。网页功能另需 Python、NumPy、OpenCV、`unitree_sdk2py`；视频录制需支持 `libx264` 的 FFmpeg。MuJoCo 依赖由外部仿真项目提供。

把下面两处依赖路径换成本机路径；若已安装到上述固定依赖目录，可省略对应参数。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/path/to/onnxruntime \
  -Dunitree_sdk2_DIR=/path/to/unitree-sdk2/lib/cmake/unitree_sdk2 \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j4
```

实机 Parkour/Hybrid 需要 **librealsense2 C++ SDK**，配置时增加 `-DG1_REQUIRE_REALSENSE=ON`；SDK 在自定义位置时指定 `-Drealsense2_DIR=/path/to/lib/cmake/realsense2`。仅安装 `pyrealsense2` 不满足 C++ 构建要求。无原生 SDK 时可编译仿真和离线检查。

后续修改源码后：

```bash
cmake --build build -j4
# 全新编译：删除 build/ 后重新执行上述 cmake 配置与编译命令
```

### Python 环境（无需激活）

监控、相机及仿真脚本会自动选择已配置的解释器，无需 `source`。仿真会查找外部项目的 `.venv/` 或 `venv/`；其它环境可在各终端设置以下变量（替换为已有环境的路径），后续仍直接运行 `python3 apps/...`：

```bash
export G1_SIM_PYTHON=/path/to/env/bin/python3
export PYTHON_BIN=/path/to/env/bin/python3
```

当前这台开发机的 Python 路径为 `/home/wsy/miniconda3/envs/unitree/bin/python`，外部仿真仓库为 `/home/wsy/rl/unitree_mujoco`。机器人固定依赖目录已准备好时无需设置上述变量。

### 离线检查、安装与打包

```bash
./g1 dwaq check
./g1 parkour check
./g1 hybrid check

cmake --install build            # 可选：生成 install/
(cd build && cpack -G TGZ)       # 可选：生成当前架构的安装压缩包
```

源码根目录的 `./g1` 使用 `build/`，安装目录的 `./g1` 使用 `install/bin/`。更新源码后重新编译；使用安装目录时还需重新安装。控制器更新后重启进程，网页更新后重启 monitor 并刷新浏览器。

## 2. MuJoCo 仿真与 F710

外部 `unitree_mujoco` 需包含 `simulate_python/unitree_mujoco.py` 和 G1 29DOF 场景及其依赖。

**终端一：启动仿真。** 将路径替换为实际仓库位置。

```bash
python3 apps/simulation/sim.py \
  --mujoco-root /path/to/unitree_mujoco --domain 1
```

**终端二：选择一个控制器。** F710 切到 XInput，MODE 灯熄灭；设备路径按实际情况替换。

```bash
./g1 joystick /dev/input/js0                 # 可先只读检查手柄

./g1 dwaq sim --joystick /dev/input/js0
# 或
./g1 parkour sim --joystick /dev/input/js0
# 或
./g1 hybrid sim --joystick /dev/input/js0
```

默认仿真域为 `1`、网卡为 `lo`；更换域号时，仿真、控制器和 monitor 都要同步修改，例如 `./g1 parkour sim --domain 2 --joystick /dev/input/js0`。

| F710 操作 | 功能 |
| --- | --- |
| 摇杆回中 → Start → X → 等待复位完成 → A | 使能仿真输出、复位并启动策略 |
| X → 等待复位完成 → A，不按 Start | 仅空跑计算目标，仿真机器人不跟随 |
| 左摇杆前后 / 右摇杆左右 | 速度 / 转向，无需按住 RB |
| Y | 速度清零，摇杆回中后可重新运动 |
| B | 停止并退出 |

DWaQ 还支持左摇杆左右横移。Parkour 仅支持前进和转向。

不用手柄时省略 `--joystick`：键盘 `9 → p → 等待复位完成 → r`；`w/s` 调前后速度、`q/e` 调转向、`x` 清零，DWaQ 另支持 `a/d` 横移。空格或 Ctrl+C 退出。

Parkour/Hybrid 在 MuJoCo reset 后保持策略运行和输出使能状态，刷新历史后自动继续，无需重新按 Start/X/A；仿真暂停后也可等待数据恢复。

常用仿真选项：

```bash
python3 apps/simulation/sim.py --mujoco-root /path/to/unitree_mujoco \
  --domain 1 --follow --depth-preview --terrain mixed
```

| 选项 | 功能 |
| --- | --- |
| `--follow` | 主视角跟随机器人 |
| `--depth-preview` | MuJoCo 窗口内显示深度预览 |
| `--terrain mixed` | 综合地形；也可用 `platforms`、`stones`、`gaps`、`stairs`、`hurdles` |
| `--no-depth` | 仅运行 DWaQ 时关闭深度采样；Parkour/Hybrid 不可使用 |
| `--window-backend x11\|wayland\|auto` | 窗口后端，默认 `x11` |

仿真启动器从源码运行，不包含在 `install/` 中。地形尺寸见 [terrain.json](apps/simulation/terrain.json)。

## 3. 实机运行

将示例 `eth0` 替换为连接控制板的实际网卡。实机使用 DDS 域 `0`，原配遥控器控制；F710 仅用于仿真。首次输出前按 [硬件说明](docs/HARDWARE.md) 检查模型、关节配置和现场保护。

```bash
./g1 interfaces         # 查看网卡与路由
./g1 read-only eth0     # 只读检查状态，不接管

./g1 dwaq real eth0
# 或
./g1 parkour real eth0
# 或
./g1 hybrid real eth0
```

启动时输出关闭。原配遥控器操作：

| 操作 | 功能 |
| --- | --- |
| X → 等待复位完成 → A，不按 Start | 空跑预览 |
| 摇杆回中 → Start → X → 等待复位完成 → A | 接管、复位并进入真实输出 |
| 左摇杆 / 右摇杆左右 | 速度 / 转向，无需按住 R1 |
| Select，或键盘空格/Ctrl+C | 停止并退出 |

Start 会接管并释放原运动服务；退出不会自动恢复官方运动模式。空跑后再按 Start 必须重新复位。仿真的 reset 自动恢复行为不适用于实机保护。

Parkour/Hybrid 自动在 C++ 控制器内采集 RealSense 深度，需相机及 `rt/secondary_imu` 在线。运行实机控制器前先关闭独立的相机预览服务。

## 4. 网页监控、相机预览与录制

### 关节、温度与 IMU 监控

另开终端运行。**仿真监控使用 Python 入口指定域 1；`./g1 monitor` 固定为实机域 0。**

```bash
# 仿真
python3 apps/monitor/joint_monitor.py \
  --domain 1 --interface lo --host 127.0.0.1 --port 8765

# 实机：同网段电脑访问时监听所有地址
./g1 monitor eth0 0.0.0.0 8765
```

打开 `http://127.0.0.1:8765/`，远程访问则将地址换成运行 monitor 的机器 IP。页面显示 29 个关节、双通道温度、实际/目标曲线和两路 IMU；勾选关节可绘图，支持暂停显示、搜索和角度单位切换。网页不发送运动命令。

### 复用 policy 相机画面

保持 Parkour/Hybrid 控制器运行，monitor 中会出现深度画面，无需启动另一个相机进程，也无需先启动策略。默认裁剪后为 32×18，关闭裁剪为 64×36。

页面可独立开关裁剪、填洞、滤波、归一化并调整参数，支持灰度/伪彩色和像素读数；这些设置只作用于预览副本，不修改 policy 输入。默认本机 UDP 端口 `9871`。更换端口时两端保持一致：

```bash
G1_DEPTH_PREVIEW_PORT=9872 ./g1 parkour sim --joystick /dev/input/js0
G1_DEPTH_PREVIEW_PORT=9872 python3 apps/monitor/joint_monitor.py --domain 1 --interface lo
```

### CSV、深度数值与视频录制

| 入口 | 保存内容 |
| --- | --- |
| 页面顶部「开始新记录」 | 关节状态、双通道温度和 IMU，保存为 CSV |
| 深度区「开始深度录制」 | 当前处理结果的 float32 数值，无损压缩为 `.g1depth` |
| 勾选「同时保存 MP4」（默认开启） | 同步保存当前配色的 H.264 视频 |

深度录制可选 **5/10/15/20/25/30 FPS**，默认 5 FPS；低帧率、小裁剪区域、关闭视频可减小数据量。实际帧率受源帧率限制，页面会显示实录 FPS。录制参数在开始时固定；视频用于观看，数值分析使用 `.g1depth`。

点击停止后分别下载。默认写入运行 monitor 的机器 `~/logs/`；关闭或刷新网页仍继续录制，退出 monitor 时保存并结束。显示暂停和绘图选择不影响记录。视频需要 FFmpeg；无编码器时可关闭视频，仅录制数值。视频按所选帧率播放，真实时间以数值文件时间戳为准。

```bash
# 自定义记录目录
python3 apps/monitor/joint_monitor.py --domain 1 --interface lo --record-dir "$HOME/logs/test"
# 查看深度文件信息 / 导出 NumPy 数组
python3 apps/monitor/depth_recording.py /path/to/file.g1depth
python3 apps/monitor/depth_recording.py /path/to/file.g1depth --export /tmp/depth.npz
```

归一化开启时记录单位为 0～1；需要米制数据时，开始录制前关闭归一化。文件恢复、字段和格式见 [网页与录制说明](docs/WEB.md)。

### 独立 RealSense 检查与 RGB 预览

这组入口会打开相机，使用前退出实机 Parkour/Hybrid 控制器。

```bash
./build/g1_camera_check                       # C++ 相机只读检查；安装后为 ./bin/g1_camera_check
./g1 realsense --host 127.0.0.1 --port 8766    # 深度预览
./g1 realsense --host 127.0.0.1 --port 8766 --color  # 增加 RGB
./g1 realsense --mock --host 127.0.0.1 --port 8766   # 无相机的合成画面测试
```

打开 `http://127.0.0.1:8766/`。常用参数：`--serial 序列号`、`--profile 配置.json`、`--width/--height/--fps`、`--camera-mode instinct|legacy`、`--token 密码`。完整说明见 [Parkour 与相机](docs/PARKOUR.md)。

## 5. 配置与验证

模型、复位时间和硬件参数保存在各自 JSON，使用 `--config` 可选择其它配置：

| 配置 | 路径 |
| --- | --- |
| DWaQ | [g1_locomotion.json](apps/dwaq/config/g1_locomotion.json) |
| Parkour | [g1_parkour.json](apps/parkour/config/g1_parkour.json) |
| Hybrid | [g1_hybrid.json](apps/hybrid/config/g1_hybrid.json) |
| 相机预处理 | [realsense_parkour.json](apps/parkour/config/realsense_parkour.json) |

```bash
./g1 parkour sim --config /path/to/g1_parkour.json --joystick /dev/input/js0
./g1 parkour help
./g1 hybrid help
python3 apps/monitor/joint_monitor.py --help
python3 apps/realsense/realsense_web.py --help

# 无需机器人/相机；测试解释器需装有 NumPy、OpenCV，视频测试需 FFmpeg，C++ 测试需 g++
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

构建产物不跨架构通用。更多部署、硬件及实现细节见 [ROBOT_SETUP](docs/ROBOT_SETUP.md)、[HARDWARE](docs/HARDWARE.md)、[PARKOUR](docs/PARKOUR.md)、[WEB](docs/WEB.md)；模型来源见 [ORIGIN](ORIGIN.md)，历史记录见 [CHANGELOG](CHANGELOG.md)。
