# RealSense 网页测试与 Parkour 双网络部署

本入口独立于原 DreamWaQ：`g1_dwaq` 继续使用原模型，新程序 `g1_parkour` 使用深度编码器和动作网络。
代码/模型移植自 `wangsy1999/unitree_mujoco` 提交 `7c6642f89e25f72dd7de3cf7683f166bce943fef` 的 `example/cpp/g1_parkour_policy.h`、`example/parkour/g1.json` 和 `exported/g1_parkour/`。
MuJoCo 仿真器、XML、网格仍放在外部仓库，不内置本仓库；所有验证均不使用 Isaac Sim。


## 相机链路对齐 Instinct（2026-10-22）

参考版本：`project-instinct/instinct_onboard@b6965a4202d8dceaf163dd95edb7742ed5d65e38`。

- 默认仅开启 Z16 深度，480×270、60 Hz；RGB 默认关闭，可用 `--color` 打开可选预览。
- 实机策略预处理参考该版本 `ParkourAgent.refresh_depth_frame`，当前按需关闭高斯模糊：最近邻缩放至64×36，裁剪 `[18,0,16,16]`，对小于0.2米区域用半径3的 OpenCV INPAINT_NS 补洞，跳过高斯模糊（`gaussian_kernel: 0`），裁剪0–2.5米并归一化至0–1。参数写在 `config/realsense_parkour.json`。
- 原始480×270米制深度写入本机共享内存，以32字节 `G1DEP003` UDP通知新帧；控制端在50Hz策略观测中执行缩放、裁剪、补洞和归一化，再送入深度历史，高斯模糊默认关闭。策略从策略配置同目录的 `realsense_parkour.json` 加载参数；相机预览应使用相同配置。仿真旧协议与原预处理保留。
- 深度历史按50Hz策略更新，采样步长按训练时间间隔换算：`round((5/50)/0.02)=5`，不使用相机采集频率换算。保留60项缓冲，取距最新项35/30/25/20/15/10/5/0项（8帧），帧间隔0.1秒、首尾跨度0.7秒，对齐InstinctLab Parkour训练配置。本体观测与模型权重未修改。
- 相机网页左侧显示完整原始深度，右侧在独立预览线程按相同配置生成32×18深度并放大；视场经过裁剪，不是逐像素对应的全画幅降噪对比。RealSense空间/时间滤波不叠加到此策略路径。
- 独立相机进程通过共享内存保留最新原始图，UDP仅通知；文件锁保证一致读取，忙时跳过，核对通知与图像头。时间戳映射与硬件数据时效保护保留。相机和控制器须同机同用户，先启动相机；旧相机服务需重启。
- 模型训练参数仍来自现有源码重建配置，尚无原始训练运行 YAML；此前启用高斯模糊时的处理算法通过固定版本上游函数逐像素比对；当前关闭模糊，与上游有意存在差异，不等同于已证明训练配置完全相同或完成实机运动验收。

策略线程以50Hz独立运行后处理和双网络推理，独占历史与复位；控制线程以500Hz调度，读取同步目标缓存持续下发。推理不持有缓存锁。复位/接管/重新进入策略时丢弃旧目标与在途结果，首个结果到达前保持站姿目标。首个结果等待或结果/输入快照年龄达到60ms、推理线程异常均走现有停止流程；真实输出时保留单次后处理与推理20ms超时检查。日志中的 `policy_target_age_ms` 表示目标完成后经过的时间，尚未产生结果时为-1。仿真只在新同步观测上推进策略历史。固定周期调度会跳过已错过的周期，不突发补算或补发；不保证操作系统硬实时。

默认启动仍为 `./g1 parkour real eth0`，自动启动深度相机且保持空跑。已有旧相机服务需先退出，避免复用旧分辨率或旧协议。

此前链路的历史验证（不代表当前共享内存链路实测）：14项Python测试、3项C++测试通过；仅相机采集连续检查测得59.70Hz，717帧的传输年龄0.76–32.76ms，无超时。没有启动运控或发布电机命令。

## 编译与安装

与主程序相同：使用与目标 CPU 匹配的 Unitree SDK2、ONNX Runtime、Boost、yaml-cpp 和 OpenCV C++（core/imgproc/photo，Ubuntu 为 `libopencv-dev`）。运行环境也需要对应 OpenCV 动态库。ARM64 机器人必须使用 aarch64 库。

```bash
cmake -S . -B build -DONNXRUNTIME_ROOT=/path/to/native/onnxruntime \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j4
cmake --install build
bash scripts/parkour.sh check
```

## RealSense 网页独立测试

相机外接线接到 PC2 所属 USB 3.x 接口。在 PC2 提供与 JetPack/Python 匹配的 librealsense/pyrealsense2；
网页还需 NumPy 和无 GUI 的 OpenCV，依赖范围见 `camera/requirements.txt`。Jetson 驱动安装方式需匹配实际 L4T，不自动修改内核或安装驱动。
已有 OpenCV 时不必再装另一份 headless 轮子。可使用 `.python-deps/` 或通过 `PYTHON_BIN` 指定环境。

```bash
bash scripts/realsense.sh --host 0.0.0.0 --port 8766
# 多相机时添加 --serial 实际序列号
```

同一局域网的电脑/手机访问 `http://PC2实际IP:8766/`，例如 `http://192.168.123.164:8766/`。
显示 RGB、0–2.5m 深度伪彩色、型号、序列号、图像年龄、内参和训练视场覆盖率。
这是只读服务，不导入 DDS，也不会发送电机命令。默认监听 `127.0.0.1`，外部访问需显式指定监听地址。
可选 `--token 密码` 使用 HTTP Basic（用户名 `camera`，适用于可信局域网；本服务不提供 TLS）。
拔出相机或采集失败会在网页显示错误，不再将旧画面标记在线。

无相机仅检查网页可运行：

```bash
bash scripts/realsense.sh --mock --host 127.0.0.1 --port 8766
```

该模式明确显示 `SYNTHETIC TEST`，禁止同时指定 `--depth-port`，不能用于实机策略。
安装包入口为 `./realsense ...` 或 `./g1 realsense ...`。

## 仿真：使用源仓库的 MuJoCo 相机

终端一（Python 环境需要源仓库的 MuJoCo、NumPy、pygame、unitree_sdk2py）：

```bash
python scripts/parkour_sim.py --mujoco-root /path/to/unitree_mujoco --domain 1 --port 17200
```

启动器只覆盖当前 Python 进程配置，不修改外部仓库。源仓库用训练相机内外参进行 MuJoCo 射线深度采样，
每个仿真时刻把关节、torso 角速度、重力、torso 高度和 36×64 深度打包到 `127.0.0.1:17200`。
启动器强制使用本项目 `config/g1_parkour.json`，保证关节方向签名一致。

终端二：

```bash
bash scripts/parkour.sh sim --domain 1 --port 17200
```

先 `p → 等待 Holding training pose → r` 空跑查看目标；
按 `9` 使能输出会清空空跑目标，再 `p → 等待复位 → r`。
`w/s` 前向速度±0.1 m/s，`q/e` 转向±0.1 rad/s，`x` 清零，空格/Ctrl+C 停止。
仿真 F710 可添加 `--joystick /dev/input/js0`，映射沿用原程序。
仿真域必须为正数；本程序始终将 UDP 绑定到回环地址。


## 实机单命令启动（自动相机）

在安装目录执行 `./g1 parkour real eth0`，入口会先启动 RealSense 网页服务和本机 UDP 17201 深度转发，等新帧就绪后再启动 Parkour。启动后仍为空跑，真实输出仍须 `--hardware-calibrated` 和 Start。

相机网页为 `http://机器人当前IP:8766/`。自动启动的相机日志保存在 `install/logs/parkour-camera.log`；运控退出或 Ctrl+C 时停止本次启动的相机。已有匹配 UDP 端口的真实相机服务会被复用，退出时保留；已有服务未转发深度时会明确报错，不会抢占设备。

`--port 17202` 会同时修改运控与相机的深度 UDP 端口；`--camera-web-port 8767` 修改自动相机网页端口。需要手动管理相机时使用 `./g1 parkour real eth0 --no-auto-camera`，下文的相机独立启动方式仍可使用。仿真与离线检查入口不会自动启动相机。

## 实机：RealSense + LowState + torso IMU

先运行独立相机服务，同时启用原始米制深度的本机转发：

```bash
bash scripts/realsense.sh --host 0.0.0.0 --port 8766 --depth-port 17201
```

再运行实机空跑（替换网卡）：

```bash
bash scripts/parkour.sh real eth0
```

原配遥控器沿用当前仓库映射：Start 接管、X 复位、A 策略、Select 停止。摇杆直接控制速度，无需 R1。
Start 之前只有 `rt/g1/dry_run_target`，不会创建 `rt/lowcmd` 发布者，不释放官方运动服务。
`p/X → 等待 → r/A` 可以先空跑，网页关节监控可显示空跑目标。空跑允许尚未达到训练姿态的机器人生成推理目标，实输出要求复位后接近训练姿态。

用户已确认相机按训练位置/朝向安装；实输出前仍须核对 torso IMU 坐标轴、腰部关节方向及硬件参数。
核对完成后显式添加：

```bash
bash scripts/parkour.sh real eth0 --hardware-calibrated
```

吊架上按 Start，确认 SDK MotionSwitcher 服务释放且未观察到其他 LowCmd 发布者，再 X 复位、A 策略。
使能会清空所有空跑目标，不能直接把空跑姿态切到电机。停止后发送约 0.5 秒阻尼命令。
低层开发状态和独立停止手段仍按该机器人交付文档准备。

实机输入明确分工：

| 数据 | 来源 |
| --- | --- |
| 关节位置/速度 | `rt/lowstate`，按策略顺序和 `joint_sign` 转换 |
| torso 角速度、重力 | `rt/secondary_imu` 的 `IMUState_`，四元数 wxyz；不能用 pelvis IMU 代替 |
| 深度 | RealSense Z16 × 设备 depth scale → 米制图像平面 Z → 本机 `G1DEP001` UDP |
| 速度命令 | Unitree 遥控器；vx∈[0,1]，vy固定0，wz∈[-1,1] |

相机和 C++ 推理必须在同一台 PC2，深度时间戳使用同机单调时钟。相机服务重启需同时重启控制器。
若固件没有 `rt/secondary_imu`，控制器等待后报错退出，不回退到 pelvis IMU。
程序假定 secondary IMU 已使用 torso 机体系；实际安装旋转不一致时必须先校正。绝对 torso 高度在实机输入中不可观测，实机不使用仿真的绝对高度保护。

## 深度与历史一致性

两个网络原样保留：`0-depth_encoder.onnx [1,8,18,32] → [1,128]`，`actor.onnx [1,896] → [1,29]`。
动作输入由 768 维本体历史和 128 维深度编码构成。本体各项8帧、旧到新；深度缓存37帧，取距当前35/30/25/20/15/10/5/0帧。
深度先采样为 36×64，去掉上18行和左右16列，再3×3 sigma=1模糊、裁剪0–2.5m、归一化。

RealSense 用其实际内参映射训练89.51°×58.29°视场，保持光轴Z深度，不将RGB对齐深度输入策略；零值和视场外为远处/无命中。
不直接拉伸图像，也不通过伪彩色恢复深度。相机外参记录在 `config/realsense_parkour.json` 中，作为安装要求，不会凭2D缩放自动纠正物理安装偏差。
默认相机30Hz、策略50Hz，因此部分策略帧复用最新深度；实际传感器时间特性与训练50Hz不同，需评估实机影响。可通过 `--fps` 选设备实际支持的帧率。
裁剪、深度缺失和视场覆盖率需先在网页检查；非零畸变的深度流会被拒绝，需提供已校正的深度配置。

保护沿用最新远程硬件接口：CRC/机型/有限状态及指令/电机故障和温度/倾角/目标角及变化率，主状态/torso IMU 超时100ms、相机超时150ms、流重启/回放、20ms推理超时。
不计算估算 PD 力矩，也不因实测关节位置或速度越出配置范围自动退出。初始实测角越界时，首次目标修正优先满足角度边界，可能超过变化率限制。
`g1_parkour.json` 中源训练的 `effort_limit` 仅供外部 MuJoCo 适配器限制仿真力矩，不用于 C++ 实机估算力矩拦截。
相机采集/网页运行在独立进程；C++推理与控制发布仍在同一控制线程，不是硬实时控制或独立硬件看门狗。
模型参数由源仓库从训练源码重建，缺少原训练运行的完整参数快照。集成通过不代表实机稳定跑酷；当前没有相机或机器人现场验收。

## 历史验证结果

以下为移除自动化测试前的验证记录。当前模型离线检查使用 `bash scripts/parkour.sh check`；仿真手动操作见上文。

本次 x86_64 开发机验证：6 项 C++ 检查、3 项相机网页/深度协议检查、原有3项关节网页检查通过；
MuJoCo 完成复位、策略重入、停止清零、观测超时停止。短时零命令落地站立检查通过，约4.75秒仿真时间、
最低torso高度0.815m、最大XY漂移0.246m。此结果仅覆盖当前场景，不能外推为完整跑酷或实机验证。
aarch64 编译/时延、USB RealSense 和机器人均需在目标机实测。

本次实际导入模型的校验和（与源仓库该提交的模型文件一致；源 README 的旧哈希已不对应当前文件）：

```text
90328400e49141578fdaefa1683c6346646d309d83333812bd424718288e9c5d  0-depth_encoder.onnx
e33d7653743dc81e48346cbd950b9d8a98dad10a08b38e259dbc8d1e4ba2a6bc  actor.onnx
```

接管与日志修订：LowCmd发布者在显式接管时准备，确认官方服务退出后立即发送阻尼；不再保留释放服务后的200/250ms固定等待。监测使用DDS样本发布者身份排除自身，持续检测外部指令。控制日志改为有界后台队列，繁忙时丢弃，避免慢终端阻塞控制循环。仿真启动器会清理传给外部模拟器的命令行参数，并在返回或异常时恢复原参数。
