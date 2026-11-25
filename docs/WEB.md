# 网页关节监控

网页及Python后端均已收进本仓库，无需另一个unitree_mujoco源码仓库。
监控只订阅DDS，不发布LowCmd、不释放运动服务，也不提供运动控制按钮。
它显示29个关节的角度、速度、反馈力矩、目标、误差和IMU，以及最近20秒的曲线。
实际目标与空跑目标分开处理；空跑时显示`OFF · 空跑`。

## 运行依赖

C++控制程序仍不依赖Python。仅网页需要Linux上的Python 3.10或更新版本、
CPU架构匹配的CycloneDDS C运行库、Python依赖，以及官方`unitree_sdk2py`包。
本次机器人验证环境为Ubuntu 22.04 / aarch64 / Python 3.10 / CycloneDDS Python 0.10.2。
SDK及CycloneDDS运行库和控制程序使用同一套，避免混用ROS环境中的另一套DDS库。

机器人首次使用在项目根目录执行：

```bash
bash scripts/setup_robot.sh --deps-only
```

依赖自动安装到 `~/.local/share/g1-deploy/aarch64/`。之后 shell 启动入口与直接运行 `python3 apps/monitor/joint_monitor.py ...` 都会自动选择该解释器并配置 DDS 动态库路径，无需 `source .../activate`，也不需要手动设置 `PYTHONPATH` 或 `LD_LIBRARY_PATH`。新终端中直接运行即可。

使用其它环境时可指定 `PYTHON_BIN=/path/to/venv/bin/python3`，同样无需激活。旧项目 `.deps/` 和 `.python-deps/` 仍兼容。
## 从源码目录启动

```bash
cd /home/unitree/g1_deploy
bash apps/monitor/monitor.sh enP8p1s0 192.168.123.164 8765
```

在同一网络的电脑浏览器访问`http://192.168.123.164:8765/`。
第一个参数是实际DDS网卡，第二个是HTTP监听地址，第三个是端口。
不指定地址时仅监听`127.0.0.1`；真机通信域为0，最高采样频率50Hz。
按Ctrl+C停止网页，不影响其他控制程序。不设置开机自启。

也可直接启动Python后端：

```bash
python3 apps/monitor/joint_monitor.py --domain 0 --interface enP8p1s0 \
    --host 192.168.123.164 --port 8765 --rate 50
```

直接运行 Python 后端也会在导入依赖前自动选择解释器，与 shell 入口一样无需激活环境。

## 安装目录

`cmake --install build`会同时安装`apps/monitor/`和`monitor`启动脚本：

```bash
cd install
./monitor enP8p1s0 192.168.123.164 8765
# 等效入口：./g1 monitor enP8p1s0 192.168.123.164 8765
```

Python 依赖不打进 CMake 安装包；源码和安装目录都会自动使用机器人固定目录中的 Python 环境。目标机器首次使用先运行配置脚本安装依赖。
网页静态文件按Python源码位置加载，目录整体移动后仍能使用。

## Policy 相机深度预览

监控页新增「Policy 相机 · 深度预览」。Parkour 和 Hybrid 控制器复用已有相机/仿真深度数据，后台以最高约 60 FPS（受实际相机/仿真帧率限制）向本机 monitor 发送预览副本。monitor 不打开 RealSense；独立的 `./g1 realsense` 服务仍需在实机控制器退出后使用。

更新后需重新编译控制器，再重启 monitor 并刷新网页。实机运行已有的 Parkour/Hybrid 入口即可发布预览；只要控制器正在接收相机数据，尚未启动策略时也有画面。

仿真验证使用三个终端（均在项目根目录，不需激活 Python 环境）：

```bash
# 终端一：外部 MuJoCo 项目仍需安装其仿真依赖
python3 apps/simulation/sim.py --mujoco-root /path/to/unitree_mujoco --domain 1
# 终端二：使用重新编译后的 Parkour 或 Hybrid 控制器
./g1 parkour sim
# 终端三：监控与仿真使用相同 DDS 域和网卡
python3 apps/monitor/joint_monitor.py --domain 1 --interface lo --host 127.0.0.1 --port 8765
```

浏览器打开 `http://127.0.0.1:8765/`。页面可分别开关「裁剪」「填洞」「高斯滤波」「归一化」，展开「处理参数」可修改四边裁剪量、填洞阈值/半径、滤波核/σ、深度上下限。「恢复预览默认值」读取项目 `apps/parkour/config/realsense_parkour.json` 的参数；缺省为裁剪上/下/左/右 18/0/16/16、填洞阈值 0.2 m、半径 3、滤波关闭、归一化深度范围 0～2.5 m。默认输出 32×18，关闭裁剪显示 64×36。

处理顺序为缩小到 64×36 → 可选裁剪 → 可选 `INPAINT_NS` 填洞 → 可选高斯滤波 → 可选裁剪深度范围并归一化。选择灰度或伪彩色显示，指向像素可查看数值。关闭归一化时数值为米，开启时为 0～1；配色始终按设置的深度范围映射。这里只处理预览副本，网页操作不会修改模型配置、相机采集或 policy 输入，也不用于修改策略的历史帧与延迟。

为了允许关闭裁剪并重做处理，传输的是**未裁剪的 64×36 米制 float32 深度图**，不是归一化后的 32×18 图；数据约 9 KB/帧。实际相机 480×270 的最近邻缩小、序列化和 UDP 发送在独立线程完成；控制循环只以 `try_lock` 提交最新帧引用，锁忙则丢弃预览。接收端只缓存最新帧，过期 1 秒显示过期，未收到数据时显示等待。UDP 失败、monitor 退出或端口占用均不停止 policy。

UDP 默认仅走 `127.0.0.1:9871`。更换端口时，在控制器和 monitor 两个终端使用相同环境变量，例如：

```bash
G1_DEPTH_PREVIEW_PORT=9872 ./g1 parkour sim
G1_DEPTH_PREVIEW_PORT=9872 python3 apps/monitor/joint_monitor.py --domain 1 --interface lo
```

`G1_DEPTH_PREVIEW_PORT=0` 可关闭控制器的预览发送，monitor 也支持 `--preview-port 0` 关闭接收。多个 monitor 同时运行时应分配不同端口。浏览器仍只连接 monitor 的 HTTP 端口。预览处理依赖 NumPy 和 OpenCV，机器人配置脚本已安装；其它环境可安装 `apps/monitor/requirements.txt`。

```bash
# 无需相机/DDS，验证 UDP 协议、C++ 发送端和全部 16 种处理组合
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

上述深度处理测试需要 NumPy、OpenCV，C++ 发送端测试需要 g++；HTTP/UDP 测试只使用本机回环地址。

## 深度数值录制（压缩、无损）

深度预览下方点击「开始深度录制」，默认 5 FPS，可选择 5/10/15/20/25/30 FPS。选 5 FPS 最省空间；开启裁剪默认只存 32×18。发送链路已提高帧率以支持 30 FPS 录制，不额外打开相机。源帧率不足时只保存新帧，页面显示实际录制 FPS，不通过重复帧凑数。开始时固定当前裁剪、填洞、滤波、归一化和所有数值参数；之后改变网页预览不会改动正在录制的数据。停止后选择文件下载。关闭/刷新网页仍持续录制，退出 monitor 时保存并结束。

「同时保存 MP4」默认开启，会同步生成同名 H.264 `.mp4`，按开始时的灰度/伪彩色保存；可关闭它以节省空间。视频使用原生小分辨率、单线程编码、CRF 32 压缩，无音轨；尺寸为奇数时仅在视频边缘补齐为偶数，数值数据尺寸不变。停止后分别点击「下载深度数据」「下载视频」。视频和数值文件保存相同的新帧，视频以所选恒定 FPS 播放；若源帧丢失或暂停，真实采集时序以数值文件的时间戳为准。MP4 需正常停止完成封装；异常退出可能使视频无法播放，数值文件仍可尝试尾部恢复。

视频依赖支持 `libx264` 的 FFmpeg；机器人依赖安装脚本已加入 `ffmpeg`。未安装编码器时页面会提示，关闭视频仍可录制无损深度。视频是有损可视化，不代替数值文件用于深度分析。

文件位于 monitor 的记录目录（默认 `~/logs/`），后缀 `.g1depth`。保存的是**当前处理结果的 float32 数值**，逐位无损，采用帧间 XOR 差分、字节重排和 zlib 压缩，不存配色/放大后的画面，也不把像素展开成 CSV。参数和图像尺寸只在文件头保存一次，每帧保存发送端会话、序号、原采集时间、接收 UTC 时间、来源和校验值。归一化开启时单位为 0～1；如需米制深度，开始录制前关闭归一化。已执行的裁剪/填洞/滤波与深度裁限无法从记录结果撤销。

32×18、5 FPS 的纯 float32 像素在压缩前约 675 KiB/分钟，压缩后取决于场景和运动。降低录制帧率可进一步减小体积。写盘独立于 UDP 接收线程，每秒同步磁盘；过期/重复帧不反复保存，恢复接收后自动继续。开始前至少需 100 MB 可用空间，低于 10 MB 或写入失败时停止并报告错误。

无需激活 Python 环境即可检查文件；导出 `.npz` 时需要已安装 NumPy：

```bash
python3 apps/monitor/depth_recording.py ~/logs/g1-depth-实际文件名.g1depth
python3 apps/monitor/depth_recording.py ~/logs/g1-depth-实际文件名.g1depth --export /tmp/depth.npz
```

`.npz` 包含 `depth`（帧数×高度×宽度）、`captured_at`、`received_at_unix`、`session`、`sequence`、`source` 和 JSON 字符串 `metadata`。大文件也可流式读取，避免一次载入全部帧：

```python
import numpy as np
from apps.monitor.depth_recording import read_recording

records = read_recording('recording.g1depth')
metadata = next(records)
for frame in records:
    depth = np.frombuffer(frame['data'], dtype='<f4').reshape(metadata['height'], metadata['width'])
    # depth 是此帧的精确 float32 数值，单位见 metadata['units']
```

异常断电可能损失最后尚未同步的数据；文件尾部不完整时可加 `--recover` 读取所有完整帧。`.npz` 导出会将所有帧载入内存，长记录建议使用上述流式接口。

## CSV 数据记录

网页顶部点击「开始新记录」，数据会持续写入机器人磁盘。点击「停止记录」会排空写入队列并同步文件，之后选择记录文件并点击「下载 CSV」。已有文件在监控程序重启后仍可列出和下载，不自动删除。

默认保存目录为 `~/logs/`，位于项目外；直接启动 Python 后端时可用 `--record-dir /path/to/recordings` 自定义。网页会显示实际目录、记录帧数和写入错误。同一个监控服务的所有网页共享记录状态；关闭、刷新网页或网络中断不会停止后端记录，退出监控程序会停止并保存。

每个后端采样帧对应一行，包含 UTC 接收时间、样本编号、监控时间、控制使能/空跑状态、全部 29 个关节的实测角度/速度/力矩/双通道温度、目标值/误差/Kp/Kd、Pelvis IMU 和 Torso IMU（含新鲜度标记）。目标不可用时相应字段留空。温度 0/1 对应 `LowState.motor_state[i].temperature[0/1]`，CSV 列名分别以 `temperature_0_c`、`temperature_1_c` 结尾；缺失或无效温度留空。CSV 固定使用 rad、rad/s、Nm、°C、m/s²，布尔值用 0/1，带 UTF-8 BOM；显示暂停、关节勾选和角度单位切换均不影响记录。

磁盘写入使用独立线程与有界队列，每秒 flush/fsync，正常停止时再同步一次。突然断电仍可能丢失最近尚未同步的数据。可用空间不足 100 MB 时不开始记录；写入中低于 10 MB、队列溢出或磁盘写入失败时停止并显示错误及已知丢帧数。采样上限仍由 `--rate` 决定（默认 50 Hz）。

无需 DDS 或机器人即可运行记录功能测试：

```bash
python3 -m unittest discover -s tests -p test_monitor_recording.py -v
```

## 接口

- `/`：网页。
- `/api/joints`：当前29关节、IMU、数据年龄、输出和空跑状态。
- `/api/stream`：SSE实时数据流。
- `GET /api/depth-recordings`：深度数值录制状态及文件列表。
- `POST /api/depth-recordings/start`、`POST /api/depth-recordings/stop`：开始/停止深度数值录制。
- `GET /api/depth-recordings/download?name=文件名`：下载压缩深度数据。
- `GET /api/depth/config`：预览默认参数与 UDP 端口。
- `GET /api/depth?crop=1&inpaint=1&filter=0&normalize=1`：按当前网页选项处理最新深度副本。
- `GET /api/recordings`：记录状态和文件列表。
- `POST /api/recordings/start`、`POST /api/recordings/stop`：开始/停止记录（同源 JSON 请求）。
- `GET /api/recordings/download?name=文件名`：下载已停止写入的 CSV。

监控的Enable是本控制程序的输出状态，不能据此判断机器人电源或其他服务是否在控制。
本次同步保留机器人上已运行的网页实现，只调整静态文件目录和启动路径；
机器人当前仍可继续运行原位置的服务，无需为同步重启它。
