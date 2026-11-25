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
- `GET /api/recordings`：记录状态和文件列表。
- `POST /api/recordings/start`、`POST /api/recordings/stop`：开始/停止记录（同源 JSON 请求）。
- `GET /api/recordings/download?name=文件名`：下载已停止写入的 CSV。

监控的Enable是本控制程序的输出状态，不能据此判断机器人电源或其他服务是否在控制。
本次同步保留机器人上已运行的网页实现，只调整静态文件目录和启动路径；
机器人当前仍可继续运行原位置的服务，无需为同步重启它。
