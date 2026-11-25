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

## 接口

- `/`：网页。
- `/api/joints`：当前29关节、IMU、数据年龄、输出和空跑状态。
- `/api/stream`：SSE实时数据流。

监控的Enable是本控制程序的输出状态，不能据此判断机器人电源或其他服务是否在控制。
本次同步保留机器人上已运行的网页实现，只调整静态文件目录和启动路径；
机器人当前仍可继续运行原位置的服务，无需为同步重启它。
