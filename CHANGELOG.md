# 改动总结

## 2026-10-19：机载空跑、原配遥控器与网页监控整合

本次改动基于独立仓库 `g1_deploy`，将机器人上的调试版本同步回本地，
并将原先位于 `unitree_mujoco_web` 的只读网页监控收入同一仓库。
模型和部署配置的内容保持原样，未将SSH密码、依赖安装目录或机载编译产物纳入版本管理。

### 1. 默认空跑与真实输出分离

- `./g1 run <网卡>`默认关闭输出，不按`9`即可按`p`预览复位、按`r`执行策略推理。
- 空跑读取真实LowState，不模拟机器人跟随目标；姿态或硬件就绪异常显示为警告。
  CRC、机型、有限数据和状态新鲜度仍须通过检查。
- 空跑不创建`rt/lowcmd`发布器、不释放官方运动服务，退出时不发电机阻尼命令。
  目标仅发送到`rt/g1/dry_run_target`，网页显示`OFF · 空跑`。
- 按`9`或所选手柄的Start后先执行完整硬件检查，再申请接管。
  接管会清空预览和策略历史，先进入阻尼等待，必须重新复位后才能运行真实策略。
- 真实输出继续执行关节、温度、倾角、状态超时、目标变化率及估算PD力矩检查。

主要文件：`src/main.cpp`、`include/g1_hardware.h`、`scripts/g1.sh`。

### 2. 宇树原配遥控器接入

新增`./g1 run <网卡> remote`入口，解析LowState中的40字节遥控数据，
与F710输入互斥。最终采用以下映射：

| 输入 | 行为 |
| --- | --- |
| Start | 接管并启用真实输出，与键盘9相同 |
| X | 复位；未接管时仅预览 |
| A | 复位完成后进入策略，进入时摇杆须回中 |
| Select | 软件停止并退出；已接管时先发送约0.5秒阻尼 |
| 左摇杆 | 前后、横移 |
| 右摇杆左右 | 转向 |

进入策略后直接用摇杆控制速度，不需要按住R1；摇杆回中则速度归零，死区12%。
归一化输入为`[ly,-lx,-rx]`，再由YAML限制实际速度范围。
启动时按住的Start/X/A不会触发动作，Select始终优先停止；B、Y没有映射。
仅在显式选择原配遥控器模式时，将原有遥控器A停止位替换为Select停止位。
F710的RB门控及原有按键映射保留。

主要文件：`include/g1_unitree_remote.h`、`include/g1_gamepad.h`、
`src/main.cpp`、`tests/g1_unitree_remote_test.cpp`。

### 3. 网页监控并入同一仓库

- `web/joint_monitor.py`：只订阅DDS的Python后端。
- `web/static/joint_monitor.html`：关节表格、IMU、实际/目标曲线及空跑状态。
- `scripts/monitor.sh`：按脚本位置加载资源，支持源码目录及CMake安装目录。
- `web/requirements.txt`、`docs/WEB.md`：可选Python依赖和部署说明。
- `/api/joints`提供当前状态，`/api/stream`使用SSE推送，默认最高50Hz。
- CMake安装包新增`monitor`、`web/`及网页说明，支持`./g1 monitor <网卡> [地址] [端口]`。
  Python依赖由目标机器提供，C++控制程序仍无需Python运行时。

网页来源和保留的机载修改见`ORIGIN.md`。迁移只调整静态文件路径和启动方式，
不包含原仿真工程或无关机器人模型。

### 4. 仓库整理

- 控制源码、模型、配置等19个项目文件已从机器人同步并逐文件校验。
- 新增`.gitattributes`，Shell、Python和HTML使用LF，避免Windows换行破坏Linux启动脚本。
- 扩展`.gitignore`，排除构建、安装、日志、PID、Python依赖、诊断及备份目录。
- README补齐部署、网页、空跑、真实控制和原配遥控器操作步骤。

### 5. 验证记录

验证环境：Ubuntu 22.04 / aarch64 / Python 3.10，真机DDS domain 0，
本次设备网卡`enP8p1s0`。

| 项目 | 结果 |
| --- | --- |
| 机载Release编译、安装及运行库检查 | 通过 |
| C++测试：硬件、F710、原配遥控器、ONNX策略 | 4项通过 |
| 网页测试：HTML/API/SSE、目标优先级、过期状态 | 3项通过 |
| 源码和安装目录的网页启动、29关节实时流 | 通过 |
| 默认空跑复位、推理、速度键和退出 | 通过；DDS发现确认未创建电机指令或MotionSwitcher RPC发布器 |
| 原配遥控器实测摇杆 | 4轴覆盖−1至1，松手回零；约45秒采集4.68万帧，无非法轴值 |
| 原配遥控器模式真机空跑 | 读取真实输入并完成复位预览、策略推理及退出 |

上述自动化和空跑验证没有执行Start接管或真实行走。
操作者随后确认现场调试通过；本总结不补充未记录的行走工况、持续时间或急停验收结果。
Select是软件阻尼停止，不是硬件断电；无线链路是否失联不能仅凭持续更新的LowState判断。

### 使用入口

```bash
cd /home/unitree/g1_deploy/install
./g1 run enP8p1s0 remote
# 空跑：X → 等待目标插值完成 → A，不按Start/9
# 真实控制：Start → X → 等待Holding default pose → A
# 退出：Select；也可使用键盘空格或Ctrl+C
```

另开终端运行`./g1 monitor enP8p1s0 192.168.123.164 8765`，
浏览器访问`http://192.168.123.164:8765/`。完整步骤见`README.md`和`docs/WEB.md`。
