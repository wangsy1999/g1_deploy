# 一键配置机器人（手动复制文件）

适用当前 G1 开发电脑：Linux ARM64/aarch64，已在 Ubuntu 20.04 验证。不会自动 SSH/SCP/rsync，不启动运动控制，不改系统时间或系统 glibc。

## 使用

1. 手动把 `D:\g1_deploy` 复制到机器人，例如 `/home/unitree/g1_deploy`。同时复制 `.deploy-cache/robot-arm64/`，离线依赖约 100 MB；不要复制 Windows 的 `build/`、`install/` 或另一目录的 `.deps/`。
2. 在机器人上执行：

```bash
cd ~/g1_deploy
bash scripts/setup_robot.sh
```

有缺失的 Ubuntu 编译依赖时可使用下面的命令；只有确实缺少软件包时才调用 sudo/apt，需要网络和管理员密码：

```bash
bash scripts/setup_robot.sh --install-system-deps
```

额外验证真实相机出图、网页和 UDP（只读，不启动控制程序）：

```bash
bash scripts/setup_robot.sh --verify-camera
```

仅配置依赖，不编译控制程序：

```bash
bash scripts/setup_robot.sh --deps-only
```

安装后：

```bash
cd install
./g1 check
./g1 parkour check
./g1 monitor eth0 0.0.0.0 8765
# 另开终端；启动默认空跑：
./g1 parkour real eth0
```

关节网页：http://192.168.0.200:8765/ 。Parkour 启动后的相机网页：http://192.168.0.200:8766/ 。

原配遥控器：X 复位、A 进入策略、Start 真实接管、Select 停止退出。
空跑用 X → 等待复位完成 → A。真实控制需要现场保护，Start → X → 等待复位完成 → 摇杆回中 → A。
配置已将左右 hip yaw 限制为 ±90°，其它关节保持原配置。

## 配置内容

- 项目 `.deps/runtime`：Python 3.10.15；`.deps/python`：独立虚拟环境。
- NumPy 1.26.4、CycloneDDS Python 0.10.2、宇树 Python SDK、OpenCV headless 4.11.0.86、RealSense Python 2.55.1.6486。
- `.deps/unitree-sdk2`、`.deps/cyclonedds`、`.deps/onnxruntime`：已在此机器人验证的 ARM64 原生库；构建不再依赖其它项目目录。
- `.deps/glibc-2.35`：Ubuntu 官方 ARM64 glibc 2.35。只有相机专用 Python 入口通过它的动态加载器启动；不修改 `/lib`、`/usr/lib`、系统加载器或全局环境变量。
- `scripts/python3-realsense.sh`：相机专用入口模板；由配置脚本复制到 `.deps/python/bin/python3-realsense`。网页监视器与控制程序继续使用原系统库。
- 编译 Release、运行项目测试、安装到 `install/`，并完成两套模型的离线推理检查。已有安装会备份到 `deployment-backups/`，发现本项目控制程序正在运行则停止配置，需先由操作者退出程序。
- 日志及包清单位于 `.deps/setup-logs/`。

## 离线缓存与迁移

`.deploy-cache/robot-arm64/` 当前已在本地准备齐全，包含 Python、SDK 和运行库归档、Python wheels、glibc deb，以及 SHA256SUMS。配置脚本先校验完整性，再离线安装。可用 `--cache /path/to/robot-arm64` 指定其它位置。

缓存来自官方发行包及当前机器人已验证的原生库。该缓存仅适用于 ARM64，基础系统需有兼容的 C++/OpenCV/yaml-cpp/Boost 依赖；其它架构需另备依赖包。

`.deps/`、`.deploy-cache/`、`build-sdk/` 和安装/构建目录已加入 Git 忽略。Git 保存配置流程和启动脚本，不保存平台二进制及本机虚拟环境。若使用 Git 将源码带到另一台机器，需要另外手动复制缓存；仅复制源码不足以离线配置。不得把旧目录的虚拟环境整体移动后直接使用，应在目标目录重新运行配置脚本。

此脚本不设置开机自启，也不执行电机接管。相机采集模式为 480×270、60 Hz 深度，实际采集频率由页面/日志统计。
