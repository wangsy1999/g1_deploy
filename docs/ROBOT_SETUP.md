# 一键配置机器人（自动下载依赖）

适用当前 G1 开发电脑：Linux ARM64/aarch64，原依赖流程曾在 Ubuntu 20.04 验证；新增的原生相机需单独验证。不会自动 SSH/SCP/rsync，不启动运动控制，不改系统时间或系统 glibc。

## 使用

1. 将项目复制或克隆到机器人，例如 `/home/unitree/g1_deploy`。无需预先复制依赖缓存；不要复制 Windows 的 `build/`、`install/` 或另一目录的 `.deps/`。
2. 在机器人上执行：

```bash
cd ~/g1_deploy
bash scripts/setup_robot.sh
```

默认检查并自动安装缺失的 Ubuntu 编译包；已安装的软件包跳过，只有缺包时才调用 sudo/apt，需要可用软件源、网络和管理员权限。系统依赖保存在系统中，SDK 和 Python 环境保存在机器人用户目录 `~/.local/share/g1-deploy/aarch64/`，独立于整个源码目录。删除项目后重新拉取源码，仍会自动复用；不需要保留项目里的 `.deps/`。

Python ARM64 发行包优先通过 `registry.npmmirror.com` 下载。若下载失败，脚本会依次尝试 `gh-proxy.org` 和 `ghproxy.net`。其他从 GitHub 获取的依赖默认也使用这两个代理。设置 `G1_GITHUB_MIRROR` 后，脚本会在指定的 HTTPS 镜像地址后拼接完整的 GitHub 下载链接。

pip 联网安装默认使用 `https://pypi.tuna.tsinghua.edu.cn/simple`。已设置 `PIP_INDEX_URL` 时使用该地址，`G1_PIP_INDEX_URL` 的优先级更高。可在单次部署时切换到中科大镜像。

```bash
G1_PIP_INDEX_URL=https://mirrors.ustc.edu.cn/pypi/simple bash scripts/setup_robot.sh
```

ONNX Runtime 1.18.1 ARM64 C/C++ 归档使用 npmmirror，下载失败或校验不通过时尝试 `hf-mirror.com`。两个地址使用同一个固定的 SHA-256，在线下载缓存也会在解压前校验。

私有 glibc 2.35 优先使用清华 Ubuntu Ports 镜像，失败时依次尝试中科大、阿里云和 Ubuntu 官方源。脚本从同一镜像读取 Jammy ARM64 索引和软件包，按索引中的 SHA-256 校验后解压到依赖目录。

原生 RealSense SDK 在系统中找不到时，从官方 v2.55.1 源码编译到 `~/.local/share/g1-deploy/aarch64/realsense`，使用 RSUSB 后端，不打内核补丁。部署脚本会提前从 GitHub 镜像下载并校验 nlohmann/json 3.11.3 源码归档，RealSense 的 CMake 配置直接使用本地缓存。首次源码编译需要等待，`--jobs N` 可调整并行数。

禁止脚本调用 apt（已手动安装系统依赖，项目依赖仍可联网下载）时：

```bash
bash scripts/setup_robot.sh --no-system-deps
```

额外验证 C++ 原生相机出图（只读，不启动控制程序）：

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

关节网页：http://192.168.0.200:8765/ 。Parkour 使用进程内 C++ 相机采集，不自动启动相机网页。独立网页预览须在控制器退出后运行。

原配遥控器：X 复位、A 进入策略、Start 真实接管、Select 停止退出。
空跑用 X → 等待复位完成 → A。真实控制需要现场保护，Start → X → 等待复位完成 → 摇杆回中 → A。
配置已将左右 hip yaw 限制为 ±90°，其它关节保持原配置。

## 下次全新编译

删除 `build/`、`.deps/`，甚至整个源码目录后，重新拉取项目并运行 `bash scripts/setup_robot.sh` 即可。已安装的依赖在用户目录中，会自动复用；版本匹配的 Python 包和已安装的宇树 SDK 跳过安装，仍会验证关键模块能否导入。依赖齐全时无需联网下载。监控与相机的 shell/Python 入口会自动使用这个环境，无需 `source` 激活 Python 环境。

默认依赖目录为 `~/.local/share/g1-deploy/aarch64/`，下载缓存为 `~/.cache/g1-deploy/robot-arm64/`。它们在项目外，应保留在机器人上。切换用户或重装机器人系统后需要重新安装。可通过 `G1_DEPS_DIR` 和 `G1_CACHE_DIR` 环境变量自定义目录；使用自定义依赖目录时，配置、构建和启动命令都应使用同一 `G1_DEPS_DIR`。

旧版本已经装在项目 `.deps/` 内的依赖不会整体搬移（Python 虚拟环境包含绝对路径）。新脚本会在固定目录安装一次，之后可删除旧 `.deps/`。
也可只重新构建 C++：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DG1_REQUIRE_REALSENSE=ON -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j4
cmake --install build
```

CMake 会自动搜索固定依赖目录下的 Unitree SDK、RealSense 和 ONNX Runtime；显式指定的 SDK 路径优先，同时兼容旧项目 `.deps/`。上述命令在 `build/` 不存在时同样适用，不触发依赖安装。

## 原生相机依赖

机器人构建强制要求 `librealsense2` C++ 开发库，配置脚本设置 `G1_REQUIRE_REALSENSE=ON`。SDK 可安装到系统，或放在 `~/.local/share/g1-deploy/aarch64/realsense` 并包含 CMake 配置、头文件及本机 ABI 兼容的库。旧离线缓存中的 `pyrealsense2` wheel 和私有 glibc 仅服务于独立 Python 预览，不满足 C++ 依赖；离线迁移须额外准备原生 SDK。脚本优先检测系统或 `~/.local/share/g1-deploy/aarch64/realsense` 中的 SDK，缺失时自动下载源码编译，无需配置 RealSense 专用 apt 软件源。离线模式下需提前提供该库。

实机启动 `./g1 parkour real eth0` 或 `./g1 hybrid real eth0` 即可，先关闭任何占用相机的独立预览。只读相机诊断使用 `./bin/g1_camera_check`，不需要网页端口或 Python 相机服务。

## 配置内容

以下路径均相对于固定依赖目录 `~/.local/share/g1-deploy/aarch64/`：

- `runtime`：Python 3.10.15；`python`：独立虚拟环境。
- NumPy 1.26.4、CycloneDDS Python 0.10.2、宇树 Python SDK、OpenCV headless 4.11.0.86、RealSense Python 2.55.1.6486。
- `unitree-sdk2`、`cyclonedds`、`onnxruntime`：ARM64 原生库；缺失时自动下载或编译，构建不依赖其它项目目录。
- `glibc-2.35`：Ubuntu 官方 ARM64 glibc 2.35。只有相机专用 Python 入口通过它的动态加载器启动；不修改 `/lib`、`/usr/lib`、系统加载器或全局环境变量。
- `apps/realsense/python3-realsense.sh`：相机专用入口模板；由配置脚本复制到固定依赖目录中的 `python/bin/python3-realsense`。网页监视器与控制程序继续使用原系统库。
- 编译 Release、安装到 `install/`，并完成两套模型的离线推理检查。已有安装会备份到 `deployment-backups/`，发现本项目控制程序正在运行则停止配置，需先由操作者退出程序。
- 日志及包清单位于固定依赖目录中的 `setup-logs/`。

## 下载缓存、离线运行与迁移

默认下载保存在 `~/.cache/g1-deploy/robot-arm64/downloads/`，源码及中间构建目录位于固定依赖目录中的 `sources/`、`build/`。固定版本为 Python 3.10.15、CycloneDDS 0.10.2、ONNX Runtime 1.18.1 和 RealSense 2.55.1；Unitree SDK 首次从官方 main（C++）/master（Python）分支下载，保留下载的归档以复用。所有候选地址都下载失败时，脚本会停止。修复网络后可重新运行。

完全离线运行（禁止自动下载和 apt）：

```bash
bash scripts/setup_robot.sh --offline
```

依赖已安装齐全时可直接离线重跑。首次离线配置仍需下述旧格式缓存及系统编译包、原生相机 SDK；在线下载缓存不等于完整离线安装包，尤其 pip 包及源码的构建依赖需要另行准备。

兼容已有的离线缓存（例如旧项目 `.deploy-cache/robot-arm64/`，用 `--cache` 指定），包含 Python、SDK 和运行库归档、Python wheels、glibc deb，以及 SHA256SUMS。首次安装或补齐缺失的项目依赖时，配置脚本先校验缓存完整性，再离线安装；依赖齐全时跳过缓存读取。可用 `--cache /path/to/robot-arm64` 指定其它位置。

缓存来自官方发行包及当前机器人已验证的原生库。该缓存仅适用于 ARM64，基础系统需有兼容的 C++/OpenCV/yaml-cpp/Boost 依赖；其它架构需另备依赖包。

`.deps/`、`.deploy-cache/`、`build-sdk/` 和安装/构建目录已加入 Git 忽略。Git 保存配置流程和启动脚本，不保存平台二进制及本机虚拟环境。若使用 Git 将源码带到另一台联网机器，脚本会自动下载缺失的依赖；离线机器则需要另行准备缓存。不得把旧目录的虚拟环境整体移动后直接使用，应在目标目录重新运行配置脚本。

此脚本不设置开机自启，也不执行电机接管。相机采集模式为 480×270、60 Hz 深度，实际采集频率由页面/日志统计。
