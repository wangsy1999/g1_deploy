# G1 29DOF 硬件控制与验证边界

低层通信参考 [Unitree SDK2 G1示例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_ankle_swing_example.cpp)。
使用DDS domain 0、PR关节坐标，读取LowState主IMU；官方示例将它标为pelvis。
不使用MuJoCo真值、torso射线相机、深度编码器或额外的IMU固定旋转。

## 接管过程

程序启动时默认空跑：不按9即可按p预览复位、按r进行策略推理，目标仅发往监控话题。
空跑只检查状态完整性与新鲜度，硬件就绪异常以警告显示；不模拟运动，也不证明目标可执行。
显式按键盘9或原配遥控器 Start 后，先执行完整硬件检查，再通过MotionSwitcher请求释放原服务，
确认释放并检查是否仍有其他LowCmd发布者，成功后进入阻尼等待。
接管时清除空跑复位和策略状态，进入阻尼等待。重新按p或原配遥控器 X 才开始平滑复位；
完成且各关节接近训练默认角后，r或原配遥控器 A 进入真实策略。
低层开发状态准备参考 [官方实机部署示例](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/README.md)。
吊装、现场停止和恢复官方运动模式均按该机器人交付文档操作。

## 当前软件检查

配置的`hardware`段使用DDS关节顺序，逐关节边界来源为官方rev_1_0 URDF，须与实际硬件版本核对。

- LowState CRC及`mode_machine=5`约束，29个关节的电机故障、温度和有限数值检查。
  实测关节位置、速度越界不导致自动退出；配置关节范围校验复位姿态，并可选择用于下发目标限位。
- 主IMU四元数、角速度及倾角检查。默认状态超时100ms，倾角上限1rad，温度阈值70°C。
  这些是本项目设置，不是厂商推荐阈值。状态验证失败锁定故障，需重启。
- `hardware.limit_target_position`当前配置为`true`（缺省键为`false`）；开启后，将实机下发目标角夹紧到`joint_lower/joint_upper`（DDS顺序，单位rad），不额外缩小范围。
  DWaQ 从 `apps/dwaq/config/g1_locomotion.json`、Parkour 从 `apps/parkour/config/g1_parkour.json` 各自读取 `hardware` 设置，仅在启动时读取；修改配置后重启对应程序。变化率始终不限制，指令有限值检查始终保留。
  旧配置中的`target_rate_rad_s`和`joint_velocity`字段不再读取。
- 不再根据q/dq与PD估算力矩，也不按估算值停止；旧`joint_effort`配置已移除。
  程序检查所有下发指令字段为有限数值，但不提供软件PD力矩幅值限制。
- 推理返回后检查是否超过策略周期。推理与控制同线程，不是独立看门狗或硬实时系统。

## 停止行为

空格、Ctrl+C、SIGHUP/SIGTERM、原配遥控器 Select（实机控制默认启用）、输入读取失败或软件检查故障触发停止。
已接管后发送mode=1、Kp=0、tau=0、dq=0、Kd=1的阻尼命令约0.5秒，然后退出。
DDS发送成功不代表控制板已确认执行；网络故障、SIGKILL、进程卡死或断电无法保证软件退出路径执行。

DWaQ 和 Parkour 实机控制默认使用 LowState 中的原配遥控器，F710 仅用于仿真。
`./g1 dwaq real <网卡>` 与 `./g1 parkour real <网卡>` 的遥控器操作一致：
Start接管、X复位、A策略、Select停止退出，摇杆直接控制速度，不需要按住R1。
该模式的硬件停止位检查使用Select；启动时按住的Start/X/A不触发，Select始终停止。
这不是对机器人固件硬件急停按键的声明。退出后不会自动恢复原官方运动服务。

没有真实机器人验收前，只能确认代码及离线检查可运行，不能确认实机行走稳定性或停止可靠性。

当前29关节目标范围均配置为`[-5,5]` rad，并启用`limit_target_position`。DWAQ的`actions.JointPositionAction.clip`与Parkour的`joint_clip_lower/upper`也设置相同范围，策略后处理在仿真和实机均生效；无目标变化率限制。
