# G1 29DOF 硬件控制与验证边界

低层通信参考 [Unitree SDK2 G1示例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_ankle_swing_example.cpp)。
使用DDS domain 0、PR关节坐标，读取LowState主IMU；官方示例将它标为pelvis。
不使用MuJoCo真值、torso射线相机、深度编码器或额外的IMU固定旋转。

## 接管过程

程序启动时只接收并检查状态。显式按键盘9或F710 Start后，通过MotionSwitcher请求释放原服务，
确认释放并检查是否仍有其他LowCmd发布者，成功后进入阻尼等待。
按p或F710 A才开始平滑复位；完成且各关节接近训练默认角后，r或Y进入策略。
低层开发状态准备参考 [官方实机部署示例](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_real/README.md)。
吊装、现场停止和恢复官方运动模式均按该机器人交付文档操作。

## 当前软件检查

配置的`hardware`段使用DDS关节顺序，逐关节边界来源为官方rev_1_0 URDF，须与实际硬件版本核对。

- LowState CRC及`mode_machine=5`约束，29个关节的电机故障、温度、位置和速度检查。
- 主IMU四元数、角速度及倾角检查。默认状态超时100ms，倾角上限1rad，温度阈值70°C。
  这些是本项目设置，不是厂商推荐阈值。状态验证失败锁定故障，需重启。
- 目标角度边界余量0.01rad，默认目标变化率上限10rad/s，并受配置的URDF速度边界约束。
- 用当前q/dq与PD估算指令力矩，超过配置effort则停止。它不等于驱动层力矩限幅，
  也未建立PR/AB耦合电机力矩模型。
- 推理返回后检查是否超过策略周期。推理与控制同线程，不是独立看门狗或硬实时系统。

## 停止行为

空格、Ctrl+C、SIGHUP/SIGTERM、手柄B、输入读取失败或软件检查故障触发停止。
已接管后发送mode=1、Kp=0、tau=0、dq=0、Kd=1的阻尼命令约0.5秒，然后退出。
DDS发送成功不代表控制板已确认执行；网络故障、SIGKILL、进程卡死或断电无法保证软件退出路径执行。

机器人原配遥控器A按钮在本项目中为阻尼停止请求，依据
[官方遥控数据布局](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/gamepad.hpp)解析。
F710直接从Linux读取，A为复位；它和LowState的原配遥控器是两个独立输入来源。
这不是对机器人固件硬件急停按键的声明。退出后不会自动恢复原官方运动服务。

没有真实机器人验收前，只能确认代码及离线检查可运行，不能确认实机行走稳定性或停止可靠性。
