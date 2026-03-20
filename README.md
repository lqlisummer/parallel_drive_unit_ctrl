# 并联驱动单元控制框架

这个工程基于仓库里的两类参考资料搭建：

- `linux_sdk/`：厂商提供的 Linux SDK、头文件和示例程序。
- `parallel_drive_unit_kinematics.png`：并联驱动单元的运动学关系图。

本工程把参考代码整理成一个可扩展的控制框架，默认假设：

- 电机 `m2` 的 CAN ID 为 `2`
- 电机 `m3` 的 CAN ID 为 `3`
- 两台电机都挂在 `CAN line 1`
- 主板通过以太网与上位机通信，再通过 CAN 总线串联控制两台电机

## 运动学模型

并联驱动单元的正逆解按参考图实现：

```text
theta1 = (m2 + m3) / 2
theta2 = (m2 - m3) / 2

m2 = theta1 + theta2
m3 = theta1 - theta2
```

速度和前馈努力项沿用同一组线性映射，便于把输出关节空间命令统一下发到两台电机。

## 项目结构

```text
.
├── CMakeLists.txt
├── config/
│   └── parallel_drive_unit.ini
├── include/pdu/
│   ├── config.hpp
│   ├── kinematics.hpp
│   ├── mock_motor_backend.hpp
│   ├── motor_backend.hpp
│   ├── parallel_drive_unit.hpp
│   ├── types.hpp
│   └── vendor_sdk_backend.hpp
├── src/
│   ├── backend_factory.cpp
│   ├── config.cpp
│   ├── kinematics.cpp
│   ├── main.cpp
│   ├── mock_motor_backend.cpp
│   ├── parallel_drive_unit.cpp
│   └── vendor_sdk_backend.cpp
└── tests/
    └── parallel_drive_tests.cpp
```

## 模块说明

- `kinematics`：负责关节空间与电机空间的正逆变换。
- `motor_backend`：抽象底层驱动接口，屏蔽 mock 和厂商 SDK 差异。
- `vendor_sdk_backend`：把 `RobotControl.h` 和 `CTypes.h` 封装成 RAII 风格的硬件后端。
- `parallel_drive_unit`：实现启动流程、故障清除、使能、零位控制、关节命令发送和状态反馈。
- `main.cpp`：提供一个可直接运行的轨迹入口，默认做“从当前位姿平滑移动到目标位姿，再保持”的演示。

## 构建

### 1. 本地 dry-run / 逻辑验证

```bash
cmake -S . -B build
cmake --build build
./build/parallel_drive_app --config config/parallel_drive_unit.ini --backend mock
./build/parallel_drive_tests
```

### 2. Linux 实机接入

前提：

- 在 Linux 环境编译
- `linux_sdk/lib/output/libMotorDrive.so` 可用
- 根据控制盒网络配置修改 `config/parallel_drive_unit.ini`

构建命令：

```bash
cmake -S . -B build -DPDU_BUILD_VENDOR_BACKEND=ON
cmake --build build
./build/parallel_drive_app --config config/parallel_drive_unit.ini --backend vendor
```

## 配置项

`config/parallel_drive_unit.ini` 的关键项如下：

- `[network]`：主板 ID、本机 IP、本地端口、主板 IP、主板端口
- `[motor2]` / `[motor3]`：CAN ID 与 CAN line
- `[control]`：控制模式、PID/PD 参数、上电是否自动清错和使能
- `[feedback]`：是否开启 SDK 快速上报模式
- `[runtime]`：使用 `mock` 还是 `vendor` 后端、控制循环频率、是否用大包同步发送
- `[demo]`：示例目标位姿和轨迹时长

## 控制流程

`parallel_drive_app` 的启动流程如下：

1. 读取配置文件。
2. 创建后端实例。
3. 初始化主板与两台电机。
4. 根据配置清除故障并使能。
5. 将目标关节角 `theta1/theta2` 逆解为 `m2/m3`。
6. 在 `pd_sync` 模式下用 `robot_motor_set_pos()` 为两台电机写入目标，再调用 `robot_motor_set_big_pose()` 做同步发送。
7. 读取两台电机 PVCT 反馈，并按正运动学回算出 `theta1/theta2`。

## 当前假设

- 主板网络参数默认沿用 SDK 示例中的典型值：`local_ip=192.168.3.245`、`remote_ip=192.168.3.11`、`local_port=15021`、`remote_port=14999`、`board_id=0xFD`
- 这部分在不同控制盒上可能不同，因此我把它们全部外置到了配置文件里
- 如果你后面愿意，我还可以继续把这一版扩展成带状态机、急停、限位、心跳和轨迹跟踪器的工业化版本
