# 并联驱动单元控制框架

这个工程现在按你已经在实物上跑通的 `single_motor_ctrl` 思路重构成了“可现场联调”的并联驱动单元控制工程，核心目标是：

- Linux 主机直接连接控制盒和两台电机
- 保留并联机构的关节空间运动学映射
- 提供和单电机 CLI 类似的实机联调命令
- 把实物验证过的启动、使能、下发、反馈、停机链路迁移到双电机场景

工程仍然基于仓库里的两类参考资料：

- `linux_sdk/`：厂商提供的 Linux SDK、头文件和示例程序
- `parallel_drive_unit_kinematics.png`：并联驱动单元的运动学关系图

默认假设：

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
- `motor_backend`：抽象底层驱动接口，屏蔽 `mock` 和厂商 SDK 差异。
- `vendor_sdk_backend`：基于 Linux SDK 实现双电机硬件后端，启动/停机序列对齐 `single_motor_ctrl` 的实物路径。
- `parallel_drive_unit`：封装连接、模式切换、清故障、使能/失能、置零、状态读取和关节命令发送。
- `main.cpp`：提供 Linux 实机可用的 CLI 与交互式 shell，而不只是单一 demo。

## 构建

### 1. 本地 dry-run / 逻辑验证

```bash
cmake -S . -B build
cmake --build build
./build/parallel_drive_app --backend mock info
./build/parallel_drive_app --backend mock set-pd-target 0.2 0.1 0.0 0.0
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
./build/parallel_drive_app --config config/parallel_drive_unit.ini --backend vendor info
```

## 配置项

`config/parallel_drive_unit.ini` 的关键项如下：

- `[network]`：主板 ID、本机 IP、本地端口、主板 IP、主板端口
- `[motor2]` / `[motor3]`：CAN ID 与 CAN line
- `[control]`：默认控制模式、PID/PD 参数、上电是否自动清错和使能
- `[feedback]`：是否开启 SDK 快速上报模式
- `[runtime]`：使用 `mock` 还是 `vendor` 后端、控制循环频率
- `[demo]`：示例目标位姿和轨迹时长

推荐默认把：

- `runtime.backend=vendor`
- `control.command_mode=pd`

用于 Linux 实机控制；本地开发时再切回 `mock`。

## CLI 指令

```bash
./build/parallel_drive_app --config config/parallel_drive_unit.ini info
./build/parallel_drive_app --config config/parallel_drive_unit.ini state
./build/parallel_drive_app --config config/parallel_drive_unit.ini clear-fault
./build/parallel_drive_app --config config/parallel_drive_unit.ini enable
./build/parallel_drive_app --config config/parallel_drive_unit.ini disable
./build/parallel_drive_app --config config/parallel_drive_unit.ini zero
./build/parallel_drive_app --config config/parallel_drive_unit.ini mode pd
./build/parallel_drive_app --config config/parallel_drive_unit.ini set-joint-pos 0.10 0.02
./build/parallel_drive_app --config config/parallel_drive_unit.ini set-joint-vel 0.10 -0.05
./build/parallel_drive_app --config config/parallel_drive_unit.ini set-joint-cur 0.30 0.10
./build/parallel_drive_app --config config/parallel_drive_unit.ini set-pd-target 0.20 0.05 0.00 0.00
./build/parallel_drive_app --config config/parallel_drive_unit.ini monitor 20 100
./build/parallel_drive_app --config config/parallel_drive_unit.ini shell
```

## Linux 实机联调顺序

推荐第一次上机按下面顺序操作：

1. `info`
2. `state`
3. `clear-fault`
4. `shell`
5. 在 shell 里执行 `mode pd`、`enable`
6. 先用小步命令测试，例如 `set-joint-pos 0.05 0.00`
7. 用 `monitor 20 100` 观察两台电机和关节反馈

## 控制流程

`parallel_drive_app` 的启动流程如下：

1. 读取配置文件并创建后端实例。
2. 初始化主板连接与两台电机对象。
3. 根据命令切换控制模式，并下发安全预充指令。
4. 需要驱动时执行清故障、使能或置零。
5. 将关节空间指令 `theta1/theta2` 映射为 `m2/m3`。
6. 在 `pd` 模式下使用 `robot_motor_set_pos()` 为两台电机写入位置/速度/电流组合指令，再调用 `robot_motor_set_big_pose()` 同步发送。
7. 读取两台电机 PVCT/编码器反馈，并按正运动学回算出 `theta1/theta2`。
8. 停机时先卸载当前模式下的输出，再执行失能和资源释放。

## 当前假设

- 主板网络参数默认沿用 SDK 示例中的典型值：`local_ip=192.168.3.245`、`remote_ip=192.168.3.11`、`local_port=15021`、`remote_port=14999`、`board_id=0xFD`
- 这部分在不同控制盒上可能不同，因此我把它们全部外置到了配置文件里
- `runtime.synchronized_send` 目前仅保留作兼容字段；在 `pd` 模式下硬件后端会始终走同步大包发送
- 当前仓库内测试只覆盖了 `mock` 路径；真实 Linux 控制效果仍需你上机验证网络、CAN ID、零位和机械安全边界
