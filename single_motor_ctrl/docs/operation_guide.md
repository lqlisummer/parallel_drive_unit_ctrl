# 单电机 CLI 控制操作流程

本文档面向已经把仓库部署到控制主机 `@items` 目录下的场景，假设项目目录为：

```bash
/items/single_motor_ctrl
```

如果你的实际目录名不同，只需要把下面命令中的路径替换成真实路径即可。

---

## 1. 登录控制主机并进入项目目录

```bash
cd /items/single_motor_ctrl
pwd
```

确认当前目录下至少包含以下内容：

```bash
CMakeLists.txt
configs/
include/
src/
linux_sdk/
motor_hardward_user_manual/
```

---

## 2. 确认 SDK 动态库存在

CLI 运行时依赖仓库内的 `libMotorDrive.so`，先检查是否存在：

```bash
ls -l linux_sdk/lib/output/
```

正常情况下应能看到：

- `libMotorDrive.so`
- `libMotorDrive.a`

如果库文件缺失，先不要继续调试，需先确认 SDK 是否完整拷贝到控制主机。

---

## 3. 修改控制参数配置文件

默认配置文件为：

```bash
configs/motor_cli.conf
```

编辑该文件，重点确认以下参数：

```ini
board.local_ip
board.local_port
board.remote_ip
board.remote_port
board.dev_id
board.fast_mode_enabled
board.fast_mode_hz
motor.can_id
motor.can_line_id
motor.pos_kp
motor.vel_kp
motor.vel_ki
motor.pd_kp
motor.pd_kd
```

当前这台电机的推荐配置是：

```ini
motor.can_id = 2
motor.can_line_id = 1
```

推荐修改方式：

```bash
vim configs/motor_cli.conf
```

### 参数说明

- `board.local_ip`：控制主机本机网口 IP。
- `board.local_port`：控制主机本地监听端口。
- `board.remote_ip`：控制盒/主板 IP。
- `board.remote_port`：控制盒/主板端口。
- `board.dev_id`：主板 ID，默认是 `0xFD`。
- `board.fast_mode_enabled`：是否开启快速反馈模式。
- `board.fast_mode_hz`：快速反馈频率，默认 `500`。
- `motor.can_id`：目标单电机 CAN ID；当前现场参数是 `2`。
- `motor.can_line_id`：目标电机所在 CAN 线号；当前现场参数是 `1`。
- `motor.pos_kp` / `motor.vel_kp` / `motor.vel_ki`：位置/速度环参数。
- `motor.pd_kp` / `motor.pd_kd`：PD 模式参数。

> 建议先保留默认 PID/PD 参数，只修改网络和电机地址，等基础通信打通后再调整控制参数。

---

## 4. 构建项目

在项目根目录执行：

```bash
cmake -S . -B build
cmake --build build -j4
```

构建完成后会生成：

```bash
build/motor_cli
```

如需确认 CLI 是否可执行，可以先查看帮助信息：

```bash
./build/motor_cli --help
```

---

## 5. 上电前检查

在连接真实电机前，建议先确认以下事项：

1. 控制盒已上电。
2. 控制主机与控制盒网络互通。
3. 电机供电正常。
4. 电机 CAN ID 与 `configs/motor_cli.conf` 中一致；当前这台电机请填写 `motor.can_id = 2`。
5. 电机机械端处于安全状态，避免一上电立即运动造成碰撞。

如果现场条件允许，先不要挂载负载，先空载测试基本控制链路。

---

## 6. 推荐的首次联调顺序

下面给出一个比较稳妥的首次调试顺序。

### 步骤 1：查询设备信息

```bash
./build/motor_cli --config configs/motor_cli.conf info
```

目的：

- 验证主板连接是否成功。
- 验证电机对象是否成功创建。
- 验证能够读到电机型号、主板固件版本和电机固件版本。

如果这一步失败，优先检查：

- IP/端口是否正确。
- 控制盒是否在线。
- CAN ID 和 CAN 线号是否正确。

### 步骤 2：查询当前在线电机地址

如果 `info` 能读到主板信息，但电机型号/电机固件为空，建议立刻执行：

```bash
./build/motor_cli --config configs/motor_cli.conf get-id
```

正常情况下会输出：

```text
can_id=2
can_line_id=1
```

如果这里读出来的地址和配置文件不一致，请先修正 `motor.can_id` / `motor.can_line_id`，再继续后续控制。

---

### 步骤 3：清除故障

```bash
./build/motor_cli --config configs/motor_cli.conf clear-fault
```

如果设备之前有异常停机、急停或参数写入失败，建议在正式控制前先执行一次。

---

### 步骤 4：先进入交互式 Shell

推荐用 shell 做连续调试，因为单次命令执行完后进程会退出，而 shell 模式会保持连接，便于你连续下发命令和观察状态。

启动 shell：

```bash
./build/motor_cli --config configs/motor_cli.conf shell
```

启动后会看到提示符：

```text
interactive shell started, type help for commands, quit to exit
>
```

---

## 7. Shell 模式下的详细调试步骤

下面建议按这个顺序操作。

### 7.1 查看帮助

在 shell 内输入：

```text
help
```

---

### 7.2 再次确认设备信息

```text
info
```

如果 `model=` 或 `motor_fw=` 为空，再执行：

```text
get-id
```

---

### 7.3 清故障

```text
clear-fault
```

---

### 7.4 切换到位置模式

```text
mode position
```

---

### 7.5 使能电机

```text
enable
```

> 第一次使能时，请确保电机机械端附近没有障碍物。

---

### 7.6 小步位置测试

先给一个较小的位置指令，例如：

```text
set-pos 0.1
```

如果电机响应正常，再逐步增大：

```text
set-pos 0.3
set-pos 0.5
```

不要一开始就给大位移。

---

### 7.7 查看反馈数据

```text
monitor 20 100 fast
```

含义：

- `20`：读取 20 次。
- `100`：每 100ms 读取一次。
- `fast`：使用快速反馈接口。

输出中会看到：

- `pos`
- `vel`
- `cur`
- `tor_l`
- `tor_e`
- `encoder`

---

### 7.8 切换到速度模式测试

```text
mode velocity
enable
set-vel 0.2
```

确认响应后，再根据现场情况逐步调整：

```text
set-vel 0.5
set-vel 1.0
```

测试结束后建议回零速：

```text
set-vel 0
```

---

### 7.9 电流模式测试

```text
mode current
enable
set-cur 0.2
```

第一次测试时建议从很小电流开始，不要直接给大电流。

---

### 7.10 PD 模式测试

```text
mode pd
enable
set-pd-target 0.2 0.0 0.1
```

如果你已经确认 `motor.pd_kp` / `motor.pd_kd` 合适，再继续扩大目标值。

---

### 7.11 强拖模式测试

```text
strong-drag sensorless 2
```

可选角度源：

- `force`
- `sensorless`
- `sensor`
- `fusion`

强拖功能请务必在熟悉现场工况后再使用。

---

### 7.12 置零和校准

位置置零：

```text
zero
```

编码器校准：

```text
calibrate
```

校准动作可能会引起电机运动，请确认机械端处于安全状态。

---

### 7.13 停机与退出

先失能：

```text
disable
```

然后退出 shell：

```text
quit
```

---

## 8. 非 Shell 模式下的单次命令示例

如果你只想发单条命令，也可以直接使用一条命令完成，例如：

```bash
./build/motor_cli --config configs/motor_cli.conf mode position
./build/motor_cli --config configs/motor_cli.conf enable
./build/motor_cli --config configs/motor_cli.conf set-pos 0.2
```

但更推荐 shell 模式，因为它会保持同一轮调试会话，更适合现场调试。

---

## 9. 常见问题排查

### 9.1 `failed to initialize controller`

重点检查：

- 控制盒 IP 是否正确。
- 控制主机本地 IP 是否正确。
- 网口是否接到正确网段。
- 主板是否上电。
- `motor.can_id` / `motor.can_line_id` 是否正确。

### 9.2 能连主板但查不到电机

重点检查：

- 电机 CAN ID 配置。
- 电机所在 CAN 线号。
- 电机供电。
- CAN 线连接。

### 9.3 构建成功但运行时报找不到动态库

先执行：

```bash
ldd ./build/motor_cli
```

如果 `libMotorDrive.so` 没有解析到，确认：

```bash
ls -l linux_sdk/lib/output/libMotorDrive.so
```

### 9.4 电机有响应但动作异常

优先处理：

1. 检查电机 ID 和型号是否匹配。
2. 检查模式是否切换正确。
3. 降低位置/速度/电流目标值。
4. 恢复较保守 PID/PD 参数。
5. 先执行 `clear-fault` 再重新使能。

---

## 10. 推荐的现场标准流程（简版）

如果你在现场只需要一个最短流程，可以直接按下面顺序执行：

```bash
cd /items/single_motor_ctrl
cmake -S . -B build
cmake --build build -j4
vim configs/motor_cli.conf
./build/motor_cli --config configs/motor_cli.conf info
./build/motor_cli --config configs/motor_cli.conf shell
```

进入 shell 后：

```text
clear-fault
mode position
enable
set-pos 0.1
monitor 10 100 fast
disable
quit
```

这样就完成了一次最基础的单电机联调闭环。
