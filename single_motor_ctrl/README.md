# single_motor_ctrl

基于仓库内 `linux_sdk` 的单电机 CLI 调试控制工程。

## 项目结构

```text
.
├── CMakeLists.txt
├── configs/
│   └── motor_cli.conf
├── include/
│   ├── app/
│   ├── common/
│   ├── control/
│   ├── device/
│   ├── domain/
│   └── driver/
├── src/
│   ├── app/
│   ├── common/
│   ├── control/
│   ├── device/
│   ├── domain/
│   └── driver/
└── linux_sdk/
```

## 构建

```bash
cmake -S . -B build
cmake --build build -j
```


## 控制主机部署与操作流程

如果仓库已经部署到控制主机 `/items/single_motor_ctrl`，建议直接按 `docs/operation_guide.md` 执行现场联调流程，文档中已经补充了：

- 进入控制主机目录
- 检查 SDK 动态库
- 修改 `configs/motor_cli.conf`（当前电机默认按 CAN ID `2`、CAN line `1` 配置）
- 构建 CLI
- `info / get-id / clear-fault / shell / monitor` 的推荐顺序
- 位置、速度、电流、PD、强拖、置零、校准的操作步骤
- 常见故障排查

## CLI 用法

```bash
./build/motor_cli --config configs/motor_cli.conf dump-config
./build/motor_cli --config configs/motor_cli.conf raw-get-model
./build/motor_cli --config configs/motor_cli.conf info
./build/motor_cli --config configs/motor_cli.conf get-id
./build/motor_cli --config configs/motor_cli.conf enable
./build/motor_cli --config configs/motor_cli.conf mode position
./build/motor_cli --config configs/motor_cli.conf set-pos 1.0
./build/motor_cli --config configs/motor_cli.conf set-vel 1.5
./build/motor_cli --config configs/motor_cli.conf set-cur 0.8
./build/motor_cli --config configs/motor_cli.conf set-pd-target 1.0 0.5 0.2
./build/motor_cli --config configs/motor_cli.conf monitor 20 100 fast
./build/motor_cli --config configs/motor_cli.conf shell
```

## 支持的指令

- `dump-config`：打印 CLI 实际加载到的板卡/电机配置。
- `raw-get-model`：按厂家最小示例路径直接读取电机型号，绕过控制器封装。
- `info`：查询电机型号、电机固件、主板固件。
- `get-id`：通过主板查询当前在线电机的 CAN ID 和 CAN line。
- `enable` / `disable`：直接下发使能/失能控制字。
- `clear-fault`：清故障。
- `zero`：位置置零。
- `calibrate`：编码器校准。
- `mode <...>`：切换控制模式。
- `set-pos <rad>`：位置模式下发位置指令。
- `set-vel <rad_s>`：速度模式下发转速指令。
- `set-cur <amp>`：电流模式下发电流指令。
- `set-pd-target <pos> <vel> <cur>`：PD 模式下发组合指令。
- `strong-drag <source> <voltage>`：强拖调试。
- `monitor [count] [period_ms] [fast]`：周期读取遥测。
- `shell`：进入交互式调试终端，持续连接主板并逐条下发指令。
