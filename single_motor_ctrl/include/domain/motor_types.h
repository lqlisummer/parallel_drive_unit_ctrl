#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace smc {

enum class MotorControlMode : std::uint32_t {
    None = 0,
    Current = 1,
    Velocity = 2,
    Position = 3,
    Pd = 4,
    Brake = 5,
    OpenLoop = 0xFF,
};

enum class ControlWord : std::uint32_t {
    None = 0x0000,
    ServoOff = 0xF001,
    ServoOn = 0xF002,
    ClearFault = 0xF086,
    CalibrateEncoder = 0x101F,
    ReturnZero = 0x102F,
    PositionSetZero = 0x104F,
    FrictionIdentify = 0x201F,
    CoggingIdentify = 0x202F,
    FrictionCompOff = 0x300F,
    FrictionCompOn = 0x301F,
    CoggingCompOff = 0x302F,
    CoggingCompOn = 0x303F,
    TLoadCompOff = 0x304F,
    TLoadCompOn = 0x305F,
    SoftPosLimitOff = 0x306F,
    SoftPosLimitOn = 0x307F,
    OverTempProtectOff = 0x308F,
    OverTempProtectOn = 0x309F,
    HeartBeatProtectOff = 0x30AF,
    HeartBeatProtectOn = 0x30BF,
};

enum class ThetaSource : std::uint32_t {
    ForceTheta = 0,
    SensorLess = 1,
    Sensor = 2,
    SensorFusion = 3,
};

enum class MotorLifecycleState {
    Disconnected,
    Connected,
    MotorCreated,
    Ready,
    Enabled,
    Running,
    Fault,
    Stopped,
};

struct BoardConfig {
    std::string local_ip{"192.168.3.245"};
    int local_port{15021};
    std::string remote_ip{"192.168.3.11"};
    int remote_port{14999};
    std::uint16_t dev_id{0xFD};
    bool fast_mode_enabled{true};
    int fast_mode_hz{500};
};

struct MotorConfig {
    std::uint16_t can_id{2};
    std::uint16_t can_line_id{1};
    double pos_kp{1.0};
    double vel_kp{0.05};
    double vel_ki{1.0};
    double pd_kp{50.0};
    double pd_kd{5.0};
    double default_position_rad{0.0};
    double default_velocity_rad_s{0.0};
    double default_current_a{0.0};
};

struct MotorTelemetry {
    double position_rad{0.0};
    double velocity_rad_s{0.0};
    double current_a{0.0};
    double torque_load_nm{0.0};
    double torque_elec_nm{0.0};
    int encoder_value{0};
    std::chrono::steady_clock::time_point stamp{};
};

struct MotorIdentity {
    std::string model;
    std::string firmware_version;
    std::string board_firmware_version;
};

}  // namespace smc
