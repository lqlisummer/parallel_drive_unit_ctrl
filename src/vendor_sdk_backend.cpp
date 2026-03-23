#include "pdu/vendor_sdk_backend.hpp"

#if !PDU_HAS_VENDOR_BACKEND
#error "vendor_sdk_backend.cpp requires PDU_HAS_VENDOR_BACKEND=1"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <utility>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#endif

#include "CTypes.h"
#include "RobotControl.h"

namespace pdu {
namespace {

unsigned int ToSdkControlMode(CommandMode mode) {
    switch (mode) {
        case CommandMode::kNone:
            return MOTOR_CTRL_MODE_NONE;
        case CommandMode::kCurrent:
            return MOTOR_CTRL_MODE_CURRENT;
        case CommandMode::kVelocity:
            return MOTOR_CTRL_MODE_VELOCITY;
        case CommandMode::kPosition:
            return MOTOR_CTRL_MODE_POSITION;
        case CommandMode::kPd:
            return MOTOR_CTRL_MODE_PD;
        case CommandMode::kBrake:
            return MOTOR_CTRL_MODE_BRAKE;
        case CommandMode::kOpenLoop:
            return MOTOR_CTRL_MODE_OPENLOOP;
    }
    return MOTOR_CTRL_MODE_NONE;
}

void RequireSuccess(int result, const char* action) {
    if (!result) {
        throw std::runtime_error(action);
    }
}

#if defined(__unix__) || defined(__APPLE__)
class ScopedStdoutSilencer {
public:
    ScopedStdoutSilencer() {
        fflush(stdout);
        null_fd_ = open("/dev/null", O_WRONLY);
        if (null_fd_ < 0) {
            return;
        }

        saved_stdout_ = dup(STDOUT_FILENO);
        if (saved_stdout_ < 0) {
            close(null_fd_);
            null_fd_ = -1;
            return;
        }

        if (dup2(null_fd_, STDOUT_FILENO) < 0) {
            close(saved_stdout_);
            close(null_fd_);
            saved_stdout_ = -1;
            null_fd_ = -1;
        }
    }

    ~ScopedStdoutSilencer() {
        if (saved_stdout_ >= 0) {
            fflush(stdout);
            dup2(saved_stdout_, STDOUT_FILENO);
            close(saved_stdout_);
        }
        if (null_fd_ >= 0) {
            close(null_fd_);
        }
    }

private:
    int saved_stdout_ = -1;
    int null_fd_ = -1;
};
#else
class ScopedStdoutSilencer {
public:
    ScopedStdoutSilencer() = default;
};
#endif

class VendorSdkBackend final : public IMotorBackend {
public:
    explicit VendorSdkBackend(AppConfig config) : config_(std::move(config)) {}

    ~VendorSdkBackend() override {
        Shutdown();
    }

    const char* BackendName() const override {
        return "vendor";
    }

    void Initialize() override {
        if (ctx_ != nullptr) {
            return;
        }

        ctx_ = robot_create(config_.network.board_id);
        if (ctx_ == nullptr) {
            throw std::runtime_error("robot_create failed");
        }

        try {
            RequireSuccess(
                robot_config_net(
                    ctx_,
                    config_.network.local_ip.c_str(),
                    config_.network.local_port,
                    config_.network.remote_port,
                    config_.network.remote_ip.c_str()),
                "robot_config_net failed");

            motor2_ = robot_create_motor(ctx_, config_.motor2.can_id, config_.motor2.can_line);
            motor3_ = robot_create_motor(ctx_, config_.motor3.can_id, config_.motor3.can_line);
            if (motor2_ == nullptr || motor3_ == nullptr) {
                throw std::runtime_error("robot_create_motor failed for one or more motors");
            }

            if (config_.feedback.fast_mode) {
                RequireSuccess(
                    robot_set_fast_mode(ctx_, 1, config_.feedback.fast_period_hz),
                    "robot_set_fast_mode failed");
            }
        } catch (...) {
            Shutdown();
            throw;
        }
    }

    void SetControlMode(CommandMode mode) override {
        EnsureInitialized();

        ConfigureMotor(motor2_, mode, "motor2");
        ConfigureMotor(motor3_, mode, "motor3");
        PrimeOutputs(mode);
        current_mode_ = mode;
    }

    void ClearFault() override {
        EnsureInitialized();
        RequireSuccess(robot_motor_set_control_world(motor2_, CTRL_CLEAR_FAULT), "clear fault failed on motor2");
        RequireSuccess(robot_motor_set_control_world(motor3_, CTRL_CLEAR_FAULT), "clear fault failed on motor3");
    }

    void Enable() override {
        EnsureInitialized();
        RequireSuccess(robot_motor_set_control_world(motor2_, CTRL_SERVO_ON), "servo on failed on motor2");
        RequireSuccess(robot_motor_set_control_world(motor3_, CTRL_SERVO_ON), "servo on failed on motor3");
    }

    void Disable() override {
        EnsureInitialized();
        switch (current_mode_) {
            case CommandMode::kCurrent:
                robot_motor_set_cur(motor2_, 0.0f);
                robot_motor_set_cur(motor3_, 0.0f);
                break;
            case CommandMode::kVelocity:
                robot_motor_set_vel(motor2_, 0.0f);
                robot_motor_set_vel(motor3_, 0.0f);
                break;
            case CommandMode::kPosition:
                SendPositionHold();
                break;
            case CommandMode::kPd:
                SendZeroEffortHold();
                break;
            case CommandMode::kNone:
            case CommandMode::kBrake:
            case CommandMode::kOpenLoop:
                break;
        }
        RequireSuccess(robot_motor_set_control_world(motor2_, CTRL_SERVO_OFF), "servo off failed on motor2");
        RequireSuccess(robot_motor_set_control_world(motor3_, CTRL_SERVO_OFF), "servo off failed on motor3");
    }

    void ZeroOutput() override {
        EnsureInitialized();
        RequireSuccess(robot_motor_set_control_world(motor2_, CTRL_POSITION_SET_ZERO), "zero output failed on motor2");
        RequireSuccess(robot_motor_set_control_world(motor3_, CTRL_POSITION_SET_ZERO), "zero output failed on motor3");
    }

    void SendCommand(const MotorPairCommand& command) override {
        EnsureInitialized();

        switch (current_mode_) {
            case CommandMode::kCurrent:
                robot_motor_set_cur(motor2_, static_cast<float>(command.m2.current_a));
                robot_motor_set_cur(motor3_, static_cast<float>(command.m3.current_a));
                break;
            case CommandMode::kVelocity:
                robot_motor_set_vel(motor2_, static_cast<float>(command.m2.velocity_rad_s));
                robot_motor_set_vel(motor3_, static_cast<float>(command.m3.velocity_rad_s));
                break;
            case CommandMode::kPosition:
                robot_motor_set_pose(motor2_, static_cast<float>(command.m2.position_rad));
                robot_motor_set_pose(motor3_, static_cast<float>(command.m3.position_rad));
                break;
            case CommandMode::kPd:
                robot_motor_set_pos(
                    motor2_,
                    static_cast<float>(command.m2.position_rad),
                    static_cast<float>(command.m2.velocity_rad_s),
                    static_cast<float>(command.m2.current_a));
                robot_motor_set_pos(
                    motor3_,
                    static_cast<float>(command.m3.position_rad),
                    static_cast<float>(command.m3.velocity_rad_s),
                    static_cast<float>(command.m3.current_a));
                robot_motor_set_big_pose(ctx_);
                break;
            case CommandMode::kNone:
            case CommandMode::kBrake:
            case CommandMode::kOpenLoop:
                throw std::runtime_error("Current control mode does not support joint-space commands");
        }
    }

    std::pair<MotorState, MotorState> ReadMotorStates() override {
        EnsureInitialized();
        return {ReadStateFrom(motor2_), ReadStateFrom(motor3_)};
    }

    HardwareInfo QueryHardwareInfo() override {
        EnsureInitialized();

        HardwareInfo info;
        info.sdk_version = QuerySdkVersion();
        info.board_firmware_version = QueryBoardFirmwareVersion();
        info.motor2.name = config_.motor2.name;
        info.motor2.model = QueryMotorModel(motor2_);
        info.motor2.firmware_version = QueryMotorFirmwareVersion(motor2_);
        info.motor3.name = config_.motor3.name;
        info.motor3.model = QueryMotorModel(motor3_);
        info.motor3.firmware_version = QueryMotorFirmwareVersion(motor3_);
        return info;
    }

    void Shutdown() noexcept override {
        if (ctx_ == nullptr) {
            current_mode_ = CommandMode::kNone;
            return;
        }

        BestEffortStop();

        if (motor2_ != nullptr) {
            ScopedStdoutSilencer silencer;
            robot_destroy_motor(ctx_, motor2_);
            motor2_ = nullptr;
        }
        if (motor3_ != nullptr) {
            ScopedStdoutSilencer silencer;
            robot_destroy_motor(ctx_, motor3_);
            motor3_ = nullptr;
        }

        {
            ScopedStdoutSilencer silencer;
            robot_destroy(ctx_);
        }
        ctx_ = nullptr;
        current_mode_ = CommandMode::kNone;
    }

private:
    void EnsureInitialized() const {
        if (ctx_ == nullptr || motor2_ == nullptr || motor3_ == nullptr) {
            throw std::runtime_error("Vendor backend is not initialized");
        }
    }

    void ConfigureMotor(RobotMotor* motor, CommandMode mode, const char* label) {
        const std::string mode_message = std::string("robot_motor_set_control_mode failed on ") + label;
        RequireSuccess(robot_motor_set_control_mode(motor, ToSdkControlMode(mode)), mode_message.c_str());

        robot_motor_set_pid(
            motor,
            static_cast<float>(config_.control.pid.position_kp),
            static_cast<float>(config_.control.pid.velocity_kp),
            static_cast<float>(config_.control.pid.velocity_ki));

        if (mode == CommandMode::kPd) {
            robot_motor_set_pd(
                motor,
                static_cast<float>(config_.control.pd.kp),
                static_cast<float>(config_.control.pd.kd));
        }
    }

    void PrimeOutputs(CommandMode mode) {
        switch (mode) {
            case CommandMode::kCurrent:
                robot_motor_set_cur(motor2_, 0.0f);
                robot_motor_set_cur(motor3_, 0.0f);
                break;
            case CommandMode::kVelocity:
                robot_motor_set_vel(motor2_, 0.0f);
                robot_motor_set_vel(motor3_, 0.0f);
                break;
            case CommandMode::kPosition:
                SendPositionHold();
                break;
            case CommandMode::kPd:
                for (int i = 0; i < std::max(1, config_.runtime.startup_flush_cycles); ++i) {
                    SendZeroEffortHold();
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                break;
            case CommandMode::kNone:
            case CommandMode::kBrake:
            case CommandMode::kOpenLoop:
                break;
        }
    }

    void SendPositionHold() {
        const auto [motor2_state, motor3_state] = ReadMotorStates();
        robot_motor_set_pose(motor2_, static_cast<float>(motor2_state.position_rad));
        robot_motor_set_pose(motor3_, static_cast<float>(motor3_state.position_rad));
    }

    void SendZeroEffortHold() {
        const auto [motor2_state, motor3_state] = ReadMotorStates();
        robot_motor_set_pos(motor2_, static_cast<float>(motor2_state.position_rad), 0.0f, 0.0f);
        robot_motor_set_pos(motor3_, static_cast<float>(motor3_state.position_rad), 0.0f, 0.0f);
        robot_motor_set_big_pose(ctx_);
    }

    void BestEffortStop() noexcept {
        if (ctx_ == nullptr || motor2_ == nullptr || motor3_ == nullptr) {
            return;
        }

        try {
            switch (current_mode_) {
                case CommandMode::kCurrent:
                    robot_motor_set_cur(motor2_, 0.0f);
                    robot_motor_set_cur(motor3_, 0.0f);
                    break;
                case CommandMode::kVelocity:
                    robot_motor_set_vel(motor2_, 0.0f);
                    robot_motor_set_vel(motor3_, 0.0f);
                    break;
                case CommandMode::kPosition:
                    SendPositionHold();
                    break;
                case CommandMode::kPd:
                    SendZeroEffortHold();
                    break;
                case CommandMode::kNone:
                case CommandMode::kBrake:
                case CommandMode::kOpenLoop:
                    break;
            }

            robot_motor_set_control_world(motor2_, CTRL_SERVO_OFF);
            robot_motor_set_control_world(motor3_, CTRL_SERVO_OFF);
        } catch (...) {
        }
    }

    MotorState ReadStateFrom(RobotMotor* motor) const {
        MotorState state;
        if (config_.feedback.fast_mode) {
            float pos = 0.0f;
            float vel = 0.0f;
            float cur = 0.0f;
            robot_motor_get_PVCTFast(motor, &pos, &vel, &cur);
            state.position_rad = pos;
            state.velocity_rad_s = vel;
            state.current_a = cur;
        } else {
            float pos = 0.0f;
            float vel = 0.0f;
            float cur = 0.0f;
            float tor_l = 0.0f;
            float tor_e = 0.0f;
            robot_motor_get_PVCT(motor, &pos, &vel, &cur, &tor_l, &tor_e);
            state.position_rad = pos;
            state.velocity_rad_s = vel;
            state.current_a = cur;
            state.load_torque_nm = tor_l;
            state.electromagnetic_torque_nm = tor_e;
        }

        int encoder = 0;
        robot_motor_get_EncoderValue(motor, &encoder);
        state.encoder_value = encoder;
        return state;
    }

    std::string QuerySdkVersion() const {
        char* version = get_sdk_version();
        return version != nullptr ? std::string(version) : std::string{};
    }

    std::string QueryBoardFirmwareVersion() const {
        std::array<char, 128> buffer{};
        if (!robot_motor_get_mother_board_firmware_version(ctx_, buffer.data())) {
            return {};
        }
        return std::string(buffer.data());
    }

    std::string QueryMotorFirmwareVersion(RobotMotor* motor) const {
        std::array<char, 128> buffer{};
        if (!robot_motor_get_motor_firmware_version(motor, buffer.data())) {
            return {};
        }
        return std::string(buffer.data());
    }

    std::string QueryMotorModel(RobotMotor* motor) const {
        std::array<char, 128> buffer{};
        if (!robot_motor_get_motor_model(motor, buffer.data())) {
            return {};
        }
        return std::string(buffer.data());
    }

    AppConfig config_;
    CommandMode current_mode_ = CommandMode::kNone;
    RobotCtx* ctx_ = nullptr;
    RobotMotor* motor2_ = nullptr;
    RobotMotor* motor3_ = nullptr;
};

}  // namespace

std::unique_ptr<IMotorBackend> CreateVendorBackend(const AppConfig& config) {
    return std::make_unique<VendorSdkBackend>(config);
}

}  // namespace pdu
