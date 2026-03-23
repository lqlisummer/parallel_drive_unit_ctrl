#include "pdu/mock_motor_backend.hpp"

#include <stdexcept>
#include <utility>

namespace pdu {

MockMotorBackend::MockMotorBackend(AppConfig config) : config_(std::move(config)) {}

const char* MockMotorBackend::BackendName() const {
    return "mock";
}

void MockMotorBackend::Initialize() {
    initialized_ = true;
    mode_ = CommandMode::kNone;
}

void MockMotorBackend::SetControlMode(CommandMode mode) {
    if (!initialized_) {
        throw std::runtime_error("Mock backend not initialized");
    }
    mode_ = mode;
}

void MockMotorBackend::ClearFault() {
    if (!initialized_) {
        throw std::runtime_error("Mock backend not initialized");
    }
}

void MockMotorBackend::Enable() {
    if (!initialized_) {
        throw std::runtime_error("Mock backend not initialized");
    }
    enabled_ = true;
}

void MockMotorBackend::Disable() {
    if (!initialized_) {
        throw std::runtime_error("Mock backend not initialized");
    }
    enabled_ = false;
}

void MockMotorBackend::ZeroOutput() {
    motor2_ = {};
    motor3_ = {};
}

void MockMotorBackend::SendCommand(const MotorPairCommand& command) {
    if (!enabled_) {
        throw std::runtime_error("Mock backend is not enabled");
    }

    const double dt = config_.runtime.loop_hz > 0.0 ? (1.0 / config_.runtime.loop_hz) : 0.005;
    switch (mode_) {
        case CommandMode::kCurrent:
            motor2_.current_a = command.m2.current_a;
            motor3_.current_a = command.m3.current_a;
            break;
        case CommandMode::kVelocity:
            motor2_.velocity_rad_s = command.m2.velocity_rad_s;
            motor3_.velocity_rad_s = command.m3.velocity_rad_s;
            motor2_.position_rad += command.m2.velocity_rad_s * dt;
            motor3_.position_rad += command.m3.velocity_rad_s * dt;
            break;
        case CommandMode::kPosition:
            motor2_.position_rad = command.m2.position_rad;
            motor3_.position_rad = command.m3.position_rad;
            motor2_.velocity_rad_s = command.m2.velocity_rad_s;
            motor3_.velocity_rad_s = command.m3.velocity_rad_s;
            break;
        case CommandMode::kPd:
            motor2_.position_rad = command.m2.position_rad;
            motor3_.position_rad = command.m3.position_rad;
            motor2_.velocity_rad_s = command.m2.velocity_rad_s;
            motor3_.velocity_rad_s = command.m3.velocity_rad_s;
            motor2_.current_a = command.m2.current_a;
            motor3_.current_a = command.m3.current_a;
            break;
        case CommandMode::kNone:
        case CommandMode::kBrake:
        case CommandMode::kOpenLoop:
            throw std::runtime_error("Mock backend mode does not accept joint commands");
    }

    motor2_.load_torque_nm = motor2_.current_a * 0.1;
    motor2_.electromagnetic_torque_nm = motor2_.current_a * 0.1;
    motor2_.encoder_value = static_cast<int>(motor2_.position_rad * 10000.0);

    motor3_.load_torque_nm = motor3_.current_a * 0.1;
    motor3_.electromagnetic_torque_nm = motor3_.current_a * 0.1;
    motor3_.encoder_value = static_cast<int>(motor3_.position_rad * 10000.0);
}

std::pair<MotorState, MotorState> MockMotorBackend::ReadMotorStates() {
    if (!initialized_) {
        throw std::runtime_error("Mock backend not initialized");
    }
    return {motor2_, motor3_};
}

HardwareInfo MockMotorBackend::QueryHardwareInfo() {
    return HardwareInfo{
        "mock-sdk",
        "mock-board-fw",
        {config_.motor2.name, "mock-motor", "mock-fw"},
        {config_.motor3.name, "mock-motor", "mock-fw"},
    };
}

void MockMotorBackend::Shutdown() noexcept {
    enabled_ = false;
    initialized_ = false;
    mode_ = CommandMode::kNone;
}

}  // namespace pdu
