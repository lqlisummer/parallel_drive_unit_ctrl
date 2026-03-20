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

void MockMotorBackend::ZeroOutput() {
    motor2_ = {};
    motor3_ = {};
}

void MockMotorBackend::SendCommand(const MotorPairCommand& command) {
    if (!enabled_) {
        throw std::runtime_error("Mock backend is not enabled");
    }

    motor2_.velocity_rad_s = command.m2.velocity_rad_s;
    motor2_.current_a = command.m2.current_a;
    motor2_.position_rad = command.m2.position_rad;
    motor2_.load_torque_nm = command.m2.current_a * 0.1;
    motor2_.electromagnetic_torque_nm = command.m2.current_a * 0.1;

    motor3_.velocity_rad_s = command.m3.velocity_rad_s;
    motor3_.current_a = command.m3.current_a;
    motor3_.position_rad = command.m3.position_rad;
    motor3_.load_torque_nm = command.m3.current_a * 0.1;
    motor3_.electromagnetic_torque_nm = command.m3.current_a * 0.1;
}

std::pair<MotorState, MotorState> MockMotorBackend::ReadMotorStates() {
    if (!initialized_) {
        throw std::runtime_error("Mock backend not initialized");
    }
    return {motor2_, motor3_};
}

void MockMotorBackend::Shutdown() noexcept {
    enabled_ = false;
    initialized_ = false;
}

}  // namespace pdu
