#include "pdu/parallel_drive_unit.hpp"

#include <stdexcept>
#include <utility>

#include "pdu/kinematics.hpp"

namespace pdu {

ParallelDriveUnit::ParallelDriveUnit(AppConfig config)
    : config_(std::move(config)), backend_(CreateBackend(config_)) {}

ParallelDriveUnit::~ParallelDriveUnit() {
    Shutdown();
}

void ParallelDriveUnit::Connect() {
    if (connected_) {
        return;
    }

    backend_->Initialize();
    connected_ = true;
}

void ParallelDriveUnit::Start() {
    Connect();

    if (config_.control.clear_fault_on_start) {
        ClearFault();
    }

    if (config_.control.zero_on_start) {
        ZeroOutput();
    }

    SetMode(config_.control.command_mode);

    if (config_.control.auto_enable) {
        Enable();
    }
}

void ParallelDriveUnit::Shutdown() noexcept {
    if (!connected_ || !backend_) {
        return;
    }
    try {
        backend_->Disable();
    } catch (...) {
    }
    backend_->Shutdown();
    active_mode_ = CommandMode::kNone;
    connected_ = false;
}

void ParallelDriveUnit::SetMode(CommandMode mode) {
    if (!connected_) {
        throw std::runtime_error("ParallelDriveUnit is not connected");
    }

    backend_->SetControlMode(mode);
    active_mode_ = mode;
    config_.control.command_mode = mode;
}

void ParallelDriveUnit::ClearFault() {
    if (!connected_) {
        throw std::runtime_error("ParallelDriveUnit is not connected");
    }
    backend_->ClearFault();
}

void ParallelDriveUnit::Enable() {
    if (!connected_) {
        throw std::runtime_error("ParallelDriveUnit is not connected");
    }
    if (active_mode_ == CommandMode::kNone) {
        SetMode(config_.control.command_mode);
    }
    backend_->Enable();
}

void ParallelDriveUnit::Disable() {
    if (!connected_) {
        throw std::runtime_error("ParallelDriveUnit is not connected");
    }
    backend_->Disable();
}

void ParallelDriveUnit::ZeroOutput() {
    if (!connected_) {
        throw std::runtime_error("ParallelDriveUnit is not connected");
    }
    backend_->ZeroOutput();
}

void ParallelDriveUnit::CommandJoints(const JointCommand& joint_command) {
    if (!connected_) {
        throw std::runtime_error("ParallelDriveUnit is not connected");
    }
    if (active_mode_ == CommandMode::kNone || active_mode_ == CommandMode::kBrake || active_mode_ == CommandMode::kOpenLoop) {
        throw std::runtime_error("Active mode does not accept joint commands");
    }

    const MotorPairCommand motor_command = ParallelDriveKinematics::Inverse(joint_command);
    backend_->SendCommand(motor_command);
}

JointState ParallelDriveUnit::ReadState() {
    if (!connected_) {
        throw std::runtime_error("ParallelDriveUnit is not connected");
    }

    const auto [motor2, motor3] = backend_->ReadMotorStates();
    return ParallelDriveKinematics::Forward(motor2, motor3);
}

HardwareInfo ParallelDriveUnit::QueryHardwareInfo() {
    if (!connected_) {
        throw std::runtime_error("ParallelDriveUnit is not connected");
    }
    return backend_->QueryHardwareInfo();
}

const AppConfig& ParallelDriveUnit::Config() const noexcept {
    return config_;
}

const char* ParallelDriveUnit::BackendName() const {
    return backend_ ? backend_->BackendName() : "unknown";
}

CommandMode ParallelDriveUnit::ActiveMode() const noexcept {
    return active_mode_;
}

bool ParallelDriveUnit::IsConnected() const noexcept {
    return connected_;
}

}  // namespace pdu
