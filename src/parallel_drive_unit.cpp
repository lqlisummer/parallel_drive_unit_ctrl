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

void ParallelDriveUnit::Start() {
    if (started_) {
        return;
    }

    backend_->Initialize();

    if (config_.control.clear_fault_on_start) {
        backend_->ClearFault();
    }

    if (config_.control.auto_enable) {
        backend_->Enable();
    }

    if (config_.control.zero_on_start) {
        backend_->ZeroOutput();
    }

    started_ = true;
}

void ParallelDriveUnit::Shutdown() noexcept {
    if (!started_ || !backend_) {
        return;
    }
    backend_->Shutdown();
    started_ = false;
}

void ParallelDriveUnit::CommandJoints(const JointCommand& joint_command) {
    if (!started_) {
        throw std::runtime_error("ParallelDriveUnit is not started");
    }

    const MotorPairCommand motor_command = ParallelDriveKinematics::Inverse(joint_command);
    backend_->SendCommand(motor_command);
}

JointState ParallelDriveUnit::ReadState() {
    if (!started_) {
        throw std::runtime_error("ParallelDriveUnit is not started");
    }

    const auto [motor2, motor3] = backend_->ReadMotorStates();
    return ParallelDriveKinematics::Forward(motor2, motor3);
}

const AppConfig& ParallelDriveUnit::Config() const noexcept {
    return config_;
}

const char* ParallelDriveUnit::BackendName() const {
    return backend_ ? backend_->BackendName() : "unknown";
}

}  // namespace pdu
