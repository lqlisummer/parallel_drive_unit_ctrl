#pragma once

#include <memory>

#include "pdu/config.hpp"
#include "pdu/motor_backend.hpp"
#include "pdu/types.hpp"

namespace pdu {

class ParallelDriveUnit {
public:
    explicit ParallelDriveUnit(AppConfig config);
    ~ParallelDriveUnit();

    void Connect();
    void Start();
    void Shutdown() noexcept;

    void SetMode(CommandMode mode);
    void ClearFault();
    void Enable();
    void Disable();
    void ZeroOutput();
    void CommandJoints(const JointCommand& joint_command);
    JointState ReadState();
    HardwareInfo QueryHardwareInfo();

    const AppConfig& Config() const noexcept;
    const char* BackendName() const;
    CommandMode ActiveMode() const noexcept;
    bool IsConnected() const noexcept;

private:
    AppConfig config_;
    std::unique_ptr<IMotorBackend> backend_;
    CommandMode active_mode_ = CommandMode::kNone;
    bool connected_ = false;
};

}  // namespace pdu
