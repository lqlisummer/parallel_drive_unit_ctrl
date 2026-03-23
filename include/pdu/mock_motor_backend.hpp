#pragma once

#include "pdu/motor_backend.hpp"

namespace pdu {

class MockMotorBackend final : public IMotorBackend {
public:
    explicit MockMotorBackend(AppConfig config);

    const char* BackendName() const override;
    void Initialize() override;
    void SetControlMode(CommandMode mode) override;
    void ClearFault() override;
    void Enable() override;
    void Disable() override;
    void ZeroOutput() override;
    void SendCommand(const MotorPairCommand& command) override;
    std::pair<MotorState, MotorState> ReadMotorStates() override;
    HardwareInfo QueryHardwareInfo() override;
    void Shutdown() noexcept override;

private:
    AppConfig config_;
    CommandMode mode_ = CommandMode::kNone;
    bool initialized_ = false;
    bool enabled_ = false;
    MotorState motor2_;
    MotorState motor3_;
};

}  // namespace pdu
