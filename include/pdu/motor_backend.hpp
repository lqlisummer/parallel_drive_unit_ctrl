#pragma once

#include <memory>
#include <utility>

#include "pdu/config.hpp"
#include "pdu/types.hpp"

namespace pdu {

class IMotorBackend {
public:
    virtual ~IMotorBackend() = default;

    virtual const char* BackendName() const = 0;
    virtual void Initialize() = 0;
    virtual void ClearFault() = 0;
    virtual void Enable() = 0;
    virtual void ZeroOutput() = 0;
    virtual void SendCommand(const MotorPairCommand& command) = 0;
    virtual std::pair<MotorState, MotorState> ReadMotorStates() = 0;
    virtual void Shutdown() noexcept = 0;
};

std::unique_ptr<IMotorBackend> CreateBackend(const AppConfig& config);
bool VendorBackendCompiled();

}  // namespace pdu
