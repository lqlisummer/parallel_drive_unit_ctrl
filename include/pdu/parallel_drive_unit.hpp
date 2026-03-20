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

    void Start();
    void Shutdown() noexcept;

    void CommandJoints(const JointCommand& joint_command);
    JointState ReadState();

    const AppConfig& Config() const noexcept;
    const char* BackendName() const;

private:
    AppConfig config_;
    std::unique_ptr<IMotorBackend> backend_;
    bool started_ = false;
};

}  // namespace pdu
