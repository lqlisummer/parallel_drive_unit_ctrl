#pragma once

#include "pdu/types.hpp"

namespace pdu {

class ParallelDriveKinematics {
public:
    static MotorPairCommand Inverse(const JointCommand& joint_command);
    static JointState Forward(const MotorState& motor2, const MotorState& motor3);
};

}  // namespace pdu
