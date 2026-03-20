#pragma once

#include <string>

namespace pdu {

enum class CommandMode {
    kPositionOnly,
    kPdSync,
};

struct MotorCommand {
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double current_a = 0.0;
};

struct MotorPairCommand {
    MotorCommand m2;
    MotorCommand m3;
};

struct JointCommand {
    double theta1_rad = 0.0;
    double theta2_rad = 0.0;
    double theta1_velocity_rad_s = 0.0;
    double theta2_velocity_rad_s = 0.0;
    double theta1_effort_ff = 0.0;
    double theta2_effort_ff = 0.0;
};

struct MotorState {
    double position_rad = 0.0;
    double velocity_rad_s = 0.0;
    double current_a = 0.0;
    double load_torque_nm = 0.0;
    double electromagnetic_torque_nm = 0.0;
};

struct JointState {
    double theta1_rad = 0.0;
    double theta2_rad = 0.0;
    double theta1_velocity_rad_s = 0.0;
    double theta2_velocity_rad_s = 0.0;
    double theta1_load_torque_nm = 0.0;
    double theta2_load_torque_nm = 0.0;
    MotorState motor2;
    MotorState motor3;
};

std::string ToString(CommandMode mode);
CommandMode ParseCommandMode(const std::string& value);

}  // namespace pdu
