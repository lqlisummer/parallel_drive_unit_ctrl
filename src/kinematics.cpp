#include "pdu/kinematics.hpp"

namespace pdu {

MotorPairCommand ParallelDriveKinematics::Inverse(const JointCommand& joint_command) {
    MotorPairCommand command;

    command.m2.position_rad = joint_command.theta1_rad + joint_command.theta2_rad;
    command.m3.position_rad = joint_command.theta1_rad - joint_command.theta2_rad;

    command.m2.velocity_rad_s = joint_command.theta1_velocity_rad_s + joint_command.theta2_velocity_rad_s;
    command.m3.velocity_rad_s = joint_command.theta1_velocity_rad_s - joint_command.theta2_velocity_rad_s;

    command.m2.current_a = joint_command.theta1_effort_ff + joint_command.theta2_effort_ff;
    command.m3.current_a = joint_command.theta1_effort_ff - joint_command.theta2_effort_ff;

    return command;
}

JointState ParallelDriveKinematics::Forward(const MotorState& motor2, const MotorState& motor3) {
    JointState state;
    state.motor2 = motor2;
    state.motor3 = motor3;

    state.theta1_rad = (motor2.position_rad + motor3.position_rad) / 2.0;
    state.theta2_rad = (motor2.position_rad - motor3.position_rad) / 2.0;

    state.theta1_velocity_rad_s = (motor2.velocity_rad_s + motor3.velocity_rad_s) / 2.0;
    state.theta2_velocity_rad_s = (motor2.velocity_rad_s - motor3.velocity_rad_s) / 2.0;

    state.theta1_load_torque_nm = (motor2.load_torque_nm + motor3.load_torque_nm) / 2.0;
    state.theta2_load_torque_nm = (motor2.load_torque_nm - motor3.load_torque_nm) / 2.0;

    return state;
}

}  // namespace pdu
