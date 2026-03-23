#include <cmath>
#include <iostream>
#include <stdexcept>

#include "pdu/config.hpp"
#include "pdu/kinematics.hpp"
#include "pdu/parallel_drive_unit.hpp"

namespace {

void ExpectNear(double actual, double expected, double tolerance, const char* label) {
    if (std::fabs(actual - expected) > tolerance) {
        throw std::runtime_error(std::string(label) + " mismatch");
    }
}

}  // namespace

int main() {
    using namespace pdu;

    {
        if (ParseCommandMode("pd_sync") != CommandMode::kPd) {
            throw std::runtime_error("pd_sync alias parse failed");
        }
        if (ParseCommandMode("velocity") != CommandMode::kVelocity) {
            throw std::runtime_error("velocity parse failed");
        }
        if (ToString(CommandMode::kCurrent) != "current") {
            throw std::runtime_error("current stringify failed");
        }
    }

    {
        JointCommand joint_command;
        joint_command.theta1_rad = 0.5;
        joint_command.theta2_rad = 0.2;
        joint_command.theta1_velocity_rad_s = 1.2;
        joint_command.theta2_velocity_rad_s = -0.4;
        joint_command.theta1_effort_ff = 2.0;
        joint_command.theta2_effort_ff = 0.5;

        const MotorPairCommand motor_command = ParallelDriveKinematics::Inverse(joint_command);
        ExpectNear(motor_command.m2.position_rad, 0.7, 1e-9, "m2 position");
        ExpectNear(motor_command.m3.position_rad, 0.3, 1e-9, "m3 position");
        ExpectNear(motor_command.m2.velocity_rad_s, 0.8, 1e-9, "m2 velocity");
        ExpectNear(motor_command.m3.velocity_rad_s, 1.6, 1e-9, "m3 velocity");
        ExpectNear(motor_command.m2.current_a, 2.5, 1e-9, "m2 current");
        ExpectNear(motor_command.m3.current_a, 1.5, 1e-9, "m3 current");
    }

    {
        MotorState motor2;
        motor2.position_rad = 0.7;
        motor2.velocity_rad_s = 0.8;
        motor2.load_torque_nm = 2.5;

        MotorState motor3;
        motor3.position_rad = 0.3;
        motor3.velocity_rad_s = 1.6;
        motor3.load_torque_nm = 1.5;

        const JointState state = ParallelDriveKinematics::Forward(motor2, motor3);
        ExpectNear(state.theta1_rad, 0.5, 1e-9, "theta1");
        ExpectNear(state.theta2_rad, 0.2, 1e-9, "theta2");
        ExpectNear(state.theta1_velocity_rad_s, 1.2, 1e-9, "theta1 velocity");
        ExpectNear(state.theta2_velocity_rad_s, -0.4, 1e-9, "theta2 velocity");
        ExpectNear(state.theta1_load_torque_nm, 2.0, 1e-9, "theta1 load torque");
        ExpectNear(state.theta2_load_torque_nm, 0.5, 1e-9, "theta2 load torque");
    }

    {
        AppConfig config;
        config.runtime.backend = "mock";

        ParallelDriveUnit unit(config);
        unit.Connect();

        const HardwareInfo info = unit.QueryHardwareInfo();
        if (info.motor2.name != "m2" || info.motor3.name != "m3") {
            throw std::runtime_error("mock hardware info mismatch");
        }

        unit.SetMode(CommandMode::kPd);
        unit.Enable();

        JointCommand command;
        command.theta1_rad = 0.25;
        command.theta2_rad = -0.05;
        command.theta1_velocity_rad_s = 0.2;
        command.theta2_velocity_rad_s = 0.1;
        command.theta1_effort_ff = 1.0;
        command.theta2_effort_ff = 0.2;

        unit.CommandJoints(command);
        const JointState state = unit.ReadState();
        ExpectNear(state.theta1_rad, 0.25, 1e-9, "mock theta1");
        ExpectNear(state.theta2_rad, -0.05, 1e-9, "mock theta2");

        unit.SetMode(CommandMode::kVelocity);
        JointCommand velocity_command;
        velocity_command.theta1_velocity_rad_s = 0.4;
        velocity_command.theta2_velocity_rad_s = -0.1;
        unit.CommandJoints(velocity_command);
        const JointState velocity_state = unit.ReadState();
        ExpectNear(velocity_state.theta1_velocity_rad_s, 0.4, 1e-9, "mock theta1 velocity");
        ExpectNear(velocity_state.theta2_velocity_rad_s, -0.1, 1e-9, "mock theta2 velocity");

        unit.Shutdown();
    }

    std::cout << "parallel_drive_tests passed\n";
    return 0;
}
