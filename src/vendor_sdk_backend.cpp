#include "pdu/vendor_sdk_backend.hpp"

#if !PDU_HAS_VENDOR_BACKEND
#error "vendor_sdk_backend.cpp requires PDU_HAS_VENDOR_BACKEND=1"
#endif

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

#include "CTypes.h"
#include "RobotControl.h"

namespace pdu {
namespace {

unsigned int ToSdkControlMode(CommandMode mode) {
    switch (mode) {
        case CommandMode::kPositionOnly:
            return MOTOR_CTRL_MODE_POSITION;
        case CommandMode::kPdSync:
            return MOTOR_CTRL_MODE_PD;
    }
    return MOTOR_CTRL_MODE_POSITION;
}

void RequireSuccess(int result, const char* action) {
    if (!result) {
        throw std::runtime_error(action);
    }
}

class VendorSdkBackend final : public IMotorBackend {
public:
    explicit VendorSdkBackend(AppConfig config) : config_(std::move(config)) {}

    ~VendorSdkBackend() override {
        Shutdown();
    }

    const char* BackendName() const override {
        return "vendor";
    }

    void Initialize() override {
        if (ctx_ != nullptr) {
            return;
        }

        ctx_ = robot_create(config_.network.board_id);
        if (ctx_ == nullptr) {
            throw std::runtime_error("robot_create failed");
        }

        try {
            RequireSuccess(
                robot_config_net(
                    ctx_,
                    config_.network.local_ip.c_str(),
                    config_.network.local_port,
                    config_.network.remote_port,
                    config_.network.remote_ip.c_str()),
                "robot_config_net failed");

            motor2_ = robot_create_motor(ctx_, config_.motor2.can_id, config_.motor2.can_line);
            motor3_ = robot_create_motor(ctx_, config_.motor3.can_id, config_.motor3.can_line);
            if (motor2_ == nullptr || motor3_ == nullptr) {
                throw std::runtime_error("robot_create_motor failed for one or more motors");
            }

            if (config_.feedback.fast_mode) {
                RequireSuccess(
                    robot_set_fast_mode(ctx_, 1, config_.feedback.fast_period_hz),
                    "robot_set_fast_mode failed");
            }

            ConfigureMotor(motor2_);
            ConfigureMotor(motor3_);

            if (config_.control.command_mode == CommandMode::kPdSync) {
                PrimeSynchronizedPipeline();
            }
        } catch (...) {
            Shutdown();
            throw;
        }
    }

    void ClearFault() override {
        EnsureInitialized();
        RequireSuccess(robot_motor_set_control_world(motor2_, CTRL_CLEAR_FAULT), "clear fault failed on motor2");
        RequireSuccess(robot_motor_set_control_world(motor3_, CTRL_CLEAR_FAULT), "clear fault failed on motor3");
    }

    void Enable() override {
        EnsureInitialized();
        RequireSuccess(robot_motor_set_control_world(motor2_, CTRL_SERVO_ON), "servo on failed on motor2");
        RequireSuccess(robot_motor_set_control_world(motor3_, CTRL_SERVO_ON), "servo on failed on motor3");
    }

    void ZeroOutput() override {
        EnsureInitialized();
        RequireSuccess(robot_motor_set_control_world(motor2_, CTRL_POSITION_SET_ZERO), "zero output failed on motor2");
        RequireSuccess(robot_motor_set_control_world(motor3_, CTRL_POSITION_SET_ZERO), "zero output failed on motor3");
    }

    void SendCommand(const MotorPairCommand& command) override {
        EnsureInitialized();

        if (config_.control.command_mode == CommandMode::kPdSync) {
            robot_motor_set_pos(
                motor2_,
                static_cast<float>(command.m2.position_rad),
                static_cast<float>(command.m2.velocity_rad_s),
                static_cast<float>(command.m2.current_a));
            robot_motor_set_pos(
                motor3_,
                static_cast<float>(command.m3.position_rad),
                static_cast<float>(command.m3.velocity_rad_s),
                static_cast<float>(command.m3.current_a));

            if (config_.runtime.synchronized_send) {
                robot_motor_set_big_pose(ctx_);
            }
        } else {
            robot_motor_set_pose(motor2_, static_cast<float>(command.m2.position_rad));
            robot_motor_set_pose(motor3_, static_cast<float>(command.m3.position_rad));
        }
    }

    std::pair<MotorState, MotorState> ReadMotorStates() override {
        EnsureInitialized();
        return {ReadStateFrom(motor2_), ReadStateFrom(motor3_)};
    }

    void Shutdown() noexcept override {
        if (motor2_ != nullptr && ctx_ != nullptr) {
            robot_destroy_motor(ctx_, motor2_);
            motor2_ = nullptr;
        }
        if (motor3_ != nullptr && ctx_ != nullptr) {
            robot_destroy_motor(ctx_, motor3_);
            motor3_ = nullptr;
        }
        if (ctx_ != nullptr) {
            robot_destroy(ctx_);
            ctx_ = nullptr;
        }
    }

private:
    void EnsureInitialized() const {
        if (ctx_ == nullptr || motor2_ == nullptr || motor3_ == nullptr) {
            throw std::runtime_error("Vendor backend is not initialized");
        }
    }

    void ConfigureMotor(RobotMotor* motor) {
        RequireSuccess(
            robot_motor_set_control_mode(motor, ToSdkControlMode(config_.control.command_mode)),
            "robot_motor_set_control_mode failed");

        robot_motor_set_pid(
            motor,
            static_cast<float>(config_.control.pid.position_kp),
            static_cast<float>(config_.control.pid.velocity_kp),
            static_cast<float>(config_.control.pid.velocity_ki));

        if (config_.control.command_mode == CommandMode::kPdSync) {
            robot_motor_set_pd(
                motor,
                static_cast<float>(config_.control.pd.kp),
                static_cast<float>(config_.control.pd.kd));
        }
    }

    void PrimeSynchronizedPipeline() {
        for (int i = 0; i < config_.runtime.startup_flush_cycles; ++i) {
            robot_motor_set_pos(motor2_, 0.0f, 0.0f, 0.0f);
            robot_motor_set_pos(motor3_, 0.0f, 0.0f, 0.0f);
            robot_motor_set_big_pose(ctx_);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    MotorState ReadStateFrom(RobotMotor* motor) const {
        MotorState state;
        if (config_.feedback.fast_mode) {
            float pos = 0.0f;
            float vel = 0.0f;
            float cur = 0.0f;
            robot_motor_get_PVCTFast(motor, &pos, &vel, &cur);
            state.position_rad = pos;
            state.velocity_rad_s = vel;
            state.current_a = cur;
            return state;
        }

        float pos = 0.0f;
        float vel = 0.0f;
        float cur = 0.0f;
        float tor_l = 0.0f;
        float tor_e = 0.0f;
        robot_motor_get_PVCT(motor, &pos, &vel, &cur, &tor_l, &tor_e);
        state.position_rad = pos;
        state.velocity_rad_s = vel;
        state.current_a = cur;
        state.load_torque_nm = tor_l;
        state.electromagnetic_torque_nm = tor_e;
        return state;
    }

    AppConfig config_;
    RobotCtx* ctx_ = nullptr;
    RobotMotor* motor2_ = nullptr;
    RobotMotor* motor3_ = nullptr;
};

}  // namespace

std::unique_ptr<IMotorBackend> CreateVendorBackend(const AppConfig& config) {
    return std::make_unique<VendorSdkBackend>(config);
}

}  // namespace pdu
