#pragma once

#include <cstdint>
#include <string>

#include "pdu/types.hpp"

namespace pdu {

struct NetworkConfig {
    uint16_t board_id = 0xFD;
    std::string local_ip = "192.168.3.245";
    int local_port = 15021;
    std::string remote_ip = "192.168.3.11";
    int remote_port = 14999;
};

struct MotorConfig {
    std::string name = "motor";
    uint16_t can_id = 0;
    uint16_t can_line = 1;
};

struct PidGains {
    double position_kp = 1.0;
    double velocity_kp = 0.05;
    double velocity_ki = 1.0;
};

struct PdGains {
    double kp = 50.0;
    double kd = 5.0;
};

struct ControlConfig {
    CommandMode command_mode = CommandMode::kPdSync;
    bool auto_enable = true;
    bool clear_fault_on_start = true;
    bool zero_on_start = false;
    PidGains pid;
    PdGains pd;
};

struct FeedbackConfig {
    bool fast_mode = false;
    int fast_period_hz = 500;
};

struct RuntimeConfig {
    std::string backend = "mock";
    double loop_hz = 200.0;
    bool synchronized_send = true;
    int startup_flush_cycles = 2;
    bool print_feedback = true;
};

struct DemoConfig {
    double theta1_target_rad = 0.6;
    double theta2_target_rad = 0.15;
    double duration_s = 4.0;
    double hold_s = 2.0;
    int ramp_steps = 400;
};

struct AppConfig {
    NetworkConfig network;
    MotorConfig motor2 = {"m2", 2, 1};
    MotorConfig motor3 = {"m3", 3, 1};
    ControlConfig control;
    FeedbackConfig feedback;
    RuntimeConfig runtime;
    DemoConfig demo;
};

AppConfig LoadConfig(const std::string& path);

}  // namespace pdu
