#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "pdu/config.hpp"
#include "pdu/motor_backend.hpp"
#include "pdu/parallel_drive_unit.hpp"

namespace {

struct CliOptions {
    std::string config_path = "config/parallel_drive_unit.ini";
    std::string backend_override;
    std::vector<std::string> command_args;
};

CliOptions ParseArgs(int argc, char* argv[]) {
    CliOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--config requires a path");
            }
            options.config_path = argv[++i];
        } else if (arg == "--backend") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--backend requires a value");
            }
            options.backend_override = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: parallel_drive_app [--config <path>] [--backend <mock|vendor>] <command> [args]\n"
                << "Run `parallel_drive_app help` for the command list.\n";
            std::exit(0);
        } else {
            options.command_args.emplace_back(arg);
        }
    }
    return options;
}

void PrintUsage() {
    std::cout
        << "parallel_drive_app [--config <path>] [--backend <mock|vendor>] <command> [args]\n\n"
        << "Commands:\n"
        << "  help\n"
        << "  dump-config\n"
        << "  info\n"
        << "  state\n"
        << "  clear-fault\n"
        << "  enable\n"
        << "  disable\n"
        << "  zero\n"
        << "  mode <position|velocity|current|pd|brake|openloop>\n"
        << "  set-joint-pos <theta1_rad> <theta2_rad>\n"
        << "  set-joint-vel <theta1_vel_rad_s> <theta2_vel_rad_s>\n"
        << "  set-joint-cur <theta1_current_a> <theta2_current_a>\n"
        << "  set-pd-target <theta1_rad> <theta2_rad> <theta1_vel_rad_s> <theta2_vel_rad_s> [theta1_ff_a] [theta2_ff_a]\n"
        << "  monitor [count] [period_ms]\n"
        << "  demo\n"
        << "  shell\n";
}

double ParseDoubleArg(const std::string& value, const char* label) {
    try {
        return std::stod(value);
    } catch (...) {
        throw std::runtime_error(std::string("Invalid numeric value for ") + label + ": " + value);
    }
}

int ParseIntArg(const std::string& value, const char* label) {
    try {
        return std::stoi(value);
    } catch (...) {
        throw std::runtime_error(std::string("Invalid integer value for ") + label + ": " + value);
    }
}

double Lerp(double start, double end, double alpha) {
    return start + (end - start) * alpha;
}

void PrintConfig(const pdu::AppConfig& config) {
    std::cout
        << "[network]\n"
        << "board_id=" << config.network.board_id << "\n"
        << "local_ip=" << config.network.local_ip << "\n"
        << "local_port=" << config.network.local_port << "\n"
        << "remote_ip=" << config.network.remote_ip << "\n"
        << "remote_port=" << config.network.remote_port << "\n\n"
        << "[motor2]\n"
        << "name=" << config.motor2.name << "\n"
        << "can_id=" << config.motor2.can_id << "\n"
        << "can_line=" << config.motor2.can_line << "\n\n"
        << "[motor3]\n"
        << "name=" << config.motor3.name << "\n"
        << "can_id=" << config.motor3.can_id << "\n"
        << "can_line=" << config.motor3.can_line << "\n\n"
        << "[control]\n"
        << "command_mode=" << pdu::ToString(config.control.command_mode) << "\n"
        << "auto_enable=" << (config.control.auto_enable ? "true" : "false") << "\n"
        << "clear_fault_on_start=" << (config.control.clear_fault_on_start ? "true" : "false") << "\n"
        << "zero_on_start=" << (config.control.zero_on_start ? "true" : "false") << "\n"
        << "position_kp=" << config.control.pid.position_kp << "\n"
        << "velocity_kp=" << config.control.pid.velocity_kp << "\n"
        << "velocity_ki=" << config.control.pid.velocity_ki << "\n"
        << "pd_kp=" << config.control.pd.kp << "\n"
        << "pd_kd=" << config.control.pd.kd << "\n\n"
        << "[feedback]\n"
        << "fast_mode=" << (config.feedback.fast_mode ? "true" : "false") << "\n"
        << "fast_period_hz=" << config.feedback.fast_period_hz << "\n\n"
        << "[runtime]\n"
        << "backend=" << config.runtime.backend << "\n"
        << "loop_hz=" << config.runtime.loop_hz << "\n"
        << "synchronized_send=" << (config.runtime.synchronized_send ? "true" : "false") << "\n"
        << "startup_flush_cycles=" << config.runtime.startup_flush_cycles << "\n"
        << "print_feedback=" << (config.runtime.print_feedback ? "true" : "false") << "\n\n"
        << "[demo]\n"
        << "theta1_target_rad=" << config.demo.theta1_target_rad << "\n"
        << "theta2_target_rad=" << config.demo.theta2_target_rad << "\n"
        << "duration_s=" << config.demo.duration_s << "\n"
        << "hold_s=" << config.demo.hold_s << "\n"
        << "ramp_steps=" << config.demo.ramp_steps << "\n";
}

void PrintState(const pdu::JointState& state) {
    std::cout << std::fixed << std::setprecision(6)
              << "theta1=" << state.theta1_rad
              << " theta2=" << state.theta2_rad
              << " theta1_vel=" << state.theta1_velocity_rad_s
              << " theta2_vel=" << state.theta2_velocity_rad_s
              << " m2_pos=" << state.motor2.position_rad
              << " m3_pos=" << state.motor3.position_rad
              << " m2_vel=" << state.motor2.velocity_rad_s
              << " m3_vel=" << state.motor3.velocity_rad_s
              << " m2_cur=" << state.motor2.current_a
              << " m3_cur=" << state.motor3.current_a
              << " m2_tl=" << state.motor2.load_torque_nm
              << " m3_tl=" << state.motor3.load_torque_nm
              << " m2_enc=" << state.motor2.encoder_value
              << " m3_enc=" << state.motor3.encoder_value
              << "\n";
}

void PrintHardwareInfo(const pdu::ParallelDriveUnit& unit, const pdu::HardwareInfo& info) {
    std::cout
        << "backend=" << unit.BackendName() << "\n"
        << "active_mode=" << pdu::ToString(unit.ActiveMode()) << "\n"
        << "vendor_backend_compiled=" << (pdu::VendorBackendCompiled() ? "yes" : "no") << "\n"
        << "sdk_version=" << info.sdk_version << "\n"
        << "board_firmware_version=" << info.board_firmware_version << "\n"
        << info.motor2.name << "_model=" << info.motor2.model << "\n"
        << info.motor2.name << "_firmware_version=" << info.motor2.firmware_version << "\n"
        << info.motor3.name << "_model=" << info.motor3.model << "\n"
        << info.motor3.name << "_firmware_version=" << info.motor3.firmware_version << "\n";
}

pdu::JointCommand MakePositionCommand(double theta1, double theta2) {
    pdu::JointCommand command;
    command.theta1_rad = theta1;
    command.theta2_rad = theta2;
    return command;
}

pdu::JointCommand MakeVelocityCommand(double theta1_vel, double theta2_vel) {
    pdu::JointCommand command;
    command.theta1_velocity_rad_s = theta1_vel;
    command.theta2_velocity_rad_s = theta2_vel;
    return command;
}

pdu::JointCommand MakeCurrentCommand(double theta1_cur, double theta2_cur) {
    pdu::JointCommand command;
    command.theta1_effort_ff = theta1_cur;
    command.theta2_effort_ff = theta2_cur;
    return command;
}

pdu::JointCommand MakePdCommand(
    double theta1,
    double theta2,
    double theta1_vel,
    double theta2_vel,
    double theta1_ff,
    double theta2_ff) {
    pdu::JointCommand command;
    command.theta1_rad = theta1;
    command.theta2_rad = theta2;
    command.theta1_velocity_rad_s = theta1_vel;
    command.theta2_velocity_rad_s = theta2_vel;
    command.theta1_effort_ff = theta1_ff;
    command.theta2_effort_ff = theta2_ff;
    return command;
}

int RunDemo(pdu::ParallelDriveUnit& unit) {
    unit.Start();
    const pdu::AppConfig& config = unit.Config();
    if (!config.control.auto_enable) {
        unit.Enable();
    }

    const pdu::JointState initial = unit.ReadState();
    std::cout << "Initial state:\n";
    PrintState(initial);

    const double duration_s = config.demo.duration_s > 0.0 ? config.demo.duration_s : 1.0;
    const int ramp_steps = config.demo.ramp_steps > 0 ? config.demo.ramp_steps : 1;
    const double loop_hz = config.runtime.loop_hz > 0.0 ? config.runtime.loop_hz : 200.0;
    const auto period = std::chrono::duration<double>(1.0 / loop_hz);

    for (int step = 1; step <= ramp_steps; ++step) {
        const double alpha = static_cast<double>(step) / static_cast<double>(ramp_steps);
        pdu::JointCommand command;
        command.theta1_rad = Lerp(initial.theta1_rad, config.demo.theta1_target_rad, alpha);
        command.theta2_rad = Lerp(initial.theta2_rad, config.demo.theta2_target_rad, alpha);
        command.theta1_velocity_rad_s = (config.demo.theta1_target_rad - initial.theta1_rad) / duration_s;
        command.theta2_velocity_rad_s = (config.demo.theta2_target_rad - initial.theta2_rad) / duration_s;

        unit.CommandJoints(command);

        if (config.runtime.print_feedback && (step == 1 || step == ramp_steps || step % 50 == 0)) {
            PrintState(unit.ReadState());
        }

        std::this_thread::sleep_for(period);
    }

    const pdu::JointCommand hold_command = MakePdCommand(
        config.demo.theta1_target_rad,
        config.demo.theta2_target_rad,
        0.0,
        0.0,
        0.0,
        0.0);

    const int hold_steps = static_cast<int>(std::ceil(config.demo.hold_s * loop_hz));
    for (int i = 0; i < hold_steps; ++i) {
        unit.CommandJoints(hold_command);
        if (config.runtime.print_feedback && (i == 0 || i == hold_steps - 1 || i % 100 == 0)) {
            PrintState(unit.ReadState());
        }
        std::this_thread::sleep_for(period);
    }

    std::cout << "Final state:\n";
    PrintState(unit.ReadState());
    return 0;
}

int RunMonitor(pdu::ParallelDriveUnit& unit, int count, int period_ms) {
    unit.Connect();
    const int samples = count > 0 ? count : 10;
    const auto period = std::chrono::milliseconds(period_ms > 0 ? period_ms : 100);
    for (int i = 0; i < samples; ++i) {
        PrintState(unit.ReadState());
        if (i + 1 < samples) {
            std::this_thread::sleep_for(period);
        }
    }
    return 0;
}

std::vector<std::string> SplitCommandLine(const std::string& line) {
    std::istringstream input(line);
    std::vector<std::string> parts;
    std::string token;
    while (input >> token) {
        parts.emplace_back(token);
    }
    return parts;
}

int ExecuteCommand(pdu::ParallelDriveUnit& unit, const std::vector<std::string>& args, bool shell_mode);

int RunShell(pdu::ParallelDriveUnit& unit) {
    unit.Connect();
    std::cout << "interactive shell started, type help for commands, quit to exit\n";

    std::string line;
    while (std::cout << "> " && std::getline(std::cin, line)) {
        const std::vector<std::string> parts = SplitCommandLine(line);
        if (parts.empty()) {
            continue;
        }

        const std::string& cmd = parts.front();
        if (cmd == "quit" || cmd == "exit") {
            return 0;
        }

        try {
            ExecuteCommand(unit, parts, true);
        } catch (const std::exception& ex) {
            std::cout << "failed: " << ex.what() << "\n";
        }
    }

    return 0;
}

int ExecuteCommand(pdu::ParallelDriveUnit& unit, const std::vector<std::string>& args, bool shell_mode) {
    const std::string& cmd = args.front();

    if (cmd == "help") {
        PrintUsage();
        return 0;
    }

    if (cmd == "dump-config") {
        PrintConfig(unit.Config());
        return 0;
    }

    if (cmd == "info") {
        unit.Connect();
        PrintHardwareInfo(unit, unit.QueryHardwareInfo());
        PrintState(unit.ReadState());
        return 0;
    }

    if (cmd == "state") {
        unit.Connect();
        PrintState(unit.ReadState());
        return 0;
    }

    if (cmd == "clear-fault") {
        unit.Connect();
        unit.ClearFault();
        std::cout << "ok\n";
        return 0;
    }

    if (cmd == "enable") {
        unit.Connect();
        unit.Enable();
        std::cout << "ok\n";
        return 0;
    }

    if (cmd == "disable") {
        unit.Connect();
        unit.Disable();
        std::cout << "ok\n";
        return 0;
    }

    if (cmd == "zero") {
        unit.Connect();
        unit.ZeroOutput();
        std::cout << "ok\n";
        return 0;
    }

    if (cmd == "mode") {
        if (args.size() < 2) {
            throw std::runtime_error("mode requires a value");
        }
        unit.Connect();
        unit.SetMode(pdu::ParseCommandMode(args[1]));
        std::cout << "ok\n";
        return 0;
    }

    if (cmd == "set-joint-pos") {
        if (args.size() < 3) {
            throw std::runtime_error("set-joint-pos requires <theta1_rad> <theta2_rad>");
        }
        unit.Connect();
        unit.SetMode(pdu::CommandMode::kPosition);
        unit.Enable();
        unit.CommandJoints(MakePositionCommand(ParseDoubleArg(args[1], "theta1_rad"), ParseDoubleArg(args[2], "theta2_rad")));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PrintState(unit.ReadState());
        return 0;
    }

    if (cmd == "set-joint-vel") {
        if (args.size() < 3) {
            throw std::runtime_error("set-joint-vel requires <theta1_vel_rad_s> <theta2_vel_rad_s>");
        }
        unit.Connect();
        unit.SetMode(pdu::CommandMode::kVelocity);
        unit.Enable();
        unit.CommandJoints(MakeVelocityCommand(ParseDoubleArg(args[1], "theta1_vel_rad_s"), ParseDoubleArg(args[2], "theta2_vel_rad_s")));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PrintState(unit.ReadState());
        return 0;
    }

    if (cmd == "set-joint-cur") {
        if (args.size() < 3) {
            throw std::runtime_error("set-joint-cur requires <theta1_current_a> <theta2_current_a>");
        }
        unit.Connect();
        unit.SetMode(pdu::CommandMode::kCurrent);
        unit.Enable();
        unit.CommandJoints(MakeCurrentCommand(ParseDoubleArg(args[1], "theta1_current_a"), ParseDoubleArg(args[2], "theta2_current_a")));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PrintState(unit.ReadState());
        return 0;
    }

    if (cmd == "set-pd-target") {
        if (args.size() < 5) {
            throw std::runtime_error("set-pd-target requires <theta1_rad> <theta2_rad> <theta1_vel_rad_s> <theta2_vel_rad_s> [theta1_ff_a] [theta2_ff_a]");
        }
        const double theta1_ff = args.size() >= 6 ? ParseDoubleArg(args[5], "theta1_ff_a") : 0.0;
        const double theta2_ff = args.size() >= 7 ? ParseDoubleArg(args[6], "theta2_ff_a") : 0.0;

        unit.Connect();
        unit.SetMode(pdu::CommandMode::kPd);
        unit.Enable();
        unit.CommandJoints(MakePdCommand(
            ParseDoubleArg(args[1], "theta1_rad"),
            ParseDoubleArg(args[2], "theta2_rad"),
            ParseDoubleArg(args[3], "theta1_vel_rad_s"),
            ParseDoubleArg(args[4], "theta2_vel_rad_s"),
            theta1_ff,
            theta2_ff));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PrintState(unit.ReadState());
        return 0;
    }

    if (cmd == "monitor") {
        const int count = args.size() >= 2 ? ParseIntArg(args[1], "count") : 10;
        const int period_ms = args.size() >= 3 ? ParseIntArg(args[2], "period_ms") : 100;
        return RunMonitor(unit, count, period_ms);
    }

    if (cmd == "demo") {
        return RunDemo(unit);
    }

    if (cmd == "shell") {
        if (shell_mode) {
            std::cout << "already in shell\n";
            return 0;
        }
        return RunShell(unit);
    }

    throw std::runtime_error("Unknown command: " + cmd);
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const CliOptions options = ParseArgs(argc, argv);
        pdu::AppConfig config = pdu::LoadConfig(options.config_path);
        if (!options.backend_override.empty()) {
            config.runtime.backend = options.backend_override;
        }

        pdu::ParallelDriveUnit unit(config);
        const std::vector<std::string> command_args = options.command_args.empty()
            ? std::vector<std::string>{"demo"}
            : options.command_args;

        return ExecuteCommand(unit, command_args, false);
    } catch (const std::exception& ex) {
        std::cerr << "parallel_drive_app failed: " << ex.what() << "\n";
        return 1;
    }
}
