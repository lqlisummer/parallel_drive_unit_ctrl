#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "pdu/config.hpp"
#include "pdu/motor_backend.hpp"
#include "pdu/parallel_drive_unit.hpp"

namespace {

struct CliOptions {
    std::string config_path = "config/parallel_drive_unit.ini";
    std::string backend_override;
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
                << "Usage: parallel_drive_app [--config <path>] [--backend <mock|vendor>]\n"
                << "Example: ./parallel_drive_app --config config/parallel_drive_unit.ini --backend mock\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return options;
}

double Lerp(double start, double end, double alpha) {
    return start + (end - start) * alpha;
}

void PrintSummary(const pdu::AppConfig& config, const pdu::ParallelDriveUnit& unit) {
    std::cout << "Parallel drive unit backend: " << unit.BackendName() << "\n";
    std::cout << "Motor m2 -> CAN ID " << config.motor2.can_id << ", line " << config.motor2.can_line << "\n";
    std::cout << "Motor m3 -> CAN ID " << config.motor3.can_id << ", line " << config.motor3.can_line << "\n";
    std::cout << "Command mode: " << pdu::ToString(config.control.command_mode) << "\n";
    std::cout << "Vendor backend compiled: " << (pdu::VendorBackendCompiled() ? "yes" : "no") << "\n";
}

void PrintState(const pdu::JointState& state) {
    std::cout << std::fixed << std::setprecision(4)
              << "theta1=" << state.theta1_rad << " rad, "
              << "theta2=" << state.theta2_rad << " rad, "
              << "m2=" << state.motor2.position_rad << " rad, "
              << "m3=" << state.motor3.position_rad << " rad, "
              << "m2_vel=" << state.motor2.velocity_rad_s << " rad/s, "
              << "m3_vel=" << state.motor3.velocity_rad_s << " rad/s\n";
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
        unit.Start();

        PrintSummary(config, unit);

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

        const pdu::JointCommand hold_command{
            config.demo.theta1_target_rad,
            config.demo.theta2_target_rad,
            0.0,
            0.0,
            0.0,
            0.0,
        };

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
        unit.Shutdown();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "parallel_drive_app failed: " << ex.what() << "\n";
        return 1;
    }
}
