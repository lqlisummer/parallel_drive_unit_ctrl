#include "pdu/config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace pdu {
namespace {

using SectionMap = std::unordered_map<std::string, std::unordered_map<std::string, std::string>>;

std::string Trim(const std::string& input) {
    const auto begin = std::find_if_not(input.begin(), input.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    const auto end = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();
    if (begin >= end) {
        return {};
    }
    return std::string(begin, end);
}

std::string ToLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

bool ParseBool(const std::string& value) {
    const std::string normalized = ToLower(Trim(value));
    if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
        return true;
    }
    if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
        return false;
    }
    throw std::runtime_error("Invalid boolean value: " + value);
}

int ParseInt(const std::string& value) {
    return std::stoi(Trim(value), nullptr, 0);
}

uint16_t ParseU16(const std::string& value) {
    return static_cast<uint16_t>(std::stoul(Trim(value), nullptr, 0));
}

double ParseDouble(const std::string& value) {
    return std::stod(Trim(value));
}

SectionMap ParseIni(const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    SectionMap sections;
    std::string current_section;
    std::string line;
    int line_number = 0;

    while (std::getline(input, line)) {
        ++line_number;
        const std::string trimmed = Trim(line);
        if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';') {
            continue;
        }

        if (trimmed.front() == '[' && trimmed.back() == ']') {
            current_section = ToLower(Trim(trimmed.substr(1, trimmed.size() - 2)));
            continue;
        }

        const auto equal_pos = trimmed.find('=');
        if (equal_pos == std::string::npos) {
            std::ostringstream oss;
            oss << "Invalid config line " << line_number << ": " << line;
            throw std::runtime_error(oss.str());
        }

        const std::string key = ToLower(Trim(trimmed.substr(0, equal_pos)));
        const std::string value = Trim(trimmed.substr(equal_pos + 1));
        sections[current_section][key] = value;
    }

    return sections;
}

template <typename Callback>
void ApplySection(const SectionMap& sections, const std::string& section_name, Callback&& callback) {
    const auto it = sections.find(section_name);
    if (it == sections.end()) {
        return;
    }
    for (const auto& [key, value] : it->second) {
        callback(key, value);
    }
}

}  // namespace

std::string ToString(CommandMode mode) {
    switch (mode) {
        case CommandMode::kPositionOnly:
            return "position_only";
        case CommandMode::kPdSync:
            return "pd_sync";
    }
    return "unknown";
}

CommandMode ParseCommandMode(const std::string& value) {
    const std::string normalized = ToLower(Trim(value));
    if (normalized == "position" || normalized == "position_only") {
        return CommandMode::kPositionOnly;
    }
    if (normalized == "pd" || normalized == "pd_sync") {
        return CommandMode::kPdSync;
    }
    throw std::runtime_error("Unsupported command mode: " + value);
}

AppConfig LoadConfig(const std::string& path) {
    AppConfig config;
    const SectionMap sections = ParseIni(path);

    ApplySection(sections, "network", [&](const std::string& key, const std::string& value) {
        if (key == "board_id") {
            config.network.board_id = ParseU16(value);
        } else if (key == "local_ip") {
            config.network.local_ip = value;
        } else if (key == "local_port") {
            config.network.local_port = ParseInt(value);
        } else if (key == "remote_ip") {
            config.network.remote_ip = value;
        } else if (key == "remote_port") {
            config.network.remote_port = ParseInt(value);
        } else {
            throw std::runtime_error("Unknown [network] key: " + key);
        }
    });

    ApplySection(sections, "motor2", [&](const std::string& key, const std::string& value) {
        if (key == "name") {
            config.motor2.name = value;
        } else if (key == "can_id") {
            config.motor2.can_id = ParseU16(value);
        } else if (key == "can_line") {
            config.motor2.can_line = ParseU16(value);
        } else {
            throw std::runtime_error("Unknown [motor2] key: " + key);
        }
    });

    ApplySection(sections, "motor3", [&](const std::string& key, const std::string& value) {
        if (key == "name") {
            config.motor3.name = value;
        } else if (key == "can_id") {
            config.motor3.can_id = ParseU16(value);
        } else if (key == "can_line") {
            config.motor3.can_line = ParseU16(value);
        } else {
            throw std::runtime_error("Unknown [motor3] key: " + key);
        }
    });

    ApplySection(sections, "control", [&](const std::string& key, const std::string& value) {
        if (key == "command_mode") {
            config.control.command_mode = ParseCommandMode(value);
        } else if (key == "auto_enable") {
            config.control.auto_enable = ParseBool(value);
        } else if (key == "clear_fault_on_start") {
            config.control.clear_fault_on_start = ParseBool(value);
        } else if (key == "zero_on_start") {
            config.control.zero_on_start = ParseBool(value);
        } else if (key == "position_kp") {
            config.control.pid.position_kp = ParseDouble(value);
        } else if (key == "velocity_kp") {
            config.control.pid.velocity_kp = ParseDouble(value);
        } else if (key == "velocity_ki") {
            config.control.pid.velocity_ki = ParseDouble(value);
        } else if (key == "pd_kp") {
            config.control.pd.kp = ParseDouble(value);
        } else if (key == "pd_kd") {
            config.control.pd.kd = ParseDouble(value);
        } else {
            throw std::runtime_error("Unknown [control] key: " + key);
        }
    });

    ApplySection(sections, "feedback", [&](const std::string& key, const std::string& value) {
        if (key == "fast_mode") {
            config.feedback.fast_mode = ParseBool(value);
        } else if (key == "fast_period_hz") {
            config.feedback.fast_period_hz = ParseInt(value);
        } else {
            throw std::runtime_error("Unknown [feedback] key: " + key);
        }
    });

    ApplySection(sections, "runtime", [&](const std::string& key, const std::string& value) {
        if (key == "backend") {
            config.runtime.backend = ToLower(Trim(value));
        } else if (key == "loop_hz") {
            config.runtime.loop_hz = ParseDouble(value);
        } else if (key == "synchronized_send") {
            config.runtime.synchronized_send = ParseBool(value);
        } else if (key == "startup_flush_cycles") {
            config.runtime.startup_flush_cycles = ParseInt(value);
        } else if (key == "print_feedback") {
            config.runtime.print_feedback = ParseBool(value);
        } else {
            throw std::runtime_error("Unknown [runtime] key: " + key);
        }
    });

    ApplySection(sections, "demo", [&](const std::string& key, const std::string& value) {
        if (key == "theta1_target_rad") {
            config.demo.theta1_target_rad = ParseDouble(value);
        } else if (key == "theta2_target_rad") {
            config.demo.theta2_target_rad = ParseDouble(value);
        } else if (key == "duration_s") {
            config.demo.duration_s = ParseDouble(value);
        } else if (key == "hold_s") {
            config.demo.hold_s = ParseDouble(value);
        } else if (key == "ramp_steps") {
            config.demo.ramp_steps = ParseInt(value);
        } else {
            throw std::runtime_error("Unknown [demo] key: " + key);
        }
    });

    return config;
}

}  // namespace pdu
