#include "common/config_loader.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace smc {
namespace {

std::string trim(const std::string& input) {
    auto begin = input.begin();
    while (begin != input.end() && std::isspace(static_cast<unsigned char>(*begin))) {
        ++begin;
    }
    auto end = input.end();
    do {
        if (end == begin) {
            break;
        }
        --end;
    } while (std::isspace(static_cast<unsigned char>(*end)));
    return begin == input.end() ? std::string{} : std::string(begin, end + 1);
}

bool toBool(const std::string& value) {
    const std::string lower = [&]() {
        std::string s = value;
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }();
    return lower == "1" || lower == "true" || lower == "yes" || lower == "on";
}

}  // namespace

bool ConfigLoader::load(const std::string& path, BoardConfig& board, MotorConfig& motor, std::string* error) {
    std::ifstream in(path);
    if (!in) {
        if (error) {
            *error = "failed to open config file: " + path;
        }
        return false;
    }

    std::unordered_map<std::string, std::string> kv;
    std::string line;
    std::size_t line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        const auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        const auto sep = line.find('=');
        if (sep == std::string::npos) {
            if (error) {
                *error = "invalid config line " + std::to_string(line_no) + ": " + line;
            }
            return false;
        }
        kv[trim(line.substr(0, sep))] = trim(line.substr(sep + 1));
    }

    auto apply = [&](const std::string& key, auto&& fn) {
        auto it = kv.find(key);
        if (it != kv.end()) {
            fn(it->second);
        }
    };

    apply("board.local_ip", [&](const std::string& v) { board.local_ip = v; });
    apply("board.local_port", [&](const std::string& v) { board.local_port = std::stoi(v); });
    apply("board.remote_ip", [&](const std::string& v) { board.remote_ip = v; });
    apply("board.remote_port", [&](const std::string& v) { board.remote_port = std::stoi(v); });
    apply("board.dev_id", [&](const std::string& v) { board.dev_id = static_cast<std::uint16_t>(std::stoul(v, nullptr, 0)); });
    apply("board.fast_mode_enabled", [&](const std::string& v) { board.fast_mode_enabled = toBool(v); });
    apply("board.fast_mode_hz", [&](const std::string& v) { board.fast_mode_hz = std::stoi(v); });

    apply("motor.can_id", [&](const std::string& v) { motor.can_id = static_cast<std::uint16_t>(std::stoul(v, nullptr, 0)); });
    apply("motor.can_line_id", [&](const std::string& v) { motor.can_line_id = static_cast<std::uint16_t>(std::stoul(v, nullptr, 0)); });
    apply("motor.pos_kp", [&](const std::string& v) { motor.pos_kp = std::stod(v); });
    apply("motor.vel_kp", [&](const std::string& v) { motor.vel_kp = std::stod(v); });
    apply("motor.vel_ki", [&](const std::string& v) { motor.vel_ki = std::stod(v); });
    apply("motor.pd_kp", [&](const std::string& v) { motor.pd_kp = std::stod(v); });
    apply("motor.pd_kd", [&](const std::string& v) { motor.pd_kd = std::stod(v); });
    apply("motor.default_position_rad", [&](const std::string& v) { motor.default_position_rad = std::stod(v); });
    apply("motor.default_velocity_rad_s", [&](const std::string& v) { motor.default_velocity_rad_s = std::stod(v); });
    apply("motor.default_current_a", [&](const std::string& v) { motor.default_current_a = std::stod(v); });

    return true;
}

}  // namespace smc
