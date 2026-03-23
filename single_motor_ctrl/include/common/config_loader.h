#pragma once

#include <string>

#include "domain/motor_types.h"

namespace smc {

class ConfigLoader {
public:
    static bool load(const std::string& path, BoardConfig& board, MotorConfig& motor, std::string* error = nullptr);
};

}  // namespace smc
