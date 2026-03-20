#pragma once

#include <memory>

#include "pdu/motor_backend.hpp"

namespace pdu {

std::unique_ptr<IMotorBackend> CreateVendorBackend(const AppConfig& config);

}  // namespace pdu
