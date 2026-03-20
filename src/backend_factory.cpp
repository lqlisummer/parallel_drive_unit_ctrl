#include "pdu/motor_backend.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

#include "pdu/mock_motor_backend.hpp"

#if PDU_HAS_VENDOR_BACKEND
#include "pdu/vendor_sdk_backend.hpp"
#endif

namespace pdu {
namespace {

std::string Normalize(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

}  // namespace

std::unique_ptr<IMotorBackend> CreateBackend(const AppConfig& config) {
    const std::string backend = Normalize(config.runtime.backend);

    if (backend == "mock" || backend == "dry-run" || backend == "dryrun") {
        return std::make_unique<MockMotorBackend>(config);
    }

    if (backend == "vendor" || backend == "sdk" || backend == "hardware") {
#if PDU_HAS_VENDOR_BACKEND
        return CreateVendorBackend(config);
#else
        throw std::runtime_error("Vendor backend requested, but this build does not include the Linux MotorDrive SDK backend");
#endif
    }

    throw std::runtime_error("Unsupported backend: " + config.runtime.backend);
}

bool VendorBackendCompiled() {
#if PDU_HAS_VENDOR_BACKEND
    return true;
#else
    return false;
#endif
}

}  // namespace pdu
