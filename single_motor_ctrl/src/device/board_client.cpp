#include "device/board_client.h"

#include <array>
#include <cstdio>
#include <utility>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
#endif

extern "C" {
#include "RobotControl.h"
}

namespace smc {
namespace {

#if defined(__unix__) || defined(__APPLE__)
class ScopedStdoutSilencer {
public:
    ScopedStdoutSilencer() {
        fflush(stdout);
        null_fd_ = open("/dev/null", O_WRONLY);
        if (null_fd_ < 0) {
            return;
        }
        saved_stdout_ = dup(STDOUT_FILENO);
        if (saved_stdout_ < 0) {
            close(null_fd_);
            null_fd_ = -1;
            return;
        }
        if (dup2(null_fd_, STDOUT_FILENO) < 0) {
            close(saved_stdout_);
            close(null_fd_);
            saved_stdout_ = -1;
            null_fd_ = -1;
        }
    }

    ~ScopedStdoutSilencer() {
        if (saved_stdout_ >= 0) {
            fflush(stdout);
            dup2(saved_stdout_, STDOUT_FILENO);
            close(saved_stdout_);
        }
        if (null_fd_ >= 0) {
            close(null_fd_);
        }
    }

private:
    int saved_stdout_{-1};
    int null_fd_{-1};
};
#else
class ScopedStdoutSilencer {
public:
    ScopedStdoutSilencer() = default;
};
#endif

}  // namespace

BoardClient::BoardClient(BoardConfig config) : config_(std::move(config)) {}

BoardClient::~BoardClient() {
    disconnect();
}

bool BoardClient::connect() {
    if (connected_ && ctx_ != nullptr) {
        return true;
    }

    ctx_ = robot_create(config_.dev_id);
    if (ctx_ == nullptr) {
        connected_ = false;
        return false;
    }

    const int ok = robot_config_net(ctx_,
                                    config_.local_ip.c_str(),
                                    config_.local_port,
                                    config_.remote_port,
                                    config_.remote_ip.c_str());
    if (!ok) {
        robot_destroy(ctx_);
        ctx_ = nullptr;
        connected_ = false;
        return false;
    }

    connected_ = true;
    if (config_.fast_mode_enabled) {
        setFastMode(true, config_.fast_mode_hz);
    }
    return true;
}

void BoardClient::disconnect() {
    if (ctx_ != nullptr) {
        ScopedStdoutSilencer silencer;
        robot_destroy(ctx_);
        ctx_ = nullptr;
    }
    connected_ = false;
}

bool BoardClient::isConnected() const noexcept {
    return connected_ && ctx_ != nullptr;
}

bool BoardClient::setFastMode(bool enable, int hz) {
    if (ctx_ == nullptr) {
        return false;
    }
    return robot_set_fast_mode(ctx_, enable ? 1 : 0, hz) != 0;
}

bool BoardClient::queryMotorAddress(std::uint16_t& can_id, std::uint16_t& can_line_id) const {
    if (ctx_ == nullptr) {
        return false;
    }

    unsigned short raw_can_id = 0;
    unsigned short raw_can_line_id = 0;
    if (!robot_motor_get_motor_id(ctx_, &raw_can_id, &raw_can_line_id)) {
        return false;
    }

    can_id = raw_can_id;
    can_line_id = raw_can_line_id;
    return true;
}

std::string BoardClient::queryFirmwareVersion() const {
    if (ctx_ == nullptr) {
        return {};
    }
    std::array<char, 128> buffer{};
    const int ok = robot_motor_get_mother_board_firmware_version(ctx_, buffer.data());
    if (!ok) {
        return {};
    }
    return std::string(buffer.data());
}

std::string BoardClient::sdkVersion() const {
    char* version = get_sdk_version();
    if (version == nullptr) {
        return {};
    }
    return std::string(version);
}

RobotCtx* BoardClient::raw() const noexcept {
    return ctx_;
}

const BoardConfig& BoardClient::config() const noexcept {
    return config_;
}

}  // namespace smc
