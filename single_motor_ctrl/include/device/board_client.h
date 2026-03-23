#pragma once

#include <string>

#include "domain/motor_types.h"

struct RobotCtx;

namespace smc {

class BoardClient {
public:
    explicit BoardClient(BoardConfig config);
    ~BoardClient();

    BoardClient(const BoardClient&) = delete;
    BoardClient& operator=(const BoardClient&) = delete;

    bool connect();
    void disconnect();

    bool isConnected() const noexcept;
    bool setFastMode(bool enable, int hz);
    bool queryMotorAddress(std::uint16_t& can_id, std::uint16_t& can_line_id) const;
    std::string queryFirmwareVersion() const;
    std::string sdkVersion() const;

    RobotCtx* raw() const noexcept;
    const BoardConfig& config() const noexcept;

private:
    BoardConfig config_;
    RobotCtx* ctx_{nullptr};
    bool connected_{false};
};

}  // namespace smc
