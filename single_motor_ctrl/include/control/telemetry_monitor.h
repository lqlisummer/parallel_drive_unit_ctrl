#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "domain/motor_types.h"

namespace smc {

class MotorDriver;

class TelemetryMonitor {
public:
    explicit TelemetryMonitor(MotorDriver& driver);
    ~TelemetryMonitor();

    TelemetryMonitor(const TelemetryMonitor&) = delete;
    TelemetryMonitor& operator=(const TelemetryMonitor&) = delete;

    void start(int period_ms, bool fast_feedback = false);
    void stop();
    bool running() const noexcept;
    MotorTelemetry latest() const;

private:
    void loop(int period_ms, bool fast_feedback);

    MotorDriver& driver_;
    mutable std::mutex mutex_;
    MotorTelemetry latest_{};
    std::thread worker_;
    std::atomic_bool running_{false};
};

}  // namespace smc
