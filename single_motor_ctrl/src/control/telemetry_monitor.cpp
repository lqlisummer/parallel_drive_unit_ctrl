#include "control/telemetry_monitor.h"

#include <chrono>

#include "driver/motor_driver.h"

namespace smc {

TelemetryMonitor::TelemetryMonitor(MotorDriver& driver) : driver_(driver) {}

TelemetryMonitor::~TelemetryMonitor() {
    stop();
}

void TelemetryMonitor::start(int period_ms, bool fast_feedback) {
    if (running_) {
        return;
    }
    running_ = true;
    worker_ = std::thread(&TelemetryMonitor::loop, this, period_ms, fast_feedback);
}

void TelemetryMonitor::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool TelemetryMonitor::running() const noexcept {
    return running_;
}

MotorTelemetry TelemetryMonitor::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

void TelemetryMonitor::loop(int period_ms, bool fast_feedback) {
    const auto period = std::chrono::milliseconds(period_ms > 0 ? period_ms : 10);
    while (running_) {
        MotorTelemetry telemetry = fast_feedback ? driver_.readPvctFast() : driver_.readPvct();
        telemetry.encoder_value = driver_.readEncoder();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = telemetry;
        }
        std::this_thread::sleep_for(period);
    }
}

}  // namespace smc
