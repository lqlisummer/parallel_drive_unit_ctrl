#pragma once

#include "control/safety_manager.h"
#include "control/telemetry_monitor.h"
#include "device/board_client.h"
#include "driver/motor_driver.h"

namespace smc {

class SingleMotorController {
public:
    SingleMotorController(BoardConfig board_cfg, MotorConfig motor_cfg);
    ~SingleMotorController();

    SingleMotorController(const SingleMotorController&) = delete;
    SingleMotorController& operator=(const SingleMotorController&) = delete;

    bool initialize();
    bool start(MotorControlMode mode, bool fast_feedback = false, int monitor_period_ms = 10);
    void stop();
    void shutdown();

    bool setMode(MotorControlMode mode);
    void moveTo(double position_rad);
    void spin(double velocity_rad_s);
    void applyCurrent(double current_a);
    void setPdTarget(double position_rad, double velocity_rad_s, double current_a);

    bool clearFault();
    bool enable();
    bool disable();
    bool zero();
    bool calibrate();
    bool setStrongDragging(ThetaSource source, double voltage);

    bool detectMotorAddress(std::uint16_t& can_id, std::uint16_t& can_line_id) const;
    MotorTelemetry telemetry() const;
    MotorIdentity identify() const;
    MotorLifecycleState state() const noexcept;
    MotorControlMode activeMode() const noexcept;
    MotorDriver& driver() noexcept;
    const MotorDriver& driver() const noexcept;

private:
    BoardClient board_;
    MotorDriver driver_;
    SafetyManager safety_;
    TelemetryMonitor monitor_;
    MotorLifecycleState state_{MotorLifecycleState::Disconnected};
    MotorControlMode active_mode_{MotorControlMode::None};
};

}  // namespace smc
