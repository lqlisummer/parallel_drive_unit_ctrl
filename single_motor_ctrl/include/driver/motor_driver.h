#pragma once

#include <string>

#include "domain/motor_types.h"

struct RobotMotor;

namespace smc {

class BoardClient;

class MotorDriver {
public:
    MotorDriver(BoardClient& board, MotorConfig config);
    ~MotorDriver();

    MotorDriver(const MotorDriver&) = delete;
    MotorDriver& operator=(const MotorDriver&) = delete;

    bool create();
    void destroy();

    bool isCreated() const noexcept;
    bool setControlMode(MotorControlMode mode);
    bool setControlWord(ControlWord word);
    bool enable();
    bool disable();
    bool clearFault();

    void setPid(double pos_kp, double vel_kp, double vel_ki);
    void setPd(double kp, double kd);
    void setPosition(double position_rad);
    void setVelocity(double velocity_rad_s);
    void setCurrent(double current_a);
    void setPosVelCur(double position_rad, double velocity_rad_s, double current_a);
    void flushCommand();

    MotorTelemetry readPvct() const;
    MotorTelemetry readPvctFast() const;
    int readEncoder() const;

    std::string queryMotorModel() const;
    std::string queryFirmwareVersion() const;

    bool setZero();
    bool calibrateEncoder();
    bool setStrongDragging(ThetaSource source, double voltage);
    bool getStrongDraggingMode(ThetaSource& source) const;

    const MotorConfig& config() const noexcept;

private:
    BoardClient& board_;
    MotorConfig config_;
    RobotMotor* motor_{nullptr};
    bool created_{false};
};

}  // namespace smc
