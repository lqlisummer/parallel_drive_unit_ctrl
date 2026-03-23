#pragma once

#include "domain/motor_types.h"

namespace smc {

class MotorDriver;

class SafetyManager {
public:
    explicit SafetyManager(MotorDriver& driver);

    bool startupSequence(MotorControlMode mode);
    void shutdownSequence();
    bool recoverFromFault();
    bool clearFault();
    bool enable();
    bool disable();
    bool zeroIfNeeded();
    bool calibrateIfNeeded();

private:
    MotorDriver& driver_;
};

}  // namespace smc
