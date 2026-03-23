#include "control/safety_manager.h"

#include "driver/motor_driver.h"

namespace smc {

SafetyManager::SafetyManager(MotorDriver& driver) : driver_(driver) {}

bool SafetyManager::startupSequence(MotorControlMode mode) {
    if (!driver_.isCreated()) {
        return false;
    }
    if (!driver_.setControlMode(mode)) {
        return false;
    }

    const auto& cfg = driver_.config();
    driver_.setPid(cfg.pos_kp, cfg.vel_kp, cfg.vel_ki);
    if (mode == MotorControlMode::Pd) {
        driver_.setPd(cfg.pd_kp, cfg.pd_kd);
    }
    driver_.setPosVelCur(cfg.default_position_rad, cfg.default_velocity_rad_s, cfg.default_current_a);
    driver_.flushCommand();
    return driver_.enable();
}

void SafetyManager::shutdownSequence() {
    if (!driver_.isCreated()) {
        return;
    }
    driver_.setVelocity(0.0);
    driver_.setCurrent(0.0);
    driver_.disable();
}

bool SafetyManager::recoverFromFault() {
    return driver_.isCreated() && driver_.clearFault();
}

bool SafetyManager::clearFault() {
    return driver_.clearFault();
}

bool SafetyManager::enable() {
    return driver_.enable();
}

bool SafetyManager::disable() {
    return driver_.disable();
}

bool SafetyManager::zeroIfNeeded() {
    return driver_.isCreated() && driver_.setZero();
}

bool SafetyManager::calibrateIfNeeded() {
    return driver_.isCreated() && driver_.calibrateEncoder();
}

}  // namespace smc
