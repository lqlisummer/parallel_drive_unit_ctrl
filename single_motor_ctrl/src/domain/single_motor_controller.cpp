#include "domain/single_motor_controller.h"

#include <utility>

namespace smc {

SingleMotorController::SingleMotorController(BoardConfig board_cfg, MotorConfig motor_cfg)
    : board_(std::move(board_cfg)),
      driver_(board_, std::move(motor_cfg)),
      safety_(driver_),
      monitor_(driver_) {}

SingleMotorController::~SingleMotorController() {
    shutdown();
}

bool SingleMotorController::initialize() {
    if (!board_.connect()) {
        state_ = MotorLifecycleState::Disconnected;
        return false;
    }
    state_ = MotorLifecycleState::Connected;
    if (!driver_.create()) {
        return false;
    }
    state_ = MotorLifecycleState::MotorCreated;
    return true;
}

bool SingleMotorController::start(MotorControlMode mode, bool fast_feedback, int monitor_period_ms) {
    if (!driver_.isCreated()) {
        return false;
    }
    if (!safety_.startupSequence(mode)) {
        state_ = MotorLifecycleState::Fault;
        return false;
    }
    active_mode_ = mode;
    state_ = MotorLifecycleState::Enabled;
    monitor_.start(monitor_period_ms, fast_feedback);
    state_ = MotorLifecycleState::Running;
    return true;
}

void SingleMotorController::stop() {
    monitor_.stop();
    safety_.shutdownSequence();
    state_ = MotorLifecycleState::Stopped;
}

void SingleMotorController::shutdown() {
    monitor_.stop();
    safety_.shutdownSequence();
    driver_.destroy();
    board_.disconnect();
    active_mode_ = MotorControlMode::None;
    state_ = MotorLifecycleState::Disconnected;
}

bool SingleMotorController::setMode(MotorControlMode mode) {
    if (!driver_.isCreated()) {
        return false;
    }
    if (!driver_.setControlMode(mode)) {
        return false;
    }
    active_mode_ = mode;
    return true;
}

void SingleMotorController::moveTo(double position_rad) {
    driver_.setPosition(position_rad);
}

void SingleMotorController::spin(double velocity_rad_s) {
    driver_.setVelocity(velocity_rad_s);
}

void SingleMotorController::applyCurrent(double current_a) {
    driver_.setCurrent(current_a);
}

void SingleMotorController::setPdTarget(double position_rad, double velocity_rad_s, double current_a) {
    driver_.setPosVelCur(position_rad, velocity_rad_s, current_a);
    driver_.flushCommand();
}

bool SingleMotorController::clearFault() {
    return safety_.clearFault();
}

bool SingleMotorController::enable() {
    return safety_.enable();
}

bool SingleMotorController::disable() {
    return safety_.disable();
}

bool SingleMotorController::zero() {
    return safety_.zeroIfNeeded();
}

bool SingleMotorController::calibrate() {
    return safety_.calibrateIfNeeded();
}

bool SingleMotorController::setStrongDragging(ThetaSource source, double voltage) {
    return driver_.setStrongDragging(source, voltage);
}

bool SingleMotorController::detectMotorAddress(std::uint16_t& can_id, std::uint16_t& can_line_id) const {
    return board_.queryMotorAddress(can_id, can_line_id);
}

MotorTelemetry SingleMotorController::telemetry() const {
    return monitor_.latest();
}

MotorIdentity SingleMotorController::identify() const {
    MotorIdentity id;
    id.model = driver_.queryMotorModel();
    id.firmware_version = driver_.queryFirmwareVersion();
    id.board_firmware_version = board_.queryFirmwareVersion();
    return id;
}

MotorLifecycleState SingleMotorController::state() const noexcept {
    return state_;
}

MotorControlMode SingleMotorController::activeMode() const noexcept {
    return active_mode_;
}

MotorDriver& SingleMotorController::driver() noexcept {
    return driver_;
}

const MotorDriver& SingleMotorController::driver() const noexcept {
    return driver_;
}

}  // namespace smc
