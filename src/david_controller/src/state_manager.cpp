#include "state_manager.h"




void StateManager::set_arm_state(const ArmState& new_data) {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    arm_state_ = new_data;
}

ArmState StateManager::get_arm_state() {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    return arm_state_;
}

void StateManager::set_control_mode(const ControlMode& mode) {
    std::lock_guard<std::mutex> lock(control_mode_mutex_);
    control_mode_ = mode;
}

ControlMode StateManager::get_control_mode() {
    std::lock_guard<std::mutex> lock(control_mode_mutex_);
    return control_mode_;
}

