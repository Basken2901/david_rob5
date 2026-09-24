#include "state_manager.h"




void StateManager::set_arm_state(const ArmState& new_data) {
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    arm_state_ = new_data;
}

ArmState StateManager::get_arm_state(){
    std::lock_guard<std::mutex> lock(arm_state_mutex_);
    return arm_state_;
}

