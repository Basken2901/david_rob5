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

///Arm state management local

void StateManager::setLocalPosition(const Stamped3DVector& position) {
    std::lock_guard<std::mutex> lock(arm_position_mutex_);
    arm_position_ = position;
}

Stamped3DVector StateManager::getLocalPosition() {
    std::lock_guard<std::mutex> lock(arm_position_mutex_);
    return arm_position_;
}

///Arm state management global

void StateManager::setGlobalPosition(const Stamped3DVector& position) {
    std::lock_guard<std::mutex> lock(arm_global_position_mutex_);
    arm_global_position_ = position;
}

Stamped3DVector StateManager::getGlobalPosition() {
    std::lock_guard<std::mutex> lock(arm_global_position_mutex_);
    return arm_global_position_;
}

void StateManager::setGlobalOrientation(const Eigen::Quaterniond& orientation) {
    std::lock_guard<std::mutex> lock(arm_global_orientation_mutex_);
    arm_global_orientation_ = orientation;
}

Eigen::Quaterniond StateManager::getGlobalOrientation() {
    std::lock_guard<std::mutex> lock(arm_global_orientation_mutex_);
    return arm_global_orientation_;
}

void StateManager::setGlobalVelocity(const Stamped3DVector& velocity) {
    std::lock_guard<std::mutex> lock(arm_global_velocity_mutex_);
    arm_global_velocity_ = velocity;
}

Stamped3DVector StateManager::getGlobalVelocity() {
    std::lock_guard<std::mutex> lock(arm_global_velocity_mutex_);
    return arm_global_velocity_;
}

void StateManager::setGlobalAcceleration(const Stamped3DVector& acceleration) {
    std::lock_guard<std::mutex> lock(arm_global_acceleration_mutex_);
    arm_global_acceleration_ = acceleration;
}

Stamped3DVector StateManager::getGlobalAcceleration() {
    std::lock_guard<std::mutex> lock(arm_global_acceleration_mutex_);
    return arm_global_acceleration_;
}

///Joint state management

void StateManager::setJointState(const JointState& state) {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    joint_state_ = state;
}

JointState StateManager::getJointState() {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    return joint_state_;
}