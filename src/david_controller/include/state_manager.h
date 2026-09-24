#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"

enum class TrajectoryMode {
    UNINITIALIZED = -1,
    INACTIVE = 0,
    ACTIVE = 1,
    COMPLETED = 2
};

struct ArmState {
    //! Create ros timestamp
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    TrajectoryMode trajectory_mode = TrajectoryMode::INACTIVE;
    rclcpp::Time trajectory_start_time;
    rclcpp::Duration trajectory_duration = rclcpp::Duration(0, 0);
};

class StateManager {
    public:
        void set_arm_state(const ArmState& new_data);
        ArmState get_arm_state();
        

    private:
        ArmState arm_state_;
        std::mutex arm_state_mutex_;
 

};