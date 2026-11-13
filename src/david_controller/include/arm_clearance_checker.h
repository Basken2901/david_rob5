#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_scene/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

class ArmClearanceChecker
{
public:
    explicit ArmClearanceChecker(rclcpp::Node& node);

    void check_clearance(const sensor_msgs::msg::JointState& msg);
    bool motion_allowed() const;

private:
    void load_model(const std::string& description);

    rclcpp::Node& node_;
    std::vector<std::string> joint_names_;
    std::unique_ptr<planning_scene::PlanningScene> scene_;
    collision_detection::AllowedCollisionMatrix collision_matrix_;
    bool clearance_safe_ = false;
    std::chrono::steady_clock::time_point last_valid_check_{};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
};
