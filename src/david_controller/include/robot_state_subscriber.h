#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "state_manager.h"


class RobotStateSubscriber : public rclcpp::Node
{
public:
    explicit RobotStateSubscriber(
        const std::shared_ptr<StateManager>& state_manager);

private:

    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg);

    void updateCartesianState();

    std::shared_ptr<StateManager> state_manager_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr tf_timer_;

    std::string base_frame_;
    std::string tool_frame_;
};