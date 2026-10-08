#include "robot_state_subscriber.h"
#include <Eigen/Geometry>
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


RobotStateSubscriber::RobotStateSubscriber(
    const std::shared_ptr<StateManager>& state_manager)
    : Node("robot_state_subscriber"),
      state_manager_(state_manager)
{
    // Frames used for Cartesian state
    base_frame_ = this->declare_parameter<std::string>(
        "base_frame", "base_link");

    tool_frame_ = this->declare_parameter<std::string>(
        "tool_frame", "tool0");


    // Joint states
    joint_state_sub_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(
                &RobotStateSubscriber::jointStateCallback,
                this,
                std::placeholders::_1));


    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
        this->get_clock());

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_);


    // Check TF periodically
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(
            &RobotStateSubscriber::updateCartesianState,
            this));
}

void RobotStateSubscriber::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    JointState state;

    state.timestamp = msg->header.stamp;
    state.names = msg->name;
    state.positions = msg->position;
    state.velocities = msg->velocity;
    state.efforts = msg->effort;

    state_manager_->setJointState(state);
}

void RobotStateSubscriber::updateCartesianState()
{
    try
    {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(
                base_frame_,
                tool_frame_,
                tf2::TimePointZero);


        // Position
        Stamped3DVector position(
            rclcpp::Time(transform.header.stamp),
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        state_manager_->setGlobalPosition(position);


        // Orientation
        const auto& rotation = transform.transform.rotation;

        Eigen::Quaterniond orientation(
            rotation.w,
            rotation.x,
            rotation.y,
            rotation.z);

        state_manager_->setGlobalOrientation(orientation);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_DEBUG(
            this->get_logger(),
            "Could not get transform %s -> %s: %s",
            base_frame_.c_str(),
            tool_frame_.c_str(),
            ex.what());
    }
}