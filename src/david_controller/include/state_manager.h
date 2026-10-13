#pragma once

#include <chrono>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>

constexpr std::size_t UR5E_DOF = 6;

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

struct Stamped3DVector {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    Eigen::Vector3d data = Eigen::Vector3d::Zero();

    // Default constructor
    Stamped3DVector() = default;

    // Constructor with timestamp and components (added for controlMode and manualAidedMode)
    Stamped3DVector(const rclcpp::Time& ts, double x, double y, double z)
        : timestamp(ts), data(x, y, z) {}

    // Get individual components
    double x() const { return data.x(); }
    double y() const { return data.y(); }
    double z() const { return data.z(); }

    // Set individual components
    void setX(double value) { data.x() = value; }
    void setY(double value) { data.y() = value; }
    void setZ(double value) { data.z() = value; }

    // Get the full vector
    const Eigen::Vector3d& vector() const { return data; }
    Eigen::Vector3d& vector() { return data; }

    // Get and set timestamp
    rclcpp::Time getTime() const { return timestamp; }
    void setTime(const rclcpp::Time& new_time) { timestamp = new_time; }
};

struct JointState
{
    std::array<std::string, UR5E_DOF> names = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    std::array<double, UR5E_DOF> position{};
    std::array<double, UR5E_DOF> velocity{};
    std::array<double, UR5E_DOF> effort{};
};

struct VRState
{
    geometry_msgs::msg::PoseStamped left_pose;
    geometry_msgs::msg::PoseStamped right_pose;
    geometry_msgs::msg::PoseStamped head_pose;

    bool left_clutch = false;
    bool right_clutch = false;

    float left_gripper = 0.0f;
    float right_gripper = 0.0f;
};

class StateManager {    
    public:
        void set_arm_state(const ArmState& new_data);
        ArmState get_arm_state();
        void setLocalPosition(const Stamped3DVector& position);
        Stamped3DVector getLocalPosition();
        void setGlobalPosition(const Stamped3DVector& position);
        Stamped3DVector getGlobalPosition();
        void setGlobalOrientation(const Eigen::Quaterniond& orientation);
        Eigen::Quaterniond getGlobalOrientation();
        void setGlobalVelocity(const Stamped3DVector& velocity);
        Stamped3DVector getGlobalVelocity();
        void setGlobalAcceleration(const Stamped3DVector& acceleration);
        Stamped3DVector getGlobalAcceleration();
        void setJointStateLeft(const JointState& state);
        JointState getJointStateLeft();
        void setJointStateRight(const JointState& state);
        JointState getJointStateRight();
        void setVRControllerLeft(const VRState& state);
        VRState getVRControllerLeft();
        void setVRControllerRight(const VRState& state);
        VRState getVRControllerRight();
        void setVRHead(const VRState& state);
        VRState getVRHead();

    private:
        ArmState arm_state_;
        std::mutex arm_state_mutex_;
        Stamped3DVector arm_position_;
        std::mutex arm_position_mutex_;
        Stamped3DVector arm_global_position_;
        std::mutex arm_global_position_mutex_;
        Eigen::Quaterniond arm_global_orientation_;
        std::mutex arm_global_orientation_mutex_;
        Stamped3DVector arm_global_velocity_;
        std::mutex arm_global_velocity_mutex_;
        Stamped3DVector arm_global_acceleration_;
        std::mutex arm_global_acceleration_mutex_;
        JointState joint_state_left_;
        std::mutex joint_state_left_mutex_;
        JointState joint_state_right_;
        std::mutex joint_state_right_mutex_;
        VRState vr_controller_left_;
        std::mutex vr_controller_left_mutex_;
        VRState vr_controller_right_;
        std::mutex vr_controller_right_mutex_;
        VRState vr_head_;
        std::mutex vr_head_mutex_;

};