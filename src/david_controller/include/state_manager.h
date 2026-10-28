#pragma once

#include <chrono>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>

constexpr std::size_t UR5E_DOF = 6;

enum class TrajectoryMode {
    UNINITIALIZED = -1,
    INACTIVE = 0,
    ACTIVE = 1,
    COMPLETED = 2
};


enum class ControlMode {
    SAFETY = 0,
    STARTUP = 1,
    OPERATION = 2,
    DEADMAN = 3,
    TEST = 4
};



struct PGains {
    double Kp = 0.251;
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

struct TrajectoryPoint {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

struct ScalarPoint {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
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

class StateManager {    
    public:
        void set_arm_state(const ArmState& new_data);
        ArmState get_arm_state();
        void set_control_mode(const ControlMode& new_mode);
        ControlMode get_control_mode();
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
        void setJointPositions(const sensor_msgs::msg::JointState& msg);
        std::array<double, 6> getJointPositions();
        void setLeftJointPositions(const sensor_msgs::msg::JointState& msg);
        std::array<double, 6> getLeftJointPositions();
        void setRightJointPositions(const sensor_msgs::msg::JointState& msg);
        std::array<double, 6> getRightJointPositions();
        void setLeftTarget(const geometry_msgs::msg::PoseStamped& target);
        geometry_msgs::msg::PoseStamped getLeftTarget();
        void setRightTarget(const geometry_msgs::msg::PoseStamped& target);
        geometry_msgs::msg::PoseStamped getRightTarget();
        void setVRHeadState(const geometry_msgs::msg::PoseStamped& head_state);
        geometry_msgs::msg::PoseStamped getVRHeadState();
        

    private:
        ArmState arm_state_;
        std::mutex arm_state_mutex_;
        ControlMode control_mode_;
        std::mutex control_mode_mutex_;

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

        std::mutex joint_positions_mutex_;
        std::array<double, 6> joint_positions_{};

        std::mutex left_joint_positions_mutex_;
        std::array<double, 6> left_joint_positions_{};
        std::mutex right_joint_positions_mutex_;
        std::array<double, 6> right_joint_positions_{};

        geometry_msgs::msg::PoseStamped left_target_;
        std::mutex left_target_mutex_;
        geometry_msgs::msg::PoseStamped right_target_;
        std::mutex right_target_mutex_;
        geometry_msgs::msg::PoseStamped vr_head_state_;
        std::mutex vr_head_state_mutex_;

        static constexpr std::array<const char*, 6> JOINT_NAMES = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

};
