#pragma once

#include <chrono>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
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

};