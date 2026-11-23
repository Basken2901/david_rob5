#pragma once

#include <iostream>

#include <vector>
#include <string>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <array>

#include "transformations.h"
#include "state_manager.h"

struct Waypoint {
    Eigen::Vector3d position;
    double yaw;
    double linear_velocity;   // 0 = use default
    double angular_velocity;  // 0 = use default
};

struct TrajectoryInitState {
    Eigen::Vector3d position;
    Eigen::Vector3d position_target_prev;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double yaw;
};

struct TrajectorySegment {
    std::vector<double> coefficient;
};

using JointArray = std::array<double, UR5E_DOF>;

struct JointTrajectoryPoint {
    JointArray position{};
    JointArray velocity{};
    JointArray acceleration{};
};

class PathPlanner
{
   public:
      std::vector<double> generate_polynomium_coefficients(double start, double end, double start_vel, double start_acc, double time);

      // One 7th order polynomial per joint from q_start to q_end, ending at rest
      bool generate_trajectory(const JointArray& q_start, const JointArray& q_end,
                               const JointArray& qd_start = {}, const JointArray& qdd_start = {});
      double calculateDuration(const JointArray& q_start, const JointArray& q_end) const;
      ScalarPoint evaluate_polynomial(const std::vector<double>& coefficient, double time) const;
      JointTrajectoryPoint get_trajectory_point(double time) const;   // time is clamped to [0, total_time]

      double get_total_time() const;
      bool has_trajectory() const;
      void set_limits(double max_joint_velocity, double max_joint_acceleration, double min_duration);
      const std::array<TrajectorySegment, UR5E_DOF>& get_segments() const { return segments; }   // !TEMP

   private:
      mutable std::recursive_mutex planner_mutex_;

      // Kept under the 0.5 rad/s clamp in the controller so the P term has room to correct
      double max_joint_velocity_     = 0.4;   // rad/s
      double max_joint_acceleration_ = 0.8;   // rad/s^2
      double min_duration_           = 0.5;   // s

      double total_time = 0.0;
      bool has_trajectory_ = false;
      std::array<TrajectorySegment, UR5E_DOF> segments;
};