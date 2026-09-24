#pragma once

#include <iostream>

#include <vector>
#include <string>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "transformations.h"

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

class PathPlanner
{
   public:
      std::vector<double> generate_polynomium_coefficients(double start, double end, double start_vel, double start_acc, double time);
      bool generate_trajectory(
       const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos, const Eigen::Quaterniond& start_quat, const Eigen::Quaterniond& end_quat,
       const Eigen::Vector3d& current_velocity, const Eigen::Vector3d& current_acceleration);
      float calculateDuration(float distance, float velocity, float min_velocity, float max_velocity) const;


      double get_total_time() const;
      const TrajectorySegment (&get_segments() const)[3] { return segments; }   // !TEMP

   private:
      Transformations transformations;
      mutable std::recursive_mutex planner_mutex_;
      
      float current_angular_velocity_ = 0.15; //!! MIGHT HAVE TO CHANGE ALL THESE, PLUS MOVE TO BETTER PLACE
      float min_linear_velocity_ = 0.3;        
      float min_angular_velocity_ = 0.3;       
      float max_linear_velocity_ = 0.8;        
      float max_angular_velocity_ = 0.8;  

      double total_time;
      TrajectorySegment segments[3]; 
      
      Eigen::Vector3d start_vel_;
      Eigen::Vector3d start_acc_;
      Eigen::Quaterniond start_quat_;
      Eigen::Quaterniond end_quat_;
};
