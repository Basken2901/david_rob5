#include "path_planner.h"
#include "state_manager.h"

std::vector<double> PathPlanner::generate_polynomium_coefficients(
    double start, double end, double start_vel, double start_acc, double time)
{
    Eigen::MatrixXd A(8, 8);
    Eigen::VectorXd B(8);

    A << 1, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0, 0, 0,
         0, 0, 0, 6, 0, 0, 0, 0,
         1, time, pow(time, 2), pow(time, 3), pow(time, 4), pow(time, 5), pow(time, 6), pow(time, 7),
         0, 1, 2 * time, 3 * pow(time, 2), 4 * pow(time, 3), 5 * pow(time, 4), 6 * pow(time, 5), 7 * pow(time, 6),
         0, 0, 2, 6 * time, 12 * pow(time, 2), 20 * pow(time, 3), 30 * pow(time, 4), 42 * pow(time, 5),
         0, 0, 0, 6, 24 * time, 60 * pow(time, 2), 120 * pow(time, 3), 210 * pow(time, 4);

    B << start, start_vel, start_acc, 0, end, 0, 0, 0;

    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(B);

    return std::vector<double>{coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7]};
}

bool PathPlanner::generate_trajectory(
    const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos, const Eigen::Quaterniond& start_quat, const Eigen::Quaterniond& end_quat,
    const Eigen::Vector3d& current_velocity, const Eigen::Vector3d& current_acceleration)
{
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    
    float distance = (end_pos - start_pos).norm();
    float current_yaw = transformations.unwrapAngle(transformations.quaternionToEuler(start_quat).yaw, 2 * M_PI, 0);
    float target_yaw = transformations.unwrapAngle(transformations.quaternionToEuler(end_quat).yaw, 2 * M_PI, 0);
    float distance_angular = std::fabs(std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw))); //Returns abosulute value of atan2
    float trajectory_duration_cartesian = calculateDuration(distance, current_velocity.norm(), min_linear_velocity_, max_linear_velocity_);
    float trajectory_duration_angular = calculateDuration(distance_angular, current_angular_velocity_, min_angular_velocity_, max_angular_velocity_); //! MIGHT HAVE TO FIX FOR ANGULAR!
    total_time = std::max({trajectory_duration_cartesian, trajectory_duration_angular, 0.1f});
    
    start_vel_ = current_velocity;
    start_acc_ = current_acceleration;
    start_quat_ = start_quat.normalized();
    end_quat_ = end_quat.normalized();

    for (int i = 0; i < 3; ++i) {
            segments[i].coefficient = generate_polynomium_coefficients(
                start_pos(i), end_pos(i), start_vel_(i), start_acc_(i), total_time);
        }
    return true;
} 


float PathPlanner::calculateDuration(float distance, float velocity, float min_velocity, float max_velocity) const {
    // Clamp velocity to valid range
    float safe_velocity = std::clamp(velocity, min_velocity, max_velocity);

    // Calculate and return duration
    return distance / safe_velocity;
}

double PathPlanner::get_total_time() const {
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    return total_time;
}
