#include "path_planner.h"
#include "state_manager.h"

#include <algorithm>
#include <cmath>

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

bool PathPlanner::generate_trajectory(const JointArray& q_start, const JointArray& q_end,
                                      const JointArray& qd_start, const JointArray& qdd_start)
{
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

    for (std::size_t i = 0; i < UR5E_DOF; ++i) {
        if (!std::isfinite(q_start[i]) || !std::isfinite(q_end[i]) ||
            !std::isfinite(qd_start[i]) || !std::isfinite(qdd_start[i]))
            return false;
    }

    // One shared duration, set by the joint that moves furthest, so all joints start and stop together
    total_time = calculateDuration(q_start, q_end);

    for (std::size_t i = 0; i < UR5E_DOF; ++i) {
        segments[i].coefficient = generate_polynomium_coefficients(
            q_start[i], q_end[i], qd_start[i], qdd_start[i], total_time);
    }
    has_trajectory_ = true;
    return true;
}

double PathPlanner::calculateDuration(const JointArray& q_start, const JointArray& q_end) const
{
    // Rest-to-rest 7th order polynomial over distance D in time T:
    //   peak |q_dot|  = 35/16         * D / T
    //   peak |q_ddot| = 84*sqrt(5)/25 * D / T^2
    constexpr double PEAK_VEL_FACTOR = 35.0 / 16.0;
    const double PEAK_ACC_FACTOR = 84.0 * std::sqrt(5.0) / 25.0;

    double duration = min_duration_;
    for (std::size_t i = 0; i < UR5E_DOF; ++i) {
        const double distance = std::fabs(q_end[i] - q_start[i]);
        const double t_vel = PEAK_VEL_FACTOR * distance / max_joint_velocity_;
        const double t_acc = std::sqrt(PEAK_ACC_FACTOR * distance / max_joint_acceleration_);
        duration = std::max({duration, t_vel, t_acc});
    }
    return duration;
}

void PathPlanner::set_limits(double max_joint_velocity, double max_joint_acceleration, double min_duration)
{
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    if (max_joint_velocity > 0.0)     max_joint_velocity_     = max_joint_velocity;
    if (max_joint_acceleration > 0.0) max_joint_acceleration_ = max_joint_acceleration;
    if (min_duration > 0.0)           min_duration_           = min_duration;
}

double PathPlanner::get_total_time() const {
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    return total_time;
}

bool PathPlanner::has_trajectory() const {
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);
    return has_trajectory_;
}


ScalarPoint PathPlanner::evaluate_polynomial(const std::vector<double>& coefficient, double time) const{
    ScalarPoint point;   // already zeroed by default members

    for (size_t i = 0; i < coefficient.size(); ++i)
        point.position += coefficient[i] * std::pow(time, i);

    for (size_t i = 1; i < coefficient.size(); ++i)
        point.velocity += i * coefficient[i] * std::pow(time, i - 1);

    for (size_t i = 2; i < coefficient.size(); ++i)
        point.acceleration += i * (i - 1) * coefficient[i] * std::pow(time, i - 2);

    return point;
}

JointTrajectoryPoint PathPlanner::get_trajectory_point(double time) const {
    std::lock_guard<std::recursive_mutex> lock(planner_mutex_);

    // The polynomial runs away outside [0, T], so never evaluate it there
    const double t = std::clamp(time, 0.0, total_time);

    JointTrajectoryPoint point;
    for (std::size_t i = 0; i < UR5E_DOF; ++i) {
        const ScalarPoint s = evaluate_polynomial(segments[i].coefficient, t);
        point.position[i]     = s.position;
        point.velocity[i]     = s.velocity;
        point.acceleration[i] = s.acceleration;
    }
    return point;
}