# pragma once

#include <array>
#include <cstddef>
#include <algorithm>

#include "state_manager.h"
#include "kinematics.h"

struct DlsGains {
    double k_lin      = 3.0;   // [1/s]
    double k_ang      = 3.0;   // [1/s]
    double v_max      = 0.25;  // [m/s]
    double w_max      = 1.0;   // [rad/s]
    double eps        = 0.08;  // sigma_min where damping starts
    double lambda_max = 0.08;  // damping at sigma_min = 0
    double qdot_max   = 0.5;   // [rad/s]
};

enum class RotationMapping { WORLD, TOOL };

class Controller
{
    public:
        void set_gains(const PGains& p_gains);
        std::array<double, 6> p_controller(const std::array<double, 6>& current,
                                               const std::array<double, 6>& target);
        Eigen::Matrix4d calculate_relative_target(const Eigen::Matrix4d& A0,
                                          const Eigen::Matrix4d& C0,
                                          const Eigen::Matrix4d& C,
                                          double k,
                                          double k_rot,
                                          const Eigen::Matrix3d& X,
                                        const Eigen::Matrix3d& controller_to_tool,
                                                  RotationMapping mapping);
        Eigen::Matrix<double, 6, 1> pose_error(const Eigen::Matrix4d& T_current,
                                                   const Eigen::Matrix4d& T_target) const;
        std::array<double, 6> dls_controller(const Eigen::MatrixXd& Jacobian, double sigma_min,
                                                 const Eigen::Matrix4d& T_current, const Eigen::Matrix4d& T_target);
        std::array<double, 6> joint_limit_guard(const std::array<double, 6>& q,
                                                std::array<double, 6> qdot,
                                                int* limiting_joint = nullptr) const;
        std::array<double, 6> joint_trajectory_controller(const std::array<double, 6>& current,
                                                const std::array<double, 6>& desired_pos,
                                                const std::array<double, 6>& desired_vel) const;
    private:
        PGains p_gains_;
        DlsGains dls_;
        static constexpr std::array<double, 6> Q_MAX = { 2*M_PI,  2*M_PI,  M_PI,  2*M_PI,  2*M_PI,  2*M_PI};
        static constexpr std::array<double, 6> Q_MIN = {-2*M_PI, -2*M_PI, -M_PI, -2*M_PI, -2*M_PI, -2*M_PI};
        static constexpr double LIMIT_SLOW = 30.0 * M_PI / 180.0;   // start slowing down this far from a limit
        static constexpr double LIMIT_STOP =  5.0 * M_PI / 180.0;
};
