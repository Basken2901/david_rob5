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
                                          const Eigen::Matrix3d& X);
        Eigen::Matrix<double, 6, 1> pose_error(const Eigen::Matrix4d& T_current,
                                                   const Eigen::Matrix4d& T_target) const;
        std::array<double, 6> dls_controller(const Eigen::MatrixXd& Jacobian, double sigma_min,
                                                 const Eigen::Matrix4d& T_current, const Eigen::Matrix4d& T_target);
    private:
            PGains p_gains_;
            DlsGains dls_;
};
