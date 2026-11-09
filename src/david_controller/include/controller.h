# pragma once

#include <array>
#include <cstddef>
#include <algorithm>

#include "state_manager.h"
#include "kinematics.h"

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
    private:
            PGains p_gains_;
};
