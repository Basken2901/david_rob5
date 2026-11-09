#include "controller.h"

void Controller::set_gains(
    const PGains& p_gains
)
{
    p_gains_ = p_gains;
}

std::array<double, 6> Controller::p_controller(const std::array<double, 6>& current,
                                               const std::array<double, 6>& target)
{
    const double kp = p_gains_.Kp;
    const double max_vel = 0.5;  // rad/s, safety limit

    std::array<double, 6> vel{};
    for (size_t i = 0; i < 6; ++i)
    {
        double error = target[i] - current[i];
        vel[i] = std::clamp(kp * error, -max_vel, max_vel);
    }
    return vel;
}



Eigen::Matrix4d Controller::calculate_relative_target(const Eigen::Matrix4d& A0,
                                                      const Eigen::Matrix4d& C0,
                                                      const Eigen::Matrix4d& C,
                                                      double k,
                                                      const Eigen::Matrix3d& X)
{
    // How far the hand moved since the latch, converted to robot/world axes
    const Eigen::Vector3d dp = X * (C.block<3,1>(0,3) - C0.block<3,1>(0,3));

    // How much the hand rotated since the latch, converted the same way
    const Eigen::Matrix3d dR_vr = C.block<3,3>(0,0) * C0.block<3,3>(0,0).transpose();
    const Eigen::Matrix3d dR    = X * dR_vr * X.transpose();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = dR * A0.block<3,3>(0,0);
    T.block<3,1>(0,3) = A0.block<3,1>(0,3) + k * dp;
    return T;
}
