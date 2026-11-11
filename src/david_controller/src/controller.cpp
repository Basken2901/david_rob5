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
    //T.block<3,3>(0,0) = A0.block<3,3>(0,0);   
    T.block<3,1>(0,3) = A0.block<3,1>(0,3) + k * dp;
    return T;
}

Eigen::Matrix<double, 6, 1> Controller::pose_error(const Eigen::Matrix4d& T_current,
                                                   const Eigen::Matrix4d& T_target) const
{
    Eigen::Matrix<double, 6, 1> error_matrix;
    error_matrix.head<3>() = T_target.block<3,1>(0,3) - T_current.block<3,1>(0,3);

 
    const Eigen::Matrix3d Rotation_error = T_target.block<3,3>(0,0) * T_current.block<3,3>(0,0).transpose();
    const Eigen::AngleAxisd aa(Rotation_error);
    error_matrix.tail<3>() = aa.angle() * aa.axis();
    return error_matrix;
}

std::array<double, 6> Controller::dls_controller(const Eigen::MatrixXd& J, double sigma_min,
                                                 const Eigen::Matrix4d& T_current, const Eigen::Matrix4d& T_target)
{
    // 1) error -> desired twist (cap the length, keep the direction)
    const Eigen::Matrix<double, 6, 1> error_matrix = pose_error(T_current, T_target);
    Eigen::Vector3d v = dls_.k_lin * error_matrix.head<3>();
    Eigen::Vector3d w = dls_.k_ang * error_matrix.tail<3>();
    if (v.norm() > dls_.v_max) v *= dls_.v_max / v.norm();
    if (w.norm() > dls_.w_max) w *= dls_.w_max / w.norm();
    Eigen::Matrix<double, 6, 1> twist;
    twist << v, w;

    // 2) variable damping: 0 when sigma_min >= eps, rises to lambda_max at sigma_min = 0
    double lambda2 = 0.0;
    if (sigma_min < dls_.eps) {
        const double r = sigma_min / dls_.eps;
        lambda2 = (1.0 - r * r) * dls_.lambda_max * dls_.lambda_max;
    }

    // 3) qdot = J^T (J J^T + lambda^2 I)^-1 twist  -> solve, never invert
    const Eigen::Matrix<double, 6, 6> A =
        J * J.transpose() + lambda2 * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> qdot_e = J.transpose() * A.ldlt().solve(twist);

    // 4) NaN guard + uniform scaling (all joints together, so the direction is kept)
    std::array<double, 6> qdot{};
    double worst = 0.0;
    for (int i = 0; i < 6; ++i) {
        if (!std::isfinite(qdot_e(i))) { qdot.fill(0.0); return qdot; }
        qdot[i] = qdot_e(i);
        worst = std::max(worst, std::abs(qdot[i]));
    }
    if (worst > dls_.qdot_max)
        for (auto& x : qdot) x *= dls_.qdot_max / worst;
    return qdot;
}
