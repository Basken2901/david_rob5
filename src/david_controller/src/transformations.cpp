#include "transformations.h"

double Transformations::unwrapAngle(double angle, double max, double min) 
const {
    // Unwrap the angle to be within the range [min, max]    
    while (angle > max) angle -= 2.0 * M_PI;
    while (angle < min) angle += 2.0 * M_PI;
    return angle;
}

EulerAngles Transformations::quaternionToEuler(const Eigen::Quaterniond& q) const {
    // Get Euler angles in ZYX convention. eulerAngles(2,1,0) returns [Z, Y, X] = [yaw, pitch, roll].
    Eigen::Vector3d zyx = q.toRotationMatrix().eulerAngles(2, 1, 0);

    // Extract yaw, pitch, roll
    double yaw = zyx.x();
    double pitch = zyx.y();
    double roll = zyx.z();

    // Normalize pitch to [-π/2, π/2] to avoid gimbal lock ambiguities
    if (std::abs(pitch) > M_PI / 2) {
        // Adjust yaw and flip pitch and roll to maintain equivalent rotation
        yaw += M_PI;
        pitch = M_PI - pitch; // Reflect pitch around π
        roll += M_PI;
    }

    // Unwrap angles to [0, 2π]
    yaw = unwrapAngle(yaw, 2 * M_PI, 0);
    pitch = unwrapAngle(pitch, 2 * M_PI, 0);
    roll = unwrapAngle(roll, 2 * M_PI, 0);

    return EulerAngles{roll, pitch, yaw};
}


Eigen::Matrix4d Transformations::link_transformation_matrix(double theta, double alpha, double a, double d)
{
    const double ct = std::cos(theta), st = std::sin(theta);
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    Eigen::Matrix4d T;
    T << ct, -st, 0, a,
        st*ca, ct*ca, -sa, -sa*d,
        st*sa, ct*sa, ca, ca*d,
        0, 0, 0,1;
    return T;
}

Eigen::Matrix4d Transformations::standard_link(double theta, double alpha, double a, double d)
{
    const double ct = std::cos(theta), st = std::sin(theta);
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    Eigen::Matrix4d T;
    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0,        sa,       ca,      d,
         0,         0,        0,      1;
    return T;
}

Eigen::Matrix4d Transformations::inverse_rigid(const Eigen::Matrix4d& T)
{
    Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
    Ti.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
    Ti.block<3, 1>(0, 3) = -Ti.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);
    return Ti;
}

double Transformations::wrap(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle <= 0.0) angle += 2.0 * M_PI;
    return angle - M_PI;
}

Eigen::Vector3d Transformations::quaternion_to_euler(const Eigen::Quaterniond& q) const {
    // Get Euler angles in ZYX convention (yaw, pitch, roll)
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    
    // Extract yaw, pitch, roll
    double yaw = euler.x();
    double pitch = euler.y();
    double roll = euler.z();

    // Normalize pitch to [-π/2, π/2] to avoid gimbal lock ambiguities
    if (std::abs(pitch) > M_PI / 2) {
        // Adjust yaw and flip pitch and roll to maintain equivalent rotation
        yaw += M_PI;
        pitch = M_PI - pitch; // Reflect pitch around π
        roll += M_PI;
    }

    // Unwrap angles to [0, 2π]
    yaw = unwrapAngle(yaw, M_PI, -M_PI);
    pitch = unwrapAngle(pitch, 2 * M_PI, 0);
    roll = unwrapAngle(roll, 2 * M_PI, 0);

    // Ensure yaw is in [0, 2π] and consistent with input
    return Eigen::Vector3d(roll, pitch, yaw);
}

