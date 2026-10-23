#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>




struct EulerAngles {
    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
};



struct Transformations{
    double unwrapAngle(double angle, double max, double min) const;
    EulerAngles quaternionToEuler(const Eigen::Quaterniond& q) const;
    static Eigen::Matrix4d link_transformation_matrix(double theta, double alpha, double a, double d);
    static Eigen::Matrix4d standard_link(double theta, double alpha, double a, double d);
    static Eigen::Matrix4d inverse_rigid(const Eigen::Matrix4d& T);
    static double wrap(double angle);
    Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) const;
};