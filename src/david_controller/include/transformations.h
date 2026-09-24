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
};