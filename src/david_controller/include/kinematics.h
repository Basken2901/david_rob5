#pragma once

#include <eigen3/Eigen/Dense>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

#include "transformations.h"

struct ForwardMovements{
    double x;
    double y;
    double z;
};
struct DHParam {
    std::array<double, 6> alpha = {0.0, M_PI / 2, 0.0, 0.0, M_PI / 2, -M_PI / 2};
    std::array<double, 6> a     = {0.0, 0.0, -0.425, -0.3922, 0.0, 0.0};
    std::array<double, 6> d     = {0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996};
};

struct TempJoint {
    std::array<double, 6> theta = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

struct JointsTransform {
    Eigen::Matrix4d Joint_1;
    Eigen::Matrix4d Joint_2;
    Eigen::Matrix4d Joint_3;
    Eigen::Matrix4d Joint_4;
    Eigen::Matrix4d Joint_5;
    Eigen::Matrix4d Joint_6;
};

class ForwardKinematics
{
    public:
        ForwardMovements forward_kinematics(const DHParam& dh, const TempJoint& th);
    private:
        Transformations transformations;
};