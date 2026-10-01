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

struct Pose{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
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
        Pose forward_kinematics(const DHParam& dh, const TempJoint& th);
    private:
        Transformations transformations;
};

class InverseKinematics
{
    public:
        static Eigen::Matrix4d pose_to_matrix(const Pose& position);
        std::vector<TempJoint> all_solutions(const DHParam& dh, const Pose& target) const;
        std::optional<TempJoint> inverse_kinematics(const DHParam& dh, const Pose& target, const TempJoint& current) const;
    private:
        Transformations transformations;
};