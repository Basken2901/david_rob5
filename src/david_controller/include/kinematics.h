#pragma once

#include <eigen3/Eigen/Dense>
#include <string>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>
//#include <Eigen/SVD> // for svd of jacobian

#include "transformations.h"
#include <std_msgs/msg/float64_multi_array.hpp>

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

    TempJoint() = default;
    TempJoint(const std::array<double, 6>& a) : theta(a) {} //Fucking cool! A constructor so we can just insert a normal std::array into this
};

struct JointsTransform {
    Eigen::Matrix4d Joint_1;
    Eigen::Matrix4d Joint_2;
    Eigen::Matrix4d Joint_3;
    Eigen::Matrix4d Joint_4;
    Eigen::Matrix4d Joint_5;
    Eigen::Matrix4d Joint_6;
};


struct SingularityInfo{ //to keep information about singularity of the robot
    Eigen::VectorXd singular_values; // values from svd (sigma)(sorted: largest first)
    double sigma_min; // smallest singular value - close to zero -> close to singularity
    double sigma_max; // largest singular value 
    double condition_number; // ratio of largest to smallest singular value (can also be used to determine closeness to singularity)
};


class SingularityPrevention
{
    public:

        // std_msgs::msg::String singularity_msg; // message to be published to the user about the singularity info
        double scaling_factor = 1.0; //scaling factor for velocity

        
        Eigen::MatrixXd jacobian_computation(const DHParam& dh, const TempJoint& th); // declarion of jacobian_computation function (function 1)

        SingularityInfo compute_singularity_info(const Eigen::MatrixXd & J) const; // declaration of compute_singularity_info function (function 2)

        std::array<double, 6> velocity_scaler(const SingularityInfo & info, const std::array<double, 6>& cmd); // declaration of velocity scaling function 1)
        // std_msgs::msg::Float64MultiArray velocity_scaler(const SingularityInfo & info, const std_msgs::msg::Float64MultiArray& cmd); // declaration of velocity scaling function 3)

        //std_msgs::msg::Float64MultiArray prevent_singularity(const geometry_msgs::msg::TwistStamped& twist, const double scaling_factor) const; // declaration of prevent_singularity function return vel output  by scaling the velocity.
        
    private:
        double lower_singularity_threshold = 0.01; //threshold for full stop
        double upper_singularity_threshold = 0.1; //threshold for full speed
        
        Transformations transformations;
};



class ForwardKinematics
{
    public:
        // ForwardMovements forward_kinematics(const DHParam& dh, const TempJoint& th); !!!!!!!!!!!
         //used in the jacobian_computation function
        Eigen::Matrix4d forward_kinematics_matrix(const DHParam& dh, const TempJoint& th) ;
        Pose forward_kinematics(const DHParam& dh, const TempJoint& th);
    private:
        Transformations transformations;
};

class InverseKinematics
{
    public:
        static Eigen::Matrix4d pose_to_matrix(const Pose& position);
        std::vector<TempJoint> all_solutions(const DHParam& dh, const Eigen::Matrix4d& T_0_6) const;
        std::optional<TempJoint> inverse_kinematics(const DHParam& dh, const Eigen::Matrix4d& T_0_6,
                                            const TempJoint& current) const;
        std::optional<TempJoint> inverse_kinematics(const DHParam& dh, const Pose& target,
                                                    const TempJoint& current) const;
    private:
        Transformations transformations;
};