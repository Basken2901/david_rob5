#include "kinematics.h"


//using ur5e {
//
//
//}

Eigen::MatrixXd SingularityPrevention::jacobian_computation(const DHParam& dh, const TempJoint& th)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd J(6, 6); //Jacobian matrix

    std::vector<Eigen::Vector3d> z(6); // 6 of 3d vectors for the z-axis of each joint
    std::vector<Eigen::Vector3d> p(6); // 6 of 3d vectors for the position of each joint

    for (int i = 0; i < 6; i++) { //deriving the z-axis and position of each joint
        T = T * transformations.link_transformation_matrix(th.theta[i], dh.alpha[i], dh.a[i], dh.d[i]); // get the transformation from joint 0 to joint i. 
        z[i] = T.block<3, 1>(0, 2); // extract the z-axis of the current joint
        p[i] = T.block<3, 1>(0, 3); // Extract the position of the current joint
    }


    Eigen::Vector3d p_end = p[5]; // Position of the end-effector

    for (int i = 0; i < 6; i++) {
        Eigen::Vector3d Jv = z[i].cross(p_end - p[i]); // Linear velocity component
        Eigen::Vector3d Jw = z[i]; // Angular velocity component

        J.block<3, 1>(0, i) = Jv; // inserting the linear velocity component for joint i into jacobian matrix
        J.block<3, 1>(3, i) = Jw; // inserting the angular velocity component for joint i into jacobian matrix
    }

    return J;
}

SingularityInfo SingularityPrevention::compute_singularity_info(const Eigen::MatrixXd & J) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV); // U and V only neede for singular vectors not for singular values. 
    Eigen::VectorXd singular_values = svd.singularValues();

    SingularityInfo info; // varibale name: info, variabel type: SingularityInfo
    info.singular_values = singular_values;    //assign the values to the underlying variable under "info"
    info.sigma_min = singular_values.minCoeff();
    info.sigma_max = singular_values.maxCoeff();
    info.condition_number = info.sigma_max / info.sigma_min;

    return info;
}

std_msgs::msg::Float64MultiArray SingularityPrevention::velocity_scaler(const SingularityInfo & info, const std_msgs::msg::Float64MultiArray& cmd) // no "const" after is this wont allow is to assign new values to scalin factor variable
{
    const double sigma = info.sigma_min;

    if (!std::isfinite(sigma) || upper_singularity_threshold <= lower_singularity_threshold) {
        scaling_factor = 0.0;                      // can't judge the situation: stop
    } else {
        double x = (sigma - lower_singularity_threshold) /
                   (upper_singularity_threshold - lower_singularity_threshold); //maps sigma between lower an upper threshold. 
        x = std::clamp(x, 0.0, 1.0);               
        scaling_factor = x * x * (3.0 - 2.0 * x);  // smoothstep - s-curve (velocity changes gradually and not immidaitly as sigma changes) - https://en.wikipedia.org/wiki/Smoothstep
    }


    auto scaled = cmd;                       // keeps layout and size
    for (auto & v : scaled.data) v *= scaling_factor;

    //NaN og inf velocity cmd - returns zero if any of the scaled velocities are NaN or inf.
    for (double v : scaled.data) {
        if (!std::isfinite(v)) {
            std::fill(scaled.data.begin(), scaled.data.end(), 0.0);
            return scaled;
        }
    }
    //too fast? - takes the fastest joint velocity and scales all joint velocity down to the maximum allowed joint velocity to keep the trajectory. 
    const double qdot_max = 1.0;
    double worst = 0.0;
    for (double v : scaled.data) worst = std::max(worst, std::abs(v));

    if (worst > qdot_max) {
        for (auto & v : scaled.data) v *= qdot_max / worst;
    }
    string singularity_msg = "Singularity info: velocity scaling factor: " + std::to_string(scaling_factor) + ", sigma_min: " + std::to_string(info.sigma_min);
    return scaled;
}



ForwardMovements ForwardKinematics::forward_kinematics(const DHParam& dh, const TempJoint& th)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 6; i++) {
        T = T * transformations.link_transformation_matrix(th.theta[i], dh.alpha[i], dh.a[i], dh.d[i]);
    }
    
    ForwardMovements end_point;

    end_point.x = T(0, 3);
    end_point.y = T(1, 3);
    end_point.z = T(2, 3);
    return end_point;
}



