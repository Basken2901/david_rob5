#include "kinematics.h"


//using ur5e {
//
//
//}


 Pose ForwardKinematics::forward_kinematics(const DHParam& dh, const TempJoint& th)
 {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i = 0; i < 6; i++) {
        T = T * transformations.link_transformation_matrix(th.theta[i], dh.alpha[i], dh.a[i], dh.d[i]);
    }
    
    Pose end_point;

    end_point.x = T(0, 3);
    end_point.y = T(1, 3);
    end_point.z = T(2, 3);
    return end_point;
 }

Eigen::Matrix4d InverseKinematics::pose_to_matrix(const Pose& position)
{
    const Eigen::Matrix3d rotation_m =
        (Eigen::AngleAxisd(position.yaw,   Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(position.pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(position.roll,  Eigen::Vector3d::UnitX())).toRotationMatrix(); //Its basically the same as Rz * Ry * Rx

    Eigen::Matrix4d transformation_m = Eigen::Matrix4d::Identity();
    transformation_m.block<3, 3>(0, 0) = rotation_m; //Puts the rotation matrix into the tranformation matrix

    transformation_m(0,3) = position.x;
    transformation_m(1,3) = position.y;
    transformation_m(2,3) = position.z;

    return transformation_m;
}

std::vector<TempJoint> InverseKinematics::all_solutions(const DHParam& dh, const Pose& target) const
{
    std::vector<TempJoint> solutions;

    const double a2 = dh.a[2], a3 = dh.a[3];
    const double d4 = dh.d[3], d6 = dh.d[5];

    const Eigen::Matrix4d T_0_6 = InverseKinematics::pose_to_matrix(target);
    const Eigen::Matrix3d R = T_0_6.block<3,3>(0,0);
    const Eigen::Vector3d p = T_0_6.block<3,1>(0,3);

    const Eigen::Vector3d p_0_5 = p - d6 * R.col(2);
    const double r_0_5 = std::hypot(p_0_5.x(), p_0_5.y());
    if (r_0_5 < std::abs(d4)) return solutions; //A reachability check, kinda a singularity check for the shoulder

    const double phi1 = std::atan2(p_0_5.y(), p_0_5.x());
    const double phi2 = std::acos(std::clamp(d4/r_0_5, -1.0, 1.0));

    for (int s1: {+1, -1}) {
        const double t1 = phi1 + s1 * phi2 + M_PI /2;
        const double c1 = std::cos(t1), sn1 = std::sin(t1);

        //Theta 5
        const double c5 = (p.x() * sn1 - p.y() * c1 - d4) / d6;
        if (std::abs(c5) > 1.0 + 1e-9) continue;
        const double t5abs = std::acos(std::clamp(c5, -1.0, 1.0));

        for (int s5 : {+1, -1}) {
            const double t5 = s5 * t5abs;
            const double sn5 = std::sin(t5);
 
            // theta6 (free when the wrist is singular, sin(theta5) = 0)
            double t6 = 0.0;
            if (std::abs(sn5) > 1e-9) {
                t6 = std::atan2((-R(0, 1) * sn1 + R(1, 1) * c1) / sn5,
                                ( R(0, 0) * sn1 - R(1, 0) * c1) / sn5);
            }
 
            // Reduce to a planar problem for joints 2, 3, 4
            const Eigen::Matrix4d T_0_1 = transformations.standard_link(t1, dh.alpha[1], dh.a[1], dh.d[0]);
            const Eigen::Matrix4d T_4_6 = transformations.standard_link(t5, dh.alpha[5], dh.a[5], dh.d[4])
                            * transformations.standard_link(t6, 0.0, 0.0, dh.d[5]);
            const Eigen::Matrix4d T_1_4 = transformations.inverse_rigid(T_0_1) * T_0_6 * transformations.inverse_rigid(T_4_6);
 
            const Eigen::Vector4d p13 = T_1_4 * Eigen::Vector4d(0.0, -d4, 0.0, 1.0);
            const double r13sq = p13.x() * p13.x() + p13.y() * p13.y();
            const double r13 = std::sqrt(r13sq);
 
            // theta3 (elbow up / down)
            const double c3 = (r13sq - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
            if (std::abs(c3) > 1.0 + 1e-9) continue;  // out of reach
            const double t3abs = std::acos(std::clamp(c3, -1.0, 1.0));

            for (int s3 : {+1, -1}) {
                const double t3 = s3 * t3abs;
                const double t2 = -std::atan2(p13.y(), -p13.x()) + std::asin(a3 * std::sin(t3) / r13);
 
                const Eigen::Matrix4d T_1_3 = transformations.standard_link(t2, dh.alpha[2], dh.a[2], dh.d[1])
                            * transformations.standard_link(t3, dh.alpha[3], dh.a[3], dh.d[2]);
                const Eigen::Matrix4d T_3_4 = transformations.inverse_rigid(T_1_3) * T_1_4;
                const double t4 = std::atan2(T_3_4(1, 0), T_3_4(0, 0));
 
                TempJoint temp_solutions;
                temp_solutions.theta = {transformations.wrap(t1), transformations.wrap(t2), transformations.wrap(t3), 
                    transformations.wrap(t4), transformations.wrap(t5), transformations.wrap(t6)};
                solutions.push_back(temp_solutions);
            }
        }
    }
    return solutions;

}

std::optional<TempJoint> InverseKinematics::inverse_kinematics(const DHParam& dh, const Pose& target,
                                                               const TempJoint& current) const
{
    std::optional<TempJoint> best;
    double best_cost = std::numeric_limits<double>::infinity();
 
    for (TempJoint sol : InverseKinematics::all_solutions(dh, target)) {
        double cost = 0.0;
        for (int i = 0; i < 6; i++) {
            // UR joints can rotate +-2pi, so pick the equivalent angle nearest the current one
            sol.theta[i] = current.theta[i] + transformations.wrap(sol.theta[i] - current.theta[i]);
            const double diff = sol.theta[i] - current.theta[i];
            cost += diff * diff;
        }
        if (cost < best_cost) {
            best_cost = cost;
            best = sol;
        }
    }
    return best;
}
