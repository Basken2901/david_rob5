#include "kinematics.h"


//using ur5e {
//
//
//}


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

