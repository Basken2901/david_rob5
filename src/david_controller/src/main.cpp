#include <cstring>
#include <iostream>
#include "rclcpp/rclcpp.hpp"


#include "path_planner.h"
#include "state_manager.h"


void activate_trajectory(float trajectory_duration){
    StateManager state_manager; //! Temp
    ArmState arm_state = state_manager.get_arm_state();
    arm_state.trajectory_mode = TrajectoryMode::ACTIVE;
    rclcpp::Clock clock(RCL_SYSTEM_TIME); //! TEMP
    arm_state.trajectory_start_time = clock.now();
    arm_state.trajectory_duration = rclcpp::Duration::from_seconds(trajectory_duration);
    state_manager.set_arm_state(arm_state);
}

double evalPoly(const std::vector<double>& c, double t) { //! TEMP
    double result = 0.0;
    for (int k = static_cast<int>(c.size()) - 1; k >= 0; --k) {
        result = result * t + c[k];
    }
    return result;
}

void TestPlan(){
    PathPlanner planner;  
    std::vector<Waypoint> waypoints;

    TrajectoryInitState init_state = {
        .position = {0.0, 0.0, 0.0},
        .position_target_prev = {0.0, 0.0, 0.0},
        .orientation = {1.0, 0.0, 0.0, 0.0},
        .velocity = {0.0, 0.0, 0.0},
        .acceleration = {0.0, 0.0, 0.0},
        .yaw = 0.0
    };

    Eigen::Vector3d target_position = {2.0, 2.0, 1.0};
    Eigen::Quaterniond target_quaternion = {1.0, 0.0, 0.0, 0.0};
    

    //std::vector<Waypoint> waypoints = { //! FOR LATER MULTIWAYPOINT TESTING
    //    {init_state.position, init_state.yaw, 0.0, 0.0},
    //    {init_state.position + Eigen::Vector3d(1.0, 0.0, 0.0), init_state.yaw, 0.0, 0.0},
    //    {init_state.position + Eigen::Vector3d(1.0, 1.0, 0.0), init_state.yaw + 1.57, 0.0, 0.0},
    //    {init_state.position + Eigen::Vector3d(0.0, 1.0, 0.0), init_state.yaw + 3.14, 0.0, 0.0},
    //    {init_state.position, init_state.yaw, 0.0, 0.0}
    //};
//
    planner.generate_trajectory(init_state.position, target_position, init_state.orientation, target_quaternion, init_state.velocity, init_state.acceleration);

    double T = planner.get_total_time();
    const auto* segments = planner.get_segments();   // note: pointer version

    std::cout << "total_time = " << T << "\n";
    for (double t = 0.0; t <= T + 1e-6; t += T / 10.0) {
        double x = evalPoly(segments[0].coefficient, t);
        double y = evalPoly(segments[1].coefficient, t);
        double z = evalPoly(segments[2].coefficient, t);
        std::cout << "t=" << t << "  pos=(" << x << ", " << y << ", " << z << ")\n";
    }

    activate_trajectory(T);

}




int main(int argc, char * argv[])
{
    PathPlanner planner; 
    StateManager state_manager;
    std::vector<double> Test_raw = PathPlanner().generate_polynomium_coefficients(0, 3, 0, 0, 6);
    for (double val : Test_raw) std::cout << val << " ";
    std::cout << std::endl;

    TestPlan();   // <-- add this

    return 0;
}