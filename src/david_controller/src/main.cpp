#include <chrono>
#include <functional>
#include <memory>
#include <cstring>
#include <iostream>

#include "rclcpp/rclcpp.hpp"  // all headers (those ending with .hpp) must be specififed in the package.xml file. 
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "state_manager.h"
#include "path_planner.h"
#include "transformations.h"

using namespace std::chrono_literals;




class control_node : public rclcpp::Node{ //our control_node is derived from the "rclcpp::Node" class that is the base class for all ROS2 nodes.



    public:
        control_node() : Node("control_node") //custructor of node object??
        {
        std::cout << "control_node initialized" << std::endl;

        //StateManagers



        //Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>( //message name is msg (need to define in a msg folder?)
        "/servo_node/delta_twist_cmds", 10); //double check the topic name


        }

    private: // here goes callback functions and member variables (variables that should only belong to this class and not be accessed by other classes)
        
        geometry_msgs::msg::TwistStamped msg; // creates twist stamped variable named msg:)
        //msg.header.stamp = this->get_clock()->now(); //msg message gets the current time from the ros2 clock and assigns it to the header.stamp variable
        //msg.header.frame_id = "base_link"; // assigns the frame_id which the velocity message regards to. 
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

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


        
};

//TrajectoryInitState init_state = {
//        .position = {0.0, 0.0, 0.0},
//        .position_target_prev = {0.0, 0.0, 0.0},
//        .orientation = {1.0, 0.0, 0.0, 0.0},cd
//        .velocity = {0.0, 0.0, 0.0},
//        .acceleration = {0.0, 0.0, 0.0},
//        .yaw = 0.0
//    };




int main(int argc, char * argv[])  // main function: should contain as little code as possible, just to call the other functions
{
    //const char p = 'P';
    //test(p);

    rclcpp::init(argc, argv); //initializing ros2 for the program
    rclcpp::spin(std::make_shared<control_node>()); //keeps node active 
    rclcpp::shutdown(); //shuts down the node when the program is terminated

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped> ::SharedPtr publisher_; 
    

    

    geometry_msgs::msg::TwistStamped msg; //Remember, msg have to exist in the different instances. This one is not the same you made inside the class

    publisher_->publish(msg);


    std::cout << "System terminated" <<std::endl;
    return 0;
}