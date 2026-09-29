#include <chrono>
#include <functional>
#include <memory>
#include <cstring>
#include <iostream>

#include "rclcpp/rclcpp.hpp"  // all headers (those ending with .hpp) must be specififed in the package.xml file. 
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

#include "path_planner.h"
#include "state_manager.h"



class ControlNode : public rclcpp::Node{ //our control_node is derived from the "rclcpp::Node" class that is the base class for all ROS2 nodes.



    public:
        ControlNode() : Node("control_node") //custructor of node object??
        {
        std::cout << "ControlNode initialized" << std::endl;

        //StateManagers

        //Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>( 
        "/servo_node/delta_twist_cmds", 10); //double check the topic name

        //Timer for publishing cmd_vel messages
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&ControlNode::vel_cmd_callback, this)); // 
        }
        
        
    private: // here goes callback functions and member variables (variables that should only belong to this class and not be accessed by other classes)
        


        //callback function
        void vel_cmd_callback() // as of now, this function creates twist.stamped messages containing a point in time, frame-id, and a linear velocity in the x direction. The message is then published to the topic "/servo_node/delta_twist_cmds"
            {
            geometry_msgs::msg::TwistStamped cmd_vel; // creates blank twist stamped variable named cmd_vel
            
            cmd_vel.header.stamp = this->get_clock()->now(); //cmd_vel message gets the current time from this node's clock and assigns it to the header.stamp variable of the cmd_vel message. T.
            cmd_vel.header.frame_id = "base_link"; // cmd_vel message gets the frame_id set to "base_link"
            
            cmd_vel.twist.linear.x = 0.05; //example of an actual twist.stamped message. other variables stay a 0 unless other is specified. 

            //publisher_->publish(cmd_vel); //publishes the message to the topic
            cmd_vel_pub_->publish(cmd_vel); //publishes the message to the topic
            }

        //member variables
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped> ::SharedPtr cmd_vel_pub_; //publisher variable used in the callback function and the constructor;
        rclcpp::TimerBase::SharedPtr timer_;



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



//#include "path_planner.h"


int main(int argc, char * argv[])  // main function: should contain as little code as possible, just to call the other functions
{
    //const char p = 'P';
    //test(p);

    rclcpp::init(argc, argv); //initializing ros2 for the program
    rclcpp::spin(std::make_shared<ControlNode>()); //keeps node active 
    rclcpp::shutdown(); //shuts down the node when the program is terminated

    std::cout << "System terminated" <<std::endl;
    return 0;
}