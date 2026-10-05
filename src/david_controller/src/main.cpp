#include <chrono>
#include <functional>
#include <memory>
#include <cstring>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <cstdio>

#include <array>
#include <mutex>
#include <string>
#include <atomic>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"  // all headers (those ending with .hpp) must be specififed in the package.xml file. 
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


using namespace std::chrono_literals;

#include "path_planner.h"
#include "state_manager.h"
#include "kinematics.h"
#include "transformations.h"
#include "controller.h"




class ControlNode : public rclcpp::Node{ //our control_node is derived from the "rclcpp::Node" class that is the base class for all ROS2 nodes.



    public:
        ControlNode() : Node("control_node") //custructor of node object??
        {
        std::cout << "control_node initialized" << std::endl; //! For testing the control loop
        state_manager.set_control_mode(ControlMode::DEADMAN);
        
        //StateManagers

        //Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>( 
        "/servo_node/delta_twist_cmds", 10); //double check the topic name
        vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/forward_velocity_controller/commands", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            setJointPositions(*msg);
        });

        //Timer for publishing cmd_vel messages
        //timer_ = this->create_wall_timer(
        //    std::chrono::milliseconds(50), // 20 Hz
        //    std::bind(&ControlNode::vel_cmd_callback, this)); // 
        start_time_ = this->now();
        start_control_loop();

        }
        ~ControlNode() { stop_keyboard_control(); }

    private: // here goes callback functions and member variables (variables that should only belong to this class and not be accessed by other classes)
        
        StateManager state_manager;
        Transformations transformations;
        ForwardKinematics forward_kinematics;
        InverseKinematics inverse_kinematics;
        PathPlanner path_planner;
        Controller controller;
        DHParam dh_;
        TempJoint joints_;

        //msg.header.stamp = this->get_clock()->now(); //msg message gets the current time from the ros2 clock and assigns it to the header.stamp variable
        //msg.header.frame_id = "base_link"; // assigns the frame_id which the velocity message regards to. 
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time start_time_;//! Test
        rclcpp::TimerBase::SharedPtr control_timer;
        rclcpp::Time last_time;
        std::atomic<bool> joint_positions_received_{false}; //To wait until joint angles arrive

        
        std::thread keyboard_thread;

        std::atomic<bool> key_up{false}, key_down{false}, key_left{false},
                  key_right{false}, key_z{false}, key_x{false};
        std::atomic<bool> keyboard_running{false};

        Pose target_pose_;
        bool target_initialized_ = false;
        static constexpr double STEP = 0.005;

        std::array<double, 6> joint_positions_{}; //! TEMP
        std::mutex joint_positions_mutex_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        static constexpr std::array<const char*, 6> JOINT_NAMES = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}; //! Test

        void vel_cmd_callback(geometry_msgs::msg::TwistStamped cmd_vel) // as of now, this function creates twist.stamped messages containing a point in time, frame-id, and a linear velocity in the x direction. The message is then published to the topic "/servo_node/delta_twist_cmds"
        {// creates blank twist stamped variable named cmd_vel
            cmd_vel.header.stamp = this->get_clock()->now(); //cmd_vel message gets the current time from this node's clock and assigns it to the header.stamp variable of the cmd_vel message. T.
            cmd_vel.header.frame_id = "base_link"; // cmd_vel message gets the frame_id set to "base_link" 
            cmd_vel_pub_->publish(cmd_vel); //publishes the message to the topic
        }
        void setJointPositions(const sensor_msgs::msg::JointState& msg) {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
                for (size_t j = 0; j < JOINT_NAMES.size(); ++j) {
                    if (msg.name[i] == JOINT_NAMES[j]) {
                        joint_positions_[j] = msg.position[i];
                        break;
                    }
                }
            }

            joint_positions_received_ = true;
        }

        std::array<double, 6> getJointPositions() {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            return joint_positions_;
        }

        void activate_trajectory(float trajectory_duration){
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

        static std::string fmt(const std::array<double, 6>& a) //Help to see what we publish
        {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "[%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f]",
                          a[0], a[1], a[2], a[3], a[4], a[5]);
            return buf;
        }

        void start_keyboard_control()
        {
            keyboard_running = true;
            keyboard_thread = std::thread([this](){
                int tty_fd = open("/dev/tty", O_RDWR);
                if (tty_fd < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Cannot open keyboard input");
                    keyboard_running = false;
                    return;
                }
                struct termios oldt, newt;
                tcgetattr(tty_fd, &oldt);
                newt = oldt;
                newt.c_lflag &= ~(ICANON | ECHO);
                tcsetattr(tty_fd, TCSANOW, &newt);
                fcntl(tty_fd, F_SETFL, O_NONBLOCK);

                RCLCPP_INFO(this->get_logger(), "Keyboard control active: W/S up/down for Y-axis, A/D for X-axis and z x for Z-axis. Q to quit");
                
                while (keyboard_running)
                {
                    key_up = key_down = key_left = key_right = key_z = key_x = false;
                    char c;
                    while (read(tty_fd, &c, 1) > 0)
                    {
                        switch (tolower(c))
                        {
                            case 'w': key_up    = true; break;
                            case 's': key_down  = true; break;
                            case 'a': key_left  = true; break;
                            case 'd': key_right = true; break;
                            case 'z': key_x = true; break; 
                            case 'x': key_z = true; break; 
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                 }
                tcsetattr(tty_fd, TCSANOW, &oldt);
                close(tty_fd);
            });
        }

        void stop_keyboard_control()
        {
            keyboard_running = false;
            if (keyboard_thread.joinable())
                keyboard_thread.join();
        }

        void start_control_loop()
        {
            ArmState arm_state = state_manager.get_arm_state();

                if (control_timer){
                    control_timer->cancel();
                    control_timer.reset();
                }
                control_timer = this->create_wall_timer(
                    std::chrono::milliseconds(50),  //20 Hz
                    [this]() {control_loop();}
                );
            
        }

        void control_loop()
        {
            auto now = get_clock()->now();
//
            //if (last_time.nanoseconds() == 0)
            //{
            //    last_time = now;
            //}
            const double time = (this->now() - start_time_).seconds();
            geometry_msgs::msg::TwistStamped msg;
            Pose end_point;
            TempJoint tempth;

       
            
            //target.x = -0.5;
            //target.y = -0.2;
            //target.z =  0.4;
            //target.roll  = M_PI;   // tool pointing straight down
            //target.pitch = 0.0;
            //target.yaw   = 0.0;

            //std::optional<TempJoint> result = inverse_kinematics.inverse_kinematics(dh_, target, joints_);
            switch (state_manager.get_control_mode())
            {
                case ControlMode::SAFETY:
                    // ...
                    vel_cmd_callback(msg);
                    break;
                case ControlMode::STARTUP:
                    // ...
                    vel_cmd_callback(msg);
                    break;
                case ControlMode::OPERATION:
                    
                    {
                        //std_msgs::msg::Float64MultiArray cmd;
                        //cmd.data = {0.3 * std::sin(2.0 * M_PI * 0.25 * t), 0.0, 0.0, 0.0, 0.0, 0.0};  // elbow only
                        //vel_pub_->publish(cmd);
                        //th.theta[0] = 0.3;
                        //th.theta[1] = -1.2;
                        //th.theta[2] = 1.5;
                        //end_point = forward_kinematics.forward_kinematics(dh, th);
//
                        //std::cout << "x: " << end_point.x << "\n"
                        //     << "y: " << end_point.y << "\n"
                        //     << "z: " << end_point.z << "\n";

                        //TrajectoryPoint pt = path_planner.get_trajectory_point(time);
                        //EulerAngles rpy = transformations.quaternionToEuler(pt.orientation);
                        //std::cout << "Roll: " << rpy.roll << "\n"
                        //        "Pitch: " << rpy.pitch << "\n"
                        //        "Yaw: " << rpy.yaw << "\n";
//
                        ////// ...
                        //if (result) {
                        //    const TempJoint& goal = *result;
                        //    RCLCPP_INFO(get_logger(), "Joints: %.3f %.3f %.3f %.3f %.3f %.3f",
                        //                goal.theta[0], goal.theta[1], goal.theta[2],
                        //                goal.theta[3], goal.theta[4], goal.theta[5]);
                        //    tempth.theta[0] = goal.theta[0];
                        //    tempth.theta[1] = goal.theta[1];
                        //    tempth.theta[2] = goal.theta[2];
                        //    tempth.theta[3] = goal.theta[3];
                        //    tempth.theta[4] = goal.theta[4];
                        //    tempth.theta[5] = goal.theta[5];
                        //    end_point = forward_kinematics.forward_kinematics(dh_, tempth); 
                        //    std::cout << "x: " << end_point.x << "\n"
                        //        << "y: " << end_point.y << "\n"
                        //        << "z: " << end_point.z << "\n";
                        //    // send goal.theta to the robot here
                        //} else {
                        //    RCLCPP_WARN(get_logger(), "Target pose is out of reach");
                        //}
                        break;
                    }
                case ControlMode::DEADMAN: //! Making temp manual
                    // ...
                    if (!joint_positions_received_)
                        break;
                    if (!keyboard_running)
                        start_keyboard_control();

                    auto q = getJointPositions();
                    TempJoint current{q};
                    if (!target_initialized_)
                    {
                        target_pose_ = forward_kinematics.forward_kinematics(dh_, current); 
                        target_initialized_ = true;
                    }
                    Pose previous = target_pose_;
                    

                    if (key_up)    target_pose_.y += STEP;
                    if (key_down)  target_pose_.y -= STEP;
                    if (key_left)  target_pose_.x += STEP;
                    if (key_right) target_pose_.x -= STEP;
                    if (key_x)     target_pose_.z += STEP;
                    if (key_z)     target_pose_.z -= STEP;


                    std::optional<TempJoint> ik = inverse_kinematics.inverse_kinematics(dh_, target_pose_, current);

                    if (!ik)
                    {
                        target_pose_ = previous;  // unreachable, so undo the step
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK failed, target unreachable");
                        break;
                    }
                    std::array<double, 6> q_target = ik->theta;
                    auto vel = controller.p_controller(q, q_target);
                    bool any_key = key_up || key_down || key_left || key_right || key_z || key_x;
                    std::array<double, 6> err{};
                    for (std::size_t i = 0; i < 6; ++i)
                        err[i] = q_target[i] - q[i];

                    if (any_key)
                    {
                        RCLCPP_INFO(get_logger(), "----");
                        RCLCPP_INFO(get_logger(), "keys  up=%d down=%d left=%d right=%d z=%d x=%d",
                                    (bool)key_up, (bool)key_down, (bool)key_left, (bool)key_right, (bool)key_z, (bool)key_x);
                        RCLCPP_INFO(get_logger(), "pose  x=%.4f y=%.4f z=%.4f",
                                    target_pose_.x, target_pose_.y, target_pose_.z);
                        RCLCPP_INFO(get_logger(), "q     %s", fmt(q).c_str());
                        RCLCPP_INFO(get_logger(), "ik    %s", fmt(q_target).c_str());
                        RCLCPP_INFO(get_logger(), "err   %s", fmt(err).c_str());
                        RCLCPP_INFO(get_logger(), "vel   %s", fmt(vel).c_str());
                    }
                    else
                    {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                             "idle  err %s", fmt(err).c_str());
                    }
                    std_msgs::msg::Float64MultiArray cmd;
                    cmd.data.assign(vel.begin(), vel.end());
                    vel_pub_->publish(cmd);

                    break;
            }
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

    rclcpp::init(argc, argv); //initializing ros2 for the program
    rclcpp::spin(std::make_shared<ControlNode>()); //keeps node active 
    rclcpp::shutdown(); //shuts down the node when the program is terminated

    return 0;
}