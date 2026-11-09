#include <chrono>
#include <functional>
#include <memory>
#include <cstring>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <cstdio>
#include <optional> 

#include <array>
#include <algorithm>
#include <mutex>
#include <string>
#include <string_view>
#include <atomic>
#include <thread>
#include <unistd.h>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp" 
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp> 
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interface/msg/vr_input.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

#include "path_planner.h"
#include "state_manager.h"
#include "kinematics.h"
#include "transformations.h"
#include "controller.h"


struct Arm {
    std::string name;          // "left" or "right"
    std::string base_frame;    // "left_base" (UR base, not base_link)
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub;

    Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_base_world = Eigen::Matrix4d::Identity();
    bool base_known = false;

    std::optional<Eigen::Matrix4d> target_world;   

    PathPlanner planner;
    StateManager state_manager;
    TrajectoryMode trajectory_mode = TrajectoryMode::INACTIVE;
    rclcpp::Time trajectory_start{0, 0, RCL_ROS_TIME};
};

struct FixedPoses { 
    Eigen::Vector3d pos; 
    Eigen::Quaterniond quat;
    std::string name;
 };



struct JogKeys {
    std::atomic<bool> up{false}, down{false}, left{false}, right{false}, z_up{false}, z_down{false}, home{false};
    void clear() { up = down = left = right = z_up = z_down = home = false; }
};


class ControlNode : public rclcpp::Node{ //our control_node is derived from the "rclcpp::Node" class that is the base class for all ROS2 nodes.



    public:
        ControlNode() : Node("control_node") //custructor of node object??
        {
        std::cout << "control_node initialized" << std::endl;
        std::cout <<  R"(
                    ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣤⣴⣶⣶⣿⠿⠿⠿⢿⣶⣶⣤⣀⣀⣀⣠⣤⣤⣦⠀⠀
            ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⣿⡿⠿⠛⠛⠉⠉⠀⠀⠀⠀⠀⠈⢿⡏⠉⢻⣿⣿⣿⣿⣿⡆⠀
            ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣼⠋⠀⠀⠀⣴⣶⡄⠀⠀⢰⣿⠀⠀⠀⠘⣷⡀⠀⢹⣿⣿⣿⣿⣿⠀
            ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣾⣇⣀⣤⣤⣤⣾⣿⣶⣶⣶⣿⣿⣿⣿⣿⣿⣿⣷⣾⣿⣿⣿⣿⣿⣿⡆ 
            ⠀⠀⣠⣴⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠿⠛⠉⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠃
            ⠀⣰⠋⠛⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣏⣁⣀⣠⣤⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀
            ⣰⣷⣦⣤⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠟⠛⣿⣿⣿⣿⣿⣿⣿⠁⠈⠙⢿⣿⣿⣿⣿⠀⣿⠀
            ⣿⣿⣿⣿⣿⣷⡀⠀⠈⠉⠉⠉⠉⠁⠀⠀⠀⠀⢀⣼⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠘⣿⣿⣿⣿⠀⣿⠀
            ⣿⣿⣿⣿⣿⣿⣷⣤⣀⣀⣀⣀⣀⣀⣀⣠⣤⣾⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀⢀⣿⣿⣿⣿⣀⣿⠀
            ⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⢀⣼⣿⠛⠛⠛⠛⠃⠀
            ⠀⠈⠙⠻⢿⣿⣿⣿⠿⠟⠛⠛⠛⠛⠛⠉⠉⠉⠉⠉⠀⠈⠻⣿⣿⣿⣷⣶⣶⣿⡿⠁⠀⠀⠀⠀⠀⠀
            ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀
        )" << std::endl;

        //Declare initial parameter
        this->declare_parameter<double>("p_controller.kp", 1.0);

        p_gains.Kp = this->get_parameter("p_controller.kp").as_double();

        std::cout << "Controller Gains = " << p_gains.Kp << std::endl;

        controller.set_gains(
            p_gains
        );
        state_manager.set_control_mode(ControlMode::TEST);
        
        //StateManagers

        //Publishers
        left_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/left_forward_velocity_controller/commands", 10);
        right_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/right_forward_velocity_controller/commands", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        left_arm_.name = "left";
        left_arm_.base_frame = "left_base";
        left_arm_.vel_pub = left_vel_pub_;

        right_arm_.name = "right";
        right_arm_.base_frame = "right_base";
        right_arm_.vel_pub = right_vel_pub_;

        //Subscribers
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            joint_callback(*msg);
        });

        vr_sub_ = this->create_subscription<interface::msg::VRInput>(
        "/vr/input", 10,
        [this](const interface::msg::VRInput::SharedPtr msg)
        {
            vr_callback(*msg);
            vr_grip_callback(*msg);
        });



        start_time_ = this->now();
        start_control_loop();
        rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
            [this]() { publishZeroVelocity(); });
        }
        ~ControlNode() { stop_keyboard_control(); }
        

    private: // here goes callback functions and member variables (variables that should only belong to this class and not be accessed by other classes)
        
        //-------------Create object of classes------
        StateManager state_manager;
        Transformations transformations;
        ForwardKinematics forward_kinematics;
        InverseKinematics inverse_kinematics;
        PathPlanner path_planner;
        Controller controller;
        SingularityPrevention singularity_prevention; // new

        Arm left_arm_;
        Arm right_arm_;

        const std::map<std::string, FixedPoses> fixed_target_poses_ = {
            {"home_left", {{0.458, 0.207, 0.921}, Eigen::Quaterniond(0, 1, 0, 0), "home_left"}},
            {"home_right",  {{0.415, 0.377, 0.949}, Eigen::Quaterniond(0, 1, 0, 0), "home_right"}}
        };
        //-----------Variables-----------------
        PGains p_gains;
        DHParam dh_;
        TempJoint joints_;
        Pose target_pose_;
        SingularityInfo singularity_info_; // new 
        double scaling_factor = 1.0; // new

        //-----------Publishers and subcribers-----------
        //rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
        //rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_; //Outdated
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_vel_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Subscription<interface::msg::VRInput>::SharedPtr vr_sub_;

        //-----------Timers---------------------
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time start_time_;//! Test
        rclcpp::TimerBase::SharedPtr control_timer;
        rclcpp::Time last_time;


        //------Thread safety for when running the keyboard command-----------
        std::thread keyboard_thread;
        JogKeys left_keys_;
        JogKeys right_keys_;
        std::atomic<bool> keyboard_running{false};
        std::mutex main_joint_positions_mutex_;
        std::atomic<bool> joint_positions_received_{false}; //To wait until joint angles arrive
        rclcpp::Time last_joint_update_{0, 0, RCL_ROS_TIME}; //for safety
        bool target_initialized_ = false;
        static constexpr double STEP = 0.005;


        //--------------------CallBacks---------------
        void joint_callback(const sensor_msgs::msg::JointState& msg)
        {
            sensor_msgs::msg::JointState left_msg;
            sensor_msgs::msg::JointState right_msg;
            left_msg.header = msg.header;
            right_msg.header = msg.header;

            constexpr std::string_view LEFT_PREFIX = "left_";
            constexpr std::string_view RIGHT_PREFIX = "right_";

            const std::size_t value_count = std::min(msg.name.size(), msg.position.size());
            for (std::size_t i = 0; i < value_count; ++i) {
                const std::string& name = msg.name[i];
                sensor_msgs::msg::JointState* arm_msg = nullptr;
                std::string_view prefix;

                if (name.starts_with(LEFT_PREFIX)) {
                    arm_msg = &left_msg;
                    prefix = LEFT_PREFIX;
                } else if (name.starts_with(RIGHT_PREFIX)) {
                    arm_msg = &right_msg;
                    prefix = RIGHT_PREFIX;
                } else {
                    continue;
                }

                arm_msg->name.emplace_back(name.substr(prefix.size()));
                arm_msg->position.push_back(msg.position[i]);
            }

            constexpr std::size_t EXPECTED_JOINT_COUNT = 6;
            if (left_msg.name.size() != EXPECTED_JOINT_COUNT ||
                right_msg.name.size() != EXPECTED_JOINT_COUNT) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Expected 6 joints per arm, received left=%zu right=%zu",
                    left_msg.name.size(), right_msg.name.size());
                joint_positions_received_ = false;
                return;
            }

            state_manager.setLeftJointPositions(left_msg);
            state_manager.setRightJointPositions(right_msg);
            state_manager.setJointPositions(msg);
            joint_positions_received_ = true;
            last_joint_update_ = this->now();

        }
        void vr_grip_callback(const interface::msg::VRInput& msg)
        {
            state_manager.setLeftGrip(msg.left.dms);
            state_manager.setRightGrip(msg.right.dms);
        }

        void vr_callback(const interface::msg::VRInput& msg)
        {
            if (msg.left.pose_available) {
                geometry_msgs::msg::PoseStamped left_target;
                left_target.header = msg.header;
                left_target.pose = msg.left.pose;
                state_manager.setLeftTarget(left_target);
            }

            if (msg.right.pose_available) {
                geometry_msgs::msg::PoseStamped right_target;
                right_target.header = msg.header;
                right_target.pose = msg.right.pose;
                state_manager.setRightTarget(right_target);
            }

            if (msg.head.pose_available) {
                geometry_msgs::msg::PoseStamped vr_head_state;
                vr_head_state.header = msg.header;
                vr_head_state.pose = msg.head.pose;
                state_manager.setVRHeadState(vr_head_state);
            }
        }
        bool lookupBase(Arm& arm)
        {
            if (arm.base_known) return true;
            try {
                auto tf = tf_buffer_->lookupTransform("world", arm.base_frame, tf2::TimePointZero);
                arm.T_world_base = tf2::transformToEigen(tf).matrix();
                arm.T_base_world = arm.T_world_base.inverse();
                arm.base_known = true;
            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s: waiting for base TF: %s",
                                     arm.name.c_str(), e.what());
            }
            return arm.base_known;
        }
        void publishZero(Arm& arm)
        {
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data.assign(6, 0.0);
            arm.vel_pub->publish(cmd);
        }

        void publishZeroVelocity() //!!! FIX
        {
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data.assign(6, 0.0);
            left_vel_pub_->publish(cmd);
            right_vel_pub_->publish(cmd);
        }
        static std::string fmt(const std::array<double, 6>& a) //Help to see what we publish
        {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "[%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f]",
                          a[0], a[1], a[2], a[3], a[4], a[5]);
            return buf;
        }
        bool trackWorldTarget(Arm& arm, const TempJoint& q)
        {
            if (!arm.target_world) { publishZero(arm); return true; }
        
            const Eigen::Matrix4d target_base = arm.T_base_world * *arm.target_world;
            auto ik = inverse_kinematics.inverse_kinematics(dh_, target_base, q);
            if (!ik) {
                publishZero(arm);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s: IK failed, target unreachable",
                                     arm.name.c_str());
                return false;
            }
        
            auto vel = controller.p_controller(q.theta, ik->theta);
            auto J = singularity_prevention.jacobian_computation(dh_, q);
            singularity_info_ = singularity_prevention.compute_singularity_info(J);
            auto vel_scaled = singularity_prevention.velocity_scaler(singularity_info_, vel);
        
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data.assign(vel_scaled.begin(), vel_scaled.end());
            arm.vel_pub->publish(cmd);
            return true;
        }
        //------------------------Trajectory functions------------------------------------------

        void activate_trajectory(Arm& arm, const TempJoint& q, const FixedPoses& target)
        {
            Eigen::Matrix4d start_pose = arm.target_world ? *arm.target_world
                : arm.T_world_base * forward_kinematics.forward_kinematics_matrix(dh_, q);

            arm.planner.generate_trajectory(
                start_pose.block<3,1>(0,3), target.pos,
                Eigen::Quaterniond(start_pose.block<3,3>(0,0)), target.quat,
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
                
            RCLCPP_INFO(get_logger(), "%s: moving to '%s' (%.2f, %.2f, %.2f), duration %.2f s",
                arm.name.c_str(), target.name.c_str(),
                target.pos.x(), target.pos.y(), target.pos.z(),
                arm.planner.get_total_time());

            arm.trajectory_start = this->now();
            arm.trajectory_mode = TrajectoryMode::ACTIVE;
        }
        //---------------------Keyboard functions---------------------------------------------
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

                RCLCPP_INFO(this->get_logger(), "Keyboard control active: W/S up/down for Y-axis, A/D for X-axis and z x for Z-axis. H to home");
                
                while (keyboard_running)
                {
                    left_keys_.clear();
                    right_keys_.clear();
                    char c;
                    while (read(tty_fd, &c, 1) > 0)
                    {
                        switch (tolower(c))
                        {
                            // left arm
                            case 'w': left_keys_.up     = true; break;
                            case 's': left_keys_.down   = true; break;
                            case 'a': left_keys_.left   = true; break;
                            case 'd': left_keys_.right  = true; break;
                            case 'z': left_keys_.z_up   = true; break;
                            case 'x': left_keys_.z_down = true; break;
                            // right arm
                            case 'i': right_keys_.up     = true; break;
                            case 'k': right_keys_.down   = true; break;
                            case 'j': right_keys_.left   = true; break;
                            case 'l': right_keys_.right  = true; break;
                            case 'n': right_keys_.z_up   = true; break;
                            case 'm': right_keys_.z_down = true; break;

                            case 'h': left_keys_.home = true; break;
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
        void keyboardJog(Arm& arm, const TempJoint& q, const JogKeys& keys)
        {
            if (!arm.target_world)
                arm.target_world = arm.T_world_base * forward_kinematics.forward_kinematics_matrix(dh_, q);
        
            Eigen::Matrix4d& T = *arm.target_world;
            if (keys.up)     T(1, 3) += STEP;
            if (keys.down)   T(1, 3) -= STEP;
            if (keys.left)   T(0, 3) += STEP;
            if (keys.right)  T(0, 3) -= STEP;
            if (keys.z_up)   T(2, 3) += STEP;
            if (keys.z_down) T(2, 3) -= STEP;
            
        }
        //------------------------------------Control loop----------------------------

        void start_control_loop()
        {

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


            TempJoint tempth;         
            

            switch (state_manager.get_control_mode())
            {
                case ControlMode::SAFETY:
                    // ...
                    break;
                case ControlMode::STARTUP:
                    {
                        bool fresh;
                        {
                        std::lock_guard<std::mutex> lock(main_joint_positions_mutex_);
                        fresh = joint_positions_received_ &&
                                (this->now() - last_joint_update_).seconds() <= 0.1;
                        }
                        if (!fresh) { publishZeroVelocity(); break; }

                        const bool left_ok  = lookupBase(left_arm_);    
                        const bool right_ok = lookupBase(right_arm_);
                        
                        if (!left_ok || !right_ok) { publishZeroVelocity(); break; }

                        TempJoint left_q{state_manager.getLeftJointPositions()};
                        TempJoint right_q{state_manager.getRightJointPositions()};

                        if(left_arm_.trajectory_mode == TrajectoryMode::INACTIVE || right_arm_.trajectory_mode == TrajectoryMode::INACTIVE){

                            activate_trajectory(left_arm_, left_q, fixed_target_poses_.at("home_left"));
                            activate_trajectory(right_arm_, right_q, fixed_target_poses_.at("home_right"));

                            
                        }
                        else if (left_arm_.trajectory_mode == TrajectoryMode::ACTIVE || right_arm_.trajectory_mode == TrajectoryMode::ACTIVE){

                            const auto previous_left  = left_arm_.target_world;
                            const auto previous_right = right_arm_.target_world;

                            const double T_total_left = left_arm_.planner.get_total_time();
                            const double T_total_right = right_arm_.planner.get_total_time();
                            const double t_left = std::min((this->now() - left_arm_.trajectory_start).seconds(), T_total_left);
                            const double t_right = std::min((this->now() - right_arm_.trajectory_start).seconds(), T_total_right);

                            TrajectoryPoint point_left = left_arm_.planner.get_trajectory_point(t_left);
                            TrajectoryPoint point_right = right_arm_.planner.get_trajectory_point(t_right);
                            Eigen::Matrix4d T_left = Eigen::Matrix4d::Identity();
                            Eigen::Matrix4d T_right = Eigen::Matrix4d::Identity();
                            T_left.block<3,3>(0,0) = point_left.orientation.toRotationMatrix();
                            T_right.block<3,3>(0,0) = point_right.orientation.toRotationMatrix();
                            T_left.block<3,1>(0,3) = point_left.position;
                            T_right.block<3,1>(0,3) = point_right.position;
                            left_arm_.target_world = T_left;
                            right_arm_.target_world = T_right;

                            

                            if (!trackWorldTarget(left_arm_, left_q))
                            {
                                left_arm_.target_world = previous_left; 
                                RCLCPP_ERROR(get_logger(), "Move to '%s' aborted: IK failed",
                                fixed_target_poses_.at("home_left").name.c_str());
                                left_arm_.trajectory_mode = TrajectoryMode::INACTIVE;  
                                state_manager.set_control_mode(ControlMode::TEST);      
                                break;
                            }
                            if (!trackWorldTarget(right_arm_, right_q)) 
                            {
                                right_arm_.target_world = previous_right; 
                                RCLCPP_ERROR(get_logger(), "Move to '%s' aborted: IK failed",
                                fixed_target_poses_.at("home_right").name.c_str());
                                right_arm_.trajectory_mode = TrajectoryMode::INACTIVE;
                                state_manager.set_control_mode(ControlMode::TEST);      
                                break;
                            }
                            
                            //trackWorldTarget(right_arm_, right_q);

                            if(t_left >= T_total_left) left_arm_.trajectory_mode = TrajectoryMode::COMPLETED;
                            if(t_right >= T_total_right) right_arm_.trajectory_mode = TrajectoryMode::COMPLETED;

                        }
                        else if (left_arm_.trajectory_mode == TrajectoryMode::COMPLETED) {
                            left_arm_.trajectory_mode = TrajectoryMode::INACTIVE;
                            right_arm_.trajectory_mode = TrajectoryMode::INACTIVE;
                            state_manager.set_control_mode(ControlMode::TEST);
                        }
                    

                    break;
                    }
                    
                case ControlMode::OPERATION:
                    
                {   
                  break;
                }
                case ControlMode::DEADMAN:
                    // ...
                    {
                    
                    break;
                    }
                case ControlMode::TEST:
                {
                    bool fresh;
                    {
                        std::lock_guard<std::mutex> lock(main_joint_positions_mutex_);
                        fresh = joint_positions_received_ &&
                                (this->now() - last_joint_update_).seconds() <= 0.1;
                    }
                    if (!fresh) { publishZeroVelocity(); break; }
                    
                    const bool left_ok  = lookupBase(left_arm_);    
                    const bool right_ok = lookupBase(right_arm_);

                    if (!left_ok || !right_ok) { publishZeroVelocity(); break; }

                    TempJoint left_q{state_manager.getLeftJointPositions()};
                    TempJoint right_q{state_manager.getRightJointPositions()};

                    Eigen::Matrix4d current_pose =
                        right_arm_.T_world_base * forward_kinematics.forward_kinematics_matrix(dh_, right_q);

                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                        "Right current: x=%.3f y=%.3f z=%.3f",
                        current_pose(0, 3), current_pose(1, 3), current_pose(2, 3));
                    // --- Keyboard target source ---
                    if (!keyboard_running) start_keyboard_control();
                    const auto previous_left = left_arm_.target_world;
                    const auto previous_right = right_arm_.target_world;

                    keyboardJog(left_arm_, left_q, left_keys_);
                    if (left_keys_.home == true){
                        state_manager.set_control_mode(ControlMode::STARTUP);
                    }
                    keyboardJog(right_arm_, right_q, right_keys_);
                    // -------------------------------
                
                    if (!trackWorldTarget(left_arm_, left_q))
                        left_arm_.target_world = previous_left; 
                    
                    if (!trackWorldTarget(right_arm_, right_q))
                        right_arm_.target_world = previous_right;  
                    break;
                }
            }
        }


        
};




int main(int argc, char * argv[])  // main function: should contain as little code as possible, just to call the other functions
{

    rclcpp::init(argc, argv); //initializing ros2 for the program
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node); //keeps node active 
    rclcpp::shutdown(); //shuts down the node when the program is terminated

    return 0;
}
