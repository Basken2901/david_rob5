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
#include "ur_msgs/srv/set_io.hpp"

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
#include "arm_clearance_checker.h"

struct Clutch {
    bool engaged = false;
    Eigen::Matrix4d C0 = Eigen::Matrix4d::Identity();   // controller pose at latch
    Eigen::Matrix4d A0 = Eigen::Matrix4d::Identity();   // arm target at latch
};

struct Arm {
    std::string name;          // "left" or "right"
    std::string base_frame;    // "left_base" (UR base, not base_link)
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub;

    Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_base_world = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d controller_to_tool = Eigen::Matrix3d::Identity();   // controller axes -> tool axes
    bool base_known = false;

    std::optional<Eigen::Matrix4d> target_world;   
    std::optional<Eigen::Matrix4d> last_valid;  

    PathPlanner planner;
    StateManager state_manager;
    TrajectoryMode trajectory_mode = TrajectoryMode::INACTIVE;
    rclcpp::Time trajectory_start{0, 0, RCL_ROS_TIME};

    Clutch clutch;
    rclcpp::Time last_log{0, 0, RCL_ROS_TIME};
    

    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client;
    bool gripper_closed = false;
    bool prev_trigger = false;


    
};

struct FixedPoses { 
    Eigen::Vector3d pos; 
    Eigen::Quaterniond quat;
    std::string name;
 };



struct JogKeys {
    std::atomic<bool> up{false}, down{false}, left{false}, right{false}, z_up{false}, z_down{false}, home{false} , operate{false}, s_move{false};
    void clear() { up = down = left = right = z_up = z_down = home = operate = s_move = false; }
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
        this->declare_parameter<double>("dms_threshold", 0.5);
        this->declare_parameter<double>("relative_scaling_factor", 0.5);
        this->declare_parameter<double>("rotation_scaling_factor", 0.5); 
        this->declare_parameter<double>("tool_offset_z", 0.15);
        this->declare_parameter<std::string>("rotation_mapping", "tool");    // "tool" or "world" (old behaviour)
        this->declare_parameter<std::vector<double>>("left_controller_twist_axis",  {0.0, 0.0, -1.0});
        this->declare_parameter<std::vector<double>>("right_controller_twist_axis", {0.0, 0.0, -1.0});
        this->declare_parameter<double>("left_tool_twist_offset_deg",  0.0);
        this->declare_parameter<double>("right_tool_twist_offset_deg", 0.0);
        
        p_gains.Kp = this->get_parameter("p_controller.kp").as_double();
        dms_threshold = this->get_parameter("dms_threshold").as_double();
        relative_scaling_factor = this->get_parameter("relative_scaling_factor").as_double();
        rotation_scaling_factor = this->get_parameter("rotation_scaling_factor").as_double();
        dh_.d[5] = 0.0996 + this->get_parameter("tool_offset_z").as_double(); 
        rotation_mapping_ = this->get_parameter("rotation_mapping").as_string() == "world"
                                ? RotationMapping::WORLD : RotationMapping::TOOL;

        std::cout << "Controller Gains = " << p_gains.Kp << std::endl;
        std::cout << "Dead Man Switch Threshold = " << dms_threshold << std::endl;
        std::cout << "Teleoperation Relative Scaling = " << relative_scaling_factor << std::endl;
        std::cout << "Teleoperation Rotation Scaling = " << rotation_scaling_factor << std::endl;

        controller.set_gains(
            p_gains
        );
        state_manager.set_control_mode(ControlMode::TEST);
        
        //StateManagers
        if (declare_parameter<bool>("collision_check_enabled", true))
        {
            arm_clearance_checker_ = std::make_unique<ArmClearanceChecker>(*this);
        }

        //Publishers
        left_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/left_forward_velocity_controller/commands", 10);
        right_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/right_forward_velocity_controller/commands", 10);

        left_arm_.controller_to_tool  = make_controller_to_tool("left");
        right_arm_.controller_to_tool = make_controller_to_tool("right");

        left_arm_.io_client  = create_client<ur_msgs::srv::SetIO>("/left_io_and_status_controller/set_io");
        right_arm_.io_client = create_client<ur_msgs::srv::SetIO>("/right_io_and_status_controller/set_io");

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
        

    private: 
        
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

        bool button_a = false;
        bool button_y = false;
        bool button_b = false;
        bool right_trigger_ = false;
        bool left_trigger_ = false;

        static constexpr int8_t GRIPPER_CLOSE_PIN = 16;   
        static constexpr int8_t GRIPPER_OPEN_PIN  = 17; 

        const std::map<std::string, FixedPoses> fixed_target_poses_ = {
            {"home_left", {{0.384, 0.241, 1.088}, Eigen::Quaterniond(0, 1, 0, 0), "home_left"}},
            {"home_right",  {{0.256, -0.285, 1.22}, Eigen::Quaterniond(0, 1, 0, 0), "home_right"}}
        };

        Pose right_idle_;
        Pose left_idle_;
        Pose right_engaged_;
        Pose left_engaged_;
        rclcpp::Time left_vr_stamp_{0, 0, RCL_ROS_TIME};
        rclcpp::Time right_vr_stamp_{0, 0, RCL_ROS_TIME};
        //-----------Variables-----------------
        PGains p_gains;
        DHParam dh_;
        TempJoint joints_;
        Pose target_pose_;
        SingularityInfo singularity_info_; 
        double scaling_factor = 1.0; 
        double relative_scaling_factor;
        double rotation_scaling_factor;
        RotationMapping rotation_mapping_ = RotationMapping::TOOL;
        double dms_threshold;
        bool s_move_was_active_ = false;

        //-----------Publishers and subcribers-----------
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_vel_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Subscription<interface::msg::VRInput>::SharedPtr vr_sub_;
        std::unique_ptr<ArmClearanceChecker> arm_clearance_checker_;

        //-----------Timers---------------------
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time start_time_;
        rclcpp::TimerBase::SharedPtr control_timer;
    


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
            if (arm_clearance_checker_)
            {
                arm_clearance_checker_->check_clearance(msg);
                if (!arm_clearance_checker_->motion_allowed())
                    publishZeroVelocity();
            }

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

            button_a = msg.right.a;
            button_y = msg.left.y;

            right_trigger_ = msg.right.trigger;
            left_trigger_  = msg.left.trigger;

            
        }
        void vr_callback(const interface::msg::VRInput& msg)
        {
            if (msg.left.pose_available) {
                geometry_msgs::msg::PoseStamped left_target;
                left_target.header = msg.header;
                left_target.pose = msg.left.pose;
                left_vr_stamp_ = this->now();
                state_manager.setLeftTarget(left_target);
            }

            if (msg.right.pose_available) {
                geometry_msgs::msg::PoseStamped right_target;
                right_target.header = msg.header;
                right_target.pose = msg.right.pose;
                right_vr_stamp_ = this->now();
                state_manager.setRightTarget(right_target);
            }

            if (msg.head.pose_available) {
                geometry_msgs::msg::PoseStamped vr_head_state;
                vr_head_state.header = msg.header;
                vr_head_state.pose = msg.head.pose;
                state_manager.setVRHeadState(vr_head_state);
            }
        }
        bool transform_base(Arm& arm)
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
        Eigen::Matrix3d make_controller_to_tool(const std::string& side)
        {
            const auto p = this->get_parameter(side + "_controller_twist_axis").as_double_array();
            const double offset_deg = this->get_parameter(side + "_tool_twist_offset_deg").as_double();

            Eigen::Vector3d axis = -Eigen::Vector3d::UnitZ();
            if (p.size() == 3 && Eigen::Vector3d(p[0], p[1], p[2]).norm() > 1e-6) {
                axis = Eigen::Vector3d(p[0], p[1], p[2]).normalized();
            } else {
                RCLCPP_WARN(get_logger(), "%s_controller_twist_axis needs 3 numbers (not all zero), using [0, 0, -1]",
                            side.c_str());
            }
            // 1) turn the controller's twist axis onto tool z (joint 6)
            // 2) spin about tool z so the tilt directions line up as well
            const Eigen::Matrix3d align =
                Eigen::Quaterniond::FromTwoVectors(axis, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const Eigen::Matrix3d spin =
                Eigen::AngleAxisd(offset_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            return spin * align;
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

        void set_gripper(Arm& arm, bool close)
        {
            if (!arm.io_client->service_is_ready()) {
                RCLCPP_WARN(get_logger(), "%s gripper: set_io service not available", arm.name.c_str());
                return;
            }
            const int8_t pin_off = close ? GRIPPER_OPEN_PIN  : GRIPPER_CLOSE_PIN;
            const int8_t pin_on  = close ? GRIPPER_CLOSE_PIN : GRIPPER_OPEN_PIN;

            auto make_req = [](int8_t pin, float state) {
                auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
                req->fun   = ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
                req->pin   = pin;
                req->state = state;
                return req;
            };

            // Turn the other pin off first, and only then this one on, so both are never on together
            auto client = arm.io_client;
            auto logger = get_logger();
            const std::string name = arm.name;
            client->async_send_request(make_req(pin_off, 0.0f),
                [client, logger, name, pin_on, make_req](rclcpp::Client<ur_msgs::srv::SetIO>::SharedFuture f) {
                    if (!f.get()->success) {
                        RCLCPP_WARN(logger, "%s gripper: could not switch output off", name.c_str());
                        return;
                    }
                    client->async_send_request(make_req(pin_on, 1.0f));
                });
            RCLCPP_INFO(logger, "%s gripper: %s", name.c_str(), close ? "close" : "open");
        }
        //static std::string fmt(const std::array<double, 6>& a) //Help to see what we publish
        //{
        //    char buf[128];
        //    std::snprintf(buf, sizeof(buf), "[%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f]",
        //                  a[0], a[1], a[2], a[3], a[4], a[5]);
        //    return buf;
        //}
        void publishMotionVelocity(Arm& arm, std_msgs::msg::Float64MultiArray& cmd)
        {
            const double scale = state_manager.get_control_mode() != ControlMode::TEST && arm_clearance_checker_
                ? arm_clearance_checker_->velocity_scale() : 1.0;
            for (double& velocity : cmd.data)
                velocity = scale == 0.0 ? 0.0 : velocity * scale;
            arm.vel_pub->publish(cmd);
        }

        bool compute_velocity_publisher(Arm& arm, const TempJoint& q)
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
            arm.last_valid = arm.target_world;

            auto vel = controller.p_controller(q.theta, ik->theta);
            auto J = singularity_prevention.jacobian_computation(dh_, q);
            singularity_info_ = singularity_prevention.compute_singularity_info(J);
            auto vel_scaled = singularity_prevention.velocity_scaler(singularity_info_, vel);
            vel_scaled = controller.joint_limit_guard(q.theta, vel_scaled);

            std_msgs::msg::Float64MultiArray cmd;
            cmd.data.assign(vel_scaled.begin(), vel_scaled.end());
            publishMotionVelocity(arm, cmd);
            return true;
        }

        void compute_dls_publisher(Arm& arm, const TempJoint& q)
        {
            if (!arm.target_world) { publishZero(arm); return; }
        
            const Eigen::Matrix4d target_base = arm.T_base_world * *arm.target_world;   
            const Eigen::Matrix4d target_current = forward_kinematics.forward_kinematics_matrix(dh_, q);
            const Eigen::MatrixXd J     = singularity_prevention.jacobian_computation(dh_, q);
            const SingularityInfo info  = singularity_prevention.compute_singularity_info(J);
        
            int limiting = -1;
            const auto qdot = controller.joint_limit_guard(
                q.theta, controller.dls_controller(J, info.sigma_min, target_current, target_base), &limiting);
            if (limiting >= 0)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "%s: joint %d is at %.0f deg, close to its limit. Slowing/stopped: rotate back the other way",
                    arm.name.c_str(), limiting + 1, q.theta[limiting] * 180.0 / M_PI);
        
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data.assign(qdot.begin(), qdot.end());
            publishMotionVelocity(arm, cmd);
        }

        void track_target(Arm& arm, const TempJoint& q){
            if (compute_velocity_publisher(arm, q)) return;
                arm.target_world = arm.last_valid;
            if (arm.target_world) compute_velocity_publisher(arm,q);
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
                            case 'g': left_keys_.operate = true; break;
                            case 't': left_keys_.s_move = true; break;
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
        void small_movement(Arm& arm)
        {
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data = {0.0, 0.0, 0.3, 0.0, 0.3, 0.0};   // joints 3 and 5, same as your manual command
            publishMotionVelocity(arm, cmd);
        }
        
        void operate_arm(Arm& arm, const TempJoint& q,
                 const geometry_msgs::msg::PoseStamped& input_pose,
                 float dms, const rclcpp::Time& last_vr_update){
            if (!arm.target_world)
                arm.target_world = arm.T_world_base * forward_kinematics.forward_kinematics_matrix(dh_, q);

            const bool fresh  = (this->now() - last_vr_update).seconds() < 0.2;
            const bool active = fresh && dms > dms_threshold; 
            const Eigen::Matrix4d C = transformations.pose_to_matrix(input_pose.pose);

            if (active && !arm.clutch.engaged) {
                arm.clutch.engaged = true;
                arm.clutch.C0 = C;
                arm.clutch.A0 = *arm.target_world;
            } else if (!active) {
                arm.clutch.engaged = false;
                arm.target_world = arm.T_world_base *
                       forward_kinematics.forward_kinematics_matrix(dh_, q);
                publishZero(arm);
                return;

            }

            const auto previous = arm.target_world;
            if (arm.clutch.engaged)
                arm.target_world = controller.calculate_relative_target(arm.clutch.A0, arm.clutch.C0, C,
                                                        relative_scaling_factor,
                                                        rotation_scaling_factor,
                                                        Eigen::Matrix3d::Identity(),
                                                        arm.controller_to_tool,
                                                        rotation_mapping_);
            compute_dls_publisher(arm, q);
            
            if ((this->now() - arm.last_log).seconds() >= 0.5) {
                arm.last_log = this->now();
                
                const Eigen::Matrix3d R_hand = arm.clutch.C0.block<3,3>(0,0).transpose() * C.block<3,3>(0,0);
                const Eigen::AngleAxisd hand(R_hand);
                RCLCPP_INFO(get_logger(),
                    "%s: dms=%.2f fresh=%d engaged=%d target=(%.3f, %.3f, %.3f) | hand turned %.0f deg about controller axis [%.2f, %.2f, %.2f]",
                    arm.name.c_str(), dms, fresh, arm.clutch.engaged,
                    (*arm.target_world)(0,3), (*arm.target_world)(1,3), (*arm.target_world)(2,3),
                    hand.angle() * 180.0 / M_PI, hand.axis().x(), hand.axis().y(), hand.axis().z());
            }
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
            // Keyboard control may override clearance stops; automatic modes remain protected.
            if (state_manager.get_control_mode() != ControlMode::TEST &&
                arm_clearance_checker_ && !arm_clearance_checker_->motion_allowed())
            {
                publishZeroVelocity();
                // Discard old targets so they cannot pull the arms back after recovery.
                for (Arm* arm : {&left_arm_, &right_arm_})
                {
                    arm->target_world.reset();
                    arm->last_valid.reset();
                    arm->clutch.engaged = false;
                    arm->trajectory_mode = TrajectoryMode::INACTIVE;
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Both arms stopped: clearance <= 5 cm or clearance data unavailable/stale");
                return;
            }

            auto now = get_clock()->now();

            const double time = (this->now() - start_time_).seconds();       
            

            switch (state_manager.get_control_mode())
            {
                case ControlMode::SAFETY:
                    // ...
                    break;
                case ControlMode::STARTUP: //! Ugly code, needs fixing
                    {
                        bool fresh;
                        {
                        std::lock_guard<std::mutex> lock(main_joint_positions_mutex_);
                        fresh = joint_positions_received_ &&
                                (this->now() - last_joint_update_).seconds() <= 0.1;
                        }
                        if (!fresh) { publishZeroVelocity(); break; }

                        const bool left_ok  = transform_base(left_arm_);    
                        const bool right_ok = transform_base(right_arm_);
                        
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

                            

                            if (!compute_velocity_publisher(left_arm_, left_q))
                            {
                                left_arm_.target_world = previous_left; 
                                RCLCPP_ERROR(get_logger(), "Move to '%s' aborted: IK failed",
                                fixed_target_poses_.at("home_left").name.c_str());
                                left_arm_.trajectory_mode = TrajectoryMode::INACTIVE;  
                                state_manager.set_control_mode(ControlMode::TEST);      
                                break;
                            }
                            if (!compute_velocity_publisher(right_arm_, right_q)) 
                            {
                                right_arm_.target_world = previous_right; 
                                RCLCPP_ERROR(get_logger(), "Move to '%s' aborted: IK failed",
                                fixed_target_poses_.at("home_right").name.c_str());
                                right_arm_.trajectory_mode = TrajectoryMode::INACTIVE;
                                state_manager.set_control_mode(ControlMode::TEST);      
                                break;
                            }
                            
             

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
                    
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                        "OPERATION MODE!");
                    bool fresh;
                    {
                        std::lock_guard<std::mutex> lock(main_joint_positions_mutex_);
                        fresh = joint_positions_received_ &&
                                (this->now() - last_joint_update_).seconds() <= 0.1;
                    }
                    if (!fresh) { publishZeroVelocity(); break; }

                    const bool left_ok  = transform_base(left_arm_);
                    const bool right_ok = transform_base(right_arm_);
                    if (!left_ok || !right_ok) { publishZeroVelocity(); break; }
                    TempJoint left_q{state_manager.getLeftJointPositions()};
                    TempJoint right_q{state_manager.getRightJointPositions()};

                    operate_arm(left_arm_,  left_q,  state_manager.getLeftTarget(),  state_manager.getLeftGrip(),  left_vr_stamp_);
                    operate_arm(right_arm_, right_q, state_manager.getRightTarget(), state_manager.getRightGrip(), right_vr_stamp_);

                    if(button_a) state_manager.set_control_mode(ControlMode::TEST);
                    if(button_y) state_manager.set_control_mode(ControlMode::STARTUP);
                                       
                    if (right_trigger_ && !right_arm_.prev_trigger) {
                        right_arm_.gripper_closed = !right_arm_.gripper_closed;
                        set_gripper(right_arm_, right_arm_.gripper_closed);
                    }
                    right_arm_.prev_trigger = right_trigger_;
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
                    
                    const bool left_ok  = transform_base(left_arm_);    
                    const bool right_ok = transform_base(right_arm_);

                    if (!left_ok || !right_ok) { publishZeroVelocity(); break; }

                    TempJoint left_q{state_manager.getLeftJointPositions()};
                    TempJoint right_q{state_manager.getRightJointPositions()};

                    Eigen::Matrix4d current_pose_r =
                        right_arm_.T_world_base * forward_kinematics.forward_kinematics_matrix(dh_, right_q);
                    Eigen::Matrix4d current_pose_l =
                        left_arm_.T_world_base * forward_kinematics.forward_kinematics_matrix(dh_, left_q);

                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                        "Right current: x=%.3f y=%.3f z=%.3f",
                        current_pose_r(0, 3), current_pose_r(1, 3), current_pose_r(2, 3));
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                        "Left current: x=%.3f y=%.3f z=%.3f",
                        current_pose_l(0, 3), current_pose_l(1, 3), current_pose_l(2, 3));
                    // --- Keyboard target source ---
                    if (!keyboard_running) start_keyboard_control();
                    const auto previous_left = left_arm_.target_world;
                    const auto previous_right = right_arm_.target_world;

                    keyboardJog(left_arm_, left_q, left_keys_);

                    if (left_keys_.s_move) {
                        small_movement(left_arm_);
                        small_movement(right_arm_);

                        // keep the targets on the real pose so nothing pulls the arm back afterwards
                        left_arm_.target_world  = left_arm_.T_world_base  *
                            forward_kinematics.forward_kinematics_matrix(dh_, left_q);
                        right_arm_.target_world = right_arm_.T_world_base *
                            forward_kinematics.forward_kinematics_matrix(dh_, right_q);

                        left_arm_.last_valid  = left_arm_.target_world;
                        right_arm_.last_valid = right_arm_.target_world;

                        s_move_was_active_ = true;
                        break;
                    }
                    if (s_move_was_active_) {          
                        publishZeroVelocity();
                        s_move_was_active_ = false;
                        break;
                    }

                    if (left_keys_.home == true){
                        publishZeroVelocity();
                        state_manager.set_control_mode(ControlMode::STARTUP);
                        break;
                    }
                    if (left_keys_.operate == true){
                        publishZeroVelocity();
                        state_manager.set_control_mode(ControlMode::OPERATION);
                        break;
                    }
                    keyboardJog(right_arm_, right_q, right_keys_);
                    // -------------------------------
                
                    if (!compute_velocity_publisher(left_arm_, left_q))
                        left_arm_.target_world = previous_left; 
                    
                    if (!compute_velocity_publisher(right_arm_, right_q))
                        right_arm_.target_world = previous_right; 
                    
                    track_target(left_arm_, left_q);
                    track_target(right_arm_, right_q);

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
