#include "arm_clearance_checker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include <moveit/collision_detection/collision_env.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

namespace
{
constexpr std::array<const char*, 2> ARM_PREFIXES{"left_", "right_"};

constexpr std::array<const char*, 6> ARM_LINKS{
    "shoulder_link", "upper_arm_link", "forearm_link",
    "wrist_1_link", "wrist_2_link", "wrist_3_link"};

constexpr std::array<const char*, 6> ARM_JOINTS{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

constexpr double MIN_CLEARANCE_METERS = 0.05;
constexpr auto MAX_CHECK_AGE = std::chrono::milliseconds(100);
}

ArmClearanceChecker::ArmClearanceChecker(rclcpp::Node& node) : node_(node)
{
    for (const char* prefix : ARM_PREFIXES)
    {
        for (const char* joint : ARM_JOINTS)
            joint_names_.push_back(std::string(prefix) + joint);
    }

    description_sub_ = node_.create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String& msg) { load_model(msg.data); });

    RCLCPP_INFO(node_.get_logger(), "Waiting for /robot_description");
}

void ArmClearanceChecker::load_model(const std::string& description)
{
    clearance_safe_ = false;
    last_valid_check_ = {};
    scene_.reset();

    auto urdf_model = std::make_shared<urdf::Model>();
    if (!urdf_model->initString(description))
    {
        RCLCPP_ERROR(node_.get_logger(), "Could not parse /robot_description");
        return;
    }

    // The URDF supplies the geometry. The matrix below selects the pairs.
    auto srdf_model = std::make_shared<srdf::Model>();
    auto scene = std::make_unique<planning_scene::PlanningScene>(
        urdf_model, srdf_model);
    const auto& model = scene->getRobotModel();

    const auto& variables = model->getVariableNames();
    for (const auto& joint : joint_names_)
    {
        if (std::find(variables.begin(), variables.end(), joint) == variables.end())
        {
            RCLCPP_ERROR(node_.get_logger(), "Missing joint %s", joint.c_str());
            return;
        }
    }

    std::vector<std::string> left_links;
    std::vector<std::string> right_links;

    for (const char* suffix : ARM_LINKS)
    {
        left_links.push_back(std::string("left_") + suffix);
        right_links.push_back(std::string("right_") + suffix);

        for (const auto* name : {&left_links.back(), &right_links.back()})
        {
            const auto* link = model->getLinkModel(*name);
            if (!link || link->getShapes().empty())
            {
                RCLCPP_ERROR(node_.get_logger(),
                             "No collision geometry loaded for %s", name->c_str());
                return;
            }
        }
    }

    collision_matrix_ = collision_detection::AllowedCollisionMatrix(
        model->getLinkModelNames(), true);
    collision_matrix_.setEntry(left_links, right_links, false);

    scene_ = std::move(scene);
    RCLCPP_INFO(node_.get_logger(), "Checking %zu cross-arm link pairs",
                left_links.size() * right_links.size());
}

bool ArmClearanceChecker::motion_allowed() const
{
    return clearance_safe_ &&
           std::chrono::steady_clock::now() - last_valid_check_ < MAX_CHECK_AGE;
}

void ArmClearanceChecker::check_clearance(
    const sensor_msgs::msg::JointState& msg)
{
    const auto now = std::chrono::steady_clock::now();
    // Every failed or incomplete measurement blocks motion immediately.
    clearance_safe_ = false;
    if (!scene_)
        return;

    std::vector<double> positions;
    positions.reserve(joint_names_.size());

    for (const auto& joint : joint_names_)
    {
        bool found = false;
        for (std::size_t i = 0; i < msg.name.size(); ++i)
        {
            if (msg.name[i] != joint)
                continue;

            if (found || i >= msg.position.size() || !std::isfinite(msg.position[i]))
            {
                RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                     "Invalid or duplicate joint %s", joint.c_str());
                return;
            }

            positions.push_back(msg.position[i]);
            found = true;
        }

        if (!found)
        {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                 "Joint state is missing %s", joint.c_str());
            return;
        }
    }

    const auto start = std::chrono::steady_clock::now();

    auto& state = scene_->getCurrentStateNonConst();
    state.setVariablePositions(joint_names_, positions);
    state.update();

    collision_detection::DistanceRequest request;
    request.acm = &collision_matrix_;

    collision_detection::DistanceResult result;
    scene_->getCollisionEnvUnpadded()->distanceSelf(request, result, state);

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    const auto& closest = result.minimum_distance;

    if (closest.link_names[0].empty() || closest.link_names[1].empty() ||
        !std::isfinite(closest.distance))
    {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                             "MoveIt returned no cross-arm distance");
        return;
    }

    if (result.collision || closest.distance <= MIN_CLEARANCE_METERS)
    {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                    "Motion blocked: %.2f cm between %s and %s (minimum %.2f cm)",
                    closest.distance * 100.0,
                    closest.link_names[0].c_str(), closest.link_names[1].c_str(),
                    MIN_CLEARANCE_METERS * 100.0);
        return;
    }

    last_valid_check_ = now;
    clearance_safe_ = true;
    RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 500,
                "%.2f cm between %s and %s (%.2f ms)",
                closest.distance * 100.0, closest.link_names[0].c_str(),
                closest.link_names[1].c_str(), elapsed_ms);
}
