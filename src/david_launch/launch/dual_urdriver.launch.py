"""
david_launch/launch/dual_urdriver.launch.py

Starts BOTH UR arms under ONE controller_manager.

  # no robots at all (fake arms that integrate your velocity commands):
  ros2 launch david_launch dual_urdriver.launch.py use_mock_hardware:=true

  # only the RIGHT robot connected, left arm faked:
  ros2 launch david_launch dual_urdriver.launch.py left_use_mock_hardware:=true right_robot_ip:=<ip>

  # URSim (defaults) or real robots:
  ros2 launch david_launch dual_urdriver.launch.py
  ros2 launch david_launch dual_urdriver.launch.py left_robot_ip:=<ip> right_robot_ip:=<ip>

Afterwards:
  /joint_states                                 <- all 12 joints (your node + Isaac read this)
  /left_forward_velocity_controller/commands    <- your node publishes 6 velocities
  /right_forward_velocity_controller/commands
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchConfiguration
    pkg = FindPackageShare("david_launch")

    args = [
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.56.101"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.56.102"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Fake BOTH arms"),
        # per-arm override, e.g. only the right robot connected: left_use_mock_hardware:=true
        DeclareLaunchArgument("left_use_mock_hardware", default_value=LaunchConfiguration("use_mock_hardware")),
        DeclareLaunchArgument("right_use_mock_hardware", default_value=LaunchConfiguration("use_mock_hardware")),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
    ]

    # ---------- 1. URDF: run xacro on our file, with the launch arguments ----------
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                PathJoinSubstitution([pkg, "urdf", "DAVID_dual.urdf.xacro"]),
                " left_robot_ip:=", lc("left_robot_ip"),
                " right_robot_ip:=", lc("right_robot_ip"),
                " left_use_mock_hardware:=", lc("left_use_mock_hardware"),
                " right_use_mock_hardware:=", lc("right_use_mock_hardware"),
                " headless_mode:=", lc("headless_mode"),
            ]),
            value_type=str,
        )
    }

    # ---------- 2. robot_state_publisher: URDF + /joint_states -> TF, and publishes /robot_description ----------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # ---------- 3. controller_manager: reads /robot_description, loads both UR hardware interfaces ----------
    controllers_file = PathJoinSubstitution([pkg, "config", "dual_ur_controllers.yaml"])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        output="screen",
    )

    # ---------- 4. spawners: ask controller_manager to load + activate controllers ----------
    def spawner(names, inactive=False):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=names
            + ["--controller-manager", "/controller_manager",
               "--controller-manager-timeout", "30"]
            + (["--inactive"] if inactive else []),
            output="screen",
        )

    spawn_active = spawner([
        "joint_state_broadcaster",
        "left_io_and_status_controller",
        "right_io_and_status_controller",
        "left_speed_scaling_state_broadcaster",
        "right_speed_scaling_state_broadcaster",
        "left_forward_velocity_controller",
        "right_forward_velocity_controller",
    ])

    # ---------- 5. per-arm helpers (real robot / URSim only) ----------
    def per_arm(arm):
        not_mock = UnlessCondition(lc(f"{arm}_use_mock_hardware"))
        return [
            # play/stop/power-on over port 29999: /<arm>/dashboard_client/...
            Node(
                package="ur_robot_driver",
                executable="dashboard_client",
                name="dashboard_client",
                namespace=arm,
                parameters=[{"robot_ip": lc(f"{arm}_robot_ip")}],
                output="screen",
                condition=not_mock,
            ),
            # deactivates the motion controllers when the pendant program stops,
            # so the arm doesn't jump to a stale command when you press play again
            Node(
                package="ur_robot_driver",
                executable="controller_stopper_node",
                name="controller_stopper",
                namespace=arm,
                parameters=[
                    {"headless_mode": lc("headless_mode")},
                    {"joint_controller_active": True},
                    {"consistent_controllers": [
                        "joint_state_broadcaster",
                        "left_io_and_status_controller",
                        "right_io_and_status_controller",
                        "left_speed_scaling_state_broadcaster",
                        "right_speed_scaling_state_broadcaster",
                    ]},
                ],
                output="screen",
                condition=not_mock,
            ),
        ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg, "rviz", "dual.rviz"])],
        output="log",
        condition=IfCondition(lc("launch_rviz")),
    )

    return LaunchDescription(
        args
        + [rsp, control_node, spawn_active, rviz]
        + per_arm("left")
        + per_arm("right")
    )
