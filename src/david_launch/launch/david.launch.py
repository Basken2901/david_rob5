import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch_ros.actions import Node

def load_yaml(pkg, rel):
    path = os.path.join(share(pkg), rel)
    with open(path) as f:
        data = yaml.safe_load(f)
    if data is None:
        raise RuntimeError(f"{path} is empty")
    return data

def generate_launch_description():
    urdf = xacro.process_file(
        os.path.join(share("ur_description"), "urdf", "ur.urdf.xacro"),
        mappings={"name": "ur", "ur_type": "ur5"},
    ).toxml()

    srdf = xacro.process_file(
        os.path.join(share("ur_moveit_config"), "srdf", "ur.srdf.xacro"),
        mappings={"name": "ur", "prefix": ""},
    ).toxml()

    servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            {"moveit_servo": load_yaml("david_launch", "config/servo.yaml")},
            {"robot_description": urdf},
            {"robot_description_semantic": srdf},
            {"robot_description_kinematics": load_yaml("ur_moveit_config", "config/kinematics.yaml")},
            {"robot_description_planning": load_yaml("ur_moveit_config", "config/joint_limits.yaml")},
        ],
    )

    return LaunchDescription([servo])