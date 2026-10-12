# Virtual-to-Physical Teleoperation of a Mobile Dual-Arm Robot
![Platform](https://img.shields.io/badge/platform-Linux%20(Ubuntu%2024.04)-orange?logo=linux)
![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue?logo=ros)
![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-6.1-76B900?logo=nvidia)
![Static Badge](https://img.shields.io/badge/Docker%20-%2029.8.0%20-%20repo?logo=docker&color=%236495ED)
![Status](https://img.shields.io/badge/status-active%20research-brightgreen)

### Enter Docker:
1. Enter the docker folder:
```
cd david_rob5/docker
```

2. Build container (Needs to be done on every pc upstart):
```
docker compose up -d --build
```
In order to check if a container is running, use:
```
docker ps
```

3. Enter the docker container:
```
docker compose exec ros2 bash
```
To exit a container
```
exit
```
### To launch Rviz and MoveIT (Temporary):
1. When starting a new session this command has to be used in order to allow Docker to use your display (Outside the container)
```
xhost +local:docker
```
2. Enter the Docker container in 2 terminals (Guide is above)
3. First container
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true launch_rviz:=true
```
This starts the UR5 simulation in Rviz

4. Second container
```
ros2 launch ur_moveit_config ur_moveit.launch.py   ur_type:=ur5 launch_rviz:=true use_sim_time:=false
```
This launches the MoveIT planner in another Rviz window


ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true launch_rviz:=true initial_joint_controller:=forward_position_controller



### To move the robot with keyboard (Temporary):
1. First have to move it out of a singularity straight arm pose
```
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]}"
```
2. Then deactivate the UR_driver controller, that is blocking ours
```
ros2 control switch_controllers --activate forward_velocity_controller --deactivate forward_position_controller
```
3. Then run our controller
```
ros2 run david_controller controller
```

The commands are W and S to move back and forth on the y-axis.
A and D for the x-axis.
Z and X for the z-axis.



### REAL ROBOT

Noter fixer senere matias....

xhost +local:root

docker compose -f docker_sim_compose.yml up -d

docker exec -it david_docker_real bash

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.57.101 \
  launch_rviz:=true


ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 1.0}"
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 0.0}"
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 1.0}"
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 0.0}"

ros2 control switch_controllers   --deactivate scaled_joint_trajectory_controller   --activate forward_velocity_controller
