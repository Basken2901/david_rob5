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
ros2 launch ur_robot_driver ur_control.launch.py \ ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy \ use_mock_hardware:=true launch_rviz:=true
```
This starts the UR5 simulation in Rviz

4. Second container
```
ros2 launch ur_moveit_config ur_moveit.launch.py \ ur_type:=ur5 launch_rviz:=true use_sim_time:=false
```
This launches the MoveIT planner in another Rviz window
