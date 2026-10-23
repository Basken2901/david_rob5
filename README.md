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
docker compose up -f docker-compose.yml up -d
```
In order to check if a container is running, use:
```
docker ps
```

3. Enter the docker container:
```
docker exec -it david_docker bash
```
This tells the docker we need to enter this container, make a shell and connect keyboard and screen to it.

To exit a container
```
exit
```

If the docker need to be reset, use (on host):
```
docker compose down
```

and build completely anew with a pull (require internet acess)

```
docker compose -f docker-compose.yml build --no-cache --pull
```

### To launch Rviz and control_node:
1. When starting a new session this command has to be used in order to allow Docker to use your display (Outside the container)
```
xhost +local:docker
```
2. Enter the Docker container and build and source like any other ros workspace, then launch:
```
ros2 launch david_launch dual_urdriver.launch.py left_use_mock_hardware:=true right_use_mock_hardware:=true launch_rviz:=true
```

4. Second container (comming later)


This launches control and topics from the simulated UR robots.

For the real robot (comming later)



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

Noter fixer senere mathias....

xhost +local:root


ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.57.101 \
  launch_rviz:=true


ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 1.0}"
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 16, state: 0.0}"
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 1.0}"
ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "{fun: 1, pin: 17, state: 0.0}"

ros2 control switch_controllers   --deactivate scaled_joint_trajectory_controller   --activate forward_velocity_controller

### Connection to the physcial robot (SSH connection)
1. Connect to the TP-LINK_2.4GHz_FFD51D Wifi
2. Change the IPV4 methods to manual

When changed to manual update these addresses
Address: 192.168.0.x (x means a value between 1 and 256. NOT 100, 101, 102 or 103!)
Netmask: 255.255.255.0
Gateway: 192.168.0.1
DNS: 8.8.8.8

3. SSH into the PC
```
ssh gr141@192.168.0.100
```
4. To logout
```
exit
```
