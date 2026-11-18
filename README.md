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

### To launch IsaacSim with steamVR and publishers

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

```
ros2 launch david_launch dual_urdriver.launch.py   left_robot_ip:=192.168.57.101 right_robot_ip:=192.168.57.102 launch_rviz:=false
```

### Connection to the physcial robot (SSH connection)
1. Connect to the OmniLAB Wifi 
2. Change the IPV4 methods to manual

When changed to manual update these addresses
Address: 192.168.0.x (x means a value between 1 and 256. NOT 100, 101, 102 or 103!)
Netmask: 255.255.255.0
Gateway: 192.168.0.1
DNS: 8.8.8.8

3. SSH into the PC
```
ssh gr141@192.168.0.251
```
4. To logout
```
exit
```
This launches the MoveIT planner in another Rviz window

### To launch IsaacSim with steamVR and publishers on real robot

1. Launch steamVR through steam
2. Connect vr headset to steamapp and link to host pc
3. In a terminal, run these commands before launching Isaac:
```
export ROS_DOMAIN_ID=1
```
Sometimes, sourcing the david install folder is required. In this case, find your path to the install folder:
```
source "$HOME/david_rob5/install/setup.bash"
```

3. Launch IsaacSim with the following launch argument:
```
XR_RUNTIME_JSON="$HOME/.steam/debian-installation/steamapps/common/SteamVR/steamxr_linux64.json" ./isaac-sim.xr.vr.sh
```
cd ~/ROB5/david_rob5/setup
chmod +x install_david_launcher.sh
./install_david_launcher.sh




### Controlling the Real robot D.A.V.I.D. :

1. Turn on both pendants for the UR5E. It's important the portable e-stop, is active when starting the system.
2. Turn on the onboard pc.
3. On both pendants, load program Shao_grip_ on one pendant and Soren_grip_ on the other, and play program. This makes it possible to execute commands to the robot.
4. SSH into the onboard pc via:
   ```
   ssh gr141@192.168.0.251
   ```
   Do this in two terminals.
5. Run following command on terminal 1:
  ```
   cd ~/ROB5/david_rob5/setup/ && ./install_david_launcher.sh && ./launch_david_real.sh
  ```
6. Run this on terminal 2, when 1 says "[INFO] [spawner-3]: process has finished cleanly [pid 53]" :
   ```
   cd ~/ROB5/david_rob5/docker/ && docker exec -it david_docker_real bash
   ```
7. On terminal 2, build workspace and source, then when ready run:
   ```
   ros2 launch david_launch main_controller.launch.py
   ```
  When the program starts, it always start in the test mode, here it's possible to move the arms with:
  Left ARM: W/S/A/D for the xy-plane, and Z/X for z-plane.
  Right Arm: I/K/J/L for the xy-plane, and N/M for z-plane.
  H button makes the arms go into a home position.

8. To activate the vr teleoperation: (Need some isaac stuff before this):
  Press G on the keyboard to inter vr controlled mode.
  Best way to stop program or go out of vr controlled mode, press CTRL+C and start the program again.
  D.A.V.I.D. will not move, unless dead man switch is active.
  
   
