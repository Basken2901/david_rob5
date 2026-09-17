# Virtual-to-Physical Teloperation of a Mobile Dual-Arm Robot
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
