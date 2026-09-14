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
