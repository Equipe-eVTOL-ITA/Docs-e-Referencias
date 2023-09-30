# Docker

## O que é

## Comandos

- ``` docker ps -all ```
- ``` docker image ls ```
- ``` docker rmi ```
- ``` docker start ```
- ``` docker restart ```
- ``` docker exec -it <container> bash ```
- ``` docker rename ```
- ``` docker stop ```
- ``` docker run ```
- ``` docker build -t <image> -f <filename> . ```
- docker run --rm --network host --privileged -v /dev:/dev -it [image_name]
- ```docker run  --net=host --ipc=host --pid=host --privileged -v /dev:/dev -it test_bridge```
- ```docker run -it --name=teste --net=host --ipc=host --pid=host --privileged -v /dev:/dev -v /dev/shm:/dev/shm  test_bridge```
- ```docker run -it --name=orb1 --net=host --ipc=host --pid=host --privileged -v /dev/:/dev/ -v /dev/shm:/dev/shm -v ~/Downloads:/config --privileged -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix orbslam3 bash```

## Referências

[1](https://github.com/eProsima/Fast-DDS/issues/2956) Docker detecta mas não consegue se inscrever nos tópicos.

[2](https://github.com/osrf/docker_images/blob/c2794739685fab1bfebeedad812a10b5676e62cb/ros/foxy/ubuntu/focal/ros1-bridge/Dockerfile) ROS1 Bridge