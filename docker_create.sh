#!/bin/bash
scriptDir=$(dirname $0 | xargs -i readlink -f {})
container_name="ros/noetic"
version="0.2"
does_exist=$(docker image ls $container_name:$version | grep -ci1 $container_name)
if [ $does_exist == "0" ] ; then
	docker build -t $container_name:$version .
fi
docker run --rm \
    --privileged \
    --name "ros-noetic" \
    --env DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$scriptDir/:/home/developer/ws/src" \
    -it $container_name:$version /bin/bash