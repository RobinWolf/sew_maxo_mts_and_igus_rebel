#!/bin/bash
##############################################################################
##                   Build the image, using dev.Dockerfile                  ##
##############################################################################
ROS_DISTRO=humble

uid=$(eval "id -u")
gid=$(eval "id -g")

#pass some arguments and settings to the dev.Dockerfile while building the image (dev.Dockerfile)

#--no-cache
docker build \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg UID="$uid" \
  --build-arg GID="$gid" \
  -f dev.Dockerfile \
  -t logilab-gazebo-dev/ros-render:"$ROS_DISTRO" .

##############################################################################
##                            Run the container                             ##
##############################################################################
SRC_CONTAINER=/home/logilab/ros2_ws/src
SRC_HOST="$(pwd)"/src                           #use a src on host machine to develop code, mout it to the container to run code inside the container

docker run \
  --name sew_navigation \
  --rm \
  -it \
  --net=host \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  -e DISPLAY="$DISPLAY" \
  -v /dev:/dev \
  --privileged \
  logilab-gazebo-dev/ros-render:"$ROS_DISTRO"

# display, joystick 0 input and network acess added to the container
# permissions for the joystick on the host machine should be: crw-rw-r--+ 1 robin robin 13, 0 Mai  6 15:21 /dev/input/js0


