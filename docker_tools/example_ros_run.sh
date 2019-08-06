#!/bin/bash

docker run \
  -it \
  -v `dirname  $(pwd)`:/home/apollo_ros_bridge \
  -v `dirname  $(pwd)`/container_root:/root \
  -v /etc/timezone:/etc/timezone:ro \
  -v /etc/localtime:/etc/localtime:ro \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_GRAPHICSSYSTEM=native \
  -e IMAGE_NAME=auroai/apollo_ros_bridge \
  -e TZ=`cat /etc/timezone` \
  --network host \
  --privileged \
 ros:indigo-ros-core-trusty

#   -v ``:/home/cyber_ros_bridge/.bazel
