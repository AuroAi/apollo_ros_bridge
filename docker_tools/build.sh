#!/bin/bash

if [ -z "$@" ]
#TODO:: add hosted docker image url
IMGNAME="apollo_ros_bridge:latest"
then
      echo "using default image name = $IMGNAME"
else
      IMGNAME=$@
      echo "using image name $IMGNAME"
fi

docker build -t  $IMGNAME .
