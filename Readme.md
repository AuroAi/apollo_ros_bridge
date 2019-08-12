
# Apollo ROS Bridge


## Overview

This is a C++ library which is used to interface messages between [ROS](http://www.ros.org/ "ROS")  and [Apollo](https://github.com/ApolloAuto/apollo "Apollo") ecosystem. It is designed to help developers use ROS libraries to communicate with Apollo and hence make development easier. 

Affiliation:  [Ridecell](http://auro.ai/ "Ridecell") \
Maintainer: Abhilash Balachandran abhilash@ridecell.com

![](images/lidar_pcl.gif "Lidar Pointcloud received from Apollo , converted and published as a ROS message. {Left} Cyber_visualizer tool showing cyber pointcloud message. {Right} Rviz showing ros pointcloud message")
Lidar Pointcloud received from Apollo , converted and published as a ROS message. {Left} Cyber_visualizer tool showing cyber pointcloud message. {Right} Rviz showing ros pointcloud message

## Dependencies

- [Apollo 3.5 ](https://github.com/ApolloAuto/apollo/tree/r3.5.0 "Apollo 3.5 ") 
- [nvidia docker ](https://github.com/NVIDIA/nvidia-docker "nvidia docker ")

## Installation

### Using Prebuilt docker image

There is a prebuild docker image which can be directly used. To use the prebuilt docker image:

    cd docker_tools
    ./run.sh

### Building docker image

To build the docker image on your machine, 

    cd docker_tools
    ./build.sh $image_name

If no image name is provided, it defaults to auroai/apollo_ros_bridge:latest

### Building bridge code

In order to build the bridge, first build the appropriate ros messages used in the bridge source code:

    cd /home/apollo_ros_bridge/ros_pkgs
    catkin build

Now, build the bridge source code using bazel. From the root workspace of the package,
     
    cd /home/apollo_ros_bridge
    bazel build cyber_ros_bridge:all


To run the example node, run

    roscore && ./bazel-bin/cyber_ros_bridge/cyber_ros_bridge

for extra logging,

    GLOG_v=4 GLOG_logtostderr=1  ./bazel-bin/cyber_ros_bridge/cyber_ros_bridge

This launches the bridge with default params (defined in the common folder).

Alternatively, to launch the bridge with custom parameters,

    cyber_launch start cyber_ros_bridge/launch/bridge_example.launch

Inside Apollo's repo, modify the following lines in [docker/scripts/dev_start.sh](https://github.com/ApolloAuto/apollo/blob/r3.5.0/docker/scripts/dev_start.sh)

Comment out the following [lines](https://github.com/ApolloAuto/apollo/blob/b7df51a491557ad656d219a8f553c286ec053419/docker/scripts/dev_start.sh#L304)

    --add-host in_dev_docker:127.0.0.1 \
    --add-host ${LOCAL_HOST}:127.0.0.1 \
    --hostname in_dev_docker \
This permits communication via ROS ecosystem

Run [Apollo ](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_5_quick_start.md "Apollo ")

Topics can be visualized with rostopic list and echo

Alternatively, publish relevant topics from ROS and view topics using [cyber_monitor](https://github.com/ApolloAuto/apollo/blob/master/docs/cyber/CyberRT_Developer_Tools.md "cyber_monitor") on Apollo's side

A sample script to run a ROS Container has been provided to help developers. This can be launched using:

```bash
cd /home/apollo_ros_bridge/docker_tools
./example_ros_run.sh
```


## Adding Custom Bridging

For detailed information on how to add own bridging of topics, refer to [adding custom bridging](https://github.com/AuroAi/apollo_ros_bridge/blob/master/docs/adding_custom_bridging.md "here")

## Copyright and License

The apollo ros bridge source code is released under the [Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0 "Apache License, Version 2.0")

## Future Updates
 - [ ] Support for apollo 5.0
 - [ ] Timer method for bridging
 - [ ] easier docker setup
