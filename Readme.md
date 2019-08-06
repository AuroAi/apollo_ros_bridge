# Apollo ROS Bridge


## Overview

This is a C++ library which is used to interface messages between [ROS](http://http://www.ros.org/ "ROS")  and [Apollo](https://github.com/ApolloAuto/apollo "Apollo") ecosystem. It is designed to help developers use ROS libraries to communicate with Apollo and hence make development easier. 

Affiliation:  [Ridecell](http://auro.ai/ "Ridecell")

Maintainer: Abhilash Balachandran abhilash@ridecell.com

## Dependencies

- [Apollo ](https://github.com/ApolloAuto/apollo "Apollo")
- [nvidia docker ](https://github.com/NVIDIA/nvidia-docker "nvidia docker ")

## Installation

### Apollo
Clone apollo auto autonomous driving framework 
```
git clone https://github.com/ApolloAuto/apollo.git
```
Note: release 5.0 is preferred but not mandatory

### Apollo ros bridge
Clone the apollo ros bridge into the apollo folder
```
cd apollo
git clone https://github.com/AuroAi/apollo_ros_bridge.git -b feature/no_dedicated_container
```

### Building docker image

To build the docker image on your machine, 

You need to tag the apollo's docker to apolloauto/apollo:dev-latest

```
docker tag apolloauto/apollo:<apollo's tag> apolloauto/apollo:dev-latest
```
<apollo's tag > is the tag that the apollo image has. Run docker images to find the tag

    cd docker_tools
    ./build_dev_bridge.sh ./dev_bridge.x86_64.dockerfile

### Launching apollo's docker

```
./dev_bridge_start.sh
./dev_bridge_into.sh
```

### Building bridge code

In order to build the bridge, first build the appropriate ros messages used in the bridge source code:

     cd apollo_ros_bridge/ros_pkgs
     catkin build

#### Modifications in Apollo repo

Minor modifications are made to the Apollo's repo'

Modify the [WORKSPACE.in](https://github.com/ApolloAuto/apollo/blob/r5.0.0/WORKSPACE.in "WORKSPACE.in") File to include building ros and the custom ros messages. Add the following lines

```bash
#ros_indigo
new_local_repository(
    name = "ros_indigo",
    build_file = "apollo_ros_bridge/ros_bazel_builds/ros_indigo.BUILD",
    path = "/opt/ros/indigo",
)

#ros_pkgs
new_local_repository(
    name = "ros_pkgs",
    build_file = "apollo_ros_bridge/ros_bazel_builds/ros_pkgs.BUILD",
    path = "apollo_ros_bridge/ros_pkgs/devel",
)
```

To automatically build the bridge, modify the following lines in the [apollo.sh](https://github.com/ApolloAuto/apollo/blob/r5.0.0/apollo.sh "apollo.sh") script [line 119](https://github.com/ApolloAuto/apollo/blob/2e8ad6fecb323915eeb74efa05cfd1647d6c6138/apollo.sh#L119 "line 119")

```bash
BUILD_TARGETS=`bazel query //modules/... union //cyber/...`
```
to

```bash
BUILD_TARGETS=`bazel query //modules/... union //cyber/... union //apollo_ros_bridge/cyber_ros_bridge/...`
```

Build apollo
```
cd /apollo 
./apollo.sh build
```

Now, build the bridge source code using bazel. From the root workspace of the package,
       
    bazel build apollo_ros_bridge/cyber_ros_bridge:all


To run the example node, run

    roscore && ./bazel-bin/apollo_ros_bridge/cyber_ros_bridge/cyber_ros_bridge

for extra logging,

    GLOG_v=4 GLOG_logtostderr=1  ./bazel-bin/apollo_ros_bridge/cyber_ros_bridge/cyber_ros_bridge

This launches the bridge with default params (defined in the common folder).

Alternatively, to launch the bridge with custom parameters,

    cyber_launch start apollo_ros_bridge/cyber_ros_bridge/launch/bridge_example.launch

For detailed information on how to add own bridging of topics, refer to here (TODO:: add link to documentation here)

Run [Apollo ](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_5_quick_start.md "Apollo ")

Topics can be visualized with rostopic list and echo

Alternatively, publish relevant topics from ROS and view topics using [cyber_monitor](https://github.com/ApolloAuto/apollo/blob/master/docs/cyber/CyberRT_Developer_Tools.md "cyber_monitor") on Apollo's side

## Copyright and License

The apollo ros bridge source code is released under the [Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0 "Apache License, Version 2.0")


