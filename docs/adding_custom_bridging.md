# Adding custom bridge code

The apollo ros bridge package supports adding bridging between any ROS message and Cyber message as long as they have been set up properly within the package.

## Terminology

### Trigger Mode

Lets say three topics have to be fused to one topic. Topic A is published at 10 hz, Topic B at 10 hz and Topic C at 100 Hz. Then, Topic C is called the **Trigger topic**. This is the topic which decides the publishing rate. All conversions are performed in callbacks of Topic C. What this means is that the converted topic will be published at 100Hz. Callbacks of Topic A and Topic B will merely store data which will then be used in callback of Topic C.

![trigger topic explanation](https://github.com/AuroAi/apollo_ros_bridge/blob/master/images/trigger_topic_explanation.jpeg "trigger topic explanation")

### Timer Mode

In this method, publishing is handled by timers whose frequency is specified by the developer. Callbacks of all topics will merely store data for use in the timer callback functions.

Currently, only trigger mode is supported. Timer Mode will be supported in the recent future

## Setting up ros messages 

Setting up of custom ROS messages is fairly easy. 
1.  Add the ROS message packages into the ros_pkgs workspace
1. Build the ROS msg package

The header files generated for the ROS messages are automatically referred to by Bazel when building the cyber_ros_bridge package using the [ros_pkgs.BUILD ](https://github.com/AuroAi/apollo_ros_bridge/blob/master/ros_bazel_builds/ros_pkgs.BUILD "ros_pkgs.BUILD ")file.

Alternatively, custom ROS workspaces can be added by modifying the WORKSPACE file and creating an appropriate BUILD file.

Since an include prefix has been included inside the BUILD file for external ros packages, headers for the ROS messages should be included with the complete path. 

For eg.

    #include "ros_pkgs/include/nav_msgs/Odometry.h"

## Configuring YAML file

The yaml file in the folder[ **yaml_defs** ](https://github.com/AuroAi/apollo_ros_bridge/tree/master/cyber_ros_bridge/yaml_defs " **yaml_defs** ") specifies topic names, which topics have to be used to bridge into relevant topics, which topic is trigger, etc.

### Yaml file structure


```yaml
    - ROS/Apollo:
       - Output_Topic_Name: name of the output ROS topic
         msg_type: message type
         Input_Topics:
           - Input_Topic_Name: Apollo topic name
             msg_type: apollo message type
             trigger_topic: Set to true for trigger topic
         Input_Topics:
           - Input_Topic_Name: Apollo topic name
             msg_type: apollo message type
             trigger_topic: Set to false for all other topics
```

Multiple topics can be mapped to one Output Topic

An [example yaml](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/yaml_defs/default.yaml "example yaml")  file is included in the yaml_defs folder

## Adding custom bridge code

### Configuring data storage
To make storing data into variables easier, two structs have been created.

Inside the file [cyber_ros_bridge/lib/Data.hpp ](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/lib/Data.hpp "cyber_ros_bridge/lib/Data.hpp "), add appropriate Apollo topics and ROS topics that will be used in bridging.

For example,

```cpp
 struct ApolloData
 {
   // extensive list of data to and from apollo
   DataField<apollo::localization::LocalizationEstimate> localization;
   DataField<apollo::planning::ADCTrajectory> adctrajectory;
   DataField<apollo::canbus::Chassis> chassis;
   DataField<apollo::control::ControlCommand> control_cmd;
   DataField<apollo::drivers::PointCloud> point_cloud;
 };

 struct ROSData
 {
   // extensive list of data to and from ROS
   DataField<nav_msgs::Path> path;
   DataField<nav_msgs::Odometry> odom;
   DataField<sensor_msgs::PointCloud2> point_cloud;
 };


```
Make sure, header files are added to both the [Data.hpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/lib/Data.hpp "Data.hpp")  file and to the [BUILD](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/lib/BUILD "BUILD")  file in the lib folder to avoid bazel build errors.

### Specifying supported topics 

To make the code safe during runtime, a list of supported topics have to be specified by the developer before compile time.

In the file [cyber_ros_bridge/core/parse_yaml.hpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/core/parse_yaml.hpp "cyber_ros_bridge/core/parse_yaml.hpp").

Find the [lines](https://github.com/AuroAi/apollo_ros_bridge/blob/4e139f726ea9aa70b4a10311f6cbca99baec3d4e/cyber_ros_bridge/core/parse_yaml.hpp#L84 "lines") which specify the supported topic list.

```cpp
 //list of supported ros topics
 std::vector<std::string> supported_ros_topics_list_ = {"nav_msgs::Odometry",
                                                        "nav_msgs::Path",
                                                        "sensor_msgs::PointCloud2"};

 //list of supported cyber topics
 std::vector<std::string> supported_cyber_topics_list_ = {"apollo::localization::LocalizationEstimate",
                                                          "apollo::control::ControlCommand",
                                                          "apollo::drivers::PointCloud",
                                                          "apollo::canbus::Chassis",
                                                          "apollo::planning::ADCTrajectory"};


```

### Creating subscribers and publishers

#### subscriber/reader and publisher/writer variables

For obvious reasons, there is a need to explicitly create variables for subscriptions (reading in the case of cyber) and publishing (writing in the case of cyber) of messages. In the file [cyber_ros_bridge/core/cyber_ros_bridge_core.hpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/core/cyber_ros_bridge_core.hpp "cyber_ros_bridge/core/cyber_ros_bridge_core.hpp"),

##### subscribers and readers
Add appropriate [subscriber or reader variables](https://github.com/AuroAi/apollo_ros_bridge/blob/4e139f726ea9aa70b4a10311f6cbca99baec3d4e/cyber_ros_bridge/core/cyber_ros_bridge_core.hpp#L69 "subscriber or reader variables")

eg:

```cpp
 // ROS subscriber objects;
 ros::Subscriber ros_pc_sub_;


 // Apollo reader objects
 std::shared_ptr<apollo::cyber::Reader<apollo::drivers::PointCloud>> apollo_point_cloud_reader_;
 std::shared_ptr<apollo::cyber::Reader<apollo::localization::LocalizationEstimate>> apollo_localization_reader_;
 std::shared_ptr<apollo::cyber::Reader<apollo::planning::ADCTrajectory>> apollo_trajectory_reader_;

```

#####  publishers and writers

Similarly, create [publishers and writer variables](https://github.com/AuroAi/apollo_ros_bridge/blob/4e139f726ea9aa70b4a10311f6cbca99baec3d4e/cyber_ros_bridge/core/cyber_ros_bridge_core.hpp#L69 "publishers and writer variables") for relevant messages 

eg:

```cpp
 // ROS Publisher objects
 ros::Publisher point_cloud_pub_;
 ros::Publisher nav_path_pub_;
 ros::Publisher localization_pub_;

 // Apollo writer objects
 std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> pc_writer_;
```

##### Registering Subscribers

###### Default subscribers
Default subscribers are non trigger subscribers.

In order to ease development,  template functions are provided for non-trigger subscribers.

In the file [cyber_ros_bridge/core/cyber_ros_bridge.cpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/core/cyber_ros_bridge_core.cpp "cyber_ros_bridge/core/cyber_ros_bridge.cpp"), in the function [RegisterDefaultSubscribers](https://github.com/AuroAi/apollo_ros_bridge/blob/4e139f726ea9aa70b4a10311f6cbca99baec3d4e/cyber_ros_bridge/core/cyber_ros_bridge_core.cpp#L86 "RegisterDefaultSubscribers"), add an **else if** statement registering the subscriber/reader. 

Example:

```cpp
// ---------------ROS Topics---------------------
 if (topic.topic_type == "sensor_msgs::PointCloud2")
 {
   ros_pc_sub_ = private_ros_node_handle_ptr_->subscribe<sensor_msgs::PointCloud2>(
       topic.topic_name, 1,
       boost::bind(&cyber_ros_bridge_core::ROScallback<sensor_msgs::PointCloud2>, this, _1, boost::ref(ros_data_.point_cloud)));
 }

 // ---------------Apollo Topics---------------------

 else if (topic.topic_type == "apollo::drivers::PointCloud")
 {
   apollo_point_cloud_reader_ = private_cyber_node_handle_ptr_->CreateReader<apollo::drivers::PointCloud>(
       topic.topic_name, boost::bind(&cyber_ros_bridge_core::ApolloCallBack<apollo::drivers::PointCloud>, this, _1,
                                     boost::ref(apollo_data_.point_cloud)));
 }


```

Note: Trigger topics can be added to default subscribers but will not be initialized as default during runtime.


###### Trigger  subscribers

Since Trigger subscribers are the topics which will perform the conversion of messages and publish them, the callbacks have to be custom coded. Examples are provided for help.

In function [RegisterTriggerSubscribers](https://github.com/AuroAi/apollo_ros_bridge/blob/4e139f726ea9aa70b4a10311f6cbca99baec3d4e/cyber_ros_bridge/core/cyber_ros_bridge_core.cpp#L126 "RegisterTriggerSubscribers"),

```cpp
// ---------------ROS Topics---------------------
 if (topic.topic_type == "sensor_msgs::PointCloud2")
 {
   ros_pc_sub_ = private_ros_node_handle_ptr_->subscribe(topic.topic_name, 1, &cyber_ros_bridge_core::ROSPCCallback, this);
 }

 // ---------------Apollo Topics---------------------
 // calling custom callbacks
 else if (topic.topic_type == "apollo::planning::ADCTrajectory")
 {
   apollo_trajectory_reader_ = private_cyber_node_handle_ptr_->CreateReader<apollo::planning::ADCTrajectory>(
       topic.topic_name, std::bind(&cyber_ros_bridge_core::ApolloTrajectoryCallback, this, std::placeholders::_1));
 }

```
##### Writing Callbacks

Since trigger subscribers handle special cases, the callbacks have to be custom coded. In the example in the package, conversion functions are written in the [lib](https://github.com/AuroAi/apollo_ros_bridge/tree/master/cyber_ros_bridge/lib "lib") folder and are called in custom callbacks.

```cpp
void cyber_ros_bridge_core::ApolloTrajectoryCallback(const std::shared_ptr<apollo::planning::ADCTrajectory> &msg)
{

  try
  {
    // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
    std::lock_guard<std::mutex> lck(apollo_data_.adctrajectory.mutex);
    // update CyberData struct
    apollo_data_.adctrajectory.data = *msg;
    // call library which converts to ros message
    nav_msgs::Path nav_path;
    ApolloTrajectoryToROSPath(msg, nav_path);

    ros_data_.path.data = nav_path;
    // publish ROS message
    nav_path_pub_.publish(nav_path);
  }
  catch (const std::exception &e)
  {
    AERROR << e.what() << '\n';
  }
}

```

#### Custom Conversion methods

For conversion of messages between Apollo and ROS, conversion libraries can be written in the [lib](https://github.com/AuroAi/apollo_ros_bridge/tree/master/cyber_ros_bridge/lib "lib") folder.

Example: For converting  `apollo::planning::ADCTrajectory` to  `nav_msgs::Path`, the following [function](https://github.com/AuroAi/apollo_ros_bridge/blob/4e139f726ea9aa70b4a10311f6cbca99baec3d4e/cyber_ros_bridge/lib/module_navigation.cpp#L23 "function") is used.

```cpp
void cyber_ros_bridge::ApolloTrajectoryToROSPath(const std::shared_ptr<apollo::planning::ADCTrajectory> &trajectory_apollo, nav_msgs::Path &path_cmd)
{
 for (const auto &trajectory_point : trajectory_apollo->trajectory_point())
 {
   geometry_msgs::PoseStamped pose;
   pose.header.frame_id = FLAGS_world_frame;
   pose.pose.position.x = trajectory_point.path_point().x();
   pose.pose.position.y = trajectory_point.path_point().y();
   pose.pose.position.z = trajectory_point.path_point().z();

   tf::Quaternion q_tf;
   q_tf.setRPY(0.0, 0.0, trajectory_point.path_point().theta());
   tf::quaternionTFToMsg(q_tf, pose.pose.orientation);

   path_cmd.poses.push_back(pose);
 }

 path_cmd.header.frame_id = FLAGS_world_frame;
 path_cmd.header.stamp = ros::Time::now();
}

```

##### Registering Publishers

To register publishers, [RegisterPublishers](https://github.com/AuroAi/apollo_ros_bridge/blob/4e139f726ea9aa70b4a10311f6cbca99baec3d4e/cyber_ros_bridge/core/cyber_ros_bridge_core.cpp#L160 "RegisterPublishers") can be used. Just copy the template and modify with relevant publisher/writer variables and message types.

```cpp
  else if (topic.topic_type == "nav_msgs::Odometry")
  {
    localization_pub_ = private_ros_node_handle_ptr_->advertise<nav_msgs::Odometry>(topic.topic_name, 1);
  }

  else if (topic.topic_type == "apollo::drivers::PointCloud")
  {
    pc_writer_ = private_cyber_node_handle_ptr_->CreateWriter<apollo::drivers::PointCloud>(topic.topic_name);
  }
```

#### Specifying runtime Flags

To help development, runtime flags can be specified using gflags. Refer to the documentation [here](https://gflags.github.io/gflags/ "here")

#### Building the code and running


In order to build the bridge, first build the appropriate ros messages used in the bridge source code:

    cd /home/cyber_ros_bridge/ros_pkgs
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

For detailed information on how to add own bridging of topics, refer to [adding custom bridging](https://github.com/AuroAi/apollo_ros_bridge/blob/master/docs/adding_custom_bridging.md "here")

Run [Apollo ](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_3_5_quick_start.md "Apollo ")

Topics can be visualized with rostopic list and echo

Alternatively, publish relevant topics from ROS and view topics using [cyber_monitor](https://github.com/ApolloAuto/apollo/blob/master/docs/cyber/CyberRT_Developer_Tools.md "cyber_monitor") on Apollo's side









