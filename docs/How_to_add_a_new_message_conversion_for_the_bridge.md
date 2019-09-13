# How to add a new message conversion for the bridge

AuroAI releases an open source tool that performs the message coversion between Apollo Cyber RT and native Ros. In this manual, we will use IMU message as an example, to explanin the details about how to add a new message conversion between ROS and Apollo.

The manual exhaustively demonstrates the exact steps for users to add their own messages:

1. Adding the appropriate Apollo's and Ros' message in the bridge.
2. Change the configuration file for corresponding input and output topics
3. Declare new message instances.
4. Insert the new topic in the supported lists.
5. Declare the topic's subscribers and readers.
6. Register the topic's trigger subscribers.
7. Register the topic's publishers.
8. Implement the callback functions.
9. Convert message between cyber's and Ros's message.

## Adding the appropriate Apollo's and Ros' message in the bridge
Regarding this example, we are reusing the existing message format, which is Ros IMU message in the file [Imu.msg](https://github.com/AuroAi/apollo_ros_bridge/blob/master/ros_pkgs/src/common_msgs-indigo-devel/sensor_msgs/msg/Imu.msg).

```sh
Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance 
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance 
```

The denifition of the IMU message in cyber RT is in the file [imu.proto](https://github.com/AuroAi/apollo_ros_bridge/blob/master/modules/drivers/gnss/proto/imu.proto).
```sh
message Imu {
  optional apollo.common.Header header = 1;
  optional double measurement_time = 2;  
  optional float measurement_span = 3 [default = 0.0];  
  optional apollo.common.Point3D linear_acceleration = 4;
  optional apollo.common.Point3D angular_velocity = 5;
}
```

## Change configuration in the default.yaml  

In the configuration file [default.yaml](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/yaml_defs/default.yaml), the topic, IMU, must be specified in both of Apollo and ROS sides, to the displayed topic (output topic), the source of the message (input topic), message type and trigger/default type.

```sh
     25   - Input_Topic_Name: /point_cloud_ros
     26       msg_type: sensor_msgs::PointCloud2
     27       trigger_topic: true

     28 + - Apollo:    # convert to cyber topic
     29 + - Output_Topic_Name: /apollo/sensor/gnss/imu_out
     30 +     msg_type: apollo::drivers::gnss::Imu
     31 +     Input_Topics:
     32 +        - Input_Topic_Name: /imu
     33 +           msg_type: sensor_msgs::Imu
     34 +           trigger_topic: true


     55   - Input_Topic_Name: /apollo/sensor/lidar128/compensator/PointCloud2
     56      msg_type: apollo::drivers::PointCloud
     57     trigger_topic: true 
     
          - ROS:   # convert to ROS topic
     58 + - Output_Topic_Name: /imu_ros
     59 +     msg_type: sensor_msgs::Imu
     60 +     Input_Topics:
     61 +      - Input_Topic_Name: /apollo/sensor/gnss/imu
     62 +          msg_type: apollo::drivers::gnss::Imu
     63 +          trigger_topic: true
```

## Declare Apollo's and ROS' IMU data structure 

In file [Data.hpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/lib/Data.hpp): 

```sh
     39   struct ApolloData
     40  {
     41      DataField<apollo::canbus::Chassis> chassis;
     42      DataField<apollo::control::ControlCommand> control_cmd;
     43      DataField<apollo::drivers::PointCloud> point_cloud;
     44 +    DataField<apollo::drivers::gnss::Imu> imu;
     45  };
    
     70  struct ROSData
     71  {    
     72      DataField<nav_msgs::Path> path;
     73      DataField<nav_msgs::Odometry> odom;
     74      DataField<sensor_msgs::PointCloud2> point_cloud;
     75 +    DataField<sensor_msgs::Imu> imu;
     76  };
```

## Insert IMU's topic into the supported list

In file [parse_yaml.hpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/core/parse_yaml.hpp), The new topic should be placed into the topic supported list. The initialization function will extract the appropriate topic in the look-up table.

```sh
     84  std::vector<std::string> supported_ros_topics_list_ = {"nav_msgs::Odometry",
     85                                                         "nav_msgs::Path",
     86 +                                                       "sensor_msgs::Imu",
     87                                                         "sensor_msgs::PointCloud2"};

     90  std::vector<std::string> supported_cyber_topics_list_ = {"apollo::localization::LocalizationEstimate",
     91                                                      "apollo::control::ControlCommand",
     92                                                      "apollo::drivers::PointCloud",
     93 +                                                  "apollo::drivers::gnss::Imu",
     94                                                   "apollo::canbus::Chassis",
     95                                                  "apollo::planning::ADCTrajectory"};
```

## Declare IMU's subscribers and readers

In file [cyber_ros_bridge_core.hpp](<https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/core/cyber_ros_bridge_core.hpp):

```sh
68      // ROS subscriber objects;
69      ros::Subscriber ros_pc_sub_;
70 +    ros::Subscriber ros_imu_sub_;

72      // ROS Publisher objects
73      ros::Publisher point_cloud_pub_;
74      ros::Publisher nav_path_pub_;
75      ros::Publisher localization_pub_;
76 +    ros::Publisher imu_pub_;

78      // Apollo reader objects
79      std::shared_ptr<apollo::cyber::Reader<apollo::drivers::PointCloud>> apollo_point_cloud_reader_;
80      std::shared_ptr<apollo::cyber::Reader<apollo::localization::LocalizationEstimate>> apollo_localization_reader_;
81      std::shared_ptr<apollo::cyber::Reader<apollo::planning::ADCTrajectory>> apollo_trajectory_reader_;
82 +    std::shared_ptr<apollo::cyber::Reader<apollo::drivers::gnss::Imu>> apollo_imu_reader_;

84      // Apollo writer objects
85      std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> pc_writer_;
86 +    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::Imu>> imu_writer_;
```

## Register IMU's trigger subscriber

In file [cyber_ros_bridge.cpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/core/cyber_ros_bridge_core.cpp), register subscribers and set up the callback functions:

```sh
130      {
131        ros_pc_sub_ = private_ros_node_handle_ptr_->subscribe(topic.topic_name, 1,            &cyber_ros_bridge_core::ROSPCCallback, this);
132      }
133 +    else if (topic.topic_type == "sensor_msgs::Imu")
134 +    {
135 +      ros_imu_sub_ = private_ros_node_handle_ptr_->subscribe(topic.topic_name, 1, &cyber_ros_bridge_core::ROSImuCallback, this);
136 +    }

152      apollo_localization_reader_ = private_cyber_node_handle_ptr_->CreateReader<apollo::localization::LocalizationEstimate>(
153      topic.topic_name, std::bind(&cyber_ros_bridge_core::ApolloLocalizationCallback, this, std::placeholders::_1));
154      }
155 +    else if (topic.topic_type == "apollo::drivers::gnss::Imu") 
156 +    {
157 +      apollo_imu_reader_ = private_cyber_node_handle_ptr_->CreateReader<apollo::drivers::gnss::Imu>(topic.topic_name, std::bind(&cyber_ros_bridge_core::ApolloImuCallback, this, std::placeholders::_1));
158 +    }
```

## Register IMU Publishers
In file [cyber_ros_bridge_core.cpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/core/cyber_ros_bridge_core.cpp):

```sh
176      {
177        point_cloud_pub_ = private_ros_node_handle_ptr_->advertise<sensor_msgs::PointCloud2>(topic.topic_name, 1);
178      }
179 +    else if (topic.topic_type == "sensor_msgs::Imu") {
180 +      imu_pub_ = private_ros_node_handle_ptr_->advertise<sensor_msgs::Imu>(topic.topic_name, 1);
181 +    }

189      {
190        pc_writer_ = private_cyber_node_handle_ptr_->CreateWriter<apollo::drivers::PointCloud>(topic.topic_name);
191      }
192 +    else if (topic.topic_type == "apollo::drivers::gnss::Imu") 
193 +    {
194 +      imu_writer_ = private_cyber_node_handle_ptr_->CreateWriter<apollo::drivers::gnss::Imu>(topic.topic_name);
195 +    }
```

## Implement call cack functions

In file [cyber_ros_bridge_core.cpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/core/cyber_ros_bridge_core.cpp), implement the callback functions, receiving the message from the appropriate channels and publishing the corresponding message.

```sh
297 +    void cyber_ros_bridge_core::ROSImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
298 +    {
299 +        try
300 +        {
301 +            // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
302 +            std::lock_guard<std::mutex> lck(ros_data_.imu.mutex);
303 +            // update ROSData struct
304 +            ros_data_.imu.data = *msg;
305 +
306 +            apollo::drivers::gnss::Imu imu_apollo;
307 +
308 +            // call library which converts to cyber message
309 +            ROSImuToApolloImu(msg, imu_apollo);
310 +
311 +            apollo_data_.imu.data = imu_apollo;
312 +            // AINFO << chassis_apollo->DebugString();
313 +            // publish cyber message
314 +            imu_writer_->Write(apollo_data_.imu.data);
315 +        }
316 +        catch (const std::exception &e)
317 +        {
318 +            AERROR << e.what() << '\n';
319 +        }
320 +    }
321 +         
322 +    void cyber_ros_bridge_core::ApolloImuCallback(const std::shared_ptr<apollo::drivers::gnss::Imu> &msg)
323 +    {
324 +        try
325 +        {
326 +            // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
327 +            std::lock_guard<std::mutex> lck(apollo_data_.imu.mutex);
328 +            // update CyberData struct
329 +            apollo_data_.imu.data = *msg;
330 +            // call library which converts to ros message
331 +            sensor_msgs::Imu imu;
332 +            ApolloImuToROSImu(msg, imu);
333 +
334 +            ros_data_.imu.data = imu;
335 +            // publish ROS message
336 +            imu_pub_.publish(imu);
337 +        }
338 +        catch (const std::exception &e)
339 +        {
340 +            AERROR << e.what() << '\n';
341 +        }
342 +    }
```

## Add IMU's conversion fucntions

In file [module_driver.cpp](https://github.com/AuroAi/apollo_ros_bridge/blob/master/cyber_ros_bridge/lib/module_drivers.cpp), implement the convrsion functions by assigning each field of the message from apollo to ROS or the opposite.
  
```sh
192      // AINFO << pc_apollo->DebugString();
193
194      }
195
196 +    void cyber_ros_bridge::ApolloImuToROSImu(const std::shared_ptr<apollo::drivers::gnss::Imu> &imu_msg,sensor_msgs::Imu &imu_ros)
197 +    {
198 +        typedef uint8_t byte;
199 +
200 +        //convert headers
201 +        auto header = imu_msg->header();
202 +        imu_ros.header.frame_id = FLAGS_world_frame;
203 +        imu_ros.header.seq = header.sequence_num();
204 +        imu_ros.header.stamp.sec = header.timestamp_sec();
205 +        //imu_ros.header.stamp = ros::Time::now();
206 +
207 +        // convert linear acceleration
208 +        imu_ros.linear_acceleration.x = imu_msg->linear_acceleration().x();
209 +        imu_ros.linear_acceleration.y = imu_msg->linear_acceleration().y();
210 +        imu_ros.linear_acceleration.z = imu_msg->linear_acceleration().z();
211 +
212 +        // convert angular velocity
213 +        imu_ros.angular_velocity.x = imu_msg->angular_velocity().x();
214 +        imu_ros.angular_velocity.y = imu_msg->angular_velocity().y();
215 +        imu_ros.angular_velocity.z = imu_msg->angular_velocity().z();
216 +
217 +        AINFO << "Received message  " << imu_msg->DebugString();
218 +   }
219 +    
220 +    void cyber_ros_bridge::ROSImuToApolloImu(const sensor_msgs::Imu::ConstPtr &imu_msg, apollo::drivers::gnss::Imu &imu_apollo)
221 +    {
222 +        // initialize the header
223 +        auto header = imu_apollo.mutable_header();
224 +
225 +        // set headers, width and height
226 +        header->set_timestamp_sec(imu_msg->header.stamp.toSec());
227 +        header->set_frame_id(imu_msg->header.frame_id);
228 +        header->set_sequence_num(imu_msg->header.seq);
229 +        
230+         imu_apollo.mutable_linear_acceleration()->set_x(imu_msg->linear_acceleration.x);
231 +        imu_apollo.mutable_linear_acceleration()->set_y(imu_msg->linear_acceleration.y);
232 +        imu_apollo.mutable_linear_acceleration()->set_z(imu_msg->linear_acceleration.z);
233 +        
234 +        imu_apollo.mutable_angular_velocity()->set_x(imu_msg->angular_velocity.x);
235 +        imu_apollo.mutable_angular_velocity()->set_y(imu_msg->angular_velocity.y);
236 +        imu_apollo.mutable_angular_velocity()->set_z(imu_msg->angular_velocity.z);
237 +        // AINFO << pc_apollo->DebugString();
238 +        }
```
