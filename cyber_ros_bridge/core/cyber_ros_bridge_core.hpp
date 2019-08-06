
/******************************************************************************
 * Copyright 2019 Ridecell . All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**************************************************************************
 * Desc: apollo ros core 

 **************************************************************************/

#ifndef _Utils_H_
#define _Utils_H_

//-------------- Common C/C++ includes --------------

//-------------- ROS libs includes --------------
#include "cyber/cyber.h"
#include "ros/include/ros/ros.h"
// #include "LinearMath/btMatrix3x3.h"

//-------------- ROS msgs includes --------------

//-------------- External libs includes (Eigen) --------------
#include "apollo_ros_bridge/cyber_ros_bridge/core/parse_yaml.hpp"
#include "apollo_ros_bridge/cyber_ros_bridge/lib/cyber_ros_bridge_lib.hpp"

// write a class for parsing the yaml file
using namespace std;

#define DEBUG_MODE

namespace cyber_ros_bridge
{
class cyber_ros_bridge_core
{
private:
  // node handles
  ros::NodeHandlePtr public_ros_node_handle_ptr_; // ros handle
  ros::NodeHandlePtr private_ros_node_handle_ptr_;
  std::unique_ptr<apollo::cyber::Node> private_cyber_node_handle_ptr_;

  // members
  // data struct for data from apollo and ros
  ApolloData apollo_data_;
  ROSData ros_data_;

  // mode trigger or timer based
  string mode_ = "Trigger";

  // supported list of topics
  std::vector<string> supported_ros_topics_list_;
  std::vector<string> supported_cyber_topics_list_;

  /*********** Subscriber and Publisher variables ************/

  // ROS subscriber objects;
  ros::Subscriber ros_pc_sub_;

  // ROS Publisher objects
  ros::Publisher point_cloud_pub_;
  ros::Publisher nav_path_pub_;
  ros::Publisher localization_pub_;

  // Apollo reader objects
  std::shared_ptr<apollo::cyber::Reader<apollo::drivers::PointCloud>> apollo_point_cloud_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::localization::LocalizationEstimate>> apollo_localization_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::planning::ADCTrajectory>> apollo_trajectory_reader_;

  // Apollo writer objects
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> pc_writer_;

  // FLAGS
  double FLAGS_buffer_size_ = 1;

  //  template function for default non trigger type ROS topic callback function
  template <typename ROSMessageType>
  void ROScallback(const typename ROSMessageType::ConstPtr &msg, DataField<ROSMessageType> &field)
  {
    field.mutex.lock();
    field.data = *msg;
    field.mutex.unlock();
#ifdef DEBUG_MODE
    ROS_INFO_STREAM("received message" << msg->header);
#endif
  }

  //  template function for default non trigger type Apollo (cyber) topic callback function
  template <typename ApolloMessageType>
  void ApolloCallBack(const std::shared_ptr<ApolloMessageType> &msg, DataField<ApolloMessageType> &field)
  {
    field.mutex.lock();
    field.data = *msg;
    field.mutex.unlock();
#ifdef DEBUG_MODE
    AINFO << "received message" << msg->DebugString();
#endif
  }

  // Initialize subscribers
  void InitializeSubscribers(const std::vector<Topic> &topics);

  // register default subscribers. All subscribers are default subscribers
  void RegisterDefaultSubscribers(const Topic &topic);

  // register trigger subscribers. Trigger subscribers are the subscribers for the topic with the highest frequency
  void RegisterTriggerSubscribers(const Topic &topic);

  // register all publishers and writers
  void RegisterPublishers(const Topic &topic);

  // specific callbacks for trigger topics
  void ApolloTrajectoryCallback(const std::shared_ptr<apollo::planning::ADCTrajectory> &msg);
  void ApolloPCCallback(const std::shared_ptr<apollo::drivers::PointCloud> &msg);
  void ApolloLocalizationCallback(const std::shared_ptr<apollo::localization::LocalizationEstimate> &msg);
  void ROSPCCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

public:
  // register subscribers and publishers
  cyber_ros_bridge_core(const ros::NodeHandlePtr &public_nh, const ros::NodeHandlePtr &private_nh,
                         std::unique_ptr<apollo::cyber::Node> &cyber_nh, const std::vector<Topic> &topic,
                         const string &mode, const std::vector<string> &supported_ros_topics_list,
                         const std::vector<string> &supported_cyber_topics_list, int &argc, char **argv);
  ~cyber_ros_bridge_core(){};
};

} // namespace cyber_ros_bridge

#endif /* _Utils_H_ */
