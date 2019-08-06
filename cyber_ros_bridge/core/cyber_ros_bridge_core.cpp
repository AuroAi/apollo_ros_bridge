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
#include "apollo_ros_bridge/cyber_ros_bridge/core/cyber_ros_bridge_core.hpp"

using namespace cyber_ros_bridge;

cyber_ros_bridge_core::cyber_ros_bridge_core(const ros::NodeHandlePtr &public_nh, const ros::NodeHandlePtr &private_nh,
                                               std::unique_ptr<apollo::cyber::Node> &cyber_nh,
                                               const std::vector<Topic> &topic, const string &mode,
                                               const std::vector<string> &supported_ros_topics_list,
                                               const std::vector<string> &supported_cyber_topics_list, int &argc,
                                               char **argv)
    : public_ros_node_handle_ptr_(std::move(public_nh)), private_ros_node_handle_ptr_(std::move(private_nh)), private_cyber_node_handle_ptr_(std::move(cyber_nh)), mode_(mode)
{
  //parse flags
  google::ParseCommandLineFlags(&argc, &argv, true);

  //   set supported topics list
  supported_ros_topics_list_ = supported_ros_topics_list;
  supported_cyber_topics_list_ = supported_cyber_topics_list;

  // initialize ros::subscribers based on entries in topic list
  InitializeSubscribers(topic); // call register subscibers
}

void cyber_ros_bridge_core::InitializeSubscribers(const std::vector<Topic> &topics)
{
  // iterate through each entry in topic and call that subscriber based on mode
  // if topic is trigger topic, call specific library function which performs the conversion

  AINFO << "initializing subscribers";
  for (const auto &topic : topics)
  {
    if (topic.topic_flow_dir == "Input")
    {
      AINFO << "topic type = " << topic.topic_type;
      if (mode_ == "Trigger")
      {
        // check if topic is trigger or not
        if (topic.is_trigger)
        {
          // call custom callback function based on topic type
          RegisterTriggerSubscribers(topic);
        }
        else
        {
          RegisterDefaultSubscribers(topic);
        }
      }
      else if (mode_ == "Timer")
      {
      }
      else
      {
        // throw an exception
        throw std::runtime_error("cyber_ros_bridge_core::InitializeSubscribers() UNSUPPORTED MODE. Supported "
                                 "mdoes "
                                 "are trigger and timer");
      }
    }
    else
    {
      RegisterPublishers(topic);
    }
  }
}

void cyber_ros_bridge_core::RegisterDefaultSubscribers(const Topic &topic)
{
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

  else if (topic.topic_type == "apollo::planning::ADCTrajectory")
  {
    apollo_trajectory_reader_ = private_cyber_node_handle_ptr_->CreateReader<apollo::planning::ADCTrajectory>(
        topic.topic_name, boost::bind(&cyber_ros_bridge_core::ApolloCallBack<apollo::planning::ADCTrajectory>, this,
                                      _1, boost::ref(apollo_data_.adctrajectory)));
  }

  else if (topic.topic_type == "apollo::localization::LocalizationEstimate")
  {
    apollo_localization_reader_ = private_cyber_node_handle_ptr_->CreateReader<apollo::localization::LocalizationEstimate>(
        topic.topic_name, boost::bind(&cyber_ros_bridge_core::ApolloCallBack<apollo::localization::LocalizationEstimate>, this,
                                      _1, boost::ref(apollo_data_.localization)));
  }
  else
  {
    AERROR << topic.topic_type;
    throw std::runtime_error("cyber_ros_bridge_core::RegisterDefaultSubscribers() Default Subscriber is not registered for this topic type = " + topic.topic_type + ". Create the "
                                                                                                                                                                     "topic functionality explicitly");
  }
}

void cyber_ros_bridge_core::RegisterTriggerSubscribers(const Topic &topic)
{
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
  else if (topic.topic_type == "apollo::drivers::PointCloud")
  {
    apollo_point_cloud_reader_ = private_cyber_node_handle_ptr_->CreateReader<apollo::drivers::PointCloud>(
        topic.topic_name, std::bind(&cyber_ros_bridge_core::ApolloPCCallback, this, std::placeholders::_1));
  }
  else if (topic.topic_type == "apollo::localization::LocalizationEstimate")
  {
    apollo_localization_reader_ = private_cyber_node_handle_ptr_->CreateReader<apollo::localization::LocalizationEstimate>(
        topic.topic_name, std::bind(&cyber_ros_bridge_core::ApolloLocalizationCallback, this, std::placeholders::_1));
  }

  else
  {
    AERROR << topic.topic_type;
    throw std::runtime_error("cyber_ros_bridge_core::RegisterTriggerSubscribers() Trigger Subscriber is not registered for this topic type. = " + topic.topic_type + ". Create the "
                                                                                                                                                                      "topic functionality explicitly");
  }
}

void cyber_ros_bridge_core::RegisterPublishers(const Topic &topic)
{

  if (topic.topic_type == "nav_msgs::Path")
  {
    nav_path_pub_ = private_ros_node_handle_ptr_->advertise<nav_msgs::Path>(topic.topic_name, 1);
  }
  else if (topic.topic_type == "sensor_msgs::PointCloud2")
  {
    point_cloud_pub_ = private_ros_node_handle_ptr_->advertise<sensor_msgs::PointCloud2>(topic.topic_name, 1);
  }
  else if (topic.topic_type == "nav_msgs::Odometry")
  {
    localization_pub_ = private_ros_node_handle_ptr_->advertise<nav_msgs::Odometry>(topic.topic_name, 1);
  }

  else if (topic.topic_type == "apollo::drivers::PointCloud")
  {
    pc_writer_ = private_cyber_node_handle_ptr_->CreateWriter<apollo::drivers::PointCloud>(topic.topic_name);
  }

  else
  {
    AERROR << topic.topic_type;
    throw std::runtime_error("cyber_ros_bridge_core::RegisterPUBLISHERS() Publisher is not registered for this topic type. Create the "
                             "topic functionality explicitly");
  }
}

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

void cyber_ros_bridge_core::ApolloLocalizationCallback(const std::shared_ptr<apollo::localization::LocalizationEstimate> &msg)
{

  try
  {
    // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
    std::lock_guard<std::mutex> lck(apollo_data_.localization.mutex);
    // update CyberData struct
    apollo_data_.localization.data = *msg;
    // call library which converts to ros message
    nav_msgs::Odometry odom;
    ApolloLocalizationToROSOdom(msg, odom);

    ros_data_.odom.data = odom;
    // publish ROS message
    localization_pub_.publish(odom);
  }
  catch (const std::exception &e)
  {
    AERROR << e.what() << '\n';
  }
}

void cyber_ros_bridge_core::ApolloPCCallback(const std::shared_ptr<apollo::drivers::PointCloud> &msg)
{
  try
  {
    // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
    std::lock_guard<std::mutex> lck(apollo_data_.point_cloud.mutex);
    // update CyberData struct
    apollo_data_.point_cloud.data = *msg;
    // call library which converts to ros message
    sensor_msgs::PointCloud2 point_cloud;
    ApolloPCToROSPC(msg, point_cloud);

    ros_data_.point_cloud.data = point_cloud;
    // publish ROS message
    point_cloud_pub_.publish(point_cloud);
  }
  catch (const std::exception &e)
  {
    AERROR << e.what() << '\n';
  }
}

void cyber_ros_bridge_core::ROSPCCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  try
  {
    // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
    std::lock_guard<std::mutex> lck(ros_data_.point_cloud.mutex);
    // update ROSData struct
    ros_data_.point_cloud.data = *msg;

    std::shared_ptr<apollo::drivers::PointCloud> pc_apollo;

    // call library which converts to cyber message
    ROSPCToApolloPc(msg, pc_apollo);

    apollo_data_.point_cloud.data = *pc_apollo;
    // AINFO << chassis_apollo->DebugString();
    // publish cyber message
    pc_writer_->Write(apollo_data_.point_cloud.data);
  }
  catch (const std::exception &e)
  {
    AERROR << e.what() << '\n';
  }
}
