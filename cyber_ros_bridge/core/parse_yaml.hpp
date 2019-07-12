

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
 * Desc: Yaml config file parser 

 **************************************************************************/

#ifndef _PARSE_CONFIG_H_
#define _PARSE_CONFIG_H_
#include <yaml-cpp/yaml.h>
#include "cyber/cyber.h"
// write a class for parsing the yaml file

// debug mode
// #define DEBUG_MODE

using namespace std;
/** @brief Namespace for apollo ros bridge */
namespace cyber_ros_bridge
{
// structure which represtents a topic
struct Topic
{
  std::string topic_name;
  std::string topic_type;
  bool use;        // is the topic used
  bool is_trigger; // is this topic trigerring publishing
  double pub_frequency;
  std::string topic_flow_dir; // represents if it is an input topic or an output topic

  Topic()
  {
    use = false;
    is_trigger = false;
  }
};

// predictate to find if the topic exists in the topic list
struct find_topic
{
  std::string topic_name;
  find_topic(std::string name) : topic_name(name)
  {
  }
  bool operator()(const Topic &topic)
  {
    return topic.topic_name == topic_name;
  }
};

class ParseConfig
{
private:
  /* data */
public:
  ParseConfig(const std::string &file_name);
  // ~ParseConfig(){}

  // supported list of topics

  // mode (timer based or trigger based)
  // default = trigger based
  std::string mode_ = "Trigger";

  //  TODO:: This can be made a param ???

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

  //  list of all topic types
  std::vector<Topic> ros_topics_;
  std::vector<Topic> cyber_topics_;

  //*******functions********

  // fill topic list
  void FillTopicList(const string &topic_name, const string &topic_type, const string &flow_direction,
                     const bool trigger_value = false, const double frequency = 10);

  // fill topic structure
  //  set default value for trigger value as false
  void FillTopicStruct(const std::string &topic_name, const std::string &topic_type, const bool &trigger_value,
                       const double &frequency, std::vector<Topic> &topics, const std::string &flow_direction);
};
} // namespace cyber_ros_bridge

#endif /* _PARSE_CONFIG_H_ */
