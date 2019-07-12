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

#include "cyber_ros_bridge/core/parse_yaml.hpp"

using namespace cyber_ros_bridge;
using namespace std;

ParseConfig::ParseConfig(const string &file_name)
{
  double frequency;
  bool trigger_value = false;
  string topic_name, type, flow_direction;
  // use yaml cpp to parse the file
  // iterate through each key in the yaml file
  // the yaml file format

  // refer to sample.yaml
  //  get mode (trigger based or timer based)

  //  For each topic in input topic
  //  For each topic in output topic
  //  call function set_topic_status
  // check if either trigger value or frequency is specified

  YAML::Node main_node = YAML::LoadFile(file_name);

  //  AINFO<<file_location;
  mode_ = main_node[0]["mode"].as<std::string>();
  if (mode_ != "Trigger" && mode_ != "Timer")
  {
    throw std::runtime_error("Wrong mode. Supported modes are = Trigger and Timer");
  }
  if (mode_ == "Timer")
  {
    throw std::runtime_error("Timer mode will be supported in the future");
  }
  // now we parse the topics in apollo
  YAML::Node node = main_node[1]["Apollo"];
  YAML::Node ros_node = main_node[2]["ROS"];

#ifdef DEBUG_MODE
  AINFO << main_node;
#endif

  for (const auto &individual_node : ros_node)
  {
    node.push_back(individual_node);
  }

  for (const auto &output_topic : node)
  {
    // parse the output topic
    topic_name = output_topic["Output_Topic_Name"].as<std::string>();
    type = output_topic["msg_type"].as<std::string>();
    if (mode_ == "Timer")
    {
      if (output_topic["publish_freq"])
      {
        flow_direction = "Output";
        frequency = output_topic["publish_freq"].as<double>();
        FillTopicList(topic_name, type, flow_direction, frequency);
      }
      else
      {
        throw std::runtime_error("Publish frequency has to be specified for output topic when using mode Timer");
      }
    }
    // Trigger
    else
    {
      flow_direction = "Output";
      FillTopicList(topic_name, type, flow_direction);
    }

    // parse input topics within the output topics
    for (const auto &input_topic : output_topic["Input_Topics"])
    {
      // parse the input topic
      topic_name = input_topic["Input_Topic_Name"].as<string>();
      type = input_topic["msg_type"].as<string>();
      if (mode_ == "Trigger")
      {
        if (input_topic["trigger_topic"])
        {
          flow_direction = "Input";
          trigger_value = input_topic["trigger_topic"].as<bool>();
        }
        else
        {
          throw std::runtime_error("Trigger value has to be specified for Input topic when using mode Trigger");
        }
        FillTopicList(topic_name, type, flow_direction, trigger_value);
      }
    }
  }

  auto all_topics = cyber_topics_;
  all_topics.insert(all_topics.end(), ros_topics_.begin(), ros_topics_.end());

#ifdef DEBUG_MODE
  for (const auto &topic : all_topics)
  {
    AINFO << endl
          << topic.topic_name << endl
          << topic.topic_type << endl
          << topic.is_trigger << endl
          << topic.topic_flow_dir;
  }
#endif
}

void ParseConfig::FillTopicList(const string &topic_name, const string &topic_type, const string &flow_direction,
                                const bool trigger_value, const double frequency)
{
  if (std::find(supported_ros_topics_list_.begin(), supported_ros_topics_list_.end(), topic_type) != supported_ros_topics_list_.end())
  {
    FillTopicStruct(topic_name, topic_type, trigger_value, frequency, ros_topics_, flow_direction);
  }

  else if (std::find(supported_cyber_topics_list_.begin(), supported_cyber_topics_list_.end(), topic_type) != supported_cyber_topics_list_.end())
  {
    FillTopicStruct(topic_name, topic_type, trigger_value, frequency, cyber_topics_, flow_direction);
  }
  else
  {
    AERROR << "Topic type = " << topic_type;
    throw std::runtime_error("UNSUPPORTED TOPIC TYPE. Create the topic functionality explicitly");
  }
}

void ParseConfig::FillTopicStruct(const std::string &topic_name, const std::string &topic_type,
                                  const bool &trigger_value, const double &frequency, std::vector<Topic> &topics,
                                  const std::string &flow_direction)
{
  // check if the incoming topic is already present in the list
  // if yes
  // if incoming topic is trigger topic, overwrite the topic trigger field to true
  // if incoming topic is not trigger topic, do nothing
  // if no, add to topic list
  if (mode_ == "Trigger")
  {
    auto it = std::find_if(topics.begin(), topics.end(), find_topic(topic_name));
    if (it != topics.end())
    {
      if (trigger_value)
      {
        it->is_trigger = true;
      }
      else
      {
        // do nothing and continue
      }
    }
    else
    {
      Topic topic;
      topic.topic_name = topic_name;
      topic.topic_type = topic_type;
      topic.use = true;
      if (flow_direction == "Input")
      {
        topic.is_trigger = trigger_value;
      }
      topic.topic_flow_dir = flow_direction;
      topics.push_back(topic);
    }
  }
  //To be implemented DO NOT REMOVE
  else if (mode_ == "Timer")
  {
    auto it = std::find_if(topics.begin(), topics.end(), find_topic(topic_name));
    if (it != topics.end())
    {
      // do nothing and continue
    }
    else
    {
      Topic topic;
      topic.topic_name = topic_name;
      topic.topic_type = topic_type;
      topic.use = true;
      if (flow_direction == "Input")
      {
        topic.pub_frequency = frequency;
      }
      topic.topic_flow_dir = flow_direction;
      topics.push_back(topic);
    }
  }
}