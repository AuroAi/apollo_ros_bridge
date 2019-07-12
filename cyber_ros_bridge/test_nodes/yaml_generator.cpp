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
 * Desc: generating sample yaml file

 **************************************************************************/

#include <yaml-cpp/yaml.h>
#include <fstream>

int main(int argc, char** argv)
{
  YAML::Node Main;

  YAML::Node Apollo;

  YAML::Node Mode;

  YAML::Node Output_Topic_list;

  YAML::Node Output_Topic;

  YAML::Node Input_Topic_list;

  YAML::Node Output_Topic_Name;
  Main.push_back(Mode);
  Main.push_back(Apollo);

  Mode["mode"] = "Trigger";
  Apollo["Apollo"] = Output_Topic_list;
  Output_Topic_list.push_back(Output_Topic);

  Output_Topic["Output_Topic_Name"] = "/apollo/localization";
  Output_Topic["msg_type"] = "apollo::localization::LocalizationEstimate";
  Output_Topic["publish_freq"] = 50;

  Output_Topic["Input_Topics"] = Input_Topic_list;

  {
    YAML::Node Input_Topic;
    Input_Topic_list.push_back(Input_Topic);
    Input_Topic["Input_Topic_Name"] = "/odom";
    Input_Topic["msg_type"] = "nav_msgs::Odometry";
    Input_Topic["trigger_topic"] = true;
  }

  {
    YAML::Node Input_Topic;

    Input_Topic_list.push_back(Input_Topic);
    Input_Topic["Input_Topic_Name"] = "/steer_report";
    Input_Topic["msg_type"] = "dbw_mkz_msgs::SteeringReport";
    Input_Topic["trigger_topic"] = false;
  }

  {
    YAML::Node Input_Topic;
    Input_Topic_list.push_back(Input_Topic);
    Input_Topic["Input_Topic_Name"] = "/brake_report";
    Input_Topic["msg_type"] = "dbw_mkz_msgs::BrakeInfoReport";
    Input_Topic["trigger_topic"] = false;
  }

  YAML::Node ROS,ros_output_topic,ros_input_topic_list,ros_Output_Topic_List;
  Main.push_back(ROS);
  ROS["ROS"] = ros_Output_Topic_List;
  ros_Output_Topic_List.push_back(ros_output_topic);

  ros_output_topic["Output_Topic_Name"] = "/control_cmd";
  ros_output_topic["msg_type"] = "dbw_mkz_msgs::SteeringCmd";
  ros_output_topic["publish_freq"] = 50;

  ros_output_topic["Input_Topics"] = ros_input_topic_list;

  {
    YAML::Node Input_Topic;
    ros_input_topic_list.push_back(Input_Topic);
    Input_Topic["Input_Topic_Name"] = "/control_cmd";
    Input_Topic["msg_type"] = "apollo::control::ControlCommand";
    Input_Topic["trigger_topic"] = true;
  }


  std::string file_location = "/apollo/cyber_ros_bridge/conf/config_example.yaml";
  std::ofstream fout(file_location.c_str(), std::ofstream::out);
  std::stringstream ss;
  ss << Main;
  fout << ss.str();
  fout.close();
}
