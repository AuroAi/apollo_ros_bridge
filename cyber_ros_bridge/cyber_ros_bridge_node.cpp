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
 * Desc: Driver node for apollo ros bridge

 **************************************************************************/
#include "cyber_ros_bridge/core/cyber_ros_bridge_core.hpp"

#include "gflags/gflags.h"

using namespace cyber_ros_bridge;
int main(int argc, char** argv)
{
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "cyber_ros_bridge");
  ros::NodeHandle n;
  ros::NodeHandlePtr publicNodeHandle(new ros::NodeHandle(""));
  ros::NodeHandlePtr privateNodeHandle(new ros::NodeHandle("~"));

  ros::Rate loop_rate(10);
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto cyber_ros_bridge_node = apollo::cyber::CreateNode("cyber_ros_bridge");
  // create listener

  std::string file_location = FLAGS_config_yaml_file;
  cyber_ros_bridge::ParseConfig parser(file_location);

  // catenate topics into a single list
  parser.cyber_topics_.insert(parser.cyber_topics_.end(), parser.ros_topics_.begin(), parser.ros_topics_.end());

  //initialize apollo ros core class object
  cyber_ros_bridge_core core_obj(publicNodeHandle, privateNodeHandle, cyber_ros_bridge_node, parser.cyber_topics_,
                                 parser.mode_, parser.supported_ros_topics_list_, parser.supported_cyber_topics_list_,
                                 argc, argv);
  while (ros::ok() && apollo::cyber::OK())
    {
      ros::spinOnce();
    }
 
  return 0;
}
