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
 * Desc: testing library class which parses the yaml

 **************************************************************************/
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "cyber/cyber.h"
#include "cyber_ros_bridge/core/parse_config.hpp"

// using namespace cyber_ros_bridge;

int main(int argc, char** argv)
{
  std::string file_location = "/apollo/cyber_ros_bridge/conf/generated.yaml";
  cyber_ros_bridge::ParseConfig parser(file_location);
  return 0;
}
