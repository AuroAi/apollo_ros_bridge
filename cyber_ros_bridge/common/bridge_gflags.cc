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
 * Desc: flags for the bridge

 **************************************************************************/
#include "bridge_gflags.h"

DEFINE_string(config_yaml_file, "/home/apollo_ros_bridge/cyber_ros_bridge/yaml_defs/default.yaml", "path to yaml config file");
DEFINE_string(world_frame, "world", "world frame");