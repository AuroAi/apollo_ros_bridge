
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
 * Desc: header file for conversion lib

 **************************************************************************/
#ifndef _cyber_ros_bridge_LIB_H_
#define _cyber_ros_bridge_LIB_H_

//-------------- Common C/C++ includes --------------

//-------------- ROS libs includes --------------
#include <tf/transform_datatypes.h>
#include "cyber/cyber.h"
#include "ros/include/ros/ros.h"
// #include "LinearMath/btMatrix3x3.h"

//-------------- ROS msgs includes --------------

//-------------- External libs includes (Eigen) --------------
#include "Data.hpp"

// gflags
#include "cyber_ros_bridge/common/bridge_gflags.h"

// write a class for parsing the yaml file
using namespace std;

#define DEBUG_MODE

namespace cyber_ros_bridge
{

//converting apollo trajectory to ROS nav_msgs::Path
void ApolloTrajectoryToROSPath(const std::shared_ptr<apollo::planning::ADCTrajectory> &trajectory_apollo, nav_msgs::Path &path_cmd);

//converting apollo localization to ROS nav_msgs::Odometry
void ApolloLocalizationToROSOdom(const std::shared_ptr<apollo::localization::LocalizationEstimate> &localization_apollo, nav_msgs::Odometry &odom);

//converting apollo point cloud to ros point cloud
void ApolloPCToROSPC(const std::shared_ptr<apollo::drivers::PointCloud> &pc_msg,sensor_msgs::PointCloud2 &pc_ros);

//converting ros point cloud to apollo point cloud
void ROSPCToApolloPc(const sensor_msgs::PointCloud2::ConstPtr &pc_msg, std::shared_ptr<apollo::drivers::PointCloud> &pc_apollo);
} // namespace cyber_ros_bridge

#endif /* _cyber_ros_bridge_LIB_H_ */
