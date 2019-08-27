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
 * Desc: File to hold data

 **************************************************************************/

#ifndef _DATA_H_
#define _DATA_H_
//-------------- Common C/C++ includes --------------
#include <mutex>
//-------------- ROS libs includes --------------
// #include "LinearMath/btMatrix3x3.h"

//-------------- ROS msgs includes --------------

//-------------- External libs includes (Eigen) --------------
#include "ros_pkgs/include/nav_msgs/Odometry.h"
#include "ros_pkgs/include/nav_msgs/Path.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include "modules/control/proto/control_cmd.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"


// write a class for parsing the yaml file

namespace cyber_ros_bridge
{
  template <typename T>
  struct DataField
  {
    T data;
    std::mutex mutex;
  };

  struct ApolloData
  {
    // extensive list of data to and from apollo
    DataField<apollo::localization::LocalizationEstimate> localization;
    DataField<apollo::planning::ADCTrajectory> adctrajectory;
    DataField<apollo::canbus::Chassis> chassis;
    DataField<apollo::control::ControlCommand> control_cmd;
    DataField<apollo::drivers::PointCloud> point_cloud;
    DataField<apollo::drivers::gnss::Imu> imu;
  };

  struct ROSData
  {
    // extensive list of data to and from ROS
    DataField<nav_msgs::Path> path;
    DataField<nav_msgs::Odometry> odom;
    DataField<sensor_msgs::PointCloud2> point_cloud;
    DataField<sensor_msgs::Imu> imu;
  };
}  // namespace cyber_ros_bridge

#endif /* _DATA_H_ */
