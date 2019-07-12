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
 * Desc: Lib file for conversion of navigation messages

 **************************************************************************/
#include "cyber_ros_bridge/lib/cyber_ros_bridge_lib.hpp"

void cyber_ros_bridge::ApolloTrajectoryToROSPath(const std::shared_ptr<apollo::planning::ADCTrajectory> &trajectory_apollo, nav_msgs::Path &path_cmd)
{
  for (const auto &trajectory_point : trajectory_apollo->trajectory_point())
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = FLAGS_world_frame;
    pose.pose.position.x = trajectory_point.path_point().x();
    pose.pose.position.y = trajectory_point.path_point().y();
    pose.pose.position.z = trajectory_point.path_point().z();

    tf::Quaternion q_tf;
    q_tf.setRPY(0.0, 0.0, trajectory_point.path_point().theta());
    tf::quaternionTFToMsg(q_tf, pose.pose.orientation);

    path_cmd.poses.push_back(pose);
  }

  path_cmd.header.frame_id = FLAGS_world_frame;
  path_cmd.header.stamp = ros::Time::now();
}

//to ros nav msgs odometry from apollo
void cyber_ros_bridge::ApolloLocalizationToROSOdom(const std::shared_ptr<apollo::localization::LocalizationEstimate> &localization_apollo, nav_msgs::Odometry &odom)
{
  //append headers
  odom.header.frame_id = FLAGS_world_frame;
  odom.header.stamp = ros::Time::now();

  //convert pose from apollo to pose in ROS
  //position
  odom.pose.pose.position.x = localization_apollo->pose().position().x();
  odom.pose.pose.position.y = localization_apollo->pose().position().y();
  odom.pose.pose.position.z = localization_apollo->pose().position().z();

  //orientation
  odom.pose.pose.orientation.x = localization_apollo->pose().orientation().qx();
  odom.pose.pose.orientation.y = localization_apollo->pose().orientation().qy();
  odom.pose.pose.orientation.z = localization_apollo->pose().orientation().qz();
  odom.pose.pose.orientation.w = localization_apollo->pose().orientation().qw();

  //converting twists
  //linear
  odom.twist.twist.linear.x = localization_apollo->pose().linear_velocity().x();
  odom.twist.twist.linear.y = localization_apollo->pose().linear_velocity().y();
  odom.twist.twist.linear.z = localization_apollo->pose().linear_velocity().z();
  //angular
  odom.twist.twist.angular.x = localization_apollo->pose().angular_velocity().x();
  odom.twist.twist.angular.y = localization_apollo->pose().angular_velocity().y();
  odom.twist.twist.angular.z = localization_apollo->pose().angular_velocity().z();
}

