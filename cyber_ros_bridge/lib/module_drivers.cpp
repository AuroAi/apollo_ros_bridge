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
 * Desc: Lib for conversion of navigation messages

 **************************************************************************/
#include "apollo_ros_bridge/cyber_ros_bridge/lib/cyber_ros_bridge_lib.hpp"

void cyber_ros_bridge::ApolloPCToROSPC(const std::shared_ptr<apollo::drivers::PointCloud> &pc_msg,sensor_msgs::PointCloud2 &pc_ros)
{
    typedef uint8_t byte;

    //convert headers
    auto header = pc_msg->header();
    pc_ros.header.frame_id = pc_msg->frame_id();
    pc_ros.header.seq = header.sequence_num();

    // pushing back fields
    // x
    sensor_msgs::PointField field;
    field.name = "x";
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    field.offset = 0;
    pc_ros.fields.push_back(field);
    // y
    field.name = "y";
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    field.offset = 4;
    pc_ros.fields.push_back(field);
    // z
    field.name = "z";
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    field.offset = 8;
    pc_ros.fields.push_back(field);
    // intensity
    field.name = "intensity";
    field.datatype = sensor_msgs::PointField::UINT32;
    field.count = 1;
    field.offset = 12;
    pc_ros.fields.push_back(field);
    // time stamp
    field.name = "timestamp";
    field.datatype = sensor_msgs::PointField::UINT32;
    field.count = 1;
    field.offset = 16;
    pc_ros.fields.push_back(field);

    pc_ros.point_step = 20;
    pc_ros.is_bigendian = false;

    int point_count = 0;

    for (auto point_apollo : pc_msg->point())
    {
        //  convert to uint8_t for pushing to data of point clouds
        float x = point_apollo.x();
        byte *b = (byte *)&(x);
        pc_ros.data.push_back(b[0]);
        pc_ros.data.push_back(b[1]);
        pc_ros.data.push_back(b[2]);
        pc_ros.data.push_back(b[3]);

        float y = point_apollo.y();
        b = (byte *)&(y);
        pc_ros.data.push_back(b[0]);
        pc_ros.data.push_back(b[1]);
        pc_ros.data.push_back(b[2]);
        pc_ros.data.push_back(b[3]);

        float z = point_apollo.z();
        b = (byte *)&(z);
        pc_ros.data.push_back(b[0]);
        pc_ros.data.push_back(b[1]);
        pc_ros.data.push_back(b[2]);
        pc_ros.data.push_back(b[3]);

        uint32_t intensity = point_apollo.intensity();
        b = (byte *)&(intensity);
        pc_ros.data.push_back(b[0]);
        pc_ros.data.push_back(b[1]);
        pc_ros.data.push_back(b[2]);
        pc_ros.data.push_back(b[3]);

        uint32_t timestamp = static_cast<uint32_t>(point_apollo.timestamp());
        b = (byte *)&(timestamp);
        pc_ros.data.push_back(b[0]);
        pc_ros.data.push_back(b[1]);
        pc_ros.data.push_back(b[2]);
        pc_ros.data.push_back(b[3]);

        pc_ros.header.stamp = ros::Time(static_cast<double>(timestamp));
        point_count++;
    }
    pc_ros.height = 1;
    pc_ros.width = point_count;

    // AINFO << "Received message  " << pc_msg->DebugString();
}

void cyber_ros_bridge::ROSPCToApolloPc(const sensor_msgs::PointCloud2::ConstPtr &pc_msg, std::shared_ptr<apollo::drivers::PointCloud> &pc_apollo)
{
    // initialize the header
    auto header = pc_apollo->mutable_header();

    // set headers, width and height
    header->set_timestamp_sec(pc_msg->header.stamp.toSec());
    header->set_frame_id(pc_msg->header.frame_id);
    header->set_sequence_num(pc_msg->header.seq);
    pc_apollo->set_frame_id(pc_msg->header.frame_id);
    pc_apollo->set_measurement_time(pc_msg->header.stamp.toSec());
    pc_apollo->set_width(pc_msg->width);
    pc_apollo->set_height(pc_msg->height);

    // account for possible offsets in point cloud data
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    int stamp_offset = -1;
    int intensity_offset = -1;
    for (const auto &field : pc_msg->fields)
    {
        if (field.name == "x")
        {
            x_offset = field.offset;
        }
        else if (field.name == "y")
        {
            y_offset = field.offset;
        }
        else if (field.name == "z")
        {
            z_offset = field.offset;
        }
        else if (field.name == "ring")
        {
            stamp_offset = field.offset;
        }
        else if (field.name == "intensity")
        {
            intensity_offset = field.offset;
        }
    }

    // if offset fields are not presetnt, we print an error

    if (x_offset == -1 || y_offset == -1 || z_offset == -1 ||
        stamp_offset == -1 || intensity_offset == -1)
    {
        std::cerr << "Field not contains x, y, z, timestamp, instensity"
                  << std::endl;
        return;
    }

    // total number of elements
    int total = pc_msg->width * pc_msg->height;
    // get data
    auto data = pc_msg->data;
    // iterate through each element and append them
    for (int i = 0; i < total; ++i)
    {
        auto cyber_point = pc_apollo->add_point();
        int offset = i * pc_msg->point_step;
        cyber_point->set_x(*reinterpret_cast<float *>(&data[offset + x_offset]));
        cyber_point->set_y(*reinterpret_cast<float *>(&data[offset + y_offset]));
        cyber_point->set_z(*reinterpret_cast<float *>(&data[offset + z_offset]));
        cyber_point->set_intensity(
            *reinterpret_cast<uint8_t *>(&data[offset + intensity_offset]));
        cyber_point->set_timestamp(static_cast<std::uint64_t>(
            *reinterpret_cast<double *>(&data[offset + stamp_offset]) * 1e9));
    }
    // AINFO << pc_apollo->DebugString();

}
