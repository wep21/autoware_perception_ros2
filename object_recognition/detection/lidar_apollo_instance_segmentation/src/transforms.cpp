/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

namespace pcl_ros
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
transformPointCloud (const std::string &target_frame, const sensor_msgs::msg::PointCloud2 &in,
                     sensor_msgs::msg::PointCloud2 &out, const tf2_ros::Buffer &tf_buffer)
{
  if (in.header.frame_id == target_frame)
  {
    out = in;
    return (true);
  }

  // Get the TF transform
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer.lookupTransform (target_frame, in.header.frame_id, tf2_ros::fromMsg(in.header.stamp));
  }
  catch (tf2::LookupException &e)
  {
    return (false);
  }
  catch (tf2::ExtrapolationException &e)
  {
    return (false);
  }

  transformPointCloud (target_frame, transform.transform, in, out);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
transformPointCloud (const std::string &target_frame, const geometry_msgs::msg::Transform &net_transform,
                     const sensor_msgs::msg::PointCloud2 &in, sensor_msgs::msg::PointCloud2 &out)
{
  if (in.header.frame_id == target_frame)
  {
    out = in;
    return;
  }

  // Get the transformation
  Eigen::Matrix4f transform;
  transformAsMatrix (net_transform, transform);

  transformPointCloud (transform, in, out);

  out.header.frame_id = target_frame;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
transformPointCloud (const Eigen::Matrix4f &transform, const sensor_msgs::msg::PointCloud2 &in,
                     sensor_msgs::msg::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1)
  {
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      in.fields[y_idx].datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      in.fields[z_idx].datatype != sensor_msgs::msg::PointField::FLOAT32)
  {
    return;
  }

  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");

  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i)
  {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;

    bool max_range_point = false;
    int distance_ptr_offset = (dist_idx < 0 ? -1 : (i*in.point_step + in.fields[dist_idx].offset)); // If dist_idx is negative, it must not be used as an index
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2]))
    {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr))  // Invalid point
      {
        pt_out = pt;
      }
      else  // max range point
      {
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    }
    else
    {
      pt_out = transform * pt;
    }

    if (max_range_point)
    {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));


    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1)
  {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i)
    {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
transformAsMatrix (const geometry_msgs::msg::Transform& bt, Eigen::Matrix4f &out_mat)
{
  out_mat = tf2::transformToEigen(bt).matrix().cast<float>();
}

} // namespace pcl_ros
