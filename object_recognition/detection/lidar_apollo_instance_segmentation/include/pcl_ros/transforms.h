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

#ifndef pcl_ROS_TRANSFORMS_H_
#define pcl_ROS_TRANSFORMS_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/transforms.h>
#include <tf2_ros/buffer.h>

namespace pcl_ros
{
  /** \brief Transform a sensor_msgs::PointCloud2 dataset from its frame to a given TF target frame.
    * \param target_frame the target TF frame
    * \param in the input PointCloud2 dataset
    * \param out the resultant transformed PointCloud2 dataset
    * \param tf_buffer a TF2 buffer object
    */
  bool
  transformPointCloud (const std::string &target_frame,
                       const sensor_msgs::msg::PointCloud2 &in,
                       sensor_msgs::msg::PointCloud2 &out,
                       const tf2_ros::Buffer &tf_buffer);

  /** \brief Transform a sensor_msgs::PointCloud2 dataset from its frame to a given TF target frame.
    * \param target_frame the target TF frame
    * \param net_transform the TF transformer object
    * \param in the input PointCloud2 dataset
    * \param out the resultant transformed PointCloud2 dataset
    */
  void
  transformPointCloud (const std::string &target_frame,
                       const geometry_msgs::msg::Transform &net_transform,
                       const sensor_msgs::msg::PointCloud2 &in,
                       sensor_msgs::msg::PointCloud2 &out);

  /** \brief Transform a sensor_msgs::PointCloud2 dataset using an Eigen 4x4 matrix.
    * \param transform the transformation to use on the points
    * \param in the input PointCloud2 dataset
    * \param out the resultant transformed PointCloud2 dataset
    */
  void
  transformPointCloud (const Eigen::Matrix4f &transform,
                       const sensor_msgs::msg::PointCloud2 &in,
                       sensor_msgs::msg::PointCloud2 &out);

  /** \brief Obtain the transformation matrix from TF into an Eigen form
    * \param bt the TF transformation
    * \param out_mat the Eigen transformation
    */
  void
  transformAsMatrix (const geometry_msgs::msg::Transform& bt, Eigen::Matrix4f &out_mat);
}

#endif // PCL_ROS_TRANSFORMS_H_