/*
 * Copyright 2020 TierIV. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lidar_apollo_instance_segmentation/lidar_apollo_instance_segmentation_node.hpp"
#include "lidar_apollo_instance_segmentation/detector.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_apollo_instance_segmentation
{
LidarInstanceSegmentationNode::LidarInstanceSegmentationNode(const rclcpp::NodeOptions & options)
: Node("lidar_apollo_instance_segmentation_node", options)
{
  pointcloud_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>("input/pointcloud", rclcpp::QoS(10), std::bind(&LidarInstanceSegmentationNode::pointCloudCallback, this, std::placeholders::_1));
  dynamic_objects_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>("output/labeled_clusters", rclcpp::QoS(10));
}

void LidarInstanceSegmentationNode::initialize()
{
  detector_ptr_ = std::make_shared<LidarApolloInstanceSegmentation>(shared_from_this());
  debugger_ptr_ = std::make_shared<Debugger>(shared_from_this());
}

void LidarInstanceSegmentationNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!detector_ptr_ || !debugger_ptr_ ) {
    RCLCPP_INFO(this->get_logger(), "Waiting for initialize...");
    return;
  }
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output_msg;
  detector_ptr_->detectDynamicObjects(*msg, output_msg);
  dynamic_objects_pub_->publish(output_msg);
  debugger_ptr_->publishColoredPointCloud(output_msg);
}
}  // namespace lidar_apollo_instance_segmentation

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_apollo_instance_segmentation::LidarInstanceSegmentationNode)
