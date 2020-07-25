// Copyright 2020 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the tensorrt_yolo3_node class.

#ifndef TENSORRT_YOLO3__TENSORRT_YOLO3_NODE_HPP_
#define TENSORRT_YOLO3__TENSORRT_YOLO3_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp>

#include <tensorrt_yolo3/visibility_control.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "TrtNet.h"
#include "data_reader.h"

namespace tensorrt_yolo3
{
/// \class TensorrtYolo3Node
/// \brief ROS 2 Node for tensorrt yolo3.
class TENSORRT_YOLO3_NODE_PUBLIC TensorrtYolo3Node : public ::rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \param[in] node_name name of the node for rclcpp scription
  /// \throw runtime error if failed to start threads or configure driver
  explicit TensorrtYolo3Node(const rclcpp::NodeOptions & options);

private:
  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr objects_pub_ptr_;

  std::unique_ptr<Tn::trtNet> net_ptr_;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & in_image);
  std::vector<float> prepareImage(cv::Mat & in_img);
  std::vector<Tn::Bbox> postProcessImg(
    std::vector<Yolo::Detection> & detections, const int classes, cv::Mat & img,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & out_objects);
  void doNms(std::vector<Yolo::Detection> & detections, int classes, float nmsThresh);
};
}  // namespace tensorrt_yolo3

#endif  // TENSORRT_YOLO3__TENSORRT_YOLO3_NODE_HPP_
