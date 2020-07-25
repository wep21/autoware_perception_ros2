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

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <lidar_apollo_instance_segmentation/lidar_apollo_instance_segmentation_node.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  rclcpp::executors::SingleThreadedExecutor exec;

  auto node = std::make_shared<lidar_apollo_instance_segmentation::LidarInstanceSegmentationNode>(options);
  node->initialize();
  exec.add_node(node);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
