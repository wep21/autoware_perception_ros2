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

#include "tensorrt_yolo3/tensorrt_yolo3_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace
{
  static constexpr const int CLASS_NUM = 80;
}  // namespace

namespace tensorrt_yolo3
{
using std::placeholders::_1;
TensorrtYolo3Node::TensorrtYolo3Node(const rclcpp::NodeOptions & options)
: Node("tensorrt_yolo3", options)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("tensorrt_yolo3");
  std::string engine_path = package_path + "/data/yolov3_416_fp32.engine";
  std::ifstream fs(engine_path);
  if (fs.is_open()) {
    net_ptr_.reset(new Tn::trtNet(engine_path));
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Could not find %s, try making TensorRT engine from caffemodel and prototxt",
      engine_path.c_str());
    const auto prototxt_file = this->declare_parameter("prototxt_file").get<std::string>();
    const auto caffemodel_file = this->declare_parameter("caffemodel_file").get<std::string>();
    std::string output_node = "yolo-det";
    std::vector<std::string> output_name;
    output_name.push_back(output_node);
    std::vector<std::vector<float>> calib_data;
    Tn::RUN_MODE run_mode = Tn::RUN_MODE::FLOAT32;
    net_ptr_.reset(
      new Tn::trtNet(prototxt_file, caffemodel_file, output_name, calib_data, run_mode));
    net_ptr_->saveEngine(engine_path);
  }

  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  image_sub_ = image_transport::create_subscription(this, "in_image_base_topic", std::bind(&TensorrtYolo3Node::imageCallback, this, _1), "raw", custom_qos);
  image_pub_ = image_transport::create_publisher(this, "out_image_base_topic", custom_qos);
  objects_pub_ptr_ = this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>("out_objects_topic", rclcpp::QoS(10));
}

void TensorrtYolo3Node::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & in_image_msg)
{
  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  int outputCount = net_ptr_->getOutputSize() / sizeof(float);
  std::unique_ptr<float[]> output_data(new float[outputCount]);

  // TODO: if this file is ros interface class, not appropriate to write this method in this file/class
  std::vector<float> input_data = prepareImage(in_image_ptr->image);
  net_ptr_->doInference(input_data.data(), output_data.get());

  // Get Output
  auto output = output_data.get();

  // first detect count
  int count = output[0];
  // later detect result
  std::vector<Yolo::Detection> result;
  result.resize(count);
  memcpy(result.data(), &output[1], count * sizeof(Yolo::Detection));

  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray out_objects;
  out_objects.header = in_image_msg->header;
  // TODO: if this file is ros interface class, not appropriate to write this method in this file/class
  auto bbox = postProcessImg(result, CLASS_NUM, in_image_ptr->image, out_objects);

  for (const auto & item : bbox) {
    cv::rectangle(
      in_image_ptr->image, cv::Point(item.left, item.top), cv::Point(item.right, item.bot),
      cv::Scalar(0, 0, 255), 3, 8, 0);
  }
  image_pub_.publish(in_image_ptr->toImageMsg());

  objects_pub_ptr_->publish(out_objects);
}

std::vector<float> TensorrtYolo3Node::prepareImage(cv::Mat & in_img)
{
  int c = 3;
  int h = 416;
  int w = 416;

  float scale = std::min(float(w) / in_img.cols, float(h) / in_img.rows);
  auto scaleSize = cv::Size(in_img.cols * scale, in_img.rows * scale);

  cv::Mat rgb;
  cv::cvtColor(in_img, rgb, CV_BGR2RGB);
  cv::Mat resized;
  cv::resize(rgb, resized, scaleSize, 0, 0, cv::INTER_CUBIC);

  cv::Mat cropped(h, w, CV_8UC3, 127);
  cv::Rect rect(
    (w - scaleSize.width) / 2, (h - scaleSize.height) / 2, scaleSize.width, scaleSize.height);
  resized.copyTo(cropped(rect));

  cv::Mat img_float;
  cropped.convertTo(img_float, CV_32FC3, 1 / 255.0);

  // HWC TO CHW
  std::vector<cv::Mat> input_channels(c);
  cv::split(img_float, input_channels);

  std::vector<float> result(h * w * c);
  auto data = result.data();
  int channelLength = h * w;
  for (int i = 0; i < c; ++i) {
    memcpy(data, input_channels[i].data, channelLength * sizeof(float));
    data += channelLength;
  }

  return result;
}

std::vector<Tn::Bbox> TensorrtYolo3Node::postProcessImg(
    std::vector<Yolo::Detection> & detections, const int classes, cv::Mat & img,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & out_objects)
{
  int h = 416;
  int w = 416;

  // scale bbox to img
  int width = img.cols;
  int height = img.rows;
  float scale = std::min(float(w) / width, float(h) / height);
  float scaleSize[] = {width * scale, height * scale};

  // correct box
  for (auto & item : detections) {
    auto & bbox = item.bbox;
    bbox[0] = (bbox[0] * w - (w - scaleSize[0]) / 2.f) / scaleSize[0];
    bbox[1] = (bbox[1] * h - (h - scaleSize[1]) / 2.f) / scaleSize[1];
    bbox[2] /= scaleSize[0];
    bbox[3] /= scaleSize[1];
  }

  // nms
  // float nmsThresh = parser::getFloatValue("nms");
  float nms_thresh = 0.45;
  if (nms_thresh > 0) doNms(detections, classes, nms_thresh);

  std::vector<Tn::Bbox> boxes;
  for (const auto & item : detections) {
    auto & b = item.bbox;
    Tn::Bbox bbox = {
      item.classId,                                        // classId
      std::max(int((b[0] - b[2] / 2.) * width), 0),        // left
      std::min(int((b[0] + b[2] / 2.) * width), width),    // right
      std::max(int((b[1] - b[3] / 2.) * height), 0),       // top
      std::min(int((b[1] + b[3] / 2.) * height), height),  // bot
      item.prob                                            // score
    };
    boxes.push_back(bbox);

    autoware_perception_msgs::msg::DynamicObjectWithFeature obj;

    obj.feature.roi.x_offset = std::max(int((b[0] - b[2] / 2.) * width), 0);
    obj.feature.roi.y_offset = std::max(int((b[1] - b[3] / 2.) * height), 0);
    double roi_width = std::min(int((b[0] + b[2] / 2.) * width), width) -
                       std::max(int((b[0] - b[2] / 2.) * width), 0);
    double roi_height = std::min(int((b[1] + b[3] / 2.) * height), height) -
                        std::max(int((b[1] - b[3] / 2.) * height), 0);
    obj.feature.roi.width = roi_width;
    obj.feature.roi.height = roi_height;
    // if (in_objects[i].x < 0)
    //     obj.feature.roi.x_offset = 0;
    // if (in_objects[i].y < 0)
    //     obj.feature.roi.y_offset = 0;
    // if (in_objects[i].w < 0)
    //     obj.feature.roi.width = 0;
    // if (in_objects[i].h < 0)
    //     obj.feature.roi.height = 0;

    obj.object.semantic.confidence = item.prob;
    if (item.classId == 2) {
      obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::CAR;
    } else if (item.classId == 0) {
      obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::PEDESTRIAN;
    } else if (item.classId == 5) {
      obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::BUS;
    } else if (item.classId == 7) {
      obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::TRUCK;
    } else if (item.classId == 1) {
      obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::BICYCLE;
    } else if (item.classId == 3) {
      obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::MOTORBIKE;
    } else {
      obj.object.semantic.type = autoware_perception_msgs::msg::Semantic::UNKNOWN;
    }
    out_objects.feature_objects.push_back(obj);
  }

  return boxes;
}

void TensorrtYolo3Node::doNms(
  std::vector<Yolo::Detection> & detections, int classes, float nms_thresh)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<std::vector<Yolo::Detection>> resClass;
  resClass.resize(classes);

  for (const auto & item : detections) resClass[item.classId].push_back(item);

  auto iouCompute = [](float * lbox, float * rbox) {
    float interBox[] = {
      std::max(lbox[0] - lbox[2] / 2.f, rbox[0] - rbox[2] / 2.f),  // left
      std::min(lbox[0] + lbox[2] / 2.f, rbox[0] + rbox[2] / 2.f),  // right
      std::max(lbox[1] - lbox[3] / 2.f, rbox[1] - rbox[3] / 2.f),  // top
      std::min(lbox[1] + lbox[3] / 2.f, rbox[1] + rbox[3] / 2.f),  // bottom
    };

    if (interBox[2] > interBox[3] || interBox[0] > interBox[1]) return 0.0f;

    float interBoxS = (interBox[1] - interBox[0]) * (interBox[3] - interBox[2]);
    return interBoxS / (lbox[2] * lbox[3] + rbox[2] * rbox[3] - interBoxS);
  };

  std::vector<Yolo::Detection> result;
  for (int i = 0; i < classes; ++i) {
    auto & dets = resClass[i];
    if (dets.size() == 0) continue;

    sort(
      dets.begin(), dets.end(), [=](const Yolo::Detection & left, const Yolo::Detection & right) {
        return left.prob > right.prob;
      });

    for (unsigned int m = 0; m < dets.size(); ++m) {
      auto & item = dets[m];
      result.push_back(item);
      for (unsigned int n = m + 1; n < dets.size(); ++n) {
        if (iouCompute(item.bbox, dets[n].bbox) > nms_thresh) {
          dets.erase(dets.begin() + n);
          --n;
        }
      }
    }
  }

  // swap(detections,result);
  detections = move(result);

  auto t_end = std::chrono::high_resolution_clock::now();
  float total = std::chrono::duration<float, std::milli>(t_end - t_start).count();
  // std::cout << "Time taken for nms is " << total << " ms." << std::endl;
}

}  // namespace tensorrt_yolo3

RCLCPP_COMPONENTS_REGISTER_NODE(tensorrt_yolo3::TensorrtYolo3Node)
