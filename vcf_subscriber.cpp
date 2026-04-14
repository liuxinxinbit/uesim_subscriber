// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace vcf_sensor
{

// ======================== 工具函数 ========================

constexpr float MAX_DEPTH = 10.0f;

inline rclcpp::QoS default_qos()
{
  rclcpp::QoS qos(1);
  return qos.reliable().durability_volatile().best_effort();
}

// ======================== RGB相机回调 ========================

void rgb_callback(const sensor_msgs::msg::CompressedImage::SharedPtr & msg, uint64_t & counter)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rgb_callback"), "图像转换失败: %s", e.what());
    return;
  }

  ++counter;
  RCLCPP_INFO(rclcpp::get_logger("rgb_callback"),
              "RGB 图像: %dx%d  序号: %lu",
              cv_ptr->image.cols, cv_ptr->image.rows, counter);

  cv::imshow("RGB Camera", cv_ptr->image);
  cv::waitKey(1);
}

// ======================== 深度相机回调 ========================

void depth_callback(const sensor_msgs::msg::Image::SharedPtr & msg, uint64_t & counter)
{
    cv::Mat depth_image;
    int HEIGHT = 480;
    int WIDTH = 640;
    depth_image = cv::Mat(HEIGHT, WIDTH, CV_32FC1, const_cast<uchar*>(msg->data.data()));
    double min_val = 0.0, max_val = MAX_DEPTH;
    cv::minMaxIdx(depth_image, &min_val, &max_val);
    double range = max_val - min_val;

    cv::Mat normalized;
    if (range > 1e-6f) {
      depth_image.convertTo(normalized, CV_8UC1, 255.0 / range, -min_val * 255.0 / range);
    } else {
      normalized = cv::Mat::zeros(depth_image.size(), CV_8UC1);
    }

    cv::equalizeHist(normalized, normalized);

    cv::Mat color_image;
    cv::applyColorMap(normalized, color_image, cv::COLORMAP_HOT);
    cv::applyColorMap(color_image, color_image, cv::COLORMAP_HOT);
    cv::imshow("Depth Camera", color_image);
    cv::waitKey(1);
}

// ======================== 激光雷达回调 ========================

void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  static uint64_t lidar_counter = 0;
  ++lidar_counter;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *cloud);

  RCLCPP_INFO(rclcpp::get_logger("lidar_callback"),
              "激光雷达点云: %ux%u  点数:%zu  序号:%lu",
              msg->width, msg->height, cloud->size(), lidar_counter);
}

// ======================== 节点类 ========================

class SensorSubscriberNode : public rclcpp::Node
{
public:
  explicit SensorSubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("sensor_subscriber", options)
  {
    auto qos = default_qos();

    sub_rgb_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/front_camera/image/compressed", qos,
      [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        rgb_callback(msg, rgb_counter_);
      });

    sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/front_depth/image", qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        depth_callback(msg, depth_counter_);
      });

    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/front_lidar", qos,
      [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        lidar_callback(msg);
      });

    RCLCPP_INFO(this->get_logger(), "SensorSubscriberNode 已启动");
    RCLCPP_INFO(this->get_logger(), "  - RGB 图像: /front_camera/image/compressed");
    RCLCPP_INFO(this->get_logger(), "  - 深度图像: /front_depth/image");
    RCLCPP_INFO(this->get_logger(), "  - 激光雷达: /front_lidar");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_rgb_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

  uint64_t rgb_counter_   = 0;
  uint64_t depth_counter_ = 0;
};

}  // namespace vcf_sensor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vcf_sensor::SensorSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
