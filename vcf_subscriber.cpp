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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fcntl.h>      // O_RDONLY
#include <sys/mman.h>   // mmap
#include <unistd.h>     // ftruncate, close
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/compressed_image.hpp>
#include "image_transport/image_transport.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using std::placeholders::_1;

#define WIDTH 1920
#define HEIGHT 1080
int64_t camera_num = 0;
int64_t lidar_num = 0;
cv::Mat Global_Depth_Image = cv::Mat::zeros(800, 1280, CV_8UC3);
cv::Mat Merge_Image = cv::Mat::zeros(1600, 2560, CV_8UC3);
class VCFSubscriber : public rclcpp::Node
{
public:
  VCFSubscriber()
  : Node("VCF_subscriber")
  {
    rclcpp::QoS qos_settings(1); // 参数1表示队列大小
    qos_settings.reliable()      // 设置可靠性
        .durability_volatile()   // 设置持久性：非持久性
        .best_effort();
    // 创建图像订阅者，订阅名为"vcf_image"的主题
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/xgb/image_raw/compressed", qos_settings, std::bind(&VCFSubscriber::image_callback, this, _1));
    subscription_depth = this->create_subscription<sensor_msgs::msg::Image>(
      "/xgb/image_raw/compressed/depth", qos_settings, std::bind(&VCFSubscriber::image_callback2, this, _1));
    RCLCPP_INFO(this->get_logger(), "VCFSubscriber 已初始化");
    subscription_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/xgb/livox/lidar", qos_settings, std::bind(&VCFSubscriber::pointcloud_callback, this, _1));
  }
  ~VCFSubscriber()
  {
    // 析构函数中不需要额外清理，因为订阅者会自动释放资源
  }
private:


  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // 使用cv_bridge将ROS图像消息转换为OpenCV图像
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
      return;
  }

    camera_num += 1;
    std::cout<< "Received camera image shape:" << cv_ptr->image.cols << "x" << cv_ptr->image.rows << " num: " << camera_num << std::endl;
    // 显示图像
    cv::imshow("Camera Image", cv_ptr->image);
    cv::waitKey(1);
  }

  void image_callback2(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::cout<< "Received depth image message." << std::endl;
    cv::Mat depth_image;
    depth_image = cv::Mat(480, 640, CV_32FC1, const_cast<uchar*>(msg->data.data()));
    // color map
    double min, max;
    cv::minMaxIdx(depth_image, &min, &max);
    cv::Mat adjMap;
    depth_image.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min * 255 / (max - min));
    cv::Mat falseColorsMap;
    cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_HOT);
    camera_num += 1;
    std::cout<< "Received depth image shape:" << depth_image.cols << "x" << depth_image.rows << " num: " << camera_num << std::endl;
    // 显示图像
    cv::imshow("Depth Image", falseColorsMap);
    cv::waitKey(1);
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    lidar_num += 1;
   std::cout<< "Received point cloud message:" << lidar_num << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    std::cout << "Received point cloud with " << cloud->points.size() << " points." << std::endl;

  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depth;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VCFSubscriber>());
  rclcpp::shutdown();
  return 0;
}
