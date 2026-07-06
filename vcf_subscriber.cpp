#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <robots_dog_msgs/msg/uni_rtk_pvh.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace uesim_subscriber
{

constexpr float kMaxVisualDepthM = 10.0f;
constexpr int kLidarViewSize = 800;
constexpr float kLidarViewRangeM = 30.0f;

rclcpp::QoS sensor_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

cv::Mat decode_compressed_image(const sensor_msgs::msg::CompressedImage & msg, int flags)
{
  const cv::Mat bytes(1, static_cast<int>(msg.data.size()), CV_8UC1, const_cast<uint8_t *>(msg.data.data()));
  return cv::imdecode(bytes, flags);
}

bool field_offset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name, uint32_t & offset)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      offset = field.offset;
      return true;
    }
  }
  return false;
}

float read_float32(const std::vector<uint8_t> & data, size_t offset)
{
  float value = 0.0f;
  if (offset + sizeof(float) <= data.size()) {
    std::memcpy(&value, data.data() + offset, sizeof(float));
  }
  return value;
}

cv::Mat make_lidar_view(const sensor_msgs::msg::PointCloud2 & msg)
{
  cv::Mat view(kLidarViewSize, kLidarViewSize, CV_8UC3, cv::Scalar(8, 8, 8));
  const int center = kLidarViewSize / 2;
  const float scale = (kLidarViewSize * 0.45f) / kLidarViewRangeM;

  cv::line(view, cv::Point(center, 0), cv::Point(center, kLidarViewSize), cv::Scalar(45, 45, 45), 1);
  cv::line(view, cv::Point(0, center), cv::Point(kLidarViewSize, center), cv::Scalar(45, 45, 45), 1);
  for (float range = 5.0f; range <= kLidarViewRangeM; range += 5.0f) {
    cv::circle(view, cv::Point(center, center), static_cast<int>(range * scale), cv::Scalar(28, 28, 28), 1);
  }

  uint32_t x_off = 0;
  uint32_t y_off = 0;
  uint32_t z_off = 0;
  uint32_t intensity_off = 0;
  const bool has_xyz = field_offset(msg, "x", x_off) && field_offset(msg, "y", y_off) && field_offset(msg, "z", z_off);
  const bool has_intensity = field_offset(msg, "intensity", intensity_off);
  if (!has_xyz || msg.point_step == 0) {
    cv::putText(view, "PointCloud2 missing x/y/z fields", cv::Point(24, 36), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(80, 220, 255), 2);
    return view;
  }

  const uint64_t point_count = static_cast<uint64_t>(msg.width) * static_cast<uint64_t>(msg.height);
  for (uint64_t i = 0; i < point_count; ++i) {
    const size_t base = static_cast<size_t>(i) * msg.point_step;
    if (base + msg.point_step > msg.data.size()) {
      break;
    }

    const float x = read_float32(msg.data, base + x_off);
    const float y = read_float32(msg.data, base + y_off);
    const float z = read_float32(msg.data, base + z_off);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (x < -5.0f || x > kLidarViewRangeM || std::abs(y) > kLidarViewRangeM) {
      continue;
    }

    const int px = center - static_cast<int>(y * scale);
    const int py = center - static_cast<int>(x * scale);
    if (px < 0 || px >= kLidarViewSize || py < 0 || py >= kLidarViewSize) {
      continue;
    }

    cv::Scalar color(80, 220, 255);
    if (has_intensity) {
      const float intensity = std::clamp(read_float32(msg.data, base + intensity_off) / 255.0f, 0.0f, 1.0f);
      color = cv::Scalar(255.0f * (1.0f - intensity), 255.0f * intensity, 80.0f);
    } else {
      const float zn = std::clamp((z + 2.0f) / 5.0f, 0.0f, 1.0f);
      color = cv::Scalar(255.0f * (1.0f - zn), 180.0f, 255.0f * zn);
    }
    view.at<cv::Vec3b>(py, px) = cv::Vec3b(
      static_cast<uint8_t>(color[0]),
      static_cast<uint8_t>(color[1]),
      static_cast<uint8_t>(color[2]));
  }

  cv::circle(view, cv::Point(center, center), 5, cv::Scalar(255, 255, 255), -1);
  cv::putText(view, "front", cv::Point(center + 10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 1);
  return view;
}

class UesimSubscriber : public rclcpp::Node
{
public:
  UesimSubscriber()
  : Node("uesim_subscriber_cpp")
  {
    const auto qos = sensor_qos();

    rgb_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      "/front_camera/image/compressed", qos,
      [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) { on_rgb(*msg); });

    depth_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      "/front_depth/image/compressed", qos,
      [this](sensor_msgs::msg::CompressedImage::SharedPtr msg) { on_depth(*msg); });

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/front_lidar/lidar", qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_lidar(*msg); });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(*msg); });

    gps_sub_ = create_subscription<robots_dog_msgs::msg::UniRtkPvh>(
      "/gps", qos,
      [this](robots_dog_msgs::msg::UniRtkPvh::SharedPtr msg) { on_gps(*msg); });

    RCLCPP_INFO(get_logger(), "UESIM C++ subscriber started.");
    RCLCPP_INFO(get_logger(), "  RGB   : /front_camera/image/compressed");
    RCLCPP_INFO(get_logger(), "  Depth : /front_depth/image/compressed");
    RCLCPP_INFO(get_logger(), "  LiDAR : /front_lidar/lidar");
    RCLCPP_INFO(get_logger(), "  IMU   : /imu");
    RCLCPP_INFO(get_logger(), "  Odom  : /odom");
    RCLCPP_INFO(get_logger(), "  GPS   : /gps");
  }

private:
  void show_image(const std::string & window, const cv::Mat & image)
  {
    if (image.empty()) {
      return;
    }
    cv::imshow(window, image);
    cv::waitKey(1);
  }

  void on_rgb(const sensor_msgs::msg::CompressedImage & msg)
  {
    const cv::Mat image = decode_compressed_image(msg, cv::IMREAD_COLOR);
    if (image.empty()) {
      RCLCPP_WARN(get_logger(), "Failed to decode RGB compressed image.");
      return;
    }

    ++rgb_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "RGB image: %dx%d format=%s count=%lu",
      image.cols, image.rows, msg.format.c_str(), rgb_count_);
    show_image("UESIM RGB", image);
  }

  void on_depth(const sensor_msgs::msg::CompressedImage & msg)
  {
    const cv::Mat encoded = decode_compressed_image(msg, cv::IMREAD_COLOR);
    if (encoded.empty()) {
      RCLCPP_WARN(get_logger(), "Failed to decode depth compressed image.");
      return;
    }

    std::vector<cv::Mat> channels;
    cv::split(encoded, channels);
    if (channels.size() < 3) {
      RCLCPP_WARN(get_logger(), "Depth PNG has %zu channels, expected BGR/RGB data.", channels.size());
      return;
    }

    cv::Mat red;
    cv::Mat green;
    channels[2].convertTo(red, CV_32FC1);
    channels[1].convertTo(green, CV_32FC1);

    cv::Mat depth_m = red + green / 255.0f;
    cv::min(depth_m, kMaxVisualDepthM, depth_m);

    cv::Mat depth_u8;
    depth_m.convertTo(depth_u8, CV_8UC1, 255.0 / kMaxVisualDepthM);

    cv::Mat depth_color;
    cv::applyColorMap(depth_u8, depth_color, cv::COLORMAP_TURBO);

    ++depth_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Depth image: %dx%d format=%s count=%lu",
      encoded.cols, encoded.rows, msg.format.c_str(), depth_count_);
    show_image("UESIM Depth", depth_color);
  }

  void on_lidar(const sensor_msgs::msg::PointCloud2 & msg)
  {
    ++lidar_count_;
    const uint64_t point_count = static_cast<uint64_t>(msg.width) * static_cast<uint64_t>(msg.height);

    uint32_t x_off = 0;
    uint32_t y_off = 0;
    uint32_t z_off = 0;
    uint32_t intensity_off = 0;
    const bool has_xyz = field_offset(msg, "x", x_off) && field_offset(msg, "y", y_off) && field_offset(msg, "z", z_off);
    const bool has_intensity = field_offset(msg, "intensity", intensity_off);

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
    if (has_xyz && msg.point_step > 0 && msg.data.size() >= msg.point_step) {
      x = read_float32(msg.data, x_off);
      y = read_float32(msg.data, y_off);
      z = read_float32(msg.data, z_off);
      if (has_intensity) {
        intensity = read_float32(msg.data, intensity_off);
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "LiDAR: points=%lu point_step=%u first=(%.3f, %.3f, %.3f) intensity=%.1f count=%lu",
      point_count, msg.point_step, x, y, z, intensity, lidar_count_);
    show_image("UESIM LiDAR BEV", make_lidar_view(msg));
  }

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    ++imu_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "IMU frame=%s q=(%.4f, %.4f, %.4f, %.4f) gyro=(%.3f, %.3f, %.3f) acc=(%.3f, %.3f, %.3f) count=%lu",
      msg.header.frame_id.c_str(),
      msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
      imu_count_);
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    ++odom_count_;
    const auto & p = msg.pose.pose.position;
    const auto & v = msg.twist.twist.linear;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Odom frame=%s child=%s pos=(%.3f, %.3f, %.3f) lin_vel=(%.3f, %.3f, %.3f) count=%lu",
      msg.header.frame_id.c_str(), msg.child_frame_id.c_str(),
      p.x, p.y, p.z, v.x, v.y, v.z, odom_count_);
  }

  void on_gps(const robots_dog_msgs::msg::UniRtkPvh & msg)
  {
    ++gps_count_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "GPS lat=%.8f lon=%.8f alt=%.3f heading=%.2f svs=%u/%u count=%lu",
      msg.bestnav.latitude_deg, msg.bestnav.longitude_deg, msg.bestnav.altitude_m,
      msg.heading.heading_deg,
      static_cast<unsigned>(msg.bestnav.soln_svs_num),
      static_cast<unsigned>(msg.bestnav.svs_num),
      gps_count_);
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<robots_dog_msgs::msg::UniRtkPvh>::SharedPtr gps_sub_;

  uint64_t rgb_count_ = 0;
  uint64_t depth_count_ = 0;
  uint64_t lidar_count_ = 0;
  uint64_t imu_count_ = 0;
  uint64_t odom_count_ = 0;
  uint64_t gps_count_ = 0;
};

}  // namespace uesim_subscriber

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uesim_subscriber::UesimSubscriber>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
