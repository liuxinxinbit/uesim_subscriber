# uesim_subscriber

ROS2 订阅节点，用于接收和可视化 UE (Unreal Engine) 模拟器的传感器数据，包括 RGB 图像、深度图像和 LiDAR 点云。

## 功能特性

- 📷 **RGB 相机数据订阅** - 订阅压缩图像并实时显示
- 🔍 **深度图像处理** - 接收深度数据并使用热力图着色可视化
- ☁️ **点云数据处理** - 订阅 LiDAR 点云数据并转换为 PCL 格式
- 📊 **实时可视化** - 使用 OpenCV 窗口实时显示传感器数据

## 系统要求

- ROS2 (Humble/Foxy 或更高版本)
- Ubuntu 20.04/22.04
- OpenCV 4.x
- PCL (Point Cloud Library)
- cv_bridge
- image_transport

## 安装依赖

```bash
# 安装 ROS2 依赖包
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-pcl-conversions \
    libpcl-dev \
    libopencv-dev
```

## 编译

```bash
# 进入工作区根目录
cd /home/xin/ros_ws

# 编译该包
colcon build --packages-select uesim_subscriber

# 加载环境变量
source install/setup.bash
```

## 运行

```bash
# 启动订阅节点
ros2 run uesim_subscriber subscriber_uesim_vc
```

## 订阅的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/xgb/image_raw/compressed` | `sensor_msgs/CompressedImage` | RGB 压缩图像 (BGR8, 640×480) |
| `/xgb/image_raw/compressed/depth` | `sensor_msgs/Image` | 深度图像 (32FC1) |
| `/xgb/livox/lidar` | `sensor_msgs/PointCloud2` | LiDAR 点云数据 (PointXYZI) |

## QoS 配置

所有订阅使用以下 QoS 设置：
- **可靠性**: Reliable
- **持久性**: Volatile
- **传输模式**: Best Effort
- **队列大小**: 1

## 节点架构

```
VCFSubscriber (Visual + Cloud Fusion Subscriber)
├── image_callback        -> 处理 RGB 图像
├── image_callback2       -> 处理深度图像
└── pointcloud_callback   -> 处理点云数据
```

## 可视化窗口

运行后会弹出以下 OpenCV 窗口：
- **Camera Image** - 显示 RGB 相机图像
- **Depth Image** - 显示热力图着色的深度图像

## 开发说明

### 代码结构
- `vcf_subscriber.cpp` - 主订阅节点实现
- `CMakeLists.txt` - CMake 构建配置
- `package.xml` - ROS2 包清单

### 关键实现细节

1. **深度图可视化**：深度数据首先归一化到 0-255 范围，然后应用 HOT 色彩映射
2. **全局变量**：`Global_Depth_Image` 和 `Merge_Image` 用于跨回调的图像融合
3. **计数器**：`camera_num` 和 `lidar_num` 追踪接收的消息数量

### 添加新的传感器订阅

```cpp
// 在构造函数中添加
subscription_new_ = this->create_subscription<sensor_msgs::msg::YourType>(
  "/your/topic", qos_settings, 
  std::bind(&VCFSubscriber::your_callback, this, _1));

// 添加回调函数
void your_callback(const sensor_msgs::msg::YourType::SharedPtr msg) {
  // 处理数据
}

// 在 private 部分添加成员变量
rclcpp::Subscription<sensor_msgs::msg::YourType>::SharedPtr subscription_new_;
```

## 故障排除

### 没有图像显示
- 确认 UE 模拟器正在运行并发布数据
- 检查话题名称是否正确：`ros2 topic list`
- 验证话题是否有数据：`ros2 topic echo /xgb/image_raw/compressed --no-arr`

### 编译错误
- 确认已安装所有依赖包
- 检查 ROS2 环境变量：`echo $ROS_DISTRO`
- 清理并重新编译：`rm -rf build install log && colcon build`

### 点云数据无法接收
- 检查 PCL 库是否正确安装：`pkg-config --modversion pcl_common`
- 确认 QoS 设置与发布者匹配

## 许可证

Apache License 2.0

## 作者

- liuxinxin

## 维护者

- liuxinxin (liuxinxin@zsibot.org)
- huangyue (huangyue@zsibot.org)
