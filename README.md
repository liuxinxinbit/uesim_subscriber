# uesim_subscriber

ROS2 订阅节点，用于接收和可视化 Unreal Engine 模拟器的传感器数据（RGB 图像、深度图像和 LiDAR 点云）。本包提供 C++ 和 Python 两种实现。

## 功能特性

- **RGB 相机数据订阅** - 订阅压缩图像并实时显示
- **深度图像处理** - 接收深度数据并使用热力图着色可视化
- **点云数据处理** - 订阅 LiDAR 点云数据并转换为 PCL 格式
- **实时可视化** - 使用 OpenCV 窗口实时显示传感器数据
- **双语言实现** - 同时提供 C++ 和 Python 版本

## 系统要求

- ROS2 (Humble/Foxy 或更高版本)
- Ubuntu 20.04/22.04
- OpenCV 4.x
- PCL (Point Cloud Library)
- cv_bridge
- image_transport
- Python 3.x

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
    libopencv-dev \
    python3-opencv
```

## 编译

```bash
# 进入工作区根目录
cd /home/user/ros_ws

# 编译该包
colcon build --packages-select uesim_subscriber

# 加载环境变量
source install/setup.bash
```

## 运行

### C++ 版本

```bash
ros2 run uesim_subscriber subscriber_uesim_vc
```

### Python 版本

```bash
ros2 run uesim_subscriber sensor_subscriber.py
```

## 订阅的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/front_camera/image/compressed` | `sensor_msgs/CompressedImage` | RGB 压缩图像 (BGR8) |
| `/front_depth/image` | `sensor_msgs/Image` | 深度图像 (32FC1, 640×480) |
| `/front_lidar` | `sensor_msgs/PointCloud2` | LiDAR 点云数据 (PointXYZI) |

## QoS 配置

所有订阅使用以下 QoS 设置：

- **可靠性**: Best Effort
- **持久性**: Volatile
- **队列大小**: 1

## 节点架构

### C++ 版本 (subscriber_uesim_vc)

```
sensor_subscriber (VCFSubscriber)
├── rgb_callback      -> 处理 RGB 图像，显示到 "RGB Camera" 窗口
├── depth_callback    -> 处理深度图像，热力图显示到 "Depth Camera" 窗口
└── lidar_callback    -> 处理点云数据，记录日志信息
```

### Python 版本 (sensor_subscriber.py)

```
sensor_subscriber_py
├── RGBCallback       -> 处理 RGB 图像，显示到 "RGB Camera" 窗口
├── DepthCallback      -> 处理深度图像，热力图显示到 "Depth Camera" 窗口
└── LidarCallback      -> 处理点云数据
```

## 可视化窗口

运行后会弹出以下 OpenCV 窗口：

- **RGB Camera** - 显示 RGB 相机图像
- **Depth Camera** - 显示热力图 (COLORMAP_HOT) 着色的深度图像

深度图像处理流程：归一化 -> 直方图均衡化 -> HOT 色彩映射 (应用两次以增强对比度)

## 代码结构

```
uesim_subscriber/
├── CMakeLists.txt                    # CMake 构建配置
├── package.xml                       # ROS2 包清单
├── vcf_subscriber.cpp                 # C++ 订阅节点实现
└── uesim_subscriber/
    └── sensor_subscriber.py           # Python 订阅节点实现
```

## 关键实现细节

### 深度图可视化

1. 将深度数据归一化到 0-255 范围
2. 应用直方图均衡化增强对比度
3. 应用 COLORMAP_HOT 色彩映射（应用两次增强效果）

### 点云处理

- C++ 版本使用 `pcl::fromROSMsg` 转换
- Python 版本使用 `struct.unpack_from` 手动解析 PointCloud2 格式

### 计数器

- `rgb_counter_` / `RGBCallback.counter` - RGB 图像接收计数
- `depth_counter_` / `DepthCallback.counter` - 深度图像接收计数
- `lidar_counter` / `LidarCallback.counter` - 点云数据接收计数

## 添加新的传感器订阅

### C++ 版本

```cpp
// 在构造函数中添加
subscription_new_ = this->create_subscription<sensor_msgs::msg::YourType>(
    "/your/topic", qos_settings,
    std::bind(&SensorSubscriberNode::your_callback, this, _1));

// 添加回调函数
void your_callback(const sensor_msgs::msg::YourType::SharedPtr msg) {
    // 处理数据
}

// 在 private 部分添加成员变量
rclcpp::Subscription<sensor_msgs::msg::YourType>::SharedPtr subscription_new_;
```

### Python 版本

```python
def your_callback(msg):
    # 处理数据
    pass

self.your_sub = self.create_subscription(
    YourType,
    '/your/topic',
    your_callback,
    qos
)
```

## 故障排除

### 没有图像显示

- 确认 UE 模拟器正在运行并发布数据
- 检查话题名称是否正确：`ros2 topic list`
- 验证话题是否有数据：`ros2 topic echo /front_camera/image/compressed --no-arr`

### 编译错误

- 确认已安装所有依赖包
- 检查 ROS2 环境变量：`echo $ROS_DISTRO`
- 清理并重新编译：`rm -rf build install log && colcon build`

### 点云数据无法接收

- 检查 PCL 库是否正确安装：`pkg-config --modversion pcl_common`
- 确认 QoS 设置与发布者匹配
- 验证 PointCloud2 消息字段结构

## 许可证

Apache License 2.0
