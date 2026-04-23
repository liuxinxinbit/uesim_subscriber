# uesim_subscriber

`uesim_subscriber` 是一个 ROS 2 订阅示例包，用于接收 Unreal Engine 模拟环境发布的三类传感器数据：

- RGB 压缩图像
- 深度压缩图像
- LiDAR 点云

本包同时提供 C++ 和 Python 两种实现，便于调试、验证消息流以及做后续二次开发。

## 功能概览

- 订阅 `/front_camera/image/compressed` 并用 OpenCV 实时显示 RGB 图像
- 订阅 `/front_depth/image/compressed`，将编码后的深度图恢复为伪彩色热力图显示
- 订阅 `/front_lidar`，输出点云尺寸、点数和接收序号
- C++ 与 Python 版本统一使用 `Best Effort + Volatile + depth=1` 的 QoS
- Python 版本在无图形环境时会自动关闭窗口显示，但仍继续订阅和打印日志

## 包结构

```text
uesim_subscriber/
├── CMakeLists.txt
├── package.xml
├── README.md
├── vcf_subscriber.cpp
└── uesim_subscriber/
    ├── __init__.py
    └── sensor_subscriber.py
```

## 依赖

### ROS 2 依赖

- `rclcpp`
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`
- `pcl_conversions`
- `ament_cmake_python`

### 系统依赖

- OpenCV
- PCL
- Python 3
- `python3-opencv`

示例安装命令：

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-pcl-conversions \
  libopencv-dev \
  libpcl-dev \
  python3-opencv
```

## 编译

在 ROS 2 工作区根目录执行：

```bash
colcon build --packages-select uesim_subscriber
source install/setup.bash
```

## 运行

### C++ 节点

- 可执行文件：`subscriber_uesim_vc`
- 节点名：`sensor_subscriber`

```bash
ros2 run uesim_subscriber subscriber_uesim_vc
```

### Python 节点

- 入口脚本：`sensor_subscriber.py`
- 节点名：`sensor_subscriber_py`

```bash
ros2 run uesim_subscriber sensor_subscriber.py
```

## 订阅话题

| 话题 | 消息类型 | 说明 |
| --- | --- | --- |
| `/front_camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | RGB 压缩图像 |
| `/front_depth/image/compressed` | `sensor_msgs/msg/CompressedImage` | 深度压缩图像 |
| `/front_lidar` | `sensor_msgs/msg/PointCloud2` | LiDAR 点云 |

> 注意：源码中的启动日志把深度话题打印成了 `/front_depth/image`，但实际订阅的是 `/front_depth/image/compressed`。

## 数据处理逻辑

### RGB 图像

- C++ 版本通过 `cv_bridge::toCvCopy(..., BGR8)` 转换压缩图像
- Python 版本通过 `cv2.imdecode(..., cv2.IMREAD_COLOR)` 解码
- 两个版本都会输出图像宽高和接收计数

### 深度图像

深度图并不是直接使用 `sensor_msgs/Image` 浮点深度，而是从压缩后的三通道图像中恢复：

```text
depth = R + G / 255.0
```

然后执行以下步骤：

1. 将深度值裁剪到最大 `10.0 m`
2. 归一化到 `0-255`
3. 应用 `COLORMAP_HOT` 生成热力图

### LiDAR 点云

- C++ 版本使用 `pcl::fromROSMsg` 转为 `pcl::PointCloud<pcl::PointXYZI>`
- Python 版本直接根据 `PointCloud2` 的 `width * height` 统计点数
- 两个版本都会输出点云宽高、点数和接收序号

## QoS 配置

两个实现都使用等价的 QoS：

- History depth：`1`
- Reliability：`BEST_EFFORT`
- Durability：`VOLATILE`

适用于高频传感器流的低延迟订阅场景。

## 可视化行为

### C++ 版本

- 打开 `RGB Camera` 窗口显示 RGB 图像
- 打开 `Depth Camera` 窗口显示深度热力图

### Python 版本

- 与 C++ 相同，显示 `RGB Camera` 和 `Depth Camera`
- 如果未检测到 `DISPLAY` 或 `WAYLAND_DISPLAY`，则自动禁用窗口显示，仅保留订阅与日志输出
- 如果 `cv2.imshow` 运行失败，也会自动关闭后续显示，避免节点退出

## 日志输出

运行过程中会持续打印类似信息：

- RGB 图像尺寸与序号
- 深度图像尺寸与序号
- 点云宽高、点数与序号

这适合用于确认模拟器是否稳定发布数据。

## 常见问题

### 没有看到图像窗口

- 确认当前环境有桌面显示会话
- 检查是否设置了 `DISPLAY` 或 `WAYLAND_DISPLAY`
- 在无图形环境中，Python 版会自动退化为仅日志模式；C++ 版仍会尝试调用 OpenCV 窗口

### 没有收到消息

- 用 `ros2 topic list` 确认话题存在
- 用 `ros2 topic echo /front_lidar --once` 或 `ros2 topic hz /front_camera/image/compressed` 检查是否有数据
- 确认发布端 QoS 与订阅端兼容

### 深度图显示异常

- 确认发布端输出的是与当前解码逻辑匹配的压缩三通道深度图
- 当前实现假定深度编码方式为 `R + G/255.0`

## 许可证

Apache License 2.0
