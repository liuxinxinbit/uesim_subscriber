# uesim_subscriber

UESIM ROS2 订阅示例包，演示如何接收、解析和显示从 `ue_zenoh_bridge` 转出的传感器数据。

提供两个版本：

- C++：`subscriber_uesim_vc`
- Python：`sensor_subscriber.py`

## 订阅内容

| Topic | Type | 示例处理 |
| --- | --- | --- |
| `/front_camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | 解码并显示 RGB 图像 |
| `/front_depth/image/compressed` | `sensor_msgs/msg/CompressedImage` | 解码 PNG 深度图并伪彩显示 |
| `/front_lidar/lidar` | `sensor_msgs/msg/PointCloud2` | 显示 2D 俯视点云，并打印首点坐标 |
| `/imu` | `sensor_msgs/msg/Imu` | 打印姿态、角速度、线加速度 |
| `/odom` | `nav_msgs/msg/Odometry` | 打印位置和线速度 |
| `/gps` | `robots_dog_msgs/msg/UniRtkPvh` | 打印经纬度、高度和航向 |

## 安装

```bash
sudo apt update
sudo apt install -y python3-opencv libopencv-dev
```

## 编译

```bash
cd /home/xin/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uesim_subscriber
source install/setup.bash
```

## 使用

先启动 UE 仿真和 bridge：

```bash
source /opt/ros/humble/setup.bash
source /home/xin/ros_ws/install/setup.bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --key-expr 'rt/**'
```

运行 C++ 示例：

```bash
ros2 run uesim_subscriber subscriber_uesim_vc
```

运行 Python 示例：

```bash
ros2 run uesim_subscriber sensor_subscriber.py
```

运行后会显示 RGB、深度和 LiDAR 俯视图窗口，同时在终端打印 IMU、里程计和 GPS 数据。LiDAR 窗口中，白点为雷达位置，`front` 方向为车体前方。

## 查看数据

```bash
ros2 topic list
ros2 topic echo /imu
ros2 topic echo /odom
ros2 topic echo /gps
ros2 topic echo /front_lidar/lidar --no-arr
```

深度图是压缩 PNG，topic 是：

```text
/front_depth/image/compressed
```

在 rqt 中查看时，选择 `/front_depth/image` 并使用 `compressed` transport。

## 无图形界面

没有桌面环境时，Python 示例会自动禁用 OpenCV 窗口，只打印日志。C++ 示例需要图形环境显示窗口；如果只想看日志，可以注释 `cv::imshow` 相关代码。
