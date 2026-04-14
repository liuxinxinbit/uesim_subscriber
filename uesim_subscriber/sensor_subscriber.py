#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from sensor_msgs.msg import PointField
import struct


MAX_DEPTH = 10.0


def depth_to_colormap(depth_array: np.ndarray) -> np.ndarray:
    """将深度图像转换为伪彩色图像并应用直方图均衡化"""
    depth_trunc = np.clip(depth_array, 0, MAX_DEPTH)
    min_val, max_val = depth_trunc.min(), depth_trunc.max()
    range_val = max_val - min_val

    if range_val > 1e-6:
        normalized = ((depth_trunc - min_val) / range_val * 255).astype(np.uint8)
    else:
        normalized = np.zeros(depth_trunc.shape, dtype=np.uint8)

    equalized = cv2.equalizeHist(normalized)
    color_map = cv2.applyColorMap(equalized, cv2.COLORMAP_HOT)
    return color_map


def parse_xyzilidar(msg: PointCloud2) -> np.ndarray:
    """将 PointCloud2 消息解析为 numpy 数组"""
    points = []
    for i in range(0, msg.row_step, msg.point_step):
        x = struct.unpack_from('f', msg.data, i + msg.fields[0].offset)[0]
        y = struct.unpack_from('f', msg.data, i + msg.fields[1].offset)[0]
        z = struct.unpack_from('f', msg.data, i + msg.fields[2].offset)[0]
        intensity = struct.unpack_from('f', msg.data, i + msg.fields[3].offset)[0]
        points.append([x, y, z, intensity])
    return np.array(points, dtype=np.float32)


class RGBCallback:
    def __init__(self):
        self.counter = 0

    def __call__(self, msg: CompressedImage):
        self.counter += 1
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        print("rgb callback")
        if image is None:
            return
        cv2.imshow('RGB Camera', image)
        cv2.waitKey(1)


class DepthCallback:
    def __init__(self):
        self.counter = 0

    def __call__(self, msg: Image):
        self.counter += 1
        HEIGHT, WIDTH = 480, 640
        print("depth callback")
        depth_image = np.ndarray(
            shape=(HEIGHT, WIDTH),
            dtype=np.float32,
            buffer=msg.data
        ).copy()

        min_val, max_val = depth_image.min(), depth_image.max()
        range_val = max_val - min_val

        if range_val > 1e-6:
            normalized = ((depth_image - min_val) / range_val * 255).astype(np.uint8)
        else:
            normalized = np.zeros(depth_image.shape, dtype=np.uint8)

        equalized = cv2.equalizeHist(normalized)
        color_map = cv2.applyColorMap(equalized, cv2.COLORMAP_HOT)
        color_map = cv2.applyColorMap(color_map, cv2.COLORMAP_HOT)

        cv2.imshow('Depth Camera', color_map)
        cv2.waitKey(1)


class LidarCallback:
    def __init__(self):
        self.counter = 0

    def __call__(self, msg: PointCloud2):
        self.counter += 1
        print("lidar callback")

class SensorSubscriberNode(Node):
    def __init__(self):
        super().__init__('sensor_subscriber_py')

        qos = rclpy.qos.QoSProfile(depth=1)
        qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        qos.durability = rclpy.qos.DurabilityPolicy.VOLATILE

        self.rgb_sub = self.create_subscription(
            CompressedImage,
            '/front_camera/image/compressed',
            RGBCallback(),
            qos
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/front_depth/image',
            DepthCallback(),
            qos
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/front_lidar',
            LidarCallback(),
            qos
        )

        self.get_logger().info('SensorSubscriberNode (Python) 已启动')
        self.get_logger().info('  - RGB 图像: /front_camera/image/compressed')
        self.get_logger().info('  - 深度图像: /front_depth/image')
        self.get_logger().info('  - 激光雷达: /front_lidar')


def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
