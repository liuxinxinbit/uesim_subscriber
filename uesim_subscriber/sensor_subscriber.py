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

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

MAX_DEPTH = 10.0


def default_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


class ImageDisplay:
    def __init__(self, logger):
        self._logger = logger
        self._gui_enabled = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        self._warned_no_gui = False
        self._warned_imshow_failure = False

    def show(self, window_name: str, image: np.ndarray) -> None:
        if not self._gui_enabled:
            if not self._warned_no_gui:
                self._logger.warning('未检测到图形显示环境，已禁用图像窗口显示，仅继续订阅数据')
                self._warned_no_gui = True
            return

        try:
            cv2.imshow(window_name, image)
            cv2.waitKey(1)
        except cv2.error as exc:
            if not self._warned_imshow_failure:
                self._logger.warning(f'OpenCV 窗口显示失败，已自动关闭显示功能: {exc}')
                self._warned_imshow_failure = True
            self._gui_enabled = False

class RGBCallback:
    def __init__(self, display: ImageDisplay, logger):
        self.counter = 0
        self.display = display
        self.logger = logger

    def __call__(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            self.logger.warning('RGB 图像解码失败')
            return

        self.counter += 1
        self.logger.info(f'RGB 图像: {image.shape[1]}x{image.shape[0]}  序号: {self.counter}')
        self.display.show('RGB Camera', image)


class DepthCallback:
    def __init__(self, display: ImageDisplay, logger):
        self.counter = 0
        self.display = display
        self.logger = logger

    def __call__(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            self.logger.warning('深度图像解码失败')
            return

        # 与 C++ 版本一致：depth = R + G/255，并截断最大深度
        r = image[:, :, 2].astype(np.float32)
        g = image[:, :, 1].astype(np.float32)
        depth_float = r + g / 255.0
        depth_float = np.minimum(depth_float, MAX_DEPTH)

        # 与 C++ 版本一致：normalize + HOT colormap
        depth_u8 = cv2.normalize(depth_float, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_HOT)

        self.counter += 1
        self.logger.info(f'深度图像: {image.shape[1]}x{image.shape[0]}  序号: {self.counter}')
        self.display.show('Depth Camera', depth_color)

class LidarCallback:
    def __init__(self, logger):
        self.counter = 0
        self.logger = logger

    def __call__(self, msg: PointCloud2):
        self.counter += 1
        point_count = int(msg.width) * int(msg.height)
        self.logger.info(
            f'激光雷达点云: {msg.width}x{msg.height}  点数:{point_count}  序号:{self.counter}'
        )

class SensorSubscriberNode(Node):
    def __init__(self):
        super().__init__('sensor_subscriber_py')
        self.display = ImageDisplay(self.get_logger())
        qos = default_qos()

        self.rgb_sub = self.create_subscription(
            CompressedImage,
            '/front_camera/image/compressed',
            RGBCallback(self.display, self.get_logger()),
            qos
        )
        self.depth_sub = self.create_subscription(
            CompressedImage,
            '/front_depth/image/compressed',
            DepthCallback(self.display, self.get_logger()),
            qos
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/front_lidar',
            LidarCallback(self.get_logger()),
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
