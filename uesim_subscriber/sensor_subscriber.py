#!/usr/bin/env python3

import os
import struct
from typing import Optional

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robots_dog_msgs.msg import UniRtkPvh
from sensor_msgs.msg import CompressedImage, Imu, PointCloud2


MAX_VISUAL_DEPTH_M = 10.0
LIDAR_VIEW_SIZE = 800
LIDAR_VIEW_RANGE_M = 30.0


def sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=5)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


def field_offset(msg: PointCloud2, name: str) -> Optional[int]:
    for field in msg.fields:
        if field.name == name:
            return field.offset
    return None


def read_float32(data: bytes, offset: int) -> float:
    if offset + 4 > len(data):
        return 0.0
    return struct.unpack_from('<f', data, offset)[0]


def make_lidar_view(msg: PointCloud2) -> np.ndarray:
    view = np.full((LIDAR_VIEW_SIZE, LIDAR_VIEW_SIZE, 3), 8, dtype=np.uint8)
    center = LIDAR_VIEW_SIZE // 2
    scale = (LIDAR_VIEW_SIZE * 0.45) / LIDAR_VIEW_RANGE_M

    cv2.line(view, (center, 0), (center, LIDAR_VIEW_SIZE), (45, 45, 45), 1)
    cv2.line(view, (0, center), (LIDAR_VIEW_SIZE, center), (45, 45, 45), 1)
    for distance_m in range(5, int(LIDAR_VIEW_RANGE_M) + 1, 5):
        cv2.circle(view, (center, center), int(distance_m * scale), (28, 28, 28), 1)

    x_off = field_offset(msg, 'x')
    y_off = field_offset(msg, 'y')
    z_off = field_offset(msg, 'z')
    intensity_off = field_offset(msg, 'intensity')
    if None in (x_off, y_off, z_off) or msg.point_step == 0:
        cv2.putText(view, 'PointCloud2 missing x/y/z fields', (24, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 220, 255), 2)
        return view

    data = memoryview(msg.data)
    point_count = int(msg.width) * int(msg.height)
    for index in range(point_count):
        base = index * msg.point_step
        if base + msg.point_step > len(data):
            break

        x = read_float32(data, base + x_off)
        y = read_float32(data, base + y_off)
        z = read_float32(data, base + z_off)
        if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
            continue
        if x < -5.0 or x > LIDAR_VIEW_RANGE_M or abs(y) > LIDAR_VIEW_RANGE_M:
            continue

        px = center - int(y * scale)
        py = center - int(x * scale)
        if px < 0 or px >= LIDAR_VIEW_SIZE or py < 0 or py >= LIDAR_VIEW_SIZE:
            continue

        if intensity_off is not None:
            intensity = float(np.clip(read_float32(data, base + intensity_off) / 255.0, 0.0, 1.0))
            color = (int(255 * (1.0 - intensity)), int(255 * intensity), 80)
        else:
            zn = float(np.clip((z + 2.0) / 5.0, 0.0, 1.0))
            color = (int(255 * (1.0 - zn)), 180, int(255 * zn))
        view[py, px] = color

    cv2.circle(view, (center, center), 5, (255, 255, 255), -1)
    cv2.putText(view, 'front', (center + 10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
    return view


class ImageDisplay:
    def __init__(self, logger):
        self._logger = logger
        self._enabled = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        self._warned = False

    def show(self, window_name: str, image: np.ndarray) -> None:
        if not self._enabled:
            if not self._warned:
                self._logger.warning('未检测到图形显示环境，仅打印传感器数据。')
                self._warned = True
            return

        try:
            cv2.imshow(window_name, image)
            cv2.waitKey(1)
        except cv2.error as exc:
            self._logger.warning(f'OpenCV 显示失败，自动关闭窗口显示: {exc}')
            self._enabled = False


class UesimSubscriber(Node):
    def __init__(self):
        super().__init__('uesim_subscriber_py')
        self.display = ImageDisplay(self.get_logger())
        self.rgb_count = 0
        self.depth_count = 0
        self.lidar_count = 0
        self.imu_count = 0
        self.odom_count = 0
        self.gps_count = 0

        qos = sensor_qos()
        self.create_subscription(CompressedImage, '/front_camera/image/compressed', self.on_rgb, qos)
        self.create_subscription(CompressedImage, '/front_depth/image/compressed', self.on_depth, qos)
        self.create_subscription(PointCloud2, '/front_lidar/lidar', self.on_lidar, qos)
        self.create_subscription(Imu, '/imu', self.on_imu, qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos)
        self.create_subscription(UniRtkPvh, '/gps', self.on_gps, qos)

        self.get_logger().info('UESIM Python subscriber started.')
        self.get_logger().info('  RGB   : /front_camera/image/compressed')
        self.get_logger().info('  Depth : /front_depth/image/compressed')
        self.get_logger().info('  LiDAR : /front_lidar/lidar')
        self.get_logger().info('  IMU   : /imu')
        self.get_logger().info('  Odom  : /odom')
        self.get_logger().info('  GPS   : /gps')

    def decode_image(self, msg: CompressedImage) -> Optional[np.ndarray]:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return image

    def on_rgb(self, msg: CompressedImage) -> None:
        image = self.decode_image(msg)
        if image is None:
            self.get_logger().warning('RGB compressed image decode failed.')
            return
        self.rgb_count += 1
        self.get_logger().info(
            f'RGB image: {image.shape[1]}x{image.shape[0]} format={msg.format} count={self.rgb_count}',
            throttle_duration_sec=2.0,
        )
        self.display.show('UESIM RGB', image)

    def on_depth(self, msg: CompressedImage) -> None:
        encoded = self.decode_image(msg)
        if encoded is None:
            self.get_logger().warning('Depth compressed image decode failed.')
            return

        red = encoded[:, :, 2].astype(np.float32)
        green = encoded[:, :, 1].astype(np.float32)
        depth_m = np.minimum(red + green / 255.0, MAX_VISUAL_DEPTH_M)
        depth_u8 = np.clip(depth_m * (255.0 / MAX_VISUAL_DEPTH_M), 0, 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)

        self.depth_count += 1
        self.get_logger().info(
            f'Depth image: {encoded.shape[1]}x{encoded.shape[0]} format={msg.format} count={self.depth_count}',
            throttle_duration_sec=2.0,
        )
        self.display.show('UESIM Depth', depth_color)

    def on_lidar(self, msg: PointCloud2) -> None:
        self.lidar_count += 1
        point_count = int(msg.width) * int(msg.height)
        x_off = field_offset(msg, 'x')
        y_off = field_offset(msg, 'y')
        z_off = field_offset(msg, 'z')
        intensity_off = field_offset(msg, 'intensity')

        x = y = z = intensity = 0.0
        if None not in (x_off, y_off, z_off) and len(msg.data) >= msg.point_step:
            x = read_float32(msg.data, x_off)
            y = read_float32(msg.data, y_off)
            z = read_float32(msg.data, z_off)
            if intensity_off is not None:
                intensity = read_float32(msg.data, intensity_off)

        self.get_logger().info(
            f'LiDAR: points={point_count} point_step={msg.point_step} '
            f'first=({x:.3f}, {y:.3f}, {z:.3f}) intensity={intensity:.1f} count={self.lidar_count}',
            throttle_duration_sec=2.0,
        )
        self.display.show('UESIM LiDAR BEV', make_lidar_view(msg))

    def on_imu(self, msg: Imu) -> None:
        self.imu_count += 1
        q = msg.orientation
        gyro = msg.angular_velocity
        acc = msg.linear_acceleration
        self.get_logger().info(
            f'IMU frame={msg.header.frame_id} '
            f'q=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}) '
            f'gyro=({gyro.x:.3f}, {gyro.y:.3f}, {gyro.z:.3f}) '
            f'acc=({acc.x:.3f}, {acc.y:.3f}, {acc.z:.3f}) count={self.imu_count}',
            throttle_duration_sec=1.0,
        )

    def on_odom(self, msg: Odometry) -> None:
        self.odom_count += 1
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.get_logger().info(
            f'Odom frame={msg.header.frame_id} child={msg.child_frame_id} '
            f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            f'lin_vel=({v.x:.3f}, {v.y:.3f}, {v.z:.3f}) count={self.odom_count}',
            throttle_duration_sec=1.0,
        )

    def on_gps(self, msg: UniRtkPvh) -> None:
        self.gps_count += 1
        nav = msg.bestnav
        heading = msg.heading
        self.get_logger().info(
            f'GPS lat={nav.latitude_deg:.8f} lon={nav.longitude_deg:.8f} '
            f'alt={nav.altitude_m:.3f} heading={heading.heading_deg:.2f} '
            f'svs={nav.soln_svs_num}/{nav.svs_num} count={self.gps_count}',
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = UesimSubscriber()
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
