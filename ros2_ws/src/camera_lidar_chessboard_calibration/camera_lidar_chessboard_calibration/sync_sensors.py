"""Publish timestamp-aligned camera, MID360 point cloud and IMU triplets."""
from collections import deque
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def set_stamp(msg, value):
    msg.header.stamp.sec = value // 1_000_000_000
    msg.header.stamp.nanosec = value % 1_000_000_000


class SensorSynchronizer(Node):
    """Use each LiDAR scan as a sample and select its nearest image and IMU."""
    def __init__(self):
        super().__init__('sensor_synchronizer')
        self.declare_parameter('sync_tolerance_ms', 60.0)
        self.declare_parameter('buffer_seconds', 2.0)
        self.slop_ns = int(float(self.get_parameter('sync_tolerance_ms').value) * 1e6)
        self.keep_ns = int(float(self.get_parameter('buffer_seconds').value) * 1e9)
        self.images, self.clouds, self.imus = deque(), deque(), deque()
        self.image_pub = self.create_publisher(Image, '/aligned/camera/image_raw', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, '/aligned/livox/lidar', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/aligned/livox/imu', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/aligned/sync_status', 10)
        self.create_subscription(Image, '/camera/image_raw', lambda m: self.add(self.images, m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/livox/lidar', lambda m: self.add(self.clouds, m), qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', lambda m: self.add(self.imus, m), qos_profile_sensor_data)
        self.create_timer(0.01, self.process)
        self.matched = self.dropped = 0

    def add(self, queue, msg):
        queue.append(msg)
        newest = stamp_ns(msg)
        while queue and newest - stamp_ns(queue[0]) > self.keep_ns:
            queue.popleft()

    @staticmethod
    def nearest(queue, target):
        return min(queue, key=lambda m: abs(stamp_ns(m) - target)) if queue else None

    def process(self):
        if not self.clouds or not self.images or not self.imus:
            return
        # Wait one tolerance window so a slightly later sample can still be selected.
        newest = min(stamp_ns(self.images[-1]), stamp_ns(self.imus[-1]))
        while self.clouds and newest >= stamp_ns(self.clouds[0]) + self.slop_ns:
            cloud = self.clouds.popleft(); reference = stamp_ns(cloud)
            image = self.nearest(self.images, reference); imu = self.nearest(self.imus, reference)
            image_error = abs(stamp_ns(image) - reference); imu_error = abs(stamp_ns(imu) - reference)
            if image_error > self.slop_ns or imu_error > self.slop_ns:
                self.dropped += 1
                continue
            image_out, cloud_out, imu_out = deepcopy(image), deepcopy(cloud), deepcopy(imu)
            # All aligned outputs carry the scan midpoint stamp. Original topics retain device/receive stamps.
            set_stamp(image_out, reference); set_stamp(cloud_out, reference); set_stamp(imu_out, reference)
            self.image_pub.publish(image_out); self.cloud_pub.publish(cloud_out); self.imu_pub.publish(imu_out)
            self.matched += 1
            status = String()
            status.data = (f'matched={self.matched} dropped={self.dropped} '
                           f'image_error_ms={image_error/1e6:.3f} imu_error_ms={imu_error/1e6:.3f}')
            self.status_pub.publish(status)
            if self.matched == 1 or self.matched % 50 == 0:
                self.get_logger().info(status.data)


def main():
    rclpy.init(); node = SensorSynchronizer()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.try_shutdown()
