#!/usr/bin/env python3
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyk4a
from pyk4a import Config, PyK4A, ColorResolution, DepthMode, ImageFormat, FPS, connected_device_count

# === Camera Control and Configuration Settings ===
# Modify these parameters as needed
SET_COLOR_RESOLUTION = ColorResolution.RES_1080P
SET_DEPTH_MODE = DepthMode.NFOV_UNBINNED  # Enable depth
SET_CAMERA_FPS = FPS.FPS_30
SET_COLOR_FORMAT = ImageFormat.COLOR_BGRA32  # Use BGRA32 for direct BGR slice
SET_SYNCHRONIZED_IMAGES_ONLY = True       # Sync color & depth for transformation

class RGBDPublisher(Node):
    def __init__(self):
        super().__init__('azure_kinect_rgbd_publisher')
        self.bridge = CvBridge()
        # Publishers for RGB and aligned Depth
        self.rgb_pub = self.create_publisher(Image, '/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/depth', 10)

        # Timer based on FPS
        period = 1.0 / self._fps_value()
        self.get_logger().info(f'Timer period set to {period:.3f} seconds (FPS: {self._fps_value()})')
        self.timer = self.create_timer(period, self.timer_callback)

        # Initialize Kinect device
        num_devices = connected_device_count()
        if num_devices == 0:
            self.get_logger().error('No Azure Kinect DK devices detected.')
            sys.exit(1)
        device_id = 0
        config = Config(
            color_resolution=SET_COLOR_RESOLUTION,
            depth_mode=SET_DEPTH_MODE,
            camera_fps=SET_CAMERA_FPS,
            color_format=SET_COLOR_FORMAT,
            synchronized_images_only=SET_SYNCHRONIZED_IMAGES_ONLY
        )
        self.k4a = PyK4A(config, device_id=device_id)
        try:
            self.k4a.start()
            self.get_logger().info(f'Started Azure Kinect DK device {device_id}')
        except Exception as e:
            self.get_logger().error(f'Failed to start device: {e}')
            sys.exit(1)

    def _fps_value(self):
        return {FPS.FPS_5: 5, FPS.FPS_15: 15, FPS.FPS_30: 30}.get(SET_CAMERA_FPS, 30)

    def timer_callback(self):
        # Capture synchronized color and depth, with depth transformed to color space
        capture = self.k4a.get_capture()
        stamp = self.get_clock().now().to_msg()

        # Publish color image
        if capture.color is not None:
            bgr = capture.color[:, :, :3]  # BGRA -> BGR
            rgb_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            rgb_msg.header.stamp = stamp
            self.rgb_pub.publish(rgb_msg)

        # Publish depth aligned to color
        if capture.transformed_depth is not None:
            depth_aligned = capture.transformed_depth  # already in color image coordinates
            depth_msg = self.bridge.cv2_to_imgmsg(depth_aligned, encoding='mono16')
            depth_msg.header.stamp = stamp
            self.depth_pub.publish(depth_msg)

    def destroy_node(self):
        self.k4a.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RGBDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()