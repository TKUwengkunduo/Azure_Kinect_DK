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
SET_DEPTH_MODE = DepthMode.OFF  # Depth disabled, only RGB
SET_CAMERA_FPS = FPS.FPS_30
SET_COLOR_FORMAT = ImageFormat.COLOR_MJPG  # or COLOR_BGRA32, COLOR_NV12, COLOR_YUY2
SET_SYNCHRONIZED_IMAGES_ONLY = False

class RGBPublisher(Node):
    def __init__(self):
        super().__init__('azure_kinect_rgb_publisher')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/rgb', 10)

        # Calculate and print timer period based on FPS
        period = 1.0 / self._fps_value()
        self.get_logger().info(f'Timer period set to {period:.3f} seconds (FPS: {self._fps_value()})')
        self.timer = self.create_timer(period, self.timer_callback)

        # Initialize Kinect device
        num_devices = connected_device_count()
        if num_devices == 0:
            self.get_logger().error('No Azure Kinect DK devices detected.')
            sys.exit(1)

        # Select first device by default
        device_id = 0
        self.k4a = PyK4A(
            Config(
                color_resolution=SET_COLOR_RESOLUTION,
                depth_mode=SET_DEPTH_MODE,
                camera_fps=SET_CAMERA_FPS,
                color_format=SET_COLOR_FORMAT,
                synchronized_images_only=SET_SYNCHRONIZED_IMAGES_ONLY
            ),
            device_id=device_id
        )
        try:
            self.k4a.start()
            self.get_logger().info(f'Started Azure Kinect DK device {device_id}')
        except Exception as e:
            self.get_logger().error(f'Failed to start device: {e}')
            sys.exit(1)

    def _fps_value(self):
        # Convert FPS enum to numeric value
        return {FPS.FPS_5: 5, FPS.FPS_15: 15, FPS.FPS_30: 30}.get(SET_CAMERA_FPS, 30)

    def timer_callback(self):
        capture = self.k4a.get_capture()
        if capture.color is None:
            return
        try:
            if SET_COLOR_FORMAT == ImageFormat.COLOR_BGRA32:
                img = capture.color[:, :, :3]
            elif SET_COLOR_FORMAT == ImageFormat.COLOR_MJPG:
                # Decode MJPG (JPEG) buffer to BGR image
                buf = np.frombuffer(capture.color, dtype=np.uint8)
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is None:
                    raise ValueError('Failed to decode MJPG frame')
            elif SET_COLOR_FORMAT == ImageFormat.COLOR_NV12:
                img = cv2.cvtColor(capture.color, cv2.COLOR_YUV2BGRA_NV12)[:, :, :3]
            elif SET_COLOR_FORMAT == ImageFormat.COLOR_YUY2:
                img = cv2.cvtColor(capture.color, cv2.COLOR_YUV2BGRA_YUY2)[:, :, :3]
            else:
                img = capture.color[:, :, :3]

            # Publish ROS Image message
            ros_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().warning(f'Error processing/publishing image: {e}')

    def destroy_node(self):
        # Cleanup
        self.k4a.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RGBPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()