#!/usr/bin/env python3
"""
Hazard Mask Post-Processing and Fusion Node.
Subscribes:
  - /perception/hazard_mask (sensor_msgs/msg/Image)
  - /perception/depth/image_raw (sensor_msgs/msg/Image, 32FC1)
Publishes:
  - /perception/hazard_filtered (sensor_msgs/msg/Image, filtered for morphological noise)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class HazardMaskFilterNode(Node):
    def __init__(self):
        super().__init__('hazard_mask_node')

        self.declare_parameter('input_mask_topic', '/perception/hazard_mask')
        self.declare_parameter('output_mask_topic', '/perception/hazard_filtered')

        self.input_mask_topic = self.get_parameter('input_mask_topic').value
        self.output_mask_topic = self.get_parameter('output_mask_topic').value

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            self.input_mask_topic,
            self.callback,
            10
        )
        self.pub = self.create_publisher(Image, self.output_mask_topic, 10)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    def callback(self, msg: Image):
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Failed to parse mask: {e}')
            return

        # Morphological open to remove specks, morphological close to connect clusters
        filtered = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        filtered = cv2.morphologyEx(filtered, cv2.MORPH_CLOSE, self.kernel)

        out_msg = self.bridge.cv2_to_imgmsg(filtered, encoding='mono8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HazardMaskFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
