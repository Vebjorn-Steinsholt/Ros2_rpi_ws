#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import time


class ImageSaver(Node):
    def __init__(self, topic, outdir, max_images=30, delay=0.3):
        super().__init__('image_saver')
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.bridge = CvBridge()
        self.outdir = os.path.expanduser(outdir)
        os.makedirs(self.outdir, exist_ok=True)
        self.count = 0
        self.max_images = max_images
        self.delay = delay
        self.get_logger().info(f"Saving up to {max_images} images from {topic} to {self.outdir}")

    def cb(self, msg):
        if self.count >= self.max_images:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return
        filename = os.path.join(self.outdir, f"img_{self.count:03d}.jpg")
        cv2.imwrite(filename, cv_img)
        self.get_logger().info(f"Saved {filename}")
        self.count += 1
        time.sleep(self.delay)


def main(argv=None):
    if argv is None:
        argv = sys.argv
    if len(argv) < 4:
        print("Usage: save_images.py <image_topic> <outdir> <max_images> [delay]")
        return
    topic = argv[1]
    outdir = argv[2]
    max_images = int(argv[3])
    delay = float(argv[4]) if len(argv) > 4 else 0.3
    rclpy.init()
    node = ImageSaver(topic, outdir, max_images, delay)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
