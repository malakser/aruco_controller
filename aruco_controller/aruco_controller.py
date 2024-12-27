#!/usr/bin/env python3

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import cv2.aruco as aruco # ArUco library
import numpy as np


class ArUcoController(Node):
  def __init__(self):
    super().__init__('aruco_controller')
    self.subscription = self.create_subscription(
      Image,
      'image_raw',
      self.listener_callback,
      10
    )
    self.bridge = CvBridge()
    self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    self.parameters = aruco.DetectorParameters_create()
    self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

  def listener_callback(self, image_data):
    cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
    corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
    cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
    msg = Twist()
    if ids is not None:
      corner_mean = np.mean(corners[0], axis=1).astype(int)[0]
      is_up = corner_mean[1] < cv_image.shape[0] // 2
      color = (0, 0, 255) if is_up else (0, 255, 0)
      vel = 1.0 if is_up else -1.0
      msg.linear.x = vel
      cv_image = cv2.circle(cv_image, tuple(corner_mean), 5, color, -1)
    self.publisher.publish(msg)
    cv2.imshow("camera", cv_image)
    cv2.waitKey(1)


def main(args=None):
  rclpy.init(args=args)
  node = ArUcoController()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
