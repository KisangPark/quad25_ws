"""
Fake robot node to check topic

subscription: command/setAction, tag_return, tag_position
show logger

+ publish image topic to image_raw
"""


import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool #Float32MultiArray
from rclpy.qos import QoSProfile

import cv2
import os


class FAKE_ROBOT(Node):
    def __init__(self):
        super().__init__('fake_robot')
        qos_profile = QoSProfile(depth=10)

        #subscription -> 3 subscriptions
        #1) action velocity subscription
        self.subscription_1 = self.create_subscription(
            Twist,
            '/command/setAction', # length 3 vector of integer angles
            self.logger_velocity,
            qos_profile
        )
        self.subscription_1 #prevent unused variable warning

        #2) tag return logger
        self.subscription_2 = self.create_subscription(
            Bool,
            '/tag_return', # length 3 vector of integer angles
            self.logger_return,
            qos_profile
        )
        self.subscription_2

        #3) tag position logger
        self.subscription_3 = self.create_subscription(
            Point,
            '/tag_position', # length 3 vector of integer angles
            self.logger_position,
            qos_profile
        )
        self.subscription_3

        #self.timer = self.create_timer(0.1, self.publish_camera)

    #total 3 callbacks
    def logger_velocity(self, msg):
        linear_vel = msg.linear.x #float32
        self.get_logger().info('fake robot velocity: %f' %linear_vel)

    def logger_return(self, msg):
        value = 1 if msg.data == True else 0
        self.get_logger().info('fake robot return: %d' %value)

    def logger_position(self, msg):
        self.get_logger().info('fake robot tag: %d, %d, %d' %msg.x %msg.y %msg.z)

    # def publish_camera(self):
    #     imager = Image()
    #     cap = cv2.VideoCapture(0)
    #     while cap.isOpened():
    #         ret, frame = cap.read()
    #         size_tuple = np.shape(frame)
    #         flattened_image = np.array(frame).flatten()
    #         imager.data = flattened_image
    #         imager.height = size_tuple[0]
    #         imager.width = size_tuple[1]
    #         self.publisher.publish(imager)

def main(args=None):
    #main function call
    rclpy.init(args=args)
    node = FAKE_ROBOT()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()