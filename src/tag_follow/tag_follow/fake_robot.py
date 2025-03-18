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

from cv_bridge import CvBridge

import cv2
import os

os.environ['RCUTILS_LOGGING_LEVEL'] = 'DEBUG'

class FAKE_ROBOT(Node):
    def __init__(self):
        super().__init__('fake_robot')
        qos_profile = QoSProfile(depth=10)

        #subscription -> 3 subscriptions
        #1) action velocity subscription
        self.subscription = self.create_subscription(
            Twist,
            'mcu/command/manual_twist', # length 3 vector of integer angles
            self.logger_velocity,
            qos_profile
        )
        self.subscription #prevent unused variable warning

        #2) tag return logger
        # self.subscription_2 = self.create_subscription(
        #     Bool,
        #     '/tag_return', # length 3 vector of integer angles
        #     self.logger_return,
        #     qos_profile
        # )
        # self.subscription_2

        #3) tag position logger
        # self.subscription_3 = self.create_subscription(
        #     Point,
        #     '/tag_location', # length 3 vector of integer angles
        #     self.logger_position,
        #     qos_profile
        # )
        # self.subscription_3

        #publish camera frame to ROS Image
        self.publisher = self.create_publisher(Image,
        '/argus/ar0234_front_left/image_raw', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_camera)

    #total 3 callbacks
    def logger_velocity(self, msg):
        linear_vel = msg.linear.x #float32
        self.get_logger().info("angle received from controller")
        #self.get_logger().info("fake robot velocity: %f" %linear_vel)
        

    # def logger_return(self, msg):
    #     value = 1 if msg.data == True else 0
    #     self.get_logger().info('fake robot return: %d' %value)

    # def logger_position(self, msg):
    #     x = msg.x
    #     y = msg.y
    #     z = msg.z
    #     self.get_logger().info('fake robot tag: %f' %z)
    #     #error: not enough arguments for string


    def publish_camera(self):
        imager = Image()
        cap = cv2.VideoCapture(0)
        while cap.isOpened():
            ret, frame = cap.read()
            size_tuple = np.shape(frame)
            #flattened_image = np.array(frame).flatten()
            # print(size_tuple)
            # cv2.imshow('test', frame)
            # cv2.waitKey(0)

            #use cv bridge to convert
            imager = CvBridge().cv2_to_imgmsg(frame, "bgr8")

            # imager.data = frame
            # imager.height = size_tuple[0]
            # imager.width = size_tuple[1]
            self.publisher.publish(imager)


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