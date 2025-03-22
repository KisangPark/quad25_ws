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

import argparse

#from video_parser import aruco_tag

#os.environ['RCUTILS_LOGGING_LEVEL'] = 'DEBUG'


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

        #publish camera frame to ROS Image
        self.publisher = self.create_publisher(Image,
        '/user/image_raw', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_camera)

        self.parser = argparse.ArgumentParser()
        self.parser.add_argument(
            '--video', type=str, default="http://192.168.168.105:8080/stream?topic=/argus/ar0234_front_left/image_raw")
        self.parser.add_argument('--frame_name', type=str, default='front_left')
        self.parser.add_argument('--output', type=str, default="output.mp4")
        self.parser.add_argument('--codec', type=str, default="mp4v")
        self.parser.add_argument('--fps', type=int, default=10)
        self.parser.add_argument('--width', type=int, default=960)
        self.parser.add_argument('--height', type=int, default=540)

        #arguments error handling
        self.args, unknown = self.parser.parse_known_args()
        if unknown:
            print(f'Warning: Unrecognized arguments: {unknown}')

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

    def get_frame(self):
        #first, get frame
        

        # Open the video from http server
        cap = cv2.VideoCapture(self.args.video)

        # Check if camera opened successfully
        if not cap.isOpened():
            print("Error opening video stream or file")

        # Read until video is completed
        while cap.isOpened():
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret:
                return frame

    def publish_camera(self):
        #use get_frame
        frame = self.get_frame() # okay by this

        imager = Image()
        #frame = self.video_parser.get_online_camera()
        cv2.imshow('test', frame)
        #print("hello")
        cv2.waitKey(10)

        imager = CvBridge().cv2_to_imgmsg(frame, "bgr8")

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

"""
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
"""