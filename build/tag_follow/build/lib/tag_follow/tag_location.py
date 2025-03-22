"""
return the distance & xy location of aruco tag
publish 2 topic: Float32 and point message
    1) float32 for distance
    2) point for aruco tag xy coordinate
    (or just use z value to take depth)

take topic of camera -> image_raw
process -> publish

specifications
    1) if no tag detected -> no action (audio mode)
"""

import cv2
import numpy as np
from cv2 import aruco

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool #Float32MultiArray
from rclpy.qos import QoSProfile

from cv_bridge import CvBridge

#load calibration data -> saved from calibration.py
inner_matrix = np.load("/home/kisangpark/v60_ws/src/tag_follow/matrix.npy")
distortion = np.load("/home/kisangpark/v60_ws/src/tag_follow/distortion.npy")
real_size = 15

#load aruco dictionary, 16 binary!
arucodict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


"""
function: return position

    1) get frame from outside
    2) return the position (transition vector)
"""
def return_position(frame):

    #marker detection -> initialize parameters for detector
    detector_param = aruco.DetectorParameters_create()

    if frame is not None: #numpy array
        #gray frame, detect marker
        #gray frame
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #return corners
        corners, ID, reject = aruco.detectMarkers(frame_gray, arucodict, parameters=detector_param)
        #if corners, estimate
        if corners:
            i=0
            rotation, transition, _ = aruco.estimatePoseSingleMarkers(corners, real_size, inner_matrix, distortion)
            
            #transition: 3xn matrix (n number of tags) -> 1x1x3 matrix for one tag
            #print(transition.shape)
            #print(transition)

            #two cases: one aruco tag or multiple
            #ignore multiple cases

            if transition is not None:
                return_transition = np.array(transition[0][0])
                return True,return_transition


    return False, np.array([0.0,0.0,50.0])  



class LOCATOR(Node):
    def __init__(self):
        super().__init__('locator')
        qos_profile = QoSProfile(depth=10)

        #subscriber
        self.subscription = self.create_subscription(
            Image,#ROS2 image topic
            '/user/image_raw', # length 3 vector of integer angles
            self.get_position,
            qos_profile
        )
        self.subscription #prevent unused variable warning

        #publisher
        self.point_publisher = self.create_publisher(Point,
        '/tag_location', qos_profile)
        self.ret_publisher = self.create_publisher(Bool,
        '/tag_return', qos_profile)

        #set timer
        self.point_timer = self.create_timer(0.1, self.publish_point)
        self.ret_timer = self.create_timer(0.1, self.tag_recognition)

        self.transition = np.array([0.0,0.0,50.0])
        self.ret = False


    def get_position(self, msg):
        #get image topic, return the position of aruco tag
        #use return_position function 
        solved_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        self.ret, self.transition = return_position(solved_image)

        #cv2.imshow('test', solved_image)
        #cv2.waitKey(200)

    
    def tag_recognition(self):
        #get self.ret, publish
        flag = Bool()
        flag.data = self.ret
        self.ret_publisher.publish(flag)


    def publish_point(self):
        #if true -> publish point
        point = Point()
        point.x = float(self.transition[0])
        point.y = float(self.transition[1])
        point.z = float(self.transition[2])

        #publish
        #self.get_logger().info('successfully transitioned, publishing...')
        self.point_publisher.publish(point)



def main(args=None):
    #main function call
    rclpy.init(args=args)
    node = LOCATOR()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()