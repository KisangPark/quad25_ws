"""
Fake robot node to check topic
    -> subscibe /command/Action and aruco tag position topic
"""


import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray #Float32MultiArray
from rclpy.qos import QoSProfile


class GET_FRAME(Node):
    def __init__(self):
        super().__init__('get_frame')
        qos_profile = QoSProfile(depth=10)

        #subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/angle', # length 3 vector of integer angles
            self.record_angle,
            qos_profile
        )
        self.subscription #prevent unused variable warning

        #publisher
        self.publisher = self.create_publisher(Float32MultiArray,
        'state', qos_profile)

        #set timer
        self.timer = self.create_timer(0.1, self.get_frame)


    def record_angle(self, msg):
        #self.get_logger().info('angle received from controller')
        self.servo_angle = msg.data
        #record angle from subscription


    def get_frame(self):
        #read camera
        try:
            frame = self.image
            #ret, frame = self.cap.read()   #*********************************************************************************
            #cv2.imshow('test', frame) #works only when run separately
            #cv2.waitKey(1)
            #process image -> get coordinate of vertices, 4x2 matrix
            vertex_list = detect_red_box(frame) # ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!
            #get coordinate of arm tip, 1x2 vector
            arm_tip = detect_green_dot(frame)

            #self.get_logger().info("got box and dot")

            # append all
            state = np.array(vertex_list).flatten().tolist()
            state += arm_tip
            try:
                state += self.servo_angle #angle info received
                #self.get_logger().info("servo angle appended")
            except:
                state += np.zeros(3).tolist()

        except:
            self.get_logger().info("cap not opened")
            state = np.zeros(13)

        #publish
        state_array = Float32MultiArray()
        state_array.data = np.float32(state).tolist()
        #self.get_logger().info('successfully transitioned, publishing...')
        self.publisher.publish(state_array)



def main(args=None):
    #main function call
    rclpy.init(args=args)
    node = GET_FRAME()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()