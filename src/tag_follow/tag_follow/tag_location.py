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

#load calibration data -> saved from calibration.py
inner_matrix = np.load("matrix.npy")
distortion = np.load("distortion.npy")
real_size = 15

#load aruco dictionary, 16 binary!
arucodict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)



def return_position():

    #marker detection -> initialize parameters for detector
    detector_param = aruco.DetectorParameters_create()

    if frame:
        #gray frame, detect marker
        #gray frame
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #return corners
        corners, ID, reject = aruco.detectMarkers(frame_gray, arucodict, parameters=detector_param)
        #if corners, estimate
        if corners:
            i=0
            rotation, transition, _ = aruco.estimatePoseSingleMarkers(corners, real_size, inner_matrix, distortion)

            for id, corner in zip(ID, corners):
                cv2.polylines(frame, [corner.astype(np.int32)], True, (0,255,255), 4, cv2.LINE_AA)
                    
                corner = corner.reshape(4,2)
                corner = corner.astype(int)
                top_right = corner[0].ravel() #np.ravel() -> contiguous flattened array!
                top_left = corner[1].ravel()
                bottom_right = corner[2].ravel()
                bottom_left = corner[3].ravel()

                #distance from the camera
                distance = np.sqrt(transition[i][0][0]**2 + transition[i][0][1]**2 + transition[i][0][2]**2)

                cv2.putText(frame, f"distance: {distance}", top_right, cv2.FONT_HERSHEY_PLAIN, 2.0, (200,100,0), 2, cv2.LINE_AA,)
                i+=1

        # Display the resulting frame
        cv2.imshow(args.frame_name, frame)

        # Press Q on keyboard to exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break


    # When everything done, release the video capture and writer objects
    cap.release()

    # Closes my window
    cv2.destroyWindow(args.frame_name)



import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import Float32MultiArray #Float32MultiArray
from rclpy.qos import QoSProfile


class TAG_LOCATION(Node):
    def __init__(self):
        super().__init__('tag_location')
        qos_profile = QoSProfile(depth=10)

        #subscriber
        self.subscription = self.create_subscription(
            Image,#ROS2 image topic
            '/image_raw', # length 3 vector of integer angles
            self.return_position,
            qos_profile
        )
        self.subscription #prevent unused variable warning

        #publisher
        self.publisher = self.create_publisher(Point,
        '/tag_location_point', qos_profile)

        #set timer
        self.timer = self.create_timer(0.1, self.publish_point)


    def get_position(self, msg):
        #get image topic, return the position of aruco tag
        #use return_position function 



    def publish_point(self):

        #publish
        state_array = Float32MultiArray()
        state_array.data = np.float32(state).tolist()
        #self.get_logger().info('successfully transitioned, publishing...')
        self.publisher.publish(state_array)



def main(args=None):
    #main function call
    rclpy.init(args=args)
    node = TAG_LOCATION()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()