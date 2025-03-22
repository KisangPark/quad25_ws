import cv2
import argparse

import numpy as np
from cv2 import aruco

#load calibration data -> saved from calibration.py
inner_matrix = np.load("matrix.npy")
distortion = np.load("distortion.npy")
real_size = 15

#load aruco dictionary, 16 binary!
arucodict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)



class ONLINE_VIDEO_PARSER():
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument(
            '--video', type=str, default="http://192.168.168.105:8080/stream?topic=/argus/ar0234_front_left/image_raw")
        self.parser.add_argument('--frame_name', type=str, default='front_left')
        self.parser.add_argument('--output', type=str, default="output.mp4")
        self.parser.add_argument('--codec', type=str, default="mp4v")
        self.parser.add_argument('--fps', type=int, default=10)
        self.parser.add_argument('--width', type=int, default=960)
        self.parser.add_argument('--height', type=int, default=540)
        
    
    def __del__(self):
        pass

    def get_online_camera(self):
        #get arguments, parse the video
        args = self.parser.parse_args()

        # Open the video from http server
        cap = cv2.VideoCapture(args.video)

        # Check if camera opened successfully
        if not cap.isOpened():
            print("Error opening video stream or file")

        # Read until video is completed
        while cap.isOpened():
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret:
                return frame